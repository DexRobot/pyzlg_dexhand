from typing import Dict, List, Optional, Any, Union
import numpy as np
import time
import json
from dataclasses import dataclass, asdict
from datetime import datetime
import logging
import os
from pathlib import Path
import queue
import threading
from contextlib import contextmanager

from .dexhand_interface import (
    ControlMode,
    HandFeedback,
    StampedTactileFeedback,
    JointFeedback,
)

logger = logging.getLogger(__name__)


@dataclass
class LogEntry:
    """Base class for log entries"""

    timestamp: float
    hand: str
    entry_type: str


@dataclass
class CommandLogEntry(LogEntry):
    """Log entry for a hand command"""

    command_type: str  # move_joints, reset_joints, etc.
    joint_commands: Dict[str, float]  # Joint name to commanded position
    control_mode: ControlMode


@dataclass
class FeedbackLogEntry(LogEntry):
    """Log entry for hand feedback"""

    joints: Dict[str, JointFeedback]  # Joint name to feedback
    tactile: Dict[str, StampedTactileFeedback]  # Fingertip name to tactile data


class LogWriter(threading.Thread):
    """Background thread for writing log entries to files"""

    def __init__(self, session_dir: Path):
        super().__init__(daemon=True)
        self.session_dir = session_dir
        self.queue = queue.Queue()
        self.running = True

        # Keep file handles open
        self.files = {}
        for hand in ["left", "right"]:
            self.files[f"{hand}_commands"] = open(
                session_dir / f"{hand}_commands.jsonl", "a"
            )
            self.files[f"{hand}_feedback"] = open(
                session_dir / f"{hand}_feedback.jsonl", "a"
            )

    def run(self):
        """Process log entries from queue"""
        while self.running or not self.queue.empty():
            try:
                entry = self.queue.get(timeout=0.1)
                self._write_entry(entry)
                self.queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"Error writing log entry: {e}")

    def _write_entry(self, entry: LogEntry):
        """Write a single log entry to appropriate file"""
        if isinstance(entry, CommandLogEntry):
            file = self.files[f"{entry.hand}_commands"]
        elif isinstance(entry, FeedbackLogEntry):
            file = self.files[f"{entry.hand}_feedback"]
        else:
            logger.error(f"Unknown entry type: {type(entry)}")
            return

        try:
            json.dump(asdict(entry), file)
            file.write("\n")
            file.flush()  # Ensure data is written
        except Exception as e:
            logger.error(f"Error writing to file: {e}")

    def stop(self):
        """Stop the writer thread and close files"""
        self.running = False
        self.join()  # Wait for queue to empty

        # Close all files
        for file in self.files.values():
            try:
                file.close()
            except Exception as e:
                logger.error(f"Error closing file: {e}")


class DexHandLogger:
    """Logger for dexterous hand commands and feedback with background writing"""

    def __init__(self, log_dir: str = "dexhand_logs"):
        """Initialize hand logger

        Args:
            log_dir: Directory to store log files
        """
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)

        # Create session directory with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = self.log_dir / timestamp
        self.session_dir.mkdir()

        # Initialize log writer thread
        self.writer = LogWriter(self.session_dir)
        self.writer.start()

        # Initialize in-memory buffers with thread safety
        self.command_buffers = {"left": [], "right": []}
        self.feedback_buffers = {"left": [], "right": []}
        self.buffer_lock = threading.Lock()

        self.start_time = time.time()
        logger.info(f"Logging session started in {self.session_dir}")

    def log_command(
        self,
        command_type: str,
        joint_commands: Dict[str, float],
        control_mode: ControlMode,
        hand: str,
        feedback: Optional[HandFeedback] = None,
    ):
        """Log a command without blocking

        Args:
            command_type: Type of command (move_joints, reset_joints, etc)
            joint_commands: Dictionary of joint name to commanded position
            control_mode: Control mode used
            hand: Which hand ('left' or 'right')
            feedback: Optional HandFeedback if feedback was collected
        """
        entry = CommandLogEntry(
            timestamp=time.time() - self.start_time,
            hand=hand,
            entry_type="command",
            command_type=command_type,
            joint_commands=joint_commands,
            control_mode=control_mode,
        )

        # Add to buffer thread-safely
        with self.buffer_lock:
            self.command_buffers[hand].append(entry)

        # Queue for writing without blocking
        self.writer.queue.put(entry)

        # Also log feedback if provided
        if feedback:
            self.log_feedback(feedback, hand)

    def log_feedback(self, feedback: HandFeedback, hand: str):
        """Log feedback without blocking

        Args:
            feedback: HandFeedback from polling
            hand: Which hand ('left' or 'right')
        """
        entry = FeedbackLogEntry(
            timestamp=time.time() - self.start_time,
            hand=hand,
            entry_type="feedback",
            joints=feedback.joints,
            tactile=feedback.tactile,
        )

        # Add to buffer thread-safely
        with self.buffer_lock:
            self.feedback_buffers[hand].append(entry)

        # Queue for writing without blocking
        self.writer.queue.put(entry)

    def save_metadata(self, metadata: Dict[str, Any]):
        """Save session metadata

        Args:
            metadata: Dictionary of metadata to save
        """
        metadata_path = self.session_dir / "metadata.json"
        # Write metadata directly since this isn't in the critical path
        with open(metadata_path, "w") as f:
            json.dump(metadata, f, indent=2)

    def plot_session(
        self, hands: Optional[List[str]] = None, show: bool = True, save: bool = True
    ):
        """Plot command and feedback data from the session

        This method should be called outside the control loop, typically
        during analysis or after the session.

        Args:
            hands: Which hands to plot ('left', 'right', or both)
            show: Whether to show plots interactively
            save: Whether to save plots to files
        """
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            logger.error("matplotlib is required for plotting")
            return

        if hands is None:
            hands = ["left", "right"]

        # Get thread-safe copy of buffers
        with self.buffer_lock:
            command_buffers = {
                hand: self.command_buffers[hand].copy() for hand in hands
            }
            feedback_buffers = {
                hand: self.feedback_buffers[hand].copy() for hand in hands
            }

        for hand in hands:
            if not command_buffers[hand] and not feedback_buffers[hand]:
                continue

            # Plot joint commands
            plt.figure(figsize=(12, 8))
            joint_data = {}

            # Collect command data
            for entry in command_buffers[hand]:
                t = entry.timestamp
                for joint, pos in entry.joint_commands.items():
                    if joint not in joint_data:
                        joint_data[joint] = {"times": [], "positions": []}
                    joint_data[joint]["times"].append(t)
                    joint_data[joint]["positions"].append(pos)

            # Plot each joint
            for joint, data in joint_data.items():
                plt.plot(
                    data["times"],
                    data["positions"],
                    label=f"{joint} (cmd)",
                    linestyle="--",
                )

                # Plot matching feedback if available
                fb_times = []
                fb_pos = []
                for entry in feedback_buffers[hand]:
                    if joint in entry.joints:
                        fb_times.append(entry.timestamp)
                        fb_pos.append(entry.joints[joint].angle)
                if fb_times:
                    plt.plot(fb_times, fb_pos, label=f"{joint} (actual)")

            plt.xlabel("Time (s)")
            plt.ylabel("Joint Angle (degrees)")
            plt.title(f"{hand.title()} Hand Joint Commands and Feedback")
            plt.legend()
            plt.grid(True)

            if save:
                plt.savefig(self.session_dir / f"{hand}_joints.png")

            # Plot tactile feedback
            if any(entry.tactile for entry in feedback_buffers[hand]):
                plt.figure(figsize=(12, 8))
                tactile_data = {}

                # Collect tactile data
                for entry in feedback_buffers[hand]:
                    t = entry.timestamp
                    for finger, data in entry.tactile.items():
                        if finger not in tactile_data:
                            tactile_data[finger] = {
                                "times": [],
                                "normal_force": [],
                                "tangential_force": [],
                            }
                        tactile_data[finger]["times"].append(t)
                        tactile_data[finger]["normal_force"].append(data.normal_force)
                        tactile_data[finger]["tangential_force"].append(
                            data.tangential_force
                        )

                # Plot each finger's tactile data
                for finger, data in tactile_data.items():
                    plt.plot(
                        data["times"],
                        data["normal_force"],
                        label=f"{finger} normal",
                        linestyle="-",
                    )
                    plt.plot(
                        data["times"],
                        data["tangential_force"],
                        label=f"{finger} tangential",
                        linestyle="--",
                    )

                plt.xlabel("Time (s)")
                plt.ylabel("Force (N)")
                plt.title(f"{hand.title()} Hand Tactile Feedback")
                plt.legend()
                plt.grid(True)

                if save:
                    plt.savefig(self.session_dir / f"{hand}_tactile.png")

            plt.close("all")

        if show:
            plt.show()

    def close(self):
        """Close the logger and save any remaining data"""
        # Save summary statistics
        with self.buffer_lock:
            stats = {
                "duration": time.time() - self.start_time,
                "num_commands": {
                    "left": len(self.command_buffers["left"]),
                    "right": len(self.command_buffers["right"]),
                },
                "num_feedback": {
                    "left": len(self.feedback_buffers["left"]),
                    "right": len(self.feedback_buffers["right"]),
                },
            }

        self.save_metadata(
            {"statistics": stats, "timestamp": datetime.now().isoformat()}
        )

        # Stop the writer thread and close files
        self.writer.stop()

        logger.info(f"Logging session completed: {stats}")

        # Generate plots
        self.plot_session(show=False, save=True)
