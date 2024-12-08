from typing import Dict, List, Optional, Any, Union
import numpy as np
import time
import json
from dataclasses import dataclass, asdict
from datetime import datetime
import logging
import os
from pathlib import Path

from .dexhand_interface import (
    MoveFeedback, StampedTactileFeedback, JointFeedback,
    ControlMode
)

logger = logging.getLogger(__name__)

@dataclass
class CommandLogEntry:
    """Log entry for a hand command"""
    timestamp: float
    command_type: str  # move_joints, reset_joints, etc.
    joint_commands: Dict[str, float]  # Joint name to commanded position
    control_mode: ControlMode
    hand: str  # 'left' or 'right'

@dataclass
class FeedbackLogEntry:
    """Log entry for hand feedback"""
    timestamp: float
    joints: Dict[str, JointFeedback]  # Joint name to feedback
    tactile: Dict[str, StampedTactileFeedback]  # Fingertip name to tactile data
    hand: str  # 'left' or 'right'

class DexHandLogger:
    """Logger for dexterous hand commands and feedback"""

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

        # Initialize log files for each hand
        for hand in ['left', 'right']:
            (self.session_dir / f"{hand}_commands.jsonl").touch()
            (self.session_dir / f"{hand}_feedback.jsonl").touch()

        # Initialize in-memory buffers
        self.command_buffers = {
            'left': [],
            'right': []
        }
        self.feedback_buffers = {
            'left': [],
            'right': []
        }

        self.start_time = time.time()
        logger.info(f"Logging session started in {self.session_dir}")

    def log_command(self, command_type: str, feedback: MoveFeedback, hand: str):
        """Log a command and its feedback

        Args:
            command_type: Type of command (move_joints, reset_joints, etc)
            feedback: MoveFeedback from the command
            hand: Which hand ('left' or 'right')
        """
        entry = CommandLogEntry(
            timestamp=feedback.command_timestamp,
            command_type=command_type,
            joint_commands={name: fb.angle for name, fb in feedback.joints.items()},
            control_mode=ControlMode.CASCADED_PID,  # Default mode
            hand=hand
        )

        # Add to buffer
        self.command_buffers[hand].append(entry)

        # Write to file
        with open(self.session_dir / f"{hand}_commands.jsonl", 'a') as f:
            json.dump(asdict(entry), f)
            f.write('\n')

        # Log feedback too
        self.log_feedback(feedback, hand)

    def log_feedback(self, feedback: MoveFeedback, hand: str):
        """Log feedback received from the hand

        Args:
            feedback: MoveFeedback from command
            hand: Which hand ('left' or 'right')
        """
        entry = FeedbackLogEntry(
            timestamp=time.time() - self.start_time,
            joints=feedback.joints,
            tactile=feedback.tactile,
            hand=hand
        )

        # Add to buffer
        self.feedback_buffers[hand].append(entry)

        # Write to file
        with open(self.session_dir / f"{hand}_feedback.jsonl", 'a') as f:
            # Need to convert dataclass objects to dictionaries
            serialized = {
                'timestamp': entry.timestamp,
                'joints': {
                    name: asdict(fb) for name, fb in entry.joints.items()
                },
                'tactile': {
                    name: asdict(fb) for name, fb in entry.tactile.items()
                },
                'hand': entry.hand
            }
            json.dump(serialized, f)
            f.write('\n')

    def save_metadata(self, metadata: Dict[str, Any]):
        """Save session metadata

        Args:
            metadata: Dictionary of metadata to save
        """
        metadata_path = self.session_dir / "metadata.json"
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)

    def plot_session(self, hands: Optional[List[str]] = None, show: bool = True,
                    save: bool = True):
        """Plot command and feedback data from the session

        Args:
            hands: Which hands to plot ('left', 'right', or both)
            show: Whether to show plots interactively
            save: Whether to save plots to files
        """
        try:
            import matplotlib.pyplot as plt
            import numpy as np
        except ImportError:
            logger.error("matplotlib is required for plotting")
            return

        if hands is None:
            hands = ['left', 'right']

        for hand in hands:
            if not self.command_buffers[hand] and not self.feedback_buffers[hand]:
                continue

            # Plot joint commands
            plt.figure(figsize=(12, 8))
            joint_data = {}

            # Collect command data
            for entry in self.command_buffers[hand]:
                t = entry.timestamp - self.start_time
                for joint, pos in entry.joint_commands.items():
                    if joint not in joint_data:
                        joint_data[joint] = {'times': [], 'positions': []}
                    joint_data[joint]['times'].append(t)
                    joint_data[joint]['positions'].append(pos)

            # Plot each joint
            for joint, data in joint_data.items():
                plt.plot(data['times'], data['positions'], label=joint)

            plt.xlabel('Time (s)')
            plt.ylabel('Joint Angle (degrees)')
            plt.title(f'{hand.title()} Hand Joint Commands')
            plt.legend()
            plt.grid(True)

            if save:
                plt.savefig(self.session_dir / f'{hand}_commands.png')

            # Plot tactile feedback
            if any(entry.tactile for entry in self.feedback_buffers[hand]):
                plt.figure(figsize=(12, 8))
                tactile_data = {}

                # Collect tactile data
                for entry in self.feedback_buffers[hand]:
                    t = entry.timestamp
                    for finger, data in entry.tactile.items():
                        if finger not in tactile_data:
                            tactile_data[finger] = {
                                'times': [],
                                'normal_force': [],
                                'tangential_force': []
                            }
                        tactile_data[finger]['times'].append(t)
                        tactile_data[finger]['normal_force'].append(data.normal_force)
                        tactile_data[finger]['tangential_force'].append(data.tangential_force)

                # Plot each finger's tactile data
                for finger, data in tactile_data.items():
                    plt.plot(data['times'], data['normal_force'],
                            label=f'{finger} normal', linestyle='-')
                    plt.plot(data['times'], data['tangential_force'],
                            label=f'{finger} tangential', linestyle='--')

                plt.xlabel('Time (s)')
                plt.ylabel('Force (N)')
                plt.title(f'{hand.title()} Hand Tactile Feedback')
                plt.legend()
                plt.grid(True)

                if save:
                    plt.savefig(self.session_dir / f'{hand}_tactile.png')

            plt.close('all')

        if show:
            plt.show()

    def close(self):
        """Close the logger and save any remaining data"""
        # Save summary statistics
        stats = {
            'duration': time.time() - self.start_time,
            'num_commands': {
                'left': len(self.command_buffers['left']),
                'right': len(self.command_buffers['right'])
            },
            'num_feedback': {
                'left': len(self.feedback_buffers['left']),
                'right': len(self.feedback_buffers['right'])
            }
        }

        self.save_metadata({
            'statistics': stats,
            'timestamp': datetime.now().isoformat()
        })

        logger.info(f"Logging session completed: {stats}")

        # Generate plots
        self.plot_session(show=False, save=True)
