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
    MotorCommand, JointFeedback, ControlMode,
    LeftDexHand, RightDexHand
)

logger = logging.getLogger(__name__)

@dataclass
class CommandLogEntry:
    """Log entry for a command sent to the hand"""
    timestamp: float
    positions: List[float]
    enable_motors: List[bool]
    control_mode: int
    success: bool
    hand: str  # 'left' or 'right'

@dataclass
class FeedbackLogEntry:
    """Log entry for feedback received from the hand"""
    timestamp: float
    joint_positions: List[float]
    joint_velocities: List[float]
    joint_currents: List[float]
    errors: List[int]
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

        # Initialize in-memory buffers for each hand
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

    def log_command(self,
                   positions: np.ndarray,
                   enable_motors: np.ndarray,
                   control_mode: ControlMode,
                   success: bool,
                   hand: str):
        """Log a command sent to the hand

        Args:
            positions: Array of joint positions
            enable_motors: Array of motor enable flags
            control_mode: Control mode used
            success: Whether command was sent successfully
            hand: Which hand ('left' or 'right')
        """
        entry = CommandLogEntry(
            timestamp=time.time() - self.start_time,
            positions=positions.tolist(),
            enable_motors=enable_motors.tolist(),
            control_mode=int(control_mode),
            success=success,
            hand=hand
        )

        # Add to buffer
        self.command_buffers[hand].append(entry)

        # Write to file
        with open(self.session_dir / f"{hand}_commands.jsonl", 'a') as f:
            json.dump(asdict(entry), f)
            f.write('\n')

    def log_feedback(self, feedback: Dict[int, JointFeedback], hand: str):
        """Log feedback received from the hand

        Args:
            feedback: Dictionary of joint feedback data
            hand: Which hand ('left' or 'right')
        """
        # Extract arrays of values
        joint_positions = []
        joint_velocities = []
        joint_currents = []
        errors = []

        for joint_id in sorted(feedback.keys()):
            joint = feedback[joint_id]
            # Proximal motor
            joint_positions.append(joint.proximal.position)
            joint_velocities.append(joint.proximal.velocity)
            joint_currents.append(joint.proximal.current)
            # Distal motor
            joint_positions.append(joint.distal.position)
            joint_velocities.append(joint.distal.velocity)
            joint_currents.append(joint.distal.current)

        entry = FeedbackLogEntry(
            timestamp=time.time() - self.start_time,
            joint_positions=joint_positions,
            joint_velocities=joint_velocities,
            joint_currents=joint_currents,
            errors=errors,
            hand=hand
        )

        # Add to buffer
        self.feedback_buffers[hand].append(entry)

        # Write to file
        with open(self.session_dir / f"{hand}_feedback.jsonl", 'a') as f:
            json.dump(asdict(entry), f)
            f.write('\n')

    def save_metadata(self, metadata: Dict[str, Any]):
        """Save session metadata

        Args:
            metadata: Dictionary of metadata to save
        """
        metadata_path = self.session_dir / "metadata.json"
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)

    def plot_session(self, hands: Optional[List[str]] = None, show: bool = True, save: bool = True):
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

            # Plot commands
            plt.figure(figsize=(12, 6))
            command_times = [entry.timestamp for entry in self.command_buffers[hand]]
            command_array = np.array(command_times)

            for joint in range(12):
                positions = [entry.positions[joint] for entry in self.command_buffers[hand]]
                enables = [entry.enable_motors[joint] for entry in self.command_buffers[hand]]
                positions_array = np.array(positions)
                enables_array = np.array(enables)

                # Plot enabled segments with full opacity
                if np.any(enables_array):
                    enabled_mask = enables_array
                    plt.plot(command_array[enabled_mask],
                            positions_array[enabled_mask],
                            label=f'Joint {joint}',
                            alpha=1.0)

                # Plot disabled segments with reduced opacity
                if np.any(~enables_array):
                    disabled_mask = ~enables_array
                    plt.plot(command_array[disabled_mask],
                            positions_array[disabled_mask],
                            alpha=0.3,
                            linestyle='--',
                            color=plt.gca().lines[-1].get_color() if np.any(enables_array) else None)

            plt.xlabel('Time (s)')
            plt.ylabel('Position Command')
            plt.title(f'{hand.title()} Hand Position Commands')
            plt.legend()
            plt.grid(True)

            if save:
                plt.savefig(self.session_dir / f'{hand}_commands.png')

            # TODO: feedback not working yet
            # # Plot feedback
            # plt.figure(figsize=(12, 6))
            # feedback_times = [entry.timestamp for entry in self.feedback_buffers[hand]]
            # for joint in range(12):
            #     positions = [entry.joint_positions[joint] for entry in self.feedback_buffers[hand]]
            #     plt.plot(feedback_times, positions, label=f'Joint {joint}')
            # plt.xlabel('Time (s)')
            # plt.ylabel('Position Feedback')
            # plt.title(f'{hand.title()} Hand Position Feedback')
            # plt.legend()
            # plt.grid(True)

            # if save:
            #     plt.savefig(self.session_dir / f'{hand}_feedback.png')

            # Close figures to free memory
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
