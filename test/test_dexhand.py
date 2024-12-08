import numpy as np
import time
import logging
from typing import List, Optional, Union
import argparse
from pathlib import Path

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from src.dexhand_interface import (
    LeftDexHand, RightDexHand, ControlMode, JointMotor, ZCANWrapper
)
from src.dexhand_logger import DexHandLogger

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class DexHandTest:
    """Test sequence runner for dexterous hands"""

    def __init__(self, log_dir: str = "dexhand_logs", hands: List[str] = ["left"]):
        """Initialize test runner

        Args:
            log_dir: Directory for test logs
            hands: Which hands to test ("left", "right", or both)
        """
        # Validate hand selection
        valid_hands = {"left", "right"}
        if not all(hand in valid_hands for hand in hands):
            raise ValueError(f"Invalid hand selection. Must be 'left' or 'right'")
        self.hands = hands

        # Create shared ZCAN instance if using both hands
        self.zcan = ZCANWrapper() if len(hands) > 1 else None

        # Initialize hands
        self.hands_dict = {}
        if "left" in hands:
            self.hands_dict["left"] = LeftDexHand(self.zcan)
        if "right" in hands:
            self.hands_dict["right"] = RightDexHand(self.zcan)

        # Initialize logger
        self.logger = DexHandLogger(log_dir)

    def initialize_hands(self) -> bool:
        """Initialize all selected hands

        Returns:
            bool: True if all hands initialized successfully
        """
        if self.zcan is not None:
            # Initialize shared ZCAN instance
            if not self.zcan.open():
                logger.error("Failed to open shared ZCAN instance")
                return False
        for hand_name, hand in self.hands_dict.items():
            if not hand.init():
                logger.error(f"Failed to initialize {hand_name} hand")
                return False
        return True

    def run_test_sequence(self, repeat: int = 5, sleep_time: float = 0.5):
        """Run a test sequence of hand movements

        Args:
            repeat: Number of times to repeat sequence
            sleep_time: Time between movements in seconds
        """
        # Test parameters
        self.logger.save_metadata({
            'test_params': {
                'repeat': repeat,
                'sleep_time': sleep_time,
                'hands': self.hands
            }
        })

        # Define test positions
        num_motors = 12  # Both proximal and distal motors for 6 joints
        start_positions = np.zeros(num_motors)
        end_positions = np.array([3000] * num_motors)

        logger.info(f"Starting test sequence with {repeat} repetitions")
        for i in range(repeat):
            logger.info(f"Iteration {i+1}/{repeat}")

            # Test different motor enable patterns
            patterns = [
                # All motors enabled
                np.ones(num_motors, dtype=bool),
                # Only proximal motors
                np.array([1,0] * 6, dtype=bool),
                # Only distal motors
                np.array([0,1] * 6, dtype=bool),
                # Alternating joints
                np.array([1,1,0,0] * 3, dtype=bool),
            ]

            for pattern_idx, enable_pattern in enumerate(patterns):
                logger.info(f"Testing enable pattern {pattern_idx+1}/{len(patterns)}")
                self._run_movement_cycle(start_positions, end_positions, enable_pattern, sleep_time)

    def _run_movement_cycle(self, start_pos: np.ndarray, end_pos: np.ndarray,
                          enable_pattern: np.ndarray, sleep_time: float):
        """Run one cycle of movement with given parameters

        Args:
            start_pos: Starting position array
            end_pos: Ending position array
            enable_pattern: Boolean array indicating which motors to enable
            sleep_time: Time to sleep between movements
        """
        # Move to start position
        for hand_name, hand in self.hands_dict.items():
            t = time.time()
            success = hand.send_commands(
                positions=start_pos,
                enable_motors=enable_pattern,
                control_mode=ControlMode.CASCADED_PID
            )
            logger.info(f"Time taken to send command: {time.time() - t}")
            self.logger.log_command(start_pos, enable_pattern,
                                  ControlMode.CASCADED_PID, success, hand_name)

            t = time.time()
            hand.clear_errors()
            logger.info(f"Time taken to clear errors: {time.time() - t}")

            # Get and log feedback
            feedback = hand.receive_feedback()
            self.logger.log_feedback(feedback, hand_name)

            # Log detailed feedback
            for joint_id, joint_fb in feedback.items():
                logger.info(f"{hand_name} hand Joint {joint_id} feedback:")
                logger.info(f"  Proximal - Pos: {joint_fb.proximal.position:.1f}, "
                          f"Vel: {joint_fb.proximal.velocity:.1f}, "
                          f"Current: {joint_fb.proximal.current:.1f}mA")
                logger.info(f"  Distal   - Pos: {joint_fb.distal.position:.1f}, "
                          f"Vel: {joint_fb.distal.velocity:.1f}, "
                          f"Current: {joint_fb.distal.current:.1f}mA")

        time.sleep(sleep_time)

        # Move to end position
        for hand_name, hand in self.hands_dict.items():
            t = time.time()
            success = hand.send_commands(
                positions=end_pos,
                enable_motors=enable_pattern,
                control_mode=ControlMode.CASCADED_PID
            )
            logger.info(f"Time taken to send command: {time.time() - t}")
            self.logger.log_command(end_pos, enable_pattern,
                                  ControlMode.CASCADED_PID, success, hand_name)

            t = time.time()
            hand.clear_errors()
            logger.info(f"Time taken to clear errors: {time.time() - t}")

            feedback = hand.receive_feedback()
            self.logger.log_feedback(feedback, hand_name)

        time.sleep(sleep_time)

    def close(self):
        """Clean up test resources"""
        # Close hands (only rightmost hand will close shared ZCAN)
        for hand in self.hands_dict.values():
            hand.close()

        if self.zcan is not None:
            # Close shared ZCAN instance
            self.zcan.close()

        # Close logger
        self.logger.close()

def main():
    parser = argparse.ArgumentParser(description="Test dexterous hand control")
    parser.add_argument("--hands", nargs="+", default=["left"],
                       choices=["left", "right"],
                       help="Which hands to test")
    parser.add_argument("--repeat", type=int, default=3,
                       help="Number of times to repeat test sequence")
    parser.add_argument("--sleep-time", type=float, default=0.5,
                       help="Time between movements in seconds")
    parser.add_argument("--log-dir", type=str, default="dexhand_logs",
                       help="Directory for log files")
    args = parser.parse_args()

    test = DexHandTest(args.log_dir, args.hands)

    try:
        if not test.initialize_hands():
            return

        for hand in test.hands_dict.values():
            hand.zcan.dump_channel_state(hand.config.channel)
            for i in range(6):
                hand.send_single_command(hand.config.ctrl_offset + 1 + i, np.random.randint(0, 3000), np.random.randint(0, 3000), 0x3)

        test.run_test_sequence(args.repeat, args.sleep_time)

    except KeyboardInterrupt:
        logger.info("Test interrupted by user")

    finally:
        test.close()
        logger.info("Test completed")

if __name__ == "__main__":
    main()
