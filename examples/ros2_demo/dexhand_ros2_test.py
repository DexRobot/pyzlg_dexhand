#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import argparse
import numpy as np
from enum import Enum, auto
from typing import List, Dict


class TestSequence(Enum):
    """Different test sequences available"""

    FINGER_WAVE = auto()  # Sequential finger curling
    SPREAD = auto()  # Spread fingers
    PINCH = auto()  # Thumb-to-finger pinching
    FIST = auto()  # Make fist
    ALL = auto()  # Test all movements


class DexHandTestNode(Node):
    """Test node for exercising DexHand joints"""

    # Joint limits in degrees (will be converted to radians)
    JOINT_LIMITS = {
        "thumb_rot": (0, 150),  # Thumb rotation has extended range
        "thumb_pip": (0, 90),  # Thumb PIP has different sign
        "spread": (0, 30),  # Finger spread is limited
        "standard": (0, 90),  # Standard joint range
    }

    def __init__(self, hands: List[str], cycle_time: float = 3.0):
        super().__init__("dexhand_test")

        # Store configuration
        self.hands = hands
        self.cycle_time = cycle_time

        # Convert joint limits to radians
        self.joint_limits_rad = {
            name: (np.deg2rad(min_deg), np.deg2rad(max_deg))
            for name, (min_deg, max_deg) in self.JOINT_LIMITS.items()
        }

        # Build joint lists for each hand
        self.joint_names = []
        for hand in hands:
            prefix = "l" if hand == "left" else "r"
            for finger in range(1, 6):
                self.joint_names.extend(
                    [
                        f"{prefix}_f_joint{finger}_1",  # Finger spread
                        f"{prefix}_f_joint{finger}_2",  # Finger PIP
                        f"{prefix}_f_joint{finger}_3",  # Finger DIP
                        f"{prefix}_f_joint{finger}_4",  # Finger DIP (coupled)
                    ]
                )

        # Create publisher for commands
        self.cmd_pub = self.create_publisher(
            JointState, "joint_commands", 10  # Using global joint states topic
        )

        # Create dictionary for current joint positions
        self.current_pos = {name: 0.0 for name in self.joint_names}

        # Movement settings
        self.update_rate = 50  # Hz
        self.timer_period = 1.0 / self.update_rate

        # Start test sequence timer
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Test sequence state
        self.current_sequence = None
        self.sequence_start_time = None

        self.get_logger().info(
            f"DexHand test node initialized:\n"
            f'  Hands: {", ".join(hands)}\n'
            f"  Cycle time: {cycle_time}s\n"
            f"  Update rate: {self.update_rate}Hz"
        )

    def get_joint_limit(self, joint_name: str) -> tuple:
        """Get radian limits for a joint"""
        if "joint1_1" in joint_name:  # Thumb rotation
            return self.joint_limits_rad["thumb_rot"]
        elif "joint1_2" in joint_name:  # Thumb PIP - no sign flip
            return self.joint_limits_rad["thumb_pip"]
        elif "_1" in joint_name:  # Spread joints
            return self.joint_limits_rad["spread"]
        else:  # Standard joints
            return self.joint_limits_rad["standard"]

    def interpolate_positions(self, targets: Dict[str, float], alpha: float):
        """Linearly interpolate between current and target positions"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for name, target in targets.items():
            current = self.current_pos.get(name, 0.0)
            new_pos = current + alpha * (target - current)

            # Clip to joint limits
            min_pos, max_pos = self.get_joint_limit(name)
            new_pos = np.clip(new_pos, min_pos, max_pos)

            msg.name.append(name)
            msg.position.append(new_pos)
            self.current_pos[name] = new_pos

        return msg

    def get_target_positions(
        self, sequence: TestSequence, t: float
    ) -> Dict[str, float]:
        """Get target positions for current time in sequence

        Args:
            sequence: Which sequence to generate targets for
            t: Time in sequence (0-1)

        Returns:
            Dictionary of joint name to target position (in radians)
        """
        positions = {}

        if sequence == TestSequence.FINGER_WAVE:
            # Wave flexion from pinky to index
            t = t * 5  # Stretch over 5 fingers
            if t < 1:  # Pinky
                angle = np.deg2rad(np.sin(t * np.pi) * 90)  # Full range
                for hand in self.hands:
                    prefix = "l" if hand == "left" else "r"
                    positions[f"{prefix}_f_joint5_2"] = angle  # PIP
                    positions[f"{prefix}_f_joint5_3"] = angle  # DIP
                    positions[f"{prefix}_f_joint5_4"] = angle  # DIP (coupled)
            elif t < 2:  # Ring
                angle = np.deg2rad(np.sin((t - 1) * np.pi) * 90)
                for hand in self.hands:
                    prefix = "l" if hand == "left" else "r"
                    positions[f"{prefix}_f_joint4_2"] = angle
                    positions[f"{prefix}_f_joint4_3"] = angle
                    positions[f"{prefix}_f_joint4_4"] = angle
            elif t < 3:  # Middle
                angle = np.deg2rad(np.sin((t - 2) * np.pi) * 90)
                for hand in self.hands:
                    prefix = "l" if hand == "left" else "r"
                    positions[f"{prefix}_f_joint3_2"] = angle
                    positions[f"{prefix}_f_joint3_3"] = angle
                    positions[f"{prefix}_f_joint3_4"] = angle
            elif t < 4:  # Index
                angle = np.deg2rad(np.sin((t - 3) * np.pi) * 90)
                for hand in self.hands:
                    prefix = "l" if hand == "left" else "r"
                    positions[f"{prefix}_f_joint2_2"] = angle
                    positions[f"{prefix}_f_joint2_3"] = angle
                    positions[f"{prefix}_f_joint2_4"] = angle
            elif t < 5:  # Thumb
                angle = np.deg2rad(np.sin((t - 4) * np.pi) * 30)
                for hand in self.hands:
                    prefix = "l" if hand == "left" else "r"
                    positions[f"{prefix}_f_joint1_1"] = angle  # Rotation
                    positions[f"{prefix}_f_joint1_2"] = angle  # PIP (no sign flip)
                    positions[f"{prefix}_f_joint1_3"] = angle  # DIP
                    positions[f"{prefix}_f_joint1_4"] = angle  # DIP

        elif sequence == TestSequence.SPREAD:
            # Spread fingers out and back
            angle = np.deg2rad(np.sin(t * np.pi) * 30)  # Max spread angle
            for hand in self.hands:
                prefix = "l" if hand == "left" else "r"
                positions[f"{prefix}_f_joint2_1"] = angle  # Index spread
                positions[f"{prefix}_f_joint3_1"] = angle  # Middle spread
                positions[f"{prefix}_f_joint4_1"] = angle  # Ring spread
                positions[f"{prefix}_f_joint5_1"] = angle  # Pinky spread

        elif sequence == TestSequence.PINCH:
            # Sequential thumb-to-finger pinching
            t = t * 4  # Four fingers to pinch
            thumb_rot = np.deg2rad(120)  # Base thumb rotation in radians

            for hand in self.hands:
                prefix = "l" if hand == "left" else "r"
                # Keep thumb rotated
                positions[f"{prefix}_f_joint1_1"] = thumb_rot

                # Move thumb and one finger at a time
                if t < 1:  # Index
                    s = np.sin(t * np.pi)
                    positions[f"{prefix}_f_joint1_2"] = np.deg2rad(s * 30)  # Thumb PIP
                    positions[f"{prefix}_f_joint1_3"] = np.deg2rad(s * 30)  # Thumb DIP
                    positions[f"{prefix}_f_joint1_4"] = np.deg2rad(s * 30)  # Thumb DIP
                    positions[f"{prefix}_f_joint2_2"] = np.deg2rad(s * 90)  # Index
                    positions[f"{prefix}_f_joint2_3"] = np.deg2rad(s * 90)
                    positions[f"{prefix}_f_joint2_4"] = np.deg2rad(s * 90)
                elif t < 2:  # Middle
                    s = np.sin((t - 1) * np.pi)
                    positions[f"{prefix}_f_joint1_2"] = np.deg2rad(s * 30)  # Thumb
                    positions[f"{prefix}_f_joint1_3"] = np.deg2rad(s * 30)
                    positions[f"{prefix}_f_joint1_4"] = np.deg2rad(s * 30)
                    positions[f"{prefix}_f_joint3_2"] = np.deg2rad(s * 90)  # Middle
                    positions[f"{prefix}_f_joint3_3"] = np.deg2rad(s * 90)
                    positions[f"{prefix}_f_joint3_4"] = np.deg2rad(s * 90)
                elif t < 3:  # Ring
                    s = np.sin((t - 2) * np.pi)
                    positions[f"{prefix}_f_joint1_2"] = np.deg2rad(s * 30)  # Thumb
                    positions[f"{prefix}_f_joint1_3"] = np.deg2rad(s * 30)
                    positions[f"{prefix}_f_joint1_4"] = np.deg2rad(s * 30)
                    positions[f"{prefix}_f_joint4_2"] = np.deg2rad(s * 90)  # Ring
                    positions[f"{prefix}_f_joint4_3"] = np.deg2rad(s * 90)
                    positions[f"{prefix}_f_joint4_4"] = np.deg2rad(s * 90)
                elif t < 4:  # Pinky
                    s = np.sin((t - 3) * np.pi)
                    positions[f"{prefix}_f_joint1_2"] = np.deg2rad(s * 30)  # Thumb
                    positions[f"{prefix}_f_joint1_3"] = np.deg2rad(s * 30)
                    positions[f"{prefix}_f_joint1_4"] = np.deg2rad(s * 30)
                    positions[f"{prefix}_f_joint5_2"] = np.deg2rad(s * 90)  # Pinky
                    positions[f"{prefix}_f_joint5_3"] = np.deg2rad(s * 90)
                    positions[f"{prefix}_f_joint5_4"] = np.deg2rad(s * 90)

        elif sequence == TestSequence.FIST:
            # Make a fist
            angle = np.sin(t * np.pi)
            for hand in self.hands:
                prefix = "l" if hand == "left" else "r"
                # Curl all fingers
                positions[f"{prefix}_f_joint2_2"] = np.deg2rad(angle * 90)  # Index
                positions[f"{prefix}_f_joint2_3"] = np.deg2rad(angle * 90)
                positions[f"{prefix}_f_joint2_4"] = np.deg2rad(angle * 90)
                positions[f"{prefix}_f_joint3_2"] = np.deg2rad(angle * 90)  # Middle
                positions[f"{prefix}_f_joint3_3"] = np.deg2rad(angle * 90)
                positions[f"{prefix}_f_joint3_4"] = np.deg2rad(angle * 90)
                positions[f"{prefix}_f_joint4_2"] = np.deg2rad(angle * 90)  # Ring
                positions[f"{prefix}_f_joint4_3"] = np.deg2rad(angle * 90)
                positions[f"{prefix}_f_joint4_4"] = np.deg2rad(angle * 90)
                positions[f"{prefix}_f_joint5_2"] = np.deg2rad(angle * 90)  # Pinky
                positions[f"{prefix}_f_joint5_3"] = np.deg2rad(angle * 90)
                positions[f"{prefix}_f_joint5_4"] = np.deg2rad(angle * 90)
                # Thumb
                positions[f"{prefix}_f_joint1_1"] = np.deg2rad(angle * 120)  # Rotation
                positions[f"{prefix}_f_joint1_2"] = np.deg2rad(angle * 30)  # PIP
                positions[f"{prefix}_f_joint1_3"] = np.deg2rad(angle * 30)  # DIP
                positions[f"{prefix}_f_joint1_4"] = np.deg2rad(angle * 30)  # DIP

        return positions

    def timer_callback(self):
        """Execute test sequences"""
        now = self.get_clock().now()

        if self.current_sequence is None:
            # Start with finger wave
            self.current_sequence = TestSequence.FINGER_WAVE
            self.sequence_start_time = now
            self.get_logger().info(f"Starting sequence: {self.current_sequence.name}")
            return

        # Get time in current sequence
        t = (now - self.sequence_start_time).nanoseconds / 1e9

        if t > self.cycle_time:
            # Call the ros2 service to reset the hand
            reset_client = self.create_client(Trigger, "/reset_hands")
            if reset_client.wait_for_service(timeout_sec=1.0):
                request = Trigger.Request()
                reset_client.call_async(request)
            else:
                self.get_logger().warn("Reset service not available")

            # Move to next sequence
            if self.current_sequence == TestSequence.FINGER_WAVE:
                next_seq = TestSequence.SPREAD
            elif self.current_sequence == TestSequence.SPREAD:
                next_seq = TestSequence.PINCH
            elif self.current_sequence == TestSequence.PINCH:
                next_seq = TestSequence.FIST
            elif self.current_sequence == TestSequence.FIST:
                next_seq = TestSequence.FINGER_WAVE

            self.current_sequence = next_seq
            self.sequence_start_time = now
            self.get_logger().info(f"Starting sequence: {self.current_sequence.name}")
            return

        # Get normalized time in sequence (0-1)
        t_norm = t / self.cycle_time

        # Get target positions for current sequence
        targets = self.get_target_positions(self.current_sequence, t_norm)

        # Interpolate and publish
        if targets:
            msg = self.interpolate_positions(targets, 0.1)  # Low pass filter
            self.cmd_pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description="DexHand Test Node")
    parser.add_argument(
        "--hands",
        nargs="+",
        choices=["left", "right"],
        required=True,
        help="Which hands to test",
    )
    parser.add_argument(
        "--cycle-time",
        type=float,
        default=3.0,
        help="Time for each movement cycle (seconds)",
    )

    args = parser.parse_args()

    rclpy.init()

    try:
        node = DexHandTestNode(hands=args.hands, cycle_time=args.cycle_time)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
