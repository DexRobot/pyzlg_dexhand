#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
import argparse
import numpy as np
from typing import Dict, List, Optional
from enum import Enum
import time
import yaml
import os.path

from src.dexhand_interface import (
    LeftDexHand, RightDexHand, ControlMode, DexJoint, ZCANWrapper
)
from src.zcan_wrapper import MockZCANWrapper

class HardwareMapping(Enum):
    """Mapping between URDF and hardware joints"""
    THUMB_DIP = (DexJoint.THUMB_DIP, ["f_joint1_3", "f_joint1_4"])
    THUMB_SPREAD = (DexJoint.THUMB_PIP, ["f_joint1_2"])
    THUMB_ROTATION = (DexJoint.THUMB_ROT, ["f_joint1_1"])
    FINGER_SPREAD = (DexJoint.FINGER_SPREAD, ["f_joint2_1", "f_joint4_1", "f_joint5_1"])
    INDEX_DIP = (DexJoint.INDEX_DIP, ["f_joint2_3", "f_joint2_4"])
    INDEX_PIP = (DexJoint.INDEX_PIP, ["f_joint2_2"])
    MIDDLE_DIP = (DexJoint.MIDDLE_DIP, ["f_joint3_3", "f_joint3_4"])
    MIDDLE_PIP = (DexJoint.MIDDLE_PIP, ["f_joint3_2"])
    RING_DIP = (DexJoint.RING_DIP, ["f_joint4_3", "f_joint4_4"])
    RING_PIP = (DexJoint.RING_PIP, ["f_joint4_2"])
    PINKY_DIP = (DexJoint.PINKY_DIP, ["f_joint5_3", "f_joint5_4"])
    PINKY_PIP = (DexJoint.PINKY_PIP, ["f_joint5_2"])

class JointMapping:
    """Maps between URDF joints and hardware joints"""

    def __init__(self, prefix: str = 'l'):
        """Initialize joint mapping"""
        self.prefix = prefix

        # Create mapping from URDF joint names to hardware joints
        self.urdf_to_hw = {}
        for hw_mapping in HardwareMapping:
            dex_joint, urdf_joints = hw_mapping.value
            for urdf_joint in urdf_joints:
                full_name = f"{prefix}_{urdf_joint}"
                self.urdf_to_hw[full_name] = dex_joint

        # Get all possible URDF joint names
        self.joint_names = sorted(list(self.urdf_to_hw.keys()))

    def map_command(self, joint_values: Dict[str, float]) -> Dict[DexJoint, float]:
        """Map URDF joint values to hardware commands"""
        # Group joint values by hardware joint
        hw_joint_values = {dex_joint: [] for dex_joint in DexJoint}

        for name, value in joint_values.items():
            if name in self.urdf_to_hw:
                dex_joint = self.urdf_to_hw[name]
                hw_joint_values[dex_joint].append(value)

        # Average values for each hardware joint
        command = {}
        for dex_joint in DexJoint:
            values = hw_joint_values[dex_joint]
            if values:
                command[dex_joint] = float(np.rad2deg(sum(values) / len(values)))

        return command

class DexHandNode(Node):
    """ROS2 Node for controlling one or both DexHands"""

    def __init__(self, config: dict):
        super().__init__('dexhand')

        # Get configuration values
        hands = config.get('hands', ['right'])
        control_mode = config.get('mode', 'cascaded_pid')
        send_rate = config.get('rate', 100.0)
        filter_alpha = config.get('alpha', 0.1)
        self.command_topic = config.get('topic', '/joint_commands')
        self.is_mock = config.get('mock', False)

        # Initialize shared ZCAN
        self.zcan = ZCANWrapper() if not self.is_mock else MockZCANWrapper()
        if not self.zcan.open():
            raise RuntimeError("Failed to open ZCAN device")

        # Set up control mode
        self.control_mode_map = {
            'zero_torque': ControlMode.ZERO_TORQUE,
            'current': ControlMode.CURRENT,
            'speed': ControlMode.SPEED,
            'hall_position': ControlMode.HALL_POSITION,
            'cascaded_pid': ControlMode.CASCADED_PID,
            'protect_hall_position': ControlMode.PROTECT_HALL_POSITION
        }
        self.control_mode = self.control_mode_map.get(control_mode, ControlMode.CASCADED_PID)
        self.filter_alpha = filter_alpha

        # Initialize hands with shared ZCAN
        self.hands = {}
        self.joint_mappings = {}
        self.last_commands = {}

        for hand in hands:
            # Initialize hand with shared ZCAN
            hand_class = LeftDexHand if hand == 'left' else RightDexHand
            self.hands[hand] = hand_class(self.zcan)
            if not self.hands[hand].init():
                raise RuntimeError(f"Failed to initialize {hand} hand")

            # Initialize joint mapping
            self.joint_mappings[hand] = JointMapping('l' if hand == 'left' else 'r')

            # Initialize last command
            self.last_commands[hand] = {}


        # Initialize command subscriber with configurable topic
        self.create_subscription(
            JointState,
            self.command_topic,
            self.command_callback,
            10
        )

        # Initialize reset service
        self.create_service(
            Trigger,
            'reset_hands',
            self.reset_callback
        )

        # Set up command sending timer
        period = 1.0 / send_rate
        self.timer = self.create_timer(period, self.send_commands)

        self.get_logger().info(
            f'DexHand node initialized:\n'
            f'  Hands: {", ".join(hands)}\n'
            f'  Control mode: {control_mode}\n'
            f'  Send rate: {send_rate} Hz\n'
            f'  Filter alpha: {filter_alpha}'
        )

    def command_callback(self, msg: JointState):
        """Handle incoming joint commands"""
        try:
            # Create dictionary of joint values
            joint_values = {}
            for name, pos in zip(msg.name, msg.position):
                joint_values[name] = pos

            # Process command for each hand
            for hand, mapping in self.joint_mappings.items():
                # Map to hardware joints
                command = mapping.map_command(joint_values)

                # Apply low-pass filter
                if not self.last_commands[hand]:
                    # First command, no filtering
                    self.last_commands[hand] = command
                else:
                    for joint, value in command.items():
                        if joint not in self.last_commands[hand]:
                            self.last_commands[hand][joint] = value
                        else:
                            self.last_commands[hand][joint] = (
                                (1 - self.filter_alpha) * self.last_commands[hand][joint] +
                                self.filter_alpha * value
                            )

        except Exception as e:
            self.get_logger().error(f'Error in command callback: {str(e)}')

    def send_commands(self):
        """Send filtered commands to all hands"""
        try:
            # Send commands to each hand
            for hand, hand_interface in self.hands.items():
                command = self.last_commands.get(hand, {})

                # Use move_joints interface
                hand_interface.move_joints(
                    thumb_dip=command.get(DexJoint.THUMB_DIP),
                    thumb_pip=command.get(DexJoint.THUMB_PIP),
                    thumb_rot=command.get(DexJoint.THUMB_ROT),
                    finger_spread=command.get(DexJoint.FINGER_SPREAD),
                    index_dip=command.get(DexJoint.INDEX_DIP),
                    index_pip=command.get(DexJoint.INDEX_PIP),
                    middle_dip=command.get(DexJoint.MIDDLE_DIP),
                    middle_pip=command.get(DexJoint.MIDDLE_PIP),
                    ring_dip=command.get(DexJoint.RING_DIP),
                    ring_pip=command.get(DexJoint.RING_PIP),
                    pinky_dip=command.get(DexJoint.PINKY_DIP),
                    pinky_pip=command.get(DexJoint.PINKY_PIP),
                    control_mode=self.control_mode
                )

                # Clear errors
                hand_interface.clear_errors()

        except Exception as e:
            self.get_logger().error(f'Error sending commands: {str(e)}')

    def reset_callback(self, request, response):
        """Reset all hands with bend-straighten sequence"""
        try:
            success = True

            # First bend all joints to 30 degrees
            for hand in self.hands.values():
                hand.reset_joints(30.0)
                hand.clear_errors()

            # Wait a moment
            time.sleep(0.5)

            # Then straighten to 0 degrees
            for hand in self.hands.values():
                hand.reset_joints(0.0)
                hand.clear_errors()

            # Clear command history
            for hand in self.hands:
                self.last_commands[hand] = {}

            # Set response
            response.success = success
            response.message = 'Hand reset sequence completed successfully' if success else \
                             'Hand reset sequence failed'
            return response

        except Exception as e:
            response.success = False
            response.message = f'Error in reset sequence: {str(e)}'
            return response

    def on_shutdown(self):
        """Clean up on node shutdown"""
        for hand in self.hands.values():
            hand.close()

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='DexHand ROS2 Node')
    default_config = os.path.join(os.path.dirname(__file__), 'config', 'ros_node.yaml')
    parser.add_argument('--config', type=str, default=default_config,
                      help='Path to configuration file')
    args = parser.parse_args()

    # Load configuration
    try:
        with open(args.config, 'r') as f:
            config = yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading config file: {e}")
        return

    # Initialize ROS
    rclpy.init()

    try:
        # Create and spin node
        node = DexHandNode(config=config)

        try:
            rclpy.spin(node)
        finally:
            node.on_shutdown()
            node.destroy_node()

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
