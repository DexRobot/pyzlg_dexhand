from typing import List, Dict, Optional, Tuple
import logging
from enum import IntEnum, auto
import time
import numpy as np
from dataclasses import dataclass
from zcan_wrapper import ZCANWrapper, ZCANDeviceType, ZCANFilterConfig

logger = logging.getLogger(__name__)

class ControlMode(IntEnum):
    """Control modes for the dexterous hand"""
    ZERO_TORQUE = 0x00
    CURRENT = 0x11
    SPEED = 0x22
    HALL_POSITION = 0x33
    CASCADED_PID = 0x44
    PROTECT_HALL_POSITION = 0x55

class JointMotor(IntEnum):
    """Motor selection flags"""
    NONE = 0x00
    PROXIMAL = 0x01
    DISTAL = 0x02
    ALL = 0x03

@dataclass
class HandConfig:
    """Configuration for CAN communication"""
    channel: int
    ctrl_offset: int
    ce_offset: int
    recv_offset: int

@dataclass
class MotorCommand:
    """Command for a pair of motors (proximal and distal)"""
    proximal_position: int = 0
    distal_position: int = 0
    control_mode: ControlMode = ControlMode.CASCADED_PID
    motor_enable: int = JointMotor.ALL

@dataclass
class MotorFeedback:
    """Feedback from a single motor"""
    current: float  # mA
    velocity: float  # rpm
    position: float  # encoder counts
    angle: float    # degrees

@dataclass
class JointFeedback:
    """Feedback from a joint (proximal + distal motors)"""
    proximal: MotorFeedback
    distal: MotorFeedback

class DexHandBase:
    """Base class for dexterous hand control"""

    NUM_MOTORS = 12

    def __init__(self, config: HandConfig, zcan: Optional[ZCANWrapper] = None):
        """Initialize dexterous hand interface

        Args:
            config: CAN configuration for this hand
            zcan: Optional existing ZCANWrapper instance to share between hands
        """
        self.config = config
        self.zcan = zcan if zcan else ZCANWrapper()
        self._owns_zcan = zcan is None  # Track if we created the ZCAN instance

        # State tracking
        self._last_positions = np.zeros(self.NUM_MOTORS)
        self._motor_modes = np.zeros(self.NUM_MOTORS, dtype=int)
        self._motor_positions = np.zeros(self.NUM_MOTORS, dtype=int)
        self._motor_velocities = np.zeros(self.NUM_MOTORS, dtype=int)
        self._motor_currents = np.zeros(self.NUM_MOTORS, dtype=int)
        self._motor_errors = np.zeros(self.NUM_MOTORS, dtype=int)

    def init(self, device_type: ZCANDeviceType = ZCANDeviceType.ZCAN_USBCANFD_200U,
             device_index: int = 0) -> bool:
        """Initialize CAN communication

        Args:
            device_type: Type of CAN device
            device_index: Device index

        Returns:
            bool: True if initialization successful
        """
        # Only open device if we own the ZCAN instance
        if self._owns_zcan:
            if not self.zcan.open(device_type, device_index):
                logger.error("Failed to open CAN device")
                return False

        # Configure our channel
        if not self.zcan.configure_channel(self.config.channel):
            logger.error(f"Failed to configure channel {self.config.channel}")
            return False

        # Set up message filters
        filters = [
            # Standard frames for control
            ZCANFilterConfig(type=0, start_id=0x0, end_id=0x7FF),
            # Extended frames if needed
            ZCANFilterConfig(type=1, start_id=0x0, end_id=0x1FFFFFFF)
        ]
        if not self.zcan.set_filter(self.config.channel, filters):
            logger.error(f"Failed to set CAN filters for channel {self.config.channel}")
            return False

        return True

    def send_commands(self, positions: np.ndarray,
                     enable_motors: Optional[np.ndarray] = None,
                     control_mode: ControlMode = ControlMode.CASCADED_PID) -> bool:
        """Send position commands to all motors

        Args:
            positions: Array of 12 position commands
            enable_motors: Optional boolean array indicating which motors to enable.
                         If None, all motors are enabled.
            control_mode: Control mode for all motors

        Returns:
            bool: True if all commands sent successfully
        """
        if len(positions) != self.NUM_MOTORS:
            raise ValueError(f"Expected {self.NUM_MOTORS} positions, got {len(positions)}")

        # Default to enabling all motors
        if enable_motors is None:
            enable_motors = np.ones(self.NUM_MOTORS, dtype=bool)
        elif len(enable_motors) != self.NUM_MOTORS:
            raise ValueError(f"Expected {self.NUM_MOTORS} enable flags, got {len(enable_motors)}")

        failed_count = 0
        for i in range(0, self.NUM_MOTORS, 2):
            # Determine which motors in this pair to enable
            enable_flags = enable_motors[i:i+2]
            if not any(enable_flags):
                continue

            motor_enable = (JointMotor.PROXIMAL if enable_flags[0] else 0) | \
                         (JointMotor.DISTAL if enable_flags[1] else 0)

            # Send command
            success = self.send_single_command(
                joint_id=self.config.ctrl_offset + i//2 + 1,
                proximal_pos=int(positions[i]),
                distal_pos=int(positions[i+1]),
                control_mode=control_mode,
                motor_enable=motor_enable
            )

            if success:
                # Update last positions
                self._last_positions[i] = positions[i]
                self._last_positions[i+1] = positions[i+1]
            else:
                failed_count += 1

        return failed_count == 0

    def send_single_command(self, joint_id: int, proximal_pos: int, distal_pos: int,
                        motor_enable: int, control_mode: ControlMode = ControlMode.CASCADED_PID) -> bool:
        """Send command to a single joint (pair of motors)"""
        logger.debug(
            f"Sending command to joint {joint_id:x}:\n"
            f"  Proximal: {proximal_pos}, Distal: {distal_pos}\n"
            f"  Enable: {motor_enable:x}, Mode: {control_mode:x}"
        )

        # Build message data
        data = bytearray(6)  # Should match working implementation size
        data[0] = control_mode & 0xFF
        data[1] = motor_enable & 0xFF
        data[2] = proximal_pos & 0xFF
        data[3] = (proximal_pos >> 8) & 0xFF
        data[4] = distal_pos & 0xFF
        data[5] = (distal_pos >> 8) & 0xFF

        logger.debug(f"Message data: {[hex(x) for x in data]}")

        success = self.zcan.send_fd_message(self.config.channel, joint_id, data)
        logger.debug(f"Send result: {'Success' if success else 'Failed'}")
        return success

    def clear_errors(self) -> bool:
        """Clear error states on all motors

        Returns:
            bool: True if all clear commands sent successfully
        """
        failed_count = 0
        for i in range(self.NUM_MOTORS // 2):
            if not self.clear_single_error(self.config.ce_offset + i + 1):
                failed_count += 1
        return failed_count == 0

    def clear_single_error(self, joint_id: int) -> bool:
        """Clear error state on a single joint

        Args:
            joint_id: CAN ID for the joint

        Returns:
            bool: True if clear command sent successfully
        """
        # Standard clear error message
        data = bytes([0x03, 0xA4])
        return self.zcan.send_fd_message(self.config.channel, joint_id, data)

    def receive_feedback(self) -> Dict[int, JointFeedback]:
        """Receive and decode feedback from all joints

        Returns:
            Dict mapping joint ID to feedback data
        """
        feedback = {}
        messages = self.zcan.receive_fd_messages(self.config.channel)

        for msg_id, data, timestamp in messages:
            joint_idx = msg_id - self.config.recv_offset
            if not 0 <= joint_idx < self.NUM_MOTORS:
                continue

            # Parse feedback data
            motor_idx = joint_idx * 2
            self._motor_modes[motor_idx] = data[0]
            self._motor_positions[motor_idx] = int.from_bytes(data[1:3], 'little')
            self._motor_velocities[motor_idx] = int.from_bytes(data[3:5], 'little')
            self._motor_currents[motor_idx] = int.from_bytes(data[5:7], 'little')
            self._motor_errors[motor_idx] = data[7]

            feedback[joint_idx] = JointFeedback(
                proximal=MotorFeedback(
                    current=self._motor_currents[motor_idx],
                    velocity=self._motor_velocities[motor_idx],
                    position=self._motor_positions[motor_idx],
                    angle=0.0  # TODO: Implement angle conversion
                ),
                distal=MotorFeedback(
                    current=self._motor_currents[motor_idx + 1],
                    velocity=self._motor_velocities[motor_idx + 1],
                    position=self._motor_positions[motor_idx + 1],
                    angle=0.0  # TODO: Implement angle conversion
                )
            )

        return feedback

    def close(self):
        """Close CAN communication"""
        # Only close ZCAN if we own it
        if self._owns_zcan:
            self.zcan.close()

class LeftDexHand(DexHandBase):
    """Control interface for left dexterous hand"""
    def __init__(self, zcan: Optional[ZCANWrapper] = None):
        super().__init__(
            HandConfig(
                channel=0,
                ctrl_offset=0x100,
                ce_offset=0x00,
                recv_offset=0x20
            ),
            zcan
        )

class RightDexHand(DexHandBase):
    """Control interface for right dexterous hand"""
    def __init__(self, zcan: Optional[ZCANWrapper] = None):
        super().__init__(
            HandConfig(
                channel=1,
                ctrl_offset=0x106,
                ce_offset=0x06,
                recv_offset=0x26
            ),
            zcan
        )
