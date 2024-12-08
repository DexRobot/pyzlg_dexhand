from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Union, Tuple
import numpy as np
import os
import yaml
import logging
from pathlib import Path
from typing import List, Dict
import time

from .zcan_wrapper import ZCANWrapper
from . import dexhand_protocol as protocol
from .dexhand_protocol import BoardID
from .dexhand_protocol.commands import ControlMode, MotorCommand, ClearErrorCommand
from .dexhand_protocol.messages import BoardFeedback, ErrorInfo, MessageType


logger = logging.getLogger(__name__)

@dataclass
class HandConfig:
    """Configuration for hand hardware"""
    channel: int             # CAN channel number
    hall_scale: List[float]  # Scale coefficients for hall position modes

@dataclass
class JointFeedback:
    """Feedback for a specific joint command"""
    timestamp: float     # When feedback was received
    angle: float         # Joint angle in degrees
    encoder_position: Optional[int] = None       # Encoder position in raw units
    error: Optional[str] = None  # Error message if any
    error_cleared: Optional[bool] = None  # True if error was cleared

@dataclass
class BoardResponse:
    """Complete response from a board including both feedback and errors"""
    board_idx: int
    feedback: Optional[BoardFeedback] = None
    error: Optional[ErrorInfo] = None
    comm_error: bool = False

@dataclass
class StampedTactileFeedback:
    """Timestamped tactile feedback for a fingertip"""
    timestamp: float           # When feedback was received
    normal_force: float       # Normal force in N
    normal_force_delta: int   # Change in normal force (raw units)
    tangential_force: float   # Tangential force in N
    tangential_force_delta: int  # Change in tangential force (raw units)
    direction: int            # Force direction (0-359 degrees, fingertip is 0)
    proximity: int           # Proximity value (raw units)
    temperature: int         # Temperature in Celsius

@dataclass
class MoveFeedback:
    """Complete feedback from a move_joints command"""
    command_timestamp: float  # When command was sent
    joints: Dict[str, JointFeedback]  # Feedback per joint
    tactile: Dict[str, StampedTactileFeedback]  # Tactile data per fingertip


class DexHandBase:
    """Base class for dexterous hand control"""

    NUM_MOTORS = 12  # Total motors in hand
    NUM_BOARDS = 6   # Number of control boards

    def __init__(self, config_path: str, base_id: int, zcan: Optional[ZCANWrapper] = None):
        """Initialize dexterous hand interface

        Args:
            config_path: Path to hand's YAML config file
            base_id: Base board ID (0x01 for left, 0x07 for right)
            zcan: Optional existing ZCANWrapper instance to share between hands
        """
        self.config = self._load_config(config_path)
        self.base_id = base_id
        self.zcan = zcan if zcan else ZCANWrapper()
        self._owns_zcan = zcan is None

        # Hall position scaling factors
        self._init_hall_scaling()

    def _load_config(self, config_path: str) -> HandConfig:
        """Load hand configuration from YAML file"""
        path = Path(config_path)
        if not path.exists():
            raise FileNotFoundError(f"Config file not found: {config_path}")

        with open(path) as f:
            config = yaml.safe_load(f)

        # Validate config
        required_keys = {'channel', 'hall_scale'}
        missing = required_keys - set(config.keys())
        if missing:
            raise ValueError(f"Missing required keys in config: {missing}")

        if len(config['hall_scale']) != self.NUM_MOTORS:
            raise ValueError(f"Expected {self.NUM_MOTORS} hall scale coefficients")

        return HandConfig(**config)

    def _init_hall_scaling(self):
        """Initialize scaling factors for hall position modes"""
        # Conversion factor for hall position modes (from protocol spec):
        # - 6 counts per revolution
        # - 25:1 gear ratio
        # - 16-bit resolution
        # - 360 degrees per revolution
        factor = 6 * 25 * 2**4 / 360.0  # Converts degrees to hardware units
        self._hall_scale = np.array(self.config.hall_scale) * factor

    def init(self, device_index: int = 0) -> bool:
        """Initialize CAN communication

        Args:
            device_index: Device index for ZCAN device

        Returns:
            bool: True if initialization successful
        """
        if self._owns_zcan:
            if not self.zcan.open(device_index=device_index):
                logger.error("Failed to open CAN device")
                return False

        # Configure channel
        if not self.zcan.configure_channel(self.config.channel):
            logger.error(f"Failed to configure channel {self.config.channel}")
            return False

        return True

    def _get_command_id(self, msg_type: MessageType, board_idx: int) -> int:
        """Get command CAN ID for a board index"""
        if not 0 <= board_idx < self.NUM_BOARDS:
            raise ValueError(f"Invalid board index: {board_idx}")
        return msg_type + self.base_id + board_idx

    def send_command_with_feedback(self,
                                 board_idx: int,
                                 motor1_pos: int,
                                 motor2_pos: int,
                                 motor_enable: int = 0x03,
                                 control_mode: ControlMode = ControlMode.CASCADED_PID) -> BoardResponse:
        """Send command to a board and get all feedback

        Args:
            board_idx: Board index to command
            motor1_pos: Position command for motor 1
            motor2_pos: Position command for motor 2
            motor_enable: Motor enable flags
            control_mode: Control mode

        Returns:
            BoardResponse containing feedback and any errors
        """
        # Prepare response object
        response = BoardResponse(board_idx=board_idx)

        # Create and encode command
        command = MotorCommand(
            control_mode=control_mode,
            motor_enable=motor_enable,
            motor1_pos=motor1_pos,
            motor2_pos=motor2_pos
        )

        try:
            msg_type, data = protocol.commands.encode_command(command)
        except ValueError as e:
            logger.error(f"Failed to encode command: {e}")
            return False, None

        # Send command
        command_id = self._get_command_id(MessageType.MOTION_COMMAND, board_idx)
        if not self.zcan.send_fd_message(self.config.channel, command_id, data):
            response.comm_error = True
            return response

        # Get all feedback messages
        messages = self.zcan.receive_fd_messages(self.config.channel)
        if not messages:
            response.comm_error = True
            return response

        # Process all received messages
        for msg_id, data, timestamp in messages:
            result = protocol.messages.process_message(msg_id, data)

            if result.msg_type == MessageType.MOTION_FEEDBACK:
                response.feedback = result.feedback
            elif result.msg_type == MessageType.ERROR_MESSAGE:
                response.error = result.error

        return response

    def clear_board_error(self, board_idx: int) -> bool:
        """Clear error state for a specific board

        Args:
            board_idx: Board index to clear error for

        Returns:
            bool: True if error cleared successfully
        """
        # Create and encode clear error command
        clear_cmd = ClearErrorCommand()
        msg_type, clear_data = protocol.commands.encode_command(clear_cmd)
        clear_cmd_id = self._get_command_id(msg_type, board_idx)

        # Send command
        if not self.zcan.send_fd_message(self.config.channel, clear_cmd_id, clear_data):
            logger.error(f"Failed to send error clear command to board {board_idx}")
            return False

        # Check acknowledgment
        messages = self.zcan.receive_fd_messages(self.config.channel)
        if not messages:
            logger.error(f"No response to error clear command from board {board_idx}")
            return False

        msg_id, resp_data, _ = messages[-1]
        return protocol.commands.verify_response(clear_cmd, msg_id, resp_data)


    def move_joints(self,
                th_rot: Optional[float] = None,   # thumb rotation
                th_mcp: Optional[float] = None,   # thumb metacarpophalangeal
                th_dip: Optional[float] = None,   # thumb coupled distal joints
                ff_spr: Optional[float] = None,   # four-finger spread
                ff_mcp: Optional[float] = None,   # first finger metacarpophalangeal
                ff_dip: Optional[float] = None,   # first finger coupled distal joints
                mf_mcp: Optional[float] = None,   # middle finger metacarpophalangeal
                mf_dip: Optional[float] = None,   # middle finger coupled distal joints
                rf_mcp: Optional[float] = None,   # ring finger metacarpophalangeal
                rf_dip: Optional[float] = None,   # ring finger coupled distal joints
                lf_mcp: Optional[float] = None,   # little finger metacarpophalangeal
                lf_dip: Optional[float] = None,   # little finger coupled distal joints
                control_mode: ControlMode = ControlMode.CASCADED_PID,
                send_disabled: bool = False) -> MoveFeedback:
        """Move hand joints to specified angles and return feedback.

        For each finger, there are two independent DOFs:
        - MCP (metacarpophalangeal) joint: Controls base joint flexion
        - DIP (coupled): Controls coupled motion of PIP and DIP joints

        Additional DOFs:
        - th_rot: Thumb rotation/opposition
        - ff_spr: Four-finger spread (abduction between fingers)

        Tactile feedback available on fingertips:
        - th (thumb)
        - ff (index/first finger)
        - mf (middle finger)
        - rf (ring finger)
        - lf (little finger)

        Args:
            th_rot: Thumb rotation angle in degrees
            th_mcp: Thumb MCP flexion angle
            th_dip: Thumb coupled PIP-DIP flexion
            ff_spr: Four-finger spread angle
            ff_mcp: Index MCP flexion
            ff_dip: Index coupled PIP-DIP flexion
            mf_mcp: Middle MCP flexion
            mf_dip: Middle coupled PIP-DIP flexion
            rf_mcp: Ring MCP flexion
            rf_dip: Ring coupled PIP-DIP flexion
            lf_mcp: Little MCP flexion
            lf_dip: Little coupled PIP-DIP flexion
            control_mode: Motor control mode
            send_disabled: If True, send commands to all boards regardless of
                        whether their motors are enabled. Useful for collecting
                        feedback without moving the hand.

        Returns:
            MoveFeedback containing:
            - command_timestamp: When the command was sent
            - joints: Dict mapping joint name to JointFeedback with:
                - timestamp: When feedback was sampled
                - angle: Joint angle in degrees
                - encoder_position: Joint position in hardware units
                - error: Error message if any
            - tactile: Dict mapping fingertip name to StampedTactileFeedback with:
                - normal_force: Force perpendicular to surface (N)
                - normal_force_delta: Change in normal force
                - tangential_force: Force parallel to surface (N)
                - tangential_force_delta: Change in tangential force
                - direction: Force direction (degrees)
                - proximity: Proximity sensor value
                - temperature: Temperature in Celsius
        """
        # Record command start time
        command_timestamp = time.time()

        # Map joint angles to motor commands
        joint_names = [
            'th_dip', 'th_mcp',     # Board 0: Thumb
            'th_rot', 'ff_spr',     # Board 1: Thumb rotation & spread
            'ff_dip', 'ff_mcp',     # Board 2: First finger
            'mf_dip', 'mf_mcp',     # Board 3: Middle finger
            'rf_dip', 'rf_mcp',     # Board 4: Ring finger
            'lf_dip', 'lf_mcp'      # Board 5: Little finger
        ]

        motor_angles = [
            th_dip, th_mcp,      # Board 0
            th_rot, ff_spr,      # Board 1
            ff_dip, ff_mcp,      # Board 2
            mf_dip, mf_mcp,      # Board 3
            rf_dip, rf_mcp,      # Board 4
            lf_dip, lf_mcp       # Board 5
        ]

        # Scale angles and create enable mask
        scaled_positions = np.zeros(self.NUM_MOTORS)
        enables = [False] * self.NUM_MOTORS

        for i, (angle, name) in enumerate(zip(motor_angles, joint_names)):
            if angle is not None:
                scaled_positions[i] = self._scale_angle(i, angle, control_mode)
                enables[i] = True

        # Send commands to each board and collect all responses
        joint_feedback = {}
        tactile_feedback = {}
        board_responses: Dict[int, BoardResponse] = {}

        # First pass: send commands and collect responses
        for board_idx in range(self.NUM_BOARDS):
            base_idx = board_idx * 2
            if send_disabled or any(enables[base_idx:base_idx + 2]):
                motor_enable = 0x01 if enables[base_idx] else 0
                motor_enable |= 0x02 if enables[base_idx + 1] else 0

                response = self.send_command_with_feedback(
                    board_idx=board_idx,
                    motor1_pos=int(scaled_positions[base_idx]),
                    motor2_pos=int(scaled_positions[base_idx + 1]),
                    motor_enable=motor_enable,
                    control_mode=control_mode
                )
                board_responses[board_idx] = response

        # Second pass: handle errors and process feedback
        for board_idx, response in board_responses.items():
            base_idx = board_idx * 2
            timestamp = time.time()

            # Handle communication error
            if response.comm_error:
                error_msg = "Communication error"
                for i in range(2):
                    joint_idx = base_idx + i
                    if send_disabled or enables[joint_idx]:
                        joint_feedback[joint_names[joint_idx]] = JointFeedback(
                            timestamp=timestamp,
                            angle=float('nan'),
                            encoder_position=None,
                            error=error_msg,
                            error_cleared=None  # No error to clear
                        )
                continue

            # Handle board error
            if response.error is not None:
                error_cleared = self.clear_board_error(board_idx)
                error_msg = response.error.description

                # Even with error, we might have valid feedback
                if response.feedback is not None:
                    motors = [response.feedback.motor1, response.feedback.motor2]
                    for i in range(2):
                        joint_idx = base_idx + i
                        if send_disabled or enables[joint_idx]:
                            joint_feedback[joint_names[joint_idx]] = JointFeedback(
                                timestamp=timestamp,
                                angle=motors[i].angle,
                                encoder_position=motors[i].position,
                                error=error_msg,
                                error_cleared=error_cleared
                            )
                else:
                    # No feedback available
                    for i in range(2):
                        joint_idx = base_idx + i
                        if send_disabled or enables[joint_idx]:
                            joint_feedback[joint_names[joint_idx]] = JointFeedback(
                                timestamp=timestamp,
                                angle=float('nan'),
                                encoder_position=None,
                                error=error_msg,
                                error_cleared=error_cleared
                            )
                continue

            # Process normal feedback
            if response.feedback is not None:
                motors = [response.feedback.motor1, response.feedback.motor2]
                for i in range(2):
                    joint_idx = base_idx + i
                    if send_disabled or enables[joint_idx]:
                        joint_feedback[joint_names[joint_idx]] = JointFeedback(
                            timestamp=timestamp,
                            angle=motors[i].angle,
                            encoder_position=motors[i].position,
                            error=None,
                            error_cleared=None  # No error to clear
                        )

                # Process tactile feedback if available
                if response.feedback.tactile is not None:
                    fingertip_map = {
                        0: 'thumb',
                        2: 'index',
                        3: 'middle',
                        4: 'ring',
                        5: 'pinky'
                    }
                    if board_idx in fingertip_map:
                        tactile_feedback[fingertip_map[board_idx]] = StampedTactileFeedback(
                            timestamp=timestamp,
                            **asdict(response.feedback.tactile)
                        )

        return MoveFeedback(
            command_timestamp=command_timestamp,
            joints=joint_feedback,
            tactile=tactile_feedback
        )

    def _scale_angle(self, motor_idx: int, angle: float, control_mode: ControlMode) -> int:
        """Scale angle based on control mode"""
        if control_mode in (ControlMode.HALL_POSITION, ControlMode.PROTECT_HALL_POSITION):
            return int(angle * self._hall_scale[motor_idx])
        else:
            # For cascaded PID mode, scale to 100x for hardware units
            return int(angle * 100)

    def reset_joints(self) -> MoveFeedback:
        """Reset all joints to their zero positions.

        This is equivalent to setting all joint angles to 0 degrees.
        Uses CASCADED_PID control mode.

        Returns: MoveFeedback containing the feedback from the reset command.
        """
        return self.move_joints(
            th_rot=0, th_mcp=0, th_dip=0,
            ff_spr=0, ff_mcp=0, ff_dip=0,
            mf_mcp=0, mf_dip=0,
            rf_mcp=0, rf_dip=0,
            lf_mcp=0, lf_dip=0,
            control_mode=ControlMode.CASCADED_PID
        )

    def get_feedback(self) -> MoveFeedback:
        """Get current feedback from all joints and tactile sensors.

        This sends a command with all motors disabled to collect feedback
        without affecting hand motion.

        Returns:
            MoveFeedback containing:
            - command_timestamp: When the feedback request was sent
            - joints: Dict mapping joint name to JointFeedback with:
                - timestamp: When feedback was sampled
                - angle: Joint angle in degrees
                - encoder_position: Joint position in hardware units
                - error: Error message if any
            - tactile: Dict mapping fingertip name to StampedTactileFeedback with:
                - normal_force: Force perpendicular to surface (N)
                - normal_force_delta: Change in normal force
                - tangential_force: Force parallel to surface (N)
                - tangential_force_delta: Change in tangential force
                - direction: Force direction (degrees)
                - proximity: Proximity sensor value
                - temperature: Temperature in Celsius
        """
        return self.move_joints(
            send_disabled=True
        )

    def close(self):
        """Close CAN communication"""
        if self._owns_zcan:
            self.zcan.close()

class LeftDexHand(DexHandBase):
    """Control interface for left dexterous hand"""
    def __init__(self, zcan: Optional[ZCANWrapper] = None):
        config_path = os.path.join(os.path.dirname(__file__), "../config/left_hand.yaml")
        super().__init__(config_path, BoardID.LEFT_HAND_BASE, zcan)

class RightDexHand(DexHandBase):
    """Control interface for right dexterous hand"""
    def __init__(self, zcan: Optional[ZCANWrapper] = None):
        config_path = os.path.join(os.path.dirname(__file__), "../config/right_hand.yaml")
        super().__init__(config_path, BoardID.RIGHT_HAND_BASE, zcan)
