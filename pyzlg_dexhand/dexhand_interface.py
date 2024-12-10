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
from .dexhand_protocol.commands import (
    ControlMode,
    MotorCommand,
    ClearErrorCommand,
    FeedbackConfigCommand,
    FeedbackMode,
)
from .dexhand_protocol.messages import (
    BoardFeedback,
    ErrorInfo,
    MessageType,
    ProcessedMessage,
)


logger = logging.getLogger(__name__)


@dataclass
class HandConfig:
    """Configuration for hand hardware"""

    channel: int  # CAN channel number
    hall_scale: List[float]  # Scale coefficients for hall position modes


@dataclass
class BoardState:
    """Feedback and Error collected for a single board."""

    feedback_timestamp: float  # Timestamp of last feedback
    status_timestamp: float  # Timestamp of last status (normal / error) update
    is_normal: bool  # True if board is in normal state
    feedback: Optional[BoardFeedback] = None  # Last feedback received
    error_info: Optional[ErrorInfo] = (
        None  # Error information if board is in error state
    )


@dataclass
class JointFeedback:
    """Feedback for a specific joint command"""

    timestamp: float  # When feedback was received
    angle: float  # Joint angle in degrees
    encoder_position: Optional[int] = None  # Encoder position in raw units


@dataclass
class StampedTactileFeedback:
    """Timestamped tactile feedback for a fingertip"""

    timestamp: float  # When feedback was received
    normal_force: float  # Normal force in N
    normal_force_delta: int  # Change in normal force (raw units)
    tangential_force: float  # Tangential force in N
    tangential_force_delta: int  # Change in tangential force (raw units)
    direction: int  # Force direction (0-359 degrees, fingertip is 0)
    proximity: int  # Proximity value (raw units)
    temperature: int  # Temperature in Celsius


@dataclass
class HandFeedback:
    """Feedback data for whole hand"""

    query_timestamp: float  # When feedback was requested
    joints: Dict[str, JointFeedback]  # Feedback per joint
    tactile: Dict[str, StampedTactileFeedback]  # Tactile data per fingertip


class DexHandBase:
    """Base class for dexterous hand control"""

    NUM_MOTORS = 12  # Total motors in hand
    NUM_BOARDS = 6  # Number of control boards

    joint_names = [
        "th_dip",
        "th_mcp",  # Board 0: Thumb
        "th_rot",
        "ff_spr",  # Board 1: Thumb rotation & spread
        "ff_dip",
        "ff_mcp",  # Board 2: First finger
        "mf_dip",
        "mf_mcp",  # Board 3: Middle finger
        "rf_dip",
        "rf_mcp",  # Board 4: Ring finger
        "lf_dip",
        "lf_mcp",  # Board 5: Little finger
    ]
    finger_map = {
        0: "th",
        2: "ff",
        3: "mf",
        4: "rf",
        5: "lf",
    }  # Map from board index to finger name

    def __init__(self, config: dict, base_id: int, zcan: Optional[ZCANWrapper] = None):
        """Initialize dexterous hand interface

        Args:
            config_path: Path to hand's YAML config file
            base_id: Base board ID (0x01 for left, 0x07 for right)
            zcan: Optional existing ZCANWrapper instance to share between hands
        """
        print(config)
        self.config = HandConfig(
            channel=config["channel"], hall_scale=config["hall_scale"]
        )
        self.base_id = base_id
        self.zcan = zcan if zcan else ZCANWrapper()
        self._owns_zcan = zcan is None

        # Hall position scaling factors
        self._init_hall_scaling()

        # Maintain state for each board
        self.board_states: Dict[int, BoardState] = {
            i: BoardState(
                feedback_timestamp=0,
                feedback=None,
                status_timestamp=0,
                is_normal=True,
                error_info=None,
            )
            for i in range(self.NUM_BOARDS)
        }

    # def _load_config(self, config_path: str) -> HandConfig:
    #     """Load hand configuration from YAML file"""
    #     path = Path(config_path)
    #     if not path.exists():
    #         raise FileNotFoundError(f"Config file not found: {config_path}")

    #     with open(path) as f:
    #         config = yaml.safe_load(f)

    #     # Validate config
    #     required_keys = {"channel", "hall_scale"}
    #     missing = required_keys - set(config.keys())
    #     if missing:
    #         raise ValueError(f"Missing required keys in config: {missing}")

    #     if len(config["hall_scale"]) != self.NUM_MOTORS:
    #         raise ValueError(f"Expected {self.NUM_MOTORS} hall scale coefficients")

    #     return HandConfig(**config)

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

    def set_feedback_mode(
        self, mode: FeedbackMode, period_ms: int, enable: bool
    ) -> bool:
        """Configure feedback mode for all boards

        Args:
            mode: Feedback mode
            period_ms: Period in milliseconds (if periodic)
            enable: Enable flag

        Returns:
            bool: True if command sent successfully
        """
        # Create and encode command
        command = FeedbackConfigCommand(mode=mode, period_ms=period_ms, enable=enable)

        try:
            msg_type, data = protocol.commands.encode_command(command)
        except ValueError as e:
            logger.error(f"Failed to encode feedback config command: {e}")
            return False

        # Send command to all boards
        for board_idx in range(self.NUM_BOARDS):
            command_id = self._get_command_id(msg_type, board_idx)
            if not self.zcan.send_fd_message(self.config.channel, command_id, data):
                logger.error(
                    f"Failed to send feedback config command to board {board_idx}"
                )
                return False

        return True

    def _send_motion_command(
        self,
        board_idx: int,
        motor1_pos: int,
        motor2_pos: int,
        motor_enable: int = 0x03,
        control_mode: ControlMode = ControlMode.CASCADED_PID,
    ) -> bool:
        """Send a motion command to a specific board

        Args:
            board_idx: Board index to command
            motor1_pos: Position command for motor 1, in hardware units
            motor2_pos: Position command for motor 2, in hardware units
            motor_enable: Motor enable flags, 0x01 for motor 1, 0x02 for motor 2, 0x03 for both
            control_mode: Control mode

        Returns:
            bool: True if command sent successfully
        """
        # Create and encode command
        command = MotorCommand(
            control_mode=control_mode,
            motor_enable=motor_enable,
            motor1_pos=motor1_pos,
            motor2_pos=motor2_pos,
        )

        try:
            msg_type, data = protocol.commands.encode_command(command)
        except ValueError as e:
            logger.error(f"Failed to encode command: {e}")
            return False

        # Send command
        command_id = self._get_command_id(MessageType.MOTION_COMMAND, board_idx)
        if not self.zcan.send_fd_message(self.config.channel, command_id, data):
            logger.error("Failed to send command to ID {command_id}")
            return False

        return True

    def _refresh_board_states(self):
        """Receive CANFD frames to update the states for all boards."""
        # Get all messages
        messages = self.zcan.receive_fd_messages(self.config.channel)

        # Process all received messages
        for msg_id, data, timestamp in messages:
            try:
                result = protocol.messages.process_message(msg_id, data)
                board_idx = msg_id - result.msg_type - self.base_id

                if result.msg_type == MessageType.MOTION_FEEDBACK:
                    self.board_states[board_idx].feedback_timestamp = timestamp
                    self.board_states[board_idx].feedback = result.feedback
                elif result.msg_type == MessageType.ERROR_MESSAGE:
                    self.board_states[board_idx].status_timestamp = timestamp
                    self.board_states[board_idx].is_normal = False
                    self.board_states[board_idx].error_info = result.error
                elif result.msg_type == MessageType.CONFIG_RESPONSE:
                    success, command_type = protocol.messages.verify_config_response(
                        msg_id, data
                    )
                    if (
                        success
                        and command_type == protocol.commands.CommandType.CLEAR_ERROR
                    ):
                        self.board_states[board_idx].error_info = None
                        self.board_states[board_idx].is_normal = True
            except ValueError as e:
                logger.error(f"Failed to process message: {e}")

    def _clear_board_error(self, board_idx: int) -> bool:
        """Attempt to clear error state for a board

        Args:
            board_idx: Board index to clear error for

        Returns:
            bool: True if error clearance command sent successfully
        """
        # Create and encode clear error command
        clear_cmd = ClearErrorCommand()
        msg_type, clear_data = protocol.commands.encode_command(clear_cmd)
        clear_cmd_id = self._get_command_id(msg_type, board_idx)

        # Send command
        if not self.zcan.send_fd_message(self.config.channel, clear_cmd_id, clear_data):
            logger.error(f"Failed to send error clear command to board {board_idx}")
            return False

        return True

    def move_joints(
        self,
        th_rot: Optional[float] = None,  # thumb rotation
        th_mcp: Optional[float] = None,  # thumb metacarpophalangeal
        th_dip: Optional[float] = None,  # thumb coupled distal joints
        ff_spr: Optional[float] = None,  # four-finger spread
        ff_mcp: Optional[float] = None,  # first finger metacarpophalangeal
        ff_dip: Optional[float] = None,  # first finger coupled distal joints
        mf_mcp: Optional[float] = None,  # middle finger metacarpophalangeal
        mf_dip: Optional[float] = None,  # middle finger coupled distal joints
        rf_mcp: Optional[float] = None,  # ring finger metacarpophalangeal
        rf_dip: Optional[float] = None,  # ring finger coupled distal joints
        lf_mcp: Optional[float] = None,  # little finger metacarpophalangeal
        lf_dip: Optional[float] = None,  # little finger coupled distal joints
        control_mode: ControlMode = ControlMode.CASCADED_PID,
    ):
        """Move hand joints to specified angles.

        For each finger, there are two independent DOFs:
        - MCP (metacarpophalangeal) joint: Controls base joint flexion
        - DIP (coupled): Controls coupled motion of PIP and DIP joints

        Additional DOFs:
        - th_rot: Thumb rotation/opposition
        - ff_spr: Four-finger spread (abduction between fingers)

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
        """
        # Record command start time
        command_timestamp = time.time()

        # Map joint angles to motor commands
        motor_angles = [
            th_dip,
            th_mcp,  # Board 0
            th_rot,
            ff_spr,  # Board 1
            ff_dip,
            ff_mcp,  # Board 2
            mf_dip,
            mf_mcp,  # Board 3
            rf_dip,
            rf_mcp,  # Board 4
            lf_dip,
            lf_mcp,  # Board 5
        ]

        # Scale angles and create enable mask
        scaled_positions = np.zeros(self.NUM_MOTORS)
        enables = [False] * self.NUM_MOTORS

        for i, (angle, name) in enumerate(zip(motor_angles, self.joint_names)):
            if angle is not None:
                scaled_positions[i] = self._scale_angle(i, angle, control_mode)
                enables[i] = True

        for board_idx in range(self.NUM_BOARDS):
            base_idx = board_idx * 2
            if any(enables[base_idx : base_idx + 2]):
                motor_enable = 0x01 if enables[base_idx] else 0
                motor_enable |= 0x02 if enables[base_idx + 1] else 0
                success = self._send_motion_command(
                    board_idx=board_idx,
                    motor1_pos=int(scaled_positions[base_idx]),
                    motor2_pos=int(scaled_positions[base_idx + 1]),
                    motor_enable=motor_enable,
                    control_mode=control_mode,
                )
                if not success:
                    logger.error(f"Failed to send command to board {board_idx}")

    def get_feedback(self) -> HandFeedback:
        """Get feedback from all joints and tactile sensors

        Returns:
            HandFeedback object.
        """
        # Record query start time
        query_timestamp = time.time()

        # Refresh board states to get feedback
        self._refresh_board_states()

        # Process feedback from all boards
        joint_feedback = {}
        tactile_feedback = {}
        for board_idx, state in self.board_states.items():
            base_idx = board_idx * 2
            timestamp = time.time()

            if state.feedback is None:
                # No feedback available
                for i in range(2):
                    joint_idx = base_idx + i
                    joint_feedback[self.joint_names[joint_idx]] = JointFeedback(
                        timestamp=timestamp,
                        angle=float("nan"),
                        encoder_position=None,
                    )
                continue

            # Process joint feedback
            motors = [state.feedback.motor1, state.feedback.motor2]
            for i in range(2):
                joint_idx = base_idx + i
                joint_feedback[self.joint_names[joint_idx]] = JointFeedback(
                    timestamp=timestamp,
                    angle=motors[i].angle,
                    encoder_position=motors[i].position,
                )

            # Process tactile feedback if available
            if state.feedback.tactile is not None:
                if board_idx in self.finger_map:
                    tactile_feedback[self.finger_map[board_idx]] = (
                        StampedTactileFeedback(
                            timestamp=timestamp, **asdict(state.feedback.tactile)
                        )
                    )

        return HandFeedback(
            query_timestamp=query_timestamp,
            joints=joint_feedback,
            tactile=tactile_feedback,
        )

    def get_errors(self) -> Dict[int, Optional[ErrorInfo]]:
        """Get error information for whole hand

        Returns:
            Dict mapping board index to ErrorInfo if an error is present
        """
        return {i: state.error_info for i, state in self.board_states.items()}

    def clear_errors(self, clear_all=True):
        """Clear all errors for the hand

        Args:
            clear_all: If True, attempt to clear errors for all boards even if not in error state
        """
        for board_idx in range(self.NUM_BOARDS):
            if clear_all or not self.board_states[board_idx].is_normal:
                self._clear_board_error(board_idx)

    def _scale_angle(
        self, motor_idx: int, angle: float, control_mode: ControlMode
    ) -> int:
        """Scale angle based on control mode"""
        if control_mode in (
            ControlMode.HALL_POSITION,
            ControlMode.PROTECT_HALL_POSITION,
        ):
            return int(angle * self._hall_scale[motor_idx])
        else:
            # For cascaded PID mode, scale to 100x for hardware units
            return int(angle * 100)

    def reset_joints(self):
        """Reset all joints to their zero positions.

        This is equivalent to setting all joint angles to 0 degrees.
        Uses CASCADED_PID control mode.
        """
        self.move_joints(
            th_rot=0,
            th_mcp=0,
            th_dip=0,
            ff_spr=0,
            ff_mcp=0,
            ff_dip=0,
            mf_mcp=0,
            mf_dip=0,
            rf_mcp=0,
            rf_dip=0,
            lf_mcp=0,
            lf_dip=0,
            control_mode=ControlMode.CASCADED_PID,
        )

    def close(self):
        """Close CAN communication"""
        if self._owns_zcan:
            self.zcan.close()


class LeftDexHand(DexHandBase):
    """Control interface for left dexterous hand"""

    def __init__(self, zcan: Optional[ZCANWrapper] = None):
        config_path = os.path.join(
            os.path.dirname(__file__), "../config", "config.yaml"
        )
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
        super().__init__(
            config["DexHand"]["left_hand"],
            BoardID.LEFT_HAND_BASE,
            zcan,
        )


class RightDexHand(DexHandBase):
    """Control interface for right dexterous hand"""

    def __init__(self, zcan: Optional[ZCANWrapper] = None):
        config_path = os.path.join(
            os.path.dirname(__file__), "../config", "config.yaml"
        )
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
        super().__init__(
            config["DexHand"]["right_hand"],
            BoardID.RIGHT_HAND_BASE,
            zcan,
        )
