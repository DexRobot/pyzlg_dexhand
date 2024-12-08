"""Host to board command generation and encoding"""

from enum import IntEnum
from dataclasses import dataclass
from typing import Union
from . import MessageType

class CommandType(IntEnum):
    """Command type identifiers"""
    MOTOR_COMMAND = 0x00      # Motor control command
    CLEAR_ERROR = 0xA4        # Clear error state
    CONFIG_FEEDBACK = 0x74    # Configure feedback settings

class ControlMode(IntEnum):
    """Motor control modes"""
    ZERO_TORQUE = 0x00
    CURRENT = 0x11
    SPEED = 0x22
    HALL_POSITION = 0x33
    CASCADED_PID = 0x44
    PROTECT_HALL_POSITION = 0x55

class FeedbackMode(IntEnum):
    """Feedback operation modes"""
    PERIODIC = 0x01   # Send feedback at regular intervals
    QUERY = 0x02      # Send feedback only when requested
    ON_CHANGE = 0x03  # Send feedback when values change

@dataclass(frozen=True)
class MotorCommand:
    """Command for a pair of motors"""
    control_mode: ControlMode  # Control mode (e.g., CASCADED_PID)
    motor_enable: int         # Motor enable flags (0x01, 0x02, 0x03)
    motor1_pos: int          # Position command for motor 1 (-32768 to 32767)
    motor2_pos: int          # Position command for motor 2 (-32768 to 32767)

@dataclass(frozen=True)
class ClearErrorCommand:
    """Command to clear error state"""
    pass  # No additional data needed

@dataclass(frozen=True)
class FeedbackConfigCommand:
    """Command to configure feedback behavior"""
    mode: FeedbackMode     # Feedback mode
    period_ms: int        # Period in milliseconds (if periodic)
    enable: bool          # Enable flag

Command = Union[MotorCommand, ClearErrorCommand, FeedbackConfigCommand]

def encode_command(command: Command) -> tuple[MessageType, bytes]:
    """Encode a command for transmission

    Args:
        command: Command to encode

    Returns:
        Tuple of (message_type, encoded_bytes)

    Raises:
        ValueError: If command parameters are invalid
    """
    if isinstance(command, MotorCommand):
        return (MessageType.MOTION_COMMAND, _encode_motor_command(command))
    elif isinstance(command, ClearErrorCommand):
        return (MessageType.CONFIG_COMMAND, bytes([0x03, CommandType.CLEAR_ERROR]))
    elif isinstance(command, FeedbackConfigCommand):
        return (MessageType.CONFIG_COMMAND, _encode_feedback_config(command))
    else:
        raise ValueError(f"Unknown command type: {type(command)}")


def _encode_motor_command(command: MotorCommand) -> bytes:
    """Encode a motor control command (internal helper)"""
    if not 0 <= command.motor_enable <= 0x03:
        raise ValueError(f"Invalid motor enable flags: {command.motor_enable}")

    if not (-32768 <= command.motor1_pos <= 32767):
        raise ValueError(f"Motor 1 position out of range: {command.motor1_pos}")

    if not (-32768 <= command.motor2_pos <= 32767):
        raise ValueError(f"Motor 2 position out of range: {command.motor2_pos}")

    return bytes([
        command.control_mode & 0xFF,
        command.motor_enable & 0xFF,
        command.motor1_pos & 0xFF,
        (command.motor1_pos >> 8) & 0xFF,
        command.motor2_pos & 0xFF,
        (command.motor2_pos >> 8) & 0xFF
    ])

def _encode_feedback_config(command: FeedbackConfigCommand) -> bytes:
    """Encode a feedback configuration command (internal helper)"""
    if command.mode not in FeedbackMode:
        raise ValueError(f"Invalid feedback mode: {command.mode}")
    if not 0 <= command.period_ms <= 2550:  # Max 255 * 10ms
        raise ValueError(f"Period must be 0-2550ms, got {command.period_ms}")

    period_units = command.period_ms // 10  # Convert to 10ms units
    return bytes([
        0x03,                    # Command prefix
        CommandType.CONFIG_FEEDBACK,
        command.mode,
        period_units,
        0x01 if command.enable else 0x00
    ])
