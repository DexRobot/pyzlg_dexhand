from dataclasses import dataclass, asdict
import struct
from enum import IntEnum
from typing import Optional
import logging
import numpy as np
from . import MessageType, get_message_type

logger = logging.getLogger(__name__)

class BoardError(IntEnum):
    """Motor error status"""
    MOTOR1_ERROR = 0x01
    MOTOR2_ERROR = 0x02
    BOTH_MOTORS_ERROR = 0x03

class ErrorCode(IntEnum):
    """Error codes from protocol"""
    MOTOR1_CURRENT_OVERLOAD = 0x0001
    MOTOR1_HALL_ERROR = 0x0002
    MOTOR1_STALL_ERROR = 0x0004
    MOTOR1_PARAM_ERROR = 0x0008
    MOTOR2_CURRENT_OVERLOAD = 0x0010
    MOTOR2_HALL_ERROR = 0x0020
    MOTOR2_STALL_ERROR = 0x0040
    MOTOR2_PARAM_ERROR = 0x0080

@dataclass(frozen=True)
class MotorFeedback:
    """Feedback data for a single motor"""
    current: int        # Current in mA
    velocity: int       # Speed in rpm
    position: int       # Position in encoder counts
    angle: float        # Angle in degrees

@dataclass(frozen=True)
class TactileFeedback:
    """Feedback from tactile sensor"""
    normal_force: float           # Normal force in N
    normal_force_delta: int       # Change in normal force (raw units)
    tangential_force: float       # Tangential force in N
    tangential_force_delta: int   # Change in tangential force (raw units)
    direction: int                # Force direction (0-359 degrees, fingertip is 0)
    proximity: int               # Proximity value (raw units)
    temperature: int             # Temperature in Celsius

@dataclass(frozen=True)
class BoardFeedback:
    """Complete feedback from a motor control board"""
    motor1: MotorFeedback
    motor2: MotorFeedback
    position_sensor1: float  # First position sensor reading
    position_sensor2: float  # Second position sensor reading
    tactile: Optional[TactileFeedback] = None
    encoder1: int = 0  # Raw encoder 1 value (0-4095)
    encoder2: int = 0  # Raw encoder 2 value (0-4095)

@dataclass(frozen=True)
class ErrorInfo:
    """Error information from a board"""
    error_type: BoardError    # Which motors have errors
    error_code: int         # Raw error code
    description: str        # Human-readable error description

@dataclass(frozen=True)
class ProcessedMessage:
    """Processed message from board"""
    sender_id: int
    msg_type: MessageType
    feedback: Optional[BoardFeedback] = None
    error: Optional[ErrorInfo] = None

def process_message(can_id: int, data: bytes) -> ProcessedMessage:
    """Process a received CAN message

    Args:
        can_id: CAN ID of the message
        data: Raw message bytes

    Returns:
        ProcessedMessage containing decoded information

    Raises:
        ValueError: If message cannot be decoded
    """
    msg_type = get_message_type(can_id)
    if msg_type is None:
        raise ValueError(f"Invalid message ID: {can_id:x}")

    if msg_type == MessageType.MOTION_FEEDBACK:
        feedback = _decode_feedback(data)
        return ProcessedMessage(
            sender_id=can_id,
            msg_type=MessageType.MOTION_FEEDBACK,
            feedback=feedback
        )

    elif msg_type == MessageType.ERROR_MESSAGE:
        try:
            error = _decode_error(data)
            return ProcessedMessage(
                sender_id=can_id,
                msg_type=MessageType.ERROR_MESSAGE,
                error=error
            )
        except ValueError as e:
            logger.warning(f"Failed to decode error message: {e}")
            return ProcessedMessage(
                sender_id=can_id,
                msg_type=MessageType.INVALID
            )

    else:
        # Other message types just pass through
        return ProcessedMessage(
            sender_id=can_id,
            msg_type=msg_type
        )

def _decode_feedback(data: bytes) -> BoardFeedback:
    """Decode motor feedback message (internal helper)"""
    if len(data) < 16:
        raise ValueError("Message too short for basic feedback")

    try:
        # Motor 1 data
        current1 = int.from_bytes(data[0:2], 'little', signed=True)
        velocity1 = int.from_bytes(data[2:4], 'little', signed=True)
        position1 = int.from_bytes(data[4:6], 'little', signed=True)

        # Motor 2 data
        current2 = int.from_bytes(data[6:8], 'little', signed=True)
        velocity2 = int.from_bytes(data[8:10], 'little', signed=True)
        position2 = int.from_bytes(data[10:12], 'little', signed=True)

        # Position sensors (0.01 degree units)
        pos1 = int.from_bytes(data[12:14], 'little', signed=True) / 100.0
        pos2 = int.from_bytes(data[14:16], 'little', signed=True) / 100.0

        # Create motor feedback objects once
        motor1 = MotorFeedback(current1, velocity1, position1, pos1)
        motor2 = MotorFeedback(current2, velocity2, position2, pos2)

        # Optional data
        tactile = None
        encoder1 = 0
        encoder2 = 0

        # Decode tactile if present
        if len(data) >= 42:
            tactile = TactileFeedback(
                normal_force=struct.unpack('<f', data[16:20])[0],
                normal_force_delta=int.from_bytes(data[20:24], 'little'),
                tangential_force=struct.unpack('<f', data[24:28])[0],
                tangential_force_delta=int.from_bytes(data[28:32], 'little'),
                direction=int.from_bytes(data[32:34], 'little'),
                proximity=int.from_bytes(data[34:38], 'little'),
                temperature=int.from_bytes(data[38:42], 'little')
            )
            if np.isnan(tactile.normal_force) or np.isnan(tactile.tangential_force):
                logger.warning("Invalid tactile sensor data: normal/tangential force is NaN")

        # Decode encoders if present
        if len(data) >= 46:
            encoder1 = int.from_bytes(data[42:44], 'little')
            encoder2 = int.from_bytes(data[44:46], 'little')

        # Create BoardFeedback instance once with all data
        return BoardFeedback(
            motor1=motor1,
            motor2=motor2,
            position_sensor1=pos1,
            position_sensor2=pos2,
            tactile=tactile,
            encoder1=encoder1,
            encoder2=encoder2
        )

    except (struct.error, TypeError) as e:
        raise ValueError(f"Failed to decode feedback: {str(e)}")

def _decode_error(data: bytes) -> ErrorInfo:
    """Decode an error message (internal helper)"""
    if len(data) < 3 or data[0] != 0xEE:
        raise ValueError("Invalid error message format")

    try:
        error_type = BoardError(data[1])
        error_code = data[2]
    except ValueError:
        raise ValueError(f"Invalid error type/code: {data[1]}, {data[2]}")

    # Build error description
    descriptions = []
    for code in ErrorCode:
        if error_code & code:
            descriptions.append(code.name.lower().replace('_', ' '))

    description = '; '.join(descriptions) if descriptions else "unknown error"

    return ErrorInfo(
        error_type=error_type,
        error_code=error_code,
        description=description,
    )
