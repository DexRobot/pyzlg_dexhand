"""DexHand CANFD Protocol Implementation

This package implements the DexHand CANFD communication protocol, with separate
modules for host-to-board commands and board-to-host messages.
"""

from enum import IntEnum
from typing import Optional

class MessageType(IntEnum):
    """CAN message type identifiers and their base addresses"""
    MOTION_COMMAND = 0x100     # Host -> Hand: Motion control
    CONFIG_COMMAND = 0x00      # Host -> Hand: Configuration
    MOTION_FEEDBACK = 0x180    # Hand -> Host: Motion state
    CONFIG_RESPONSE = 0x80     # Hand -> Host: Config response
    ERROR_MESSAGE = 0x600      # Hand -> Host: Errors
    INVALID = 0x1000          # Special: Message decoding failed
    UNKNOWN = 0x1001          # Special: Unknown message type

class BoardID(IntEnum):
    """Base ID values for left and right hand boards"""
    LEFT_HAND_BASE = 0x01
    RIGHT_HAND_BASE = 0x07

def get_message_type(can_id: int) -> Optional[MessageType]:
    """Determine message type from CAN ID"""
    base = can_id & 0xF80  # Keep top 7 bits
    try:
        return MessageType(base)
    except ValueError:
        return None

# Export common types
__all__ = ['MessageType', 'get_message_type', 'commands', 'messages']
