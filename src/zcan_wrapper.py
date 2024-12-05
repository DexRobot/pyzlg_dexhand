import time
from typing import Optional, List, Tuple
import logging
from ctypes import c_uint8, c_uint32, c_void_p, sizeof, memset, byref, Structure
import time
from pathlib import Path
from dataclasses import dataclass
from abc import ABC, abstractmethod
from typing import Tuple, Dict
from .zcan import (
    ZCAN, ZCANDeviceType, ZCANStatus, ZCANMessageType,
    ZCANMessageInfo, ZCANMessageHeader, ZCANMessage, ZCANFDMessage, ZCANCANFDInit, ZCANErrorMessage
)

logger = logging.getLogger(__name__)

@dataclass
class ZCANFilterConfig:
    """Configuration for CAN message filtering"""
    type: int  # 0-std_frame, 1-ext_frame
    start_id: int
    end_id: int

class ZCANFilter(Structure):
    """Low level filter configuration structure"""
    _fields_ = [
        ("type", c_uint8),
        ("pad", c_uint8 * 3),
        ("start_id", c_uint32),
        ("end_id", c_uint32)
    ]

class ZCANFilterTable(Structure):
    """Table of CAN filters"""
    _fields_ = [
        ("size", c_uint32),
        ("table", ZCANFilter * 64)
    ]

@dataclass(frozen=True)
class BitTimingConfig:
    """Bit timing configuration values"""
    tseg1: int
    tseg2: int
    sjw: int
    brp: int

@dataclass(frozen=True)
class CANFDTimingConfig:
    """Complete timing configuration for CANFD"""
    arb: BitTimingConfig  # Arbitration phase timing
    data: BitTimingConfig  # Data phase timing

    @property
    def clock_hz(self) -> int:
        """Clock frequency in Hz"""
        return 60_000_000  # 60MHz fixed clock

class TimingConfigs:
    """Predefined timing configurations for supported baudrate combinations"""

    # Define supported baudrate combinations and their timing parameters
    CONFIGS: Dict[Tuple[int, int], CANFDTimingConfig] = {
        # 1Mbps / 5Mbps
        (1_000_000, 5_000_000): CANFDTimingConfig(
            arb=BitTimingConfig(tseg1=14, tseg2=3, sjw=3, brp=2),
            data=BitTimingConfig(tseg1=1, tseg2=0, sjw=0, brp=2)
        ),
        # Additional combinations can be added here
    }

    @classmethod
    def get_config(cls, arb_baudrate: int, data_baudrate: int) -> CANFDTimingConfig:
        """Get timing configuration for specified baudrate combination

        Args:
            arb_baudrate: Arbitration baudrate in Hz
            data_baudrate: Data baudrate in Hz

        Returns:
            CANFDTimingConfig for the specified baudrates

        Raises:
            ValueError: If baudrate combination is not supported
        """
        key = (arb_baudrate, data_baudrate)
        if key not in cls.CONFIGS:
            supported = ", ".join(f"{arb/1e6:.1f}M/{data/1e6:.1f}M"
                                for arb, data in cls.CONFIGS.keys())
            raise ValueError(
                f"Unsupported baudrate combination: {arb_baudrate/1e6:.1f}M/{data_baudrate/1e6:.1f}M. "
                f"Supported combinations are: {supported}"
            )
        return cls.CONFIGS[key]

class ZCANWrapperBase(ABC):
    """Abstract base class for ZCAN wrapper implementations"""

    @abstractmethod
    def open(self, device_type: Optional[ZCANDeviceType] = None,
             device_index: Optional[int] = None) -> bool:
        """Open ZCAN device

        Args:
            device_type: Type of ZCAN device
            device_index: Device index

        Returns:
            bool: True if device opened successfully
        """
        pass

    @abstractmethod
    def configure_channel(self, channel: int,
                         arb_baudrate: int = 1000000,  # 1Mbps
                         data_baudrate: int = 5000000,  # 5Mbps
                         enable_resistance: bool = True,
                         tx_timeout: int = 200) -> bool:
        """Configure and start a CAN channel

        Args:
            channel: Channel number
            arb_baudrate: Arbitration baudrate in Hz
            data_baudrate: Data baudrate in Hz
            enable_resistance: Enable terminal resistance
            tx_timeout: TX timeout in ms

        Returns:
            bool: True if configuration successful
        """
        pass

    @abstractmethod
    def set_filter(self, channel: int, filters: List[ZCANFilterConfig]) -> bool:
        """Set message filters for a channel

        Args:
            channel: Channel number
            filters: List of filter configurations

        Returns:
            bool: True if filters set successfully
        """
        pass

    @abstractmethod
    def send_fd_message(self, channel: int, id: int, data: bytes,
                       flags: Optional[ZCANMessageInfo] = None) -> bool:
        """Send a CANFD message

        Args:
            channel: Channel number
            id: Message ID
            data: Message data bytes
            flags: Optional message flags

        Returns:
            bool: True if message sent successfully
        """
        pass

    @abstractmethod
    def receive_fd_messages(self, channel: int, max_messages: int = 100,
                          timeout_ms: int = 100) -> List[Tuple[int, bytes, int]]:
        """Receive CANFD messages

        Args:
            channel: Channel number
            max_messages: Maximum number of messages to receive
            timeout_ms: Receive timeout in milliseconds

        Returns:
            List of (id, data, timestamp) tuples for received messages
        """
        pass

    @abstractmethod
    def close(self) -> bool:
        """Close ZCAN device

        Returns:
            bool: True if device closed successfully
        """
        pass

class ZCANWrapper(ZCANWrapperBase):
    """High level wrapper for ZCAN operations"""
    # Command references
    CMD_CAN_FILTER = 0x14
    CMD_CAN_TRES = 0x18  # Terminal resistor
    CMD_CAN_TX_TIMEOUT = 0x44
    CMD_SET_SEND_QUEUE_EN = 0x103

    def __init__(self, lib_path: str = "../lib/libusbcanfd.so"):
        """Initialize ZCAN wrapper

        Args:
            lib_path: Path to ZLG driver library, relative to the current file.
        """
        lib_path_abs = Path(__file__).parent / lib_path
        self.zcan = ZCAN(lib_path_abs)
        self.device_type = ZCANDeviceType.ZCAN_USBCANFD_200U
        self.device_index = 0

    def open(self, device_type: Optional[ZCANDeviceType] = None,
             device_index: Optional[int] = None) -> bool:
        """Open ZCAN device

        Args:
            device_type: Type of ZCAN device, defaults to ZCAN_USBCANFD_200U
            device_index: Device index, defaults to 0

        Returns:
            bool: True if device opened successfully
        """
        if device_type:
            self.device_type = device_type
        if device_index is not None:
            self.device_index = device_index

        return self.zcan.open_device(self.device_type, self.device_index)

    def configure_channel(self, channel: int,
                         arb_baudrate: int = 1000000,  # 1Mbps
                         data_baudrate: int = 5000000,  # 5Mbps
                         enable_resistance: bool = True,
                         tx_timeout: int = 200) -> bool:
        """Configure and start a CAN channel

        Args:
            channel: Channel number to configure
            arb_baudrate: Arbitration baudrate in Hz
            data_baudrate: Data baudrate in Hz
            enable_resistance: Enable terminal resistance
            tx_timeout: Transmission timeout in ms

        Returns:
            bool: True if configuration successful
        """
        # Initialize channel
        init_config = self._create_init_config(arb_baudrate, data_baudrate)
        if not self.zcan.init_can(self.device_type, self.device_index, channel, init_config):
            logger.error(f"Failed to initialize channel {channel}")
            return False

        # Start channel
        if not self.zcan.start_can(self.device_type, self.device_index, channel):
            logger.error(f"Failed to start channel {channel}")
            return False

        # Check channel status after initialization
        if not self.monitor_channel_status(channel):
            logger.error(f"Channel {channel} status check failed after initialization")
            return False

        # Enable terminal resistance if requested
        if enable_resistance:
            resistance = c_uint32(1)
            if not self.zcan.set_reference(self.device_type, self.device_index, channel,
                                         self.CMD_CAN_TRES, byref(resistance)):
                logger.error(f"Failed to enable resistance on channel {channel}")
                return False

        # Set TX timeout
        timeout = c_uint32(tx_timeout)
        if not self.zcan.set_reference(self.device_type, self.device_index, channel,
                                     self.CMD_CAN_TX_TIMEOUT, byref(timeout)):
            logger.error(f"Failed to set TX timeout on channel {channel}")
            return False

        # Enable queue send
        queue_enable = c_uint32(1)
        if not self.zcan.set_reference(self.device_type, self.device_index, channel,
                                     self.CMD_SET_SEND_QUEUE_EN, byref(queue_enable)):
            logger.error(f"Failed to enable send queue on channel {channel}")
            return False

        return True

    def set_filter(self, channel: int, filters: List[ZCANFilterConfig]) -> bool:
        """Configure message filters for a channel

        Args:
            channel: Channel number
            filters: List of filter configurations

        Returns:
            bool: True if filters set successfully
        """
        filter_table = ZCANFilterTable()
        memset(byref(filter_table), 0, sizeof(filter_table))

        filter_table.size = sizeof(ZCANFilter) * len(filters)
        for i, filter_config in enumerate(filters):
            filter_table.table[i].type = filter_config.type
            filter_table.table[i].start_id = filter_config.start_id
            filter_table.table[i].end_id = filter_config.end_id

        result = self.zcan.set_reference(self.device_type, self.device_index, channel,
                                       self.CMD_CAN_FILTER, byref(filter_table))
        return bool(result)


    def send_fd_message(self, channel: int, id: int, data: bytes,
                    flags: Optional[ZCANMessageInfo] = None) -> bool:
        """Send a single CANFD message"""
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(
                f"Preparing CANFD message:\n"
                f"  Channel: {channel}\n"
                f"  ID: {id:x}\n"
                f"  Data ({len(data)} bytes): {[hex(x) for x in data]}"
            )

        message = ZCANFDMessage()
        # Zero out the entire message first
        memset(byref(message), 0, sizeof(message))

        # Set header fields exactly as in working implementation
        message.header.info.fmt = 1    # CANFD format
        message.header.info.brs = 0    # No bit rate switch
        message.header.info.est = 0    # No error state
        message.header.info.err = 0    # No error
        message.header.info.sdf = 0    # Data frame
        message.header.info.sef = 0    # Standard frame
        message.header.info.txm = 0    # Normal transmission

        message.header.ts = 0         # Timestamp
        message.header.pad = 0        # Padding
        message.header.id = id
        message.header.channel = channel
        message.header.len = len(data)

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug("Message flags set:")
            logger.debug(f"  fmt={message.header.info.fmt}, brs={message.header.info.brs}")
            logger.debug(f"  sdf={message.header.info.sdf}, sef={message.header.info.sef}")
            logger.debug(f"  err={message.header.info.err}, est={message.header.info.est}")
            logger.debug(f"  txm={message.header.info.txm}")

        # Copy data
        for i, b in enumerate(data):
            message.data[i] = b

        # Send message
        t = time.time()
        sent = self.zcan.transmit_fd(self.device_type, self.device_index, channel,
                                    [message], 1)
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(f"Transmit time: {time.time() - t:.3f}s")


        if not sent:
            logger.error("TransmitFD failed")
            error_msg = ZCANErrorMessage()
            if self.zcan.read_error_info(self.device_type, self.device_index,
                                    channel, error_msg):
                logger.error(f"Error info: {[hex(x) for x in error_msg.data[:8]]}")

        return sent == 1

    def send_fd_messages(self, channel: int, messages: List[Tuple[int, bytes]],
                      flags: Optional[ZCANMessageInfo] = None) -> int:
        """Send multiple CANFD messages

        Args:
            channel: Channel number
            messages: List of (id, data) tuples
            flags: Optional message flags to use for all messages

        Returns:
            int: Number of messages successfully sent
        """
        fd_messages = []
        for msg_id, data in messages:
            message = ZCANFDMessage()
            message.header.id = msg_id
            message.header.channel = channel
            message.header.len = len(data)

            if flags:
                message.header.info = flags
            else:
                message.header.info.fmt = 1  # CANFD
                message.header.info.brs = 1  # Use data baudrate
                message.header.info.txm = ZCANMessageType.NORMAL

            for i, b in enumerate(data):
                message.data[i] = b

            fd_messages.append(message)

        return self.zcan.transmit_fd(self.device_type, self.device_index, channel,
                                   fd_messages, len(fd_messages))

    def receive_fd_messages(self, channel: int, max_messages: int = 100,
                        timeout_ms: int = 100) -> List[Tuple[int, bytes, int]]:
        """Receive CANFD messages

        Args:
            channel: Channel number
            max_messages: Maximum number of messages to receive
            timeout_ms: Receive timeout in milliseconds

        Returns:
            List of (id, data, timestamp) tuples for received messages
        """
        messages, count = self.zcan.receive_fd(self.device_type, self.device_index,
                                             channel, max_messages, timeout_ms)

        result = []
        for msg in messages[:count]:
            data = bytes(msg.data[:msg.header.len])
            result.append((msg.header.id, data, msg.header.timestamp))

        return result

    def close(self) -> bool:
        """Close ZCAN device

        Returns:
            bool: True if device closed successfully
        """
        return self.zcan.close_device(self.device_type, self.device_index)

    def _create_init_config(self, arb_baudrate: int, data_baudrate: int) -> ZCANCANFDInit:
        """Create initialization config structure for CANFD

        Args:
            arb_baudrate: Arbitration baudrate (Hz)
            data_baudrate: Data baudrate (Hz)

        Returns:
            Initialized ZCANCANFDInit structure

        Raises:
            ValueError: If baudrate combination is not supported
        """
        timing = TimingConfigs.get_config(arb_baudrate, data_baudrate)

        config = ZCANCANFDInit()
        config.clock = timing.clock_hz
        config.mode = 0  # Normal mode

        # Arbitration bit timing
        config.abit.tseg1 = timing.arb.tseg1
        config.abit.tseg2 = timing.arb.tseg2
        config.abit.sjw = timing.arb.sjw
        config.abit.smp = 0  # Sample point setting fixed at 0
        config.abit.brp = timing.arb.brp

        # Data bit timing
        config.dbit.tseg1 = timing.data.tseg1
        config.dbit.tseg2 = timing.data.tseg2
        config.dbit.sjw = timing.data.sjw
        config.dbit.smp = 0  # Sample point setting fixed at 0
        config.dbit.brp = timing.data.brp

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(
                f"Creating CANFD init config for {arb_baudrate/1e6:.1f}M/{data_baudrate/1e6:.1f}M:\n"
                f"  Clock: {timing.clock_hz/1e6:.1f}MHz\n"
                f"  Arbitration: tseg1={timing.arb.tseg1}, tseg2={timing.arb.tseg2}, "
                f"sjw={timing.arb.sjw}, brp={timing.arb.brp}\n"
                f"  Data: tseg1={timing.data.tseg1}, tseg2={timing.data.tseg2}, "
                f"sjw={timing.data.sjw}, brp={timing.data.brp}"
            )

        return config

    def handle_error(self, channel: int) -> None:
        """Read and log error information from the device"""
        error_msg = ZCANErrorMessage()
        if self.zcan.read_error_info(self.device_type, self.device_index, channel, error_msg):
            logger.error("CAN Error Information:")
            logger.error(f"Error code: {hex(error_msg.header.id)}")
            logger.error(f"Error data: {[hex(x) for x in error_msg.data[:8]]}")
            logger.error(f"Channel: {channel}")

            # Add error recovery logic here if needed
            self.reset_channel(channel)
        else:
            logger.error("Failed to read error information")

    def reset_channel(self, channel: int) -> bool:
        """Reset a CAN channel after errors"""
        logger.info(f"Resetting channel {channel}")

        # Stop the channel
        if not self.zcan.reset_can(self.device_type, self.device_index, channel):
            logger.error("Failed to reset channel")
            return False

        # Reconfigure the channel
        return self.configure_channel(channel)

    def monitor_channel_status(self, channel: int) -> bool:
        """Monitor channel status and handle errors"""
        status = self.zcan.read_channel_status(self.device_type, self.device_index,
                                            channel)
        if status is None:
            logger.error("Failed to read channel status")
            return False

        # Log detailed status information
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(
                f"Channel {channel} status:\n"
                f"  Error Interrupt: 0x{status.errInterrupt:02x}\n"
                f"  Register Status: 0x{status.regStatus:02x}\n"
                f"  Register Mode: 0x{status.regMode:02x}\n"
                f"  RX Errors: {status.regRECounter}\n"
                f"  TX Errors: {status.regTECounter}\n"
                f"  Error Warning Limit: {status.regEWLimit}\n"
                f"  AL Capture: 0x{status.regALCapture:02x}\n"
                f"  EC Capture: 0x{status.regECCapture:02x}"
            )

        # Check for errors
        if status.errInterrupt != 0:
            logger.error(f"Channel {channel} error interrupt: 0x{status.errInterrupt:02x}")
            self.handle_error(channel)
            return False

        # Check for bus-off state
        if status.regStatus & 0x80:
            logger.error(f"Channel {channel} is bus-off")
            self.handle_error(channel)
            return False

        # Check for error warning limit
        if status.regRECounter > status.regEWLimit or status.regTECounter > status.regEWLimit:
            logger.warning(
                f"Channel {channel} error counters above warning limit\n"
                f"  RX Errors: {status.regRECounter} (limit: {status.regEWLimit})\n"
                f"  TX Errors: {status.regTECounter} (limit: {status.regEWLimit})"
            )

        return True

    def dump_channel_state(self, channel: int):
        """Dump complete channel state for debugging"""
        logger.info(f"\n=== Channel {channel} State Dump ===")

        # Get status
        status = self.zcan.read_channel_status(self.device_type, self.device_index, channel)
        if status:
            logger.info(
                f"Channel Status:\n"
                f"  Mode: 0x{status.regMode:02x}\n"
                f"  Status: 0x{status.regStatus:02x}\n"
                f"  Error Interrupt: 0x{status.errInterrupt:02x}\n"
                f"  RX Errors: {status.regRECounter}\n"
                f"  TX Errors: {status.regTECounter}\n"
                f"  AL Capture: 0x{status.regALCapture:02x}\n"
                f"  EC Capture: 0x{status.regECCapture:02x}"
            )
        else:
            logger.error("Failed to read channel status")

        logger.info("=== End State Dump ===\n")

    def dump_frame(self, message: ZCANFDMessage) -> None:
        """Dump complete frame details for debugging"""
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(
                f"\n=== CAN Frame Dump ===\n"
                f"Header:\n"
                f"  ID: 0x{message.header.id:x}\n"
                f"  Channel: {message.header.channel}\n"
                f"  Length: {message.header.len}\n"
                f"  Timestamp: {message.header.timestamp}\n"
                f"  Padding: {message.header.pad}\n"
                f"Info flags:\n"
                f"  fmt: {message.header.info.fmt}\n"      # Should be 1 for CANFD
                f"  brs: {message.header.info.brs}\n"      # Should be 0 for no bit rate switch
                f"  sdf: {message.header.info.sdf}\n"      # Should be 0 for data frame
                f"  sef: {message.header.info.sef}\n"      # Should be 0 for standard frame
                f"  err: {message.header.info.err}\n"
                f"  est: {message.header.info.est}\n"
                f"  txm: {message.header.info.txm}\n"      # Should be 0 for normal transmission
                f"Data: {[hex(x) for x in message.data[:message.header.len]]}\n"
                f"=== End Frame Dump ===\n"
            )


class MockZCANWrapper(ZCANWrapperBase):
    """Mock implementation for testing"""

    def __init__(self):
        """Initialize mock wrapper"""
        self.is_open = False
        self.channels = {}  # Track channel states
        self.message_history = []  # Track sent messages for testing

    def open(self, device_type: Optional[ZCANDeviceType] = None,
             device_index: Optional[int] = None) -> bool:
        logger.info("Mock ZCAN: Opening device")
        self.is_open = True
        return True

    def configure_channel(self, channel: int,
                         arb_baudrate: int = 1000000,
                         data_baudrate: int = 5000000,
                         enable_resistance: bool = True,
                         tx_timeout: int = 200) -> bool:
        logger.info(f"Mock ZCAN: Configuring channel {channel}")
        self.channels[channel] = {
            'arb_baudrate': arb_baudrate,
            'data_baudrate': data_baudrate,
            'resistance': enable_resistance,
            'timeout': tx_timeout
        }
        return True

    def set_filter(self, channel: int, filters: List[ZCANFilterConfig]) -> bool:
        logger.info(f"Mock ZCAN: Setting filters for channel {channel}")
        if channel in self.channels:
            self.channels[channel]['filters'] = filters
            return True
        return False

    def send_fd_message(self, channel: int, id: int, data: bytes,
                       flags: Optional[ZCANMessageInfo] = None) -> bool:
        if not self.is_open or channel not in self.channels:
            return False

        logger.info(
            f"Mock ZCAN: Sending message:\n"
            f"  Channel: {channel}\n"
            f"  ID: 0x{id:x}\n"
            f"  Data: {[hex(b) for b in data]}"
        )

        # Store message for testing
        self.message_history.append({
            'channel': channel,
            'id': id,
            'data': data,
            'flags': flags
        })
        return True

    def receive_fd_messages(self, channel: int, max_messages: int = 100,
                          timeout_ms: int = 100) -> List[Tuple[int, bytes, int]]:
        logger.info(f"Mock ZCAN: Receiving messages from channel {channel}")
        # In mock implementation, we could return some test data here
        return []

    def close(self) -> bool:
        logger.info("Mock ZCAN: Closing device")
        self.is_open = False
        self.channels.clear()
        return True

    def get_message_history(self) -> List[dict]:
        """Get history of sent messages for testing

        Returns:
            List of dictionaries containing message details
        """
        return self.message_history

    def clear_message_history(self):
        """Clear message history"""
        self.message_history.clear()
