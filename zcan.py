from ctypes import *
from enum import IntEnum, auto
from typing import Optional, List
import logging

logger = logging.getLogger(__name__)

class ZCANDeviceType(IntEnum):
    ZCAN_USBCANFD_200U = 41
    ZCAN_USBCAN_2E_U = 21
    ZCAN_CANDTU_MINI = 33

class ZCANStatus(IntEnum):
    ERROR = 0
    OK = 1
    ONLINE = 2
    OFFLINE = 3
    UNSUPPORTED = 4

class ZCANMessageType(IntEnum):
    NORMAL = 0
    SINGLE = 1
    SELF_TEST = 2
    SINGLE_SELF_TEST = 3

class ZCANMessageInfo(Structure):
    """CAN message info flags"""
    _fields_ = [
        ("txm", c_uint, 4),  # TX mode: 0-normal, 1-once, 2-self test
        ("fmt", c_uint, 4),  # 0-CAN2.0, 1-CANFD
        ("sdf", c_uint, 1),  # 0-data frame, 1-remote frame
        ("sef", c_uint, 1),  # 0-std frame, 1-ext frame
        ("err", c_uint, 1),  # Error flag
        ("brs", c_uint, 1),  # Bit rate switch
        ("est", c_uint, 1),  # Error state
        ("tx", c_uint, 1),   # Receive valid
        ("echo", c_uint, 1), # Echo frame flag
        ("qsend_100us", c_uint, 1), # Queue send delay unit
        ("qsend", c_uint, 1), # Queue send frame
        ("pad", c_uint, 15)  # Padding
    ]

class ZCANMessageHeader(Structure):
    """CAN message header"""
    _fields_ = [
        ("timestamp", c_uint32),
        ("id", c_uint32),
        ("info", ZCANMessageInfo),
        ("pad", c_uint16),
        ("channel", c_uint8),
        ("len", c_uint8)
    ]

class ZCANMessage(Structure):
    """Standard CAN2.0 message"""
    _fields_ = [
        ("header", ZCANMessageHeader),
        ("data", c_uint8 * 8)
    ]

class ZCANFDMessage(Structure):
    """CANFD message with up to 64 bytes"""
    _fields_ = [
        ("header", ZCANMessageHeader),
        ("data", c_uint8 * 64)
    ]

class ZCANErrorMessageHeader(Structure):
    """Header for CAN error messages"""
    _fields_ = [
        ("timestamp", c_uint32),  # Timestamp
        ("id", c_uint32),         # CAN ID
        ("info", ZCANMessageInfo), # Message flags
        ("pad", c_uint16),        # Padding
        ("channel", c_uint8),     # Channel number
        ("len", c_uint8)          # Data length
    ]

class ZCANErrorMessage(Structure):
    """Error message from CAN device"""
    _fields_ = [
        ("header", ZCANErrorMessageHeader),
        ("data", c_uint8 * 8)  # Error data bytes
    ]

class ZCANErrorMessageHeader(Structure):
    """Header for CAN error messages"""
    _fields_ = [
        ("timestamp", c_uint32),  # Timestamp
        ("id", c_uint32),         # CAN ID
        ("info", ZCANMessageInfo), # Message flags
        ("pad", c_uint16),        # Padding
        ("channel", c_uint8),     # Channel number
        ("len", c_uint8)          # Data length
    ]

class ZCANErrorMessage(Structure):
    """Error message from CAN device"""
    _fields_ = [
        ("header", ZCANErrorMessageHeader),
        ("data", c_uint8 * 8)  # Error data bytes
    ]

class BitConfig(Structure):
    """Bit timing configuration for CAN/CANFD"""
    _fields_ = [
        ("tseg1", c_uint8),  # Time segment 1
        ("tseg2", c_uint8),  # Time segment 2
        ("sjw", c_uint8),    # Synchronization jump width
        ("smp", c_uint8),    # Sample point
        ("brp", c_uint16),   # Baud rate prescaler
    ]

class ZCANCANFDInit(Structure):
    """Configuration for CANFD initialization"""
    _fields_ = [
        ("clock", c_uint32),  # Clock frequency (Hz)
        ("mode", c_uint32),   # Operating mode
        ("abit", BitConfig),  # Arbitration bit timing
        ("dbit", BitConfig)   # Data bit timing
    ]

class ZCAN_STAT(Structure):
    """Controller status information"""
    _fields_ = [
        ("errInterrupt", c_ubyte),  # Error interrupt
        ("regMode", c_ubyte),       # Mode register
        ("regStatus", c_ubyte),     # Status register
        ("regALCapture", c_ubyte),  # Arbitration Lost Capture
        ("regECCapture", c_ubyte),  # Error Code Capture
        ("regEWLimit", c_ubyte),    # Error Warning Limit
        ("regRECounter", c_ubyte),  # RX Error Counter
        ("regTECounter", c_ubyte),  # TX Error Counter
        ("reserved", c_uint32)      # Reserved
    ]

class ZCAN:
    """Low level wrapper around ZLG USBCANFD driver"""

    def __init__(self, lib_path: str = "./libusbcanfd.so"):
        """Initialize ZCAN wrapper

        Args:
            lib_path: Path to ZLG driver library
        """
        self._lib = cdll.LoadLibrary(lib_path)
        self._configure_lib_functions()

    def _configure_lib_functions(self):
        """Configure function signatures for the loaded library"""
        # OpenDevice
        self._lib.VCI_OpenDevice.argtypes = [c_uint32, c_uint32, c_uint32]
        self._lib.VCI_OpenDevice.restype = c_uint32

        # InitCAN
        self._lib.VCI_InitCAN.argtypes = [c_uint32, c_uint32, c_uint32, POINTER(ZCANCANFDInit)]
        self._lib.VCI_InitCAN.restype = c_uint32

        # StartCAN
        self._lib.VCI_StartCAN.argtypes = [c_uint32, c_uint32, c_uint32]
        self._lib.VCI_StartCAN.restype = c_uint32

        # TransmitFD
        self._lib.VCI_TransmitFD.argtypes = [c_uint32, c_uint32, c_uint32, POINTER(ZCANFDMessage), c_uint32]
        self._lib.VCI_TransmitFD.restype = c_uint32

        # ReceiveFD
        self._lib.VCI_ReceiveFD.argtypes = [c_uint32, c_uint32, c_uint32, POINTER(ZCANFDMessage), c_uint32, c_uint32]
        self._lib.VCI_ReceiveFD.restype = c_uint32

        # CloseDevice
        self._lib.VCI_CloseDevice.argtypes = [c_uint32, c_uint32]
        self._lib.VCI_CloseDevice.restype = c_uint32

        # GetReference & SetReference
        self._lib.VCI_GetReference.argtypes = [c_uint32, c_uint32, c_uint32, c_uint32, c_void_p]
        self._lib.VCI_GetReference.restype = c_uint32
        self._lib.VCI_SetReference.argtypes = [c_uint32, c_uint32, c_uint32, c_uint32, c_void_p]
        self._lib.VCI_SetReference.restype = c_uint32

    def open_device(self, device_type: ZCANDeviceType, device_index: int = 0, reserved: int = 0) -> bool:
        """Open USBCAN device

        Args:
            device_type: Type of ZCAN device
            device_index: Device index, usually 0
            reserved: Reserved parameter

        Returns:
            bool: True if device opened successfully
        """
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(f"VCI_OpenDevice: {device_type}, {device_index}, {reserved}")
        result = self._lib.VCI_OpenDevice(device_type, device_index, reserved)
        return bool(result)

    def init_can(self, device_type: int, device_index: int, channel: int,
                init_config: ZCANCANFDInit) -> bool:
        """Initialize CAN channel with given configuration

        Args:
            device_type: Type of ZCAN device
            device_index: Device index
            channel: Channel number to initialize
            init_config: Channel configuration

        Returns:
            bool: True if initialization successful
        """
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(f"VCI_InitCAN: {device_type}, {device_index}, {channel}, {bytes(memoryview(init_config)).hex(' ')}")
        result = self._lib.VCI_InitCAN(device_type, device_index, channel, byref(init_config))
        return bool(result)

    def start_can(self, device_type: int, device_index: int, channel: int) -> bool:
        """Start CAN channel

        Args:
            device_type: Type of ZCAN device
            device_index: Device index
            channel: Channel number to start

        Returns:
            bool: True if start successful
        """
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(f"VCI_StartCAN: {device_type}, {device_index}, {channel}")
        result = self._lib.VCI_StartCAN(device_type, device_index, channel)
        return bool(result)

    def transmit_fd(self, device_type: int, device_index: int, channel: int,
                    messages: List[ZCANFDMessage], count: int) -> int:
        """Transmit CANFD messages with error handling"""
        try:
            message_array = (ZCANFDMessage * count)(*messages)
            result = self._lib.VCI_TransmitFD(device_type, device_index, channel,
                                        message_array, count)
            return result
        except Exception as e:
            logger.error(f"Exception in transmit_fd: {e}")
            return 0

    def receive_fd(self, device_type: int, device_index: int, channel: int,
                  count: int, timeout: int = 100) -> tuple[List[ZCANFDMessage], int]:
        """Receive CANFD messages

        Args:
            device_type: Type of ZCAN device
            device_index: Device index
            channel: Channel number
            count: Maximum number of messages to receive
            timeout: Receive timeout in milliseconds

        Returns:
            Tuple containing:
            - List of received messages
            - Number of messages actually received
        """
        message_array = (ZCANFDMessage * count)()
        received = self._lib.VCI_ReceiveFD(device_type, device_index, channel,
                                         message_array, count, timeout)
        return list(message_array[:received]), received

    def close_device(self, device_type: int, device_index: int) -> bool:
        """Close USBCAN device

        Args:
            device_type: Type of ZCAN device
            device_index: Device index

        Returns:
            bool: True if device closed successfully
        """
        result = self._lib.VCI_CloseDevice(device_type, device_index)
        return bool(result)

    def get_reference(self, device_type: int, device_index: int, channel: int,
                     ref_type: int, data: c_void_p) -> bool:
        """Get device reference parameter

        Args:
            device_type: Type of ZCAN device
            device_index: Device index
            channel: Channel number
            ref_type: Reference parameter type
            data: Pointer to receive data

        Returns:
            bool: True if reference got successfully
        """
        result = self._lib.VCI_GetReference(device_type, device_index, channel,
                                          ref_type, data)
        return bool(result)

    def set_reference(self, device_type: int, device_index: int, channel: int,
                     ref_type: int, data: c_void_p) -> bool:
        """Set device reference parameter

        Args:
            device_type: Type of ZCAN device
            device_index: Device index
            channel: Channel number
            ref_type: Reference parameter type
            data: Pointer to data to set

        Returns:
            bool: True if reference set successfully
        """
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(f"VCI_SetReference: {device_type}, {device_index}, {channel}, {ref_type}")
        result = self._lib.VCI_SetReference(device_type, device_index, channel,
                                          ref_type, data)
        return bool(result)

    def read_error_info(self, device_type: int, device_index: int,
                       channel: int, error_msg: ZCANErrorMessage) -> bool:
        """Read error information from device

        Args:
            device_type: Type of ZCAN device
            device_index: Device index
            channel: Channel number
            error_msg: Structure to receive error info

        Returns:
            bool: True if error info read successfully
        """
        # Configure function if not already done
        if not hasattr(self._lib.VCI_ReadErrInfo, 'argtypes'):
            self._lib.VCI_ReadErrInfo.argtypes = [
                c_uint32, c_uint32, c_uint32, POINTER(ZCANErrorMessage)
            ]
            self._lib.VCI_ReadErrInfo.restype = c_uint32

        result = self._lib.VCI_ReadErrInfo(
            device_type, device_index, channel, byref(error_msg)
        )
        return bool(result)

    def read_channel_status(self, device_type: int, device_index: int,
                          channel: int) -> Optional[ZCAN_STAT]:
        """Read channel status information

        Returns:
            Optional[ZCAN_STAT]: Status structure if successful, None on failure
        """
        status = ZCAN_STAT()
        ret = self._lib.VCI_ReadCANStatus(device_type, device_index,
                                        channel, byref(status))
        return status if ret == ZCANStatus.OK else None
