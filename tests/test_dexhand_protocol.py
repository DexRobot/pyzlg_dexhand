"""Tests for the DexHand CANFD protocol implementation"""

import pytest
from dataclasses import asdict
import struct
from pyzlg_dexhand.dexhand_protocol import MessageType, get_message_type
from pyzlg_dexhand.dexhand_protocol import commands, messages

class TestMessageTypes:
    """Test message type identification and CAN ID handling"""

    @pytest.mark.parametrize("can_id,expected_type", [
        (0x101, MessageType.MOTION_COMMAND),    # Board 1 motion command
        (0x001, MessageType.CONFIG_COMMAND),    # Board 1 config command
        (0x181, MessageType.MOTION_FEEDBACK),   # Board 1 feedback
        (0x081, MessageType.CONFIG_RESPONSE),   # Board 1 config response
        (0x601, MessageType.ERROR_MESSAGE),     # Board 1 error
        # Test byte boundaries
        (0x0FF, MessageType.CONFIG_RESPONSE),    # Last config ID
        (0x100, MessageType.MOTION_COMMAND),    # First motion command ID
        (0x17F, MessageType.MOTION_COMMAND),    # Last motion command ID
        (0x180, MessageType.MOTION_FEEDBACK),   # First feedback ID
    ])
    def test_message_type_identification(self, can_id: int, expected_type: MessageType):
        """Test correct identification of message types from CAN IDs"""
        assert get_message_type(can_id) == expected_type

class TestCommandEncoding:
    """Test command encoding according to protocol specification"""

    def test_motor_command_encoding(self):
        """Test motor command encoding matches protocol specification"""
        # Create a motor command with known values
        cmd = commands.MotorCommand(
            control_mode=commands.ControlMode.CASCADED_PID,  # 0x44
            motor_enable=0x03,  # Both motors enabled
            motor1_pos=1000,    # Test positive value
            motor2_pos=-2000    # Test negative value
        )
        msg_type, data = commands.encode_command(cmd)

        # Verify message type
        assert msg_type == MessageType.MOTION_COMMAND

        # Verify each byte according to protocol
        assert data[0] == 0x44  # Control mode
        assert data[1] == 0x03  # Enable flags
        # Motor 1 position (little endian)
        assert data[2] == (1000 & 0xFF)
        assert data[3] == ((1000 >> 8) & 0xFF)
        # Motor 2 position (little endian, signed)
        assert data[4] == (-2000 & 0xFF)
        assert data[5] == ((-2000 >> 8) & 0xFF)

    def test_clear_error_encoding(self):
        """Test clear error command encoding matches protocol specification"""
        cmd = commands.ClearErrorCommand()
        msg_type, data = commands.encode_command(cmd)

        assert msg_type == MessageType.CONFIG_COMMAND
        # Verify exact bytes from protocol doc
        assert data == bytes([0x03, 0xA4])

    def test_feedback_config_encoding(self):
        """Test feedback configuration encoding matches protocol specification"""
        # Test periodic feedback config
        cmd = commands.FeedbackConfigCommand(
            mode=commands.FeedbackMode.PERIODIC,
            period_ms=500,  # Should be converted to 50 (10ms units)
            enable=True
        )
        msg_type, data = commands.encode_command(cmd)

        assert msg_type == MessageType.CONFIG_COMMAND
        assert len(data) == 5
        assert data[0] == 0x03  # Command prefix
        assert data[1] == 0x74  # Feedback config command
        assert data[2] == 0x01  # Periodic mode
        assert data[3] == 50    # 500ms / 10ms
        assert data[4] == 0x01  # Enable

    def test_motor_command_validation(self):
        """Test motor command parameter validation"""
        # Test position range limits
        with pytest.raises(ValueError, match="position out of range"):
            cmd = commands.MotorCommand(
                control_mode=commands.ControlMode.CASCADED_PID,
                motor_enable=0x03,
                motor1_pos=32768,  # Just over limit
                motor2_pos=0
            )
            commands.encode_command(cmd)

        with pytest.raises(ValueError, match="position out of range"):
            cmd = commands.MotorCommand(
                control_mode=commands.ControlMode.CASCADED_PID,
                motor_enable=0x03,
                motor1_pos=0,
                motor2_pos=-32769  # Just under limit
            )
            commands.encode_command(cmd)

        # Test enable flags
        with pytest.raises(ValueError, match="enable flags"):
            cmd = commands.MotorCommand(
                control_mode=commands.ControlMode.CASCADED_PID,
                motor_enable=0x04,  # Invalid flag
                motor1_pos=0,
                motor2_pos=0
            )
            commands.encode_command(cmd)

class TestMessageDecoding:
    """Test message decoding according to protocol specification"""

    def test_feedback_decoding(self):
        """Test feedback mefrom .dexhand_protocol import (
    BoardFeedback, MotorFeedback, ErrorInfo, MessageType,
    CommandType, MotorCommand, FeedbackConfig, ProtocolCommand,
    ControlMode, BoardID
)ssage decoding matches protocol specification"""
        # Construct feedback data according to protocol
        data = bytearray(46)  # Full message with encoders

        # Motor 1 data (bytes 0-5)
        struct.pack_into('<h', data, 0, 100)     # Current (mA)
        struct.pack_into('<h', data, 2, 200)     # Velocity (rpm)
        struct.pack_into('<h', data, 4, 1000)    # Position (counts)

        # Motor 2 data (bytes 6-11)
        struct.pack_into('<h', data, 6, -150)    # Current (mA)
        struct.pack_into('<h', data, 8, -250)    # Velocity (rpm)
        struct.pack_into('<h', data, 10, -2000)  # Position (counts)

        # Position sensors (bytes 12-15)
        struct.pack_into('<h', data, 12, 4500)   # 45.00 degrees
        struct.pack_into('<h', data, 14, -9000)  # -90.00 degrees

        # Tactile data (bytes 16-41)
        struct.pack_into('<f', data, 16, 1.5)    # Normal force (N)
        struct.pack_into('<I', data, 20, 100)    # Force delta
        struct.pack_into('<f', data, 24, 0.5)    # Tangential force (N)
        struct.pack_into('<I', data, 28, 50)     # Tangential delta
        struct.pack_into('<H', data, 32, 180)    # Direction
        struct.pack_into('<I', data, 34, 500)    # Proximity
        struct.pack_into('<I', data, 38, 25)     # Temperature (Celsius)

        # Encoder values (bytes 42-45)
        struct.pack_into('<H', data, 42, 2048)   # Encoder 1
        struct.pack_into('<H', data, 44, 1024)   # Encoder 2

        # Decode and verify values
        msg = messages.process_message(0x181, bytes(data))
        assert msg.msg_type == MessageType.MOTION_FEEDBACK
        assert msg.feedback is not None

        # Verify motor 1 feedback
        assert msg.feedback.motor1.current == 100
        assert msg.feedback.motor1.velocity == 200
        assert msg.feedback.motor1.position == 1000
        assert msg.feedback.motor1.angle == 45.0

        # Verify motor 2 feedback
        assert msg.feedback.motor2.current == -150
        assert msg.feedback.motor2.velocity == -250
        assert msg.feedback.motor2.position == -2000
        assert msg.feedback.motor2.angle == -90.0

        # Verify tactile feedback
        assert msg.feedback.tactile is not None
        assert msg.feedback.tactile.normal_force == pytest.approx(1.5)
        assert msg.feedback.tactile.normal_force_delta == 100
        assert msg.feedback.tactile.tangential_force == pytest.approx(0.5)
        assert msg.feedback.tactile.tangential_force_delta == 50
        assert msg.feedback.tactile.direction == 180
        assert msg.feedback.tactile.proximity == 500
        assert msg.feedback.tactile.temperature == 25

        # Verify encoder values
        assert msg.feedback.encoder1 == 2048
        assert msg.feedback.encoder2 == 1024

    def test_error_decoding(self):
        """Test error message decoding matches protocol specification"""
        # Test motor 1 current overload error
        data = bytes([
            0xEE,  # Error message marker
            messages.BoardError.MOTOR1_ERROR,
            messages.ErrorCode.MOTOR1_CURRENT_OVERLOAD
        ])
        msg = messages.process_message(0x601, data)
        assert msg.msg_type == MessageType.ERROR_MESSAGE
        assert msg.error is not None
        assert msg.error.error_type == messages.BoardError.MOTOR1_ERROR
        assert msg.error.error_code == messages.ErrorCode.MOTOR1_CURRENT_OVERLOAD
        assert "current overload" in msg.error.description.lower()

        # Test multiple errors
        data = bytes([
            0xEE,  # Error message marker
            messages.BoardError.BOTH_MOTORS_ERROR,
            (messages.ErrorCode.MOTOR1_HALL_ERROR |
             messages.ErrorCode.MOTOR2_HALL_ERROR)
        ])
        msg = messages.process_message(0x601, data)
        assert msg.error is not None
        assert msg.error.error_type == messages.BoardError.BOTH_MOTORS_ERROR
        assert "hall" in msg.error.description.lower()

    def test_feedback_validation(self, caplog):
        """Test feedback message validation"""
        # Test minimum length requirement
        with pytest.raises(ValueError, match="Message too short"):
            messages.process_message(0x181, bytes([0] * 15))

    def test_error_validation(self):
        """Test error message validation"""
        # Test invalid error marker
        processed_message = messages.process_message(0x601, bytes([0xFF, 0x01, 0x01]))
        assert processed_message.msg_type == MessageType.INVALID

        # Test invalid error type
        processed_message = messages.process_message(0x601, bytes([0xEE, 0xFF, 0x01]))
        assert processed_message.msg_type == MessageType.INVALID

    def test_process_error_message(self):
        """Test processing of a real error message captured from hardware"""
        # Real message data captured from hardware
        can_id = 0x609
        data = b'\xee\x01\x04\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'

        result = messages.process_message(can_id, data)

        # Verify message type
        assert result.msg_type == MessageType.ERROR_MESSAGE
        assert result.sender_id == can_id

        # Verify error information
        assert result.error is not None
        assert result.error.error_type == messages.BoardError.MOTOR1_ERROR  # 0x01
        assert result.error.error_code == 0x04  # Motor 1 stall error
        assert "stall error" in result.error.description.lower()

        # Verify the error is for the correct board (from CAN ID)
        board_id = can_id & 0x0F
        assert board_id == 0x09


class TestResponseVerification:
    """Test command response verification"""

    def test_clear_error_response(self):
        """Test clear error command response verification"""
        cmd = commands.ClearErrorCommand()

        # Valid response
        assert commands.verify_response(cmd, 0x081, bytes([0x03, 0xA4, 0x00, 0x01]))

        # Invalid responses
        assert not commands.verify_response(cmd, 0x081, bytes([0x03, 0xA4, 0x00, 0x00]))  # Failed
        assert not commands.verify_response(cmd, 0x081, bytes([0x03, 0xA4]))  # Too short
        assert not commands.verify_response(cmd, 0x081, bytes([0x03, 0x74, 0x00, 0x01]))  # Wrong command

    def test_feedback_config_response(self):
        """Test feedback configuration response verification"""
        cmd = commands.FeedbackConfigCommand(
            mode=commands.FeedbackMode.PERIODIC,
            period_ms=100,
            enable=True
        )

        # Valid response
        assert commands.verify_response(cmd, 0x081, bytes([0x03, 0x74, 0x00, 0x01]))

        # Invalid responses
        assert not commands.verify_response(cmd, 0x081, bytes([0x03, 0x74, 0x00, 0x00]))  # Failed
        assert not commands.verify_response(cmd, 0x081, bytes([0x03, 0x74]))  # Too short
        assert not commands.verify_response(cmd, 0x081, bytes([0x03, 0xA4, 0x00, 0x01]))  # Wrong command

    def test_motor_command_response(self):
        """Test that motor commands don't expect responses"""
        cmd = commands.MotorCommand(
            control_mode=commands.ControlMode.CASCADED_PID,
            motor_enable=0x03,
            motor1_pos=0,
            motor2_pos=0
        )
        # Motor commands should never verify responses
        assert not commands.verify_response(cmd, 0x181, bytes([0x00] * 4))

if __name__ == '__main__':
    pytest.main([__file__])
