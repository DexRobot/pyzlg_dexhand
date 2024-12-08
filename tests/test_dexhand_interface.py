import pytest
import numpy as np
from unittest.mock import Mock, patch, call
from dataclasses import asdict
import yaml
from pathlib import Path

from pyzlg_dexhand.dexhand_interface import (
    DexHandBase, LeftDexHand, RightDexHand, HandConfig,
    JointFeedback, MoveFeedback, StampedTactileFeedback,
)
from pyzlg_dexhand.dexhand_protocol import BoardID, MessageType
from pyzlg_dexhand.dexhand_protocol.commands import (
    ControlMode, CommandType
)
from pyzlg_dexhand.dexhand_protocol.messages import (
    BoardFeedback, MotorFeedback, ErrorInfo, ProcessedMessage, BoardError
)

# Test data generators
def create_mock_feedback(timestamp=1000.0):
    """Create mock feedback data for testing"""
    return BoardFeedback(
        motor1=MotorFeedback(100, 200, 1000, 45.0),
        motor2=MotorFeedback(-150, -250, -2000, -90.0),
        position_sensor1=45.0,
        position_sensor2=-90.0,
        tactile=StampedTactileFeedback(
            timestamp=timestamp,
            normal_force=1.5,
            normal_force_delta=100,
            tangential_force=0.5,
            tangential_force_delta=50,
            direction=180,
            proximity=500,
            temperature=25
        ),
        encoder1=2048,
        encoder2=1024
    )

class TestHandConfiguration:
    """Test hand configuration loading and validation"""

    def test_config_loading(self, tmp_path):
        """Test loading valid configuration"""
        config_file = tmp_path / "test_config.yaml"
        config_data = {
            "channel": 0,
            "hall_scale": [2.1] * 12
        }
        config_file.write_text(yaml.dump(config_data))

        hand = DexHandBase(str(config_file), BoardID.LEFT_HAND_BASE)
        assert hand.config.channel == 0
        assert len(hand.config.hall_scale) == hand.NUM_MOTORS
        assert hand.config.hall_scale[0] == 2.1

    def test_invalid_config(self, tmp_path):
        """Test loading invalid configuration"""
        config_file = tmp_path / "invalid_config.yaml"
        config_data = {
            "channel": 0,
            # Missing hall_scale
        }
        config_file.write_text(yaml.dump(config_data))

        with pytest.raises(ValueError, match="Missing required keys"):
            DexHandBase(str(config_file), BoardID.LEFT_HAND_BASE)

    def test_wrong_scale_length(self, tmp_path):
        """Test config with wrong number of scale factors"""
        config_file = tmp_path / "wrong_scale.yaml"
        config_data = {
            "channel": 0,
            "hall_scale": [2.1] * 6  # Wrong length
        }
        config_file.write_text(yaml.dump(config_data))

        with pytest.raises(ValueError, match="Expected.*hall scale coefficients"):
            DexHandBase(str(config_file), BoardID.LEFT_HAND_BASE)

class TestHandInitialization:
    """Test hand initialization and device setup"""

    def test_successful_init(self):
        """Test successful hand initialization"""
        mock_zcan = Mock()
        mock_zcan.open.return_value = True
        mock_zcan.configure_channel.return_value = True

        hand = LeftDexHand(zcan=mock_zcan)
        assert hand.init(device_index=0)

        mock_zcan.configure_channel.assert_called_once_with(hand.config.channel)

    def test_init_failure(self):
        """Test handling of initialization failures"""
        mock_zcan = Mock()
        mock_zcan.open.return_value = True
        mock_zcan.configure_channel.return_value = False

        hand = LeftDexHand(zcan=mock_zcan)
        assert not hand.init(device_index=0)

    def test_shared_zcan(self):
        """Test initialization with shared ZCAN instance"""
        mock_zcan = Mock()
        mock_zcan.configure_channel.return_value = True

        left_hand = LeftDexHand(zcan=mock_zcan)
        right_hand = RightDexHand(zcan=mock_zcan)

        assert left_hand.init()
        assert right_hand.init()

        # Verify ZCAN is only configured for each channel
        assert mock_zcan.configure_channel.call_count == 2
        assert mock_zcan.configure_channel.call_args_list == [
            call(left_hand.config.channel),
            call(right_hand.config.channel)
        ]

class TestCommandExecution:
    """Test command execution and feedback handling"""

    @pytest.fixture
    def mock_hand(self):
        """Create a mock hand with configured mocks"""
        mock_zcan = Mock()
        mock_zcan.configure_channel.return_value = True
        hand = LeftDexHand(zcan=mock_zcan)
        hand.init()
        return hand

    def test_successful_command(self, mock_hand):
        """Test successful command execution"""
        mock_feedback = create_mock_feedback()

        mock_hand.zcan.send_fd_message.return_value = True
        mock_hand.zcan.receive_fd_messages.return_value = [(0x181, b'0' * 46, 1000)]

        with patch('pyzlg_dexhand.dexhand_protocol.messages.process_message') as mock_process_message:
            mock_process_message.return_value = ProcessedMessage(
                sender_id=0x181,
                msg_type=MessageType.MOTION_FEEDBACK,
                feedback=mock_feedback
            )

            feedback = mock_hand.move_joints(
                th_rot=30.0,
                th_mcp=45.0,
                control_mode=ControlMode.CASCADED_PID
            )

            assert isinstance(feedback, MoveFeedback)
            assert feedback.command_timestamp > 0
            assert 'th_rot' in feedback.joints
            assert feedback.joints['th_rot'].angle == 45.0
            assert 'th' in feedback.tactile  # Thumb tactile feedback

    def test_error_handling(self, mock_hand):
        """Test handling of hardware errors"""
        mock_error = ErrorInfo(
            error_type=BoardError.MOTOR1_ERROR,
            error_code=0x01,
            description="Test error"
        )

        mock_hand.zcan.send_fd_message.return_value = True
        mock_hand.zcan.receive_fd_messages.return_value = [(0x601, b'error', 1000)]
        with patch('pyzlg_dexhand.dexhand_protocol.messages.process_message') as mock_process_message:
            mock_process_message.return_value = ProcessedMessage(
                sender_id=0x601,
                msg_type=MessageType.ERROR_MESSAGE,
                error=mock_error
            )

            feedback = mock_hand.move_joints(th_rot=30.0)

            assert isinstance(feedback, MoveFeedback)
            assert 'th_rot' in feedback.joints

    def test_communication_failure(self, mock_hand):
        """Test handling of communication failures"""
        mock_hand.zcan.send_fd_message.return_value = False

        feedback = mock_hand.move_joints(th_rot=30.0)

        assert isinstance(feedback, MoveFeedback)
        assert 'th_rot' in feedback.joints
        assert "Communication error" in feedback.joints['th_rot'].error

class TestJointControl:
    """Test joint control functionality"""

    @pytest.fixture
    def mock_hand(self):
        mock_zcan = Mock()
        mock_zcan.configure_channel.return_value = True
        hand = LeftDexHand(zcan=mock_zcan)
        hand.init()
        return hand

    def test_angle_scaling(self, mock_hand):
        """Test angle scaling for different control modes"""
        # Test CASCADED_PID mode
        scaled = mock_hand._scale_angle(0, 45.0, ControlMode.CASCADED_PID)
        assert scaled == 4500  # 45 * 100

        # Test HALL_POSITION mode
        scaled = mock_hand._scale_angle(0, 45.0, ControlMode.HALL_POSITION)
        assert isinstance(scaled, int)
        assert scaled != 0  # Should be scaled by hall factor

    def test_reset_joints(self, mock_hand):
        """Test joint reset functionality"""
        mock_feedback = create_mock_feedback()
        mock_hand.zcan.send_fd_message.return_value = True
        mock_hand.zcan.receive_fd_messages.return_value = [(0x181, b'dummy', 1000)]

        with patch('pyzlg_dexhand.dexhand_protocol.messages.process_message') as mock_process_message:
            mock_process_message.return_value = ProcessedMessage(
                sender_id=0x181,
                msg_type=MessageType.MOTION_FEEDBACK,
                feedback=mock_feedback
            )
            feedback = mock_hand.reset_joints()
            assert isinstance(feedback, MoveFeedback)

    def test_get_feedback(self, mock_hand):
        """Test feedback collection without motion"""
        mock_feedback = create_mock_feedback()
        mock_hand.zcan.send_fd_message.return_value = True
        mock_hand.zcan.receive_fd_messages.return_value = [(0x181, b'dummy', 1000)]

        with patch('pyzlg_dexhand.dexhand_protocol.messages.process_message') as mock_process_message:
            mock_process_message.return_value = ProcessedMessage(
                sender_id=0x181,
                msg_type=MessageType.MOTION_FEEDBACK,
                feedback=mock_feedback
            )

            feedback = mock_hand.get_feedback()
            assert isinstance(feedback, MoveFeedback)
            assert len(feedback.joints) > 0
            assert len(feedback.tactile) > 0

class TestBoardAddressing:
    """Test correct board addressing and ID handling"""

    @pytest.fixture
    def mock_hand(self):
        """Create a mock hand with configured mocks"""
        mock_zcan = Mock()
        mock_zcan.configure_channel.return_value = True
        hand = LeftDexHand(zcan=mock_zcan)
        hand.init()
        return hand

    def test_left_right_separation(self):
        """Test left and right hand board ID separation"""
        left_hand = LeftDexHand()
        right_hand = RightDexHand()

        # Verify base IDs are different
        assert left_hand.base_id == BoardID.LEFT_HAND_BASE
        assert right_hand.base_id == BoardID.RIGHT_HAND_BASE

        # Verify no ID overlap
        left_ids = set(range(left_hand.base_id,
                           left_hand.base_id + left_hand.NUM_BOARDS))
        right_ids = set(range(right_hand.base_id,
                            right_hand.base_id + right_hand.NUM_BOARDS))
        assert not (left_ids & right_ids)  # No intersection

    def test_board_command_ids(self, mock_hand):
        """Test command ID generation for boards"""
        for i in range(mock_hand.NUM_BOARDS):
            cmd_id = mock_hand._get_command_id(MessageType.MOTION_COMMAND, i)
            assert MessageType.MOTION_COMMAND <= cmd_id < MessageType.MOTION_FEEDBACK
            assert (cmd_id - MessageType.MOTION_COMMAND) >= mock_hand.base_id

        with pytest.raises(ValueError):
            mock_hand._get_command_id(MessageType.MOTION_COMMAND, mock_hand.NUM_BOARDS)  # Invalid board index

if __name__ == '__main__':
    pytest.main([__file__])
