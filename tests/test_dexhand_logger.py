"""Tests for DexHandLogger"""

import pytest
import json
import time
from pathlib import Path
from dataclasses import asdict
import numpy as np
from unittest.mock import patch, Mock

from pyzlg_dexhand.dexhand_logger import DexHandLogger
from pyzlg_dexhand.dexhand_interface import (
    MoveFeedback, JointFeedback, StampedTactileFeedback,
    ControlMode
)

@pytest.fixture
def test_logger(tmp_path):
    """Create a logger instance with temporary directory"""
    return DexHandLogger(str(tmp_path))

@pytest.fixture
def mock_feedback():
    """Create mock MoveFeedback for testing"""
    return MoveFeedback(
        command_timestamp=time.time(),
        joints={
            'th_rot': JointFeedback(
                timestamp=time.time(),
                angle=45.0,
                encoder_position=1000,
                error=None,
                error_cleared=None
            ),
            'th_mcp': JointFeedback(
                timestamp=time.time(),
                angle=30.0,
                encoder_position=500,
                error=None,
                error_cleared=None
            )
        },
        tactile={
            'th': StampedTactileFeedback(
                timestamp=time.time(),
                normal_force=1.5,
                normal_force_delta=100,
                tangential_force=0.5,
                tangential_force_delta=50,
                direction=180,
                proximity=500,
                temperature=25
            )
        }
    )

class TestDexHandLogger:
    """Test DexHandLogger functionality"""

    def test_initialization(self, test_logger):
        """Test logger initialization"""
        assert test_logger.log_dir.exists()
        assert test_logger.session_dir.exists()
        assert (test_logger.session_dir / "left_commands.jsonl").exists()
        assert (test_logger.session_dir / "right_commands.jsonl").exists()
        assert (test_logger.session_dir / "left_feedback.jsonl").exists()
        assert (test_logger.session_dir / "right_feedback.jsonl").exists()

    def test_log_command(self, test_logger, mock_feedback):
        """Test command logging"""
        test_logger.log_command("move_joints", mock_feedback, "left")

        # Check buffer
        assert len(test_logger.command_buffers["left"]) == 1
        cmd = test_logger.command_buffers["left"][0]
        assert cmd.command_type == "move_joints"
        assert "th_rot" in cmd.joint_commands
        assert cmd.joint_commands["th_rot"] == 45.0

        # Check file
        cmd_file = test_logger.session_dir / "left_commands.jsonl"
        with open(cmd_file) as f:
            lines = f.readlines()
            assert len(lines) == 1
            data = json.loads(lines[0])
            assert data["command_type"] == "move_joints"
            assert data["joint_commands"]["th_rot"] == 45.0

    def test_log_feedback(self, test_logger, mock_feedback):
        """Test feedback logging"""
        test_logger.log_feedback(mock_feedback, "left")

        # Check buffer
        assert len(test_logger.feedback_buffers["left"]) == 1
        fb = test_logger.feedback_buffers["left"][0]
        assert "th_rot" in fb.joints
        assert "th" in fb.tactile

        # Check file
        fb_file = test_logger.session_dir / "left_feedback.jsonl"
        with open(fb_file) as f:
            lines = f.readlines()
            assert len(lines) == 1
            data = json.loads(lines[0])
            assert "joints" in data
            assert "th_rot" in data["joints"]
            assert "tactile" in data
            assert "th" in data["tactile"]

    def test_save_metadata(self, test_logger):
        """Test metadata saving"""
        metadata = {
            "test_param": 123,
            "test_string": "value"
        }
        test_logger.save_metadata(metadata)

        meta_file = test_logger.session_dir / "metadata.json"
        assert meta_file.exists()

        with open(meta_file) as f:
            loaded = json.load(f)
            assert loaded == metadata

    @pytest.mark.skipif(not pytest.importorskip("matplotlib", reason="matplotlib required"),
                     reason="matplotlib not available")
    def test_plot_session(self, test_logger, mock_feedback):
        """Test plotting functionality"""
        # Log some test data
        test_logger.log_command("move_joints", mock_feedback, "left")
        test_logger.log_feedback(mock_feedback, "left")

        # Test plotting without showing
        test_logger.plot_session(show=False, save=True)

        # Check that plot files were created
        assert (test_logger.session_dir / "left_commands.png").exists()
        assert (test_logger.session_dir / "left_tactile.png").exists()

    def test_close(self, test_logger, mock_feedback):
        """Test logger closing"""
        # Log some test data
        test_logger.log_command("move_joints", mock_feedback, "left")
        test_logger.log_feedback(mock_feedback, "left")

        # Close logger
        test_logger.close()

        # Check metadata file
        meta_file = test_logger.session_dir / "metadata.json"
        assert meta_file.exists()

        with open(meta_file) as f:
            metadata = json.load(f)
            assert "statistics" in metadata
            assert metadata["statistics"]["num_commands"]["left"]
