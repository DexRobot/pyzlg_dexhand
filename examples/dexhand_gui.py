#!/usr/bin/env python3
"""GUI application for controlling DexHand using sliders"""

import sys
import os

try:
    from PyQt6.QtWidgets import (
        QApplication,
        QMainWindow,
        QWidget,
        QVBoxLayout,
        QHBoxLayout,
        QLabel,
        QSlider,
        QPushButton,
        QComboBox,
        QGroupBox,
        QGridLayout,
        QLineEdit,
        QMessageBox,
    )
    from PyQt6.QtCore import Qt
    from PyQt6.QtGui import QClipboard
except ImportError:
    print("Error: This application requires PyQt6.")
    print("Please install it with: pip install PyQt6")
    sys.exit(1)

import logging
from typing import Dict, Optional
from dataclasses import dataclass

from pyzlg_dexhand import (
    LeftDexHand,
    RightDexHand,
    ControlMode,
    ZCANWrapper,
    MockZCANWrapper
)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class JointConfig:
    """Configuration for a joint slider"""
    name: str           # Internal joint name
    label: str          # Display label
    min_angle: float    # Minimum angle in degrees
    max_angle: float    # Maximum angle in degrees
    default: float = 0  # Default position

# Joint configurations with proper ranges
JOINT_CONFIGS = {
    "th_rot":  JointConfig("th_rot", "Thumb Rotation", 0, 150),
    "th_mcp":  JointConfig("th_mcp", "Thumb MCP", 0, 90),
    "th_dip":  JointConfig("th_dip", "Thumb DIP", 0, 90),
    "ff_spr":  JointConfig("ff_spr", "Finger Spread", 0, 30),
    "ff_mcp":  JointConfig("ff_mcp", "Index MCP", 0, 90),
    "ff_dip":  JointConfig("ff_dip", "Index DIP", 0, 90),
    "mf_mcp":  JointConfig("mf_mcp", "Middle MCP", 0, 90),
    "mf_dip":  JointConfig("mf_dip", "Middle DIP", 0, 90),
    "rf_mcp":  JointConfig("rf_mcp", "Ring MCP", 0, 90),
    "rf_dip":  JointConfig("rf_dip", "Ring DIP", 0, 90),
    "lf_mcp":  JointConfig("lf_mcp", "Little MCP", 0, 90),
    "lf_dip":  JointConfig("lf_dip", "Little DIP", 0, 90),
}

class JointSlider(QWidget):
    """Widget combining a slider with labels and input box for joint control"""

    def __init__(self, config: JointConfig, parent=None):
        super().__init__(parent)
        self.config = config
        self._updating = False  # Prevent recursive updates

        # Create layout
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)

        # Add joint name label
        self.name_label = QLabel(config.label)
        self.name_label.setMinimumWidth(100)
        layout.addWidget(self.name_label)

        # Create slider
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setMinimum(int(config.min_angle * 10))  # 0.1 degree precision
        self.slider.setMaximum(int(config.max_angle * 10))
        self.slider.setValue(int(config.default * 10))
        self.slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.slider.setTickInterval(100)  # 10 degree ticks
        layout.addWidget(self.slider)

        # Add value input box
        self.value_edit = QLineEdit()
        self.value_edit.setMaximumWidth(60)
        self.value_edit.setText(f"{config.default:.1f}")
        layout.addWidget(self.value_edit)
        layout.addWidget(QLabel("°"))

        # Connect signals
        self.slider.valueChanged.connect(self._slider_changed)
        self.value_edit.returnPressed.connect(self._edit_changed)

        self.setLayout(layout)

    def _slider_changed(self, value: int):
        """Handle slider movement"""
        if self._updating:
            return
        self._updating = True
        angle = value / 10.0
        self.value_edit.setText(f"{angle:.1f}")
        self._updating = False

    def _edit_changed(self):
        """Handle manual value entry"""
        if self._updating:
            return
        try:
            value = float(self.value_edit.text())
            value = min(max(value, self.config.min_angle), self.config.max_angle)
            self._updating = True
            self.slider.setValue(int(value * 10))
            self.value_edit.setText(f"{value:.1f}")
            self._updating = False
        except ValueError:
            # Restore previous value on invalid input
            self._updating = True
            self.value_edit.setText(f"{self.value():.1f}")
            self._updating = False

    def value(self) -> float:
        """Get current joint angle in degrees"""
        return self.slider.value() / 10.0

    def setValue(self, angle: float):
        """Set joint angle"""
        self._updating = True
        angle = min(max(angle, self.config.min_angle), self.config.max_angle)
        self.slider.setValue(int(angle * 10))
        self.value_edit.setText(f"{angle:.1f}")
        self._updating = False

class DexHandControl(QMainWindow):
    """Main window for DexHand control interface"""

    def __init__(self):
        super().__init__()

        # Initialize hardware interface
        self.zcan = ZCANWrapper()

        # Initialize both hands (but don't connect yet)
        self.left_hand = None
        self.right_hand = None
        self.current_hand = None

        # Store last positions for each hand
        self.left_positions = {name: config.default for name, config in JOINT_CONFIGS.items()}
        self.right_positions = {name: config.default for name, config in JOINT_CONFIGS.items()}

        # Create and initialize UI
        self.init_ui()

        # Initialize hardware
        try:
            if not self.zcan.open():
                QMessageBox.critical(self, "Error", "Failed to open ZCAN device")
                sys.exit(1)

            # Initialize both hands
            self.left_hand = LeftDexHand(self.zcan)
            self.right_hand = RightDexHand(self.zcan)

            if not self.left_hand.init() or not self.right_hand.init():
                QMessageBox.critical(self, "Error", "Failed to initialize hands")
                sys.exit(1)

            # Start with left hand
            self.select_hand("Left Hand")

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error initializing hardware: {e}")
            sys.exit(1)

    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle('DexHand Control')

        # Create main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout()

        # Create control panel
        control_panel = QWidget()
        control_layout = QHBoxLayout()

        # Hand selection dropdown
        self.hand_combo = QComboBox()
        self.hand_combo.addItems(['Left Hand', 'Right Hand'])
        self.hand_combo.currentTextChanged.connect(self.select_hand)
        control_layout.addWidget(QLabel("Select Hand:"))
        control_layout.addWidget(self.hand_combo)

        # Reset button
        reset_btn = QPushButton("Reset All")
        reset_btn.clicked.connect(self.reset_joints)
        control_layout.addWidget(reset_btn)

        # Copy positions button
        copy_btn = QPushButton("Copy Positions")
        copy_btn.clicked.connect(self.copy_positions)
        control_layout.addWidget(copy_btn)

        control_panel.setLayout(control_layout)
        layout.addWidget(control_panel)

        # Create groups for different fingers
        thumb_group = self._create_joint_group("Thumb", ["th_rot", "th_mcp", "th_dip"])
        index_group = self._create_joint_group("Index", ["ff_spr", "ff_mcp", "ff_dip"])
        middle_group = self._create_joint_group("Middle", ["mf_mcp", "mf_dip"])
        ring_group = self._create_joint_group("Ring", ["rf_mcp", "rf_dip"])
        little_group = self._create_joint_group("Little", ["lf_mcp", "lf_dip"])

        # Add groups to grid layout
        grid = QGridLayout()
        grid.addWidget(thumb_group, 0, 0)
        grid.addWidget(index_group, 0, 1)
        grid.addWidget(middle_group, 1, 0)
        grid.addWidget(ring_group, 1, 1)
        grid.addWidget(little_group, 2, 0)

        layout.addLayout(grid)
        main_widget.setLayout(layout)

    def _create_joint_group(self, title: str, joint_names: list) -> QGroupBox:
        """Create a group box containing sliders for the specified joints"""
        group = QGroupBox(title)
        layout = QVBoxLayout()

        for name in joint_names:
            config = JOINT_CONFIGS[name]
            slider = JointSlider(config)
            slider.slider.valueChanged.connect(self.sliders_changed)
            slider.value_edit.returnPressed.connect(self.sliders_changed)
            layout.addWidget(slider)
            setattr(self, f"slider_{name}", slider)

        group.setLayout(layout)
        return group

    def select_hand(self, hand_name: str):
        """Switch between left and right hand control"""
        # Save current positions
        if self.current_hand == self.left_hand:
            self.left_positions = self._get_slider_positions()
        elif self.current_hand == self.right_hand:
            self.right_positions = self._get_slider_positions()

        # Switch hands
        if hand_name == "Left Hand":
            self.current_hand = self.left_hand
            self._set_slider_positions(self.left_positions)
        else:
            self.current_hand = self.right_hand
            self._set_slider_positions(self.right_positions)

        # Send current positions
        self.sliders_changed()

    def _get_slider_positions(self) -> Dict[str, float]:
        """Get current positions of all sliders"""
        return {
            name: getattr(self, f"slider_{name}").value()
            for name in JOINT_CONFIGS
        }

    def _set_slider_positions(self, positions: Dict[str, float]):
        """Set all slider positions"""
        for name, value in positions.items():
            getattr(self, f"slider_{name}").setValue(value)

    def sliders_changed(self):
        """Send updated joint positions when any slider changes"""
        if not self.current_hand:
            return

        try:
            # Collect all joint positions
            positions = self._get_slider_positions()

            # Send to hand
            self.current_hand.move_joints(**positions)

            # Clear any errors
            self.current_hand.clear_errors()

        except Exception as e:
            logger.error(f"Error sending joint commands: {e}")

    def reset_joints(self):
        """Reset all joints to zero position"""
        if not self.current_hand:
            return

        try:
            # Reset all sliders
            for name in JOINT_CONFIGS:
                slider = getattr(self, f"slider_{name}")
                slider.setValue(0)

            # Update stored positions
            if self.current_hand == self.left_hand:
                self.left_positions = {name: 0.0 for name in JOINT_CONFIGS}
            else:
                self.right_positions = {name: 0.0 for name in JOINT_CONFIGS}

            # Send command
            self.sliders_changed()

        except Exception as e:
            logger.error(f"Error resetting joints: {e}")

    def copy_positions(self):
        """Copy current joint positions to clipboard as Python dict"""
        try:
            positions = self._get_slider_positions()
            # Format as Python dictionary string with proper indentation
            text = "{\n"
            text += "".join(f"    '{k}': {v:.1f},\n" for k, v in sorted(positions.items()))
            text += "}"

            # Copy to clipboard
            clipboard = QApplication.clipboard()
            clipboard.setText(text)

        except Exception as e:
            logger.error(f"Error copying positions: {e}")

    def closeEvent(self, event):
        """Clean up when window is closed"""
        try:
            if self.left_hand:
                self.left_hand.close()
            if self.right_hand:
                self.right_hand.close()
        except Exception as e:
            logger.error(f"Error closing hands: {e}")
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = DexHandControl()
    window.resize(800, 600)
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
