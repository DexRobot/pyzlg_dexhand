[English](README.md) | [中文](README_zh.md)

# DexHand Python Interface

Python interface for controlling dexterous robotic hands over CANFD using ZLG USBCANFD adapters. Provides both direct control and ROS2 integration.

## Overview

This package provides:

- CANFD communication interface for DexHand hardware
- Joint-space control interface with feedback processing
- Built-in data logging and visualization tools
- ROS2 interface implementation
- Hardware testing utilities

## Prerequisites

- Linux environment
- Python 3.8+
- ZLG USBCANFD adapter (tested with USBCANFD-200U)
- ROS1/ROS2 (optional, for ROS interface)

## Hardware setup

Please refer to the following diagram:

![DexHand Connection Diagram](docs/assets/connection.svg)

## Installation

1. Download or clone the repository:

   ```bash
   git clone https://gitee.com/DexRobot/pyzlg_dexhand.git
   ```

2. Install the package:

   ```bash
   pip install -e .
   ```

3. Configure USB permissions:

   ```bash
   sudo ./tools/setup_usb_can.sh
   ```

   The setup script will:

   - Create a canbus group
   - Add your user to the group
   - Set up udev rules for the USBCANFD adapter
   - Configure appropriate permissions

   **You may need to log out and back in for the changes to take effect.**

4. Edit `config/config.yaml` to match your hardware setup, especially **channels and ZCAN device type**.

## Usage Examples

### 1. Hardware Testing

Run hardware tests:

```bash
python tools/hardware_test/test_dexhand.py --hands right
```

This should move the hand through a series of predefined motions.

### 2. Interactive Testing

Launch interactive control interface:

```bash
python tools/hardware_test/test_dexhand_interactive.py --hands right
```

This provides an IPython shell with initialized hand objects and helper functions.

Example commands:

```python
right_hand.move_joints(th_rot=30)  # Rotate thumb
right_hand.move_joints(ff_mcp=60, ff_dip=60)  # Curl index finger
right_hand.move_joints(ff_spr=20, control_mode=ControlMode.PROTECT_HALL_POSITION)  # Spread all fingers, with alternative control mode
right_hand.get_feedback()
right_hand.reset_joints()
right_hand.clear_errors()    # Clear all error states
```

You can explore the API with tab completion and help commands.

### 3. ROS Integration

The SDK provides a ROS interface that supports both ROS1 (rospy) and ROS2 (rclpy) environments, automatically detecting and using the appropriate framework.

Usage:

```bash
# Launch the ROS node with default configuration
python examples/ros_node/dexhand_ros.py

# Run the demo publisher (for testing)
python examples/ros_node/dexhand_ros_publisher_demo.py --hands right --cycle-time 3.0
```

Interface:

| Topic (default)     | Type                     | Direction | Description                           |
| ------------------- | ------------------------ | --------- | ------------------------------------- |
| `/joint_commands`   | `sensor_msgs/JointState` | Input     | Joint position commands               |
| `/joint_states`     | `sensor_msgs/JointState` | Output    | Joint position feedback (Coming soon) |
| `/tactile_feedback` | TBD                      | Output    | Tactile sensor data (Coming soon)     |

Topic names configurable via `config/config.yaml`.

| Service        | Type               | Description                     |
| -------------- | ------------------ | ------------------------------- |
| `/reset_hands` | `std_srvs/Trigger` | Reset hands to default position |

Notes:

- Joint names in commands match the URDF file specifications
- Configuration can be customized through `config/config.yaml`
- All features work identically in both ROS1 and ROS2 environments

### 4. Programming Interface

Example code:

```python
from pyzlg_dexhand import LeftDexHand, RightDexHand, ControlMode

# Initialize hand
hand = RightDexHand()
hand.init()

# Move thumb
hand.move_joints(
    th_rot=30,  # Thumb rotation (0-150 degrees)
    th_mcp=45,  # Thumb MCP flexion (0-90 degrees)
    th_dip=45,  # Thumb coupled distal flexion
    control_mode=ControlMode.CASCADED_PID
)

# Get feedback
feedback = hand.get_feedback()
print(f"Thumb angle: {feedback.joints['th_rot'].angle}")
print(f"Tactile force: {feedback.tactile['th'].normal_force}")
```

Notes:

- Control Modes

  - `CASCADED_PID`: Provides precise position control with higher stiffness
  - `PROTECT_HALL_POSITION`: Offers smoother response but requires joints to be in zero position at power-on

- Error Handling

  - When a finger's motion is obstructed by an object, it may enter an error state and become unresponsive to control signals. For reliable continuous control, call `hand.clear_errors()` after sending each command

## Architecture

### Core Components

#### 1. ZCAN Layer (zcan.py)

- Raw CANFD frame handling
- Hardware initialization
- Error handling
- Message filtering

#### 2. Protocol Layer (dexhand_protocol/)

- Command encoding/decoding
- Message parsing
- Error detection
- Feedback processing

#### 3. Interface Layer (dexhand_interface.py)

- High-level control API
- Joint space mapping
- Feedback processing
- Error recovery

### Applications

The package includes example applications built using the core interface:

- ROS2 interface (examples/ros2_demo/)
- Hardware testing tools (tools/hardware_test/)
- Interactive testing shell

## Data Logging

Built-in logging for analysis and debugging:

```python
from pyzlg_dexhand import DexHandLogger

# Initialize logger
logger = DexHandLogger()

# Log commands and feedback
logger.log_command(command_type, joint_commands, control_mode, hand)
logger.log_feedback(feedback_data, hand)

# Generate analysis
logger.plot_session(show=True, save=True)
```

Logs include:

- Joint commands and feedback
- Tactile sensor data
- Error states
- Timing information

## Configuration

Configuration files in `config/`:

- `config.yaml`: Left/Right hand parameters, ROS2 node settings, and ZCAN configuration

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/improvement`)
3. Follow the existing code structure and documentation
4. Add appropriate error handling and logging
5. Update tests as needed
6. Submit a pull request

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

Note: This software is provided as-is. While we strive to maintain compatibility with DexHand products, use this software at your own risk.
