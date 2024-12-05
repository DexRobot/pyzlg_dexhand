# DexHand Control Interface

Python interface for controlling dexterous robotic hands over CANFD using ZLG USBCANFD adapters. Provides both direct control and ROS2 integration.

## Prerequisites

- Linux environment
- Python 3.8+
- ZLG USBCANFD adapter (tested with USBCANFD-200U)
- ROS2 (optional, for ROS interface)

## Quickstart Guide

### 1. USB Setup

First, configure USB permissions for the CANFD adapter:

```bash
# Run the setup script (only needed once after connecting adapter)
./setup_usb_can.sh
```

### 2. Test Device Connection

Use the basic test script to verify hardware communication:

```bash
# Test left hand only
python test/test_dexhand.py --hands left

# Test both hands
python test/test_dexhand.py --hands left right
```

The test script will run through a sequence of movements to verify proper operation.

### 3. Interactive Control

For manual testing and control, use the interactive interface:

```bash
# Start interactive control of left hand
python test/test_dexhand_interactive.py --hands left
```

This launches an IPython shell with the hand interface loaded. Example commands:

```python
# Move individual joints
left_hand.move_joints(thumb_rot=30)
left_hand.move_joints(finger_spread=20)

# Reset to default position
left_hand.reset_joints()

# Clear any errors
left_hand.clear_errors()
```

### 4. ROS2 Interface

For integration with ROS2 applications:

```bash
# Start ROS2 node with the default configuration (see config/ros_node.yaml for the configuration options)
python dexhand_ros2.py

# Test hand movements using ROS2 test node
python test/dexhand_ros2_test.py --hands right --cycle-time 3.0
```

The ROS2 node subscribes to `/joint_commands` for commands by default and provides a `/reset_hands` service.

### 5. Python API

For custom control and integration, refer to the interfaces provided by `dexhand_interface.py`.

## Architecture Overview

The project is organized in multiple abstraction layers:

### 1. ZCAN Layer (zcan.py)
- Lowest level interface to ZLG CANFD adapter
- Handles raw CANFD frame construction and transmission
- Maps C driver functions to Python interface
- Key classes: `ZCAN`, `ZCANMessage`, `ZCANFDMessage`

### 2. ZCAN Wrapper Layer (zcan_wrapper.py)
- Provides high-level wrapper around ZCAN functionality
- Handles device initialization and configuration
- Manages message filtering and error handling
- Converts between byte data and CANFD frames
- Key class: `ZCANWrapper`

### 3. DexHand Interface Layer (dexhand_interface.py)
- Implements semantic hand control interface
- Converts joint angles to CAN messages
- Handles command scaling and feedback parsing
- Provides both left and right hand implementations
- Key classes: `DexHandBase`, `LeftDexHand`, `RightDexHand`

### 4. ROS2 Interface Layer (dexhand_ros2.py)
- Provides ROS2 integration
- Maps between ROS joint states and hand commands
- Implements command filtering and rate limiting
- Handles multiple hand coordination
- Key class: `DexHandNode`

## Control Modes

The following control modes are supported:

- `ZERO_TORQUE` (0x00): Motors disabled
- `CURRENT` (0x11): Direct current control
- `SPEED` (0x22): Velocity control
- `HALL_POSITION` (0x33): Position control using hall sensors
- `CASCADED_PID` (0x44): Cascaded position/velocity control
- `PROTECT_HALL_POSITION` (0x55): Protected position control

## Data Logging

The system includes built-in logging functionality:

```python
# Initialize logger
logger = DexHandLogger()

# Log commands and feedback
logger.log_command(positions, enables, control_mode, success, hand)
logger.log_feedback(feedback_data, hand)

# Generate plots
logger.plot_session(show=True, save=True)
```

Logs are saved in `dexhand_logs/` with timestamps and can be visualized using the included plotting tools.

## Contributing

When contributing to this project:

1. Follow the existing code structure and abstraction layers
2. Add appropriate error handling and logging
3. Update tests as needed
4. Document new features or changes

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

While this software is primarily designed for use with DexHand products, it is open source and we welcome contributions from the community. Whether you're fixing bugs, improving documentation, or adding new features, please feel free to submit pull requests.

Note: This software is provided as-is. While we strive to maintain compatibility with DexHand products, use this software at your own risk.
