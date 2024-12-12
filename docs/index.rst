DexHand Python Interface
=========================

Python interface for controlling dexterous robotic hands over CANFD using ZLG USBCANFD adapters. Provides both direct control and ROS2 integration.

.. toctree::
   :maxdepth: 2
   :caption: Contents:


Installation
-------------

1. Install the package:

.. code-block:: bash

    pip install -e .

2. Configure USB permissions:

.. code-block:: bash

    sudo ./tools/setup_usb_can.sh

The setup script will:

- Create a canbus group
- Add your user to the group
- Set up udev rules for the USBCANFD adapter
- Configure appropriate permissions

You may need to log out and back in for the changes to take effect.

Quick Start
------------

1. Hardware Testing

Run hardware tests:

.. code-block:: bash

    python tools/hardware_test/test_dexhand.py --hands right

This should move the hand through a series of predefined motions.

2. Interactive Testing

Launch interactive control interface:

.. code-block:: bash

    python tools/hardware_test/test_dexhand_interactive.py --hands right

This provides an IPython shell with initialized hand objects and helper functions.

Example commands:

.. code-block:: python

    right_hand.move_joints(th_rot=30)  # Rotate thumb
    right_hand.move_joints(ff_mcp=60, ff_dip=60)  # Curl index finger
    right_hand.move_joints(ff_spr=20, control_mode=ControlMode.PROTECT_HALL_POSITION)  # Spread all fingers
    right_hand.get_feedback()
    right_hand.reset_joints()
    right_hand.clear_errors()    # Clear all error states

You can explore the API with tab completion and help commands.

3. ROS2 Integration

.. code-block:: bash

    # Start node with default config
    python examples/ros2_demo/dexhand_ros2.py

    # Test movements with an example publisher
    python examples/ros2_demo/dexhand_ros2_test.py --hands right --cycle-time 3.0

4. Programming Interface

Example code:

.. code-block:: python

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

API Reference
--------------

.. automodule:: pyzlg_dexhand.dexhand_interface
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: pyzlg_dexhand.dexhand_protocol
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: pyzlg_dexhand.dexhand_protocol.commands
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: pyzlg_dexhand.dexhand_protocol.messages
   :members:
   :undoc-members:
   :show-inheritance:
    
.. automodule:: pyzlg_dexhand.zcan_wrapper
   :members:
   :undoc-members:
   :show-inheritance: