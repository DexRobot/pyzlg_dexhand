Control Modes and Advanced Options
=============================

The DexHand interface supports several control modes and advanced options to optimize hand behavior for different applications.

Control Modes
-----------

IMPEDANCE_GRASP Mode (0x77)
^^^^^^^^^^^^^^^^^^^^^^^^^

The IMPEDANCE_GRASP mode is optimized for safe grasping operations:

* Provides a soft, compliant grasp that adapts to object shape
* Automatically detects contact with objects and reduces force
* Prevents damage to both the hand and manipulated objects
* Maintains position control until contact is detected

This is the default and recommended mode for most grasping operations. The impedance value in feedback indicates resistance to movement (lower values indicate higher resistance).

Example usage:

.. code-block:: python

    hand.move_joints(
        th_mcp=30,
        ff_mcp=45,
        mf_mcp=45,
        rf_mcp=45,
        lf_mcp=45,
        control_mode=ControlMode.IMPEDANCE_GRASP
    )

MIT_TORQUE Mode (0x66)
^^^^^^^^^^^^^^^^^^^

The MIT_TORQUE mode allows direct proportional force control while maintaining position control:

* Named after the MIT control approach
* Useful when fine force control is needed during manipulation
* Enables interaction with delicate objects
* Provides direct control over both position and applied force

In this mode, you can set a target position while dynamically adjusting the force/torque limits during movement, creating a balance between position tracking and force limitation.

Example usage:

.. code-block:: python

    hand.move_joints(
        th_mcp=30,
        ff_mcp=45,
        control_mode=ControlMode.MIT_TORQUE
    )

Other Control Modes
^^^^^^^^^^^^^^^^

* ``CASCADED_PID`` (0x44): Default precise position control with high stiffness
* ``HALL_POSITION`` (0x33): Direct hall sensor position control
* ``PROTECT_HALL_POSITION`` (0x55): Safe hall position control requiring zero position at startup
* ``CURRENT`` (0x11): Direct current control for force-based applications
* ``SPEED`` (0x22): Velocity control mode
* ``ZERO_TORQUE`` (0x00): Disables motor torque, allowing free movement

Advanced Movement Options
----------------------

Broadcast Mode (Default)
^^^^^^^^^^^^^^^^^^^^

By default, the move_joints() method uses broadcast mode for more efficient control:

.. code-block:: python

    # Uses broadcast mode by default
    hand.move_joints(
        th_mcp=30,
        ff_mcp=45
    )

    # Explicitly disable broadcast mode if needed
    hand.move_joints(
        th_mcp=30,
        ff_mcp=45,
        use_broadcast=False  # Use per-board commands instead
    )

Benefits of broadcast mode:
* Sends a single command to control all motors simultaneously
* Reduces communication overhead
* Decreases latency for multi-joint movements
* Improves synchronization between joints

Per-Joint Parameter Control
^^^^^^^^^^^^^^^^^^^^^

Fine-tune individual joint parameters using the JointCommand class:

.. code-block:: python

    from pyzlg_dexhand import JointCommand

    # Basic position control (using defaults for current and velocity)
    hand.move_joints(
        th_mcp=30
    )
    
    # Detailed control with JointCommand
    hand.move_joints(
        # Position with custom current (mA)
        th_mcp=JointCommand(position=30, current=25),
        
        # Position with custom velocity (RPM)
        ff_mcp=JointCommand(position=45, velocity=20000),
        
        # Full control: position, current, and velocity
        ff_dip=JointCommand(position=45, current=30, velocity=10000)
    )
    
    # Mix and match simple and detailed control in the same command
    hand.move_joints(
        th_rot=45,  # Just position (using defaults)
        th_mcp=JointCommand(position=30, current=25, velocity=12000)  # Full control
    )

Error Handling Options
^^^^^^^^^^^^^^^^^^

Efficiently clear errors using the broadcast parameter:

.. code-block:: python

    # Efficiently clear all errors with a single command
    hand.clear_errors(use_broadcast=True)  # Recommended for better performance