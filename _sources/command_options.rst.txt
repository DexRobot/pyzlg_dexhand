Joint Control Options
==================

The DexHand interface provides flexible ways to control the robotic hand, with options to optimize for different use cases.

move_joints Function
------------------

The primary interface for controlling the DexHand is the ``move_joints()`` function, which allows you to control specific joints with precise parameters:

.. code-block:: python

    # Standard joint control
    hand.move_joints(
        th_rot=30,     # Thumb rotation
        th_mcp=45,     # Thumb MCP flexion
        ff_mcp=60,     # Index finger MCP
        control_mode=ControlMode.IMPEDANCE_GRASP
    )

JointCommand Parameters
--------------------

The ``move_joints()`` function now supports the ``JointCommand`` type, which gives you more granular control over position, current, and velocity for each joint:

.. code-block:: python

    from pyzlg_dexhand import JointCommand

    # Move a single joint with position only
    hand.move_joints(th_rot=45)
    
    # Move a joint with full control (position, current, velocity)
    hand.move_joints(
        th_rot=JointCommand(position=45, current=30, velocity=10000)
    )
    
    # Mix and match parameter styles for different joints
    hand.move_joints(
        th_rot=45,  # Position only (using defaults for current and velocity)
        ff_mcp=JointCommand(position=30, current=50),  # Position and current
        mf_dip=JointCommand(position=20, velocity=5000)  # Position and velocity
    )

This approach is more flexible than the older style of using ``speeds`` and ``currents`` parameters, which are still supported for backward compatibility.

Control Mode Selection
^^^^^^^^^^^^^^^^^^^^^

Control the behavior of the motors using different control strategies:

.. code-block:: python

    # Using specific control mode
    hand.move_joints(
        th_mcp=30,
        ff_mcp=45,
        control_mode=ControlMode.MIT_TORQUE
    )

Broadcast Mode (Default)
---------------------

The ``move_joints()`` function uses an efficient broadcast mode by default that optimizes communication with the hand:

.. code-block:: python

    # Broadcast mode is used by default (more efficient)
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

**Benefits of broadcast mode:**

* More efficient: Sends a single message instead of multiple per-board commands
* Lower latency: All motors receive commands simultaneously
* Recommended for applications requiring high update rates
* Better synchronization of multi-joint movements

**When to use per-board commands (disable broadcast):**

* When you need to control specific boards individually
* In rare cases where broadcast causes timing issues
* When debugging specific boards or motors
* When very fine-grained control timing is required

**Implementation note:**

With broadcast mode (default), the system uses a more efficient communication protocol internally that sends one command affecting all motors, rather than individual commands to each board.

Additional Options
------------------

The ``move_joints()`` function supports several additional parameters:

.. code-block:: python

    hand.move_joints(
        th_rot=30,
        th_mcp=45,
        control_mode=ControlMode.IMPEDANCE_GRASP,
        use_broadcast=True,          # More efficient communication (default)
        clear_error=True,            # Clear any errors during command (broadcast only)
        request_feedback=True,       # Request feedback after command (broadcast only)
        log_level=LogLevel.INFO      # Logging level for this operation
    )

Error Handling
------------

The ``clear_errors()`` function accepts a ``use_broadcast`` parameter that also enables more efficient communication:

.. code-block:: python

    # Efficiently clear all errors with a single command
    hand.clear_errors(use_broadcast=True)  # Recommended

    # Or clear specific error states
    hand.clear_errors(clear_all=False, use_broadcast=False)

Using this optimized approach is recommended in most cases for better performance.