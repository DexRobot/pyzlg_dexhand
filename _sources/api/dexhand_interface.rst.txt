DexHand Interface
===============

The DexHand interface module provides high-level control of dexterous robotic hands.

Base Interface
------------

.. autoclass:: pyzlg_dexhand.dexhand_interface.DexHandBase
    :members:
    :special-members: __init__
    :inherited-members:
    :exclude-members: NUM_MOTORS, NUM_BOARDS

    .. attribute:: NUM_MOTORS

        Total number of motors in the hand (12)

    .. attribute:: NUM_BOARDS

        Number of control boards (6)

    .. attribute:: joint_names

        List of joint names in order:

        - th_dip: Thumb distal joint
        - th_mcp: Thumb metacarpophalangeal joint
        - th_rot: Thumb rotation
        - ff_spr: Four-finger spread
        - ff_dip: Index finger distal joint
        - ff_mcp: Index finger metacarpophalangeal
        - mf_dip: Middle finger distal joint
        - mf_mcp: Middle finger metacarpophalangeal
        - rf_dip: Ring finger distal joint
        - rf_mcp: Ring finger metacarpophalangeal
        - lf_dip: Little finger distal joint
        - lf_mcp: Little finger metacarpophalangeal

Hand Implementations
-----------------

.. autoclass:: pyzlg_dexhand.dexhand_interface.LeftDexHand
    :members:
    :show-inheritance:
    :special-members: __init__

.. autoclass:: pyzlg_dexhand.dexhand_interface.RightDexHand
    :members:
    :show-inheritance:
    :special-members: __init__

Data Classes
----------

.. autoclass:: pyzlg_dexhand.dexhand_interface.HandConfig
    :members:
    :undoc-members:

.. autoclass:: pyzlg_dexhand.dexhand_interface.JointFeedback
    :members:
    :undoc-members:

.. autoclass:: pyzlg_dexhand.dexhand_interface.StampedTactileFeedback
    :members:
    :undoc-members:

.. autoclass:: pyzlg_dexhand.dexhand_interface.HandFeedback
    :members:
    :undoc-members:

Enumerations
----------

.. autoclass:: pyzlg_dexhand.dexhand_interface.ControlMode
    :members:
    :undoc-members:

Example Usage
-----------

Basic Joint Control
^^^^^^^^^^^^^^^^

.. code-block:: python

    from pyzlg_dexhand import RightDexHand, ControlMode

    # Initialize hand
    hand = RightDexHand()
    hand.init()

    # Move thumb
    hand.move_joints(
        th_rot=30,     # Thumb rotation
        th_mcp=45,     # Thumb MCP flexion
        th_dip=45,     # Thumb coupled distal flexion
        control_mode=ControlMode.CASCADED_PID
    )

Feedback Handling
^^^^^^^^^^^^^^^

.. code-block:: python

    # Get feedback
    feedback = hand.get_feedback()

    # Access joint feedback
    thumb_angle = feedback.joints['th_rot'].angle
    thumb_encoder = feedback.joints['th_rot'].encoder_position

    # Access tactile feedback
    thumb_force = feedback.tactile['th'].normal_force
    thumb_dir = feedback.tactile['th'].direction

Error Handling
^^^^^^^^^^^^

.. code-block:: python

    # Clear any error states
    hand.clear_errors(clear_all=True)

    # Check for specific errors
    errors = hand.get_errors()
    for board_idx, error in errors.items():
        if error:
            print(f"Board {board_idx} error: {error.description}")

Notes
-----

Control Modes
^^^^^^^^^^^

* ``CASCADED_PID``: Default mode for precise position control. Provides highest stiffness and position accuracy.
* ``HALL_POSITION``: Direct hall sensor position control. Less precise but faster response.
* ``PROTECT_HALL_POSITION``: Safe hall position control requiring zero position at startup.
* ``CURRENT``: Direct current control for force-based applications.
* ``SPEED``: Velocity control mode.
* ``ZERO_TORQUE``: Disables motor torque, allowing free movement.

Joint Limits
^^^^^^^^^^

Default joint angle limits (in degrees):

* Thumb rotation (th_rot): 0-150
* Metacarpophalangeal joints (mcp): 0-90
* Distal joints (dip): 0-90
* Four-finger spread (ff_spr): 0-30

Error Handling
^^^^^^^^^^^^

When a finger encounters an obstacle or abnormal condition:

1. The board may enter an error state
2. Motors on that board become unresponsive
3. ``clear_errors()`` must be called to restore operation
4. Consider clearing errors after each command for robust operation
