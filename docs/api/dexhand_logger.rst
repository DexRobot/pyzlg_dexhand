DexHand Logger
=============

The DexHand logger module provides data logging and analysis capabilities for hand operation.

Core Components
-------------

Log Entry Types
^^^^^^^^^^^^^

.. autoclass:: pyzlg_dexhand.dexhand_logger.LogEntry
    :members:
    :undoc-members:

.. autoclass:: pyzlg_dexhand.dexhand_logger.CommandLogEntry
    :members:
    :undoc-members:

.. autoclass:: pyzlg_dexhand.dexhand_logger.FeedbackLogEntry
    :members:
    :undoc-members:

Logger Classes
-----------

Main Logger
^^^^^^^^^

.. autoclass:: pyzlg_dexhand.dexhand_logger.DexHandLogger
    :members:
    :special-members: __init__

Background Writer
^^^^^^^^^^^^^^

.. autoclass:: pyzlg_dexhand.dexhand_logger.LogWriter
    :members:
    :special-members: __init__

Examples
-------

Basic Logging
^^^^^^^^^^^

.. code-block:: python

    from pyzlg_dexhand import DexHandLogger, ControlMode

    # Initialize logger
    logger = DexHandLogger(log_dir="dexhand_logs")

    # Log a command
    logger.log_command(
        command_type="move_joints",
        joint_commands={"th_rot": 30, "th_mcp": 45},
        control_mode=ControlMode.CASCADED_PID,
        hand="right"
    )

    # Log feedback
    logger.log_feedback(feedback_data, hand="right")

    # Save session metadata
    logger.save_metadata({
        "operator": "user1",
        "experiment": "grasp_test",
        "conditions": "normal"
    })

    # Generate plots and close
    logger.plot_session(show=False, save=True)
    logger.close()

Visualization
^^^^^^^^^^^

.. code-block:: python

    # Plot specific hands
    logger.plot_session(
        hands=["right"],
        show=True,
        save=True
    )

Analysis
^^^^^^^

.. code-block:: python

    # Get session statistics
    with logger.buffer_lock:
        cmd_count = len(logger.command_buffers["right"])
        fb_count = len(logger.feedback_buffers["right"])
        print(f"Commands: {cmd_count}, Feedback: {fb_count}")

Log File Structure
----------------

Directory Layout
^^^^^^^^^^^^^

Each logging session creates a timestamped directory::

    dexhand_logs/
    └── YYYYMMDD_HHMMSS/
        ├── metadata.json
        ├── left_commands.jsonl
        ├── left_feedback.jsonl
        ├── right_commands.jsonl
        ├── right_feedback.jsonl
        ├── left_joints.png
        ├── left_tactile.png
        ├── right_joints.png
        └── right_tactile.png

File Formats
^^^^^^^^^^

Command Log (JSONL)::

    {
        "timestamp": 1234567890.123,
        "hand": "right",
        "entry_type": "command",
        "command_type": "move_joints",
        "joint_commands": {
            "th_rot": 30.0,
            "th_mcp": 45.0
        },
        "control_mode": "CASCADED_PID"
    }

Feedback Log (JSONL)::

    {
        "timestamp": 1234567890.123,
        "hand": "right",
        "entry_type": "feedback",
        "joints": {
            "th_rot": {
                "timestamp": 1234567890.123,
                "angle": 30.0,
                "encoder_position": 1000
            }
        },
        "tactile": {
            "th": {
                "timestamp": 1234567890.123,
                "normal_force": 1.5,
                "tangential_force": 0.5
            }
        }
    }

Metadata (JSON)::

    {
        "timestamp": "2024-01-01T12:00:00",
        "statistics": {
            "duration": 60.0,
            "num_commands": {
                "right": 100
            },
            "num_feedback": {
                "right": 500
            }
        },
        "experiment_info": {
            "operator": "user1",
            "conditions": "normal"
        }
    }

Visualization
-----------

Joint Plots
^^^^^^^^^

* Command vs actual joint angles over time
* Tracking error analysis
* Multiple joints overlaid for comparison

Tactile Plots
^^^^^^^^^^^

* Normal and tangential forces over time
* Force vector visualization
* Contact location heatmaps

Notes
-----

Thread Safety
^^^^^^^^^^^

* Uses background thread for file writing
* Thread-safe buffers for data access
* Safe shutdown with data flushing

Memory Management
^^^^^^^^^^^^^^

* In-memory buffers for fast access
* Periodic flushing to disk
* Configurable buffer sizes (coming soon)

Visualization Options
^^^^^^^^^^^^^^^^^

* Real-time plotting (coming soon)
* Customizable plot layouts
* Export to various formats
