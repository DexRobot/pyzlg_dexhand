DexHand Protocol
==============

The DexHand protocol implements the CANFD communication layer between host and hand hardware.

Core Protocol
-----------

Message Types
^^^^^^^^^^^

.. autoclass:: pyzlg_dexhand.dexhand_protocol.MessageType
    :members:
    :undoc-members:

Board IDs
^^^^^^^^

.. autoclass:: pyzlg_dexhand.dexhand_protocol.BoardID
    :members:
    :undoc-members:

Commands
-------

Command Types
^^^^^^^^^^^

.. autoclass:: pyzlg_dexhand.dexhand_protocol.commands.CommandType
    :members:
    :undoc-members:

Control Modes
^^^^^^^^^^^

.. autoclass:: pyzlg_dexhand.dexhand_protocol.commands.ControlMode
    :members:
    :undoc-members:

Feedback Modes
^^^^^^^^^^^^

.. autoclass:: pyzlg_dexhand.dexhand_protocol.commands.FeedbackMode
    :members:
    :undoc-members:

Command Classes
^^^^^^^^^^^^

.. autoclass:: pyzlg_dexhand.dexhand_protocol.commands.MotorCommand
    :members:
    :undoc-members:

.. autoclass:: pyzlg_dexhand.dexhand_protocol.commands.ClearErrorCommand
    :members:
    :undoc-members:

.. autoclass:: pyzlg_dexhand.dexhand_protocol.commands.FeedbackConfigCommand
    :members:
    :undoc-members:

Command Encoding
^^^^^^^^^^^^^

.. autofunction:: pyzlg_dexhand.dexhand_protocol.commands.encode_command

Messages
-------

Error Types
^^^^^^^^^

.. autoclass:: pyzlg_dexhand.dexhand_protocol.messages.BoardError
    :members:
    :undoc-members:

.. autoclass:: pyzlg_dexhand.dexhand_protocol.messages.ErrorCode
    :members:
    :undoc-members:

Feedback Classes
^^^^^^^^^^^^^

.. autoclass:: pyzlg_dexhand.dexhand_protocol.messages.MotorFeedback
    :members:
    :undoc-members:

.. autoclass:: pyzlg_dexhand.dexhand_protocol.messages.TactileFeedback
    :members:
    :undoc-members:

.. autoclass:: pyzlg_dexhand.dexhand_protocol.messages.BoardFeedback
    :members:
    :undoc-members:

.. autoclass:: pyzlg_dexhand.dexhand_protocol.messages.ErrorInfo
    :members:
    :undoc-members:

Message Processing
^^^^^^^^^^^^^^^

.. autoclass:: pyzlg_dexhand.dexhand_protocol.messages.ProcessedMessage
    :members:
    :undoc-members:

.. autofunction:: pyzlg_dexhand.dexhand_protocol.messages.process_message

Protocol Specification
-------------------

Message Structure
^^^^^^^^^^^^^^^

The protocol uses standard CANFD frames with extended identifiers. Message IDs are constructed as:

* Base message type (e.g., 0x100 for motion commands)
* Board ID offset (0x01-0x06 for left hand, 0x07-0x0C for right hand)

Command Format
^^^^^^^^^^^^

Motor Command::

    Byte 0: Control mode
    Byte 1: Motor enable flags (0x01=motor1, 0x02=motor2, 0x03=both)
    Byte 2-3: Motor 1 position (little endian)
    Byte 4-5: Motor 2 position (little endian)

Clear Error Command::

    Byte 0: 0x03 (Command prefix)
    Byte 1: 0xA4 (Clear error command)

Feedback Config Command::

    Byte 0: 0x03 (Command prefix)
    Byte 1: 0x74 (Feedback config command)
    Byte 2: Feedback mode
    Byte 3: Period (10ms units)
    Byte 4: Enable flag

Example Usage
-----------

Command Generation
^^^^^^^^^^^^^^^

.. code-block:: python

    from pyzlg_dexhand.dexhand_protocol.commands import (
        MotorCommand,
        ControlMode,
        encode_command
    )

    # Create motor command
    command = MotorCommand(
        control_mode=ControlMode.CASCADED_PID,
        motor_enable=0x03,  # Both motors enabled
        motor1_pos=1000,
        motor2_pos=-2000
    )

    # Encode for transmission
    msg_type, data = encode_command(command)

Message Processing
^^^^^^^^^^^^^^^

.. code-block:: python

    from pyzlg_dexhand.dexhand_protocol.messages import process_message

    # Process received message
    can_id = 0x181  # Example: Board 1 feedback
    data = b'...'   # Raw message data

    result = process_message(can_id, data)

    if result.msg_type == MessageType.MOTION_FEEDBACK:
        print(f"Motor 1 position: {result.feedback.motor1.position}")
        print(f"Motor 2 position: {result.feedback.motor2.position}")
    elif result.msg_type == MessageType.ERROR_MESSAGE:
        print(f"Error: {result.error.description}")

Notes
-----

Message Flow
^^^^^^^^^^

1. Host sends commands using appropriate message type
2. Hand processes command and returns feedback/response
3. Any errors are reported via error messages

Error Handling
^^^^^^^^^^^^

1. Check message type from CAN ID
2. Process messages according to type
3. Handle errors by:
   - Parsing error information
   - Clearing errors when safe
   - Resuming normal operation
