ZCAN Wrapper
===========

The ZCAN wrapper module provides low-level hardware communication with ZLG USBCANFD devices.

Core Components
-------------

Device Types
^^^^^^^^^^

.. autoclass:: pyzlg_dexhand.zcan.ZCANDeviceType
    :members:
    :undoc-members:

Status Codes
^^^^^^^^^^

.. autoclass:: pyzlg_dexhand.zcan.ZCANStatus
    :members:
    :undoc-members:

Message Types
^^^^^^^^^^^

.. autoclass:: pyzlg_dexhand.zcan.ZCANMessageType
    :members:
    :undoc-members:

Base Wrapper
----------

.. autoclass:: pyzlg_dexhand.zcan_wrapper.ZCANWrapperBase
    :members:
    :special-members: __init__

Hardware Implementation
--------------------

.. autoclass:: pyzlg_dexhand.zcan_wrapper.ZCANWrapper
    :members:
    :show-inheritance:
    :special-members: __init__

Mock Implementation
----------------

.. autoclass:: pyzlg_dexhand.zcan_wrapper.MockZCANWrapper
    :members:
    :show-inheritance:
    :special-members: __init__

Configuration
-----------

BitTimingConfig
^^^^^^^^^^^^^

.. autoclass:: pyzlg_dexhand.zcan_wrapper.BitTimingConfig
    :members:
    :undoc-members:

CANFDTimingConfig
^^^^^^^^^^^^^^

.. autoclass:: pyzlg_dexhand.zcan_wrapper.CANFDTimingConfig
    :members:
    :undoc-members:

Filter Configuration
^^^^^^^^^^^^^^^^^

.. autoclass:: pyzlg_dexhand.zcan_wrapper.ZCANFilterConfig
    :members:
    :undoc-members:

Examples
-------

Basic Usage
^^^^^^^^^

.. code-block:: python

    from pyzlg_dexhand.zcan_wrapper import ZCANWrapper, ZCANDeviceType

    # Initialize wrapper
    zcan = ZCANWrapper()
    zcan.open(device_type=ZCANDeviceType.ZCAN_USBCANFD_200U)

    # Configure channel
    zcan.configure_channel(
        channel=0,
        arb_baudrate=1000000,  # 1Mbps
        data_baudrate=5000000  # 5Mbps
    )

    # Send message
    zcan.send_fd_message(
        channel=0,
        id=0x101,
        data=b'\x44\x03\x00\x00\x00\x00'  # Example command
    )

    # Receive messages
    messages = zcan.receive_fd_messages(
        channel=0,
        max_messages=10,
        timeout_ms=100
    )
    for msg_id, data, timestamp in messages:
        print(f"Message ID: {msg_id:x}, Data: {data.hex()}")

    # Clean up
    zcan.close()

Message Filtering
^^^^^^^^^^^^^^

.. code-block:: python

    # Configure message filters
    filters = [
        ZCANFilterConfig(
            type=0,  # Standard frame
            start_id=0x100,
            end_id=0x1FF
        ),
        ZCANFilterConfig(
            type=0,
            start_id=0x600,
            end_id=0x6FF
        )
    ]
    zcan.set_filter(channel=0, filters=filters)

Error Handling
^^^^^^^^^^^

.. code-block:: python

    # Monitor channel status
    if not zcan.monitor_channel_status(channel=0):
        # Handle error condition
        zcan.handle_error(channel=0)

    # Reset channel on severe errors
    zcan.reset_channel(channel=0)

Notes
-----

Device Setup
^^^^^^^^^^

1. Initialize ZCAN device with appropriate type
2. Configure channel timing parameters
3. Set up message filters if needed
4. Monitor channel status during operation

Timing Configuration
^^^^^^^^^^^^^^^^^

Supported baudrate combinations:

* Arbitration phase: 1Mbps
* Data phase: 5Mbps

Default timing parameters:

* Arbitration phase:
   * TSEG1: 14
   * TSEG2: 3
   * SJW: 3
   * BRP: 2

* Data phase:
   * TSEG1: 1
   * TSEG2: 0
   * SJW: 0
   * BRP: 2

Error Recovery
^^^^^^^^^^^

1. Monitor channel status
2. Read error information when detected
3. Attempt channel reset if necessary
4. Reconfigure channel after reset
