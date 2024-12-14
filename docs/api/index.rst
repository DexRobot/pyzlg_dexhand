API Reference
============

This section provides detailed API documentation for each module in the DexHand Python Interface.

.. toctree::
   :maxdepth: 2

   dexhand_interface
   dexhand_protocol
   zcan_wrapper
   dexhand_logger

Core Components
-------------

:doc:`dexhand_interface`
    High-level interface for hand control including ``DexHandBase``, ``LeftDexHand``, ``RightDexHand`` and associated data structures

:doc:`dexhand_protocol`
    Protocol implementation including command generation, message parsing, and protocol definitions

:doc:`zcan_wrapper`
    Hardware communication layer providing both real and mock implementations for ZCAN devices

:doc:`dexhand_logger`
    Data logging and visualization utilities
