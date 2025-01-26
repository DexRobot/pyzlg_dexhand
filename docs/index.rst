DexHand Python Hardware Interface
=================================

Python interface for controlling dexterous robotic hands over CANFD using ZLG USBCANFD adapters. This package provides both direct control and ROS integration capabilities.

Features
--------

* CANFD communication interface for DexHand hardware
* Joint-space control interface with feedback processing
* Built-in data logging and visualization tools
* ROS integration with both ROS1 and ROS2 support
* Hardware testing utilities

Documentation Contents
--------------------

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   quickstart
   api/index

Getting Started
-------------

For new users:

1. Check the :doc:`quickstart` guide for installation and basic usage
2. Review the API documentation starting with :doc:`api/dexhand_interface`
3. Explore example applications in the Github repository

Prerequisites
------------

* Linux environment
* Python 3.8+
* ZLG USBCANFD adapter (tested with USBCANFD-200U)
* ROS1/ROS2 (optional, for ROS integration)

Installation
-----------

Install via pip::

    pip install -e .

Configure USB permissions::

    sudo ./tools/setup_usb_can.sh

Next Steps
---------

* Try the interactive testing interface::

    python tools/hardware_test/test_dexhand_interactive.py --hands right

* Experiment with ROS integration::

    python examples/ros_node/dexhand_ros.py

* Check out the :doc:`api/dexhand_logger` for data collection and analysis

Support
-------

For issues, questions, or contributions:

* Check our `GitHub Issues <https://github.com/dexrobot/pyzlg_dexhand/issues>`_
* Submit pull requests via GitHub
