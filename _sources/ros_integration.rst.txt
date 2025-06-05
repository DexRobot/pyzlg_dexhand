ROS Integration
==============

The DexHand package provides comprehensive ROS integration supporting both ROS1 and ROS2 environments through a unified API.

Setup and Launch
--------------

Start the ROS node with the default configuration:

.. code-block:: bash

    # Launch the ROS node
    python examples/ros_node/dexhand_ros.py
    
    # For testing, run the demo publisher
    python examples/ros_node/dexhand_ros_publisher_demo.py --hands right --cycle-time 3.0
    
    # Or run the continuous motion publisher
    python examples/ros_node/continuous_joint_publisher.py --pattern sine --amplitude 30 --period 5

Configuration Options
-------------------

The ROS node behavior can be customized through ``config/config.yaml`` under the ``ROS_Node`` section:

+-------------------+--------------------------------------------------+----------------+
| Option            | Description                                      | Default        |
+===================+==================================================+================+
| ``hands``         | Hands to control (left, right, or both)          | ["right"]      |
+-------------------+--------------------------------------------------+----------------+
| ``mode``          | Control mode (impedance_grasp, mit_torque, cascaded_pid, zero_torque, etc.) | "impedance_grasp" |
+-------------------+--------------------------------------------------+----------------+
| ``rate``          | Command send rate in Hz                          | 100.0          |
+-------------------+--------------------------------------------------+----------------+
| ``alpha``         | Filter coefficient (0.0-1.0)                     | 0.1            |
+-------------------+--------------------------------------------------+----------------+
| ``use_broadcast`` | Use efficient broadcast commands                 | true           |
+-------------------+--------------------------------------------------+----------------+
| ``enable_feedback``| Enable feedback publishing                      | false          |
+-------------------+--------------------------------------------------+----------------+
| ``mock``          | Use mock hardware for testing                    | false          |
+-------------------+--------------------------------------------------+----------------+

Topic and Service Interface
------------------------

Topics
^^^^^

+-----------------------------+---------------------------+----------+--------------------------------+
| Topic (default)             | Type                      | Direction| Description                    |
+=============================+===========================+==========+================================+
| /left_hand/joint_commands   | sensor_msgs/JointState    | Input    | Left hand joint commands       |
+-----------------------------+---------------------------+----------+--------------------------------+
| /right_hand/joint_commands  | sensor_msgs/JointState    | Input    | Right hand joint commands      |
+-----------------------------+---------------------------+----------+--------------------------------+
| /left_hand/joint_states     | sensor_msgs/JointState    | Output   | Left hand joint feedback       |
+-----------------------------+---------------------------+----------+--------------------------------+
| /right_hand/joint_states    | sensor_msgs/JointState    | Output   | Right hand joint feedback      |
+-----------------------------+---------------------------+----------+--------------------------------+
| /left_hand/touch_sensors    | Float64MultiArray         | Output   | Left hand touch sensor data    |
+-----------------------------+---------------------------+----------+--------------------------------+
| /right_hand/touch_sensors   | Float64MultiArray         | Output   | Right hand touch sensor data   |
+-----------------------------+---------------------------+----------+--------------------------------+
| /left_hand/motor_feedback   | Float64MultiArray         | Output   | Left hand detailed motor data  |
+-----------------------------+---------------------------+----------+--------------------------------+
| /right_hand/motor_feedback  | Float64MultiArray         | Output   | Right hand detailed motor data |
+-----------------------------+---------------------------+----------+--------------------------------+

Services
^^^^^^

+---------------------+-------------------+--------------------------------+
| Service             | Type              | Description                    |
+=====================+===================+================================+
| /dexhand/reset_hands | std_srvs/Trigger  | Reset hands to default position|
+---------------------+-------------------+--------------------------------+

Feedback Data Structure
---------------------

Touch Sensor Data
^^^^^^^^^^^^^^

Touch sensor data is published as ``Float64MultiArray`` with 40 values (8 values for each of the 5 fingers):

* Format per finger: ``[timestamp, normal_force, normal_force_delta, tangential_force, tangential_force_delta, direction, proximity, temperature]``
* Direction is in degrees (0-359, fingertip is 0°) or -1 if invalid
* Finger mapping:
  * Index 0: thumb (th)
  * Index 1: index finger (ff)
  * Index 2: middle finger (mf)
  * Index 3: ring finger (rf)
  * Index 4: little finger (lf)

Motor Feedback Data
^^^^^^^^^^^^^^^

Motor feedback is published as ``Float64MultiArray`` with 84 values (7 values for each of the 12 motors):

* Format per motor: ``[timestamp, angle, encoder_position, current, velocity, error_code, impedance]``
* Impedance values: lower values indicate higher resistance to movement
* Motor ordering follows joint names: th_dip, th_mcp, th_rot, ff_spr, etc.

Example Usage
-----------

Publishing Joint Commands
^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

    # Simple ROS1/ROS2 agnostic publisher
    from ros_compat import create_ros_node, Publisher
    from sensor_msgs.msg import JointState
    
    # Create node and publisher
    node = create_ros_node('hand_command_publisher')
    pub = Publisher('/right_hand/joint_commands', JointState, queue_size=10)
    
    # Create message
    msg = JointState()
    msg.name = ['r_f_joint1_2', 'r_f_joint2_2']  # URDF joint names
    msg.position = [0.5, 0.5]  # Radians
    
    # Publish
    msg.header.stamp = node.get_time()
    pub.publish(msg)