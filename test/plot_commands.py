# %% Import libraries
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import numpy as np

# %% Define bag reading function
def read_joint_states(bag_path):
    joint_data = {}
    timestamps = []

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/joint_states':
            msg_type = get_message('sensor_msgs/msg/JointState')
            joint_state = deserialize_message(data, msg_type)

            timestamps.append(t)
            for i, name in enumerate(joint_state.name):
                if name not in joint_data:
                    joint_data[name] = []
                joint_data[name].append(joint_state.position[i])

    return timestamps, joint_data

# %% Read bag data
bag_path = 'data/rosbag2_2024_11_11-23_20_45'
timestamps, joint_data = read_joint_states(bag_path)

# %% Filter joint data with name starting from r_
joint_data_filtered = {name: positions for name, positions in joint_data.items() if name.startswith('r_')}

# %% Create plot
# Calculate grid dimensions for 20 subplots
n_cols = 4  # 4 columns
n_rows = 5  # 5 rows to accommodate 20 joints

# Create figure with subplots
plt.figure(figsize=(20, 16))

# Plot each joint in its own subplot
for idx, (joint_name, positions) in enumerate(joint_data_filtered.items(), 1):
    plt.subplot(n_rows, n_cols, idx)
    time_sec = [(t - timestamps[0])/1e9 for t in timestamps]
    plt.plot(time_sec, positions)

    # Add labels and grid for each subplot
    plt.title(joint_name)
    plt.xlabel('Time (s)')
    plt.ylabel('Position (rad)')
    plt.grid(True)

# Adjust layout to prevent overlap
plt.tight_layout()
plt.show()

# %%
