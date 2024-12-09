import os
import sys
import pytest

# Add the project root directory to the Python path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, project_root)

# Disable ROS launch testing for unit tests
def pytest_configure(config):
    # Remove the launch_testing plugin if it's present
    plugin = config.pluginmanager.get_plugin('launch_testing')
    if plugin:
        config.pluginmanager.unregister(plugin)
