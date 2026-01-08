import unittest
import os
import launch
import launch_ros
import launch_testing
import pytest

@pytest.mark.launch_test
def generate_test_description():
    container = launch_ros.actions.ComposableNodeContainer(
        name='ac_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package='ac_driver',
                plugin='robosense::ac::MSPublisher',
                name='ms_node',
            )
        ],
        output='screen',
    )

    return launch.LaunchDescription([
        container,
        launch_testing.actions.ReadyToTest(),
    ])

class TestMSNodeLink(unittest.TestCase):
    def test_node_active(self, proc_info, proc_output):
        # This is a simple check if the container starts correctly
        # In a real environment with hardware, we would check topics
        proc_output.assert_stdout_contains('ac_container', timeout=10)
