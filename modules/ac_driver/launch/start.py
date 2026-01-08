from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer, LifecycleNode
from launch_ros.descriptions import ComposableNode
from launch_ros.events.lifecycle import ChangeState
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    rviz_config_file = os.path.join(
        get_package_share_directory('ac_driver'),
        'rviz',
        'rviz2_config.rviz'
    )

    ms_node = LifecycleNode(
        package='ac_driver',
        plugin='robosense::ac::MSPublisher',
        name='ms_node',
        namespace='',
        parameters=[{
            'image_input_fps': 30,
            'imu_input_fps': 200,
            'enable_jpeg': False,
            'jpeg_quality': 70
        }],
        output='screen',
    )

    codec_node = LifecycleNode(
        package='ac_codec',
        plugin='robosense::ac::CodecPublisher',
        name='codec_node',
        namespace='',
        output='screen',
    )

    # Use ComposableNodeContainer for zero-copy
    container = ComposableNodeContainer(
        name='ac_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ac_driver',
                plugin='robosense::ac::MSPublisher',
                name='ms_node',
                parameters=[{
                    'image_input_fps': 30,
                    'imu_input_fps': 200,
                    'enable_jpeg': False,
                    'jpeg_quality': 70
                }]
            ),
            ComposableNode(
                package='ac_codec',
                plugin='robosense::ac::CodecPublisher',
                name='codec_node',
            )
        ],
        output='screen',
    )

    # Note: ComposableNode does not natively support lifecycle management in the same way LifecycleNode does 
    # when defined as ComposableNode. However, since we refactored the classes themselves, 
    # we can use the lifecycle tools to manage them once they are up.
    # To use the lifecycle-aware launch features, we would normally use LifecycleNode.
    # But for zero-copy, we need them in a container.

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        container,
        rviz2_node
    ])
