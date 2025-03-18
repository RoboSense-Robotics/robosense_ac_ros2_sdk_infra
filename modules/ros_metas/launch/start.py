from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    rviz_config_file = os.path.join(
        get_package_share_directory('metas_ros'),
        'rviz',
        'rviz2_config.rviz'
    )
        
    return LaunchDescription([

        Node(
            package='metas_ros',
            executable='ms_node',
            name='ms_node',
            output='screen'
        ),
        # Node(
        #     package='metas_ros',
        #     executable='point_tf_node',
        #     name='point_tf_node',
        #     output='screen'
        # ),

        # Node(
        #     package='imu_filter_madgwick',
        #     executable='imu_filter_madgwick_node',
        #     name='imu_filter_madgwick',
        #     output='screen',
        #     parameters=[{
        #         'use_mag': False,
        #         'world_frame': 'enu',
        #         'publish_tf': True,
        #         'frequency': 50.0,
        #         'gain': 0.1,
        #     }],
        #     remappings=[
        #         ('/imu/data_raw', '/rs_imu'),
        #     ]
        # ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])
