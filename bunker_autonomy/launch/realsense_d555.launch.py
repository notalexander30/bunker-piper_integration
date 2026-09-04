import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_rviz = LaunchConfiguration('launch_rviz')
    config_path = os.path.join(
        get_package_share_directory('bunker_autonomy'),
        'config',
        'realsense_d555.yaml',
    )

    with open(config_path, encoding='utf-8') as config_file:
        camera_parameters = yaml.safe_load(config_file)

    rviz_config_path = os.path.join(
        get_package_share_directory('bunker_autonomy'),
        'config',
        'realsense_d555.rviz',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Start RViz with the lightweight RealSense profile.',
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='camera',
            name='camera',
            parameters=[camera_parameters],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='realsense_rviz2',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(launch_rviz),
            output='screen',
        ),
    ])
