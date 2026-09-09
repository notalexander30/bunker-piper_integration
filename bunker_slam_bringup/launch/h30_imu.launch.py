# Copyright 2026 Dase Orin
"""Start the WHEELTEC/YESENSE H30 UART driver without fusing it into the EKF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('bunker_slam_bringup'), 'config', 'h30_imu.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value=''),
        DeclareLaunchArgument('baud_rate', default_value='460800'),
        DeclareLaunchArgument('frame_id', default_value='imu_link'),
        Node(
            package='yesense_std_ros2',
            executable='yesense_node_publisher',
            namespace='yesense',
            name='yesense_pub',
            output='screen',
            parameters=[config_path, {
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'driver_type': 'ros_serial',
            }],
        ),
    ])
