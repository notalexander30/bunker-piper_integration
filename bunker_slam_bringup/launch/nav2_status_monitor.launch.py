"""Colored terminal monitor for Nav2, self-exploration and Bunker motion state."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('refresh_hz', default_value='2.0'),
        Node(
            package='bunker_autonomy',
            executable='nav2_status_monitor',
            name='nav2_status_monitor',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'refresh_hz': LaunchConfiguration('refresh_hz'),
            }],
        ),
    ])
