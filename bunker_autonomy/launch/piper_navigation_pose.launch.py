import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('bunker_autonomy'), 'config', 'piper_navigation_pose.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('trigger_on_start', default_value='false'),
        DeclareLaunchArgument('allow_motion', default_value='false'),
        Node(
            package='bunker_autonomy', executable='piper_navigation_pose',
            name='piper_navigation_pose', output='screen',
            parameters=[config, {
                'trigger_on_start': LaunchConfiguration('trigger_on_start'),
                'allow_motion': LaunchConfiguration('allow_motion'),
            }],
        ),
    ])
