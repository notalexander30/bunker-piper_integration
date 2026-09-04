from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = Path(
        get_package_share_directory('piper_x_aruco_wall_approach')
    ) / 'config' / 'wall_approach.yaml'

    return LaunchDescription([
        DeclareLaunchArgument('execute', default_value='false'),
        DeclareLaunchArgument('clearance', default_value='0.05'),
        DeclareLaunchArgument('prefer_elbow_motion', default_value='true'),
        DeclareLaunchArgument('goal_orientation_tolerance', default_value='0.35'),
        DeclareLaunchArgument(
            'point_cloud_topic',
            default_value='/front_camera/depth/color/points',
        ),
        DeclareLaunchArgument('tool_roll', default_value='0.0'),
        Node(
            package='piper_x_aruco_wall_approach',
            executable='wall_approach_node',
            name='wall_approach_node',
            output='screen',
            parameters=[
                str(config_file),
                {
                    'execute': LaunchConfiguration('execute'),
                    'clearance': LaunchConfiguration('clearance'),
                    'point_cloud_topic': LaunchConfiguration('point_cloud_topic'),
                    'tool_roll': LaunchConfiguration('tool_roll'),
                    'prefer_elbow_motion': LaunchConfiguration('prefer_elbow_motion'),
                    'goal_orientation_tolerance': LaunchConfiguration(
                        'goal_orientation_tolerance'
                    ),
                }
            ],
        ),
    ])
