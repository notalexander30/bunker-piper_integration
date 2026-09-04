"""AMCL localization over a saved 2D map.

Do not run this at the same time as RTAB-Map localization if both publish
map -> odom.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('bunker_slam_bringup'),
        'config',
        'amcl_nav2_params.yaml',
    ])
    lifecycle_nodes = ['map_server', 'amcl']
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='/ros2_ws/maps/map.yaml'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'yaml_filename': LaunchConfiguration('map'),
            }],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'scan_topic': LaunchConfiguration('scan_topic'),
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': lifecycle_nodes,
            }],
        ),
    ])
