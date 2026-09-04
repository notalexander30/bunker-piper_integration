import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('bunker_autonomy')
    base_share = get_package_share_directory('bunker_base')
    follower_share = get_package_share_directory('bunker_object_follower')
    config = os.path.join(share, 'config', 'door_search_d435i.yaml')
    follower_config = os.path.join(follower_share, 'config', 'door_follower_d435i.yaml')
    rviz_config = os.path.join(follower_share, 'rviz', 'door_follower_d435i.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='dry_run'),
        DeclareLaunchArgument('launch_chassis', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('detector_device', default_value='0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(base_share, 'launch', 'bunker_base.launch.py')),
          condition=IfCondition(LaunchConfiguration('launch_chassis')),
          launch_arguments={'port_name': 'can3', 'is_bunker_mini': 'true'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'realsense_d435i.launch.py')),
          # The door mission uses its own YOLO/depth RViz layout below.
          launch_arguments={'camera_rviz': 'false'}.items()),
        # Dedicated YOLO-only launch. It can also be started independently for
        # one-component-per-terminal debugging.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                follower_share, 'launch', 'door_yolo_d435i.launch.py'
            )),
            launch_arguments={
                'detector_device': LaunchConfiguration('detector_device'),
            }.items(),
        ),
        Node(
            package='bunker_object_follower',
            executable='target_mux_node',
            name='door_target_mux',
            output='screen',
            parameters=[
                follower_config,
                {
                    'local_target_topic': '/door_search/local_target_status',
                    'output_target_topic': '/target_status',
                    'output_detected_topic': '/target_detected',
                    'semantic_fallback_enabled': False,
                },
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='door_yolo_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'find_trash_can_demo.launch.py')),
          launch_arguments={'config_file': config, 'mode': LaunchConfiguration('mode'),
                            'navigation_mode': 'depth_only',
                            'depth_image_topic': '/camera/camera/aligned_depth_to_color/image_raw',
                            'launch_vlm_monitor': 'false'}.items()),
    ])
