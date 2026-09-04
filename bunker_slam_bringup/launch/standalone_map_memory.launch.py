"""Standalone RTAB-Map, map-frame semantic memory, and optional RViz."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _workspace_path(*parts):
    prefix = Path(get_package_prefix('bunker_slam_bringup'))
    workspace = prefix.parent.parent if prefix.parent.name == 'install' else Path('/ros2_ws')
    return str(workspace.joinpath(*parts))


def generate_launch_description():
    slam_share = get_package_share_directory('bunker_slam_bringup')
    detector_config = os.path.join(
        get_package_share_directory('bunker_object_follower'),
        'config',
        'standalone_door_rgbd.yaml',
    )
    mapping_launch = os.path.join(
        slam_share, 'launch', 'minimal_rgbd_mapping.launch.py'
    )
    semantic_launch = os.path.join(
        get_package_share_directory('semantic_memory'),
        'launch',
        'semantic_memory.launch.py',
    )
    rviz_config = os.path.join(
        slam_share, 'rviz', 'standalone_door_rgbd_mapping.rviz'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'database_path',
            default_value=_workspace_path('vlm_results', 'maps', 'handheld_door_rgbd.db'),
        ),
        DeclareLaunchArgument(
            'memory_file',
            default_value=_workspace_path('vlm_results', 'semantic', 'handheld_door_memory.json'),
        ),
        DeclareLaunchArgument('reset_database', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
        LogInfo(msg=(
            'Standalone map/memory: RTAB-Map + semantic memory + optional RViz. '
            'Camera/odometry and door YOLO must be supplied by terminals 1 and 3.'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch),
            launch_arguments={
                'database_path': LaunchConfiguration('database_path'),
                'reset_database': LaunchConfiguration('reset_database'),
                'start_rgbd_odometry': 'false',
                'start_rgbd_sync': 'false',
                'start_live_cloud': 'false',
                'start_rtabmap': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(semantic_launch),
            launch_arguments={
                'params_file': detector_config,
                'memory_file': LaunchConfiguration('memory_file'),
                'rtabmap_database_path': LaunchConfiguration('database_path'),
                'yolo_detection_topic': '/door_search/local_target_status',
                'stationary_test_map': 'false',
            }.items(),
        ),
        Node(
            package='rviz2', executable='rviz2', name='standalone_yolo_mapping_rviz',
            output='screen', arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            additional_env={
                'DISPLAY': os.environ.get('DISPLAY', ':1'),
                'XAUTHORITY': os.environ.get('XAUTHORITY', '/tmp/.docker.xauth'),
                'QT_QPA_PLATFORM': 'xcb',
                'QT_X11_NO_MITSHM': '1',
                'XDG_RUNTIME_DIR': os.environ.get('XDG_RUNTIME_DIR', '/tmp/xdg-runtime'),
            },
        ),
    ])
