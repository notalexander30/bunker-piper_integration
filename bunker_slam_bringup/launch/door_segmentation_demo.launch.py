"""Full stationary door-segmentation demo: RTAB-Map + SQLite memory only."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_rtabmap_database():
    prefix = Path(get_package_prefix('bunker_slam_bringup'))
    workspace = prefix.parent.parent if prefix.parent.name == 'install' else Path('/ros2_ws')
    return str(workspace / 'src' / 'bunker_slam_bringup' / 'maps' / 'bunker_acceptance.db')


def generate_launch_description():
    bringup = get_package_share_directory('bunker_slam_bringup')
    follower = get_package_share_directory('bunker_object_follower')
    localization = os.path.join(bringup, 'launch', 'localization_bringup.launch.py')
    memory = os.path.join(follower, 'launch', 'door_segmentation_memory.launch.py')
    rviz_config = os.path.join(bringup, 'rviz', 'door_segmentation_demo.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('rtabmap_database_path', default_value=_default_rtabmap_database()),
        DeclareLaunchArgument('door_database_path', default_value='/root/vlm_results/door_objects.sqlite3'),
        DeclareLaunchArgument('weights_path', default_value='/ros2_ws/models/door-seg.pt'),
        DeclareLaunchArgument('detector_device', default_value='0'),
        DeclareLaunchArgument('bunker_port', default_value='can3'),
        DeclareLaunchArgument('arm_can_port', default_value='can2'),
        DeclareLaunchArgument('serial_no', default_value='261222075829'),
        DeclareLaunchArgument('configure_can', default_value='false'),
        DeclareLaunchArgument('imu_gate_timeout', default_value='30.0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        LogInfo(msg='Door segmentation demo: RTAB-Map localization + door SQLite memory. Door perception starts no controller, avoidance planner, or command mux.'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(localization), launch_arguments={
            'database_path': LaunchConfiguration('rtabmap_database_path'),
            'bunker_port': LaunchConfiguration('bunker_port'),
            'arm_can_port': LaunchConfiguration('arm_can_port'),
            'serial_no': LaunchConfiguration('serial_no'),
            'configure_can': LaunchConfiguration('configure_can'),
            'start_base': 'true', 'start_arm': 'true', 'start_camera': 'true',
            'use_rviz': 'false', 'imu_gate_timeout': LaunchConfiguration('imu_gate_timeout'),
        }.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(memory), launch_arguments={
            'weights_path': LaunchConfiguration('weights_path'),
            'database_path': LaunchConfiguration('door_database_path'),
            'detector_device': LaunchConfiguration('detector_device'),
        }.items()),
        Node(
            package='rviz2', executable='rviz2', name='door_segmentation_rviz',
            output='screen', arguments=['-d', rviz_config],
            additional_env={'QT_X11_NO_MITSHM': '1'},
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
