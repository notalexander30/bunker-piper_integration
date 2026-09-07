"""Bunker localization + Open Images door YOLO + JSON semantic memory.

This launch starts no exploration/controller node and publishes no movement
command. It is the perception-and-memory prerequisite for a supervised demo.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_database_path():
    prefix = Path(get_package_prefix('bunker_slam_bringup'))
    workspace = prefix.parent.parent if prefix.parent.name == 'install' else Path('/ros2_ws')
    return str(workspace / 'src' / 'bunker_slam_bringup' / 'maps' / 'bunker_acceptance.db')


def generate_launch_description():
    bringup = get_package_share_directory('bunker_slam_bringup')
    follower = get_package_share_directory('bunker_object_follower')
    semantic = get_package_share_directory('semantic_memory')
    config = os.path.join(follower, 'config', 'semantic_memory_door_d435i.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('database_path', default_value=_default_database_path()),
        DeclareLaunchArgument('memory_file', default_value='/root/vlm_results/door_semantic_memory.json'),
        DeclareLaunchArgument('weights_path', default_value='/ros2_ws/models/yolov8s-oiv7.pt'),
        DeclareLaunchArgument('detector_device', default_value='0'),
        DeclareLaunchArgument('bunker_port', default_value='can3'),
        DeclareLaunchArgument('arm_can_port', default_value='can2'),
        DeclareLaunchArgument('serial_no', default_value='261222075829'),
        DeclareLaunchArgument('configure_can', default_value='false'),
        DeclareLaunchArgument('imu_gate_timeout', default_value='30.0'),
        LogInfo(
            msg='Door semantic demo: base/camera localization + YOLO Door + '
            'map-frame JSON. No forward-exploration or cmd_vel command node is started.'
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            bringup, 'launch', 'localization_bringup.launch.py')), launch_arguments={
            'database_path': LaunchConfiguration('database_path'),
            'bunker_port': LaunchConfiguration('bunker_port'),
            'arm_can_port': LaunchConfiguration('arm_can_port'),
            'serial_no': LaunchConfiguration('serial_no'),
            'configure_can': LaunchConfiguration('configure_can'),
            'start_base': 'true', 'start_arm': 'true', 'start_camera': 'true',
            'use_rviz': 'false', 'imu_gate_timeout': LaunchConfiguration('imu_gate_timeout'),
        }.items()),
        Node(package='bunker_object_follower', executable='detector_tracker_node',
             name='detector_tracker_node', output='screen', emulate_tty=True,
             parameters=[config, {
                 'weights_path': LaunchConfiguration('weights_path'),
                 'device': ParameterValue(LaunchConfiguration('detector_device'), value_type=str),
             }]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            semantic, 'launch', 'semantic_memory.launch.py')), launch_arguments={
            'params_file': config, 'memory_file': LaunchConfiguration('memory_file'),
            'rtabmap_database_path': LaunchConfiguration('database_path'),
            'yolo_detection_topic': '/door_search/local_target_status',
        }.items()),
    ])
