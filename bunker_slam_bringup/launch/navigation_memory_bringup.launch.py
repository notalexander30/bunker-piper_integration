"""RTAB-Map SLAM/localization with YOLO and geometric semantic memory.

The camera publishes RGB and aligned depth. RTAB-Map builds or localizes in the
metric map, while YOLO/depth observations are stored as labelled 3-D objects in
that same ``map`` frame. A second depth-only camera-frame memory is retained for
geometric debugging. No controller, command mux, or cmd_vel publisher starts.
"""

import os
from pathlib import Path

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def default_database_path():
    prefix = Path(get_package_prefix('bunker_slam_bringup'))
    workspace = prefix.parent.parent if prefix.parent.name == 'install' else Path('/ros2_ws')
    return str(workspace / 'src' / 'bunker_slam_bringup' / 'maps' / 'bunker_rgbd.db')


def generate_launch_description():
    bringup_share = get_package_share_directory('bunker_slam_bringup')
    follower_share = get_package_share_directory('bunker_object_follower')
    semantic_share = get_package_share_directory('semantic_memory')

    slam_launch = os.path.join(
        bringup_share, 'launch', 'slam_bringup.launch.py')
    semantic_launch = os.path.join(
        semantic_share, 'launch', 'semantic_memory.launch.py')
    detector_config = os.path.join(
        follower_share, 'config', 'semantic_memory_door_d435i.yaml')
    rviz_config = os.path.join(
        bringup_share, 'rviz', 'semantic_yolo_demo.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping',
                              choices=['mapping', 'localization']),
        DeclareLaunchArgument('database_path', default_value=default_database_path()),
        DeclareLaunchArgument('reset_database', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('bunker_port', default_value='can3'),
        DeclareLaunchArgument('arm_can_port', default_value='can2'),
        DeclareLaunchArgument('serial_no', default_value='261222075829'),
        DeclareLaunchArgument('configure_can', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('start_base', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('start_arm', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('start_camera', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('imu_gate_timeout', default_value='30.0'),
        DeclareLaunchArgument('detector_config', default_value=detector_config),
        DeclareLaunchArgument('weights_path',
                              default_value='/ros2_ws/models/yolov8s-oiv7.pt'),
        DeclareLaunchArgument('detector_device', default_value='0'),
        DeclareLaunchArgument(
            'semantic_memory_file',
            default_value='/root/vlm_results/navigation_door_semantic_memory.json'),
        DeclareLaunchArgument(
            'geometry_memory_file',
            default_value='/root/vlm_results/navigation_depth_geometry_memory.json'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument(
            'arm_mount_xyz', default_value='0.807474630 0.0 0.545498689'),
        DeclareLaunchArgument(
            'arm_mount_rpy', default_value='-1.483492816 0.029092513 1.573342313'),
        DeclareLaunchArgument(
            'camera_mount_xyz', default_value='-0.0106 -0.0175 -0.0125'),
        DeclareLaunchArgument('camera_mount_rpy', default_value='0.0 0.0 0.0'),
        LogInfo(msg=(
            'Navigation memory: RGB + aligned depth + YOLO + map-frame '
            'geometric memory + RTAB-Map. No controller or cmd_vel publisher.')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'mode': LaunchConfiguration('mode'),
                'database_path': LaunchConfiguration('database_path'),
                'reset_database': LaunchConfiguration('reset_database'),
                'bunker_port': LaunchConfiguration('bunker_port'),
                'arm_can_port': LaunchConfiguration('arm_can_port'),
                'serial_no': LaunchConfiguration('serial_no'),
                'enable_pointcloud': 'false',
                'configure_can': LaunchConfiguration('configure_can'),
                'start_base': LaunchConfiguration('start_base'),
                'start_arm': LaunchConfiguration('start_arm'),
                'start_camera': LaunchConfiguration('start_camera'),
                'use_rviz': 'false',
                'imu_gate_timeout': LaunchConfiguration('imu_gate_timeout'),
                'arm_mapping_pose': 'false',
                'arm_mount_xyz': LaunchConfiguration('arm_mount_xyz'),
                'arm_mount_rpy': LaunchConfiguration('arm_mount_rpy'),
                'camera_mount_xyz': LaunchConfiguration('camera_mount_xyz'),
                'camera_mount_rpy': LaunchConfiguration('camera_mount_rpy'),
            }.items(),
        ),
        Node(
            package='bunker_object_follower',
            executable='detector_tracker_node',
            name='detector_tracker_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('detector_config'),
                {
                    'weights_path': LaunchConfiguration('weights_path'),
                    'device': ParameterValue(
                        LaunchConfiguration('detector_device'), value_type=str),
                },
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(semantic_launch),
            launch_arguments={
                'params_file': LaunchConfiguration('detector_config'),
                'memory_file': LaunchConfiguration('semantic_memory_file'),
                'rtabmap_database_path': LaunchConfiguration('database_path'),
                'yolo_detection_topic': '/door_search/local_target_status',
            }.items(),
        ),
        Node(
            package='bunker_object_follower',
            executable='depth_geometry_memory_node',
            name='depth_geometry_memory_node',
            output='screen',
            parameters=[{
                'memory_file': LaunchConfiguration('geometry_memory_file'),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='navigation_memory_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            additional_env={'QT_X11_NO_MITSHM': '1'},
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
