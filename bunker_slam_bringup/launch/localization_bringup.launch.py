# Copyright 2026 Dase Orin
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

import os
from pathlib import Path

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def default_database_path():
    prefix = Path(get_package_prefix('bunker_slam_bringup'))
    if prefix.parent.name == 'install':
        workspace = prefix.parent.parent
    elif prefix.name == 'install':
        workspace = prefix.parent
    else:
        workspace = Path(os.environ.get('ROS_WS', '/ros2_ws'))
    return str(workspace / 'src' / 'bunker_slam_bringup' / 'maps' / 'bunker_rgbd.db')


def generate_launch_description():
    slam_launch = os.path.join(
        get_package_share_directory('bunker_slam_bringup'),
        'launch', 'slam_bringup.launch.py')

    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            'mode': 'localization',
            'reset_database': 'false',
            'database_path': LaunchConfiguration('database_path'),
            'bunker_port': LaunchConfiguration('bunker_port'),
            'arm_can_port': LaunchConfiguration('arm_can_port'),
            'serial_no': LaunchConfiguration('serial_no'),
            'enable_pointcloud': LaunchConfiguration('enable_pointcloud'),
            'configure_can': LaunchConfiguration('configure_can'),
            'start_base': LaunchConfiguration('start_base'),
            'start_arm': LaunchConfiguration('start_arm'),
            'start_camera': LaunchConfiguration('start_camera'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'imu_gate_timeout': LaunchConfiguration('imu_gate_timeout'),
            'arm_mapping_pose': 'false',
            'arm_mount_xyz': LaunchConfiguration('arm_mount_xyz'),
            'arm_mount_rpy': LaunchConfiguration('arm_mount_rpy'),
            'camera_mount_xyz': LaunchConfiguration('camera_mount_xyz'),
            'camera_mount_rpy': LaunchConfiguration('camera_mount_rpy'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('database_path', default_value=default_database_path()),
        DeclareLaunchArgument('bunker_port', default_value=''),
        DeclareLaunchArgument('arm_can_port', default_value=''),
        DeclareLaunchArgument('serial_no', default_value='261222075829'),
        DeclareLaunchArgument('enable_pointcloud', default_value='true'),
        DeclareLaunchArgument('configure_can', default_value='true'),
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('start_arm', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('imu_gate_timeout', default_value='30.0'),
        DeclareLaunchArgument(
            'arm_mount_xyz', default_value='0.807474630 0.0 0.545498689'),
        DeclareLaunchArgument(
            'arm_mount_rpy', default_value='-1.483492816 0.029092513 1.573342313'),
        DeclareLaunchArgument(
            'camera_mount_xyz', default_value='-0.0106 -0.0175 -0.0125'),
        DeclareLaunchArgument('camera_mount_rpy', default_value='0.0 0.0 0.0'),
        include,
    ])
