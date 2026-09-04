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
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def default_database_path():
    prefix = Path(get_package_prefix('bunker_slam_bringup'))
    if prefix.parent.name == 'install':
        workspace = prefix.parent.parent
    elif prefix.name == 'install':
        workspace = prefix.parent
    else:
        workspace = Path(os.environ.get('ROS_WS', '/ros2_ws'))
    source_maps = workspace / 'src' / 'bunker_slam_bringup' / 'maps'
    return str(source_maps / 'bunker_rgbd.db')


def launch_rtabmap(context):
    mode = LaunchConfiguration('mode').perform(context)
    reset = LaunchConfiguration('reset_database').perform(context).lower() == 'true'
    database_path = str(Path(
        LaunchConfiguration('database_path').perform(context)).expanduser().resolve())

    if mode not in ('mapping', 'localization'):
        raise RuntimeError("mode must be 'mapping' or 'localization'")
    if mode == 'localization' and reset:
        raise RuntimeError('reset_database is forbidden in localization mode')
    if mode == 'localization':
        database = Path(database_path)
        if not database.is_file() or database.stat().st_size == 0:
            raise RuntimeError(
                f'Localization database does not exist or is empty: {database_path}')
    else:
        Path(database_path).parent.mkdir(parents=True, exist_ok=True)

    config_path = os.path.join(
        get_package_share_directory('bunker_slam_bringup'),
        'config', 'rtabmap.yaml')

    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[config_path],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('rgbd_image', '/rgbd_image'),
        ],
    )

    mode_parameters = {
        'database_path': database_path,
        'Mem/IncrementalMemory': 'true' if mode == 'mapping' else 'false',
        'Mem/InitWMWithAllNodes': 'false' if mode == 'mapping' else 'true',
    }
    arguments = ['--delete_db_on_start'] if reset else []
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[config_path, mode_parameters],
        arguments=arguments,
        remappings=[
            ('rgbd_image', '/rgbd_image'),
            ('odom', '/odometry/filtered'),
        ],
    )

    return [
        LogInfo(msg=f'RTAB-Map {mode} database: {database_path}'),
        rgbd_sync,
        rtabmap,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='mapping', choices=['mapping', 'localization']),
        DeclareLaunchArgument(
            'database_path', default_value=default_database_path()),
        DeclareLaunchArgument(
            'reset_database', default_value='false', choices=['true', 'false'],
            description='Explicitly delete the selected DB before mapping.'),
        OpaqueFunction(function=launch_rtabmap),
    ])
