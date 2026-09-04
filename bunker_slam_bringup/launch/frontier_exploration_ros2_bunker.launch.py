# Copyright 2026 Dase Orin
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""
Bunker wrapper for the upstream frontier_exploration_ros2 package.

This starts the upstream C++ explorer with Bunker/Nav2 topic and frame names.
It is cold-idle by default, so it will not send Nav2 goals until commanded
through frontier_exploration_ctl or the control_exploration service.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('bunker_slam_bringup')
    frontier_share = get_package_share_directory('frontier_exploration_ros2')
    default_params = os.path.join(
        bringup_share, 'config', 'frontier_exploration_ros2_bunker.yaml')
    upstream_launch = os.path.join(
        frontier_share, 'launch', 'frontier_explorer.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'autostart',
            default_value='false',
            choices=['true', 'false'],
            description='Keep false for operator-gated startup.'),
        DeclareLaunchArgument(
            'control_service_enabled',
            default_value='true',
            choices=['true', 'false']),
        LogInfo(msg=(
            'Starting upstream frontier_exploration_ros2 with Bunker topic '
            'mapping. Default is cold-idle; use frontier_exploration_ctl start '
            'after Nav2 drive mode and manual goal checks pass.')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(upstream_launch),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'log_level': LaunchConfiguration('log_level'),
                'autostart': LaunchConfiguration('autostart'),
                'control_service_enabled': LaunchConfiguration(
                    'control_service_enabled'),
                'map_qos_autodetect_on_startup': 'true',
                'map_qos_autodetect_timeout_s': '2.0',
                'map_qos_durability': 'transient_local',
                'costmap_qos_reliability': 'reliable',
            }.items(),
        ),
    ])
