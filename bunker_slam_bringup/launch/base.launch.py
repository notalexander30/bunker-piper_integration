# Copyright 2026 Dase Orin
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    bunker_launch = os.path.join(
        get_package_share_directory('bunker_base'), 'launch', 'bunker_base.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('bunker_port', default_value='can3'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bunker_launch),
            launch_arguments={
                'port_name': LaunchConfiguration('bunker_port'),
                'is_bunker_mini': 'true',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'odom_topic_name': '/wheel/odometry',
                # The EKF is the only odom -> base_link owner.
                'publish_tf': 'false',
                'simulated_robot': 'false',
                'use_sim_time': 'false',
            }.items(),
        ),
    ])
