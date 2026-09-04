# Copyright 2026 Dase Orin
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('bunker_slam_bringup')
    bias_config_path = os.path.join(package_share, 'config', 'imu_bias.yaml')
    config_path = os.path.join(
        package_share, 'config', 'ekf.yaml')
    return LaunchDescription([
        Node(
            package='bunker_slam_bringup',
            executable='imu_bias_corrector.py',
            name='imu_bias_corrector',
            output='screen',
            parameters=[bias_config_path],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_path],
            remappings=[('odometry/filtered', '/odometry/filtered')],
        ),
    ])
