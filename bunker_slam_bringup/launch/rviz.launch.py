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
    config_path = os.path.join(
        get_package_share_directory('bunker_slam_bringup'),
        'rviz', 'bunker_rgbd_slam.rviz')
    return LaunchDescription([
        Node(
            package='rviz2', executable='rviz2', name='bunker_slam_rviz2',
            arguments=['-d', config_path], output='screen'),
    ])
