"""RViz preset for standalone D435i RGB-D visual mapping."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('bunker_slam_bringup'),
        'rviz', 'camera_rgbd_mapping.rviz')
    return LaunchDescription([
        Node(
            package='rviz2', executable='rviz2',
            name='camera_rgbd_mapping_rviz2',
            arguments=['-d', config_path], output='screen'),
    ])
