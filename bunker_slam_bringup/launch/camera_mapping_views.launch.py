"""Open separate 3-D reconstruction and 2-D occupancy RViz windows."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('bunker_slam_bringup')
    three_d = os.path.join(share, 'rviz', 'camera_rgbd_mapping.rviz')
    occupancy = os.path.join(share, 'rviz', 'camera_occupancy_mapping.rviz')
    return LaunchDescription([
        Node(
            package='rviz2', executable='rviz2',
            name='camera_3d_mapping_rviz2',
            arguments=['-d', three_d], output='screen'),
        Node(
            package='rviz2', executable='rviz2',
            name='camera_2d_occupancy_rviz2',
            arguments=['-d', occupancy], output='screen'),
    ])
