# Copyright 2026 Dase Orin
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('bunker_slam_bringup'),
        'config', 'realsense.yaml')
    with open(config_path, encoding='utf-8') as config_file:
        camera_parameters = yaml.safe_load(config_file)

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_no', default_value='',
            description='Discovered Intel RealSense D435i serial number.'),
        DeclareLaunchArgument(
            'enable_pointcloud', default_value='false',
            choices=['true', 'false'],
            description=(
                'Publish the registered per-frame RGB-D PointCloud2 used by '
                'the Nav2 local costmap. Leave false for basic mapping.')),
        LogInfo(msg=(
            'D435i RGB-D: RGB=/camera/camera/color/image_raw, '
            'depth=/camera/camera/aligned_depth_to_color/image_raw. '
            'Vehicle IMU is provided by the external H30.')),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='camera',
            name='camera',
            output='screen',
            emulate_tty=True,
            parameters=[
                camera_parameters,
                {
                    'serial_no': ParameterValue(
                        LaunchConfiguration('serial_no'), value_type=str),
                    'pointcloud.enable': ParameterValue(
                        LaunchConfiguration('enable_pointcloud'), value_type=bool),
                },
            ],
        ),
    ])
