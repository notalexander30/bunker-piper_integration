# Copyright 2026 Dase Orin
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""
Launch Door-only YOLOv8m and map-frame semantic memory for the robot camera.

The RealSense and RTAB-Map stacks are deliberately external so this launch can
own one terminal without duplicating camera, odometry, TF, or motion nodes.
"""

import os
from pathlib import Path

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def workspace_path(*parts):
    prefix = Path(get_package_prefix('bunker_slam_bringup'))
    workspace = prefix.parent.parent if prefix.parent.name == 'install' else Path('/ros2_ws')
    return str(workspace.joinpath(*parts))


def generate_launch_description():
    follower_share = get_package_share_directory('bunker_object_follower')
    semantic_share = get_package_share_directory('semantic_memory')
    detector_config = os.path.join(
        follower_share, 'config', 'standalone_door_rgbd.yaml'
    )
    semantic_launch = os.path.join(
        semantic_share, 'launch', 'semantic_memory.launch.py'
    )

    detector = Node(
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
                    LaunchConfiguration('detector_device'), value_type=str
                ),
                'confidence_threshold': ParameterValue(
                    LaunchConfiguration('confidence_threshold'), value_type=float
                ),
                'inference_size': ParameterValue(
                    LaunchConfiguration('inference_size'), value_type=int
                ),
                'processing_rate_hz': ParameterValue(
                    LaunchConfiguration('processing_rate_hz'), value_type=float
                ),
            },
        ],
    )

    semantic_memory = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(semantic_launch),
        launch_arguments={
            'params_file': LaunchConfiguration('detector_config'),
            'memory_file': LaunchConfiguration('memory_file'),
            'rtabmap_database_path': LaunchConfiguration('database_path'),
            'yolo_detection_topic': '/standalone_yolo/local_target_status',
            'stationary_test_map': 'false',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('detector_config', default_value=detector_config),
        DeclareLaunchArgument(
            'weights_path',
            default_value=workspace_path('models', 'yolov8m-oiv7.pt')
        ),
        DeclareLaunchArgument('detector_device', default_value='0'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.45'),
        DeclareLaunchArgument('inference_size', default_value='416'),
        DeclareLaunchArgument('processing_rate_hz', default_value='6.0'),
        DeclareLaunchArgument(
            'database_path',
            default_value=workspace_path(
                'vlm_results', 'maps', 'bunker_door_five.db'
            ),
        ),
        DeclareLaunchArgument(
            'memory_file',
            default_value=workspace_path(
                'vlm_results', 'semantic', 'bunker_door_five.json'
            ),
        ),
        LogInfo(msg=(
            'Door-only YOLOv8m GPU detector and map-frame semantic memory. '
            'No camera, RTAB-Map, controller, TF, or cmd_vel node is started.'
        )),
        detector,
        semantic_memory,
    ])
