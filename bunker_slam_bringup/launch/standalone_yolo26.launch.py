"""Standalone local-GPU YOLO26 and depth-aware tracking."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _workspace_path(*parts):
    prefix = Path(get_package_prefix('bunker_slam_bringup'))
    workspace = prefix.parent.parent if prefix.parent.name == 'install' else Path('/ros2_ws')
    return str(workspace.joinpath(*parts))


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('bunker_object_follower'),
        'config',
        'standalone_rgbd_yolo26.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument('weights_path', default_value=_workspace_path('models', 'yolo26n.pt')),
        DeclareLaunchArgument('detector_device', default_value='0'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.45'),
        DeclareLaunchArgument('inference_size', default_value='416'),
        DeclareLaunchArgument('processing_rate_hz', default_value='6.0'),
        LogInfo(msg=(
            'Standalone YOLO: YOLO26 + depth-aware DeepSORT on local CUDA. '
            'No mapping, memory, RViz, controller or velocity publisher.'
        )),
        Node(
            package='bunker_object_follower',
            executable='detector_tracker_node',
            name='detector_tracker_node',
            output='screen',
            emulate_tty=True,
            parameters=[config, {
                'weights_path': LaunchConfiguration('weights_path'),
                'device': ParameterValue(LaunchConfiguration('detector_device'), value_type=str),
                'confidence_threshold': ParameterValue(
                    LaunchConfiguration('confidence_threshold'), value_type=float
                ),
                'inference_size': ParameterValue(
                    LaunchConfiguration('inference_size'), value_type=int
                ),
                'processing_rate_hz': ParameterValue(
                    LaunchConfiguration('processing_rate_hz'), value_type=float
                ),
            }],
        ),
    ])
