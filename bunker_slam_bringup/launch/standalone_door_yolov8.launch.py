"""Standalone YOLOv8m Open Images door detector and RGB-D tracking."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('bunker_object_follower'),
        'config',
        'standalone_door_rgbd.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'weights_path', default_value='/ros2_ws/models/yolov8m-oiv7.pt'
        ),
        DeclareLaunchArgument('detector_device', default_value='0'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.05'),
        DeclareLaunchArgument('inference_size', default_value='640'),
        DeclareLaunchArgument('processing_rate_hz', default_value='8.0'),
        LogInfo(msg=(
            'Standalone door detector: yolov8m-oiv7, Open Images Door class '
            '164, aligned-depth ranging and DeepSORT. No motion nodes.'
        )),
        Node(
            package='bunker_object_follower',
            executable='detector_tracker_node',
            name='detector_tracker_node',
            output='screen',
            emulate_tty=True,
            parameters=[config, {
                'weights_path': LaunchConfiguration('weights_path'),
                'target_class_ids': [164],
                'target_class_names': ['door'],
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
