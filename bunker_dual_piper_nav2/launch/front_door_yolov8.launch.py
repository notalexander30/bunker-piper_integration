"""Door-only YOLOv8m detector using the dual-stack front D435i."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory('bunker_object_follower')) /
                 'config' / 'standalone_door_rgbd.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('weights_path', default_value='/ros2_ws/models/yolov8m-oiv7.pt'),
        DeclareLaunchArgument('detector_device', default_value='0'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.05'),
        LogInfo(msg='Door-only YOLOv8m Open Images class 164 on /front_camera; no motion node.'),
        Node(
            package='bunker_object_follower', executable='detector_tracker_node',
            name='front_door_detector', output='screen', emulate_tty=True,
            parameters=[config, {
                'image_topic': '/front_camera/color/image_raw',
                'aligned_depth_topic': '/front_camera/aligned_depth_to_color/image_raw',
                'camera_info_topic': '/front_camera/color/camera_info',
                'target_status_topic': '/door_search/local_target_status',
                'annotated_image_topic': '/door_search/annotated_image',
                'diagnostics_topic': '/door_search/detector_status',
                'weights_path': LaunchConfiguration('weights_path'),
                'target_class_ids': [164], 'target_class_names': ['door'],
                'device': ParameterValue(LaunchConfiguration('detector_device'), value_type=str),
                'confidence_threshold': ParameterValue(
                    LaunchConfiguration('confidence_threshold'), value_type=float),
            }],
        ),
    ])
