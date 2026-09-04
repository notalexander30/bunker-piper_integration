"""Handheld D435i 6-DoF mapping plus local-GPU YOLO perception.

The RealSense camera must already be running. This launch intentionally starts
no Bunker base, PiPER arm, navigation controller, or velocity publisher.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _workspace_path(*parts):
    prefix = Path(get_package_prefix('bunker_slam_bringup'))
    workspace = prefix.parent.parent if prefix.parent.name == 'install' else Path('/ros2_ws')
    return str(workspace.joinpath(*parts))


def generate_launch_description():
    slam_share = get_package_share_directory('bunker_slam_bringup')
    detector_share = get_package_share_directory('bunker_object_follower')
    semantic_share = get_package_share_directory('semantic_memory')
    mapping_launch = os.path.join(slam_share, 'launch', 'minimal_rgbd_mapping.launch.py')
    semantic_launch = os.path.join(semantic_share, 'launch', 'semantic_memory.launch.py')
    detector_config = os.path.join(
        detector_share, 'config', 'standalone_rgbd_yolo26.yaml'
    )
    rviz_config = os.path.join(slam_share, 'rviz', 'standalone_rgbd_yolo_mapping.rviz')

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_launch),
        launch_arguments={
            'database_path': LaunchConfiguration('database_path'),
            'reset_database': LaunchConfiguration('reset_database'),
        }.items(),
    )
    detector = Node(
        package='bunker_object_follower',
        executable='detector_tracker_node',
        name='detector_tracker_node',
        output='screen',
        emulate_tty=True,
        parameters=[detector_config, {
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
    )
    semantic_memory = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(semantic_launch),
        condition=IfCondition(LaunchConfiguration('use_semantic_memory')),
        launch_arguments={
            'params_file': detector_config,
            'memory_file': LaunchConfiguration('memory_file'),
            'rtabmap_database_path': LaunchConfiguration('database_path'),
            'yolo_detection_topic': '/standalone_yolo/local_target_status',
            'stationary_test_map': 'false',
        }.items(),
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='standalone_yolo_mapping_rviz',
        output='screen', arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        additional_env={
            'DISPLAY': os.environ.get('DISPLAY', ':1'),
            'XAUTHORITY': os.environ.get('XAUTHORITY', '/tmp/.docker.xauth'),
            'QT_X11_NO_MITSHM': '1',
        },
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'database_path',
            default_value=_workspace_path('vlm_results', 'maps', 'handheld_yolo26.db'),
        ),
        DeclareLaunchArgument('reset_database', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('weights_path', default_value=_workspace_path('models', 'yolo26n.pt')),
        DeclareLaunchArgument('detector_device', default_value='0'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.45'),
        DeclareLaunchArgument('inference_size', default_value='416'),
        DeclareLaunchArgument('processing_rate_hz', default_value='6.0'),
        DeclareLaunchArgument(
            'memory_file',
            default_value=_workspace_path('vlm_results', 'semantic', 'handheld_yolo26.json'),
        ),
        DeclareLaunchArgument('use_semantic_memory', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
        LogInfo(msg=(
            'Standalone RGB-D + YOLO: full 6-DoF mapping, CUDA detector, '
            'depth-aware tracks and optional map-frame semantic memory. No motion nodes.'
        )),
        mapping,
        detector,
        semantic_memory,
        rviz,
    ])
