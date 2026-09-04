"""Terminal 1: D435i plus RGB-D preprocessing and handheld visual odometry."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    camera_launch = os.path.join(
        get_package_share_directory('bunker_autonomy'),
        'launch',
        'realsense_d435i.launch.py',
    )
    mapping_launch = os.path.join(
        get_package_share_directory('bunker_slam_bringup'),
        'launch',
        'minimal_rgbd_mapping.launch.py',
    )
    return LaunchDescription([
        DeclareLaunchArgument('serial_no', default_value='261222075829'),
        LogInfo(msg=(
            'Terminal 1 camera pipeline: D435i + RGB-D sync + 6-DoF visual '
            'odometry + diagnostic live cloud. RTAB-Map and YOLO are excluded.'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            launch_arguments={
                'serial_no': LaunchConfiguration('serial_no'),
                'camera_rviz': 'false',
                # A handheld camera has no fixed base_link mount.
                'publish_camera_static_tf': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch),
            launch_arguments={
                'reset_database': 'false',
                'start_rgbd_odometry': 'true',
                'start_rgbd_sync': 'true',
                'start_live_cloud': 'true',
                'start_rtabmap': 'false',
            }.items(),
        ),
    ])
