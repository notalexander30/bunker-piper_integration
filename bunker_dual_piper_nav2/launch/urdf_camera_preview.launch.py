"""Offline initial-pose check for the dual-arm URDF and TCP cameras.

This launch deliberately starts no CAN driver, Nav2, RTAB-Map, or arm control.
Use its mount/camera arguments to try a pose in RViz without rebuilding.
This launch starts no hardware drivers, controllers, camera nodes, mapping, or
navigation. Both arms are displayed at zero joint position.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('bunker_dual_piper_nav2')
    description_source = PythonLaunchDescriptionSource(
        PathJoinSubstitution([package_share, 'launch', 'description.launch.py'])
    )
    rviz_config = PathJoinSubstitution(
        [package_share, 'rviz', 'bunker_description.rviz']
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_rviz', default_value='true'),
        # These are only visual transforms. They never command a PiPER arm.
        DeclareLaunchArgument(
            'front_piper_mount_rpy', default_value='0 0 1.57079632679'),
        DeclareLaunchArgument('rear_piper_mount_rpy', default_value='0 0 1.57079632679'),
        DeclareLaunchArgument('front_camera_xyz', default_value='-0.03325362536889407 -0.04784194427852743 0.08630445414919316'),
        DeclareLaunchArgument('front_camera_rpy', default_value='-0.08017405150425999 -0.0031709794581423934 0.023976595541682157'),
        DeclareLaunchArgument('rear_camera_xyz', default_value='-0.0016016943280790555 -0.07988793520475261 0.0420998734823866'),
        DeclareLaunchArgument('rear_camera_rpy', default_value='-0.0041241555313950035 -0.003886773443390706 0.0554203109827187'),
        IncludeLaunchDescription(
            description_source,
            launch_arguments={
                'prefix_joint_states': 'false',
                'publish_default_joint_states': 'true',
                'use_front_joint_feedback': 'false',
                'use_rear_joint_feedback': 'false',
                'front_piper_parked_joint1': '0.0',
                'rear_piper_parked_joint1': '0.0',
                'front_piper_mount_rpy': LaunchConfiguration('front_piper_mount_rpy'),
                'rear_piper_mount_rpy': LaunchConfiguration('rear_piper_mount_rpy'),
                'front_camera_xyz': LaunchConfiguration('front_camera_xyz'),
                'front_camera_rpy': LaunchConfiguration('front_camera_rpy'),
                'rear_camera_xyz': LaunchConfiguration('rear_camera_xyz'),
                'rear_camera_rpy': LaunchConfiguration('rear_camera_rpy'),
            }.items(),
        ),
        Node(
            package='rviz2', executable='rviz2', name='urdf_camera_preview',
            output='screen', condition=IfCondition(LaunchConfiguration('start_rviz')),
            arguments=['-d', rviz_config],
        ),
    ])
