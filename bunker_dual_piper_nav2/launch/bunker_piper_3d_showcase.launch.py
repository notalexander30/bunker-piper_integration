"""Standalone 3D URDF showcase for Bunker + dual PiPER + D435i.

This launch is intentionally isolated from the Nav2 full-system startup.

It starts only:
- robot_state_publisher for the combined Bunker/PiPER/camera URDF
- one offline/default joint-state publisher from description.launch.py
- one static map -> base_link transform for RViz convenience
- optional RViz

It does not start CAN, Bunker driver, PiPER drivers, RealSense, RTAB-Map, Nav2,
EKF, AMCL, or any live joint feedback subscriber.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('bunker_dual_piper_nav2')
    description_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([package_share, 'launch', 'description.launch.py'])
    )
    rviz_config = PathJoinSubstitution(
        [package_share, 'rviz', 'bunker_description.rviz']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Start RViz with the Bunker/PiPER description config.'),
        DeclareLaunchArgument(
            'fixed_frame',
            default_value='map',
            description='Parent frame used for the static showcase transform.'),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot root frame used by the URDF.'),
        DeclareLaunchArgument(
            'front_piper_parked_joint1',
            default_value='0.0',
            description='Visual-only front PiPER shoulder preview pose.'),
        DeclareLaunchArgument(
            'rear_piper_parked_joint1',
            default_value='0.0',
            description='Visual-only rear PiPER shoulder preview pose.'),
        DeclareLaunchArgument(
            'front_piper_mount_rpy',
            default_value='0 0 1.57079632679',
            description='Visual-only front PiPER mount rotation.'),
        DeclareLaunchArgument(
            'rear_piper_mount_rpy',
            default_value='0 0 1.57079632679',
            description='Visual-only rear PiPER mount rotation.'),
        DeclareLaunchArgument(
            'front_camera_xyz',
            default_value='-0.03325362536889407 -0.04784194427852743 0.08630445414919316',
            description='Visual-only front camera offset from front PiPER flange.'),
        DeclareLaunchArgument(
            'front_camera_rpy',
            default_value='-0.08017405150425999 -0.0031709794581423934 0.023976595541682157',
            description='Visual-only front camera rotation from front PiPER flange.'),
        DeclareLaunchArgument(
            'rear_camera_xyz',
            default_value='-0.03325362536889407 -0.04784194427852743 0.08630445414919316',
            description='Visual-only rear camera offset from rear PiPER flange.'),
        DeclareLaunchArgument(
            'rear_camera_rpy',
            default_value='-0.08017405150425999 -0.0031709794581423934 -3.117616058048111',
            description='Visual-only rear camera rotation from rear PiPER flange.'),
        LogInfo(msg=(
            'Starting standalone Bunker + dual PiPER 3D URDF showcase only. '
            'No hardware, no Nav2, no camera, no live joint feedback.')),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='showcase_static_map_to_base_link',
            output='screen',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', LaunchConfiguration('fixed_frame'),
                '--child-frame-id', LaunchConfiguration('base_frame'),
            ],
        ),
        IncludeLaunchDescription(
            description_launch,
            launch_arguments={
                'prefix_joint_states': 'false',
                'publish_default_joint_states': 'true',
                'use_front_joint_feedback': 'false',
                'use_rear_joint_feedback': 'false',
                'front_piper_parked_joint1': LaunchConfiguration(
                    'front_piper_parked_joint1'),
                'rear_piper_parked_joint1': LaunchConfiguration(
                    'rear_piper_parked_joint1'),
                'front_piper_mount_rpy': LaunchConfiguration(
                    'front_piper_mount_rpy'),
                'rear_piper_mount_rpy': LaunchConfiguration(
                    'rear_piper_mount_rpy'),
                'front_camera_xyz': LaunchConfiguration('front_camera_xyz'),
                'front_camera_rpy': LaunchConfiguration('front_camera_rpy'),
                'rear_camera_xyz': LaunchConfiguration('rear_camera_xyz'),
                'rear_camera_rpy': LaunchConfiguration('rear_camera_rpy'),
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='bunker_piper_3d_showcase_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            additional_env={'QT_X11_NO_MITSHM': '1'},
            condition=IfCondition(LaunchConfiguration('start_rviz')),
        ),
    ])
