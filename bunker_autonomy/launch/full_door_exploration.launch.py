"""One-command physical bring-up: PiPER, Bunker Mini, D435i, YOLO and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    autonomy_share = get_package_share_directory('bunker_autonomy')
    base_share = get_package_share_directory('bunker_base')
    follower_share = get_package_share_directory('bunker_object_follower')
    autonomy_config = os.path.join(autonomy_share, 'config', 'door_search_d435i.yaml')
    follower_config = os.path.join(
        follower_share, 'config', 'door_follower_d435i.yaml'
    )
    rviz_config = os.path.join(
        follower_share, 'rviz', 'door_follower_d435i.rviz'
    )

    # PiPER and Bunker use separate USB-CAN adapters and incompatible bitrates.
    setup_can = ExecuteProcess(
        cmd=['bash', '-lc', (
            'set -e; piper="$1"; bunker="$2"; '
            'test -n "$piper"; test -n "$bunker"; '
            'ip link set "$piper" down 2>/dev/null || true; '
            'ip link set "$piper" type can bitrate 1000000 restart-ms 100; '
            'ip link set "$piper" up; '
            'ip link set "$bunker" down 2>/dev/null || true; '
            'ip link set "$bunker" type can bitrate 500000 restart-ms 100; '
            'ip link set "$bunker" up; '
            'ip -details link show "$piper"; '
            'ip -details link show "$bunker"'
        ), '_', LaunchConfiguration('front_piper_can'), LaunchConfiguration('bunker_can')],
        output='screen',
    )

    start_stack = TimerAction(period=2.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                autonomy_share, 'launch', 'piper_navigation_bringup.launch.py'
            )),
            launch_arguments={
                'can_port': LaunchConfiguration('front_piper_can'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                base_share, 'launch', 'bunker_base.launch.py'
            )),
            launch_arguments={
                'port_name': LaunchConfiguration('bunker_can'),
                'is_bunker_mini': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                autonomy_share, 'launch', 'realsense_d435i.launch.py'
            )),
            launch_arguments={
                'camera_rviz': 'false',
                'serial_no': LaunchConfiguration('front_camera_serial'),
            }.items(),
        ),
        Node(
            package='bunker_object_follower', executable='detector_tracker_node',
            output='screen', parameters=[follower_config],
        ),
        Node(
            package='bunker_object_follower', executable='target_mux_node',
            output='screen', parameters=[follower_config, {
                'output_target_topic': '/target_status',
                'output_detected_topic': '/target_detected',
            }],
        ),
        Node(
            package='bunker_autonomy', executable='depth_route_monitor_node',
            output='screen', parameters=[autonomy_config],
        ),
        Node(
            package='bunker_autonomy', executable='sensor_fusion_node',
            output='screen', parameters=[autonomy_config],
        ),
        Node(
            package='bunker_autonomy', executable='search_behavior_node',
            output='screen', parameters=[autonomy_config],
        ),
        Node(
            package='bunker_autonomy', executable='cmd_vel_mux_node',
            output='screen', parameters=[autonomy_config, {
                'output_cmd_vel_topic': '/cmd_vel',
            }],
        ),
        Node(
            package='bunker_autonomy', executable='operator_status_node',
            output='screen', parameters=[autonomy_config],
        ),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
    return LaunchDescription([
        DeclareLaunchArgument('front_piper_can', default_value=''),
        DeclareLaunchArgument('bunker_can', default_value=''),
        DeclareLaunchArgument('front_camera_serial', default_value=''),
        setup_can,
        start_stack,
    ])
