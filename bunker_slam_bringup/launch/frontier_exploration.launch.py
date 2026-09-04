"""Optional frontier exploration over RTAB-Map/Nav2.

Launch this only after mapping sensors, RTAB-Map mapping mode, odometry, TF and
Nav2 are already healthy.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'dry_run', default_value='true', choices=['true', 'false'],
            description='true logs frontier goals without moving the robot.'),
        DeclareLaunchArgument(
            'return_to_start', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'completion_mode', default_value='time_limit',
            choices=['time_limit', 'frontier_ratio', 'aruco_found']),
        DeclareLaunchArgument('time_limit_sec', default_value='120.0'),
        DeclareLaunchArgument('frontier_stop_ratio', default_value='0.20'),
        DeclareLaunchArgument('aruco_found_topic', default_value='/aruco_landmarks/found'),
        DeclareLaunchArgument('frontier_max_distance_m', default_value='4.0'),
        DeclareLaunchArgument('frontier_min_distance_m', default_value='0.75'),
        DeclareLaunchArgument('frontier_cluster_min_cells', default_value='12'),
        DeclareLaunchArgument('no_frontier_return_cycles', default_value='2'),
        DeclareLaunchArgument('safety_stop_topic', default_value='/safety_stop'),
        DeclareLaunchArgument('mux_reason_topic', default_value='/cmd_vel_mux/reason'),
        DeclareLaunchArgument('clear_pose_min_age_sec', default_value='1.0'),
        DeclareLaunchArgument('recovery_goal_min_distance_m', default_value='0.35'),
        LogInfo(msg=(
            'Frontier exploration requires a healthy manual Nav2 goal first. '
            'Use dry_run:=true until goals look correct.')),
        Node(
            package='bunker_autonomy',
            executable='frontier_explorer_node',
            name='frontier_explorer',
            output='screen',
            parameters=[{
                'dry_run': LaunchConfiguration('dry_run'),
                'return_to_start': LaunchConfiguration('return_to_start'),
                'completion_mode': LaunchConfiguration('completion_mode'),
                'time_limit_sec': LaunchConfiguration('time_limit_sec'),
                'frontier_stop_ratio': LaunchConfiguration('frontier_stop_ratio'),
                'aruco_found_topic': LaunchConfiguration('aruco_found_topic'),
                'frontier_max_distance_m': LaunchConfiguration(
                    'frontier_max_distance_m'),
                'frontier_min_distance_m': LaunchConfiguration(
                    'frontier_min_distance_m'),
                'frontier_cluster_min_cells': LaunchConfiguration(
                    'frontier_cluster_min_cells'),
                'no_frontier_return_cycles': LaunchConfiguration(
                    'no_frontier_return_cycles'),
                'safety_stop_topic': LaunchConfiguration('safety_stop_topic'),
                'mux_reason_topic': LaunchConfiguration('mux_reason_topic'),
                'clear_pose_min_age_sec': LaunchConfiguration(
                    'clear_pose_min_age_sec'),
                'recovery_goal_min_distance_m': LaunchConfiguration(
                    'recovery_goal_min_distance_m'),
            }],
        ),
    ])
