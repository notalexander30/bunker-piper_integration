"""EKF for Bunker wheel odometry plus H30 yaw-rate IMU."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('bunker_slam_bringup'),
        'config',
        'ekf_h30.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_config),
        DeclareLaunchArgument('output_odom_topic', default_value='/odom'),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[('odometry/filtered', LaunchConfiguration('output_odom_topic'))],
        ),
    ])
