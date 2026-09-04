import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('bunker_autonomy')
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'realsense_d435i.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('launch_camera')),
        launch_arguments={
            'serial_no': LaunchConfiguration('camera_serial_no'),
            'publish_camera_static_tf': LaunchConfiguration('publish_camera_static_tf'),
            'camera_tf_parent_frame': LaunchConfiguration('camera_tf_parent_frame'),
            'camera_tf_child_frame': LaunchConfiguration('camera_tf_child_frame'),
            'camera_x': LaunchConfiguration('camera_x'),
            'camera_y': LaunchConfiguration('camera_y'),
            'camera_z': LaunchConfiguration('camera_z'),
            'camera_roll': LaunchConfiguration('camera_roll'),
            'camera_pitch': LaunchConfiguration('camera_pitch'),
            'camera_yaw': LaunchConfiguration('camera_yaw'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('launch_camera', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('camera_serial_no', default_value=''),
        DeclareLaunchArgument('publish_camera_static_tf', default_value='true'),
        DeclareLaunchArgument('camera_tf_parent_frame', default_value='base_link'),
        DeclareLaunchArgument('camera_tf_child_frame', default_value='camera_link'),
        DeclareLaunchArgument('camera_x', default_value='0.60'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='0.62'),
        DeclareLaunchArgument('camera_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'view_config_file',
            default_value=os.path.join(
                package_share,
                'config',
                'autonomy_d435i.yaml',
            ),
            description='D435i navigation config whose ROI is visualized.',
        ),
        DeclareLaunchArgument(
            'report_directory',
            default_value='~/vlm_results/d435i_calibration',
            description='Directory for timestamped intrinsic/view reports.',
        ),
        LogInfo(
            msg=(
                'Calibration view: white lines show ROI and LEFT/FRONT/RIGHT '
                'sectors. Edit autonomy_d435i.yaml, rebuild, and relaunch to tune.'
            )
        ),
        camera_launch,
        Node(
            package='bunker_autonomy',
            executable='d435i_calibration_node',
            name='d435i_calibration_node',
            parameters=[{
                'config_file': LaunchConfiguration('view_config_file'),
                'report_directory': LaunchConfiguration('report_directory'),
            }],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='d435i_calibration_rviz',
            arguments=[
                '-d',
                os.path.join(
                    package_share,
                    'config',
                    'realsense_d435i_calibration.rviz',
                ),
            ],
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
            output='screen',
        ),
    ])
