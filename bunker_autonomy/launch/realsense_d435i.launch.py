import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_rviz = LaunchConfiguration('camera_rviz')
    config_path = os.path.join(
        get_package_share_directory('bunker_autonomy'),
        'config',
        'realsense_d435i.yaml',
    )
    with open(config_path, encoding='utf-8') as config_file:
        camera_parameters = yaml.safe_load(config_file)

    rviz_config_path = os.path.join(
        get_package_share_directory('bunker_autonomy'),
        'config',
        'realsense_d435i.rviz',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_rviz',
            default_value='false',
            description='Start the basic D435i camera RViz viewer.',
        ),
        DeclareLaunchArgument(
            'serial_no',
            default_value='',
            description='Optional D435i serial number when more than one camera is connected.',
        ),
        DeclareLaunchArgument(
            'publish_camera_static_tf',
            default_value='true',
            description=(
                'Publish a fixed parent-to-camera_link transform. Leave false when '
                'the arm URDF already publishes a dynamic camera transform.'
            ),
        ),
        DeclareLaunchArgument('camera_tf_parent_frame', default_value='base_link'),
        DeclareLaunchArgument('camera_tf_child_frame', default_value='camera_link'),
        # Measured locked arm navigation pose: D435i centred laterally, 0.60 m
        # forward and 0.62 m above the Bunker Mini base_link origin.
        DeclareLaunchArgument('camera_x', default_value='0.60'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='0.62'),
        DeclareLaunchArgument('camera_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),
        LogInfo(
            msg=(
                'Starting one D435i: RGB -> /camera/camera/color/image_raw; '
                'navigation depth -> '
                '/camera/camera/aligned_depth_to_color/image_raw'
            )
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='camera',
            name='camera',
            parameters=[
                camera_parameters,
                # RealSense serial numbers are numeric-looking identifiers, not
                # integers.  Without this explicit type ROS parses e.g.
                # 243322074578 as an integer and the driver rejects it.
                {'serial_no': ParameterValue(
                    LaunchConfiguration('serial_no'), value_type=str)},
            ],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='d435i_rviz2',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(camera_rviz),
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='d435i_mount_static_transform_publisher',
            condition=IfCondition(LaunchConfiguration('publish_camera_static_tf')),
            arguments=[
                '--x',
                LaunchConfiguration('camera_x'),
                '--y',
                LaunchConfiguration('camera_y'),
                '--z',
                LaunchConfiguration('camera_z'),
                '--roll',
                LaunchConfiguration('camera_roll'),
                '--pitch',
                LaunchConfiguration('camera_pitch'),
                '--yaw',
                LaunchConfiguration('camera_yaw'),
                '--frame-id',
                LaunchConfiguration('camera_tf_parent_frame'),
                '--child-frame-id',
                LaunchConfiguration('camera_tf_child_frame'),
            ],
            output='screen',
        ),
    ])
