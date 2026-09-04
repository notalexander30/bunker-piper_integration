import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


RGB_TOPIC = '/camera/camera/color/image_raw'
ALIGNED_DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'


def generate_launch_description():
    autonomy_share = get_package_share_directory('bunker_autonomy')

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(autonomy_share, 'launch', 'realsense_d435i.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('launch_camera')),
        launch_arguments={
            'serial_no': LaunchConfiguration('camera_serial_no'),
            'publish_camera_static_tf': LaunchConfiguration(
                'publish_camera_static_tf'
            ),
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

    simple_vlm_node = Node(
        package='simple_vlm',
        executable='simple_vlm_node',
        name='simple_vlm_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('launch_simple_vlm')),
        parameters=[
            {
                'config_file': LaunchConfiguration('simple_vlm_config'),
                'image_topic': RGB_TOPIC,
                'model_name': LaunchConfiguration('vlm_model_name'),
            }
        ],
    )

    autonomy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(autonomy_share, 'launch', 'find_trash_can_demo.launch.py')
        ),
        launch_arguments={
            'mode': LaunchConfiguration('mode'),
            'navigation_mode': 'depth_only',
            'config_file': LaunchConfiguration('autonomy_config'),
            'depth_image_topic': ALIGNED_DEPTH_TOPIC,
            'enable_data_logging': LaunchConfiguration('enable_data_logging'),
            'publish_lidar_static_tf': 'false',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='dry_run',
            description='Use dry_run first; drive publishes commands to the chassis.',
        ),
        DeclareLaunchArgument(
            'launch_camera',
            default_value='true',
            description='Start the single D435i camera process.',
        ),
        DeclareLaunchArgument(
            'launch_simple_vlm',
            default_value='true',
            description='Start simple_vlm with the D435i RGB stream.',
        ),
        DeclareLaunchArgument(
            'camera_serial_no',
            default_value='',
            description='Optional D435i serial number.',
        ),
        DeclareLaunchArgument(
            'simple_vlm_config',
            default_value=os.path.join(
                get_package_share_directory('simple_vlm'), 'config', 'config.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'vlm_model_name',
            default_value='gpt-5.4-mini',
            description='Low-latency GPT-5.6 family tier for semantic planning.',
        ),
        DeclareLaunchArgument(
            'autonomy_config',
            default_value=os.path.join(
                autonomy_share, 'config', 'autonomy_d435i.yaml'
            ),
        ),
        DeclareLaunchArgument('enable_data_logging', default_value='true'),
        DeclareLaunchArgument('publish_camera_static_tf', default_value='false'),
        DeclareLaunchArgument('camera_tf_parent_frame', default_value='base_link'),
        DeclareLaunchArgument('camera_tf_child_frame', default_value='camera_link'),
        DeclareLaunchArgument('camera_x', default_value='0.0'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='0.0'),
        DeclareLaunchArgument('camera_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),
        LogInfo(
            msg=(
                'D435i-only pipeline: RGB -> simple_vlm; aligned depth -> '
                'navigation safety; no LiDAR node is started or required.'
            )
        ),
        camera_launch,
        simple_vlm_node,
        autonomy_launch,
    ])
