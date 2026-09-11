import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ALIGNED_DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    launch_realsense = LaunchConfiguration('launch_realsense')
    launch_simple_vlm = LaunchConfiguration('launch_simple_vlm')
    simple_vlm_config = LaunchConfiguration('simple_vlm_config')

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bunker_autonomy'),
                'launch',
                'realsense_d435i.launch.py',
            )
        ),
        condition=IfCondition(launch_realsense),
    )

    simple_vlm_node = Node(
        package='simple_vlm',
        executable='simple_vlm_node',
        name='simple_vlm_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(launch_simple_vlm),
        parameters=[
            {
                'config_file': simple_vlm_config,
                'image_topic': '/camera/camera/color/image_raw',
            }
        ],
    )

    autonomy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bunker_autonomy'),
                'launch',
                'find_trash_can_demo.launch.py',
            )
        ),
        launch_arguments={
            'mode': mode,
            'config_file': LaunchConfiguration('autonomy_config'),
            'depth_image_topic': ALIGNED_DEPTH_TOPIC,
            'navigation_mode': 'depth_only',
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
            'launch_realsense',
            default_value='true',
            description=(
                'Start the D435i camera process. Set false only when one external '
                'D435i driver already publishes the expected topics.'
            ),
        ),
        DeclareLaunchArgument(
            'launch_simple_vlm',
            default_value='true',
            description='Start simple_vlm and feed it the RealSense color stream.',
        ),
        DeclareLaunchArgument(
            'simple_vlm_config',
            default_value=os.path.join(
                get_package_share_directory('bunker_autonomy'),
                'config',
                'simple_vlm_vla.yaml',
            ),
            description='Path to the Nav-Man simple_vlm YAML configuration.',
        ),
        DeclareLaunchArgument(
            'autonomy_config',
            default_value=os.path.join(
                get_package_share_directory('bunker_autonomy'),
                'config',
                'autonomy_d435i.yaml',
            ),
            description='Path to the bunker autonomy YAML configuration.',
        ),
        DeclareLaunchArgument(
            'enable_data_logging',
            default_value='true',
            description='Write mission logs under ~/vlm_results/autonomy_logs.',
        ),
        LogInfo(
            msg=(
                'D435i-only VLA workflow: RGB -> action proposal; aligned depth '
                'validates navigation and target range. No LiDAR is required. '
                'Follow /autonomy_notice for concise status.'
            )
        ),
        realsense_launch,
        simple_vlm_node,
        autonomy_launch,
    ])
