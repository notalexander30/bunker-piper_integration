import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from bunker_autonomy.launch_safety import resolve_output_cmd_vel_topic


def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    mode = LaunchConfiguration('mode').perform(context).strip().lower()
    output_cmd_vel_topic_override = LaunchConfiguration('output_cmd_vel_topic').perform(
        context
    ).strip()
    depth_image_topic_override = LaunchConfiguration('depth_image_topic').perform(
        context
    ).strip()
    navigation_mode = LaunchConfiguration('navigation_mode').perform(context).strip().lower()
    if navigation_mode not in ('lidar_depth', 'depth_only'):
        raise ValueError(
            "navigation_mode must be 'lidar_depth' or 'depth_only', got %r"
            % navigation_mode
        )

    output_cmd_vel_topic = resolve_output_cmd_vel_topic(
        mode,
        output_cmd_vel_topic_override,
    )

    cmd_vel_mux_parameters = [config_file]
    cmd_vel_mux_parameters.append({'output_cmd_vel_topic': output_cmd_vel_topic})
    operator_status_parameters = [
        config_file,
        {
            'mode': mode,
            'navigation_mode': navigation_mode,
            'output_cmd_vel_topic': output_cmd_vel_topic,
        },
    ]
    data_logger_parameters = [
        config_file,
        {
            'output_cmd_vel_topic': output_cmd_vel_topic,
        },
    ]
    depth_monitor_parameters = [config_file]
    sensor_fusion_parameters = [
        config_file,
        {'fusion_mode': navigation_mode},
    ]
    lidar_navigation_condition = IfCondition(
        PythonExpression([
            "'",
            LaunchConfiguration('navigation_mode'),
            "' == 'lidar_depth'",
        ])
    )
    lidar_static_tf_condition = IfCondition(
        PythonExpression([
            "'",
            LaunchConfiguration('navigation_mode'),
            "' == 'lidar_depth' and '",
            LaunchConfiguration('publish_lidar_static_tf'),
            "'.lower() == 'true'",
        ])
    )
    # operator_status_node is the single concise INFO/WARN stream. Background
    # nodes remain visible only for genuine errors.
    background_log_arguments = ['--ros-args', '--log-level', 'error']
    if depth_image_topic_override:
        depth_monitor_parameters.append(
            {'depth_image_topic': depth_image_topic_override}
        )

    mode_log = (
        'DRY RUN: commands go to /cmd_vel_debug and the robot will not move.'
        if mode == 'dry_run'
        else 'DRIVE: commands go to /cmd_vel and the chassis can move.'
    )

    return [
        LogInfo(msg='Starting bunker_autonomy find-trash-can demo nodes.'),
        LogInfo(
            msg=(
                'RealSense depth is the navigation source. In lidar_depth mode, '
                'LiDAR is also required. Camera and simple_vlm may be supplied by '
                'a parent launch or launched separately.'
            )
        ),
        LogInfo(msg='Navigation mode: %s' % navigation_mode),
        LogInfo(msg='Autonomy mode: %s' % mode_log),
        LogInfo(msg='Final cmd_vel output topic: %s' % output_cmd_vel_topic),
        LogInfo(msg='Human-readable status topic: /autonomy_notice'),
        LogInfo(msg='Stop attribution topic: /safety_stop_reason'),
        LogInfo(
            msg='Data logging enabled: ~/vlm_results/autonomy_logs/*.jsonl',
            condition=IfCondition(LaunchConfiguration('enable_data_logging')),
        ),
        LogInfo(
            msg=(
                'Stop transition log: '
                '~/vlm_results/autonomy_logs/safety_events_*.jsonl'
            ),
            condition=IfCondition(LaunchConfiguration('enable_data_logging')),
        ),
        LogInfo(
            msg=(
                'VLM/depth data-flow log: '
                '~/vlm_results/autonomy_logs/perception_trace_*.jsonl'
            ),
            condition=IfCondition(LaunchConfiguration('enable_data_logging')),
        ),
        LogInfo(
            msg='Publishing static LiDAR TF from launch arguments.',
            condition=lidar_static_tf_condition,
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_transform_publisher',
            output='screen',
            emulate_tty=True,
            condition=lidar_static_tf_condition,
            arguments=[
                '--x',
                LaunchConfiguration('lidar_x'),
                '--y',
                LaunchConfiguration('lidar_y'),
                '--z',
                LaunchConfiguration('lidar_z'),
                '--roll',
                LaunchConfiguration('lidar_roll'),
                '--pitch',
                LaunchConfiguration('lidar_pitch'),
                '--yaw',
                LaunchConfiguration('lidar_yaw'),
                '--frame-id',
                LaunchConfiguration('lidar_tf_parent_frame'),
                '--child-frame-id',
                LaunchConfiguration('lidar_tf_child_frame'),
            ],
        ),
        Node(
            package='bunker_autonomy',
            executable='safety_monitor_node',
            name='safety_monitor_node',
            output='screen',
            emulate_tty=True,
            parameters=[config_file],
            arguments=background_log_arguments,
            condition=lidar_navigation_condition,
        ),
        Node(
            package='bunker_autonomy',
            executable='depth_route_monitor_node',
            name='depth_route_monitor_node',
            output='screen',
            emulate_tty=True,
            parameters=depth_monitor_parameters,
            arguments=background_log_arguments,
        ),
        Node(
            package='bunker_autonomy',
            executable='sensor_fusion_node',
            name='sensor_fusion_node',
            output='screen',
            emulate_tty=True,
            parameters=sensor_fusion_parameters,
            arguments=background_log_arguments,
        ),
        Node(
            package='bunker_autonomy',
            executable='vlm_goal_monitor_node',
            name='vlm_goal_monitor_node',
            output='screen',
            emulate_tty=True,
            parameters=[config_file],
            arguments=background_log_arguments,
            condition=IfCondition(LaunchConfiguration('launch_vlm_monitor')),
        ),
        Node(
            package='bunker_autonomy',
            executable='search_behavior_node',
            name='search_behavior_node',
            output='screen',
            emulate_tty=True,
            parameters=[config_file],
            on_exit=[Shutdown(reason='Mission behavior finished.')],
            arguments=background_log_arguments,
        ),
        Node(
            package='bunker_autonomy',
            executable='cmd_vel_mux_node',
            name='cmd_vel_mux_node',
            output='screen',
            emulate_tty=True,
            parameters=cmd_vel_mux_parameters,
            arguments=background_log_arguments,
        ),
        Node(
            package='bunker_autonomy',
            executable='operator_status_node',
            name='operator_status_node',
            output='screen',
            emulate_tty=True,
            parameters=operator_status_parameters,
        ),
        Node(
            package='bunker_autonomy',
            executable='data_logger_node',
            name='data_logger_node',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('enable_data_logging')),
            parameters=data_logger_parameters,
            arguments=background_log_arguments,
        ),
    ]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('bunker_autonomy'),
        'config',
        'autonomy.yaml',
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to bunker_autonomy YAML parameter file.',
    )
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='dry_run',
        description=(
            "Autonomy output mode. Use 'dry_run' for /cmd_vel_debug with no robot "
            "movement, or 'drive' for /cmd_vel."
        ),
    )
    output_cmd_vel_topic_arg = DeclareLaunchArgument(
        'output_cmd_vel_topic',
        default_value='',
        description=(
            'Optional final Twist output topic override. Dry-run mode only permits '
            '/cmd_vel_debug.'
        ),
    )
    launch_vlm_monitor_arg = DeclareLaunchArgument(
        'launch_vlm_monitor',
        default_value='true',
        description='Start vlm_goal_monitor_node. Disable when another detector publishes /target_status.',
    )
    enable_data_logging_arg = DeclareLaunchArgument(
        'enable_data_logging',
        default_value='true',
        description='Write mission debug and ML data to ~/vlm_results/autonomy_logs.',
    )
    depth_image_topic_arg = DeclareLaunchArgument(
        'depth_image_topic',
        default_value='',
        description=(
            'Optional RealSense depth image topic override. The stable default comes '
            'from autonomy.yaml and uses depth/image_rect_raw.'
        ),
    )
    navigation_mode_arg = DeclareLaunchArgument(
        'navigation_mode',
        default_value='lidar_depth',
        description=(
            "Use 'depth_only' for D435i-only navigation, or 'lidar_depth' for "
            'the legacy two-sensor pipeline.'
        ),
    )
    publish_lidar_static_tf_arg = DeclareLaunchArgument(
        'publish_lidar_static_tf',
        default_value='false',
        description=(
            'Publish a static transform for the LiDAR frame. '
            'Use measured LiDAR pose values before chassis tests.'
        ),
    )
    lidar_tf_parent_frame_arg = DeclareLaunchArgument(
        'lidar_tf_parent_frame',
        default_value='base_link',
        description='Parent frame for the optional static LiDAR transform.',
    )
    lidar_tf_child_frame_arg = DeclareLaunchArgument(
        'lidar_tf_child_frame',
        default_value='laser_link',
        description='Child frame for the optional static LiDAR transform.',
    )
    lidar_x_arg = DeclareLaunchArgument(
        'lidar_x',
        default_value='0.0',
        description='LiDAR x offset from parent frame in meters.',
    )
    lidar_y_arg = DeclareLaunchArgument(
        'lidar_y',
        default_value='0.0',
        description='LiDAR y offset from parent frame in meters.',
    )
    lidar_z_arg = DeclareLaunchArgument(
        'lidar_z',
        default_value='0.0',
        description='LiDAR z offset from parent frame in meters.',
    )
    lidar_roll_arg = DeclareLaunchArgument(
        'lidar_roll',
        default_value='0.0',
        description='LiDAR roll from parent frame in radians.',
    )
    lidar_pitch_arg = DeclareLaunchArgument(
        'lidar_pitch',
        default_value='0.0',
        description='LiDAR pitch from parent frame in radians.',
    )
    lidar_yaw_arg = DeclareLaunchArgument(
        'lidar_yaw',
        default_value='0.0',
        description='LiDAR yaw from parent frame in radians.',
    )

    return LaunchDescription([
        config_file_arg,
        mode_arg,
        output_cmd_vel_topic_arg,
        launch_vlm_monitor_arg,
        enable_data_logging_arg,
        depth_image_topic_arg,
        navigation_mode_arg,
        publish_lidar_static_tf_arg,
        lidar_tf_parent_frame_arg,
        lidar_tf_child_frame_arg,
        lidar_x_arg,
        lidar_y_arg,
        lidar_z_arg,
        lidar_roll_arg,
        lidar_pitch_arg,
        lidar_yaw_arg,
        OpaqueFunction(function=launch_setup),
    ])
