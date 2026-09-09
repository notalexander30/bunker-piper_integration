# Copyright 2026 Dase Orin
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

import os
from pathlib import Path

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def default_database_path():
    prefix = Path(get_package_prefix('bunker_slam_bringup'))
    if prefix.parent.name == 'install':
        workspace = prefix.parent.parent
    elif prefix.name == 'install':
        workspace = prefix.parent
    else:
        workspace = Path(os.environ.get('ROS_WS', '/ros2_ws'))
    return str(workspace / 'src' / 'bunker_slam_bringup' / 'maps' / 'bunker_rgbd.db')


def include_launch(filename, arguments=None, condition=None):
    action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('bunker_slam_bringup'), 'launch', filename)),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )
    return action


def usb_serial_for_can(interface):
    path = (Path('/sys/class/net') / interface / 'device').resolve()
    while path != path.parent:
        serial_path = path / 'serial'
        if serial_path.is_file():
            return serial_path.read_text(encoding='utf-8').strip()
        path = path.parent
    return None


def resolve_can_launch_configurations(context):
    requested_arm = LaunchConfiguration('arm_can_port').perform(context)
    requested_bunker = LaunchConfiguration('bunker_port').perform(context)
    if requested_arm != 'auto' and requested_bunker != 'auto':
        return []

    interfaces_by_serial = {}
    for path in sorted(Path('/sys/class/net').glob('can*')):
        serial = usb_serial_for_can(path.name)
        if serial:
            interfaces_by_serial.setdefault(serial, []).append(path.name)

    def resolve(role, requested, serial_argument):
        if requested != 'auto':
            return requested
        serial = LaunchConfiguration(serial_argument).perform(context)
        matches = interfaces_by_serial.get(serial, [])
        if len(matches) != 1:
            raise RuntimeError(
                f'Expected exactly one {role} USB-CAN adapter with serial '
                f'{serial}; found {len(matches)}')
        return matches[0]

    arm = resolve('PiPER', requested_arm, 'arm_can_usb_serial')
    bunker = resolve('Bunker', requested_bunker, 'bunker_can_usb_serial')
    if arm == bunker:
        raise RuntimeError('PiPER and Bunker resolved to the same CAN interface')
    return [
        SetLaunchConfiguration('arm_can_port', arm),
        SetLaunchConfiguration('bunker_port', bunker),
        LogInfo(msg=f'Resolved USB-CAN adapters: PiPER={arm}, Bunker={bunker}'),
    ]


def generate_launch_description():
    bunker_port = LaunchConfiguration('bunker_port')
    arm_can_port = LaunchConfiguration('arm_can_port')
    configure_can = LaunchConfiguration('configure_can')
    start_base = LaunchConfiguration('start_base')
    start_arm = LaunchConfiguration('start_arm')
    start_camera = LaunchConfiguration('start_camera')
    use_rviz = LaunchConfiguration('use_rviz')
    imu_gate_timeout = LaunchConfiguration('imu_gate_timeout')
    require_corrected_imu = LaunchConfiguration('require_corrected_imu')

    configure_script = os.path.join(
        get_package_prefix('bunker_slam_bringup'),
        'lib', 'bunker_slam_bringup', 'configure_can.sh')

    can_setup = ExecuteProcess(
        cmd=[configure_script, arm_can_port, bunker_port],
        output='screen',
        condition=IfCondition(configure_can),
    )

    camera = include_launch(
        'realsense.launch.py', {
            'serial_no': LaunchConfiguration('serial_no'),
            'enable_pointcloud': LaunchConfiguration('enable_pointcloud'),
        },
        IfCondition(start_camera))

    def stack_actions_after_can():
        base = include_launch(
            'base.launch.py', {'bunker_port': bunker_port},
            IfCondition(start_base))
        robot_tf = include_launch(
            'robot_tf.launch.py', {
                'arm_can_port': arm_can_port,
                'start_arm_driver': start_arm,
                'start_rear_arm_driver': LaunchConfiguration(
                    'start_rear_arm'),
                'rear_arm_can_port': LaunchConfiguration('rear_arm_can_port'),
                'joint_states_topic': LaunchConfiguration('joint_states_topic'),
                'rear_joint_states_topic': LaunchConfiguration(
                    'rear_joint_states_topic'),
                'use_locked_arm_state_fallback': LaunchConfiguration(
                    'use_locked_arm_state_fallback'),
                'arm_mapping_pose': LaunchConfiguration('arm_mapping_pose'),
                'rear_arm_mapping_pose': LaunchConfiguration(
                    'rear_arm_mapping_pose'),
                'arm_mount_xyz': LaunchConfiguration('arm_mount_xyz'),
                'arm_mount_rpy': LaunchConfiguration('arm_mount_rpy'),
                'camera_mount_xyz': LaunchConfiguration('camera_mount_xyz'),
                'camera_mount_rpy': LaunchConfiguration('camera_mount_rpy'),
            })
        ekf = include_launch('ekf.launch.py')
        imu_gate = Node(
            package='bunker_slam_bringup',
            executable='wait_for_corrected_imu.py',
            name='corrected_imu_startup_gate',
            output='screen',
            arguments=['--timeout-seconds', imu_gate_timeout],
            condition=IfCondition(require_corrected_imu),
        )
        rtabmap = include_launch('rtabmap.launch.py', {
            'mode': LaunchConfiguration('mode'),
            'database_path': LaunchConfiguration('database_path'),
            'reset_database': LaunchConfiguration('reset_database'),
        })
        rviz = include_launch(
            'rviz.launch.py', condition=IfCondition(use_rviz))
        rtabmap_without_imu_gate = include_launch('rtabmap.launch.py', {
            'mode': LaunchConfiguration('mode'),
            'database_path': LaunchConfiguration('database_path'),
            'reset_database': LaunchConfiguration('reset_database'),
        })
        rviz_without_imu_gate = include_launch(
            'rviz.launch.py', condition=IfCondition(use_rviz))

        def on_imu_gate_exit(event, _context):
            if event.returncode != 0:
                reason = (
                    'Corrected IMU readiness gate failed with exit code '
                    f'{event.returncode}'
                )
                return [EmitEvent(event=Shutdown(reason=reason))]
            return [
                LogInfo(msg=(
                    'Corrected IMU readiness gate passed; starting '
                    'RTAB-Map and RViz.')),
                rtabmap,
                rviz,
            ]

        imu_gate_handler = RegisterEventHandler(OnProcessExit(
            target_action=imu_gate,
            on_exit=on_imu_gate_exit,
        ))
        return [
            base,
            robot_tf,
            imu_gate_handler,
            TimerAction(period=1.0, actions=[ekf, imu_gate]),
            TimerAction(
                period=2.0,
                actions=[rtabmap_without_imu_gate, rviz_without_imu_gate],
                condition=UnlessCondition(require_corrected_imu)),
        ]

    def on_can_setup_exit(event, _context):
        if event.returncode != 0:
            reason = f'CAN configuration failed with exit code {event.returncode}'
            return [EmitEvent(event=Shutdown(reason=reason))]
        return [
            LogInfo(msg='CAN configuration succeeded; starting base and arm.'),
            *stack_actions_after_can(),
        ]

    can_setup_handler = RegisterEventHandler(OnProcessExit(
        target_action=can_setup,
        on_exit=on_can_setup_exit,
    ))
    stack_without_can_setup = GroupAction(
        actions=stack_actions_after_can(),
        condition=UnlessCondition(configure_can),
    )

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping',
                              choices=['mapping', 'localization']),
        DeclareLaunchArgument('bunker_port', default_value='auto'),
        DeclareLaunchArgument('arm_can_port', default_value='auto'),
        DeclareLaunchArgument(
            'arm_can_usb_serial', default_value='FRONT_PIPER_ADAPTER_SERIAL'),
        DeclareLaunchArgument(
            'bunker_can_usb_serial', default_value='BUNKER_ADAPTER_SERIAL'),
        DeclareLaunchArgument('serial_no', default_value=''),
        DeclareLaunchArgument('enable_pointcloud', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('configure_can', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('start_base', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('start_arm', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('start_rear_arm', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument(
            'rear_arm_can_port', default_value='',
            description='Confirmed CAN interface for the physical rear PiPER.'),
        DeclareLaunchArgument(
            'joint_states_topic',
            default_value='/piper/feedback/joint_states',
            description='Unprefixed front PiPER feedback topic.'),
        DeclareLaunchArgument(
            'rear_joint_states_topic',
            default_value='/piper_rear/feedback/joint_states',
            description='Unprefixed rear PiPER feedback topic.'),
        DeclareLaunchArgument(
            'use_locked_arm_state_fallback', default_value='false',
            choices=['true', 'false']),
        DeclareLaunchArgument('start_camera', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument(
            'imu_gate_timeout', default_value='30.0',
            description=(
                'Seconds to wait for the first valid bias-corrected IMU '
                'message before shutting down.')),
        DeclareLaunchArgument(
            'require_corrected_imu', default_value='false',
            choices=['true', 'false'],
            description=(
                'Require the optional bias-corrected camera IMU before '
                'RTAB-Map. False uses Bunker wheel linear/yaw velocity.')),
        DeclareLaunchArgument(
            'reset_database', default_value='false', choices=['true', 'false'],
            description='Destructive only when explicitly true in mapping mode.'),
        DeclareLaunchArgument('database_path', default_value=default_database_path()),
        DeclareLaunchArgument(
            'arm_mapping_pose', default_value='false', choices=['true', 'false'],
            description='OPT-IN command of the provisional physical PiPER pose.'),
        DeclareLaunchArgument(
            'rear_arm_mapping_pose', default_value='false',
            choices=['true', 'false'],
            description='OPT-IN command of the mirrored rear PiPER pose.'),
        DeclareLaunchArgument(
            'arm_mount_xyz', default_value='0.807474630 0.0 0.545498689'),
        DeclareLaunchArgument(
            'arm_mount_rpy', default_value='-1.483492816 0.029092513 1.573342313'),
        DeclareLaunchArgument(
            'camera_mount_xyz', default_value='-0.0106 -0.0175 -0.0125'),
        DeclareLaunchArgument('camera_mount_rpy', default_value='0.0 0.0 0.0'),
        OpaqueFunction(function=resolve_can_launch_configurations),
        LogInfo(msg=(
            'TF ownership: RTAB-Map map->odom; EKF odom->base_link; '
            'robot_state_publisher base_link->camera_link; RealSense internal TF.')),
        can_setup_handler,
        can_setup,
        camera,
        stack_without_can_setup,
    ])
