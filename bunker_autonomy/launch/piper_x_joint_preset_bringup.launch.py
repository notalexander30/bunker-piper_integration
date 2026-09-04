"""Start one PiPER-X driver on can3 and optionally run a YAML preset."""

import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def validate_piper_x_can(context):
    interface = LaunchConfiguration('can_port').perform(context).strip()
    result = subprocess.run(
        ['ip', '-details', 'link', 'show', 'dev', interface],
        check=False,
        capture_output=True,
        text=True,
    )
    details = result.stdout
    if result.returncode != 0 or 'link/can' not in details:
        raise RuntimeError(
            f"PiPER-X CAN interface '{interface}' does not exist or is not CAN. "
            'Expected PiPER-X on can3.')
    if not re.search(r'<[^>]*\bUP\b[^>]*>', details):
        raise RuntimeError(
            f"PiPER-X CAN interface '{interface}' is DOWN. Run: "
            f'ip link set {interface} type can bitrate 1000000 restart-ms 100 && '
            f'ip link set {interface} up')
    if not re.search(r'\bbitrate\s+1000000\b', details):
        raise RuntimeError(
            f"PiPER-X CAN interface '{interface}' is not at 1000000 bit/s. Run: "
            f'ip link set {interface} down && '
            f'ip link set {interface} type can bitrate 1000000 restart-ms 100 && '
            f'ip link set {interface} up')
    return [LogInfo(msg=f'PiPER-X CAN preflight passed: {interface} is UP at 1000000 bit/s.')]


def generate_launch_description():
    autonomy_share = get_package_share_directory('bunker_autonomy')
    piper_share = get_package_share_directory('agx_arm_ctrl')
    driver_launch = f'{piper_share}/launch/start_single_agx_arm.launch.py'
    default_preset_file = os.path.join(
        autonomy_share, 'config', 'piper_x_joint_preset.yaml')

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(driver_launch),
        launch_arguments={
            'can_port': LaunchConfiguration('can_port'),
            'arm_type': 'piper_x',
            'effector_type': LaunchConfiguration('effector_type'),
            'namespace': 'piper_x',
            'auto_enable': 'false',
            'control_enabled': 'false',
            'speed_percent': LaunchConfiguration('speed_percent'),
            'fast_mode': 'false',
        }.items(),
    )

    preset_runner = Node(
        package='bunker_autonomy',
        executable='piper_x_joint_preset',
        name='piper_x_joint_preset',
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_preset')),
        parameters=[{
            'trigger_on_start': True,
            'allow_motion': ParameterValue(LaunchConfiguration('allow_motion'), value_type=bool),
            'preset_file': LaunchConfiguration('preset_file'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'can_port',
            default_value='can3',
            description='SocketCAN interface for the rear PiPER-X arm. Current assignment: can3.'),
        DeclareLaunchArgument(
            'speed_percent',
            default_value='5',
            description='Low speed limit for first PiPER-X preset tests.'),
        DeclareLaunchArgument(
            'effector_type',
            default_value='none',
            choices=['none', 'agx_gripper', 'revo2', 'revo2_touch'],
            description='PiPER-X end effector. Keep none unless a gripper/hand is connected.'),
        DeclareLaunchArgument(
            'preset_file',
            default_value=default_preset_file,
            description='YAML file containing PiPER-X preset_steps.'),
        DeclareLaunchArgument(
            'run_preset',
            default_value='false',
            choices=['true', 'false'],
            description='Run the one-joint YAML preset after feedback/services are ready.'),
        DeclareLaunchArgument(
            'allow_motion',
            default_value='false',
            choices=['true', 'false'],
            description='Second explicit gate required for physical PiPER-X motion.'),
        LogInfo(msg='PiPER-X bringup: can3 driver first, optional YAML preset second.'),
        OpaqueFunction(function=validate_piper_x_can),
        driver,
        TimerAction(period=4.0, actions=[preset_runner]),
    ])
