"""Start one PiPER driver and return it to the reviewed navigation pose."""

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


def validate_piper_can(context):
    """Fail before driver startup when the selected link cannot answer PiPER."""
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
            f"PiPER CAN interface '{interface}' does not exist or is not CAN. "
            'Run the CAN resolver/configuration command from the runbook.')
    if not re.search(r'<[^>]*\bUP\b[^>]*>', details):
        raise RuntimeError(
            f"PiPER CAN interface '{interface}' is DOWN. Run: "
            'ros2 run bunker_slam_bringup configure_can.sh can3 can4 can2')
    if not re.search(r'\bbitrate\s+1000000\b', details):
        raise RuntimeError(
            f"PiPER CAN interface '{interface}' is not at 1000000 bit/s. "
            'Run: ros2 run bunker_slam_bringup configure_can.sh can3 can4 can2')
    return [LogInfo(msg=(
        f'PiPER CAN preflight passed: {interface} is UP at 1000000 bit/s.'))]


def generate_launch_description():
    autonomy_share = get_package_share_directory('bunker_autonomy')
    piper_share = get_package_share_directory('agx_arm_ctrl')
    pose_config = os.path.join(autonomy_share, 'config', 'piper_navigation_pose.yaml')
    driver_launch = os.path.join(piper_share, 'launch', 'start_single_agx_arm.launch.py')

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(driver_launch),
        launch_arguments={
            'can_port': LaunchConfiguration('can_port'),
            'arm_type': 'piper',
            # Official PiPER parallel gripper.  The mission node still requires
            # live feedback before it can issue its one-shot indicator command.
            'effector_type': 'agx_gripper',
            'namespace': 'piper',
            # The pose manager enables the arm only after feedback arrives.
            'auto_enable': 'false',
            'control_enabled': 'false',
            'speed_percent': LaunchConfiguration('speed_percent'),
            'gripper_default_effort': '0.20',
        }.items(),
    )
    pose_manager = Node(
        package='bunker_autonomy', executable='piper_navigation_pose',
        name='piper_navigation_pose', output='screen',
        condition=IfCondition(LaunchConfiguration('initialize_pose')),
        parameters=[pose_config, {
            'trigger_on_start': True,
            'allow_motion': ParameterValue(
                LaunchConfiguration('allow_motion'), value_type=bool),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'can_port', default_value='can3',
            description=(
                'Serial-verified PiPER SocketCAN interface. Resolve it before '
                'launch; the current rear-arm navigation adapter is can3.')),
        DeclareLaunchArgument(
            'speed_percent', default_value='25',
            description='PiPER speed limit for the initialization move.'),
        DeclareLaunchArgument(
            'initialize_pose', default_value='false',
            choices=['true', 'false'],
            description='Start the saved-pose manager after feedback begins.'),
        DeclareLaunchArgument(
            'allow_motion', default_value='false',
            choices=['true', 'false'],
            description='Second explicit gate required for physical motion.'),
        LogInfo(msg='PiPER navigation bring-up: feedback first, then the saved low-speed pose.'),
        OpaqueFunction(function=validate_piper_can),
        driver,
        # Allow the CAN feedback thread and services to become ready first.
        TimerAction(period=4.0, actions=[pose_manager]),
    ])
