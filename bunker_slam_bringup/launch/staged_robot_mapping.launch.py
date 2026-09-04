# Copyright 2026 Dase Orin
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""
Start CAN, PiPER navigation pose, then Bunker/EKF/URDF/RTAB-Map.

The Bunker driver is not started unless the PiPER reaches the reviewed joint
pose and the PiPER control gate closes successfully.
"""

import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('bunker_slam_bringup')
    autonomy_share = get_package_share_directory('bunker_autonomy')
    piper_share = get_package_share_directory('agx_arm_ctrl')

    configure_script = os.path.join(
        get_package_prefix('bunker_slam_bringup'),
        'lib', 'bunker_slam_bringup', 'configure_can.sh')
    slam_launch = os.path.join(
        bringup_share, 'launch', 'slam_bringup.launch.py')
    piper_launch = os.path.join(
        piper_share, 'launch', 'start_single_agx_arm.launch.py')
    pose_config = os.path.join(
        autonomy_share, 'config', 'piper_navigation_pose.yaml')

    configure_can = ExecuteProcess(
        cmd=[
            configure_script,
            LaunchConfiguration('arm_can_port'),
            LaunchConfiguration('bunker_can_port'),
        ],
        output='screen',
    )

    piper_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(piper_launch),
        launch_arguments={
            'namespace': 'piper',
            'can_port': LaunchConfiguration('arm_can_port'),
            'arm_type': 'piper',
            'effector_type': 'agx_gripper',
            'auto_enable': 'false',
            'control_enabled': 'false',
            'speed_percent': LaunchConfiguration('piper_speed_percent'),
            'gripper_default_effort': '0.20',
        }.items(),
    )

    pose_manager = Node(
        package='bunker_autonomy',
        executable='piper_navigation_pose',
        name='piper_navigation_pose',
        output='screen',
        parameters=[
            pose_config,
            {
                'trigger_on_start': True,
                'allow_motion': ParameterValue(
                    LaunchConfiguration('allow_arm_motion'), value_type=bool),
            },
        ],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            'mode': LaunchConfiguration('mode'),
            'database_path': LaunchConfiguration('database_path'),
            'reset_database': LaunchConfiguration('reset_database'),
            'bunker_port': LaunchConfiguration('bunker_can_port'),
            'arm_can_port': LaunchConfiguration('arm_can_port'),
            'configure_can': 'false',
            'start_camera': 'false',
            'start_base': 'true',
            # The staged PiPER driver remains alive after pose initialization.
            'start_arm': 'false',
            'joint_states_topic': '/piper/feedback/joint_states',
            'use_locked_arm_state_fallback': 'false',
            'arm_mapping_pose': 'false',
            'require_corrected_imu': 'false',
            'enable_pointcloud': 'false',
            'use_rviz': 'false',
        }.items(),
    )

    def after_pose(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason=(
                'PiPER did not reach and secure the reviewed navigation pose; '
                f'pose manager exit code {event.returncode}. Bunker not started.'
            )))]
        return [
            LogInfo(msg=(
                'PiPER navigation pose verified and control gate closed; '
                'starting Bunker feedback, EKF, URDF/TF and RTAB-Map.')),
            slam,
        ]

    pose_exit_handler = RegisterEventHandler(OnProcessExit(
        target_action=pose_manager,
        on_exit=after_pose,
    ))

    def after_can(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason=(
                f'CAN setup failed with exit code {event.returncode}; '
                'no robot driver was started.'
            )))]
        return [
            LogInfo(msg=(
                'Both CAN links are configured. Starting feedback-only PiPER '
                'driver, then the guarded navigation-pose manager.')),
            piper_driver,
            TimerAction(period=4.0, actions=[pose_manager]),
        ]

    can_exit_handler = RegisterEventHandler(OnProcessExit(
        target_action=configure_can,
        on_exit=after_can,
    ))

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping',
                              choices=['mapping', 'localization']),
        DeclareLaunchArgument('database_path'),
        DeclareLaunchArgument('reset_database', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('arm_can_port'),
        DeclareLaunchArgument('bunker_can_port'),
        DeclareLaunchArgument('piper_speed_percent', default_value='10'),
        DeclareLaunchArgument(
            'allow_arm_motion', default_value='false',
            choices=['true', 'false'],
            description=(
                'Explicit physical-motion gate for the saved PiPER pose '
                '[-1.6, 0, 0, 0, 0, 0] rad.')),
        LogInfo(msg=(
            'Staged robot startup: configure both CAN links -> initialize and '
            'verify PiPER -> start Bunker/EKF/URDF/RTAB-Map.')),
        pose_exit_handler,
        can_exit_handler,
        configure_can,
    ])
