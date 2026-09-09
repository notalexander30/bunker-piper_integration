# Copyright 2026 Dase Orin
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dual_share = get_package_share_directory('bunker_dual_piper_nav2')
    pose_config = os.path.join(
        get_package_share_directory('bunker_autonomy'),
        'config', 'piper_navigation_pose.yaml')

    arm_can_port = LaunchConfiguration('arm_can_port')
    start_arm_driver = LaunchConfiguration('start_arm_driver')
    start_rear_arm_driver = LaunchConfiguration('start_rear_arm_driver')
    arm_mapping_pose = LaunchConfiguration('arm_mapping_pose')
    rear_arm_mapping_pose = LaunchConfiguration('rear_arm_mapping_pose')
    joint_states_topic = LaunchConfiguration('joint_states_topic')

    arm_driver = Node(
        package='agx_arm_ctrl',
        executable='agx_arm_ctrl_single',
        namespace='piper',
        name='agx_arm_ctrl_single_node',
        condition=IfCondition(start_arm_driver),
        output='screen',
        parameters=[{
            'can_port': arm_can_port,
            'pub_rate': 50,
            'auto_enable': False,
            'fast_mode': False,
            'arm_type': 'piper',
            'speed_percent': 10,
            'enable_timeout': 5.0,
            'effector_type': 'agx_gripper',
            'revo2_type': 'left',
            'tcp_offset': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'gripper_default_effort': 0.20,
            # Feedback is safe; all physical control remains gated off.
            'control_enabled': False,
        }],
        remappings=[('feedback/joint_states', joint_states_topic)],
    )

    rear_arm_driver = Node(
        package='agx_arm_ctrl',
        executable='agx_arm_ctrl_single',
        namespace='piper_rear',
        name='agx_arm_ctrl_single_node',
        condition=IfCondition(start_rear_arm_driver),
        output='screen',
        parameters=[{
            'can_port': LaunchConfiguration('rear_arm_can_port'),
            'pub_rate': 50,
            'auto_enable': False,
            'fast_mode': False,
            'arm_type': 'piper',
            'speed_percent': 10,
            'enable_timeout': 5.0,
            'effector_type': 'agx_gripper',
            'revo2_type': 'left',
            'tcp_offset': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'gripper_default_effort': 0.20,
            'control_enabled': False,
        }],
        remappings=[
            ('feedback/joint_states', LaunchConfiguration(
                'rear_joint_states_topic')),
        ],
    )

    dual_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            dual_share, 'launch', 'description.launch.py')),
        launch_arguments={
            'prefix_joint_states': 'true',
            'publish_default_joint_states': 'false',
            'front_joint_states_topic': joint_states_topic,
            'rear_joint_states_topic': LaunchConfiguration(
                'rear_joint_states_topic'),
            'use_front_joint_feedback': 'true',
            'use_rear_joint_feedback': 'false',
            # Preserve the installed RealSense/RTAB-Map camera frame contract.
            'front_camera_name': 'camera',
            # The live RealSense node is the sole publisher of its optical frames.
            'front_camera_nominal_extrinsics': 'false',
            'front_piper_mount_rpy': '0 0 1.57079632679',
            'rear_piper_mount_rpy': '0 0 1.57079632679',
            'front_camera_xyz': (
                '-0.03325362536889407 -0.04784194427852743 '
                '0.08630445414919316'),
            'front_camera_rpy': (
                '-0.08017405150425999 -0.0031709794581423934 '
                '0.023976595541682157'),
        }.items(),
    )

    # This is deliberately opt-in because it commands real hardware. The saved
    # pose is provisional and must be visually cleared by an operator first.
    pose_manager = Node(
        package='bunker_autonomy',
        executable='piper_navigation_pose',
        # Keep this name aligned with the root key in the parameters file.
        name='piper_navigation_pose',
        condition=IfCondition(arm_mapping_pose),
        output='screen',
        parameters=[pose_config, {
            'trigger_on_start': True,
            'allow_motion': True,
            'feedback_topic': joint_states_topic,
        }],
    )

    rear_pose_manager = Node(
        package='bunker_autonomy',
        executable='piper_navigation_pose',
        name='rear_piper_navigation_pose',
        condition=IfCondition(rear_arm_mapping_pose),
        output='screen',
        parameters=[pose_config, {
            'trigger_on_start': True,
            'allow_motion': True,
            'joint_positions': [-1.6, 0.0, 0.0, 0.0, 0.0, 0.0],
            'feedback_topic': LaunchConfiguration('rear_joint_states_topic'),
            'command_topic': '/piper_rear/control/joint_states',
            'arm_enable_service': '/piper_rear/enable_agx_arm',
            'control_gate_service': '/piper_rear/control_enable',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('arm_can_port', default_value=''),
        DeclareLaunchArgument('start_arm_driver', default_value='true'),
        DeclareLaunchArgument('start_rear_arm_driver', default_value='false'),
        DeclareLaunchArgument(
            'rear_arm_can_port', default_value='',
            description='Confirmed CAN interface for the physical rear PiPER.'),
        DeclareLaunchArgument(
            'joint_states_topic', default_value='/piper/feedback/joint_states',
            description=(
                'Unprefixed front PiPER feedback consumed by the dual-arm '
                'joint-state prefixer.')),
        DeclareLaunchArgument(
            'rear_joint_states_topic',
            default_value='/piper_rear/feedback/joint_states',
            description='Unprefixed rear PiPER feedback topic.'),
        DeclareLaunchArgument(
            'use_locked_arm_state_fallback', default_value='false',
            choices=['true', 'false'],
            description=(
                'Publish documented stationary joint values when live PiPER '
                'feedback is unavailable. This never commands the arm.')),
        DeclareLaunchArgument(
            'arm_mapping_pose', default_value='false',
            description='OPT-IN physical move to the provisional saved PiPER pose.'),
        DeclareLaunchArgument(
            'rear_arm_mapping_pose', default_value='false',
            description='OPT-IN physical rear PiPER move to its mirrored pose.'),
        DeclareLaunchArgument(
            'arm_mount_xyz', default_value='0.807474630 0.0 0.545498689'),
        DeclareLaunchArgument(
            'arm_mount_rpy', default_value='-1.483492816 0.029092513 1.573342313'),
        DeclareLaunchArgument(
            'camera_mount_xyz', default_value='-0.0106 -0.0175 -0.0125'),
        DeclareLaunchArgument('camera_mount_rpy', default_value='0.0 0.0 0.0'),
        arm_driver,
        rear_arm_driver,
        dual_description,
        TimerAction(period=4.0, actions=[pose_manager]),
        TimerAction(period=4.0, actions=[rear_pose_manager]),
    ])
