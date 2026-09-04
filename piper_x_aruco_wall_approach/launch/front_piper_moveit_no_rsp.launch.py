import sys
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


agx_moveit_launch_dir = Path(get_package_share_directory("agx_arm_moveit")) / "launch"
sys.path.insert(0, str(agx_moveit_launch_dir))

from _moveit_config_builder import build_moveit_config  # noqa: E402


def _build_ros2_controllers_file(namespace: str, arm_type: str, effector_type: str) -> str:
    arm_joints = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    if arm_type == "nero":
        arm_joints.append("joint7")

    ns = namespace.strip("/")
    cm_node = f"/{ns}/controller_manager" if ns else "/controller_manager"
    arm_controller = f"/{ns}/arm_controller" if ns else "/arm_controller"

    controllers = {
        "arm_controller": {
            "type": "joint_trajectory_controller/JointTrajectoryController",
        },
        "joint_state_broadcaster": {
            "type": "joint_state_broadcaster/JointStateBroadcaster",
        },
    }
    config = {
        cm_node: {
            "ros__parameters": {
                "update_rate": 200,
                **controllers,
            },
        },
        arm_controller: {
            "ros__parameters": {
                "joints": arm_joints,
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
            },
        },
    }

    if effector_type == "agx_gripper":
        config[cm_node]["ros__parameters"]["gripper_controller"] = {
            "type": "joint_trajectory_controller/JointTrajectoryController",
        }
        gripper_controller = f"/{ns}/gripper_controller" if ns else "/gripper_controller"
        config[gripper_controller] = {
            "ros__parameters": {
                "joints": ["gripper"],
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
            },
        }

    tmp = tempfile.NamedTemporaryFile(
        mode="w",
        suffix=".yaml",
        prefix="ros2_controllers_no_rsp_",
        delete=False,
    )
    yaml.safe_dump(config, tmp, default_flow_style=False)
    tmp.close()
    return tmp.name


def _launch(context):
    namespace = LaunchConfiguration("namespace").perform(context)
    arm_type = LaunchConfiguration("arm_type").perform(context)
    effector_type = LaunchConfiguration("effector_type").perform(context)
    control_topic = LaunchConfiguration("control_topic")

    moveit_config = build_moveit_config(context)
    ros2_controllers_yaml = _build_ros2_controllers_file(namespace, arm_type, effector_type)

    return [
        GroupAction(
            actions=[
                PushRosNamespace(namespace),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(agx_moveit_launch_dir / "move_group.launch.py")),
                    launch_arguments={
                        "arm_type": arm_type,
                        "effector_type": effector_type,
                        "revo2_type": LaunchConfiguration("revo2_type"),
                        "tcp_offset": LaunchConfiguration("tcp_offset"),
                        "follow": "true",
                        "feedback_topic": LaunchConfiguration("feedback_topic"),
                        "control_topic": control_topic,
                    }.items(),
                ),
                Node(
                    package="controller_manager",
                    executable="ros2_control_node",
                    parameters=[
                        moveit_config.robot_description,
                        ros2_controllers_yaml,
                    ],
                    remappings=[("joint_states", control_topic)],
                    output="screen",
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(agx_moveit_launch_dir / "spawn_controllers.launch.py"))
                ),
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="front_piper"),
        DeclareLaunchArgument("arm_type", default_value="piper_x"),
        DeclareLaunchArgument("effector_type", default_value="agx_gripper"),
        DeclareLaunchArgument("revo2_type", default_value="left"),
        DeclareLaunchArgument("tcp_offset", default_value="[0.0, 0.0, 0.1425, 0.0, 0.0, 0.0]"),
        DeclareLaunchArgument("feedback_topic", default_value="/front_piper/feedback/joint_states"),
        DeclareLaunchArgument("control_topic", default_value="/front_piper/control/joint_states"),
        OpaqueFunction(function=_launch),
    ])
