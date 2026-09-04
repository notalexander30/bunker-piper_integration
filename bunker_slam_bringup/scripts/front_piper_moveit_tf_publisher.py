#!/usr/bin/env python3
"""Publish TF for the unprefixed front PiPER MoveIt model.

The main Bunker robot_state_publisher uses the combined Trystan URDF with
front_piper_* link names.  The Illiyas/AgileX MoveIt model intentionally uses
the upstream unprefixed names (base_link, link1..link6, tcp_link, gripper_*).
RViz and MoveIt therefore need a second robot_state_publisher for that model.

The AgileX URDF contains a fixed world -> base_link joint.  This script removes
that joint before starting robot_state_publisher so the global TF tree keeps the
real robot owner of base_link (odom/map -> base_link) and this publisher only
adds the arm subtree under base_link.  A separate identity world -> map static
TF is published for MoveIt/RViz compatibility, so the MoveIt planning frame can
resolve into the Nav2 tree without giving base_link two parents.
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).parent))
from front_piper_move_group_only import build_moveit_config  # noqa: E402


def robot_description_without_world_parent() -> str:
    moveit_config, _ = build_moveit_config()
    robot_description = moveit_config.robot_description["robot_description"]
    root = ET.fromstring(robot_description)

    for joint in list(root.findall("joint")):
        if joint.attrib.get("name") == "world_to_base_link":
            root.remove(joint)

    world_link_is_used = False
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if (
            parent is not None
            and parent.attrib.get("link") == "world"
            or child is not None
            and child.attrib.get("link") == "world"
        ):
            world_link_is_used = True
            break

    if not world_link_is_used:
        for link in list(root.findall("link")):
            if link.attrib.get("name") == "world":
                root.remove(link)

    return ET.tostring(root, encoding="unicode")


def main() -> int:
    world_to_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="front_piper_moveit_world_to_map_tf",
        output="screen",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "world",
            "map",
        ],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="front_piper_moveit_robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_without_world_parent(),
                "publish_frequency": 30.0,
                "ignore_timestamp": True,
            }
        ],
        remappings=[
            ("joint_states", "/front_piper/feedback/joint_states"),
        ],
    )
    launch_service = LaunchService()
    launch_service.include_launch_description(
        LaunchDescription([world_to_map, robot_state_publisher])
    )
    return launch_service.run()


if __name__ == "__main__":
    raise SystemExit(main())
