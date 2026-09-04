#!/usr/bin/env python3
"""Integrated MoveIt model for the real front PiPER branch in Nav-Man.

MoveIt must use the same prefixed links/joints that the Bunker robot URDF
publishes on TF.  Do not use the standalone AgileX base_link/link1 model here.
"""

from pathlib import Path


DESCRIPTION_PATHS = [
    Path("/home/dase-orin/ros2_ws/src/bunker_slam_bringup/descriptions/nav_man_full_robot.urdf"),
    Path("/ros2_ws/src/bunker_slam_bringup/descriptions/nav_man_full_robot.urdf"),
    Path(
        "/ros2_ws/install/bunker_slam_bringup/share/"
        "bunker_slam_bringup/descriptions/nav_man_full_robot.urdf"
    ),
]


def robot_description() -> str:
    for path in DESCRIPTION_PATHS:
        if path.is_file():
            return path.read_text()
    raise FileNotFoundError(
        "nav_man_full_robot.urdf was not found in source or install descriptions"
    )


def robot_description_semantic() -> str:
    return """<?xml version="1.0"?>
<robot name="bunker_dual_piper_d435i">
  <group name="arm">
    <chain base_link="front_piper_base_link" tip_link="front_piper_flange_link"/>
  </group>
  <group name="gripper">
    <link name="front_piper_gripper_base"/>
    <link name="front_piper_link7"/>
    <link name="front_piper_link8"/>
    <joint name="front_piper_joint7"/>
    <joint name="front_piper_joint8"/>
  </group>
  <group name="rear_arm">
    <chain base_link="rear_piper_base_link" tip_link="rear_piper_flange_link"/>
  </group>
  <group name="rear_gripper">
    <link name="rear_piper_gripper_base"/>
    <link name="rear_piper_link7"/>
    <link name="rear_piper_link8"/>
    <joint name="rear_piper_joint7"/>
    <joint name="rear_piper_joint8"/>
  </group>
  <end_effector
    name="gripper_eef"
    parent_link="front_piper_flange_link"
    parent_group="arm"
    group="gripper"/>
  <end_effector
    name="rear_gripper_eef"
    parent_link="rear_piper_flange_link"
    parent_group="rear_arm"
    group="rear_gripper"/>
  <group_state name="home" group="arm">
    <joint name="front_piper_joint1" value="0"/>
    <joint name="front_piper_joint2" value="0"/>
    <joint name="front_piper_joint3" value="0"/>
    <joint name="front_piper_joint4" value="0"/>
    <joint name="front_piper_joint5" value="0"/>
    <joint name="front_piper_joint6" value="0"/>
  </group_state>
  <group_state name="home" group="rear_arm">
    <joint name="rear_piper_joint1" value="0"/>
    <joint name="rear_piper_joint2" value="0"/>
    <joint name="rear_piper_joint3" value="0"/>
    <joint name="rear_piper_joint4" value="0"/>
    <joint name="rear_piper_joint5" value="0"/>
    <joint name="rear_piper_joint6" value="0"/>
  </group_state>
  <disable_collisions link1="base_link" link2="base_footprint" reason="Fixed"/>
  <disable_collisions link1="base_link" link2="upper_frame_link" reason="Fixed"/>
  <disable_collisions link1="base_link" link2="front_mounting_plate_link" reason="Never"/>
  <disable_collisions link1="base_link" link2="rear_mounting_plate_link" reason="Never"/>
  <disable_collisions link1="upper_frame_link" link2="front_mounting_plate_link" reason="Fixed"/>
  <disable_collisions link1="upper_frame_link" link2="rear_mounting_plate_link" reason="Fixed"/>
  <disable_collisions link1="front_mounting_plate_link" link2="front_piper_base_link" reason="Fixed"/>
  <disable_collisions link1="front_mounting_plate_link" link2="front_piper_link1" reason="Never"/>
  <disable_collisions link1="rear_mounting_plate_link" link2="rear_piper_base_link" reason="Fixed"/>
  <disable_collisions link1="rear_mounting_plate_link" link2="rear_piper_link1" reason="Never"/>
  <disable_collisions link1="front_piper_base_link" link2="front_piper_link1" reason="Adjacent"/>
  <disable_collisions link1="front_piper_base_link" link2="front_piper_link2" reason="Never"/>
  <disable_collisions link1="front_piper_base_link" link2="front_piper_link3" reason="Never"/>
  <disable_collisions link1="front_piper_link1" link2="front_piper_link2" reason="Adjacent"/>
  <disable_collisions link1="front_piper_link1" link2="front_piper_link3" reason="Never"/>
  <disable_collisions link1="front_piper_link2" link2="front_piper_link3" reason="Adjacent"/>
  <disable_collisions link1="front_piper_link2" link2="front_piper_link4" reason="Never"/>
  <disable_collisions link1="front_piper_link3" link2="front_piper_link4" reason="Adjacent"/>
  <disable_collisions link1="front_piper_link3" link2="front_piper_link5" reason="Never"/>
  <disable_collisions link1="front_piper_link3" link2="front_piper_link6" reason="Never"/>
  <disable_collisions link1="front_piper_link4" link2="front_piper_link5" reason="Adjacent"/>
  <disable_collisions link1="front_piper_link4" link2="front_piper_link6" reason="Never"/>
  <disable_collisions link1="front_piper_link5" link2="front_piper_link6" reason="Adjacent"/>
  <disable_collisions link1="front_piper_link6" link2="front_piper_flange_link" reason="Adjacent"/>
  <disable_collisions link1="front_piper_flange_link" link2="front_piper_gripper_base" reason="Adjacent"/>
  <disable_collisions link1="front_piper_gripper_base" link2="front_piper_link7" reason="Adjacent"/>
  <disable_collisions link1="front_piper_gripper_base" link2="front_piper_link8" reason="Adjacent"/>
  <disable_collisions link1="front_piper_link7" link2="front_piper_link8" reason="Adjacent"/>
  <disable_collisions link1="rear_piper_base_link" link2="rear_piper_link1" reason="Adjacent"/>
  <disable_collisions link1="rear_piper_base_link" link2="rear_piper_link2" reason="Never"/>
  <disable_collisions link1="rear_piper_base_link" link2="rear_piper_link3" reason="Never"/>
  <disable_collisions link1="rear_piper_link1" link2="rear_piper_link2" reason="Adjacent"/>
  <disable_collisions link1="rear_piper_link1" link2="rear_piper_link3" reason="Never"/>
  <disable_collisions link1="rear_piper_link2" link2="rear_piper_link3" reason="Adjacent"/>
  <disable_collisions link1="rear_piper_link2" link2="rear_piper_link4" reason="Never"/>
  <disable_collisions link1="rear_piper_link3" link2="rear_piper_link4" reason="Adjacent"/>
  <disable_collisions link1="rear_piper_link3" link2="rear_piper_link5" reason="Never"/>
  <disable_collisions link1="rear_piper_link3" link2="rear_piper_link6" reason="Never"/>
  <disable_collisions link1="rear_piper_link4" link2="rear_piper_link5" reason="Adjacent"/>
  <disable_collisions link1="rear_piper_link4" link2="rear_piper_link6" reason="Never"/>
  <disable_collisions link1="rear_piper_link5" link2="rear_piper_link6" reason="Adjacent"/>
  <disable_collisions link1="rear_piper_link6" link2="rear_piper_flange_link" reason="Adjacent"/>
  <disable_collisions link1="rear_piper_flange_link" link2="rear_piper_gripper_base" reason="Adjacent"/>
  <disable_collisions link1="rear_piper_gripper_base" link2="rear_piper_link7" reason="Adjacent"/>
  <disable_collisions link1="rear_piper_gripper_base" link2="rear_piper_link8" reason="Adjacent"/>
  <disable_collisions link1="rear_piper_link7" link2="rear_piper_link8" reason="Adjacent"/>
</robot>
"""


def kinematics() -> dict:
    return {
        "robot_description_kinematics": {
            "arm": {
                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                "kinematics_solver_search_resolution": 0.005,
                "kinematics_solver_timeout": 0.05,
                "kinematics_solver_attempts": 3,
            }
        }
    }


def joint_limits() -> dict:
    return {
        "robot_description_planning": {
            "joint_limits": {
                f"front_piper_joint{i}": {
                    "has_velocity_limits": True,
                    "max_velocity": 1.0,
                    "has_acceleration_limits": True,
                    "max_acceleration": 1.0,
                }
                for i in range(1, 7)
            }
        }
    }


def trajectory_execution() -> dict:
    return {
        "moveit_manage_controllers": True,
        "moveit_controller_manager": (
            "moveit_simple_controller_manager/MoveItSimpleControllerManager"
        ),
        "moveit_simple_controller_manager": {
            "controller_names": ["arm_controller"],
            "arm_controller": {
                "type": "FollowJointTrajectory",
                "joints": [f"front_piper_joint{i}" for i in range(1, 7)],
                "action_ns": "follow_joint_trajectory",
                "default": True,
            },
        },
    }


def planning_pipelines() -> dict:
    return {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        },
    }


def moveit_parameters() -> list[dict]:
    return [
        {"robot_description": robot_description()},
        {"robot_description_semantic": robot_description_semantic()},
        kinematics(),
        joint_limits(),
        trajectory_execution(),
        planning_pipelines(),
    ]
