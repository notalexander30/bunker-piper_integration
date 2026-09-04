#!/usr/bin/env python3

import argparse
import json
import math
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from fastapi import Depends, FastAPI, Header, HTTPException, Request, status
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty, SetBool
from trajectory_msgs.msg import JointTrajectoryPoint
import uvicorn

from piper_x_aruco_wall_approach.srv import RunMarkerTask, SearchMarker

try:
    from controller_manager_msgs.srv import ListHardwareComponents
except ImportError:  # pragma: no cover - only used on systems without ros2_control msgs installed.
    ListHardwareComponents = None  # type: ignore[assignment]


SUPPORTED_ARMS = {"front", "rear"}


def normalize_arm(arm: str) -> str:
    value = str(arm or "front").strip().lower()
    aliases = {
        "front_piper": "front",
        "/front_piper": "front",
        "rear_piper": "rear",
        "/rear_piper": "rear",
        "back": "rear",
        "back_piper": "rear",
        "/back_piper": "rear",
    }
    value = aliases.get(value, value)
    if value not in SUPPORTED_ARMS:
        raise ValueError("arm must be 'front' or 'rear'")
    return value


def arm_namespace(arm: str) -> str:
    return f"/{normalize_arm(arm)}_piper"


def arm_joint_state_topic(arm: str) -> str:
    return f"{arm_namespace(arm)}/feedback/joint_states"


def arm_trajectory_action(arm: str) -> str:
    return f"{arm_namespace(arm)}/arm_controller/follow_joint_trajectory"


def arm_enable_service(arm: str) -> str:
    return f"{arm_namespace(arm)}/enable_agx_arm"


def nav_pose_positions(arm: str) -> list[float]:
    return [-1.6, 0.0, 0.0, 0.0, 0.0, 0.0] if normalize_arm(arm) == "front" else [1.6, 0.0, 0.0, 0.0, 0.0, 0.0]


def namespace_from_action(action_name: str) -> str:
    parts = action_name.strip("/").split("/")
    if len(parts) <= 2:
        return ""
    return "/" + "/".join(parts[:-2])


def controller_manager_service_from_action(action_name: str) -> str:
    namespace = namespace_from_action(action_name)
    if not namespace:
        return "/controller_manager/list_hardware_components"
    return f"{namespace}/controller_manager/list_hardware_components"


def nav_pose_mapping_settle_s() -> float:
    try:
        value = float(os.environ.get("PIPER_X_NAV_POSE_MAPPING_SETTLE_S", "1.0"))
    except ValueError:
        return 1.0
    if not math.isfinite(value):
        return 1.0
    return max(0.0, min(value, 10.0))


def execution_allowed_from_env() -> bool:
    return os.environ.get("PIPER_TOUCH_ALLOW_EXECUTION", "").strip().lower() in {"1", "true"}


def token_from_env() -> Optional[str]:
    token = os.environ.get("PIPER_TOUCH_API_TOKEN", "").strip()
    return token or None


class MarkerTaskRequest(BaseModel):
    execute: bool = False
    arm: str = Field(default="front")
    pre_clearance_m: float = Field(default=0.05)
    final_clearance_m: float = Field(default=0.005)
    retract_after: bool = True
    retract_distance_m: float = Field(default=0.05)
    final_velocity_scaling: float = Field(default=0.16)
    return_home_after: bool = False
    home_duration_s: float = Field(default=6.0)


class SearchMarkerRequest(BaseModel):
    execute: bool = False
    arm: str = Field(default="front")
    direction: str = Field(default="auto")
    max_steps: int = Field(default=0)


class HomeRequest(BaseModel):
    execute: bool = False
    arm: str = Field(default="front")
    duration_s: float = Field(default=6.0)


class ClearActiveTasksRequest(BaseModel):
    clear_command_lock: bool = False


class SaveHomeRequest(BaseModel):
    pose_name: str = Field(default="manipulation_pose")


@dataclass
class HealthSnapshot:
    ros_ok: bool
    marker_pose_available: bool
    point_cloud_available: bool
    moveit_available: bool
    marker_task_service_available: bool
    home_action_available: bool
    joint_state_available: bool
    configured_marker_id: int
    configured_marker_size_m: float
    execution_allowed: bool
    hardware_ready: bool = True
    hardware_fake: bool = False
    hardware_components: Optional[list[Dict[str, Any]]] = None
    hardware_message: Optional[str] = None
    search_marker_service_available: bool = True
    marker_pose_age_s: Optional[float] = None
    point_cloud_age_s: Optional[float] = None
    marker_pose_header_age_s: Optional[float] = None
    point_cloud_header_age_s: Optional[float] = None
    joint_state_age_s: Optional[float] = None
    joint_state_header_age_s: Optional[float] = None
    marker_timeout_s: Optional[float] = None
    point_cloud_timeout_s: Optional[float] = None
    joint_state_timeout_s: Optional[float] = None

    def as_dict(self) -> Dict[str, Any]:
        camera_ready = self.ros_ok and self.point_cloud_available
        moveit_ready = self.ros_ok and self.moveit_available
        joint_state_ready = self.ros_ok and self.joint_state_available
        marker_visible = self.marker_pose_available
        system_ready = (
            self.ros_ok
            and self.moveit_available
            and self.marker_task_service_available
            and self.search_marker_service_available
            and self.home_action_available
            and self.joint_state_available
            and self.point_cloud_available
        )
        ready_for_search = (
            self.ros_ok
            and self.moveit_available
            and self.search_marker_service_available
            and self.joint_state_available
        )
        ready_for_approach = (
            system_ready
            and marker_visible
        )
        status_value = "ready" if system_ready else "not_ready"
        return {
            "status": status_value,
            "system_ready": system_ready,
            "camera_ready": camera_ready,
            "moveit_ready": moveit_ready,
            "joint_state_ready": joint_state_ready,
            "marker_visible": marker_visible,
            "ready_for_search": ready_for_search,
            "ready_for_approach": ready_for_approach,
            "ros_ok": self.ros_ok,
            "marker_pose_available": self.marker_pose_available,
            "point_cloud_available": self.point_cloud_available,
            "moveit_available": self.moveit_available,
            "marker_task_service_available": self.marker_task_service_available,
            "search_marker_service_available": self.search_marker_service_available,
            "home_action_available": self.home_action_available,
            "joint_state_available": self.joint_state_available,
            "configured_marker_id": self.configured_marker_id,
            "configured_marker_size_m": self.configured_marker_size_m,
            "execution_allowed": self.execution_allowed,
            "physical_execution_ready": self.execution_allowed and self.moveit_available,
            "hardware_ready": self.hardware_ready,
            "hardware_fake": self.hardware_fake,
            "hardware_components": self.hardware_components or [],
            "hardware_message": self.hardware_message,
            "marker_pose_age_s": self.marker_pose_age_s,
            "point_cloud_age_s": self.point_cloud_age_s,
            "marker_pose_header_age_s": self.marker_pose_header_age_s,
            "point_cloud_header_age_s": self.point_cloud_header_age_s,
            "joint_state_age_s": self.joint_state_age_s,
            "joint_state_header_age_s": self.joint_state_header_age_s,
            "marker_timeout_s": self.marker_timeout_s,
            "point_cloud_timeout_s": self.point_cloud_timeout_s,
            "joint_state_timeout_s": self.joint_state_timeout_s,
        }


class RosMarkerTaskAdapter:
    def begin_task(self) -> None:
        return None

    def end_task(self) -> None:
        return None

    def health(self) -> HealthSnapshot:
        raise NotImplementedError

    def run_task(self, mode: str, request: MarkerTaskRequest) -> Dict[str, Any]:
        raise NotImplementedError

    def ensure_arm_enabled(self) -> Dict[str, Any]:
        raise NotImplementedError

    def search_marker(self, request: SearchMarkerRequest) -> Dict[str, Any]:
        raise NotImplementedError

    def go_home(self, request: HomeRequest) -> Dict[str, Any]:
        raise NotImplementedError

    def save_home(self, request: SaveHomeRequest) -> Dict[str, Any]:
        raise NotImplementedError

    def go_previous(self, request: HomeRequest) -> Dict[str, Any]:
        raise NotImplementedError

    def save_previous(self) -> Dict[str, Any]:
        raise NotImplementedError

    def go_found_marker(self, request: HomeRequest) -> Dict[str, Any]:
        raise NotImplementedError

    def go_nav_pose(self, request: HomeRequest) -> Dict[str, Any]:
        raise NotImplementedError

    def save_found_marker(self) -> Dict[str, Any]:
        raise NotImplementedError

    def active_task_status(self) -> Dict[str, Any]:
        return {"active_task_count": 0, "finished": True}

    def clear_active_tasks(self) -> Dict[str, Any]:
        return {
            "success": True,
            "stage": "active_tasks_cleared",
            "message": "no ROS task bookkeeping was active",
            "active_task_count": 0,
            "finished": True,
        }

    def pause_mapping_for_manipulation(self) -> Dict[str, Any]:
        return {
            "success": False,
            "available": False,
            "service": "/rtabmap/pause",
            "message": "ROS adapter not initialized",
        }

    def resume_mapping_for_navigation(self) -> Dict[str, Any]:
        return {
            "success": False,
            "available": False,
            "service": "/rtabmap/resume",
            "message": "ROS adapter not initialized",
        }


class MarkerTaskBridge(Node, RosMarkerTaskAdapter):
    def __init__(
        self,
        marker_pose_topic: str,
        point_cloud_topic: str,
        marker_id: int,
        marker_size_m: float,
        marker_timeout_s: float,
        point_cloud_timeout_s: float,
        home_pose_file: str,
        previous_pose_file: str,
        found_marker_pose_file: str,
        joint_state_topic: str,
        joint_state_timeout_s: float,
        trajectory_action: str,
        enable_service: str,
        controller_manager_service: Optional[str] = None,
        command_joint_prefix: str = "front_piper_",
    ):
        super().__init__("piper_touch_marker_api_bridge")
        self.marker_pose_topic = marker_pose_topic
        self.point_cloud_topic = point_cloud_topic
        self.marker_id = marker_id
        self.marker_size_m = marker_size_m
        self.marker_timeout_s = marker_timeout_s
        self.point_cloud_timeout_s = point_cloud_timeout_s
        self.joint_state_topic = joint_state_topic
        self.joint_state_timeout_s = joint_state_timeout_s
        self.trajectory_action = trajectory_action
        self.enable_service = enable_service
        self.command_joint_prefix = str(command_joint_prefix or "front_piper_")
        self._task_state_lock = threading.Lock()
        self._active_task_count = 0
        self._finished_state = False
        self._finished_qos = QoSProfile(depth=1)
        self._finished_qos.reliability = ReliabilityPolicy.RELIABLE
        self._finished_publisher = self.create_publisher(
            Bool, "/manipulation_task/finished", self._finished_qos
        )
        self._publish_finished(False)
        self.create_timer(0.5, self._republish_finished)
        self.controller_manager_service = (
            controller_manager_service
            or controller_manager_service_from_action(self.trajectory_action)
        )
        self.home_pose_file = str(Path(home_pose_file).expanduser())
        self.previous_pose_file = str(Path(previous_pose_file).expanduser())
        self.found_marker_pose_file = str(Path(found_marker_pose_file).expanduser())
        self.home_joint_names, self.home_positions = self._load_home_pose(self.home_pose_file)
        self.previous_joint_names, self.previous_positions = self._try_load_saved_pose(
            self.previous_pose_file,
            "previous",
        )
        self.found_marker_joint_names, self.found_marker_positions = self._try_load_saved_pose(
            self.found_marker_pose_file,
            "found_marker",
        )
        self._marker_received_monotonic_s: Optional[float] = None
        self._cloud_received_monotonic_s: Optional[float] = None
        self._joint_state_received_monotonic_s: Optional[float] = None
        self._marker_header_stamp_s: Optional[float] = None
        self._cloud_header_stamp_s: Optional[float] = None
        self._joint_state_header_stamp_s: Optional[float] = None
        self._latest_joint_state: Optional[JointState] = None
        self._lock = threading.Lock()
        self._client = self.create_client(RunMarkerTask, "/run_marker_task")
        self._search_client = self.create_client(SearchMarker, "/search_marker")
        self._enable_client = self.create_client(SetBool, self.enable_service)
        self._rtabmap_pause_client = self.create_client(Empty, "/rtabmap/pause")
        self._rtabmap_resume_client = self.create_client(Empty, "/rtabmap/resume")
        self._hardware_client = (
            self.create_client(ListHardwareComponents, self.controller_manager_service)
            if ListHardwareComponents is not None
            else None
        )
        self._home_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.trajectory_action,
        )
        self._enable_clients: dict[str, Any] = {"front": self._enable_client}
        self._trajectory_clients: dict[str, Any] = {"front": self._home_client}
        self._hardware_clients: dict[str, Any] = {"front": self._hardware_client}
        self._joint_state_topics_by_arm = {
            "front": arm_joint_state_topic("front"),
            "rear": arm_joint_state_topic("rear"),
        }
        self._joint_states_by_arm: dict[str, JointState] = {}
        self._joint_state_received_by_arm: dict[str, float] = {}
        self.create_subscription(PoseStamped, marker_pose_topic, self._marker_callback, 10)
        self.create_subscription(PointCloud2, point_cloud_topic, self._cloud_callback, 10)
        self.create_subscription(JointState, joint_state_topic, self._joint_state_callback, 10)
        for arm, topic in self._joint_state_topics_by_arm.items():
            if topic != joint_state_topic:
                self.create_subscription(
                    JointState,
                    topic,
                    lambda msg, selected_arm=arm: self._arm_joint_state_callback(selected_arm, msg),
                    10,
                )

    def _publish_finished(self, finished: bool) -> None:
        self._finished_state = bool(finished)
        message = Bool()
        message.data = self._finished_state
        self._finished_publisher.publish(message)

    def _republish_finished(self) -> None:
        with self._task_state_lock:
            self._publish_finished(self._finished_state)

    def _call_empty_service_if_available(self, client, service_name: str) -> Dict[str, Any]:
        if not client.wait_for_service(timeout_sec=0.2):
            return {
                "success": False,
                "available": False,
                "service": service_name,
                "message": f"{service_name} service unavailable",
            }
        future = client.call_async(Empty.Request())
        deadline = time.monotonic() + 1.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return {
                "success": False,
                "available": True,
                "service": service_name,
                "message": f"timed out waiting for {service_name}",
            }
        return {
            "success": future.result() is not None,
            "available": True,
            "service": service_name,
            "message": f"called {service_name}",
        }

    def pause_mapping_for_manipulation(self) -> Dict[str, Any]:
        return self._call_empty_service_if_available(self._rtabmap_pause_client, "/rtabmap/pause")

    def resume_mapping_for_navigation(self) -> Dict[str, Any]:
        return self._call_empty_service_if_available(self._rtabmap_resume_client, "/rtabmap/resume")

    def begin_task(self) -> None:
        with self._task_state_lock:
            self._active_task_count += 1
            self._publish_finished(False)

    def end_task(self) -> None:
        with self._task_state_lock:
            self._active_task_count = max(0, self._active_task_count - 1)
            self._publish_finished(self._active_task_count == 0)

    def active_task_status(self) -> Dict[str, Any]:
        with self._task_state_lock:
            return {
                "active_task_count": self._active_task_count,
                "finished": self._finished_state,
            }

    def clear_active_tasks(self) -> Dict[str, Any]:
        with self._task_state_lock:
            previous_count = self._active_task_count
            self._active_task_count = 0
            self._publish_finished(True)
            return {
                "success": True,
                "stage": "active_tasks_cleared",
                "message": "cleared in-memory PiPER marker task bookkeeping",
                "previous_active_task_count": previous_count,
                "active_task_count": self._active_task_count,
                "finished": self._finished_state,
            }

    def _client_for_arm(self, arm: str):
        selected = normalize_arm(arm)
        if selected == "front":
            return self._home_client
        if selected not in self._trajectory_clients:
            self._trajectory_clients[selected] = ActionClient(
                self,
                FollowJointTrajectory,
                arm_trajectory_action(selected),
            )
        return self._trajectory_clients[selected]

    def _enable_client_for_arm(self, arm: str):
        selected = normalize_arm(arm)
        if selected == "front":
            return self._enable_client
        if selected not in self._enable_clients:
            self._enable_clients[selected] = self.create_client(SetBool, arm_enable_service(selected))
        return self._enable_clients[selected]

    def _hardware_client_for_arm(self, arm: str):
        selected = normalize_arm(arm)
        if ListHardwareComponents is None:
            return None
        if selected not in self._hardware_clients:
            self._hardware_clients[selected] = self.create_client(
                ListHardwareComponents,
                controller_manager_service_from_action(arm_trajectory_action(selected)),
            )
        return self._hardware_clients[selected]

    def hardware_status(self, arm: str = "front") -> Dict[str, Any]:
        selected = normalize_arm(arm)
        hardware_client = self._hardware_client_for_arm(selected)
        service_name = (
            self.controller_manager_service
            if selected == "front"
            else controller_manager_service_from_action(arm_trajectory_action(selected))
        )
        if ListHardwareComponents is None or hardware_client is None:
            return {
                "ready": False,
                "fake": False,
                "components": [],
                "message": "controller_manager_msgs is unavailable, so physical hardware cannot be verified",
            }
        if not hardware_client.wait_for_service(timeout_sec=0.2):
            return {
                "ready": False,
                "fake": False,
                "components": [],
                "message": f"controller manager service unavailable: {service_name}",
            }

        future = hardware_client.call_async(ListHardwareComponents.Request())
        deadline = time.monotonic() + 1.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return {
                "ready": False,
                "fake": False,
                "components": [],
                "message": f"timed out checking hardware through {service_name}",
            }
        response = future.result()
        if response is None:
            return {
                "ready": False,
                "fake": False,
                "components": [],
                "message": f"{service_name} returned no hardware response",
            }

        components = []
        fake = False
        active = False
        for component in getattr(response, "component", []):
            state = getattr(getattr(component, "state", None), "label", "")
            plugin_name = str(getattr(component, "plugin_name", ""))
            class_type = str(getattr(component, "class_type", ""))
            name = str(getattr(component, "name", ""))
            component_type = str(getattr(component, "type", ""))
            components.append(
                {
                    "name": name,
                    "type": component_type,
                    "plugin_name": plugin_name,
                    "class_type": class_type,
                    "state": state,
                }
            )
            active = active or state == "active"
            fake = (
                fake
                or name == "FakeSystem"
                or "mock_components" in plugin_name
                or "GenericSystem" in plugin_name
                or "mock_components" in class_type
                or "GenericSystem" in class_type
            )

        if fake:
            return {
                "ready": True,
                "fake": True,
                "components": components,
                "message": (
                    "AgileX MoveIt demo controller is backed by ros2_control FakeSystem/mock_components; "
                    "allowed as a trajectory sampler into the AGX driver control topic"
                ),
            }
        if not components:
            return {
                "ready": False,
                "fake": False,
                "components": components,
                "message": f"no hardware components reported by {service_name}",
            }
        if not active:
            return {
                "ready": False,
                "fake": False,
                "components": components,
                "message": f"no active hardware components reported by {service_name}",
            }
        return {
            "ready": True,
            "fake": False,
            "components": components,
            "message": "physical ros2_control hardware verified",
        }

    @staticmethod
    def _default_home_pose_file() -> str:
        configured = os.environ.get("PIPER_X_MANIPULATION_POSE_FILE", "").strip()
        if not configured:
            configured = os.environ.get("PIPER_X_HOME_POSE_FILE", "").strip()
        if configured:
            return configured
        if Path("/ros2_ws").is_dir():
            return "/ros2_ws/config/piper_x_manipulation_pose.yaml"
        return str(Path.home() / ".config" / "abotclaw" / "piper_x" / "piper_x_manipulation_pose.yaml")

    @staticmethod
    def _packaged_home_pose_file() -> str:
        return str(
            Path(get_package_share_directory("piper_x_aruco_wall_approach"))
            / "config"
            / "piper_x_manipulation_pose.yaml"
        )

    @staticmethod
    def _default_previous_pose_file(home_pose_file: str) -> str:
        return str(Path(home_pose_file).expanduser().with_name("piper_x_previous_pose.yaml"))

    @staticmethod
    def _default_found_marker_pose_file(home_pose_file: str) -> str:
        return str(Path(home_pose_file).expanduser().with_name("piper_x_found_marker_pose.yaml"))

    @staticmethod
    def _load_saved_pose(path: str, pose_name: str):
        with open(path, "r", encoding="utf-8") as file:
            data = yaml.safe_load(file)
        saved_pose_name = data.get("pose_name")
        accepted_pose_names = {"home", "manipulation_pose"} if pose_name in {"home", "manipulation_pose"} else {pose_name}
        if saved_pose_name not in accepted_pose_names:
            raise ValueError(f"saved pose file {path} has pose_name={saved_pose_name!r}, expected {pose_name!r}")
        joint_state = data.get("joint_state", {})
        names = list(joint_state.get("names", []))
        positions = list(joint_state.get("positions_rad", []))
        if len(names) != len(positions):
            raise ValueError(f"saved pose has mismatched names/positions: {path}")
        selected = [
            (name, float(position))
            for name, position in zip(names, positions)
            if str(name).startswith("joint")
        ]
        if [name for name, _ in selected] != ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]:
            raise ValueError(f"saved pose must contain joint1..joint6 in order: {path}")
        return [name for name, _ in selected], [position for _, position in selected]

    @classmethod
    def _load_home_pose(cls, path: str):
        requested = Path(path).expanduser()
        if requested.is_file():
            return cls._load_saved_pose(str(requested), "home")
        # The packaged pose is only a bootstrap fallback. New saves are
        # written to the persistent runtime path and take precedence.
        try:
            return cls._load_saved_pose(cls._packaged_home_pose_file(), "manipulation_pose")
        except ValueError:
            return cls._load_saved_pose(cls._packaged_home_pose_file(), "home")

    @classmethod
    def _try_load_saved_pose(cls, path: str, pose_name: str):
        if not Path(path).expanduser().is_file():
            return None, None
        return cls._load_saved_pose(path, pose_name)

    @staticmethod
    def _stamp_to_seconds(msg: Any) -> Optional[float]:
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        if stamp is None:
            return None
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _marker_callback(self, msg: PoseStamped) -> None:
        with self._lock:
            self._marker_received_monotonic_s = time.monotonic()
            self._marker_header_stamp_s = self._stamp_to_seconds(msg)

    def _cloud_callback(self, msg: PointCloud2) -> None:
        with self._lock:
            self._cloud_received_monotonic_s = time.monotonic()
            self._cloud_header_stamp_s = self._stamp_to_seconds(msg)

    def _joint_state_callback(self, msg: JointState) -> None:
        with self._lock:
            self._latest_joint_state = msg
            self._joint_state_received_monotonic_s = time.monotonic()
            self._joint_state_header_stamp_s = self._stamp_to_seconds(msg)
            self._joint_states_by_arm["front"] = msg
            self._joint_state_received_by_arm["front"] = self._joint_state_received_monotonic_s

    def _arm_joint_state_callback(self, arm: str, msg: JointState) -> None:
        with self._lock:
            now = time.monotonic()
            self._joint_states_by_arm[normalize_arm(arm)] = msg
            self._joint_state_received_by_arm[normalize_arm(arm)] = now

    def health(self) -> HealthSnapshot:
        now_monotonic_s = time.monotonic()
        now_wall_s = time.time()
        with self._lock:
            marker_received_monotonic_s = self._marker_received_monotonic_s
            cloud_received_monotonic_s = self._cloud_received_monotonic_s
            joint_state_received_monotonic_s = self._joint_state_received_monotonic_s
            marker_header_stamp_s = self._marker_header_stamp_s
            cloud_header_stamp_s = self._cloud_header_stamp_s
            joint_state_header_stamp_s = self._joint_state_header_stamp_s
        marker_age_s = (
            now_monotonic_s - marker_received_monotonic_s
            if marker_received_monotonic_s is not None
            else None
        )
        cloud_age_s = (
            now_monotonic_s - cloud_received_monotonic_s
            if cloud_received_monotonic_s is not None
            else None
        )
        joint_state_age_s = (
            now_monotonic_s - joint_state_received_monotonic_s
            if joint_state_received_monotonic_s is not None
            else None
        )
        marker_header_age_s = (
            now_wall_s - marker_header_stamp_s
            if marker_header_stamp_s is not None
            else None
        )
        cloud_header_age_s = (
            now_wall_s - cloud_header_stamp_s
            if cloud_header_stamp_s is not None
            else None
        )
        joint_state_header_age_s = (
            now_wall_s - joint_state_header_stamp_s
            if joint_state_header_stamp_s is not None
            else None
        )
        marker_fresh = marker_age_s is not None and marker_age_s <= self.marker_timeout_s
        cloud_fresh = cloud_age_s is not None and cloud_age_s <= self.point_cloud_timeout_s
        joint_state_fresh = (
            joint_state_age_s is not None
            and joint_state_age_s <= self.joint_state_timeout_s
        )
        hardware_status = self.hardware_status()
        names_and_types = dict(self.get_service_names_and_types())
        moveit_namespace = namespace_from_action(self.trajectory_action)
        moveit_available = (
            f"{moveit_namespace}/move_action/_action/send_goal" in names_and_types
            or f"{moveit_namespace}/plan_kinematic_path" in names_and_types
        )
        home_action_available = f"{self.trajectory_action}/_action/send_goal" in names_and_types
        return HealthSnapshot(
            ros_ok=rclpy.ok(),
            marker_pose_available=marker_fresh,
            point_cloud_available=cloud_fresh,
            moveit_available=moveit_available,
            marker_task_service_available=self._client.service_is_ready(),
            search_marker_service_available=self._search_client.service_is_ready(),
            home_action_available=home_action_available,
            joint_state_available=joint_state_fresh,
            configured_marker_id=self.marker_id,
            configured_marker_size_m=self.marker_size_m,
            execution_allowed=execution_allowed_from_env(),
            hardware_ready=bool(hardware_status.get("ready", False)),
            hardware_fake=bool(hardware_status.get("fake", False)),
            hardware_components=list(hardware_status.get("components", [])),
            hardware_message=str(hardware_status.get("message", "")),
            marker_pose_age_s=marker_age_s,
            point_cloud_age_s=cloud_age_s,
            marker_pose_header_age_s=marker_header_age_s,
            point_cloud_header_age_s=cloud_header_age_s,
            joint_state_age_s=joint_state_age_s,
            joint_state_header_age_s=joint_state_header_age_s,
            marker_timeout_s=self.marker_timeout_s,
            point_cloud_timeout_s=self.point_cloud_timeout_s,
            joint_state_timeout_s=self.joint_state_timeout_s,
        )

    def run_task(self, mode: str, request: MarkerTaskRequest) -> Dict[str, Any]:
        if normalize_arm(request.arm) != "front":
            raise RuntimeError("marker approach/touch is currently implemented only for arm='front'")
        if not self._client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("ROS service /run_marker_task is not available")

        service_request = RunMarkerTask.Request()
        service_request.mode = mode
        service_request.execute = request.execute
        service_request.pre_clearance_m = request.pre_clearance_m
        service_request.final_clearance_m = request.final_clearance_m
        service_request.retract_distance_m = request.retract_distance_m
        service_request.final_velocity_scaling = request.final_velocity_scaling
        service_request.retract_after = request.retract_after

        future = self._client.call_async(service_request)
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
        response = future.result()
        if response is None:
            raise RuntimeError("ROS service /run_marker_task returned no response")
        return {
            "success": bool(response.success),
            "stage": str(response.stage),
            "message": str(response.message),
            "contact_confirmed": bool(response.contact_confirmed),
            "completion_type": str(response.completion_type),
        }

    def ensure_arm_enabled(self) -> Dict[str, Any]:
        return self.ensure_selected_arm_enabled("front")

    def ensure_selected_arm_enabled(self, arm: str) -> Dict[str, Any]:
        selected = normalize_arm(arm)
        client = self._enable_client_for_arm(selected)
        service_name = self.enable_service if selected == "front" else arm_enable_service(selected)
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f"ROS service {service_name} is not available")

        service_request = SetBool.Request()
        service_request.data = True
        future = client.call_async(service_request)
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            raise TimeoutError(f"timed out waiting for {service_name} response")
        response = future.result()
        if response is None:
            raise RuntimeError(f"ROS service {service_name} returned no response")
        if not bool(response.success):
            raise RuntimeError(f"failed to enable PiPER-X arm: {response.message}")
        return {
            "success": True,
            "stage": "arm_enabled",
            "message": str(response.message),
            "arm": selected,
        }

    def search_marker(self, request: SearchMarkerRequest) -> Dict[str, Any]:
        if normalize_arm(request.arm) != "front":
            raise RuntimeError("marker search is currently implemented only for arm='front'")
        if not self._search_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("ROS service /search_marker is not available")

        service_request = SearchMarker.Request()
        service_request.execute = request.execute
        service_request.direction = request.direction
        service_request.max_steps = request.max_steps
        future = self._search_client.call_async(service_request)
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
        response = future.result()
        if response is None:
            raise RuntimeError("ROS service /search_marker returned no response")
        return {
            "success": bool(response.success),
            "marker_found": bool(response.marker_found),
            "marker_id": int(response.marker_id),
            "found_at_pose": str(response.found_at_pose),
            "steps_used": int(response.poses_checked),
            "poses_checked": int(response.poses_checked),
            "stage": str(response.stage),
            "message": str(response.message),
        }

    def _execute_saved_joint_pose(
        self,
        pose_name: str,
        joint_names,
        positions,
        request: HomeRequest,
    ) -> Dict[str, Any]:
        selected_arm = normalize_arm(request.arm)
        trajectory_action = self.trajectory_action if selected_arm == "front" else arm_trajectory_action(selected_arm)
        client = self._client_for_arm(selected_arm)
        if not client.wait_for_server(timeout_sec=1.0):
            raise RuntimeError(f"action {trajectory_action} is not available")
        if joint_names is None or positions is None:
            raise RuntimeError(f"saved pose '{pose_name}' is not available")
        if not request.execute:
            return {
                "success": True,
                "stage": "complete",
                "message": f"{pose_name} pose command is ready (execute=false)",
                "contact_confirmed": False,
                "completion_type": f"saved_{pose_name}_pose",
                "arm": selected_arm,
                "trajectory_action": trajectory_action,
            }
        goal = FollowJointTrajectory.Goal()
        command_names = self._command_joint_names(joint_names, selected_arm)
        current_positions = self._snapshot(command_names)
        if current_positions is None:
            raise RuntimeError(
                f"physical feedback is stale or missing before {pose_name} trajectory"
            )
        goal.trajectory.joint_names = command_names
        duration = float(request.duration_s)
        start_point = JointTrajectoryPoint()
        start_point.positions = list(current_positions)
        start_point.velocities = [0.0] * len(current_positions)
        start_point.time_from_start = Duration(sec=0, nanosec=0)
        target_point = JointTrajectoryPoint()
        target_point.positions = list(positions)
        target_point.velocities = [0.0] * len(positions)
        target_point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration - int(duration)) * 1_000_000_000),
        )
        goal.trajectory.points = [start_point, target_point]
        goal.goal_time_tolerance = Duration(sec=2, nanosec=0)

        send_future = client.send_goal_async(goal)
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and not send_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not send_future.done():
            raise TimeoutError(f"timed out sending {pose_name} trajectory goal")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"{pose_name} trajectory goal was rejected")

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + max(20.0, request.duration_s + 10.0)
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not result_future.done():
            raise TimeoutError(f"timed out waiting for {pose_name} trajectory result")
        result = result_future.result()
        if result is None:
            raise RuntimeError(f"{pose_name} trajectory returned no result")
        error_code = int(result.result.error_code)
        if error_code != 0:
            return {
                "success": False,
                "stage": f"{pose_name}_execution",
                "message": f"{pose_name} trajectory failed with error_code={error_code}",
                "contact_confirmed": False,
                "completion_type": f"saved_{pose_name}_pose",
                "arm": selected_arm,
                "trajectory_action": trajectory_action,
            }
        return {
            "success": True,
            "stage": "complete",
            "message": f"{pose_name} trajectory completed",
            "contact_confirmed": False,
            "completion_type": f"saved_{pose_name}_pose",
            "arm": selected_arm,
            "trajectory_action": trajectory_action,
        }

    @staticmethod
    def _raw_joint_name(name: str) -> str:
        for prefix in ("front_piper_", "rear_piper_"):
            if name.startswith(prefix):
                return name[len(prefix):]
        return name

    def _command_joint_names(self, names, arm: str) -> list[str]:
        prefix = self.command_joint_prefix if normalize_arm(arm) == "front" else "rear_piper_"
        return [f"{prefix}{self._raw_joint_name(str(name))}" for name in names]

    def _snapshot(self, names) -> Optional[list[float]]:
        """Return fresh physical feedback in the requested joint-name order."""
        now = time.monotonic()
        with self._lock:
            joint_state = self._latest_joint_state
            received = self._joint_state_received_monotonic_s
        if joint_state is None or received is None or now - received > self.joint_state_timeout_s:
            return None

        positions = dict(zip(joint_state.name, joint_state.position))
        values = []
        for name in names:
            requested_name = str(name)
            raw_name = self._raw_joint_name(requested_name)
            # Prefer the action's exact prefixed name, then accept the raw
            # driver name from /front_piper/feedback/joint_states.  This keeps
            # the physical feedback gate compatible with either topic shape.
            feedback_name = requested_name if requested_name in positions else raw_name
            if feedback_name not in positions:
                return None
            value = float(positions[feedback_name])
            if not math.isfinite(value):
                return None
            values.append(value)
        return values

    def go_home(self, request: HomeRequest) -> Dict[str, Any]:
        return self._execute_saved_joint_pose("manipulation_pose", self.home_joint_names, self.home_positions, request)

    def go_previous(self, request: HomeRequest) -> Dict[str, Any]:
        if self.previous_joint_names is None or self.previous_positions is None:
            self.previous_joint_names, self.previous_positions = self._try_load_saved_pose(
                self.previous_pose_file,
                "previous",
            )
        return self._execute_saved_joint_pose(
            "previous",
            self.previous_joint_names,
            self.previous_positions,
            request,
        )

    def go_found_marker(self, request: HomeRequest) -> Dict[str, Any]:
        if self.found_marker_joint_names is None or self.found_marker_positions is None:
            self.found_marker_joint_names, self.found_marker_positions = self._try_load_saved_pose(
                self.found_marker_pose_file,
                "found_marker",
            )
        return self._execute_saved_joint_pose(
            "found_marker",
            self.found_marker_joint_names,
            self.found_marker_positions,
            request,
        )

    def go_nav_pose(self, request: HomeRequest) -> Dict[str, Any]:
        selected_arm = normalize_arm(request.arm)
        return self._execute_saved_joint_pose(
            "nav_pose",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            nav_pose_positions(selected_arm),
            request,
        )

    def _save_current_pose(self, path: str, pose_name: str, updated_by: str) -> Dict[str, Any]:
        now_monotonic_s = time.monotonic()
        with self._lock:
            joint_state = self._latest_joint_state
            received_monotonic_s = self._joint_state_received_monotonic_s
        if joint_state is None or received_monotonic_s is None:
            raise RuntimeError(f"no joint state has been received on {self.joint_state_topic}")
        age_s = now_monotonic_s - received_monotonic_s
        if age_s > self.joint_state_timeout_s:
            raise RuntimeError(
                f"latest joint state is stale: age_s={age_s:.3f}, timeout_s={self.joint_state_timeout_s:.3f}"
            )

        joint_positions = dict(zip(joint_state.name, joint_state.position))
        joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        selected_positions = {
            name: joint_positions.get(
                f"front_piper_{name}", joint_positions.get(name)
            )
            for name in joint_names
        }
        missing = [name for name, position in selected_positions.items() if position is None]
        if missing:
            raise RuntimeError(f"joint state missing required joints: {missing}")
        positions = [float(selected_positions[name]) for name in joint_names]
        names = list(joint_names)
        saved_positions = list(positions)
        if "gripper" in joint_positions:
            names.append("gripper")
            saved_positions.append(float(joint_positions["gripper"]))

        stamp_s = self._stamp_to_seconds(joint_state)
        stamp_sec = int(stamp_s) if stamp_s is not None else 0
        stamp_nanosec = int((stamp_s - stamp_sec) * 1_000_000_000) if stamp_s is not None else 0
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "pose_name": pose_name,
            "updated_by": updated_by,
            "captured_at_wall_time_s": time.time(),
            "captured_at_ros_time": {
                "sec": stamp_sec,
                "nanosec": stamp_nanosec,
            },
            "source_topic": self.joint_state_topic,
            "source_type": "sensor_msgs/msg/JointState",
            "arm": {
                "arm_type": "piper_x",
                "effector_type": "agx_gripper",
                "can_port": "can2",
            },
            "moveit": {
                "namespace": "/front_piper",
                "planning_group": "arm",
                "tip_link": "front_piper_flange_link",
                "tcp_offset": [0.0, 0.0, 0.1425, 0.0, 0.0, 0.0],
            },
            "tf": {
                "base_frame": "base_link",
                "flange_frame": "front_piper_flange_link",
                "camera_optical_frame": "front_camera_color_optical_frame",
            },
            "joint_state": {
                "names": names,
                "positions_rad": saved_positions,
            },
        }
        tmp_path = path.with_suffix(path.suffix + ".tmp")
        with open(tmp_path, "w", encoding="utf-8") as file:
            yaml.safe_dump(data, file, sort_keys=False)
        os.replace(tmp_path, path)

        return {
            "success": True,
            "stage": "complete",
            "message": f"saved current pose as {pose_name}: {path}",
            "contact_confirmed": False,
            "completion_type": f"saved_{pose_name}_pose_update",
            f"{pose_name}_pose_file": str(path),
            "joint_names": joint_names,
            "positions_rad": positions,
        }

    def save_home(self, request: SaveHomeRequest) -> Dict[str, Any]:
        if request.pose_name not in {"home", "manipulation_pose"}:
            raise ValueError("pose_name must be 'manipulation_pose' (legacy alias: 'home')")
        result = self._save_current_pose(
            self.home_pose_file,
            "manipulation_pose",
            "piper_touch_marker_api_save_home",
        )
        self.home_joint_names = result["joint_names"]
        self.home_positions = result["positions_rad"]
        return result

    def save_previous(self) -> Dict[str, Any]:
        result = self._save_current_pose(
            self.previous_pose_file,
            "previous",
            "piper_touch_marker_api_save_previous",
        )
        self.previous_joint_names = result["joint_names"]
        self.previous_positions = result["positions_rad"]
        return result

    def save_found_marker(self) -> Dict[str, Any]:
        result = self._save_current_pose(
            self.found_marker_pose_file,
            "found_marker",
            "piper_touch_marker_api_search_found_marker",
        )
        self.found_marker_joint_names = result["joint_names"]
        self.found_marker_positions = result["positions_rad"]
        return result


class FakeUnavailableAdapter(RosMarkerTaskAdapter):
    def health(self) -> HealthSnapshot:
        return HealthSnapshot(
            ros_ok=False,
            marker_pose_available=False,
            point_cloud_available=False,
            moveit_available=False,
            marker_task_service_available=False,
            home_action_available=False,
            joint_state_available=False,
            configured_marker_id=6,
            configured_marker_size_m=0.06,
            execution_allowed=execution_allowed_from_env(),
            hardware_ready=False,
            hardware_fake=False,
            search_marker_service_available=False,
        )

    def run_task(self, mode: str, request: MarkerTaskRequest) -> Dict[str, Any]:
        raise RuntimeError("ROS adapter not initialized")

    def ensure_arm_enabled(self) -> Dict[str, Any]:
        raise RuntimeError("ROS adapter not initialized")

    def search_marker(self, request: SearchMarkerRequest) -> Dict[str, Any]:
        raise RuntimeError("ROS adapter not initialized")

    def go_home(self, request: HomeRequest) -> Dict[str, Any]:
        raise RuntimeError("ROS adapter not initialized")

    def save_home(self, request: SaveHomeRequest) -> Dict[str, Any]:
        raise RuntimeError("ROS adapter not initialized")

    def go_previous(self, request: HomeRequest) -> Dict[str, Any]:
        raise RuntimeError("ROS adapter not initialized")

    def save_previous(self) -> Dict[str, Any]:
        raise RuntimeError("ROS adapter not initialized")

    def go_found_marker(self, request: HomeRequest) -> Dict[str, Any]:
        raise RuntimeError("ROS adapter not initialized")

    def go_nav_pose(self, request: HomeRequest) -> Dict[str, Any]:
        raise RuntimeError("ROS adapter not initialized")

    def save_found_marker(self) -> Dict[str, Any]:
        raise RuntimeError("ROS adapter not initialized")


def create_app(adapter: RosMarkerTaskAdapter, api_token: Optional[str] = None) -> FastAPI:
    app = FastAPI(title="PiPER-X Touch Marker API", version="0.1.0")
    command_lock = threading.Lock()

    @app.middleware("http")
    async def add_command_completion_fields(request: Request, call_next):
        is_task_request = (
            request.url.path.startswith("/tools/")
            and not request.url.path.endswith("/clear-active-tasks")
        )
        if is_task_request:
            adapter.begin_task()
        try:
            response = await call_next(request)
        finally:
            if is_task_request:
                adapter.end_task()
        if not request.url.path.startswith("/tools/"):
            return response
        body = b"".join([chunk async for chunk in response.body_iterator])
        try:
            payload = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return response
        if isinstance(payload, dict):
            payload.setdefault("success", response.status_code < 400)
            payload.setdefault("finished", response.status_code >= 400 or bool(payload.get("success")))
        return JSONResponse(status_code=response.status_code, content=payload)

    def validate_request(mode: str, request: MarkerTaskRequest) -> None:
        try:
            selected_arm = normalize_arm(request.arm)
        except ValueError as exc:
            raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(exc)) from exc
        if mode not in {"approach", "touch"}:
            raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=f"unsupported mode: {mode}")
        if selected_arm != "front":
            raise HTTPException(
                status_code=status.HTTP_501_NOT_IMPLEMENTED,
                detail="marker approach/touch is currently implemented only for arm='front'",
            )
        if mode == "approach" and (not math.isfinite(request.pre_clearance_m) or request.pre_clearance_m < 0.0):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="pre_clearance_m must be finite and non-negative",
            )
        if mode == "touch":
            if not math.isfinite(request.final_clearance_m) or request.final_clearance_m < 0.003:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="final_clearance_m must be finite and at least 0.003 m",
                )
            if (
                not math.isfinite(request.final_velocity_scaling)
                or request.final_velocity_scaling <= 0.0
                or request.final_velocity_scaling > 0.25
            ):
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="final_velocity_scaling must be in (0.0, 0.25]",
                )
        if request.return_home_after:
            validate_home_request(HomeRequest(execute=request.execute, arm=request.arm, duration_s=request.home_duration_s))

    def validate_home_request(request: HomeRequest) -> None:
        try:
            normalize_arm(request.arm)
        except ValueError as exc:
            raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(exc)) from exc
        if not math.isfinite(request.duration_s) or request.duration_s <= 0.0 or request.duration_s > 30.0:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="duration_s must be finite and in (0.0, 30.0]",
            )

    def require_search_readiness(health_snapshot: HealthSnapshot) -> None:
        if not health_snapshot.search_marker_service_available:
            raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail="marker search service unavailable")
        if not health_snapshot.moveit_available:
            raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail="MoveIt is unavailable")
        if not health_snapshot.joint_state_available:
            raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail="fresh joint state unavailable")

    def release_command_lock() -> bool:
        try:
            command_lock.release()
            return True
        except RuntimeError:
            return False

    def command_lock_status() -> Dict[str, Any]:
        acquired = command_lock.acquire(blocking=False)
        if acquired:
            command_lock.release()
        return {"command_lock_active": not acquired}

    def prepare_manipulation_pose(arm: str, execute: bool) -> Optional[Dict[str, Any]]:
        """Park the selected arm in its manipulation pose before any task motion."""
        if not execute:
            return None
        result = adapter.go_home(HomeRequest(execute=True, arm=arm, duration_s=6.0))
        if not result.get("success", False):
            raise HTTPException(
                status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                detail={
                    "success": False,
                    "stage": "manipulation_pose",
                    "message": result.get("message", "failed to reach manipulation pose"),
                    "manipulation_pose": result,
                },
            )
        return result

    def require_approach_readiness(health_snapshot: HealthSnapshot) -> None:
        if not health_snapshot.marker_task_service_available:
            raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail="marker task service unavailable")
        if not health_snapshot.marker_pose_available:
            raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail="ArUco marker pose unavailable")
        if not health_snapshot.point_cloud_available:
            raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail="RealSense point cloud unavailable")
        if not health_snapshot.moveit_available:
            raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail="MoveIt is unavailable")
        if not health_snapshot.joint_state_available:
            raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail="fresh joint state unavailable")

    def require_auth(request: Request, authorization: Optional[str] = Header(default=None)) -> None:
        if api_token is None:
            if request.client and request.client.host in {"127.0.0.1", "::1", "localhost", "testclient"}:
                return
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="remote access requires PIPER_TOUCH_API_TOKEN",
            )
        expected = f"Bearer {api_token}"
        if authorization != expected:
            raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="invalid bearer token")

    @app.get("/health")
    def health(_: None = Depends(require_auth)) -> Dict[str, Any]:
        return adapter.health().as_dict()

    @app.post("/tools/piper/clear-active-tasks")
    def clear_active_tasks(
        request: ClearActiveTasksRequest,
        _: None = Depends(require_auth),
    ) -> Dict[str, Any]:
        before_tasks = adapter.active_task_status()
        lock_state = command_lock_status()
        forced_lock_release = False
        lock_release_error = None
        if lock_state["command_lock_active"]:
            if not request.clear_command_lock:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail={
                        "success": False,
                        "stage": "command_lock_active",
                        "message": (
                            "a PiPER marker command lock is active; retry with "
                            "clear_command_lock=true only after confirming no arm motion is still running"
                        ),
                        "active_tasks_before": before_tasks,
                        **lock_state,
                    },
                )
            try:
                forced_lock_release = release_command_lock()
                if not forced_lock_release:
                    raise RuntimeError("command lock was already released")
            except RuntimeError as exc:
                lock_release_error = str(exc)
        clear_result = adapter.clear_active_tasks()
        after_lock_state = command_lock_status()
        return {
            "success": lock_release_error is None,
            "finished": True,
            "stage": "active_tasks_cleared" if lock_release_error is None else "command_lock_clear_error",
            "message": (
                "cleared active PiPER marker task bookkeeping"
                if lock_release_error is None
                else f"active task bookkeeping cleared, but command lock release failed: {lock_release_error}"
            ),
            "active_tasks_before": before_tasks,
            "active_tasks_after": clear_result,
            "command_lock_before": lock_state,
            "command_lock_after": after_lock_state,
            "forced_command_lock_release": forced_lock_release,
            "ros_nodes_stopped": False,
            "ros_services_stopped": False,
            "note": (
                "This endpoint only clears in-process API task bookkeeping and the API command lock. "
                "It does not kill or restart search_marker_node, wall_approach_node, MoveIt, drivers, "
                "camera nodes, or RTAB-Map."
            ),
        }

    def ensure_request_arm_enabled(arm: str) -> Dict[str, Any]:
        if hasattr(adapter, "ensure_selected_arm_enabled"):
            return adapter.ensure_selected_arm_enabled(arm)  # type: ignore[attr-defined]
        return adapter.ensure_arm_enabled()

    def run_endpoint(mode: str, request: MarkerTaskRequest) -> Dict[str, Any]:
        validate_request(mode, request)
        health_snapshot = adapter.health()
        if request.execute and not health_snapshot.execution_allowed:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="physical execution is disabled; set PIPER_TOUCH_ALLOW_EXECUTION=1",
            )
        if not command_lock.acquire(blocking=False):
            raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail="another PiPER marker task is active")
        try:
            mapping_pause_result = adapter.pause_mapping_for_manipulation() if request.execute else None
            arm_enable_result = ensure_request_arm_enabled(request.arm) if request.execute else None
            manipulation_pose_result = prepare_manipulation_pose(request.arm, request.execute)
            health_snapshot = adapter.health()
            search_result = None
            if not health_snapshot.marker_pose_available:
                require_search_readiness(health_snapshot)
                search_result = adapter.search_marker(SearchMarkerRequest(execute=request.execute, arm=request.arm))
                if not search_result.get("marker_found", False):
                    raise HTTPException(
                        status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                        detail={
                            "success": False,
                            "stage": "marker_not_found",
                            "message": "marker_not_found",
                            "search": search_result,
                        },
                    )
                health_snapshot = adapter.health()
            require_approach_readiness(health_snapshot)
            if request.execute and request.return_home_after and not health_snapshot.home_action_available:
                raise HTTPException(
                    status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                    detail="home trajectory action unavailable",
                )
            previous_saved = adapter.save_previous() if request.execute and normalize_arm(request.arm) == "front" else None
            result = adapter.run_task(mode, request)
            if search_result is not None and "search_result" not in result:
                result["search_result"] = search_result
            if arm_enable_result is not None:
                result["arm_enable"] = arm_enable_result
            if manipulation_pose_result is not None:
                result["manipulation_pose"] = manipulation_pose_result
            if previous_saved is not None:
                result["previous_pose_saved_before_motion"] = previous_saved
            if mapping_pause_result is not None:
                result["mapping_pause"] = mapping_pause_result
            if result.get("success", False) and request.execute and request.return_home_after:
                home_result = adapter.go_home(HomeRequest(execute=True, arm=request.arm, duration_s=request.home_duration_s))
                result["return_home_after"] = home_result
                if not home_result.get("success", False):
                    result = {
                        "success": False,
                        "stage": "home_after_marker",
                        "message": home_result.get("message", "home after marker failed"),
                        "contact_confirmed": False,
                        "completion_type": result.get("completion_type", "geometric_surface_approach"),
                        "marker_task": result,
                        "return_home_after": home_result,
                    }
        except HTTPException:
            raise
        except TimeoutError as exc:
            raise HTTPException(status_code=status.HTTP_504_GATEWAY_TIMEOUT, detail=str(exc)) from exc
        except Exception as exc:
            raise HTTPException(status_code=status.HTTP_502_BAD_GATEWAY, detail=str(exc)) from exc
        finally:
            release_command_lock()
        if (
            not result.get("success", False)
            and result.get("stage") not in {"search_complete", "marker_not_found"}
        ):
            raise HTTPException(status_code=status.HTTP_422_UNPROCESSABLE_ENTITY, detail=result)
        return result

    def run_search_endpoint(request: SearchMarkerRequest) -> Dict[str, Any]:
        try:
            selected_arm = normalize_arm(request.arm)
        except ValueError as exc:
            raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(exc)) from exc
        if selected_arm != "front":
            raise HTTPException(
                status_code=status.HTTP_501_NOT_IMPLEMENTED,
                detail="marker search is currently implemented only for arm='front'",
            )
        health_snapshot = adapter.health()
        if request.execute and not health_snapshot.execution_allowed:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="physical execution is disabled; set PIPER_TOUCH_ALLOW_EXECUTION=1",
            )
        require_search_readiness(health_snapshot)
        if not command_lock.acquire(blocking=False):
            raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail="another PiPER marker task is active")
        try:
            mapping_pause_result = adapter.pause_mapping_for_manipulation() if request.execute else None
            if request.execute:
                ensure_request_arm_enabled(request.arm)
            require_search_readiness(adapter.health())
            manipulation_pose_result = prepare_manipulation_pose(request.arm, request.execute)
            result = adapter.search_marker(request)
            if mapping_pause_result is not None:
                result["mapping_pause"] = mapping_pause_result
            if manipulation_pose_result is not None:
                result["manipulation_pose"] = manipulation_pose_result
            if result.get("success", False) and result.get("marker_found", False):
                found_marker_saved = adapter.save_found_marker()
                result["found_marker_pose_saved"] = found_marker_saved
                result["message"] = "found marker 6 and saved current pose as found_marker"
        except HTTPException:
            raise
        except TimeoutError as exc:
            raise HTTPException(status_code=status.HTTP_504_GATEWAY_TIMEOUT, detail=str(exc)) from exc
        except Exception as exc:
            stage = "save_found_marker" if "result" in locals() and result.get("marker_found", False) else "search_marker"
            detail = {"success": False, "stage": stage, "message": str(exc)}
            if "result" in locals():
                detail["search_result"] = result
            raise HTTPException(status_code=status.HTTP_502_BAD_GATEWAY, detail=detail) from exc
        finally:
            release_command_lock()
        if (
            not result.get("success", False)
            and result.get("stage") not in {"search_complete", "marker_not_found"}
        ):
            raise HTTPException(status_code=status.HTTP_422_UNPROCESSABLE_ENTITY, detail=result)
        return result

    def run_save_home_endpoint(request: SaveHomeRequest) -> Dict[str, Any]:
        if request.pose_name not in {"home", "manipulation_pose"}:
            raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="pose_name must be 'manipulation_pose' (legacy alias: 'home')")
        health_snapshot = adapter.health()
        if not health_snapshot.joint_state_available:
            raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail="fresh joint state unavailable")
        if not command_lock.acquire(blocking=False):
            raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail="another PiPER marker task is active")
        try:
            result = adapter.save_home(request)
        except Exception as exc:
            raise HTTPException(status_code=status.HTTP_502_BAD_GATEWAY, detail=str(exc)) from exc
        finally:
            release_command_lock()
        if not result.get("success", False):
            raise HTTPException(status_code=status.HTTP_422_UNPROCESSABLE_ENTITY, detail=result)
        return result

    def run_save_previous_endpoint() -> Dict[str, Any]:
        health_snapshot = adapter.health()
        if not health_snapshot.joint_state_available:
            raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail="fresh joint state unavailable")
        if not command_lock.acquire(blocking=False):
            raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail="another PiPER marker task is active")
        try:
            result = adapter.save_previous()
        except Exception as exc:
            raise HTTPException(status_code=status.HTTP_502_BAD_GATEWAY, detail=str(exc)) from exc
        finally:
            release_command_lock()
        if not result.get("success", False):
            raise HTTPException(status_code=status.HTTP_422_UNPROCESSABLE_ENTITY, detail=result)
        return result

    def run_home_endpoint(request: HomeRequest) -> Dict[str, Any]:
        validate_home_request(request)
        health_snapshot = adapter.health()
        if request.execute and not health_snapshot.execution_allowed:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="physical execution is disabled; set PIPER_TOUCH_ALLOW_EXECUTION=1",
            )
        if normalize_arm(request.arm) == "front" and not health_snapshot.home_action_available:
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail="home trajectory action unavailable",
            )
        if not command_lock.acquire(blocking=False):
            raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail="another PiPER marker task is active")
        try:
            arm_enable_result = ensure_request_arm_enabled(request.arm) if request.execute else None
            previous_saved = adapter.save_previous() if request.execute and normalize_arm(request.arm) == "front" else None
            result = adapter.go_home(request)
            if arm_enable_result is not None:
                result["arm_enable"] = arm_enable_result
            if previous_saved is not None:
                result["previous_pose_saved_before_motion"] = previous_saved
        except TimeoutError as exc:
            raise HTTPException(status_code=status.HTTP_504_GATEWAY_TIMEOUT, detail=str(exc)) from exc
        except Exception as exc:
            raise HTTPException(status_code=status.HTTP_502_BAD_GATEWAY, detail=str(exc)) from exc
        finally:
            release_command_lock()
        if not result.get("success", False):
            raise HTTPException(status_code=status.HTTP_422_UNPROCESSABLE_ENTITY, detail=result)
        return result

    def run_previous_endpoint(request: HomeRequest) -> Dict[str, Any]:
        validate_home_request(request)
        health_snapshot = adapter.health()
        if request.execute and not health_snapshot.execution_allowed:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="physical execution is disabled; set PIPER_TOUCH_ALLOW_EXECUTION=1",
            )
        if normalize_arm(request.arm) == "front" and not health_snapshot.home_action_available:
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail="trajectory action unavailable",
            )
        if not command_lock.acquire(blocking=False):
            raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail="another PiPER marker task is active")
        try:
            arm_enable_result = ensure_request_arm_enabled(request.arm) if request.execute else None
            result = adapter.go_previous(request)
            if arm_enable_result is not None:
                result["arm_enable"] = arm_enable_result
        except TimeoutError as exc:
            raise HTTPException(status_code=status.HTTP_504_GATEWAY_TIMEOUT, detail=str(exc)) from exc
        except Exception as exc:
            raise HTTPException(status_code=status.HTTP_502_BAD_GATEWAY, detail=str(exc)) from exc
        finally:
            release_command_lock()
        if not result.get("success", False):
            raise HTTPException(status_code=status.HTTP_422_UNPROCESSABLE_ENTITY, detail=result)
        return result

    def run_found_marker_endpoint(request: HomeRequest) -> Dict[str, Any]:
        validate_home_request(request)
        health_snapshot = adapter.health()
        if request.execute and not health_snapshot.execution_allowed:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="physical execution is disabled; set PIPER_TOUCH_ALLOW_EXECUTION=1",
            )
        if not command_lock.acquire(blocking=False):
            raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail="another PiPER marker task is active")
        try:
            arm_enable_result = ensure_request_arm_enabled(request.arm) if request.execute else None
            result = adapter.go_found_marker(request)
            if arm_enable_result is not None:
                result["arm_enable"] = arm_enable_result
        except TimeoutError as exc:
            raise HTTPException(status_code=status.HTTP_504_GATEWAY_TIMEOUT, detail=str(exc)) from exc
        except Exception as exc:
            raise HTTPException(status_code=status.HTTP_502_BAD_GATEWAY, detail=str(exc)) from exc
        finally:
            release_command_lock()
        if not result.get("success", False):
            raise HTTPException(status_code=status.HTTP_422_UNPROCESSABLE_ENTITY, detail=result)
        return result

    def run_nav_pose_endpoint(request: HomeRequest) -> Dict[str, Any]:
        validate_home_request(request)
        health_snapshot = adapter.health()
        if request.execute and not health_snapshot.execution_allowed:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="physical execution is disabled; set PIPER_TOUCH_ALLOW_EXECUTION=1",
            )
        if not command_lock.acquire(blocking=False):
            raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail="another PiPER marker task is active")
        try:
            arm_enable_result = ensure_request_arm_enabled(request.arm) if request.execute else None
            previous_saved = adapter.save_previous() if request.execute and normalize_arm(request.arm) == "front" else None
            mapping_pause_result = adapter.pause_mapping_for_manipulation() if request.execute else None
            result = adapter.go_nav_pose(request)
            mapping_settle_s = nav_pose_mapping_settle_s() if request.execute and result.get("success", False) else 0.0
            if mapping_settle_s > 0.0:
                time.sleep(mapping_settle_s)
            mapping_resume_result = (
                adapter.resume_mapping_for_navigation()
                if request.execute and result.get("success", False)
                else None
            )
            if arm_enable_result is not None:
                result["arm_enable"] = arm_enable_result
            if previous_saved is not None:
                result["previous_pose_saved_before_motion"] = previous_saved
            if mapping_pause_result is not None:
                result["mapping_pause_before_nav_pose"] = mapping_pause_result
            if request.execute:
                result["mapping_resume_after_nav_pose_settle_s"] = mapping_settle_s
            if mapping_resume_result is not None:
                result["mapping_resume_after_nav_pose"] = mapping_resume_result
        except TimeoutError as exc:
            raise HTTPException(status_code=status.HTTP_504_GATEWAY_TIMEOUT, detail=str(exc)) from exc
        except Exception as exc:
            raise HTTPException(status_code=status.HTTP_502_BAD_GATEWAY, detail=str(exc)) from exc
        finally:
            release_command_lock()
        if not result.get("success", False):
            raise HTTPException(status_code=status.HTTP_422_UNPROCESSABLE_ENTITY, detail=result)
        return result

    @app.post("/tools/piper/approach-marker")
    def approach_marker(
        request: MarkerTaskRequest,
        _: None = Depends(require_auth),
    ) -> Dict[str, Any]:
        return run_endpoint("approach", request)

    @app.post("/tools/piper/touch-marker")
    def touch_marker(
        request: MarkerTaskRequest,
        _: None = Depends(require_auth),
    ) -> Dict[str, Any]:
        return run_endpoint("touch", request)

    @app.post("/tools/piper/search-marker")
    def search_marker(
        request: SearchMarkerRequest,
        _: None = Depends(require_auth),
    ) -> Dict[str, Any]:
        return run_search_endpoint(request)

    @app.post("/tools/piper/search-step")
    def search_step(
        request: SearchMarkerRequest,
        _: None = Depends(require_auth),
    ) -> Dict[str, Any]:
        if not request.direction or request.direction.strip().lower() in {"auto", "reactive"}:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=(
                    "search-step requires one direction: left, right, up, down, "
                    "up_left, up_right, down_left, down_right, center, or current"
                ),
            )
        step_request = SearchMarkerRequest(
            execute=request.execute,
            arm=request.arm,
            direction=request.direction,
            max_steps=1,
        )
        return run_search_endpoint(step_request)

    @app.post("/tools/piper/go-home")
    def go_home(
        request: HomeRequest,
        _: None = Depends(require_auth),
    ) -> Dict[str, Any]:
        return run_home_endpoint(request)

    @app.post("/tools/piper/go-manipulation-pose")
    def go_manipulation_pose(
        request: HomeRequest,
        _: None = Depends(require_auth),
    ) -> Dict[str, Any]:
        result = run_home_endpoint(request)
        result["pose"] = "manipulation_pose"
        return result

    @app.post("/tools/piper/go-previous")
    def go_previous(
        request: HomeRequest,
        _: None = Depends(require_auth),
    ) -> Dict[str, Any]:
        return run_previous_endpoint(request)

    @app.post("/tools/piper/go-found-marker")
    def go_found_marker(
        request: HomeRequest,
        _: None = Depends(require_auth),
    ) -> Dict[str, Any]:
        return run_found_marker_endpoint(request)

    @app.post("/tools/piper/go-nav-pose")
    def go_nav_pose(
        request: HomeRequest,
        _: None = Depends(require_auth),
    ) -> Dict[str, Any]:
        return run_nav_pose_endpoint(request)

    @app.post("/tools/piper/save-home")
    def save_home(
        request: SaveHomeRequest,
        _: None = Depends(require_auth),
    ) -> Dict[str, Any]:
        return run_save_home_endpoint(request)

    @app.post("/tools/piper/save-manipulation-pose")
    def save_manipulation_pose(
        request: SaveHomeRequest,
        _: None = Depends(require_auth),
    ) -> Dict[str, Any]:
        result = run_save_home_endpoint(SaveHomeRequest(pose_name="manipulation_pose"))
        result["pose"] = "manipulation_pose"
        return result

    @app.post("/tools/piper/save-previous")
    def save_previous(_: None = Depends(require_auth)) -> Dict[str, Any]:
        return run_save_previous_endpoint()

    @app.post("/tools/piper/save-found-marker")
    def save_found_marker(_: None = Depends(require_auth)) -> Dict[str, Any]:
        health_snapshot = adapter.health()
        if not health_snapshot.joint_state_available:
            raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail="fresh joint state unavailable")
        if not command_lock.acquire(blocking=False):
            raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail="another PiPER marker task is active")
        try:
            result = adapter.save_found_marker()
        except Exception as exc:
            raise HTTPException(status_code=status.HTTP_502_BAD_GATEWAY, detail=str(exc)) from exc
        finally:
            release_command_lock()
        if not result.get("success", False):
            raise HTTPException(status_code=status.HTTP_422_UNPROCESSABLE_ENTITY, detail=result)
        return result

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description="HTTP bridge for PiPER-X ArUco marker approach.")
    parser.add_argument("--host", default=os.environ.get("PIPER_TOUCH_API_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("PIPER_TOUCH_API_PORT", "8892")))
    parser.add_argument("--marker-pose-topic", default="/aruco_single/pose")
    parser.add_argument("--point-cloud-topic", default="/front_camera/depth/color/points")
    parser.add_argument("--marker-id", type=int, default=6)
    parser.add_argument("--marker-size-m", type=float, default=0.06)
    parser.add_argument("--marker-timeout-s", type=float, default=1.0)
    parser.add_argument("--point-cloud-timeout-s", type=float, default=2.0)
    parser.add_argument("--home-pose-file", default=MarkerTaskBridge._default_home_pose_file())
    parser.add_argument("--previous-pose-file", default=None)
    parser.add_argument("--found-marker-pose-file", default=None)
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--joint-state-timeout-s", type=float, default=1.0)
    parser.add_argument(
        "--trajectory-action",
        default="/front_piper/arm_controller/follow_joint_trajectory",
    )
    parser.add_argument("--enable-service", default="/front_piper/enable_agx_arm")
    parser.add_argument("--controller-manager-service", default=None)
    parser.add_argument("--command-joint-prefix", default="front_piper_")
    args, _ros_args = parser.parse_known_args()

    rclpy.init()
    adapter = MarkerTaskBridge(
        marker_pose_topic=args.marker_pose_topic,
        point_cloud_topic=args.point_cloud_topic,
        marker_id=args.marker_id,
        marker_size_m=args.marker_size_m,
        marker_timeout_s=args.marker_timeout_s,
        point_cloud_timeout_s=args.point_cloud_timeout_s,
        home_pose_file=args.home_pose_file,
        previous_pose_file=args.previous_pose_file
        or MarkerTaskBridge._default_previous_pose_file(args.home_pose_file),
        found_marker_pose_file=args.found_marker_pose_file
        or MarkerTaskBridge._default_found_marker_pose_file(args.home_pose_file),
        joint_state_topic=args.joint_state_topic,
        joint_state_timeout_s=args.joint_state_timeout_s,
        trajectory_action=args.trajectory_action,
        enable_service=args.enable_service,
        controller_manager_service=args.controller_manager_service,
        command_joint_prefix=args.command_joint_prefix,
    )
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(adapter)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    app = create_app(adapter, api_token=token_from_env())
    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="info")
    finally:
        executor.shutdown()
        adapter.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
