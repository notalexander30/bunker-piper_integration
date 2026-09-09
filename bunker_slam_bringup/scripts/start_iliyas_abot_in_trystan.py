#!/usr/bin/env python3
"""Start the Bunker Nav-Man navigation and PiPER manipulation workflow in tmux.

This script is intentionally container-local. It does not call Docker, start a
second integration container, or start an automatic handoff service. Terminal
1 remains the only hardware owner for Bunker, front/rear D435i, optional H30,
and the front/rear PiPER drivers.
"""

from __future__ import annotations

import argparse
import os
import shlex
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path


SESSION = "0"
ROS_ENV = (
    "source /opt/ros/humble/setup.bash; "
    "[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash; "
    "export ROS_DOMAIN_ID=173; "
    "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; "
    "export ROS_LOCALHOST_ONLY=1; "
)
PIPER_ARUCO_SHARE_ENV = (
    'PIPER_ARUCO_PREFIX="$(ros2 pkg prefix piper_x_aruco_wall_approach 2>/dev/null)" || '
    '{ echo "Package piper_x_aruco_wall_approach not found. Run: cd /ros2_ws && '
    'source /opt/ros/humble/setup.bash && colcon build --symlink-install '
    '--base-paths src/piper_x_aruco_wall_approach && '
    'source /ros2_ws/install/setup.bash"; exit 1; }; '
    'export PIPER_ARUCO_SHARE="$PIPER_ARUCO_PREFIX/share/piper_x_aruco_wall_approach"; '
)


def run(cmd: list[str], *, check: bool = True, capture: bool = False) -> subprocess.CompletedProcess:
    return subprocess.run(
        cmd,
        check=check,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.STDOUT if capture else None,
    )


def shell_cmd(command: str) -> str:
    return f"bash -lc {command!r}"


def have_command(name: str) -> bool:
    return shutil.which(name) is not None


def tmux(*args: str, check: bool = True) -> subprocess.CompletedProcess:
    return run(["tmux", *args], check=check)


def session_exists(name: str) -> bool:
    return tmux("has-session", "-t", f"={name}", check=False).returncode == 0


def ip_link_show(iface: str) -> bool:
    return run(["ip", "link", "show", iface], check=False).returncode == 0


def configure_can(iface: str, bitrate: int) -> bool:
    if not ip_link_show(iface):
        print(f"[warn] {iface} is missing; skipping CAN setup")
        return False
    if os.geteuid() != 0:
        print(f"[warn] not root; cannot configure {iface}. Run inside the container as root.")
        return False
    run(["ip", "link", "set", iface, "down"], check=False)
    run(["ip", "link", "set", iface, "type", "can", "bitrate", str(bitrate)], check=False)
    result = run(["ip", "link", "set", iface, "up"], check=False)
    if result.returncode != 0:
        print(f"[warn] failed to bring up {iface} at {bitrate}")
        return False
    return True


def can_has_frames(iface: str, timeout_s: float) -> bool:
    if not have_command("candump") or not ip_link_show(iface):
        return False
    cmd = ["timeout", str(timeout_s), "candump", "-L", iface]
    result = run(cmd, check=False, capture=True)
    return bool((result.stdout or "").strip())


def cleanup_stale_manipulation_processes() -> None:
    """Stop stale optional manipulation processes before creating tmux panes."""
    patterns = [
        "abotclaw_handoff_service.py",
        "abotclaw_manipulation_lifecycle.sh",
        "manipulation_task_lifecycle_listener.py",
        "start_abotclaw_handoff_service.sh",
        "start_iliyas_abotclaw_service.sh",
        "start_iliyas_manipulation_listener.sh",
        "start_manipulation_task_listener.sh",
        "front_piper_description_bridge.py",
        "front_piper_move_group_only.py",
        "front_piper_moveit_tf_publisher.py",
        "front_piper_trajectory_bridge.py",
        "touch_marker_full_stack.launch.py",
        "piper_touch_marker_api.py",
        "search_marker_node",
        "wall_approach_node",
    ]
    current_pid = os.getpid()
    for pattern in patterns:
        result = run(["pgrep", "-f", pattern], check=False, capture=True)
        pids = []
        for line in (result.stdout or "").splitlines():
            try:
                pid = int(line.strip())
            except ValueError:
                continue
            if pid != current_pid:
                pids.append(pid)
        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
    time.sleep(0.5)
    for pattern in patterns:
        result = run(["pgrep", "-f", pattern], check=False, capture=True)
        for line in (result.stdout or "").splitlines():
            try:
                pid = int(line.strip())
            except ValueError:
                continue
            if pid == current_pid:
                continue
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def preflight(args: argparse.Namespace) -> None:
    if not Path("/opt/ros/humble/setup.bash").exists():
        raise SystemExit("ERROR: run this inside the ROS Humble Trystan container.")
    if not Path("/ros2_ws/src/bunker_slam_bringup").exists():
        raise SystemExit("ERROR: /ros2_ws is not mounted inside this environment.")
    if not have_command("tmux"):
        raise SystemExit("ERROR: tmux is missing in this container.")
    required_values = {
        "--bunker-can": args.bunker_can,
        "--front-piper-can": args.front_piper_can,
        "--front-camera-serial": args.front_camera_serial,
    }
    if not args.disable_rear_piper:
        required_values["--rear-piper-can"] = args.rear_piper_can
    missing_values = [name for name, value in required_values.items() if not str(value).strip()]
    if missing_values:
        raise SystemExit(
            "ERROR: provide deployment-specific values for " + ", ".join(missing_values)
        )
    if args.abot_root:
        agent_script = (
            Path(args.abot_root)
            / "robot_layer/arm_piper_x/agent_server/start_piper_x_agent_server.sh"
        )
        if not agent_script.is_file():
            raise SystemExit(f"ERROR: ABot Agent Server script not found: {agent_script}")
    if args.mapping_mode == "localization" and not Path(args.database_path).is_file():
        raise SystemExit(
            f"ERROR: RTAB-Map database not found: {args.database_path}. "
            "Create one first with --mapping-mode mapping --reset-database, "
            "or pass --database-path for an existing map."
        )

    if args.configure_can:
        configure_can(args.front_piper_can, 1000000)
        if not args.disable_rear_piper:
            configure_can(args.rear_piper_can, 1000000)
        configure_can(args.bunker_can, 500000)

    if args.require_bunker_frames and not can_has_frames(args.bunker_can, 2.0):
        raise SystemExit(
            f"ERROR: no Bunker CAN frames observed on {args.bunker_can}. "
            "Check interface mapping, wiring, power, and bitrate before launch."
        )

    if not can_has_frames(args.front_piper_can, 1.5):
        msg = (
            f"[warn] no front PiPER frames observed on {args.front_piper_can}. "
            "The stack can start, but MoveIt execution will not be physically "
            "valid until the front PiPER firmware/driver feedback is online."
        )
        if args.require_front_piper_frames:
            raise SystemExit("ERROR: " + msg[7:])
        print(msg)


def make_commands(args: argparse.Namespace) -> dict[str, str]:
    terminal1 = " ".join(
        [
            ROS_ENV,
            "ros2 launch bunker_slam_bringup terminal1_sensors.launch.py",
            f"arm_can:={args.front_piper_can}",
            f"bunker_can:={args.bunker_can}",
            f"rear_piper_can:={args.rear_piper_can}",
            f"front_piper_can:={args.front_piper_can}",
            "launch_front_camera:=true",
            "launch_rear_camera:=false",
            "reset_front_camera_usb:=false",
            f"front_camera_serial:={args.front_camera_serial}",
            f"rear_camera_serial:={args.rear_camera_serial}",
            f"configure_can:={'true' if args.configure_can else 'false'}",
            "start_piper_drivers:=true",
            "start_front_piper_driver:=true",
            f"start_rear_piper_driver:={'false' if args.disable_rear_piper else 'true'}",
            "piper_control_enabled:=true",
            "piper_effector_type:=agx_gripper",
            f"front_piper_fw_version:={args.front_piper_fw_version}",
            f"rear_piper_fw_version:={args.rear_piper_fw_version}",
            "front_piper_tcp_offset:='[0.0, 0.0, 0.1425, 0.0, 0.0, 0.0]'",
            "run_piper_initial_pose:=true",
            f"allow_piper_motion:={'true' if args.allow_piper_motion else 'false'}",
            "start_h30_imu:=false",
        ]
    )
    mapping = " ".join(
        [
            ROS_ENV,
            "ros2 launch bunker_slam_bringup terminal3_mapping.launch.py",
            f"mode:={args.mapping_mode}",
            "localization_backend:=rtabmap",
            "mapping_camera:=front",
            f"database_path:={args.database_path}",
            f"reset_database:={'true' if args.reset_database else 'false'}",
            "use_rviz:=false",
        ]
    )
    nav2 = " ".join(
        [
            ROS_ENV,
            "ros2 launch bunker_slam_bringup nav2_bringup.launch.py",
            "mode:=dry_run",
            "allow_motion:=false",
            f"use_rviz:={'true' if args.rviz else 'false'}",
            "start_depth_safety:=true",
            "start_clicked_goal:=true",
            "start_landmark_navigator:=true",
            "start_manipulation_trigger:=false",
            f"landmark_path:={args.landmark_path}",
        ]
    )
    description = " ".join(
        [
            ROS_ENV,
            "ros2 run bunker_slam_bringup front_piper_description_bridge.py",
        ]
    )
    moveit = " ".join(
        [
            ROS_ENV,
            "ros2 run bunker_slam_bringup front_piper_move_group_only.py",
        ]
    )
    trajectory_bridge = " ".join(
        [
            ROS_ENV,
            "ros2 run bunker_slam_bringup front_piper_trajectory_bridge.py",
        ]
    )
    moveit_rviz = " ".join(
        [
            ROS_ENV,
            "ros2 run bunker_slam_bringup front_piper_moveit_rviz.py",
        ]
    )
    aruco = " ".join(
        [
            ROS_ENV,
            "ros2 run aruco_ros single --ros-args",
            "-p marker_id:=6",
            "-p marker_size:=0.06",
            "-p image_is_rectified:=true",
            "-p reference_frame:=base_link",
            "-p camera_frame:=front_camera_color_optical_frame",
            "-p marker_frame:=aruco_marker_frame",
            "-r /image:=/front_camera/color/image_raw",
            "-r /camera_info:=/front_camera/color/camera_info",
        ]
    )
    search_marker = " ".join(
        [
            ROS_ENV,
            PIPER_ARUCO_SHARE_ENV,
            'ros2 run piper_x_aruco_wall_approach search_marker_node --ros-args',
            '--params-file "$PIPER_ARUCO_SHARE/config/piper_x_search_poses.yaml"',
            "-p aruco_pose_topic:=/aruco_single/pose",
            "-p marker_id:=6",
            "-p joint_state_topic:=/joint_states",
            "-p move_group_namespace:=front_piper",
            "-p controller_manager_service:=/front_piper/controller_manager/list_hardware_components",
            "-p require_physical_hardware:=false",
            "-r joint_states:=/joint_states",
            "-r robot_description:=/robot_description",
            "-r robot_description_semantic:=/front_piper/robot_description_semantic",
        ]
    )
    wall_approach = " ".join(
        [
            ROS_ENV,
            PIPER_ARUCO_SHARE_ENV,
            'ros2 run piper_x_aruco_wall_approach wall_approach_node --ros-args',
            '--params-file "$PIPER_ARUCO_SHARE/config/wall_approach.yaml"',
            "-p execute:=false",
            "-p clearance:=0.05",
            "-p final_clearance:=0.005",
            "-p retract_after:=true",
            "-p prefer_elbow_motion:=true",
            "-p goal_orientation_tolerance:=0.35",
            "-p move_group_namespace:=front_piper",
            "-p point_cloud_topic:=/front_camera/depth/color/points",
            "-p joint_state_topic:=/joint_states",
            "-r joint_states:=/joint_states",
            "-r robot_description:=/robot_description",
            "-r robot_description_semantic:=/front_piper/robot_description_semantic",
        ]
    )
    marker_api = " ".join(
        [
            ROS_ENV,
            PIPER_ARUCO_SHARE_ENV,
            f"export PIPER_TOUCH_ALLOW_EXECUTION={'1' if args.allow_piper_motion else '0'};",
            "ros2 run piper_x_aruco_wall_approach piper_touch_marker_api.py",
            "--host 127.0.0.1",
            "--port 8892",
            "--marker-id 6",
            "--marker-size-m 0.06",
            "--point-cloud-topic /front_camera/depth/color/points",
            "--marker-timeout-s 1.0",
            "--point-cloud-timeout-s 2.0",
            '--home-pose-file "$PIPER_ARUCO_SHARE/config/piper_x_home_pose.yaml"',
            '--previous-pose-file "$PIPER_ARUCO_SHARE/config/piper_x_previous_pose.yaml"',
            '--found-marker-pose-file "$PIPER_ARUCO_SHARE/config/piper_x_found_marker_pose.yaml"',
            "--joint-state-topic /joint_states",
            "--joint-state-timeout-s 2.5",
            "--trajectory-action /front_piper/arm_controller/follow_joint_trajectory",
            "--enable-service /front_piper/enable_agx_arm",
            "--controller-manager-service /front_piper/controller_manager/list_hardware_components",
            "--command-joint-prefix front_piper_",
        ]
    )
    if args.abot_root:
        abot_root = shlex.quote(str(Path(args.abot_root).resolve()))
        agent_execution = "1" if args.allow_piper_motion else "0"
        openclaw_gateway = " ".join(
            [
                ROS_ENV,
                f"cd {abot_root};",
                f"export PIPER_X_AGENT_ALLOW_EXECUTION={agent_execution};",
                f"export PIPER_TOUCH_ALLOW_EXECUTION={agent_execution};",
                "export PIPER_X_MARKER_API_URL=http://127.0.0.1:8892;",
                "export PIPER_X_MARKER_ID=6;",
                "export PIPER_X_MARKER_SIZE_M=0.06;",
                "export PIPER_X_JOINT_STATE_TOPIC=/front_piper/feedback/joint_states;",
                "export PIPER_X_GRIPPER_CONTROL_TOPIC=/front_piper/control/joint_states;",
                "export PIPER_X_TRAJECTORY_ACTION=/front_piper/arm_controller/follow_joint_trajectory;",
                "exec ./robot_layer/arm_piper_x/agent_server/start_piper_x_agent_server.sh",
            ]
        )
    else:
        openclaw_gateway = " ".join(
            [
                ROS_ENV,
                "export OPENCLAW_GATEWAY_HOST=127.0.0.1;",
                "export OPENCLAW_GATEWAY_PORT=8893;",
                "export PIPER_TOUCH_API_URL=http://127.0.0.1:8892;",
                "ros2 run piper_x_aruco_wall_approach openclaw_gateway.py",
                "--host 127.0.0.1",
                "--port 8893",
                "--api-base http://127.0.0.1:8892",
            ]
        )
    watchdogs = " ".join(
        [
            ROS_ENV,
            "ros2 run bunker_slam_bringup watchdog_sources_monitor.sh",
        ]
    )
    frontier_mrtsp = " ".join(
        [
            ROS_ENV,
            "ros2 launch bunker_slam_bringup frontier_exploration_ros2_bunker.launch.py",
            "autostart:=false",
            "control_service_enabled:=true",
        ]
    )
    return {
        "t1_hardware": terminal1,
        "t2_mapping": mapping,
        "t3_nav2": nav2,
        "t4_srdf_description": description,
        "t5_moveit": moveit,
        "t6_trajectory_bridge": trajectory_bridge,
        "t7_moveit_rviz": moveit_rviz,
        "t8_aruco": aruco,
        "t9_marker_search": search_marker,
        "t10_wall_approach": wall_approach,
        "t11_api_8892": marker_api,
        "t12_agent_8893": openclaw_gateway,
        "t13_watchdogs": watchdogs,
        "t14_frontier_mrtsp": frontier_mrtsp,
    }


def create_tmux_session(args: argparse.Namespace) -> None:
    if session_exists(args.session):
        if not args.replace:
            raise SystemExit(
                f"ERROR: tmux session {args.session!r} already exists. "
                f"Attach with: tmux attach -t {args.session}; or rerun with --replace."
            )
        tmux("kill-session", "-t", f"={args.session}")

    if args.clean_manipulation:
        cleanup_stale_manipulation_processes()

    commands = make_commands(args)
    first_window, first_command = next(iter(commands.items()))
    tmux("new-session", "-d", "-s", args.session, "-n", first_window, shell_cmd(first_command))
    for window, command in list(commands.items())[1:]:
        tmux("new-window", "-t", f"={args.session}:", "-n", window, shell_cmd(command))
    tmux("select-window", "-t", f"={args.session}:t1_hardware")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", default=SESSION)
    parser.add_argument("--replace", action="store_true", help="replace an existing tmux session")
    parser.add_argument(
        "--no-clean-manipulation",
        dest="clean_manipulation",
        action="store_false",
        help="do not stop stale manipulation/MoveIt overlay processes before launch",
    )
    parser.set_defaults(clean_manipulation=True)
    parser.add_argument("--skip-preflight", action="store_true")
    parser.add_argument("--no-configure-can", dest="configure_can", action="store_false")
    parser.set_defaults(configure_can=True)
    parser.add_argument(
        "--disable-rear-piper",
        action="store_true",
        help="disable the rear PiPER driver and skip rear CAN setup",
    )
    parser.add_argument("--require-bunker-frames", action="store_true", default=True)
    parser.add_argument("--no-require-bunker-frames", dest="require_bunker_frames", action="store_false")
    parser.add_argument("--require-front-piper-frames", action="store_true")
    parser.add_argument("--no-rviz", dest="rviz", action="store_false")
    parser.set_defaults(rviz=True)
    parser.add_argument(
        "--allow-piper-motion",
        action="store_true",
        help="allow physical arm commands; omitted by default for safe bringup",
    )
    parser.add_argument(
        "--reset-database",
        action="store_true",
        help="delete/recreate RTAB-Map state at startup; use only when making a new map",
    )
    parser.add_argument(
        "--mapping-mode",
        choices=("mapping", "localization"),
        default="localization",
        help="create/update a map or localize against an existing RTAB-Map database",
    )
    parser.add_argument("--bunker-can", default="")
    parser.add_argument("--front-piper-can", default="")
    parser.add_argument("--rear-piper-can", default="")
    parser.add_argument("--front-camera-serial", default="")
    parser.add_argument("--rear-camera-serial", default="")
    parser.add_argument("--front-piper-fw-version", default="v189")
    parser.add_argument("--rear-piper-fw-version", default="v189")
    parser.add_argument("--h30-serial-port", default="")
    parser.add_argument("--database-path", default="/ros2_ws/maps/bunker_dual_rgbd_v2.db")
    parser.add_argument(
        "--landmark-path",
        default=(
            "/ros2_ws/src/bunker-piper_integration/"
            "bunker_slam_bringup/maps/manual_nav_landmarks.json"
        ),
    )
    parser.add_argument(
        "--abot-root",
        default="",
        help=(
            "optional ABot-Claw-piperX checkout; starts the Iliyas Agent Server "
            "on 8893 instead of the bundled compatibility gateway"
        ),
    )
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    if not args.skip_preflight:
        preflight(args)
    create_tmux_session(args)
    print(f"Started tmux session: {args.session}")
    print(f"Attach with: tmux attach -t {args.session}")
    print("Front PiPER gate starts open; check/reopen command:")
    print('  ros2 service call /front_piper/control_enable std_srvs/srv/SetBool "{data: true}"')
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
