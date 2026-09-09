#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${ROBOT_SYSTEM_CONFIG:-${SCRIPT_DIR}/robot_system.env}"
MODE=debug

usage() {
  echo "Usage: $0 [--debug|--drive] [--config /absolute/path/robot_system.env]"
  echo "  --debug  Nav2 commands go to /cmd_vel_debug (default; no base motion)."
  echo "  --drive  Nav2 commands go to /cmd_vel_autonomy (hardware commissioning)."
}

while (( $# > 0 )); do
  case "$1" in
    --debug) MODE=debug; shift ;;
    --drive) MODE=drive; shift ;;
    --config) CONFIG_FILE="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ ! -f "${CONFIG_FILE}" ]]; then
  echo "ERROR: configuration not found: ${CONFIG_FILE}" >&2
  echo "Copy robot_system.env.example to robot_system.env and fill the CAN/camera values." >&2
  exit 2
fi
# shellcheck disable=SC1090
source "${CONFIG_FILE}"

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROBOT_WS="${ROBOT_WS:-/home/dase-orin/ros2_ws}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${ROBOT_WS}/install/setup.bash"

if [[ "${START_HARDWARE_DRIVERS:-0}" == 1 ]]; then
  if [[ -z "${BUNKER_CAN:-}" ]]; then
    echo "ERROR: Set BUNKER_CAN before hardware startup." >&2
    exit 2
  fi
  if [[ "${START_PIPER_DRIVERS:-1}" == 1 && ( -z "${FRONT_PIPER_CAN:-}" || -z "${REAR_PIPER_CAN:-}" ) ]]; then
    echo "ERROR: Set FRONT_PIPER_CAN and REAR_PIPER_CAN before PiPER startup." >&2
    exit 2
  fi
fi
if [[ "${START_CAMERAS:-1}" == 1 && "${START_REAR_CAMERA:-0}" == 1 ]]; then
  if [[ "${FRONT_CAMERA_SERIAL:-''}" == "''" || "${REAR_CAMERA_SERIAL:-''}" == "''" ]]; then
    echo "ERROR: Both D435i serial numbers are required when two cameras start." >&2
    exit 2
  fi
fi
if [[ "${START_RTABMAP:-0}" == 1 && -z "${RTABMAP_LAUNCH_FILE:-}" ]]; then
  echo "ERROR: START_RTABMAP=1 requires RTABMAP_LAUNCH_FILE." >&2
  exit 2
fi
if [[ "${START_YOLO:-0}" == 1 && -z "${YOLO_LAUNCH_FILE:-}" ]]; then
  echo "ERROR: START_YOLO=1 requires YOLO_LAUNCH_FILE." >&2
  exit 2
fi

OUTPUT_TOPIC=/cmd_vel_debug
if [[ "${MODE}" == drive ]]; then
  OUTPUT_TOPIC=/cmd_vel_autonomy
  echo "DRIVE MODE: Nav2 output will reach the isolated Bunker autonomy input."
  echo "Keep the emergency stop ready and keep both PiPER arms folded."
  read -r -p "Type DRIVE to continue: " CONFIRMATION
  [[ "${CONFIRMATION}" == DRIVE ]] || { echo "Cancelled."; exit 1; }
else
  echo "DEBUG MODE: Nav2 cannot command the Bunker base."
fi

bool_word() { [[ "${1:-0}" == 1 ]] && echo true || echo false; }

exec ros2 launch bunker_dual_piper_nav2 system_bringup.launch.py \
  start_hardware_drivers:="$(bool_word "${START_HARDWARE_DRIVERS:-0}")" \
  start_bunker_driver:="$(bool_word "${START_BUNKER_DRIVER:-1}")" \
  start_piper_drivers:="$(bool_word "${START_PIPER_DRIVERS:-1}")" \
  bunker_can:="${BUNKER_CAN:-}" \
  front_piper_can:="${FRONT_PIPER_CAN:-}" \
  rear_piper_can:="${REAR_PIPER_CAN:-}" \
  start_cameras:="$(bool_word "${START_CAMERAS:-1}")" \
  launch_front_camera:="$(bool_word "${START_FRONT_CAMERA:-1}")" \
  launch_rear_camera:="$(bool_word "${START_REAR_CAMERA:-0}")" \
  front_camera_serial:="${FRONT_CAMERA_SERIAL:-}" \
  rear_camera_serial:="${REAR_CAMERA_SERIAL:-}" \
  launch_rtabmap:="$(bool_word "${START_RTABMAP:-0}")" \
  rtabmap_launch_file:="${RTABMAP_LAUNCH_FILE:-}" \
  launch_yolo:="$(bool_word "${START_YOLO:-0}")" \
  yolo_launch_file:="${YOLO_LAUNCH_FILE:-}" \
  safe_cmd_vel_output:="${OUTPUT_TOPIC}" \
  front_joint_states_topic:="${FRONT_JOINT_STATES_TOPIC:-/front_piper/feedback/joint_states}" \
  rear_joint_states_topic:="${REAR_JOINT_STATES_TOPIC:-/rear_piper/feedback/joint_states}" \
  front_camera_xyz:="${FRONT_CAMERA_XYZ:-0.000 0.000 0.000}" \
  front_camera_rpy:="${FRONT_CAMERA_RPY:--0.08017405150425999 -0.0031709794581423934 0.023976595541682157}" \
  rear_camera_xyz:="${REAR_CAMERA_XYZ:--0.0016016943280790555 -0.07988793520475261 0.0420998734823866}" \
  rear_camera_rpy:="${REAR_CAMERA_RPY:--0.0041241555313950035 -0.003886773443390706 0.0554203109827187}" \
  front_piper_mount_rpy:="${FRONT_PIPER_MOUNT_RPY:-0 0 1.57079632679}" \
  rear_piper_mount_rpy:="${REAR_PIPER_MOUNT_RPY:-0 0 1.57079632679}" \
  front_piper_legacy:="$(bool_word "${FRONT_PIPER_LEGACY:-0}")" \
  rear_piper_legacy:="$(bool_word "${REAR_PIPER_LEGACY:-0}")"
