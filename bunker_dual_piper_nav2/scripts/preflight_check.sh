#!/usr/bin/env bash
set -u

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROBOT_WS="${ROBOT_WS:-/home/dase-orin/ros2_ws}"
FAILURES=0
WARNINGS=0

pass() { printf '[PASS] %s\n' "$*"; }
warn() { printf '[WARN] %s\n' "$*"; WARNINGS=$((WARNINGS + 1)); }
fail() { printf '[FAIL] %s\n' "$*"; FAILURES=$((FAILURES + 1)); }

if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  fail "ROS ${ROS_DISTRO} setup file is missing."
fi
if [[ -f "${ROBOT_WS}/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${ROBOT_WS}/install/setup.bash"
else
  fail "Workspace is not built: ${ROBOT_WS}/install/setup.bash"
fi

for package in bunker_dual_piper_nav2 robot_state_publisher nav2_bringup realsense2_camera realsense2_description; do
  if ros2 pkg prefix "${package}" >/dev/null 2>&1; then
    pass "ROS package available: ${package}"
  else
    fail "ROS package missing: ${package}"
  fi
done

for optional_package in bunker_base agx_arm_ctrl; do
  if ros2 pkg prefix "${optional_package}" >/dev/null 2>&1; then
    pass "Hardware package available: ${optional_package}"
  else
    warn "Hardware package not indexed: ${optional_package}"
  fi
done

CAN_LIST="$(ip -brief link 2>/dev/null | awk '$1 ~ /^can[0-9]+$/ {print $1 ":" $2}')"
if [[ -n "${CAN_LIST}" ]]; then
  pass "CAN interfaces found: ${CAN_LIST//$'\n'/, }"
else
  warn "No CAN interface is currently visible."
fi

if command -v rs-enumerate-devices >/dev/null 2>&1; then
  CAMERA_COUNT="$(rs-enumerate-devices -s 2>/dev/null | grep -c 'Intel RealSense' || true)"
  if (( CAMERA_COUNT > 0 )); then
    pass "RealSense devices detected: ${CAMERA_COUNT}"
  else
    warn "No RealSense device detected."
  fi
else
  warn "rs-enumerate-devices is unavailable; camera USB presence was not checked."
fi

if command -v ros2 >/dev/null 2>&1; then
  TOPICS="$(timeout 3 ros2 topic list 2>/dev/null || true)"
  for topic in /odom /map /tf /tf_static; do
    if grep -Fxq "${topic}" <<<"${TOPICS}"; then
      pass "Live topic: ${topic}"
    else
      warn "Topic is not live yet: ${topic}"
    fi
  done
  if grep -Fxq /cmd_vel_autonomy <<<"${TOPICS}"; then
    pass "Isolated Bunker autonomy topic is present."
  else
    warn "/cmd_vel_autonomy is not present yet."
  fi
fi

echo
echo "Preflight result: ${FAILURES} failure(s), ${WARNINGS} warning(s)."
if (( FAILURES > 0 )); then
  exit 1
fi
