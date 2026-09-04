#!/usr/bin/env bash
# Run bounded live checks for the RGB-D SLAM data path and TF ownership.

set -o pipefail

usage() {
  cat <<'EOF'
Usage: verify_slam.sh

Checks required topic types and publishers, takes bounded samples of wheel/
filtered odometry, IMU, joint state and map data, then runs the TF ownership
and RGB-D synchronization helpers.

Optional environment overrides:
  TF_DURATION=8          TF ownership observation window, seconds
  RGBD_SAMPLES=10        synchronized RGB-D-CameraInfo sets required
  RGBD_TIMEOUT=20        overall RGB-D wait, seconds
  RGBD_SLOP_MS=50        maximum timestamp difference, milliseconds
  TF_LOOKUP_TIMEOUT=2.0  TF wait per synchronized set, seconds
  SAMPLE_TIMEOUT=5       per-topic sample timeout, seconds
  IMU_CALIBRATION_TIMEOUT=15  corrected-IMU startup sample timeout, seconds
  GRAPH_TIMEOUT=3        per-topic graph query timeout, seconds
  GRAPH_SPIN_TIME=1.0    DDS discovery spin time for each graph query, seconds
  ROS_UNDERLAY_SETUP, BUNKER_SLAM_ROS_WS, BUNKER_SLAM_SETUP
EOF
}

if [[ $# -ne 0 ]]; then
  if [[ $# -eq 1 && ("$1" == "-h" || "$1" == "--help") ]]; then
    usage
    exit 0
  fi
  usage >&2
  exit 2
fi

script_directory="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
underlay_setup="${ROS_UNDERLAY_SETUP:-/opt/ros/humble/setup.bash}"
slam_workspace="${BUNKER_SLAM_ROS_WS:-${ROS_WS:-/ros2_ws}}"
overlay_setup="${BUNKER_SLAM_SETUP:-${slam_workspace}/install/setup.bash}"

if [[ ! -r "$underlay_setup" || ! -r "$overlay_setup" ]]; then
  echo "FAIL: ROS setup files are not readable:" >&2
  echo "  $underlay_setup" >&2
  echo "  $overlay_setup" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$underlay_setup"
# shellcheck disable=SC1090
source "$overlay_setup"
set -u
export ROS2CLI_NO_DAEMON=1

tf_duration="${TF_DURATION:-8}"
rgbd_samples="${RGBD_SAMPLES:-10}"
rgbd_timeout="${RGBD_TIMEOUT:-20}"
rgbd_slop_ms="${RGBD_SLOP_MS:-50}"
tf_lookup_timeout="${TF_LOOKUP_TIMEOUT:-2.0}"
sample_timeout="${SAMPLE_TIMEOUT:-5}"
imu_calibration_timeout="${IMU_CALIBRATION_TIMEOUT:-15}"
graph_timeout="${GRAPH_TIMEOUT:-3}"
graph_spin_time="${GRAPH_SPIN_TIME:-1.0}"
overall_status=0

declare -A live_publishers=()

check_nodes() {
  local node_output node
  if ! node_output="$(timeout "${graph_timeout}s" ros2 node list \
      --spin-time "$graph_spin_time" 2>&1)"; then
    echo "FAIL: could not query the ROS node graph: $node_output"
    overall_status=1
    return
  fi
  for node in \
      /bunker \
      /front_piper/agx_arm_ctrl_single_node \
      /rear_piper/agx_arm_ctrl_single_node \
      /robot_state_publisher \
      /front_camera \
      /rear_camera \
      /ekf_filter_node \
      /front_rgbd_sync \
      /rear_rgbd_sync \
      /rtabmap; do
    if grep -Fxq "$node" <<<"$node_output"; then
      echo "PASS: required node $node is running"
    else
      echo "FAIL: required node $node is missing"
      overall_status=1
    fi
  done
}

check_topic() {
  local topic="$1"
  local expected_type="$2"
  local info_output actual_type publisher_count

  if ! info_output="$(timeout "${graph_timeout}s" ros2 topic info "$topic" \
      --spin-time "$graph_spin_time" 2>&1)"; then
    echo "FAIL: $topic is unavailable: $info_output"
    overall_status=1
    return
  fi
  actual_type="$(timeout "${graph_timeout}s" ros2 topic type "$topic" \
      --spin-time "$graph_spin_time" 2>/dev/null || true)"
  if [[ "$actual_type" != "$expected_type" ]]; then
    echo "FAIL: $topic type is '${actual_type:-unknown}', expected $expected_type"
    overall_status=1
  fi
  publisher_count="$(sed -n 's/^Publisher count: //p' <<<"$info_output" | head -n 1)"
  if [[ ! "$publisher_count" =~ ^[0-9]+$ || "$publisher_count" -lt 1 ]]; then
    echo "FAIL: $topic has no live publisher"
    overall_status=1
    return
  fi
  live_publishers["$topic"]=1
  echo "PASS: $topic [$actual_type], publishers=$publisher_count"
}

sample_once() {
  local topic="$1"
  local message_type="$2"
  local durability="${3:-volatile}"
  local timeout_seconds="${4:-$sample_timeout}"
  local -a qos_arguments=()

  if [[ -z "${live_publishers[$topic]:-}" ]]; then
    echo "SKIP: no publisher available for sample from $topic"
    return
  fi
  if [[ "$durability" == "transient_local" ]]; then
    qos_arguments=(--qos-durability transient_local)
  fi
  if timeout "${timeout_seconds}s" ros2 topic echo \
      "$topic" "$message_type" --once "${qos_arguments[@]}" \
      >/dev/null 2>&1; then
    echo "PASS: received one message from $topic"
  else
    echo "FAIL: no message from $topic within ${timeout_seconds}s"
    overall_status=1
  fi
}

required_topics=(
  "/wheel/odometry|nav_msgs/msg/Odometry"
  "/odometry/filtered|nav_msgs/msg/Odometry"
  "/front_camera/color/image_raw|sensor_msgs/msg/Image"
  "/front_camera/aligned_depth_to_color/image_raw|sensor_msgs/msg/Image"
  "/front_camera/depth/color/points|sensor_msgs/msg/PointCloud2"
  "/front_camera/color/camera_info|sensor_msgs/msg/CameraInfo"
  "/front_camera/imu|sensor_msgs/msg/Imu"
  "/rear_camera/color/image_raw|sensor_msgs/msg/Image"
  "/rear_camera/aligned_depth_to_color/image_raw|sensor_msgs/msg/Image"
  "/rear_camera/depth/color/points|sensor_msgs/msg/PointCloud2"
  "/rear_camera/color/camera_info|sensor_msgs/msg/CameraInfo"
  "/rear_camera/imu|sensor_msgs/msg/Imu"
  "/joint_states|sensor_msgs/msg/JointState"
  "/front_rgbd_image|rtabmap_msgs/msg/RGBDImage"
  "/rear_rgbd_image|rtabmap_msgs/msg/RGBDImage"
  "/map|nav_msgs/msg/OccupancyGrid"
  "/tf|tf2_msgs/msg/TFMessage"
  "/tf_static|tf2_msgs/msg/TFMessage"
)

echo "== Required nodes =="
check_nodes

echo "== Required topic publishers =="
for specification in "${required_topics[@]}"; do
  check_topic "${specification%%|*}" "${specification#*|}"
done

echo "== Bounded message samples =="
sample_once /wheel/odometry nav_msgs/msg/Odometry
sample_once /odometry/filtered nav_msgs/msg/Odometry
sample_once /front_camera/imu sensor_msgs/msg/Imu
sample_once /rear_camera/imu sensor_msgs/msg/Imu
sample_once /joint_states sensor_msgs/msg/JointState
sample_once /map nav_msgs/msg/OccupancyGrid transient_local

echo "== TF ownership =="
if ! python3 "${script_directory}/check_tf_ownership.py" \
    --duration "$tf_duration"; then
  overall_status=1
fi

echo "== RGB-D synchronization and timestamped TF =="
if ! python3 "${script_directory}/check_rgbd_sync.py" \
    --rgb-topic /front_camera/color/image_raw \
    --depth-topic /front_camera/aligned_depth_to_color/image_raw \
    --camera-info-topic /front_camera/color/camera_info \
    --samples "$rgbd_samples" \
    --timeout "$rgbd_timeout" \
    --slop-ms "$rgbd_slop_ms" \
    --tf-timeout "$tf_lookup_timeout"; then
  overall_status=1
fi
if ! python3 "${script_directory}/check_rgbd_sync.py" \
    --rgb-topic /rear_camera/color/image_raw \
    --depth-topic /rear_camera/aligned_depth_to_color/image_raw \
    --camera-info-topic /rear_camera/color/camera_info \
    --samples "$rgbd_samples" \
    --timeout "$rgbd_timeout" \
    --slop-ms "$rgbd_slop_ms" \
    --tf-timeout "$tf_lookup_timeout"; then
  overall_status=1
fi

if [[ "$overall_status" -eq 0 ]]; then
  echo "SLAM verification PASSED"
else
  echo "SLAM verification FAILED"
fi
exit "$overall_status"
