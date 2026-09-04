#!/usr/bin/env bash
set -Eeo pipefail

timeout_seconds="${1:-120}"
if [[ ! "$timeout_seconds" =~ ^[0-9]+$ ]] || (( timeout_seconds < 1 )); then
  echo "error: timeout must be a positive integer" >&2
  exit 2
fi

deadline=$((SECONDS + timeout_seconds))
missing=()

topic_has_message() {
  local topic="$1"
  timeout 3 ros2 topic echo --once "$topic" --field header \
    >/dev/null 2>&1
}

tf_is_ready() {
  local output
  output="$(timeout 3 ros2 run tf2_ros tf2_echo map base_link 2>&1 || true)"
  grep -q 'Translation:' <<<"$output"
}

echo "Waiting up to ${timeout_seconds}s for the Nav2 input contract..."
while (( SECONDS < deadline )); do
  missing=()
  topic_has_message /map || missing+=(/map)
  topic_has_message /odometry/filtered || missing+=(/odometry/filtered)
  topic_has_message /front_camera/color/image_raw \
    || missing+=(/front_camera/color/image_raw)
  topic_has_message /front_camera/color/camera_info \
    || missing+=(/front_camera/color/camera_info)
  topic_has_message /front_camera/aligned_depth_to_color/image_raw \
    || missing+=(/front_camera/aligned_depth_to_color/image_raw)
  topic_has_message /front_camera/depth/color/points \
    || missing+=(/front_camera/depth/color/points)
  topic_has_message /rear_camera/color/image_raw \
    || missing+=(/rear_camera/color/image_raw)
  topic_has_message /rear_camera/color/camera_info \
    || missing+=(/rear_camera/color/camera_info)
  topic_has_message /rear_camera/aligned_depth_to_color/image_raw \
    || missing+=(/rear_camera/aligned_depth_to_color/image_raw)
  topic_has_message /rear_camera/depth/color/points \
    || missing+=(/rear_camera/depth/color/points)
  tf_is_ready || missing+=(map-to-base_link-TF)

  if (( ${#missing[@]} == 0 )); then
    echo "Nav2 inputs are ready."
    exit 0
  fi
  echo "Still waiting for: ${missing[*]}"
  sleep 1
done

echo "error: Nav2 inputs were not ready after ${timeout_seconds}s: ${missing[*]}" >&2
exit 1
