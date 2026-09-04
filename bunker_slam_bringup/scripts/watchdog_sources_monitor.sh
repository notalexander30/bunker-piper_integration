#!/usr/bin/env bash
set -u

source /opt/ros/humble/setup.bash
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-173}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

topic_field() {
  local topic="$1"
  local field="${2:-data}"
  timeout 2 ros2 topic echo --once "$topic" --field "$field" 2>/dev/null || true
}

topic_owner() {
  local topic="$1"
  timeout 3 ros2 topic info -v "$topic" 2>/dev/null \
    | awk '
      /Publisher count:/ {print; next}
      /Subscription count:/ {print; next}
      /Node name:/ {print "  " $0; next}
    ' || true
}

while true; do
  clear
  date '+%F %T'
  echo "Nav-Man watchdog source monitor"
  echo
  echo "Meaning:"
  echo "  watchdog_timeout from /cmd_vel_mux/reason is published by /nav2_cmd_vel_safety_mux."
  echo "  It means no fresh /cmd_vel_autonomy command arrived within command_timeout_sec."
  echo "  That is normal when Nav2 is idle; debug it only when you expected motion."
  echo

  echo "Nodes:"
  ros2 node list 2>/dev/null \
    | grep -E 'nav2_cmd_vel_safety_mux|depth_route_monitor_node|sensor_fusion_node|realsense_stream_watchdog|controller_server|velocity_smoother|bunker$' \
    || true
  echo

  echo "Current watchdog/safety values:"
  printf '  %-24s %s\n' '/cmd_vel_mux/reason:' "$(topic_field /cmd_vel_mux/reason)"
  printf '  %-24s %s\n' '/safety_stop:' "$(topic_field /safety_stop)"
  printf '  %-24s %s\n' '/safety_stop_reason:' "$(topic_field /safety_stop_reason)"
  printf '  %-24s %s\n' '/depth_route_status:' "$(topic_field /depth_route_status)"
  printf '  %-24s %s\n' '/route_status:' "$(topic_field /route_status)"
  echo

  echo "Command path ownership:"
  echo "--- /nav2/cmd_vel_raw ---"
  topic_owner /nav2/cmd_vel_raw
  echo "--- /cmd_vel_autonomy ---"
  topic_owner /cmd_vel_autonomy
  echo "--- /cmd_vel_mux/reason ---"
  topic_owner /cmd_vel_mux/reason
  echo "--- /cmd_vel ---"
  topic_owner /cmd_vel
  echo

  echo "Camera watchdog ownership:"
  echo "--- /front_camera/color/image_raw ---"
  topic_owner /front_camera/color/image_raw
  echo "--- /front_camera/depth/color/points ---"
  topic_owner /front_camera/depth/color/points
  echo
  echo "Refreshes every 2 seconds. Ctrl-C stops only this monitor."
  sleep 2
done
