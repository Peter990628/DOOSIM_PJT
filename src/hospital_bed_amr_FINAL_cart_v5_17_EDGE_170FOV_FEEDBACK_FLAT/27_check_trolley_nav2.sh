#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
[[ -f "$ROOT/ros2_ws/install/setup.bash" ]] && source "$ROOT/ros2_ws/install/setup.bash"
set +u
export ROS_DOMAIN_ID=120
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

echo "=== TROLLEY NAV2 PRECHECK ==="
for t in /clock /trolley/scan /trolley/odom /trolley/cmd_vel; do
  echo "--- $t ---"
  ros2 topic info "$t" 2>/dev/null || true
done

echo
echo "=== TF CHECKS (2 sec each) ==="
timeout 2 ros2 run tf2_ros tf2_echo map trolley_base 2>/dev/null || true
timeout 2 ros2 run tf2_ros tf2_echo trolley_base trolley_lidar 2>/dev/null || true

echo
echo "=== TROLLEY SCAN RATE (3 sec) ==="
timeout 3 ros2 topic hz /trolley/scan || true

echo
echo "=== TROLLEY ODOM RATE (3 sec) ==="
timeout 3 ros2 topic hz /trolley/odom || true
