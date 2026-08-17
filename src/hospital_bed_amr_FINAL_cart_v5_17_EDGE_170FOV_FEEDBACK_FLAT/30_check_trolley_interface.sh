#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
[[ -f "$ROOT/ros2_ws/install/setup.bash" ]] && source "$ROOT/ros2_ws/install/setup.bash"
set +u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-120}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

show_topic() {
  local topic="$1"
  echo
  echo "===== $topic ====="
  ros2 topic info "$topic" --verbose 2>/dev/null || echo "[FAIL] topic unavailable: $topic"
}

echo "=== TROLLEY CANONICAL INTERFACE CHECK ==="
echo "Expected core chain:"
echo "RViz/manager -> /trolley/center_goal -> centerline_navigator -> /follow_path -> /trolley/cmd_vel"
echo "Sensor chain: /trolley/scan -> Local/Global Costmap"

for t in \
  /clock \
  /map \
  /trolley/scan \
  /trolley/odom \
  /trolley/center_goal \
  /trolley/centerline_path \
  /trolley/center_goal/status \
  /trolley/cmd_vel \
  /local_costmap/costmap \
  /global_costmap/costmap; do
  show_topic "$t"
done

echo
echo "===== ACTIONS ====="
ros2 action list | grep -E '^/(follow_path|navigate_to_pose|navigate_through_poses)$' || true

echo
echo "===== NODES ====="
ros2 node list | grep -E 'centerline_navigator|controller_server|planner_server|bt_navigator|map_server' || true

echo
echo "===== TF ====="
timeout 2 ros2 run tf2_ros tf2_echo map trolley_base 2>/dev/null || true
timeout 2 ros2 run tf2_ros tf2_echo trolley_base trolley_lidar 2>/dev/null || true

echo
echo "===== QUICK RULES ====="
echo "- /clock: publisher count must be 1"
echo "- /trolley/scan: BEST_EFFORT + VOLATILE; publisher 1"
echo "- /trolley/odom: RELIABLE + VOLATILE; publisher 1"
echo "- /trolley/center_goal: RELIABLE + VOLATILE; centerline subscriber present"
echo "- /trolley/centerline_path: RELIABLE + TRANSIENT_LOCAL; centerline publisher present"
echo "- /trolley/cmd_vel: RELIABLE + VOLATILE; cooperative subscriber present"
echo "- Global cmd_vel remap is intentionally retained in this test-pending build"
