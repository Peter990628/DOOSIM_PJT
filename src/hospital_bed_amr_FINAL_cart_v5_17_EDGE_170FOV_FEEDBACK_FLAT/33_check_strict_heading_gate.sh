#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$ROOT/ros2_ws/install/setup.bash" ]]; then
  source "$ROOT/ros2_ws/install/setup.bash"
fi
export ROS_DOMAIN_ID=120
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

echo "===== V4.2 STRICT HEADING GATE CHECK ====="
echo "[1] Final cmd publisher MUST include trolley_heading_gate and NOT controller_server"
ros2 topic info /trolley/cmd_vel -v || true

echo
echo "[2] Raw Nav2 command publishers"
ros2 topic info /trolley/cmd_vel_raw -v || true

echo
echo "[3] Gate parameters"
ros2 param get /trolley_heading_gate enter_angle_deg || true
ros2 param get /trolley_heading_gate exit_angle_deg || true
ros2 param get /trolley_heading_gate lookahead_distance || true

echo
echo "[EXPECTED WHILE TURNING] FINAL /trolley/cmd_vel: linear.x = 0.0, angular.z != 0"
echo "To inspect live: ros2 topic echo /trolley/cmd_vel"
