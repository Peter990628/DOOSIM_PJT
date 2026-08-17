#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/ros2_ws/install/setup.bash" 2>/dev/null || true

echo "===== V4.3 SEGMENT HEADING GATE CHECK ====="
echo "[1] Final /trolley/cmd_vel publisher"
ros2 topic info /trolley/cmd_vel -v || true

echo
echo "[2] Raw Nav2 command publishers"
ros2 topic info /trolley/cmd_vel_raw -v || true

echo
echo "[3] Gate parameters"
for p in tangent_span_distance enter_angle_deg exit_angle_deg max_drive_angular_speed; do
  ros2 param get /trolley_heading_gate "$p" || true
done

echo
echo "EXPECTED:"
echo "- turning to next segment: final linear.x = 0.0"
echo "- driving on a segment: angular.z limited to small correction"
echo "Live check: ros2 topic echo /trolley/cmd_vel"
