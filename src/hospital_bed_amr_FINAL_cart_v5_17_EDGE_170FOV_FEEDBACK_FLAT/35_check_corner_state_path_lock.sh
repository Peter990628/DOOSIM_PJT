#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
if [ -f "ros2_ws/install/setup.bash" ]; then source ros2_ws/install/setup.bash; fi

echo "===== V4.4 CORNER STATE + PATH LOCK CHECK ====="
echo "[1] Final command chain"
ros2 topic info /trolley/cmd_vel -v || true

echo
echo "[2] Raw Nav2 command chain"
ros2 topic info /trolley/cmd_vel_raw -v || true

echo
echo "[3] State-machine / path-lock parameters"
for p in corner_detect_angle_deg corner_stop_distance_m corner_exit_angle_deg replan_min_interval_sec replan_lateral_threshold_m replan_heading_threshold_deg; do
  ros2 param get /trolley_heading_gate "$p" || true
done

echo
echo "[4] Live behavior: watch launch terminal for CORNER / PATH_LOCK / STRAIGHT messages"
echo "Final cmd: ros2 topic echo /trolley/cmd_vel"
