#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
if [ -f "ros2_ws/install/setup.bash" ]; then source ros2_ws/install/setup.bash; fi

echo "===== V4.5 DOMINANT SEGMENT + PRETURN CHECK ====="
echo "[1] Final / raw command chain"
ros2 topic info /trolley/cmd_vel -v || true
ros2 topic info /trolley/cmd_vel_raw -v || true

echo
echo "[2] V4.5 parameters"
for p in dominant_segment_span_m corner_detect_angle_deg preturn_trigger_distance_m corner_exit_angle_deg consumed_corner_radius_m min_rotate_speed max_rotate_speed rotation_stall_time_sec rotation_stall_yaw_deg rotation_stall_boost_speed replan_min_interval_sec; do
  ros2 param get /trolley_heading_gate "$p" || true
done

echo
echo "[EXPECTED LOGS]"
echo "  CORNER: dominant turn ... -> PRETURN STOP + ROTATE ..."
echo "  CORNER: rotation complete -> DRIVE (corner consumed)"
echo "  STRAIGHT: DRIVE on locked dominant segment"
echo
echo "[LIVE CMD] ros2 topic echo /trolley/cmd_vel"
