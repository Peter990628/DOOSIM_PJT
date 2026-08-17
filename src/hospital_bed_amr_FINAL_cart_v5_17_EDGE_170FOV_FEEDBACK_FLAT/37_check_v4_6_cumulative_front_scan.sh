#!/usr/bin/env bash
set -u
source /opt/ros/humble/setup.bash
if [ -f "ros2_ws/install/setup.bash" ]; then source ros2_ws/install/setup.bash; fi

echo "===== V4.6 CUMULATIVE CORNER + FRONT SCAN CHECK ====="
echo "[1] Front scan filter node"
ros2 node list | grep -E '^/trolley_front_scan_filter$' || true

echo
echo "[2] Full scan + front scan topics"
ros2 topic info /trolley/scan || true
ros2 topic info /trolley/scan_front || true

echo
echo "[3] Global costmap should subscribe to /trolley/scan_front"
ros2 node info /global_costmap/global_costmap 2>/dev/null | grep -E '/trolley/scan(_front)?' || true

echo
echo "[4] Local costmap should still subscribe to full /trolley/scan"
ros2 node info /local_costmap/local_costmap 2>/dev/null | grep -E '/trolley/scan(_front)?' || true

echo
echo "[5] Gate key params"
ros2 param get /trolley_heading_gate corner_detect_angle_deg || true
ros2 param get /trolley_heading_gate corner_onset_angle_deg || true
ros2 param get /trolley_heading_gate preturn_trigger_distance_m || true
ros2 param get /trolley_heading_gate min_rotate_speed || true

echo
echo "[EXPECTED LOGS]"
echo "STRAIGHT: DRIVE (no heading-recovery stop)"
echo "CORNER: dominant turn ... -> PRETURN STOP + ROTATE ..."
echo "CORNER: rotation complete -> DRIVE (corner consumed)"
