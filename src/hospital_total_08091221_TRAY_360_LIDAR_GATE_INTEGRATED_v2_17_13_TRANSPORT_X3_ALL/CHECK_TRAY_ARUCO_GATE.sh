#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
[[ -f "$ROOT/ros2_ws/install/setup.bash" ]] && source "$ROOT/ros2_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID=117 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0

echo '============================================================'
echo '[TRAY ARUCO GATE CHECK]'
echo 'Physical layout: LEFT 40/41 | CENTER 44 | RIGHT 42/43'
echo 'AMR1 logical gate: outer 40/41 + center 44'
echo 'AMR2 logical gate: center 44 + outer 42/43'
echo '============================================================'

echo '[TOPICS]'
ros2 topic list | grep -E '^/amr[12]/tray_aruco/(result|debug_image)$' || true

echo
echo '[AMR1 one result]'
timeout 8 ros2 topic echo /amr1/tray_aruco/result --once || true

echo
echo '[AMR2 one result]'
timeout 8 ros2 topic echo /amr2/tray_aruco/result --once || true

echo
echo '[TIP] state=PAIR and center_id=44 should appear when the tray gate is visible.'
echo '[TIP] outer_source_ids should show 40/41 for AMR1 and 42/43 for AMR2.'
