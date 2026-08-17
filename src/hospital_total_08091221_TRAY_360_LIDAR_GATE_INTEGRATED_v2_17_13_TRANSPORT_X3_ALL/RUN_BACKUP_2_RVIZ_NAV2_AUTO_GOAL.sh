#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[[ -f "$ROOT/ros2_ws/install/setup.bash" ]] || { echo '[ERROR] Run ./00_SETUP_TRAY_360_INTEGRATED.sh first.'; exit 1; }
SESSION_FILE="$ROOT/output/backup_precoupled_v2_14/current_session_id"
[[ -s "$SESSION_FILE" ]] || { echo '[ERROR] Start ./RUN_BACKUP_1_ISAAC_PRECOUPLED.sh first.'; exit 2; }
SESSION="$(cat "$SESSION_FILE")"
source "$ROOT/scripts/clean_ros_env.sh"
set +u; source /opt/ros/humble/setup.bash; source "$ROOT/ros2_ws/install/setup.bash"; set -u
export ROS_DOMAIN_ID=117 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
[[ -f "$HOME/.ros/fastdds_whitelist.xml" ]] && export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
LOG="$ROOT/output/backup_precoupled_v2_14"; mkdir -p "$LOG"
cleanup(){ for p in "${GOAL_PID:-}" "${TEL_PID:-}" "${RVIZ_PID:-}" "${NAV_PID:-}"; do [[ -n "$p" ]] && kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM

echo "[BACKUP PRECHECK] Isaac session=$SESSION"
timeout 90 ros2 run hospital_tray_overlay runtime_probe --session "$SESSION" --timeout 80
# Wait for pre-coupled runtime to expose cooperative odom/scans.
for i in $(seq 1 100); do
  if ros2 topic list 2>/dev/null | grep -qx '/coop/odom' && ros2 topic list 2>/dev/null | grep -qx '/coop/scan_left'; then break; fi
  sleep 0.3
done
ros2 topic list | grep -qx '/coop/odom' || { echo '[ERROR] /coop/odom missing. Check Terminal1 for BACKUP PRECOUPLED PASS.'; exit 3; }

echo '[BACKUP] launch STATIC-MAP cooperative Nav2 (live LiDAR obstacle stop disabled only here)'
ros2 launch hospital_tray_overlay backup_precoupled_nav.launch.py >"$LOG/nav2.log" 2>&1 & NAV_PID=$!

# RViz opens automatically.
rviz2 -d "$ROOT/ros2_ws/src/hospital_tray_overlay/rviz/backup_precoupled_demo.rviz" >"$LOG/rviz.log" 2>&1 & RVIZ_PID=$!

# Wheel calculations are shown both in RViz and in this terminal via a separate process log.
python3 "$ROOT/scripts/backup_wheel_telemetry.py" --ros-args \
  -p wheel_radius_m:=0.075 -p wheel_lever_m:=0.5825 -p lateral_offset_m:=0.425 \
  >"$LOG/wheel_telemetry.log" 2>&1 & TEL_PID=$!

# Wait for map + pose lock + ready status.
timeout 80 ros2 run hospital_tray_overlay map_probe --topic /map --timeout 70
timeout 80 ros2 run hospital_tray_overlay bool_probe --topic /coopnav/initial_pose_locked --timeout 70

# V2.14.1 BACKUP ONLY: guarantee a straight departure before any Nav2 goal.
# This bypasses the centerline navigator's old rotate-before-first-segment behavior.
echo '[BACKUP] initial straight departure: 3.0m, angular.z=0 (NO RIGHT TURN)'
python3 "$ROOT/scripts/backup_straight_departure.py" --distance 3.0 --speed 0.20 --timeout 30 || true

echo '================================================================='
echo '[BACKUP DEMO START V2.14.1 - INITIAL STRAIGHT LOCKED]'
echo 'PRE-COUPLED: AMR1 + AMR2 + tray'
echo 'NAV2 TARGET: x=7.7732 y=6.329 yaw=0deg (user screenshot)'
echo 'RViz: auto-opened; wheel angular velocity text is overlaid on the cooperative base.'
echo 'LiDAR: visible, but live obstacle stop is DISABLED in this backup-only Nav2 config.'
echo '================================================================='
python3 "$ROOT/scripts/backup_auto_goal.py" --x 7.7732 --y 6.329 --yaw-deg 0 --timeout 360 & GOAL_PID=$!

# Mirror telemetry dashboard into current terminal while goal runs.
while kill -0 "$GOAL_PID" 2>/dev/null; do
  clear || true
  echo '================ V2.14 BACKUP PRE-COUPLED NAV2 ================'
  echo 'Target: x=7.7732 y=6.329 yaw=0deg'
  echo 'AMR1/AMR2 wheel angular speeds are calculated separately from cart-center V,W.'
  echo 'RViz also shows the same values above the cooperative vehicle.'
  echo '---------------------------------------------------------------'
  tail -n 12 "$LOG/wheel_telemetry.log" 2>/dev/null || true
  echo '---------------------------------------------------------------'
  timeout 1 ros2 topic echo --once /coop/center_goal/status std_msgs/msg/String 2>/dev/null | tail -n 3 || true
  sleep 0.5
done
wait "$GOAL_PID" || true
GOAL_PID=''
echo '[BACKUP] auto-goal sequence finished. Nav2 + RViz remain open for manual /coop/center_goal if needed.'
trap - EXIT INT TERM
wait "$NAV_PID"
