#!/usr/bin/env bash
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[[ -f "$ROOT/ros2_ws/install/setup.bash" ]] || { echo '[ERROR] Run ./00_SETUP_TRAY_360_INTEGRATED.sh first.'; exit 1; }
SESSION_FILE="$ROOT/output/backup_precoupled_v2_14/current_session_id"
[[ -s "$SESSION_FILE" ]] || { echo '[ERROR] Start ./RUN_BACKUP_1_ISAAC_PRECOUPLED.sh first.'; exit 2; }
SESSION="$(cat "$SESSION_FILE")"
source "$ROOT/scripts/clean_ros_env.sh"
set +u; source /opt/ros/humble/setup.bash; source "$ROOT/ros2_ws/install/setup.bash"; set -u
export ROS_DOMAIN_ID=117 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
[[ -f "$HOME/.ros/fastdds_whitelist.xml" ]] && export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
LOG="$ROOT/output/backup_guaranteed_v2_15"; mkdir -p "$LOG"

# Only BACKUP demo processes are cleaned. Normal project logic remains untouched.
pkill -f 'backup_precoupled_nav_guaranteed.launch.py' 2>/dev/null || true
pkill -f 'backup_precoupled_nav.launch.py' 2>/dev/null || true
pkill -f 'backup_guaranteed_route_v215.py' 2>/dev/null || true
pkill -f 'backup_direct_fallback_v215.py' 2>/dev/null || true
pkill -f 'backup_timed_last_resort_v215.py' 2>/dev/null || true
pkill -f 'backup_wheel_telemetry_v215.py' 2>/dev/null || true
sleep 1

cleanup(){
  for p in "${TEL_PID:-}" "${RVIZ_PID:-}" "${NAV_PID:-}" "${TF_PID:-}"; do [[ -n "$p" ]] && kill "$p" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

echo '================================================================='
echo '[V2.15 GUARANTEED BACKUP]'
echo "Isaac session=$SESSION"
echo 'Primary : Nav2 controller_server FollowPath on a known-clear rounded L path'
echo 'Fallback: direct /coop/cmd_vel odometry follower'
echo 'Last    : time-only cmd_vel route if both above cannot even start'
echo 'NO pose-lock wait / NO ArUco / NO traffic manager / NO live obstacle stop'
echo 'Original RUN_TRAY_* logic is untouched.'
echo '================================================================='

timeout 90 ros2 run hospital_tray_overlay runtime_probe --session "$SESSION" --timeout 80 || { echo '[ERROR] Isaac runtime probe failed.'; exit 3; }
# Cooperative bridge is the only hard prerequisite for movement.
for i in $(seq 1 120); do
  TOPICS="$(ros2 topic list 2>/dev/null || true)"
  if grep -qx '/coop/odom' <<<"$TOPICS" && grep -qx '/coop/cmd_vel' <<<"$TOPICS"; then break; fi
  sleep 0.25
done
ros2 topic list | grep -qx '/coop/odom' || { echo '[ERROR] /coop/odom missing. Terminal1 must remain running.'; exit 4; }

echo '[V2.15] Starting independent static map->coop_odom (no initial_pose_locked dependency)'
ros2 run tf2_ros static_transform_publisher --x -22.69 --y 11.03 --z 0.0 --yaw 0.0 --pitch 0.0 --roll 0.0 --frame-id map --child-frame-id coop_odom >"$LOG/static_tf.log" 2>&1 & TF_PID=$!
sleep 0.5
echo '[V2.15] Launching cooperative Nav2'
ros2 launch hospital_tray_overlay backup_precoupled_nav_guaranteed.launch.py >"$LOG/nav2.log" 2>&1 & NAV_PID=$!
rviz2 -d "$ROOT/ros2_ws/src/hospital_tray_overlay/rviz/backup_guaranteed_demo.rviz" >"$LOG/rviz.log" 2>&1 & RVIZ_PID=$!
python3 "$ROOT/scripts/backup_wheel_telemetry_v215.py" --ros-args \
  -p wheel_radius_m:=0.075 -p wheel_lever_m:=0.5825 -p lateral_offset_m:=0.425 \
  >"$LOG/wheel_telemetry.log" 2>&1 & TEL_PID=$!

# Do NOT wait for /coopnav/initial_pose_locked. That wait caused V2.14.2 to stall.
# Give Nav2 a bounded startup window, then let primary decide whether FollowPath is usable.
sleep 4

echo '[V2.15] PRIMARY Nav2 FollowPath start'
set +e
python3 "$ROOT/scripts/backup_guaranteed_route_v215.py" | tee "$LOG/route_primary.log"
RC=${PIPESTATUS[0]}
set -e

if [[ "$RC" -ne 0 ]]; then
  echo "[V2.15] PRIMARY failed/stalled rc=$RC -> DIRECT ODOMETRY FALLBACK"
  set +e
  python3 "$ROOT/scripts/backup_direct_fallback_v215.py" | tee "$LOG/route_fallback.log"
  FRC=${PIPESTATUS[0]}
  set -e
  if [[ "$FRC" -ne 0 ]]; then
    echo "[V2.15] ODOM FALLBACK failed rc=$FRC -> LAST RESORT time-only route"
    python3 "$ROOT/scripts/backup_timed_last_resort_v215.py" | tee "$LOG/route_last_resort.log" || true
  fi
fi

echo '================================================================='
echo '[V2.15 BACKUP ROUTE COMPLETE]'
echo 'RViz stays open. Wheel telemetry stays visible.'
echo 'The cart remains dual-FixedJoint attached.'
echo 'Ctrl+C when presentation capture is finished.'
echo '================================================================='
while true; do
  if [[ -f "$LOG/wheel_telemetry.log" ]]; then tail -n 1 "$LOG/wheel_telemetry.log" 2>/dev/null | sed 's/^/[WHEEL] /' || true; fi
  sleep 1
done
