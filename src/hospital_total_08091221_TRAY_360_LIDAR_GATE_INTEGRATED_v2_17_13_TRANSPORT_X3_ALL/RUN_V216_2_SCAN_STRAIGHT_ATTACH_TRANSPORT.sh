#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION_FILE="$ROOT/output/v216_scan_straight/current_session_id"
[[ -s "$SESSION_FILE" ]] || { echo '[ERROR] Start ./RUN_V216_1_ISAAC_SCAN_READY_EXTERNAL_SAFE.sh first.'; exit 2; }
SESSION="$(cat "$SESSION_FILE")"
source "$ROOT/scripts/clean_ros_env.sh"
set +u; source /opt/ros/humble/setup.bash; source "$ROOT/ros2_ws/install/setup.bash"; set -u
export ROS_DOMAIN_ID=117 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
[[ -f "$HOME/.ros/fastdds_whitelist.xml" ]] && export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
LOG="$ROOT/output/v216_scan_straight"; mkdir -p "$LOG"
pkill -f 'v216_scan_straight_attach_transport.py' 2>/dev/null || true
pkill -f 'amr1_tray_aruco_gate' 2>/dev/null || true
pkill -f 'amr2_tray_aruco_gate' 2>/dev/null || true
sleep 0.5
cleanup(){ for p in "${ARUCO_PID:-}" "${RVIZ_PID:-}" "${MAP_PID:-}" "${TF_PID:-}"; do [[ -n "$p" ]] && kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM

echo '================================================================='
echo '[V2.16.5 FINAL: ARUCO -> STRAIGHT ATTACH -> FAST TRANSPORT]'
echo "Isaac session=$SESSION"
echo 'DOCK: ArUco verification only. NO left/right/yaw steering.'
echo 'MOVE: AMR1 + AMR2 straight-only 2.60m into exact bay centers.'
echo 'ATTACH: Lift + dual FixedJoint. One exact ALIGN only if primary capture misses.'
echo 'TRANSPORT: actual Isaac cart world pose, high-clearance center route.'
echo 'NO blind LAST RESORT.'
echo '================================================================='

timeout 100 ros2 run hospital_tray_overlay runtime_probe --session "$SESSION" --timeout 90 || { echo '[ERROR] Isaac runtime probe failed.'; exit 3; }
for i in $(seq 1 160); do
  T="$(ros2 topic list 2>/dev/null || true)"
  if grep -qx '/amr1/world_pose' <<<"$T" && grep -qx '/amr2/world_pose' <<<"$T" && grep -qx '/cmd_vel' <<<"$T" && grep -qx '/amr2/cmd_vel' <<<"$T" && grep -qx '/amr1/camera/front/color/image_raw' <<<"$T" && grep -qx '/amr2/camera/front/color/image_raw' <<<"$T"; then break; fi
  sleep 0.25
done

ros2 launch hospital_tray_overlay v216_map_only.launch.py >"$LOG/map.log" 2>&1 & MAP_PID=$!
ros2 run tf2_ros static_transform_publisher --x -22.69 --y 11.03 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id map --child-frame-id coop_odom >"$LOG/static_tf.log" 2>&1 & TF_PID=$!
rviz2 -d "$ROOT/ros2_ws/src/hospital_tray_overlay/rviz/v216_scan_straight_transport.rviz" >"$LOG/rviz.log" 2>&1 & RVIZ_PID=$!
ros2 launch hospital_tray_overlay tray_dual_aruco.launch.py >"$LOG/aruco.log" 2>&1 & ARUCO_PID=$!
sleep 2
set +e
python3 "$ROOT/scripts/v216_scan_straight_attach_transport.py" | tee "$LOG/scenario.log"
RC=${PIPESTATUS[0]}
set -e
if [[ "$RC" -eq 0 ]]; then
  kill "$ARUCO_PID" 2>/dev/null || true
  echo '================================================================='
  echo '[V2.16 DEMO SUCCESS] ArUco -> straight insertion -> attach -> transport complete.'
  echo 'RViz remains open. Ctrl+C after capture.'
  echo '================================================================='
  while kill -0 "$RVIZ_PID" 2>/dev/null; do sleep 1; done
else
  echo "[V2.16 DEMO FAILED] rc=$RC  log=$LOG/scenario.log"; exit "$RC"
fi
