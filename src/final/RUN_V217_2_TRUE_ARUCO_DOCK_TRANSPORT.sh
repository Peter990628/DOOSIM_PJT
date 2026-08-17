#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION_FILE="$ROOT/output/v216_scan_straight/current_session_id"
[[ -s "$SESSION_FILE" ]] || { echo '[ERROR] Start ./RUN_V216_1_ISAAC_SCAN_READY_EXTERNAL_SAFE.sh first.'; exit 2; }
SESSION="$(cat "$SESSION_FILE")"
source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
[[ -f "$ROOT/ros2_ws/install/setup.bash" ]] || { echo '[ERROR] ros2_ws/install/setup.bash missing. Run ./02_build_ros_ws.sh once.'; exit 4; }
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID=115 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
[[ -f "$HOME/.ros/fastdds_whitelist.xml" ]] && export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
LOG="$ROOT/output/v217_true_aruco"; mkdir -p "$LOG"
pkill -f 'v217_true_aruco_dock_transport.py' 2>/dev/null || true
pkill -f 'v217_true_aruco_pair_scanner.py' 2>/dev/null || true
sleep 0.4
cleanup(){ for p in "${A1_PID:-}" "${A2_PID:-}" "${LIDAR_VIZ_PID:-}" "${SIGNAL_PID:-}" "${RVIZ_PID:-}" "${MAP_PID:-}" "${TF_PID:-}"; do [[ -n "$p" ]] && kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM

echo '================================================================='
echo '[V2.17 TRUE ARUCO VISUAL DOCK]'
echo 'PAIR REQUIRED: AMR1 outer(40/41)+44 AND AMR2 44+outer(42/43).'
echo 'NO 9-second success fallback. NO SINGLE-marker insertion.'
echo 'NO exact ALIGN snap. If visual docking fails, robots STOP.'
echo 'APPROACH: forward + angular.z visual servo; no lateral cmd.'
echo 'FINAL: last real ArUco lock -> short under-tray odom handoff -> ATTACH.'
echo '================================================================='

timeout 100 ros2 run hospital_tray_overlay runtime_probe --session "$SESSION" --timeout 90 || { echo '[ERROR] Isaac runtime probe failed.'; exit 3; }
ros2 launch hospital_tray_overlay v216_map_only.launch.py >"$LOG/map.log" 2>&1 & MAP_PID=$!

echo '[V2.17.7 MAP] waiting for /map_server ACTIVE...'
if ! timeout 35 ros2 run hospital_tray_overlay lifecycle_bootstrap --node /map_server --timeout 30 --watch-pid "$MAP_PID"; then
  echo '[ERROR] map_server did not reach ACTIVE.'
  tail -n 50 "$LOG/map.log" || true
  exit 21
fi

echo '[V2.17.7 MAP] waiting for valid /map OccupancyGrid...'
if ! timeout 35 ros2 run hospital_tray_overlay map_probe --topic /map --timeout 30 --min-cells 100 --watch-pid "$MAP_PID"; then
  echo '[ERROR] valid /map was not received.'
  tail -n 50 "$LOG/map.log" || true
  exit 22
fi

ros2 run tf2_ros static_transform_publisher --x -22.69 --y 11.03 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id map --child-frame-id coop_odom >"$LOG/static_tf.log" 2>&1 & TF_PID=$!
sleep 0.7
echo '[V2.17.8 LIDAR] starting always-on raw LiDAR map visualizer...'
PYTHONUNBUFFERED=1 python3 -u "$ROOT/scripts/v217_raw_lidar_map_points.py" >"$LOG/lidar_viz.log" 2>&1 & LIDAR_VIZ_PID=$!
sleep 1.5
echo '---- LiDAR visualizer startup ----'
tail -n 20 "$LOG/lidar_viz.log" || true

echo '[V2.17.12 SPEED PROOF] starting direct-command + actual-speed monitor...'
PYTHONUNBUFFERED=1 python3 -u "$ROOT/scripts/v21712_actual_speed_monitor.py" >"$LOG/motion_signal.log" 2>&1 & SIGNAL_PID=$!
echo '[V2.17.9 MAP+LIDAR READY] starting RViz.'
rviz2 -d "$ROOT/ros2_ws/src/hospital_tray_overlay/rviz/v216_scan_straight_transport.rviz" >"$LOG/rviz.log" 2>&1 & RVIZ_PID=$!
python3 "$ROOT/scripts/v217_true_aruco_pair_scanner.py" --ros-args -p amr_id:=amr1 -p image_topic:=/amr1/camera/front/color/image_raw -p result_topic:=/amr1/tray_aruco/result -p debug_image_topic:=/amr1/tray_aruco/debug_image >"$LOG/aruco_amr1.log" 2>&1 & A1_PID=$!
python3 "$ROOT/scripts/v217_true_aruco_pair_scanner.py" --ros-args -p amr_id:=amr2 -p image_topic:=/amr2/camera/front/color/image_raw -p result_topic:=/amr2/tray_aruco/result -p debug_image_topic:=/amr2/tray_aruco/debug_image >"$LOG/aruco_amr2.log" 2>&1 & A2_PID=$!
sleep 0.7
set +e
python3 "$ROOT/scripts/v217_true_aruco_dock_transport.py" | tee "$LOG/scenario.log"
RC=${PIPESTATUS[0]}
set -e
if [[ "$RC" -eq 0 ]]; then
  echo '================================================================='
  echo '[V2.17 SUCCESS] real pair ArUco visual servo -> natural insert -> attach -> transport.'
  echo '================================================================='
  while kill -0 "$RVIZ_PID" 2>/dev/null; do sleep 1; done
else
  echo "[V2.17 FAILED SAFE] rc=$RC. No fake attach was allowed. log=$LOG/scenario.log"
  echo '[V2.17.7 DEBUG] RViz/map kept open. Close RViz to exit.'
  while kill -0 "$RVIZ_PID" 2>/dev/null; do sleep 1; done
  exit "$RC"
fi
