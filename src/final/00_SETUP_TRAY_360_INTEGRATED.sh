#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo '============================================================'
echo '[SETUP] GUI 병원 이송 + Tray 360 LiDAR + ArUco 협동 운송'
echo '[BASE] final의 GUI/병원 미션/Domain 115 설정을 유지합니다.'
echo '============================================================'

python3 "$ROOT/tray_overlay/check_integration.py" "$ROOT"
chmod +x \
  "$ROOT/00_SETUP_TRAY_360_INTEGRATED.sh" \
  "$ROOT/RUN_TRAY_1_ISAAC_TOTAL_360.sh" \
  "$ROOT/RUN_TRAY_2_AUTO_TOTAL_360.sh" \
  "$ROOT/RUN_V217_1_ISAAC_SCAN_READY.sh" \
  "$ROOT/RUN_V217_2_TRUE_ARUCO_DOCK_TRANSPORT.sh" \
  "$ROOT/STOP_TRAY_INTEGRATED_ROS.sh" \
  "$ROOT/CHECK_360_LIDAR_RUNTIME.sh" \
  "$ROOT/CHECK_TRAY_ARUCO_GATE.sh" \
  "$ROOT/scripts/v217_true_aruco_dock_transport.py" \
  "$ROOT/scripts/v217_true_aruco_pair_scanner.py" \
  "$ROOT/scripts/v217_raw_lidar_map_points.py" \
  "$ROOT/scripts/v21712_actual_speed_monitor.py"

"$ROOT/02_build_ros_ws.sh"

source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u

for exe in runtime_probe map_probe bool_probe string_probe scan_probe traffic_probe lifecycle_bootstrap cooperative_transport_manager tray_aruco_pair_node; do
  ros2 pkg executables hospital_tray_overlay | grep -q "hospital_tray_overlay $exe" || {
    echo "[FAIL] missing overlay executable: $exe" >&2
    exit 2
  }
  echo "[PASS] overlay executable $exe"
done

echo '[DONE] tray integration setup/build complete'
echo '[AUTO MODE] ./RUN_TRAY_1_ISAAC_TOTAL_360.sh -> ./RUN_TRAY_2_AUTO_TOTAL_360.sh'
echo '[V2.17.13 DEMO] ./RUN_V217_1_ISAAC_SCAN_READY.sh -> ./RUN_V217_2_TRUE_ARUCO_DOCK_TRANSPORT.sh'
