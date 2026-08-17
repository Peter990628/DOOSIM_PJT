#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo '============================================================'
echo '[SETUP] hospital_total_08091221 + additive tray + 360 LiDAR + 3-post ArUco gate + map lifecycle recovery + storage I/O preflight + cache-safe setup + front ArUco + original follow camera V2.9 + final auto-dock + final staff + actual-world pose lock V2.12'
echo '[BASE] existing hospital_total files are checked byte-for-byte'
echo '============================================================'
python3 "$ROOT/tray_overlay/check_integration.py" "$ROOT"
chmod +x "$ROOT"/*.sh "$ROOT"/scripts/*.sh "$ROOT"/tray_overlay/scripts/*.py
# Use the latest baseline build procedure; it builds every package under ros2_ws/src.
"$ROOT/02_build_ros_ws.sh"
source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
for exe in runtime_probe map_probe bool_probe string_probe scan_probe traffic_probe lifecycle_bootstrap cooperative_transport_manager tray_aruco_pair_node; do
  ros2 pkg executables hospital_tray_overlay | grep -q "hospital_tray_overlay $exe" || {
    echo "[FAIL] missing overlay executable: $exe" >&2; exit 2;
  }
  echo "[PASS] overlay executable $exe"
done
mkdir -p "$ROOT/output/tray_integrated_v2_12"
bash -n "$ROOT/CHECK_ISAAC_STORAGE.sh"
bash -n "$ROOT/RECOVER_ISAAC45_STORAGE.sh"
bash -n "$ROOT/RUN_TRAY_1_ISAAC_TOTAL_360.sh"
echo '[PASS] V2.2 storage guard/recovery shell syntax'
echo '[PASS] V2.12 actual-world pose lock + straight AMR1 release + manual staff pose override + existing ArUco/convoy checks'
echo '[DONE] setup/build complete'
echo '[NEXT 1] ./RUN_TRAY_1_ISAAC_TOTAL_360.sh'
echo '[NEXT 2] after Isaac is running: ./RUN_TRAY_2_AUTO_TOTAL_360.sh'
