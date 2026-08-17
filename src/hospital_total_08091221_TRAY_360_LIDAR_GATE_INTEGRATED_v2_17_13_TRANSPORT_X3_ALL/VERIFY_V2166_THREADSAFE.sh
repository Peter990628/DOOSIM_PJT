#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 -m py_compile \
 "$ROOT/tray_overlay/scripts/final_scene_staff.py" \
 "$ROOT/tray_overlay/scripts/cooperative_warehouse_cart.py" \
 "$ROOT/tray_overlay/scripts/isaac_amr_ros_tray_scan_straight_v216.py" \
 "$ROOT/scripts/v216_scan_straight_attach_transport.py"
python3 "$ROOT/tray_overlay/check_integration.py" "$ROOT"
grep -q 'PHYSICS EDIT PAUSE' "$ROOT/tray_overlay/scripts/cooperative_warehouse_cart.py"
grep -q 'install_final_scene_staff_sync' "$ROOT/tray_overlay/scripts/final_scene_staff.py"
echo '[PASS] V2.16.6 VERIFY COMPLETE'
