#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== V2.16.5 FINAL VERIFY ==="
python3 -m py_compile \
  "$ROOT/scripts/v216_scan_straight_attach_transport.py" \
  "$ROOT/tray_overlay/scripts/final_scene_staff.py" \
  "$ROOT/tray_overlay/scripts/isaac_amr_ros_tray_scan_straight_v216.py"

python3 "$ROOT/tray_overlay/check_integration.py" "$ROOT"

python3 - "$ROOT" <<'PY'
from pathlib import Path
import json, sys
root=Path(sys.argv[1])
cfg=json.loads((root/"tray_overlay/config/isaac_config_scan_straight_v216.json").read_text())
d=cfg["scan_straight_demo"]
assert d["marker_view_standoff_from_front_m"] == 1.50
assert d["straight_insert_distance_m"] == 2.60
assert cfg["cooperative_warehouse_cart"]["nav2_max_linear_speed_mps"] == 0.72
s=(root/"scripts/v216_scan_straight_attach_transport.py").read_text()
assert "TRANSPORT_V_FAST=0.66" in s
assert "DETACH_GRACE_SEC=2.0" in s
assert "(1.0,0.05,0.55)" in s.replace(" ","")
f=(root/"tray_overlay/scripts/final_scene_staff.py").read_text()
assert "V2165_FIXED_GOAL_SIDE" in f
print("[PASS] V2.16.5 final invariants")
PY

echo "[PASS] V2.16.5 FINAL VERIFY COMPLETE"
