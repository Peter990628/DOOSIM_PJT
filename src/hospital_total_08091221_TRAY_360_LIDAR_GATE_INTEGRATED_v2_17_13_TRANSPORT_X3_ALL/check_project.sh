#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

python3 -m py_compile \
  "$ROOT/scripts/isaac_amr_ros.py" \
  "$ROOT/patient_transport_manager.py" \
  "$ROOT/ros2_ws/src/hospital_nav2/hospital_nav2/centerline_navigator.py" \
  "$ROOT/ros2_ws/src/hospital_nav2/hospital_nav2/path_conflict_manager.py" \
  "$ROOT/ros2_ws/src/hospital_nav2/launch/hospital_amr1_navigation.launch.py" \
  "$ROOT/ros2_ws/src/hospital_nav2/launch/hospital_dual_navigation.launch.py" \
  "$ROOT/ros2_ws/src/hospital_ocr_bridge/hospital_ocr_bridge/nameplate_vision.py" \
  "$ROOT/ros2_ws/src/hospital_ocr_bridge/hospital_ocr_bridge/ocr_node.py" \
  "$ROOT/ros2_ws/src/hospital_ocr_bridge/launch/amr1_ocr.launch.py" \
  "$ROOT/ros2_ws/src/hospital_ocr_bridge/launch/dual_ocr.launch.py"

find "$ROOT" -type d -name __pycache__ -prune -exec rm -rf {} +
find "$ROOT" -type f -name '*.pyc' -delete

python3 - "$ROOT/config/isaac_config.json" "$ROOT/scripts/isaac_amr_ros.py" <<'PY'
from pathlib import Path
import json
import sys
cfg = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
source = Path(sys.argv[2]).read_text(encoding="utf-8")
auto = cfg["auto_approach"]
assert float(auto["x_tolerance_px"]) > 0
assert "y_tolerance_normalized" not in auto
assert "ALIGNING_X" in source
assert "OCR bbox fallback" in source
assert "ROTATING_1F_ENTRY" in source
assert "target_yaw_deg=90.0" in source
assert "error_y_normalized" not in source
assert "last_report" not in source
ocr_source = Path(
    Path(sys.argv[2]).parents[1] / "ros2_ws/src/hospital_ocr_bridge/hospital_ocr_bridge/ocr_node.py"
).read_text(encoding="utf-8")
assert "name_only_threshold = 0.15" in ocr_source
assert "strong_wrong_name_threshold = 0.80" in ocr_source
assert '"name": "",' in ocr_source
launch_root = Path(sys.argv[2]).parents[1] / "ros2_ws/src/hospital_nav2/launch"
dual_launch = (launch_root / "hospital_dual_navigation.launch.py").read_text(encoding="utf-8")
assert "PushRosNamespace" in dual_launch
assert "'traffic_pause_topic': '/traffic_pause'" in dual_launch
assert "'traffic_pause_topic': '/amr2/traffic_pause'" in dual_launch
amr1_launch = (launch_root / "hospital_amr1_navigation.launch.py").read_text(encoding="utf-8")
assert '"traffic_pause_topic": "/traffic_pause"' in amr1_launch
traffic_source = (
    Path(sys.argv[2]).parents[1]
    / "ros2_ws/src/hospital_nav2/hospital_nav2/path_conflict_manager.py"
).read_text(encoding="utf-8")
assert 'self.declare_parameter("amr2_map_topic", "/amr2/map")' in traffic_source
center_source = (
    Path(sys.argv[2]).parents[1]
    / "ros2_ws/src/hospital_nav2/hospital_nav2/centerline_navigator.py"
).read_text(encoding="utf-8")
assert "interrupted_rotation_pending" in center_source
assert cfg["ros2"]["domain_id"] == 117
print(f"[OK] X-only centre alignment: tolerance={auto['x_tolerance_px']} px")
print(f"[OK] Stable tracking messages: {auto['stable_tracking_messages']}")
print("[OK] Y gating removed")
print("[OK] Recurring velocity/state heartbeat removed")
print("[OK] Domain 117, 1F elevator +90deg alignment and traffic pause wiring")
PY

if find "$ROOT" -type d -name third_party | grep -q .; then
  echo "[FAIL] third_party directory exists" >&2
  exit 2
fi
if find "$ROOT" -type d -name __pycache__ | grep -q .; then
  echo "[FAIL] __pycache__ directory exists" >&2
  exit 3
fi

[[ -f "$ROOT/project4/project4_hospital_bed_amr_v1_15_ocr.usd" ]]
[[ -f "$ROOT/config/isaac_config.json" ]]

echo "[OK] Python syntax"
echo "[OK] No third_party / cache files"
echo "[OK] V4 runtime stage exists"
echo "[PASS] Project source check complete"
