#!/usr/bin/env python3
from __future__ import annotations

import ast
import json
import sys
from pathlib import Path


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


root = Path(sys.argv[1]).resolve()
base_cfg = json.loads((root / "config/isaac_config.json").read_text(encoding="utf-8"))
auto_cfg = json.loads(
    (root / "tray_overlay/config/isaac_config_tray_integrated.json").read_text(encoding="utf-8")
)
demo_cfg = json.loads(
    (root / "tray_overlay/config/isaac_config_scan_straight_v216.json").read_text(encoding="utf-8")
)

domain = int(base_cfg["ros2"]["domain_id"])
require(domain == 115, f"final base domain changed unexpectedly: {domain}")
require(int(auto_cfg["ros2"]["domain_id"]) == domain, "tray auto domain differs from final")
require(int(demo_cfg["ros2"]["domain_id"]) == domain, "V2.17.13 demo domain differs from final")
print(f"[PASS] final/tray ROS_DOMAIN_ID={domain}")

required_files = [
    "app.py",
    "10_run_gui.sh",
    "tray_overlay/assets/medi_m.glb",
    "tray_overlay/assets/final_staff/doctor.glb",
    "tray_overlay/assets/final_staff/woman_doctor.glb",
    "tray_overlay/assets/final_staff/nurse_surgical_rigged.glb",
    "ros2_ws/src/hospital_tray_overlay/package.xml",
    "ros2_ws/src/hospital_tray_overlay/hospital_tray_overlay/cooperative_transport_manager.py",
    "RUN_TRAY_1_ISAAC_TOTAL_360.sh",
    "RUN_TRAY_2_AUTO_TOTAL_360.sh",
    "RUN_V217_1_ISAAC_SCAN_READY.sh",
    "RUN_V217_2_TRUE_ARUCO_DOCK_TRANSPORT.sh",
]
for relative in required_files:
    path = root / relative
    require(path.is_file() and path.stat().st_size > 0, f"missing integration file: {relative}")
print("[PASS] GUI base and tray runtime files coexist")

for marker_id in (40, 41, 42, 43, 44):
    marker = root / f"tray_overlay/markers/aruco_4x4_50_id_{marker_id}.png"
    require(marker.is_file() and marker.stat().st_size > 100, f"missing marker ID {marker_id}")
aruco_cfg = auto_cfg["tray_aruco_docking"]
require(aruco_cfg["amr1_outer_ids"] == [40, 41], "AMR1 outer IDs changed")
require(aruco_cfg["amr2_outer_ids"] == [42, 43], "AMR2 outer IDs changed")
require(int(aruco_cfg["center_id"]) == 44, "center marker must be ID 44")
print("[PASS] three-post ArUco gate: AMR1 40/41+44, AMR2 44+42/43")

bridge = (root / "tray_overlay/scripts/nav2_bridge.py").read_text(encoding="utf-8")
require("def _extract_horizontal_scan" in bridge, "360 LiDAR extraction fix missing")
require("np.squeeze" in bridge and "np.moveaxis" in bridge, "360 LiDAR array handling missing")
require("[NAV2 LIDAR 360 READY]" in bridge, "360 LiDAR readiness log missing")
print("[PASS] 360 LiDAR horizontal scan conversion present")

traffic = (
    root / "ros2_ws/src/hospital_nav2/hospital_nav2/path_conflict_manager.py"
).read_text(encoding="utf-8")
for token in (
    "/amr1/tray_docking_active",
    "/amr2/tray_docking_active",
    "TRAY_DOCK_BYPASS",
    "route_reservation/request",
):
    require(token in traffic, f"traffic integration token missing: {token}")
print("[PASS] existing route reservation + direct tray-docking bypass coexist")

v217 = (root / "scripts/v217_true_aruco_dock_transport.py").read_text(encoding="utf-8")
for token in (
    "TRANSPORT_V_FAST=1.98",
    "TRANSPORT_V_MID=0.72",
    "TRANSPORT_V_TIGHT=0.30",
    "/amr1/tray_cmd_vel",
    "/amr2/tray_cmd_vel",
):
    require(token in v217, f"V2.17.13 token missing: {token}")
print("[PASS] V2.17.13 direct docking and X3 transport constants present")

python_roots = [
    root / "tray_overlay/scripts",
    root / "ros2_ws/src/hospital_tray_overlay",
]
python_files = []
for folder in python_roots:
    python_files.extend(folder.rglob("*.py"))
python_files.extend(
    root / "scripts" / name
    for name in (
        "v217_true_aruco_dock_transport.py",
        "v217_true_aruco_pair_scanner.py",
        "v217_raw_lidar_map_points.py",
        "v21712_actual_speed_monitor.py",
    )
)
for path in python_files:
    ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
print(f"[PASS] Python AST validation: {len(python_files)} files")
print("[STATIC VALIDATION COMPLETE] Isaac/PhysX/Nav2/ArUco runtime test is still required.")
