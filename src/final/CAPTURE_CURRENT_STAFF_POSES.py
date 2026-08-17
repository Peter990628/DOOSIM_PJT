"""Run THIS FILE in the Isaac Sim Script Editor while your manually arranged scene is open.

It stores the current root world poses of the three final-demo staff into the
V2.12 sidecar config.  V2.12 then recreates the assets at exactly these captured
root positions/yaws every launch.  AMR poses are also recorded only as a useful
snapshot; Nav2 V2.12 itself locks from live /amr*/world_pose and does not use
these AMR snapshot numbers.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import time

import omni.usd
from pxr import Gf, Usd, UsdGeom


def _project_root() -> Path:
    env = os.environ.get("HOSPITAL_TRAY_PROJECT_ROOT", "").strip()
    if env and Path(env).is_dir():
        return Path(env).resolve()
    exact = Path.home() / "hospital" / "final"
    if exact.is_dir():
        return exact
    home = Path.home() / "hospital_bed_amr_projects"
    candidates = sorted(home.glob("hospital_total_08091221_TRAY_360_LIDAR_GATE_INTEGRATED_v2_12*"))
    if candidates:
        return candidates[-1].resolve()
    raise RuntimeError("V2.12 project folder not found under ~/hospital_bed_amr_projects")


def _pose(stage: Usd.Stage, path: str):
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return None
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    p = m.ExtractTranslation()
    f = m.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))
    yaw = math.degrees(math.atan2(float(f[1]), float(f[0])))
    return {"x": float(p[0]), "y": float(p[1]), "z": float(p[2]), "yaw_deg": float(yaw)}


def main() -> None:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No open Isaac stage")
    root = _project_root()
    staff_paths = {
        "DoctorMRI": "/World/HospitalStaff/DoctorMRI",
        "WomanDoctorTrayGoal": "/World/HospitalStaff/WomanDoctorTrayGoal",
        "NurseDesk": "/World/HospitalStaff/NurseDesk",
    }
    amr_paths = {"AMR1": "/World/AMR1/base_link", "AMR2": "/World/AMR2/base_link"}
    staff = {name: pose for name, path in staff_paths.items() if (pose := _pose(stage, path)) is not None}
    amr = {name: pose for name, path in amr_paths.items() if (pose := _pose(stage, path)) is not None}
    out = root / "tray_overlay/config/manual_staff_poses.json"
    payload = {
        "captured_at_unix": time.time(),
        "note": "Captured from the user's manually arranged Isaac stage. Staff root world poses are authoritative in V2.12.",
        "staff": staff,
        "amr_start_snapshot_only": amr,
    }
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print("\n================ V2.12 MANUAL POSE CAPTURE ================")
    print(f"[SAVED] {out}")
    for name, pose in staff.items():
        print(f"[STAFF] {name}: x={pose['x']:.4f} y={pose['y']:.4f} z={pose['z']:.4f} yaw={pose['yaw_deg']:.2f}deg")
    for name, pose in amr.items():
        print(f"[AMR SNAPSHOT] {name}: x={pose['x']:.4f} y={pose['y']:.4f} yaw={pose['yaw_deg']:.2f}deg")
    print("[DONE] Restart with V2.12; captured staff placement will be reapplied.")
    print("============================================================\n")


main()
