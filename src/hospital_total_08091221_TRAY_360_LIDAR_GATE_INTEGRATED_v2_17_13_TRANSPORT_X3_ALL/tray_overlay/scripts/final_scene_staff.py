#!/usr/bin/env python3
"""Final demo staff placement overlay.

Runtime-only visual placement; baseline hospital files are not edited.
- doctor.glb: beside MRI table/machine, off the longitudinal AMR lane.
- woman_doctor.glb: beside the cooperative tray destination, outside cart footprint.
- nurse_surgical_rigged.glb: beside the detected desk/table chair.
All imported staff are made non-physical so they cannot block Nav2/PhysX traffic.
"""
from __future__ import annotations

import asyncio
import math
import time
from pathlib import Path
from typing import Any

import carb
import omni.kit.app
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

import hospital_staff as hs

_TASKS: list[asyncio.Task] = []


def _xy_unit(v: Gf.Vec3d) -> tuple[float, float]:
    x, y = float(v[0]), float(v[1])
    n = math.hypot(x, y)
    if n < 1.0e-9:
        return 1.0, 0.0
    return x / n, y / n


def _prim_center(stage: Usd.Stage, prim: Usd.Prim) -> Gf.Vec3d:
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=True,
    )
    bounds = hs._world_bounds(cache, prim)
    if bounds is not None:
        return hs._center(bounds)
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()


def _disable_physics(root: Usd.Prim) -> None:
    for prim in Usd.PrimRange(root):
        try:
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
        except Exception:
            pass
        # Some converted assets preserve direct enabled attributes even if API lookup is odd.
        for name in ("physics:rigidBodyEnabled", "physics:collisionEnabled"):
            try:
                attr = prim.GetAttribute(name)
                if attr and attr.IsValid():
                    attr.Set(False)
            except Exception:
                pass


def _entry_with_common(entry: dict[str, Any], common: dict[str, Any]) -> dict[str, Any]:
    out = dict(common)
    out.update(dict(entry))
    return out


async def _instantiate_person(
    stage: Usd.Stage,
    project_root: Path,
    entry: dict[str, Any],
    common: dict[str, Any],
    x: float,
    y: float,
    yaw_deg: float,
    fallback_z: float,
    anchor_note: str,
    manual_pose: dict[str, float] | None = None,
) -> None:
    e = _entry_with_common(entry, common)
    forced = e.get("forced_visual_rotation_xyz_deg")
    if isinstance(forced, (list, tuple)) and len(forced) >= 3:
        # doctor.glb is a T-pose: its arm span is slightly larger than its body
        # height, so the old bbox heuristic could choose the arms as vertical.
        e["upright_candidate_rotations_xyz_deg"] = [[float(forced[0]), float(forced[1]), float(forced[2])]]
    source = (project_root / str(e["source_asset"])).resolve()
    converted = (project_root / str(e["converted_usd"])).resolve()
    usd_path = await hs._convert_asset(source, converted)

    root_path = str(e["root_prim"])
    existing = stage.GetPrimAtPath(root_path)
    if existing and existing.IsValid():
        stage.RemovePrim(root_path)

    root = UsdGeom.Xform.Define(stage, Sdf.Path(root_path)).GetPrim()
    visual = UsdGeom.Xform.Define(stage, Sdf.Path(root_path + "/Visual")).GetPrim()
    visual.GetReferences().ClearReferences()
    visual.GetReferences().AddReference(str(usd_path))

    if manual_pose:
        x = float(manual_pose.get("x", x))
        y = float(manual_pose.get("y", y))
        yaw_deg = float(manual_pose.get("yaw_deg", yaw_deg))
        floor_z = float(manual_pose.get("z", fallback_z))
        anchor_note = anchor_note + ":MANUAL_CAPTURE"
    else:
        floor_z = hs._find_floor_top(stage, float(x), float(y), float(fallback_z))
    hs._set_root_pose(root, float(x), float(y), float(floor_z), float(yaw_deg))
    for _ in range(8):
        await omni.kit.app.get_app().next_update_async()

    chosen_rotation, chosen_scale = await hs._choose_upright_rotation(stage, visual, e)

    # V2.16.5 final human sanity guard.
    # The normal hospital_staff helper already chooses upright rotation and human height.
    # Here we only reject pathological giant/flat results and refit once from world bbox.
    sanity_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=True,
    )
    sanity_bounds = hs._world_bounds(sanity_cache, visual)
    if sanity_bounds is not None:
        sanity_min, sanity_max = sanity_bounds
        sx = float(sanity_max[0] - sanity_min[0])
        sy = float(sanity_max[1] - sanity_min[1])
        sz = float(sanity_max[2] - sanity_min[2])
        target_h = float(e.get("target_height_stage_units", 1.72))
        if (
            math.isfinite(sz)
            and sz > 1.0e-6
            and target_h > 0.1
            and (sz > 2.25 or sz < 1.20 or max(sx, sy) > 2.25)
        ):
            api = UsdGeom.XformCommonAPI(visual)
            _tr, rot_now, scale_now, _pivot, _order = api.GetXformVectors(Usd.TimeCode.Default())
            ratio = max(0.05, min(20.0, target_h / sz))
            api.SetScale(
                Gf.Vec3f(
                    float(scale_now[0]) * ratio,
                    float(scale_now[1]) * ratio,
                    float(scale_now[2]) * ratio,
                )
            )
            for _ in range(5):
                await omni.kit.app.get_app().next_update_async()
            chosen_scale = float(scale_now[0]) * ratio
            carb.log_warn(
                f"[V2.16.5 STAFF SANITY REFIT] {e.get('role','Staff')} "
                f"bbox_before=({sx:.3f},{sy:.3f},{sz:.3f}) "
                f"target_h={target_h:.3f} ratio={ratio:.5f}"
            )

    # Fit automatic placements to floor. A captured manual pose is authoritative
    # and must not be shifted by another floor search.
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=True,
    )
    bounds = hs._world_bounds(cache, visual)
    final_z = floor_z
    if bounds is not None and not manual_pose:
        minimum, _maximum = bounds
        correction = floor_z - float(minimum[2])
        max_corr = float(e.get("max_floor_fit_m", 3.0))
        if abs(correction) <= max_corr:
            final_z = floor_z + correction
            hs._set_root_pose(root, float(x), float(y), float(final_z), float(yaw_deg))

    _disable_physics(root)
    root.SetCustomDataByKey("hospitalStaffRole", str(e.get("role", "Staff")))
    root.SetCustomDataByKey("hospitalNonPhysicalVisual", True)
    root.SetCustomDataByKey("finalDemoPlacement", anchor_note)
    carb.log_info(
        f"[FINAL STAFF] {e.get('role','Staff')} anchor={anchor_note} "
        f"world=({x:.3f},{y:.3f},{final_z:.3f}) yaw={yaw_deg:.1f} "
        f"visualR={chosen_rotation} visualS={chosen_scale:.6f} NONPHYSICAL"
    )


async def _place_final_staff(
    stage: Usd.Stage,
    project_root: Path,
    cfg: dict[str, Any],
    baseline_staff_cfg: dict[str, Any],
    fixed_mri_cfg: dict[str, Any],
    auto_transport_cfg: dict[str, Any],
) -> None:
    if not bool(cfg.get("enabled", False)):
        return
    try:
        root_path = str(cfg.get("root_prim", "/World/HospitalStaff"))
        old = stage.GetPrimAtPath(root_path)
        if old and old.IsValid():
            # Replace old reception-only staff at runtime; no source USD is changed.
            stage.RemovePrim(root_path)
        UsdGeom.Xform.Define(stage, Sdf.Path(root_path))

        manual_poses: dict[str, Any] = {}
        if bool(cfg.get("manual_pose_override_enabled", True)):
            try:
                manual_path = (project_root / str(cfg.get("manual_pose_file", "tray_overlay/config/manual_staff_poses.json"))).resolve()
                if manual_path.is_file():
                    import json
                    payload = json.loads(manual_path.read_text(encoding="utf-8"))
                    manual_poses = dict(payload.get("staff", payload)) if isinstance(payload, dict) else {}
                    carb.log_info(f"[FINAL STAFF] manual pose overrides loaded: {manual_path}")
            except Exception as exc:
                carb.log_warn(f"[FINAL STAFF] manual pose file ignored: {exc}")

        common = {
            "upright_candidate_rotations_xyz_deg": cfg.get(
                "upright_candidate_rotations_xyz_deg",
                [[0,0,0],[90,0,0],[-90,0,0],[0,90,0],[0,-90,0]],
            ),
            "min_auto_scale": float(cfg.get("min_auto_scale", 1.0e-5)),
            "max_auto_scale": float(cfg.get("max_auto_scale", 100.0)),
            "max_floor_fit_m": float(cfg.get("max_floor_fit_m", 3.0)),
            "upright_compose_frames": 5,
        }

        # 1) DoctorMRI - V2.17.1 explicit safe world pose.
        # Automatic MRI-side placement was able to intersect the MRI mesh depending
        # on the imported table/machine transforms.  Final demo now gives DoctorMRI
        # an explicit clear-floor XY position and only auto-fits Z to the 2F floor.
        doctor = dict(cfg.get("doctor_mri", {}))
        table_path = str(doctor.get("mri_table_prim") or fixed_mri_cfg.get("source_table_prim", ""))
        table = stage.GetPrimAtPath(table_path)

        if table and table.IsValid():
            tc = _prim_center(stage, table)
            anchor_xyz = fixed_mri_cfg.get(
                "world_xyz_m",
                [float(tc[0]), float(tc[1]), float(tc[2])]
            )
            ax = float(anchor_xyz[0]) if len(anchor_xyz) > 0 else float(tc[0])
            ay = float(anchor_xyz[1]) if len(anchor_xyz) > 1 else float(tc[1])
            fallback_z = float(tc[2])

            fixed_doc = dict(doctor.get("fixed_world_pose_v2171", {}))
            if fixed_doc:
                dx = float(fixed_doc.get("x", 5.65))
                dy = float(fixed_doc.get("y", 7.45))
                yaw = float(
                    fixed_doc.get(
                        "yaw_deg",
                        math.degrees(math.atan2(ay - dy, ax - dx))
                    )
                )
                anchor_note = f"V2171_FIXED_MRI_CLEAR:{table_path}"
                # Do NOT pass this as a manual pose because manual z would bypass
                # automatic floor fitting.  XY/yaw are fixed; Z is still found from floor.
                manual_doc_pose = None
            else:
                world_yaw = math.radians(
                    float(
                        doctor.get(
                            "placement_yaw_deg",
                            fixed_mri_cfg.get("fallback_yaw_deg", 0.0)
                        )
                    )
                )
                forward = (math.cos(world_yaw), math.sin(world_yaw))
                side = (-forward[1], forward[0])
                side_sign = 1.0 if float(doctor.get("side_sign", 1.0)) >= 0.0 else -1.0
                side_offset = abs(float(doctor.get("side_offset_m", 1.35)))
                fwd_offset = float(doctor.get("forward_offset_m", 0.15))
                dx = ax + side_sign * side[0] * side_offset + forward[0] * fwd_offset
                dy = ay + side_sign * side[1] * side_offset + forward[1] * fwd_offset
                yaw = math.degrees(math.atan2(ay - dy, ax - dx))
                anchor_note = f"MRI_WORLD_YAW_SIDE:{table_path}"
                manual_doc_pose = manual_poses.get("DoctorMRI")

            await _instantiate_person(
                stage, project_root, doctor, common,
                dx, dy, yaw, fallback_z,
                anchor_note, manual_doc_pose
            )
        else:
            carb.log_warn(f"[FINAL STAFF] MRI table not found for doctor: {table_path}")

        # 2) V2.16.5 destination medical staff.
        # Final demo uses an explicit world pose chosen outside the loaded tray lane.
        woman = dict(cfg.get("woman_doctor_tray_goal", {}))
        goal = dict(auto_transport_cfg.get("fixed_target", {}))
        fixed_staff = dict(woman.get("fixed_world_pose_v2165", {}))
        if fixed_staff:
            wx = float(fixed_staff.get("x", 9.50))
            wy = float(fixed_staff.get("y", 7.00))
            wz = float(fixed_staff.get("z", 0.0))
            wyaw = float(fixed_staff.get("yaw_deg", -135.0))
            manual_goal_pose = {"x": wx, "y": wy, "z": wz, "yaw_deg": wyaw}
            anchor_note = "V2165_FIXED_GOAL_SIDE"
        else:
            gx, gy = float(goal.get("x", 7.90)), float(goal.get("y", 10.13))
            gyaw = math.radians(float(goal.get("yaw_deg", 0.0)))
            lateral = float(woman.get("lateral_offset_m", 1.95))
            forward_offset = float(woman.get("forward_offset_m", 0.25))
            fx, fy = math.cos(gyaw), math.sin(gyaw)
            sx, sy = -fy, fx
            wx = gx + sx * lateral + fx * forward_offset
            wy = gy + sy * lateral + fy * forward_offset
            wyaw = math.degrees(math.atan2(gy - wy, gx - wx))
            manual_goal_pose = manual_poses.get("WomanDoctorTrayGoal")
            anchor_note = "TRAY_GOAL_SIDE"

        await _instantiate_person(
            stage, project_root, woman, common, wx, wy, wyaw, 0.0,
            anchor_note, manual_goal_pose
        )

        # 3) Nurse: next to an actual chair in the desk/table cluster.
        nurse = dict(cfg.get("nurse_desk", {}))
        cluster = hs._find_station_cluster(stage, dict(baseline_staff_cfg or {}))
        if cluster is not None:
            anchor_path, anchor, chairs, _elevator_path, _elevator = cluster
            if chairs:
                chair_path, chair = chairs[0]
                # Use a tangent to desk->chair direction so the nurse is beside,
                # not behind/in front of, the chair.
                ux, uy = hs._unit_xy(float(chair[0] - anchor[0]), float(chair[1] - anchor[1]))
                sx2, sy2 = -uy, ux
                off = float(nurse.get("chair_side_offset_m", 0.48))
                nx, ny = float(chair[0]) + sx2 * off, float(chair[1]) + sy2 * off
                nyaw = math.degrees(math.atan2(float(anchor[1]) - ny, float(anchor[0]) - nx))
                await _instantiate_person(
                    stage, project_root, nurse, common, nx, ny, nyaw, float(anchor[2]),
                    f"CHAIR_SIDE:{chair_path}", manual_poses.get("NurseDesk")
                )
            else:
                carb.log_warn("[FINAL STAFF] desk cluster found but no chair; nurse skipped")
        else:
            carb.log_warn("[FINAL STAFF] desk/table chair cluster not found; nurse skipped")

        carb.log_info(
            "[FINAL STAFF READY] doctor=MRI side, woman_doctor=tray-goal side, nurse=desk-chair side; all non-physical"
        )
    except Exception as exc:
        carb.log_warn(f"[FINAL STAFF WARNING] placement skipped without stopping Isaac: {exc}")



def install_final_scene_staff_sync(
    stage: Usd.Stage,
    project_root: Path,
    cfg: dict[str, Any],
    baseline_staff_cfg: dict[str, Any],
    fixed_mri_cfg: dict[str, Any],
    auto_transport_cfg: dict[str, Any],
    app_update,
    timeout_sec: float = 120.0,
) -> None:
    """Complete all final staff Stage edits before physics runtime."""
    if not bool((cfg or {}).get("enabled", False)):
        return
    loop = asyncio.get_event_loop()
    task = loop.create_task(_place_final_staff(
        stage, project_root, dict(cfg or {}), dict(baseline_staff_cfg or {}),
        dict(fixed_mri_cfg or {}), dict(auto_transport_cfg or {}),
    ))
    deadline=time.monotonic()+float(timeout_sec)
    while not task.done():
        app_update()
        loop.run_until_complete(asyncio.sleep(0))
        if time.monotonic()>deadline:
            task.cancel()
            raise RuntimeError("V2.16.6 final staff synchronous placement timed out")
    task.result()
    carb.log_info("[V2.16.6 STAFF BARRIER PASS] all staff USD edits completed before runtime")

def schedule_final_scene_staff(
    stage: Usd.Stage,
    project_root: Path,
    cfg: dict[str, Any],
    baseline_staff_cfg: dict[str, Any],
    fixed_mri_cfg: dict[str, Any],
    auto_transport_cfg: dict[str, Any],
) -> None:
    task = asyncio.ensure_future(
        _place_final_staff(
            stage,
            project_root,
            dict(cfg or {}),
            dict(baseline_staff_cfg or {}),
            dict(fixed_mri_cfg or {}),
            dict(auto_transport_cfg or {}),
        )
    )
    _TASKS.append(task)
