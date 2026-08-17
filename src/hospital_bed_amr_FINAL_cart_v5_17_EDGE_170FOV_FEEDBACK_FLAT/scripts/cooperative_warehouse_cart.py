#!/usr/bin/env python3
"""Cooperative warehouse cart for the existing hospital stage.

Design intent
-------------
- Reference concept: a low warehouse display/cart on four passive casters.
- No side pods, no AMR-specific exterior protrusions.
- The normal cargo deck spans the full cart footprint.
- AMR1 and AMR2 enter the open under-deck volume side-by-side, facing the
  same direction, lift 35 mm, contact two underside dock pads, and are fixed
  to the cart with two runtime FixedJoints.
- The cart's four passive caster contacts carry the vertical load. The two
  AMRs provide traction and steering only.
- During cooperative mode, AMR commands are computed from a virtual cart
  center so the two robots move as one rigid transport system.

The caster physics deliberately uses low-friction spherical contact proxies
under visual caster wheels. This is much more stable than eight swivel/axle
joints for the two-day demo while preserving the visible four-caster design.
"""
from __future__ import annotations

import asyncio
import math
import time
from pathlib import Path
from typing import Any, Callable

import omni.kit.asset_converter
import omni.kit.commands
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


def _world_matrix(prim: Usd.Prim) -> Gf.Matrix4d:
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def _world_bounds(prim: Usd.Prim):
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=True,
    )
    box = cache.ComputeWorldBound(prim).ComputeAlignedBox()
    return Gf.Vec3d(box.GetMin()), Gf.Vec3d(box.GetMax())


def _center_from_bounds(prim: Usd.Prim) -> Gf.Vec3d:
    mn, mx = _world_bounds(prim)
    return (mn + mx) * 0.5


def _size_from_bounds(prim: Usd.Prim) -> tuple[float, float, float]:
    mn, mx = _world_bounds(prim)
    return (float(mx[0]-mn[0]), float(mx[1]-mn[1]), float(mx[2]-mn[2]))


def _set_pose(prim: Usd.Prim, xyz: tuple[float,float,float], yaw_deg: float = 0.0) -> None:
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    api = UsdGeom.XformCommonAPI(prim)
    api.SetTranslate(Gf.Vec3d(*[float(v) for v in xyz]))
    api.SetRotate(Gf.Vec3f(0.0, 0.0, float(yaw_deg)), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    api.SetScale(Gf.Vec3f(1.0,1.0,1.0))


def _define_cube(
    stage: Usd.Stage,
    path: str,
    size: tuple[float,float,float],
    center: tuple[float,float,float],
    color: tuple[float,float,float],
    collision: bool = False,
) -> Usd.Prim:
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    api = UsdGeom.XformCommonAPI(cube.GetPrim())
    api.SetTranslate(Gf.Vec3d(*center))
    api.SetScale(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collision:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
    return cube.GetPrim()


def _define_sphere(
    stage: Usd.Stage,
    path: str,
    radius: float,
    center: tuple[float,float,float],
    color: tuple[float,float,float],
    collision: bool = False,
) -> Usd.Prim:
    sphere = UsdGeom.Sphere.Define(stage, path)
    sphere.CreateRadiusAttr(float(radius))
    api = UsdGeom.XformCommonAPI(sphere.GetPrim())
    api.SetTranslate(Gf.Vec3d(*center))
    sphere.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collision:
        UsdPhysics.CollisionAPI.Apply(sphere.GetPrim()).CreateCollisionEnabledAttr(True)
    return sphere.GetPrim()


def _create_trolley_lidar(
    stage: Usd.Stage,
    root_path: str,
    cfg: dict[str, Any],
    deck_top: float,
) -> str | None:
    """Create a trolley-mounted 360 deg PhysX lidar entirely from Python.

    This deliberately reuses the same RangeSensorCreateLidar API already used
    by nav2_bridge.py for the AMRs, so no Action Graph or GUI-created sensor
    prim is required.  The sensor is parented to the runtime cart root and
    therefore follows the cart automatically.
    """
    lidar_cfg = dict(cfg.get("trolley_lidar", {}))
    if not bool(lidar_cfg.get("enabled", True)):
        print("[CARGO CART LIDAR] disabled by config")
        return None

    prim_name = str(lidar_cfg.get("prim_name", "trolley_lidar"))
    full_path = f"{root_path}/{prim_name}"
    old = stage.GetPrimAtPath(full_path)
    if old and old.IsValid():
        stage.RemovePrim(full_path)

    # If Z is null/omitted, put the sensor safely above the two cargo layers.
    translation = list(lidar_cfg.get("translation", [0.0, 0.0, None]))
    while len(translation) < 3:
        translation.append(None)
    tx = float(translation[0] or 0.0)
    ty = float(translation[1] or 0.0)
    tz = float(translation[2]) if translation[2] is not None else float(deck_top + 0.55)

    result, _prim = omni.kit.commands.execute(
        "RangeSensorCreateLidar",
        path=f"/{prim_name}",
        parent=root_path,
        translation=Gf.Vec3d(tx, ty, tz),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        min_range=float(lidar_cfg.get("min_range_m", 0.15)),
        max_range=float(lidar_cfg.get("max_range_m", 12.0)),
        draw_points=bool(lidar_cfg.get("draw_points", True)),
        draw_lines=bool(lidar_cfg.get("draw_lines", False)),
        horizontal_fov=360.0,
        vertical_fov=float(lidar_cfg.get("vertical_fov_deg", 1.0)),
        horizontal_resolution=float(lidar_cfg.get("horizontal_resolution_deg", 0.5)),
        vertical_resolution=float(lidar_cfg.get("vertical_resolution_deg", 1.0)),
        rotation_rate=float(lidar_cfg.get("rotation_rate_hz", 0.0)),
        high_lod=False,
        yaw_offset=float(lidar_cfg.get("yaw_offset_deg", 0.0)),
        enable_semantics=False,
    )
    if not result:
        raise RuntimeError(f"Could not create trolley PhysX lidar: {full_path}")

    sensor_prim = stage.GetPrimAtPath(full_path)
    if sensor_prim and sensor_prim.IsValid():
        sensor_prim.SetCustomDataByKey("trolleySensor", True)
        sensor_prim.SetCustomDataByKey("sensorRole", "cooperative_navigation_lidar")

    print(
        f"[CARGO CART LIDAR] created {full_path} local=({tx:.3f},{ty:.3f},{tz:.3f}) "
        f"range={float(lidar_cfg.get('min_range_m',0.15)):.2f}-"
        f"{float(lidar_cfg.get('max_range_m',12.0)):.2f}m"
    )
    return full_path


def _define_wheel_visual(
    stage: Usd.Stage,
    path: str,
    radius: float,
    width: float,
    center: tuple[float,float,float],
) -> Usd.Prim:
    # Cylinder's default axis is Z. Rotate to a horizontal axle for caster look.
    wheel = UsdGeom.Cylinder.Define(stage, path)
    wheel.CreateRadiusAttr(float(radius))
    wheel.CreateHeightAttr(float(width))
    wheel.CreateAxisAttr(UsdGeom.Tokens.y)
    api = UsdGeom.XformCommonAPI(wheel.GetPrim())
    api.SetTranslate(Gf.Vec3d(*center))
    wheel.CreateDisplayColorAttr([Gf.Vec3f(0.035,0.035,0.04)])
    return wheel.GetPrim()


def _bind_physics_material(stage: Usd.Stage, prim: Usd.Prim, path: str, static_f: float, dynamic_f: float) -> None:
    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    phys = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    phys.CreateStaticFrictionAttr(float(static_f)).Set(float(static_f))
    phys.CreateDynamicFrictionAttr(float(dynamic_f)).Set(float(dynamic_f))
    phys.CreateRestitutionAttr(0.0).Set(0.0)
    binding = UsdShade.MaterialBindingAPI.Apply(prim)
    try:
        binding.Bind(mat, materialPurpose="physics")
    except Exception:
        rel = prim.CreateRelationship("material:binding:physics", custom=False)
        rel.SetTargets([mat.GetPath()])


def _find_best_rigid_body(root: Usd.Prim, preferred: tuple[str,...] = ()) -> Usd.Prim | None:
    bodies: list[Usd.Prim] = []
    if root and root.IsValid() and root.HasAPI(UsdPhysics.RigidBodyAPI):
        bodies.append(root)
    if root and root.IsValid():
        for prim in Usd.PrimRange(root):
            if prim != root and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                bodies.append(prim)
    if not bodies:
        return None
    def score(p: Usd.Prim):
        n = p.GetName().lower()
        s = sum(20 for token in preferred if token in n)
        if any(t in n for t in ("wheel","caster")):
            s -= 50
        return (s, -len(str(p.GetPath())))
    return max(bodies, key=score)


def _lift_body(controller: Any) -> Usd.Prim:
    lift = controller.stage.GetPrimAtPath(f"{controller.root}/lift_plate")
    if lift and lift.IsValid():
        body = _find_best_rigid_body(lift, ("lift","plate","adapter"))
        if body is not None:
            return body
    body = _find_best_rigid_body(controller.base_prim, ("base",))
    if body is None:
        raise RuntimeError(f"No rigid body available for {controller.name}")
    return body


def _quatd_to_quatf(q: Gf.Quatd) -> Gf.Quatf:
    im = q.GetImaginary()
    return Gf.Quatf(float(q.GetReal()), Gf.Vec3f(float(im[0]),float(im[1]),float(im[2])))


def _create_fixed_joint(stage: Usd.Stage, path: str, body0: Usd.Prim, body1: Usd.Prim, cfg: dict[str,Any]) -> None:
    old = stage.GetPrimAtPath(path)
    if old and old.IsValid():
        stage.RemovePrim(path)
    m0 = _world_matrix(body0)
    m1 = _world_matrix(body1)
    anchor = m0.ExtractTranslation()
    local1d = m1.GetInverse().Transform(anchor)
    local1 = Gf.Vec3f(float(local1d[0]),float(local1d[1]),float(local1d[2]))
    local_rot1 = m1.ExtractRotationQuat().GetInverse() * m0.ExtractRotationQuat()
    joint = UsdPhysics.FixedJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([body0.GetPath()])
    joint.CreateBody1Rel().SetTargets([body1.GetPath()])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0,0.0,0.0))
    joint.CreateLocalPos1Attr().Set(local1)
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateLocalRot1Attr().Set(_quatd_to_quatf(local_rot1))
    joint.CreateCollisionEnabledAttr(False)
    joint.CreateExcludeFromArticulationAttr(True)
    joint.CreateJointEnabledAttr(True)
    joint.CreateBreakForceAttr().Set(float(cfg.get("break_force_n",1.0e9)))
    joint.CreateBreakTorqueAttr().Set(float(cfg.get("break_torque_nm",1.0e9)))


def _run_async(coro, app_update: Callable[[],None]):
    loop = asyncio.get_event_loop()
    task = loop.create_task(coro)
    while not task.done():
        app_update()
        loop.run_until_complete(asyncio.sleep(0))
    return task.result()


async def _convert_asset(source: Path, output: Path) -> Path:
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.is_file() and output.stat().st_size > 1024:
        return output
    converter = omni.kit.asset_converter.get_instance()
    context = omni.kit.asset_converter.AssetConverterContext()
    for key,val in {
        "keep_all_materials": True,
        "merge_all_meshes": False,
        "ignore_materials": False,
        "ignore_animations": True,
        "embed_textures": False,
        "use_meter_as_world_unit": True,
    }.items():
        if hasattr(context,key):
            setattr(context,key,val)
    task = converter.create_converter_task(str(source),str(output),None,context)
    ok = await task.wait_until_finished()
    if not ok or not output.is_file():
        raise RuntimeError("cargo medi_m.glb conversion failed")
    return output


def _cargo_template(stage: Usd.Stage, usd_path: Path, app_update: Callable[[],None]) -> Usd.Prim | None:
    lib_path = "/World/CooperativeWarehouseCartAssetLibrary"
    old = stage.GetPrimAtPath(lib_path)
    if old and old.IsValid():
        stage.RemovePrim(lib_path)
    lib = UsdGeom.Xform.Define(stage, lib_path).GetPrim()
    lib.GetReferences().AddReference(str(usd_path))
    try:
        UsdGeom.Imageable(lib).MakeInvisible()
    except Exception:
        pass
    for _ in range(20):
        app_update()
    candidates=[]
    for p in Usd.PrimRange(lib):
        n=p.GetName().lower()
        if ("object_0" in n or "shipper" in n) and p.IsA(UsdGeom.Imageable):
            candidates.append(p)
    if not candidates:
        for p in Usd.PrimRange(lib):
            if p.IsA(UsdGeom.Mesh) and "object_1" not in p.GetName().lower():
                candidates.append(p)
    if not candidates:
        return None
    return max(candidates,key=lambda p: math.prod(max(v,1e-6) for v in _size_from_bounds(p)))


def _normalize_box(prim: Usd.Prim, desired_world_center: tuple[float,float,float], target_size: tuple[float,float,float], app_update: Callable[[],None]) -> None:
    best=None
    for rot in ((0,0,0),(90,0,0),(-90,0,0),(0,90,0),(0,-90,0),(0,0,90),(0,0,-90)):
        xf=UsdGeom.Xformable(prim); xf.ClearXformOpOrder()
        api=UsdGeom.XformCommonAPI(prim)
        api.SetTranslate(Gf.Vec3d(0,0,0)); api.SetRotate(Gf.Vec3f(*rot),UsdGeom.XformCommonAPI.RotationOrderXYZ); api.SetScale(Gf.Vec3f(1,1,1))
        app_update()
        d=_size_from_bounds(prim)
        if min(d)<1e-6: continue
        nd=[v/max(d) for v in d]; nt=[v/max(target_size) for v in target_size]
        score=sum((nd[i]-nt[i])**2 for i in range(3))
        if best is None or score<best[0]: best=(score,rot,d)
    if best is None: return
    _,rot,d=best
    ratios=[target_size[i]/d[i] for i in range(3) if d[i]>1e-6]
    scale=sum(ratios)/len(ratios)
    xf=UsdGeom.Xformable(prim); xf.ClearXformOpOrder(); api=UsdGeom.XformCommonAPI(prim)
    api.SetTranslate(Gf.Vec3d(0,0,0)); api.SetRotate(Gf.Vec3f(*rot),UsdGeom.XformCommonAPI.RotationOrderXYZ); api.SetScale(Gf.Vec3f(scale,scale,scale))
    app_update()
    c_world=_center_from_bounds(prim)
    parent_world=_world_matrix(prim.GetParent())
    inv_parent=parent_world.GetInverse()
    c_local=inv_parent.Transform(c_world)
    d_local=inv_parent.Transform(Gf.Vec3d(*desired_world_center))
    api.SetTranslate(Gf.Vec3d(float(d_local[0]-c_local[0]),float(d_local[1]-c_local[1]),float(d_local[2]-c_local[2])))
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            try: p.RemoveAPI(UsdPhysics.RigidBodyAPI)
            except Exception: pass
        if p.HasAPI(UsdPhysics.CollisionAPI):
            try: p.RemoveAPI(UsdPhysics.CollisionAPI)
            except Exception: pass


def _prim_name_score(prim: Usd.Prim, tokens: tuple[str, ...]) -> int:
    """Score a stage prim by exact/substring name match and plausible 1F object size."""
    if not prim or not prim.IsValid():
        return -10_000
    name = prim.GetName().lower()
    path = str(prim.GetPath()).lower()
    score = 0
    for token in tokens:
        t = token.lower()
        if name == t:
            score += 200
        elif t in name:
            score += 100
        elif t in path:
            score += 35
    if score <= 0:
        return score
    try:
        mn, mx = _world_bounds(prim)
        sx, sy, sz = float(mx[0]-mn[0]), float(mx[1]-mn[1]), float(mx[2]-mn[2])
        if float(mn[2]) < 2.0 and float(mx[2]) > -0.25:
            score += 25
        if 0.15 <= max(sx, sy) <= 8.0 and sz <= 4.0:
            score += 15
        # Prefer the highest-level object root over a tiny material/mesh leaf.
        score -= max(0, len(str(prim.GetPath()).split('/')) - 6)
    except Exception:
        pass
    return score


def _find_named_scene_prim(stage: Usd.Stage, tokens: tuple[str, ...]) -> Usd.Prim | None:
    best = None
    best_score = -10_000
    for prim in stage.Traverse():
        score = _prim_name_score(prim, tokens)
        if score > best_score:
            best_score, best = score, prim
    return best if best_score > 0 else None


def _candidate_obstacle_score(
    stage: Usd.Stage,
    center_xy: tuple[float, float],
    yaw_deg: float,
    length: float,
    width: float,
    ignore_prefixes: tuple[str, ...],
) -> float:
    """Cheap 2D clearance score used only to choose which side of the desk is open.

    We intentionally ignore floor-like/ceiling-like and huge environment meshes. The
    goal is not path planning; it is just to reject the wall/office side of a desk.
    """
    cx, cy = center_xy
    a = math.radians(float(yaw_deg))
    ca, sa = abs(math.cos(a)), abs(math.sin(a))
    hx = 0.5 * (ca * length + sa * width) + 0.18
    hy = 0.5 * (sa * length + ca * width) + 0.18
    cminx, cmaxx = cx-hx, cx+hx
    cminy, cmaxy = cy-hy, cy+hy
    score = 0.0
    seen = 0
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if any(p.startswith(pref) for pref in ignore_prefixes):
            continue
        if not (prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Cube)):
            continue
        try:
            mn, mx = _world_bounds(prim)
            sx, sy, sz = float(mx[0]-mn[0]), float(mx[1]-mn[1]), float(mx[2]-mn[2])
            # Ignore floors, ceilings, tiny decorative slivers, and giant map shells.
            if sz < 0.06 and max(sx, sy) > 2.5:
                continue
            if sx > 18.0 or sy > 18.0 or sz > 8.0:
                continue
            if float(mx[2]) < 0.12 or float(mn[2]) > 1.7:
                continue
            ix = max(0.0, min(cmaxx, float(mx[0])) - max(cminx, float(mn[0])))
            iy = max(0.0, min(cmaxy, float(mx[1])) - max(cminy, float(mn[1])))
            if ix > 0.0 and iy > 0.0:
                score += ix * iy * (1.0 + min(sz, 2.0))
                seen += 1
                if seen > 120:
                    break
        except Exception:
            continue
    return score


def _elevator_desk_mid_pose(stage: Usd.Stage, cfg: dict[str, Any], elevator: Any) -> tuple[float,float,float,float]:
    """Return the user-measured fixed cart pose (V4).

    No scene search, no desk/elevator bounding-box inference, and no automatic
    translation offsets are used. The user measured the vending machine and
    reception desk world coordinates directly in Isaac Sim.
    """
    placement = cfg.get("placement", {})
    fixed = placement.get("fixed_world_xyz_m", [-28.7106, 11.3750, 0.0])
    yaw = float(placement.get("fixed_yaw_deg", -52.6862796385433))
    x = float(fixed[0])
    y = float(fixed[1])
    floor_z = float(placement.get("floor_z_m", fixed[2] if len(fixed) > 2 else 0.0))
    vending = placement.get("vending_world_xy_m", [-32.4262, 16.2500])
    desk = placement.get("desk_world_xy_m", [-24.9950, 6.5000])
    print(
        f"[CARGO CART POSITION V4] USER FIXED anchors: "
        f"vending=({float(vending[0]):.4f},{float(vending[1]):.4f}) "
        f"desk=({float(desk[0]):.4f},{float(desk[1]):.4f})"
    )
    print(
        f"[CARGO CART POSITION V4] EXACT USER MIDPOINT: "
        f"pose=({x:.4f},{y:.4f},{floor_z:.4f}) yaw={yaw:.2f}deg"
    )
    return x, y, floor_z, yaw


def _prephysics_move_controller_to_world_dock(
    controller: Any,
    target_x: float,
    target_y: float,
    target_yaw_deg: float,
    floor_z: float,
    app_update: Callable[[], None],
) -> None:
    """Move the *existing* AMR root before timeline.play(), so no old-position robot remains."""
    root = controller.stage.GetPrimAtPath(controller.root)
    if not root or not root.IsValid():
        raise RuntimeError(f"AMR root missing: {controller.root}")
    # Provisional placement keeps the robot's lowest point on the requested floor.
    try:
        mn, _mx = _world_bounds(root)
        root_pos = _world_matrix(root).ExtractTranslation()
        root_z = float(root_pos[2]) + (float(floor_z) - float(mn[2]))
    except Exception:
        root_z = float(floor_z)
    _set_pose(root, (float(target_x), float(target_y), root_z), float(target_yaw_deg))
    for _ in range(3):
        app_update()
    # Correct XY by the actual base_link center after rotation, and Z by actual ground contact.
    try:
        center = _center_from_bounds(controller.base_prim)
        mn, _mx = _world_bounds(root)
        root_pos = _world_matrix(root).ExtractTranslation()
        _set_pose(
            root,
            (
                float(root_pos[0]) + (float(target_x) - float(center[0])),
                float(root_pos[1]) + (float(target_y) - float(center[1])),
                float(root_pos[2]) + (float(floor_z) - float(mn[2])),
            ),
            float(target_yaw_deg),
        )
        for _ in range(3):
            app_update()
    except Exception as exc:
        print(f"[CARGO CART START WARNING] final AMR pre-physics dock correction skipped: {exc}")
    controller.halt()
    print(
        f"[CARGO CART START] moved existing {controller.root} to cart dock BEFORE physics: "
        f"({target_x:.3f},{target_y:.3f}) yaw={target_yaw_deg:.1f}"
    )

def _build_cart(stage: Usd.Stage, project_root: Path, cfg: dict[str,Any], controllers: list[Any], elevator: Any, app_update: Callable[[],None]) -> tuple[Usd.Prim,dict[str,Any]]:
    root_path=str(cfg.get("root_path","/World/CooperativeWarehouseCart"))
    old=stage.GetPrimAtPath(root_path)
    if old and old.IsValid():
        stage.RemovePrim(root_path)
    # Defensive cleanup of temporary clone artifacts only; the real /World/AMR1 and /World/AMR2 are preserved and moved.
    for tmp in ("/World/AMR2__AMR1_CLONE_TMP", "/World/AMR2__OLD_BACKUP_TMP"):
        prim=stage.GetPrimAtPath(tmp)
        if prim and prim.IsValid():
            stage.RemovePrim(tmp)

    root=UsdGeom.Xform.Define(stage,root_path).GetPrim()
    x,y,floor_z,yaw=_elevator_desk_mid_pose(stage,cfg,elevator)
    _set_pose(root,(x,y,floor_z),yaw)
    rb=UsdPhysics.RigidBodyAPI.Apply(root)
    rb.CreateRigidBodyEnabledAttr(True)
    rb.CreateKinematicEnabledAttr(False)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(float(cfg.get("cart_mass_kg",75.0)) + float(cfg.get("equivalent_cargo_mass_kg",140.0)))

    geom=cfg.get("geometry",{})
    length=float(geom.get("length_m",2.20)); width=float(geom.get("width_m",1.70)); deck_t=float(geom.get("deck_thickness_m",0.075))
    caster_r=float(geom.get("caster_radius_m",0.075)); caster_width=float(geom.get("caster_visual_width_m",0.045))
    corner_inset=float(geom.get("caster_corner_inset_m",0.13))
    side_wall_t=float(geom.get("bay_side_wall_thickness_m",0.05))
    center_wall_t=float(geom.get("bay_center_wall_thickness_m",0.06))
    body_floor_clearance=float(geom.get("body_floor_clearance_m",0.10))

    # Measure lift plate top before moving robots. Root relocation below preserves this relative geometry.
    lift_tops=[]
    for c in controllers[:2]:
        lift=stage.GetPrimAtPath(f"{c.root}/lift_plate")
        if lift and lift.IsValid():
            try:
                _mn,_mx=_world_bounds(lift); lift_tops.append(float(_mx[2]))
            except Exception: pass
    measured=max(lift_tops) if lift_tops else float(geom.get("fallback_lift_top_z_m",0.174))
    contact_gap=float(geom.get("lift_contact_gap_m",0.015))
    dock_contact_z=measured+contact_gap
    underside=max(float(geom.get("minimum_deck_underside_m",0.335)), measured+float(geom.get("underdeck_headroom_above_lift_m",0.155)))
    deck_center_z=underside+deck_t*0.5; deck_top=underside+deck_t

    # BOX-SHAPED BODY: full cargo roof/deck + continuous left/right walls + central divider.
    # This creates exactly two rectangular through-bays carved into the lower body for AMR1/AMR2.
    _define_cube(stage,root_path+"/CargoDeck",(length,width,deck_t),(0.0,0.0,deck_center_z),(0.15,0.16,0.17),True)
    lower_h=max(0.12, underside-body_floor_clearance)
    lower_z=body_floor_clearance+lower_h*0.5
    _define_cube(stage,root_path+"/BodyLeftWall", (length,side_wall_t,lower_h),(0.0, width*0.5-side_wall_t*0.5,lower_z),(0.23,0.24,0.25),True)
    _define_cube(stage,root_path+"/BodyRightWall",(length,side_wall_t,lower_h),(0.0,-width*0.5+side_wall_t*0.5,lower_z),(0.23,0.24,0.25),True)
    _define_cube(stage,root_path+"/BodyCenterDivider",(length,center_wall_t,lower_h),(0.0,0.0,lower_z),(0.23,0.24,0.25),True)
    # A thick fascia immediately below the deck keeps the silhouette box-like instead of table-like.
    fascia_h=float(geom.get("upper_fascia_height_m",0.07))
    fascia_z=underside-fascia_h*0.5
    _define_cube(stage,root_path+"/UpperFrontFascia",(side_wall_t,width,fascia_h),( length*0.5-side_wall_t*0.5,0.0,fascia_z),(0.20,0.21,0.22),True)
    _define_cube(stage,root_path+"/UpperRearFascia", (side_wall_t,width,fascia_h),(-length*0.5+side_wall_t*0.5,0.0,fascia_z),(0.20,0.21,0.22),True)

    # Bay geometry is derived from the actual outer width; AMRs are centered in each carved opening.
    bay_w=(width-2.0*side_wall_t-center_wall_t)*0.5
    dock_y=(center_wall_t+bay_w)*0.5
    amr_width=float(geom.get("amr_width_m",0.74))
    bay_clearance=max(0.0,bay_w-amr_width)

    # Four warehouse casters carry the vertical load.
    hx=length*0.5-corner_inset; hy=width*0.5-corner_inset
    for name,sx,sy in (("FL",1,1),("FR",1,-1),("RL",-1,1),("RR",-1,-1)):
        cx,cy=sx*hx,sy*hy
        _define_cube(stage,f"{root_path}/Casters/{name}/Fork",(0.10,0.085,0.065),(cx,cy,caster_r+0.070),(0.18,0.18,0.19),False)
        _define_wheel_visual(stage,f"{root_path}/Casters/{name}/Wheel",caster_r,caster_width,(cx,cy,caster_r))
        proxy=_define_sphere(stage,f"{root_path}/Casters/{name}/ContactProxy",caster_r,(cx,cy,caster_r),(0.03,0.03,0.035),True)
        _bind_physics_material(stage,proxy,"/World/HospitalRuntimeMaterials/CoopCartCasterLowFriction",float(cfg.get("caster_static_friction",0.035)),float(cfg.get("caster_dynamic_friction",0.02)))

    # Two magnetic dock pads inside the two carved AMR bays.
    dock_x=float(geom.get("dock_x_m",0.0)); pad_len=float(geom.get("dock_pad_length_m",0.44)); pad_w=min(float(geom.get("dock_pad_width_m",0.34)), bay_w*0.65); pad_t=float(geom.get("dock_pad_thickness_m",0.012))
    pad_z=dock_contact_z+pad_t*0.5
    for idx,dy in ((1,dock_y),(2,-dock_y)):
        _define_cube(stage,f"{root_path}/DockPad_AMR{idx}",(pad_len,pad_w,pad_t),(dock_x,dy,pad_z),(0.08,0.10,0.12),False)
        point=UsdGeom.Xform.Define(stage,f"{root_path}/DockPoint_AMR{idx}").GetPrim(); _set_pose(point,(dock_x,dy,0.0),0.0)

    # Runtime trolley sensor: created from code, no GUI/Action Graph required.
    trolley_lidar_path = _create_trolley_lidar(stage, root_path, cfg, deck_top)

    # Move the EXISTING AMR1/AMR2 roots into those bays before physics starts.
    cart_world=_world_matrix(root)
    forward=cart_world.TransformDir(Gf.Vec3d(1,0,0)); cart_yaw=math.degrees(math.atan2(float(forward[1]),float(forward[0])))
    for i,c in enumerate(controllers[:2]):
        local=Gf.Vec3d(dock_x, dock_y if i==0 else -dock_y, 0.0)
        wp=cart_world.Transform(local)
        _prephysics_move_controller_to_world_dock(c,float(wp[0]),float(wp[1]),cart_yaw,floor_z,app_update)
        try:
            c._set_lift_raised(False,0.0)
        except Exception:
            pass

    # Cargo: use the actual Shipper mesh but force each carton to lie flat (short 0.22 m dimension vertical).
    cargo_cfg=cfg.get("cargo",{})
    source=project_root/str(cargo_cfg.get("source_glb","cargo_cart_assets/source/medi_m.glb"))
    converted=project_root/str(cargo_cfg.get("converted_usd","cargo_cart_assets/converted/medi_m.usd"))
    template=None
    try:
        usd=_run_async(_convert_asset(source,converted),app_update)
        template=_cargo_template(stage,usd,app_update)
        if template is not None:
            print(f"[CARGO CART BOX] source={template.GetPath()} size={_size_from_bounds(template)}")
    except Exception as exc:
        print(f"[CARGO CART BOX WARNING] actual GLB conversion failed; brown proxy fallback: {exc}")

    cargo_root=UsdGeom.Xform.Define(stage,root_path+"/CargoStack").GetPrim()
    target=tuple(float(v) for v in cargo_cfg.get("box_size_m",[0.45,0.375,0.22]))
    spacing=float(cargo_cfg.get("spacing_m",0.012))
    # 4 x 4 cartons per layer, 2 layers: 32 flat cartons, centered over the cart CG.
    layout=[]
    for layer in range(2):
        nx,ny=4,4
        z=deck_top+target[2]*(0.5+layer)+0.004
        for ix in range(nx):
            for iy in range(ny):
                bx=(ix-(nx-1)/2.0)*(target[0]+spacing)
                by=(iy-(ny-1)/2.0)*(target[1]+spacing)
                layout.append((bx,by,z))
    max_count=int(cargo_cfg.get("count",32)); layout=layout[:max_count]
    root_world=_world_matrix(root)
    for i,(lx,ly,lz) in enumerate(layout,1):
        path=f"{root_path}/CargoStack/Box_{i:02d}"
        if template is not None:
            ok,_=omni.kit.commands.execute("CopyPrim",path_from=str(template.GetPath()),path_to=path,duplicate_layers=True,combine_layers=False,flatten_references=False)
            p=stage.GetPrimAtPath(path)
            if ok and p and p.IsValid():
                wc=root_world.Transform(Gf.Vec3d(lx,ly,lz))
                _normalize_box(p,(float(wc[0]),float(wc[1]),float(wc[2])),target,app_update)
                continue
        _define_cube(stage,path,target,(lx,ly,lz),(0.48,0.29,0.12),False)

    root.SetCustomDataByKey("cooperativeWarehouseCart",True)
    root.SetCustomDataByKey("cartDesign","box body with two carved AMR through-bays; four passive casters")
    meta={"root_path":root_path,"world_pose":(x,y,floor_z,yaw),"dock_y":dock_y,"dock_x":dock_x,"underside":underside,"dock_contact_z":dock_contact_z,"deck_top":deck_top,"length":length,"width":width,"boxes":len(layout),"bay_width":bay_w,"trolley_lidar_path":trolley_lidar_path}
    print(f"[CARGO CART READY] USER FIXED midpoint pose=({x:.3f},{y:.3f},{floor_z:.3f}) yaw={yaw:.1f}")
    print(f"[CARGO CART READY] BOX BODY: two carved AMR bays width={bay_w:.3f}m, AMR side-clearance total={bay_clearance:.3f}m per bay")
    print(f"[CARGO CART READY] existing AMR1+AMR2 moved into bays before physics; no parked originals remain")
    print(f"[CARGO CART READY] 4 casters carry load; 32 cartons laid FLAT; deck={length:.2f}x{width:.2f}m")
    return root,meta


class CooperativeWarehouseCartController:
    def __init__(self,stage:Usd.Stage,cfg:dict[str,Any],controllers:list[Any],cart_root:Usd.Prim,meta:dict[str,Any],timeline:Any,app_update:Callable[[],None]):
        self.stage=stage; self.cfg=cfg; self.controllers=controllers[:2]; self.cart_root=cart_root; self.meta=meta; self.timeline=timeline; self.app_update=app_update
        self.root_path=str(meta["root_path"]); self.cart_rb=UsdPhysics.RigidBodyAPI(cart_root)
        self.joints=[f"{self.root_path}/Joints/AMR1_CartFixed",f"{self.root_path}/Joints/AMR2_CartFixed"]
        self.attached=False; self.pending=False; self.deadline=0.0
        self._manual_speed=float(cfg.get("cooperative_linear_speed_mps",0.42)); self._manual_turn=float(cfg.get("cooperative_angular_speed_rad_s",0.42))
        self._align_standoff=float(cfg.get("debug_align_longitudinal_offset_m",0.0))
        self._last_center_v=0.0; self._last_center_w=0.0
        self._sync_assist=bool(cfg.get("sync_assist_enabled",True))

    @property
    def active(self)->bool: return bool(self.attached or self.pending)

    def _cart_matrix(self): return _world_matrix(self.cart_root)

    def _target_world_pose(self,index:int)->tuple[float,float,float]:
        local=Gf.Vec3d(float(self.meta["dock_x"])+self._align_standoff, float(self.meta["dock_y"])*(1 if index==0 else -1), 0.0)
        m=self._cart_matrix(); wp=m.Transform(local); forward=m.TransformDir(Gf.Vec3d(1,0,0)); yaw=math.degrees(math.atan2(float(forward[1]),float(forward[0])))
        return float(wp[0]),float(wp[1]),yaw

    def _set_amr_pose(self,controller:Any,x:float,y:float,yaw_deg:float)->None:
        # Preserve ground contact, then correct XY so base_link center is exactly on the dock point.
        root=controller.stage.GetPrimAtPath(controller.root)
        mn,_mx=_world_bounds(root); current_root_z=float(_world_matrix(root).ExtractTranslation()[2]); floor_z=float(mn[2])
        _set_pose(root,(x,y,current_root_z-floor_z),yaw_deg)
        self.app_update()
        try:
            center=_center_from_bounds(controller.base_prim)
            rm=_world_matrix(root).ExtractTranslation()
            _set_pose(root,(float(rm[0])+(x-float(center[0])), float(rm[1])+(y-float(center[1])), float(rm[2])),yaw_deg)
        except Exception:
            pass
        controller.halt()

    def initialize_start_coupled(self)->None:
        """Place both AMRs under the cart and finish coupling before user control starts."""
        if not bool(self.cfg.get("start_coupled", False)):
            return
        print("[CARGO CART START] pre-coupled startup: existing AMR1/AMR2 were already moved into cart bays BEFORE physics")
        # Do not reposition articulations after timeline.play(). Build-time placement already removed the old parked poses.
        for c in self.controllers:
            if c.magnet.locked:
                c.request_magnet_release()
            c.halt()
        # Raise only enough to meet the magnetic dock pads; the four cart casters keep carrying load.
        settle=float(self.cfg.get("startup_lift_settle_sec",1.1))
        for c in self.controllers:
            c._set_lift_raised(True,0.0)
        deadline=time.monotonic()+settle
        while time.monotonic()<deadline:
            self.app_update()
        for c in self.controllers:
            c.halt()
        for idx,(c,jp) in enumerate(zip(self.controllers,self.joints),1):
            _create_fixed_joint(self.stage,jp,_lift_body(c),self.cart_root,self.cfg)
            print(f"\a[CARGO CART START] AMR{idx} already CLACKED -> {jp}")
        self.attached=True; self.pending=False
        self.emergency_stop()
        print("[COOPERATIVE CART MODE] READY AT START - W/S forward/reverse, A/D synchronized turn; J releases")

    def align_debug(self)->None:
        if self.attached or self.pending:
            print("[CARGO CART G] ignored while attached/pending")
            return
        if any(c.magnet.locked for c in self.controllers):
            print("[CARGO CART G] refused: release hospital bed magnet first")
            return
        print("[CARGO CART G] development alignment: AMR1/AMR2 -> under-deck dock centers")
        self.timeline.pause()
        for _ in range(2): self.app_update()
        for i,c in enumerate(self.controllers):
            x,y,yaw=self._target_world_pose(i); self._set_amr_pose(c,x,y,yaw); c._set_lift_raised(False,0.0)
        for _ in range(5): self.app_update()
        self.timeline.play()

    def _ready(self)->bool:
        if len(self.controllers)<2: return False
        if any(c.magnet.locked for c in self.controllers):
            print("[CARGO CART K] refused: one AMR is already attached to a hospital bed")
            return False
        dist_limit=float(self.cfg.get("capture_distance_m",0.24)); yaw_limit=float(self.cfg.get("capture_yaw_deg",8.0))
        ok=True
        for i,c in enumerate(self.controllers):
            x,y,yaw_target=self._target_world_pose(i); center=_center_from_bounds(c.base_prim); d=math.hypot(float(center[0])-x,float(center[1])-y)
            f=_world_matrix(c.base_prim).TransformDir(Gf.Vec3d(1,0,0)); yaw=math.degrees(math.atan2(float(f[1]),float(f[0]))); err=(yaw-yaw_target+180)%360-180
            print(f"[CARGO CART DOCK] {c.name}: distance={d:.3f}m yaw_error={err:.1f}deg")
            ok=bool(ok and d<=dist_limit and abs(err)<=yaw_limit)
        return ok

    def request_attach(self)->None:
        if self.attached:
            print("[CARGO CART K] already coupled")
            return
        if self.pending: return
        for c in self.controllers: c.halt()
        if not self._ready():
            print("[CARGO CART K] NOT READY - manually enter under cart or press G, then K")
            return
        settle=float(self.cfg.get("lift_settle_sec",1.4))
        for c in self.controllers:
            c._set_lift_raised(True,settle)
        self.pending=True; self.deadline=time.monotonic()+settle
        print("[CARGO CART K] both lifts UP -> magnetic dock pads contacting; waiting to CLACK")

    def update(self)->None:
        if not self.pending or time.monotonic()<self.deadline: return
        try:
            for idx,(c,jp) in enumerate(zip(self.controllers,self.joints),1):
                body0=_lift_body(c); _create_fixed_joint(self.stage,jp,body0,self.cart_root,self.cfg)
                print(f"\a[CARGO CART MAGNET] AMR{idx} CLACK -> {jp}")
            self.pending=False; self.attached=True
            print("[COOPERATIVE CART MODE] ON - W/S forward/reverse, A/D turn as ONE vehicle; J releases")
        except Exception as exc:
            self.pending=False
            for c in self.controllers: c._set_lift_raised(False,0.0)
            print(f"[CARGO CART K ERROR] coupling failed safely: {exc}")

    def release(self)->None:
        if self.pending: self.pending=False
        for jp in self.joints:
            prim=self.stage.GetPrimAtPath(jp)
            if prim and prim.IsValid(): self.stage.RemovePrim(jp)
        for c in self.controllers:
            c.halt(); c._set_lift_raised(False,0.6)
        self.attached=False; self._last_center_v=0.0; self._last_center_w=0.0
        self.emergency_stop()
        print("[CARGO CART J] FixedJoints released; lifts DOWN; AMRs independent")

    def commands_from_twist(self,v:float,w:float)->list[tuple[float,float,float]]:
        """Map a desired virtual trolley-center twist (V, W) to both AMRs.

        The AMRs are laterally separated by +/-dock_y and share the same yaw rate.
        This is the object-centered virtual-structure / cooperative differential
        kinematic mapping used by both keyboard control and Nav2.
        """
        max_v=float(self.cfg.get("cooperative_max_linear_speed_mps", self._manual_speed))
        max_w=float(self.cfg.get("cooperative_max_angular_speed_rad_s", self._manual_turn))
        v=max(-max_v,min(max_v,float(v)))
        w=max(-max_w,min(max_w,float(w)))
        self._last_center_v=v; self._last_center_w=w
        y=float(self.meta["dock_y"])
        return [(v-w*y,0.0,w),(v+w*y,0.0,w)]

    def commands(self,forward_key:float,yaw_key:float,speed_multiplier:float=1.0)->list[tuple[float,float,float]]:
        """Keyboard wrapper around commands_from_twist()."""
        v=float(forward_key)*self._manual_speed*float(speed_multiplier)
        w=float(yaw_key)*self._manual_turn*float(speed_multiplier)
        return self.commands_from_twist(v,w)

    def apply_sync_assist(self)->None:
        """Give the passive cart the same virtual-center twist as the two AMRs.

        The FixedJoints remain the physical coupling, while this velocity assist removes
        caster drag/joint catch-up so AMR1, AMR2 and the loaded cart visibly move as one body.
        """
        if not self.attached or not self._sync_assist:
            return
        try:
            m=self._cart_matrix()
            world_v=m.TransformDir(Gf.Vec3d(float(self._last_center_v),0.0,0.0))
            self.cart_rb.GetVelocityAttr().Set(Gf.Vec3f(float(world_v[0]),float(world_v[1]),0.0))
            self.cart_rb.GetAngularVelocityAttr().Set(Gf.Vec3f(0.0,0.0,float(math.degrees(self._last_center_w))))
        except Exception as exc:
            print(f"[CARGO CART SYNC WARNING] {exc}")

    def emergency_stop(self)->None:
        for c in self.controllers: c.halt()
        # Cart is passive; zero residual motion only for emergency stop.
        try:
            self.cart_rb.GetVelocityAttr().Set(Gf.Vec3f(0,0,0)); self.cart_rb.GetAngularVelocityAttr().Set(Gf.Vec3f(0,0,0))
        except Exception: pass

    def shutdown(self)->None:
        try: self.release()
        except Exception: pass


def install_cooperative_warehouse_cart(stage:Usd.Stage,project_root:Path,cfg:dict[str,Any],controllers:list[Any],elevator:Any,timeline:Any,app_update:Callable[[],None]):
    if not bool(cfg.get("enabled",False)):
        return None
    if len(controllers)<2:
        print("[CARGO CART WARNING] disabled: requires AMR1 + AMR2")
        return None
    cart,meta=_build_cart(stage,project_root,cfg,controllers,elevator,app_update)
    return CooperativeWarehouseCartController(stage,cfg,controllers,cart,meta,timeline,app_update)
