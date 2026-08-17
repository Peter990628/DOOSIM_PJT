#!/usr/bin/env python3
"""Cooperative warehouse cart for the existing hospital stage.

Design intent
-------------
- Reference concept: a low warehouse display/cart on four passive casters.
- No side pods, no AMR-specific exterior protrusions.
- The normal cargo deck spans the full cart footprint.
- AMR1 and AMR2 enter the open under-deck volume side-by-side, facing the
  same direction. Their ORIGINAL yellow 35 mm lift plates rise into two
  cart-mounted receiver adapters (receiver plate + hangers + upper mounts),
  then two runtime FixedJoints lock the transport assembly. V11.1 keeps the original yellow /lift_plate/Plate as the contact reference. The square upper anchor collars stay above the LiDAR open band, while the two slim portal posts continue THROUGH the receiver assembly to the final Lift-UP contact plane. Their tips overlap the yellow lift by only a few millimetres, so the coupled state visibly reads as mechanically seated without lowering the bulky square anchor blocks into the sensor window.
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


def _preview_material(stage: Usd.Stage, path: str, color: tuple[float,float,float], metallic: float = 0.0, roughness: float = 0.5) -> UsdShade.Material:
    """Small local preview material helper used only for V6 visual continuity."""
    material = UsdShade.Material.Define(stage, Sdf.Path(path))
    shader = UsdShade.Shader.Define(stage, Sdf.Path(path + "/Shader"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*[float(v) for v in color]))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _bind_preview(prim: Usd.Prim, material: UsdShade.Material) -> None:
    try:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
    except Exception:
        rel = prim.CreateRelationship("material:binding", custom=False)
        rel.SetTargets([material.GetPath()])


def _restore_original_yellow_lift(stage: Usd.Stage, controller: Any, cfg: dict[str,Any]) -> None:
    """Restore the established hospital-AMR lift visual without changing its mechanics.

    The original v1.14 AMR lift material used RGB (0.95, 0.56, 0.08).  V6
    explicitly reapplies that same yellow/orange only to the lift plate visual
    on BOTH AMRs.  The prismatic joint, travel, mass and support-pad geometry
    remain untouched.
    """
    visual_cfg = cfg.get("legacy_lift_visual", {})
    color = tuple(float(v) for v in visual_cfg.get("color_rgb", [0.95, 0.56, 0.08]))
    mat = _preview_material(
        stage,
        "/World/HospitalRuntimeMaterials/OriginalAMRLiftYellow",
        color,
        metallic=float(visual_cfg.get("metallic", 0.05)),
        roughness=float(visual_cfg.get("roughness", 0.55)),
    )
    lift_root = stage.GetPrimAtPath(f"{controller.root}/lift_plate")
    if not lift_root or not lift_root.IsValid():
        print(f"[CARGO CART V7 WARNING] {controller.name} lift_plate not found; yellow restore skipped")
        return
    candidates=[]
    exact = stage.GetPrimAtPath(f"{controller.root}/lift_plate/Plate")
    if exact and exact.IsValid():
        candidates.append(exact)
        # Bind descendants too because imported/source USDs can author a stronger
        # material directly on the visual mesh below the Plate Xform.
        for prim in Usd.PrimRange(exact):
            if prim != exact:
                candidates.append(prim)
    else:
        for prim in Usd.PrimRange(lift_root):
            if prim == lift_root:
                continue
            name=prim.GetName().lower()
            if "plate" in name and "support" not in name and "pad" not in name:
                candidates.append(prim)
    if not candidates:
        candidates=[lift_root]
    for prim in candidates:
        _bind_preview(prim, mat)
        try:
            gprim=UsdGeom.Gprim(prim)
            if gprim:
                gprim.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        except Exception:
            pass
    print(f"[CARGO CART V7] {controller.name} original yellow lift visual restored RGB={color}")


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



def _measure_lift_contact_top_world(stage: Usd.Stage, controller: Any) -> tuple[float | None, str]:
    """Return world-Z top of the actual flat lift contact geometry.

    The whole lift_plate subtree may include a tall center guide, so V8 searches
    only broad/thin horizontal plate or support geometry and ignores sensor/mast
    shapes. This leaves the original AMR lift mechanics untouched.
    """
    root = stage.GetPrimAtPath(f"{controller.root}/lift_plate")
    if not root or not root.IsValid():
        return None, "lift_plate missing"
    candidates=[]
    exact = stage.GetPrimAtPath(f"{controller.root}/lift_plate/Plate")
    exact_top=None
    if exact and exact.IsValid() and exact.IsA(UsdGeom.Imageable):
        try:
            mn,mx=_world_bounds(exact)
            sx,sy,sz=float(mx[0]-mn[0]),float(mx[1]-mn[1]),float(mx[2]-mn[2])
            exact_top=float(mx[2])
            # V9: this is the original broad yellow lift plate used by the bed
            # docking design.  If it exists, NEVER let a higher guide/support
            # in the lift subtree replace it as the contact reference.
            if sx >= 0.16 and sy >= 0.07 and sz <= 0.10:
                return exact_top, str(exact.GetPath())
        except Exception:
            pass
    banned=("lidar","camera","sensor","mast","antenna")
    for p in Usd.PrimRange(root):
        if p == root or not p.IsA(UsdGeom.Imageable):
            continue
        path_l=str(p.GetPath()).lower(); name_l=p.GetName().lower()
        if any(tok in path_l or tok in name_l for tok in banned):
            continue
        try:
            mn,mx=_world_bounds(p)
            sx,sy,sz=float(mx[0]-mn[0]),float(mx[1]-mn[1]),float(mx[2]-mn[2])
            top=float(mx[2])
        except Exception:
            continue
        if sx >= 0.16 and sy >= 0.07 and sz <= 0.085 and sx*sy >= 0.018:
            if exact_top is None or top <= exact_top + 0.075:
                candidates.append((top, sx*sy, sz, str(p.GetPath())))
    if not candidates:
        try:
            mn,mx=_world_bounds(root)
            return float(mx[2]), f"fallback whole subtree {root.GetPath()}"
        except Exception:
            return None, "no measurable contact geometry"
    candidates.sort(key=lambda e:(round(e[0],4), e[1]), reverse=True)
    top,_,_,path=candidates[0]
    return float(top), path


def _strip_physics_recursive(prim: Usd.Prim) -> None:
    """Make imported cartons visual-only so they cannot jitter independently."""
    if not prim or not prim.IsValid():
        return
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            try: p.RemoveAPI(UsdPhysics.RigidBodyAPI)
            except Exception: pass
        if p.HasAPI(UsdPhysics.CollisionAPI):
            try: p.RemoveAPI(UsdPhysics.CollisionAPI)
            except Exception: pass
        if p.HasAPI(UsdPhysics.MassAPI):
            try: p.RemoveAPI(UsdPhysics.MassAPI)
            except Exception: pass

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

    # V9 build-time fallback: measure the original yellow lift contact surface. Measuring the
    # whole lift_plate subtree can include the tall center guide and place the
    # receiver several centimetres too high.
    lift_contact_tops=[]
    for c in controllers[:2]:
        z_top, source_path = _measure_lift_contact_top_world(stage, c)
        if z_top is not None:
            lift_contact_tops.append(float(z_top))
            print(f"[CARGO CART V9 CONTACT] {c.name}: flat lift top z={z_top:.4f} from {source_path}")
    measured=max(lift_contact_tops) if lift_contact_tops else float(geom.get("fallback_lift_top_z_m",0.174))
    contact_gap=float(geom.get("lift_contact_gap_m",0.015))
    dock_contact_z=measured+contact_gap
    underside=max(float(geom.get("minimum_deck_underside_m",0.335)), measured+float(geom.get("underdeck_headroom_above_lift_m",0.155)))
    deck_center_z=underside+deck_t*0.5; deck_top=underside+deck_t

    # SENSOR-CLEAR BOX BODY (V5): the cargo deck still reads as one solid box-shaped cart,
    # but the LiDAR plane is surrounded by a continuous open band.  Structure is carried
    # by low rails, high rails and four compact corner posts instead of full-height walls.
    _define_cube(stage,root_path+"/CargoDeck",(length,width,deck_t),(0.0,0.0,deck_center_z),(0.15,0.16,0.17),True)
    sensor_bottom=float(geom.get("sensor_window_bottom_m",0.22))
    sensor_top=min(float(geom.get("sensor_window_top_m",0.41)), underside-0.025)
    if sensor_top <= sensor_bottom + 0.06:
        sensor_top = sensor_bottom + 0.10
    lower_h=max(0.06, sensor_bottom-body_floor_clearance)
    lower_z=body_floor_clearance+lower_h*0.5
    upper_h=max(0.035, underside-sensor_top)
    upper_z=sensor_top+upper_h*0.5

    # Low side rails and low center divider keep the two AMR bays mechanically distinct
    # without blocking the 360-degree lidar plane around z ~= 0.32 m.
    for suffix,yy,thick in (("Left", width*0.5-side_wall_t*0.5, side_wall_t),
                            ("Right",-width*0.5+side_wall_t*0.5, side_wall_t),
                            ("Center",0.0, center_wall_t)):
        _define_cube(stage,f"{root_path}/Body{suffix}LowerRail",(length,thick,lower_h),(0.0,yy,lower_z),(0.23,0.24,0.25),True)
        _define_cube(stage,f"{root_path}/Body{suffix}UpperRail",(length,thick,upper_h),(0.0,yy,upper_z),(0.20,0.21,0.22),True)

    # Four thin corner posts preserve the warehouse-cart box silhouette while leaving
    # front/rear and outer-side LiDAR sight lines overwhelmingly open.
    post=float(geom.get("corner_post_size_m",0.05))
    post_h=max(0.10, underside-body_floor_clearance)
    post_z=body_floor_clearance+post_h*0.5
    for name,sx,sy in (("FL",1,1),("FR",1,-1),("RL",-1,1),("RR",-1,-1)):
        _define_cube(stage,f"{root_path}/SensorClearPosts/{name}",(post,post,post_h),
                     (sx*(length*0.5-post*0.5),sy*(width*0.5-post*0.5),post_z),(0.18,0.19,0.20),True)

    # Only a shallow upper fascia is retained at the front/rear.  The entire sensor band
    # below it remains open, so both AMRs can see forward/backward as well as outward.
    fascia_h=min(float(geom.get("upper_fascia_height_m",0.07)), upper_h)
    fascia_z=underside-fascia_h*0.5
    _define_cube(stage,root_path+"/UpperFrontFascia",(side_wall_t,width,fascia_h),( length*0.5-side_wall_t*0.5,0.0,fascia_z),(0.20,0.21,0.22),True)
    _define_cube(stage,root_path+"/UpperRearFascia", (side_wall_t,width,fascia_h),(-length*0.5+side_wall_t*0.5,0.0,fascia_z),(0.20,0.21,0.22),True)

    # Bay geometry is derived from the actual outer width; AMRs are centered in each carved opening.
    bay_w=(width-2.0*side_wall_t-center_wall_t)*0.5
    dock_y=(center_wall_t+bay_w)*0.5
    amr_width=float(geom.get("amr_width_m",0.74))
    bay_clearance=max(0.0,bay_w-amr_width)

    # V2.17.4 OUTBOARD CASTERS
    # Move all four caster contact points completely outside the tray side edges,
    # keeping the AMR docking tunnel clear.
    out_gap=float(geom.get("caster_outboard_gap_m",0.035))
    hy=width*0.5 + caster_r + out_gap
    hx=length*0.5 - corner_inset
    arm_span=(caster_r+out_gap)+0.055
    arm_y_center=width*0.5 + 0.5*(caster_r+out_gap)

    for name,sx,sy in (("FL",1,1),("FR",1,-1),("RL",-1,1),("RR",-1,-1)):
        cx,cy=sx*hx,sy*hy
        _define_cube(
            stage,
            f"{root_path}/Casters/{name}/OutboardArm",
            (0.16,arm_span,0.045),
            (cx,sy*arm_y_center,caster_r+0.105),
            (0.22,0.24,0.26),
            False,
        )
        _define_cube(
            stage,
            f"{root_path}/Casters/{name}/Fork",
            (0.10,0.085,0.065),
            (cx,cy,caster_r+0.070),
            (0.18,0.18,0.19),
            False,
        )
        _define_wheel_visual(
            stage,
            f"{root_path}/Casters/{name}/Wheel",
            caster_r,
            caster_width,
            (cx,cy,caster_r),
        )
        proxy=_define_sphere(
            stage,
            f"{root_path}/Casters/{name}/ContactProxy",
            caster_r,
            (cx,cy,caster_r),
            (0.03,0.03,0.035),
            True,
        )
        _bind_physics_material(
            stage,
            proxy,
            "/World/HospitalRuntimeMaterials/CoopCartCasterLowFriction",
            float(cfg.get("caster_static_friction",0.035)),
            float(cfg.get("caster_dynamic_friction",0.02)),
        )

    print(
        f"[V2.17.4 OUTBOARD CASTERS] "
        f"tray_half_width={width*0.5:.3f}m "
        f"caster_center_y=+/-{hy:.3f}m "
        f"inner_edge=+/-{hy-caster_r:.3f}m "
        f"outside_gap={out_gap:.3f}m"
    )

    # V7 HOSPITAL-BED-STYLE COUPLER FRAMES ---------------------------------------
    # The user's hospital-bed underside already has an obvious mechanical coupling
    # bracket: a top beam fixed to the bed, two vertical drop legs and a lower
    # receiver that the yellow AMR lift physically meets.  Reproduce that visual
    # language under BOTH cart bays so the coupled state reads mechanically instead
    # of looking like a floating plate.  These members are non-colliding visuals;
    # the runtime FixedJoint remains the actual physics constraint.
    dock_x=float(geom.get("dock_x_m",0.0))
    pad_len=float(geom.get("dock_pad_length_m",0.62))
    pad_w=min(float(geom.get("dock_pad_width_m",0.40)), bay_w*0.78)
    pad_t=float(geom.get("dock_pad_thickness_m",0.018))
    lift_travel=max(float(getattr(c,"geometry_cfg",{}).get("lift_upper_limit_m",0.035)) for c in controllers[:2]) if controllers[:2] else 0.035
    contact_overlap=float(geom.get("lift_receiver_contact_overlap_m",0.0015))
    receiver_trim=float(geom.get("lift_receiver_vertical_trim_m",0.0))
    contact_surface_z=measured+lift_travel-contact_overlap+receiver_trim
    wear_t=float(geom.get("lift_receiver_wear_pad_thickness_m",0.006))
    receiver_bottom=contact_surface_z+wear_t
    pad_z=receiver_bottom+pad_t*0.5
    dock_contact_z=contact_surface_z

    adapter_cfg=geom.get("lift_adapter", {})
    graphite=tuple(float(v) for v in adapter_cfg.get("graphite_rgb",[0.12,0.15,0.16]))
    accent=tuple(float(v) for v in adapter_cfg.get("accent_rgb",[0.06,0.10,0.11]))
    frame_rgb=tuple(float(v) for v in adapter_cfg.get("bed_style_frame_rgb",[0.08,0.28,0.29]))

    # Portal frame dimensions are intentionally close to the hospital-bed coupler
    # visible in the supplied reference image.  Two slim drop posts are separated
    # laterally, joined to the deck by a top beam, and terminate at the receiver.
    # This gives a clear inverted-U silhouette from the cart front/rear view.
    post_y_offset=min(float(adapter_cfg.get("portal_post_y_offset_m",0.155)), max(0.08,pad_w*0.5-0.045))
    post_x=float(adapter_cfg.get("portal_post_x_size_m",0.075))
    post_y=float(adapter_cfg.get("portal_post_y_size_m",0.034))
    top_beam_x=float(adapter_cfg.get("portal_top_beam_x_m",0.13))
    top_beam_y=min(float(adapter_cfg.get("portal_top_beam_y_m",0.46)), bay_w*0.72)
    top_beam_h=float(adapter_cfg.get("portal_top_beam_height_m",0.055))
    receiver_len=min(float(adapter_cfg.get("receiver_length_m",0.58)),pad_len)
    receiver_w=min(float(adapter_cfg.get("receiver_width_m",0.36)),pad_w)
    receiver_h=float(adapter_cfg.get("receiver_height_m",0.028))
    post_contact_overlap=float(adapter_cfg.get("portal_post_contact_overlap_m",0.0025))
    guide_lip_h=float(adapter_cfg.get("guide_lip_height_m",0.040))
    guide_lip_t=float(adapter_cfg.get("guide_lip_thickness_m",0.020))
    mount_plate_x=float(adapter_cfg.get("deck_mount_plate_x_m",0.18))
    mount_plate_y=float(adapter_cfg.get("deck_mount_plate_y_m",0.10))
    mount_plate_h=float(adapter_cfg.get("deck_mount_plate_height_m",0.020))

    receiver_center_z=contact_surface_z + wear_t + receiver_h*0.5
    receiver_top=receiver_center_z+receiver_h*0.5
    top_beam_center_z=underside-top_beam_h*0.5
    # V10.1 hotfix: lower face of the fixed portal top beam.
    # This value is used while placing the fixed upper anchor blocks below.
    top_beam_bottom=underside-top_beam_h
    # V11.1: extend the two SLIM portal posts through the receiver all the way
    # down to the Lift-UP contact plane.  This adds the missing ~receiver thickness
    # to the visible leg length without moving the bulky upper anchor blocks.
    post_bottom=contact_surface_z-post_contact_overlap
    post_top=underside-top_beam_h
    post_h=max(0.06,post_top-post_bottom)
    post_z=post_bottom+post_h*0.5

    for idx,dy in ((1,dock_y),(2,-dock_y)):
        adapter_root=f"{root_path}/BedStyleLiftCoupler_AMR{idx}"
        UsdGeom.Xform.Define(stage,adapter_root)

        # Main horizontal receiver.  At full lift the original yellow plate meets
        # the wear pad with ~1.5 mm render overlap, so no visible air gap remains.
        _define_cube(stage,f"{root_path}/DockPad_AMR{idx}",(receiver_len*0.68,receiver_w*0.62,pad_t),
                     (dock_x,dy,pad_z),graphite,False)
        _define_cube(stage,f"{adapter_root}/ReceiverCrossHead",(receiver_len,receiver_w,receiver_h),(dock_x,dy,receiver_center_z),frame_rgb,False)
        _define_cube(stage,f"{adapter_root}/ReceiverWearPad",(receiver_len*0.84,receiver_w*0.80,wear_t),(dock_x,dy,contact_surface_z+wear_t*0.5),accent,False)

        # Two downward guide lips make the yellow lift look captured inside a socket
        # rather than merely touching a flat ceiling plate.
        for side_name,side_sign in (("Left",1.0),("Right",-1.0)):
            lip_y=dy+side_sign*(receiver_w*0.5-guide_lip_t*0.5)
            _define_cube(stage,f"{adapter_root}/GuideLip{side_name}",(receiver_len*0.88,guide_lip_t,guide_lip_h),
                         (dock_x,lip_y,contact_surface_z-guide_lip_h*0.5),frame_rgb,False)

        # Hospital-bed-like inverted-U/portal bracket: deck top beam + two vertical legs.
        _define_cube(stage,f"{adapter_root}/PortalTopBeam",(top_beam_x,top_beam_y,top_beam_h),
                     (dock_x,dy,top_beam_center_z),frame_rgb,False)
        for side_name,side_sign in (("Left",1.0),("Right",-1.0)):
            post_y_world=dy+side_sign*post_y_offset
            _define_cube(stage,f"{adapter_root}/PortalDropPost{side_name}",(post_x,post_y,post_h),
                         (dock_x,post_y_world,post_z),frame_rgb,False)
            _define_cube(stage,f"{adapter_root}/DeckMountPlate{side_name}",(mount_plate_x,mount_plate_y,mount_plate_h),
                         (dock_x,post_y_world,underside-mount_plate_h*0.5),graphite,False)
            # V10: keep the square collar/anchor ABOVE the LiDAR open band.
            # In V9 this square moved down with the receiver and visibly intruded into
            # the sensor-clear opening.  It is now a fixed upper structural collar;
            # only the slim PortalDropPost stretches down toward the raised lift.
            anchor_h=float(adapter_cfg.get("upper_anchor_block_height_m",0.030))
            anchor_clear=float(adapter_cfg.get("upper_anchor_above_sensor_clearance_m",0.010))
            anchor_bottom=max(sensor_top+anchor_clear, top_beam_bottom)
            anchor_top=min(underside-0.002, anchor_bottom+anchor_h)
            if anchor_top <= anchor_bottom + 0.010:
                anchor_bottom=max(top_beam_bottom, underside-0.032)
                anchor_top=underside-0.002
            anchor_h_eff=max(0.010,anchor_top-anchor_bottom)
            anchor_z=anchor_bottom+anchor_h_eff*0.5
            _define_cube(stage,f"{adapter_root}/UpperAnchorBlock{side_name}",(post_x*1.35,post_y*1.8,anchor_h_eff),
                         (dock_x,post_y_world,anchor_z),graphite,False)

        # A short longitudinal spine ties the coupler into the cargo deck without
        # creating a broad wall across the lidar field.
        spine_len=min(float(adapter_cfg.get("deck_spine_length_m",0.50)),pad_len*0.90)
        spine_w=float(adapter_cfg.get("deck_spine_width_m",0.055))
        spine_h=float(adapter_cfg.get("deck_spine_height_m",0.030))
        _define_cube(stage,f"{adapter_root}/DeckSpine",(spine_len,spine_w,spine_h),
                     (dock_x,dy,underside-spine_h*0.5),graphite,False)

        point=UsdGeom.Xform.Define(stage,f"{root_path}/DockPoint_AMR{idx}").GetPrim(); _set_pose(point,(dock_x,dy,0.0),0.0)

    # V11 scenario mode: the tray waits alone in the corridor.  Existing AMR1/AMR2
    # stay at their original independent starting poses and use their unchanged
    # standalone Nav2 stacks to reach the tray.  The old pre-coupled placement is
    # retained only when start_coupled=true for development/regression tests.
    cart_world=_world_matrix(root)
    forward=cart_world.TransformDir(Gf.Vec3d(1,0,0)); cart_yaw=math.degrees(math.atan2(float(forward[1]),float(forward[0])))
    start_coupled=bool(cfg.get("start_coupled", False))
    for i,c in enumerate(controllers[:2]):
        if start_coupled:
            local=Gf.Vec3d(dock_x, dock_y if i==0 else -dock_y, 0.0)
            wp=cart_world.Transform(local)
            _prephysics_move_controller_to_world_dock(c,float(wp[0]),float(wp[1]),cart_yaw,floor_z,app_update)
        _restore_original_yellow_lift(stage,c,cfg)
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
    # Build imported cartons while the whole stack is invisible. CopyPrim and
    # normalization require several app updates, so revealing only once removes
    # the startup flashing/popping seen in V7.
    try: UsdGeom.Imageable(cargo_root).MakeInvisible()
    except Exception: pass
    target=tuple(float(v) for v in cargo_cfg.get("box_size_m",[0.45,0.375,0.22]))
    spacing=float(cargo_cfg.get("spacing_m",0.002))
    deck_gap=float(cargo_cfg.get("deck_gap_m",0.001))
    vertical_gap=float(cargo_cfg.get("vertical_gap_m",0.001))
    # 4 x 4 cartons per layer, 2 layers: nearly touching seams so the load reads
    # as a properly stacked pallet rather than floating slabs.
    layout=[]
    for layer in range(2):
        nx,ny=4,4
        z=deck_top+deck_gap+target[2]*0.5+layer*(target[2]+vertical_gap)
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
                _strip_physics_recursive(p)
                wc=root_world.Transform(Gf.Vec3d(lx,ly,lz))
                _normalize_box(p,(float(wc[0]),float(wc[1]),float(wc[2])),target,app_update)
                _strip_physics_recursive(p)
                continue
        _define_cube(stage,path,target,(lx,ly,lz),(0.48,0.29,0.12),False)

    # The stack is a visual child of the cart rigid body. Show it only after all
    # 32 cartons have final transforms and imported physics APIs are gone.
    try: UsdGeom.Imageable(cargo_root).MakeVisible()
    except Exception: pass

    root.SetCustomDataByKey("cooperativeWarehouseCart",True)
    root.SetCustomDataByKey("cartDesign","V11.1 sensor-clear body; slim portal posts pass through receiver to Lift-UP contact plane; upper square anchors stay high; V11 ArUco automatic transport preserved")
    per_amr_down_tops=[]
    per_amr_lift_travels=[]
    for c in controllers[:2]:
        z_top,_src=_measure_lift_contact_top_world(stage,c)
        per_amr_down_tops.append(float(z_top) if z_top is not None else float(measured))
        per_amr_lift_travels.append(float(getattr(c,"geometry_cfg",{}).get("lift_upper_limit_m",0.035)))
    meta={"root_path":root_path,"world_pose":(x,y,floor_z,yaw),"dock_y":dock_y,"dock_x":dock_x,"underside":underside,"dock_contact_z":dock_contact_z,"deck_top":deck_top,"length":length,"width":width,"boxes":len(layout),"bay_width":bay_w,"sensor_top":sensor_top,"lift_contact_tops_down_world":per_amr_down_tops,"lift_travels_m":per_amr_lift_travels}
    print(f"[CARGO CART READY] USER FIXED midpoint pose=({x:.3f},{y:.3f},{floor_z:.3f}) yaw={yaw:.1f}")
    print(f"[CARGO CART READY] V11.1 SENSOR-CLEAR BODY + LONG-CONTACT PORTAL POSTS: bays={bay_w:.3f}m, lidar open-band z={sensor_bottom:.3f}..{sensor_top:.3f}m")
    print(f"[CARGO CART READY] V11.1 build-time yellow lift top(down)={measured:.3f}m; receiver contact={contact_surface_z:.3f}m; travel={lift_travel:.3f}m -> flush contact")
    print(f"[CARGO CART READY] AMR side-clearance total={bay_clearance:.3f}m per bay; corner posts only in lidar band")
    if start_coupled:
        print(f"[CARGO CART READY] development mode: existing AMR1+AMR2 moved into bays before physics")
    else:
        print(f"[CARGO CART READY] V11 scenario mode: tray waits alone; AMR1+AMR2 keep original independent starting poses")
    print(f"[CARGO CART READY] 4 casters carry load; cargo VISUAL-LOCKED to cart, revealed once after build; 32 cartons gap_xy={spacing:.4f}m gap_z={vertical_gap:.4f}m")
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
        # V11: once both robots physically dock, measure their actual lateral
        # locations in the cart frame. Cooperative kinematics then use those
        # measured y offsets rather than assuming perfect symmetric placement.
        nominal_y=float(meta.get("dock_y", 0.40))
        self._measured_lateral_offsets=[nominal_y, -nominal_y]
        self._measured_longitudinal_offsets=[float(meta.get("dock_x",0.0)), float(meta.get("dock_x",0.0))]
        # V10: DOWN-state lift contact is captured immediately before every lift-up.
        # The final coupler height is computed from DOWN + the original lift travel,
        # so stale/lagging post-lift BBox measurements cannot size the posts to DOWN.
        self._prelift_contact_tops: dict[int,float] = {}

    @property
    def active(self)->bool: return bool(self.attached or self.pending)

    def _pause_for_usd_physics_edit(self, reason: str) -> bool:
        was_playing=False
        try: was_playing=bool(self.timeline.is_playing())
        except Exception: was_playing=True
        if was_playing:
            print(f"[V2.16.6 PHYSICS EDIT PAUSE] {reason}")
            self.timeline.pause()
            for _ in range(int(self.cfg.get("runtime_safe_pause_frames",3))): self.app_update()
        return was_playing

    def _resume_after_usd_physics_edit(self, was_playing: bool, reason: str) -> None:
        for _ in range(int(self.cfg.get("runtime_safe_pause_frames",3))): self.app_update()
        if was_playing:
            self.timeline.play()
            print(f"[V2.16.6 PHYSICS EDIT RESUME] {reason}")


    def _cart_matrix(self): return _world_matrix(self.cart_root)

    def _target_world_pose(self,index:int)->tuple[float,float,float]:
        local=Gf.Vec3d(float(self.meta["dock_x"])+self._align_standoff, float(self.meta["dock_y"])*(1 if index==0 else -1), 0.0)
        m=self._cart_matrix(); wp=m.Transform(local); forward=m.TransformDir(Gf.Vec3d(1,0,0)); yaw=math.degrees(math.atan2(float(forward[1]),float(forward[0])))
        return float(wp[0]),float(wp[1]),yaw

    def _set_cube_local_z_and_height(self, path: str, center_z: float, height: float | None = None) -> None:
        """Adjust a visual Cube under the cart without touching X/Y or rotation."""
        prim = self.stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            return
        api = UsdGeom.XformCommonAPI(prim)
        try:
            tr, rot, scale, pivot, order = api.GetXformVectors(Usd.TimeCode.Default())
            api.SetTranslate(Gf.Vec3d(float(tr[0]), float(tr[1]), float(center_z)))
            if height is not None:
                api.SetScale(Gf.Vec3f(float(scale[0]), float(scale[1]), float(height)))
        except Exception as exc:
            print(f"[CARGO CART V10 COUPLER WARNING] could not move {path}: {exc}")

    def _capture_prelift_contact_tops(self, reason: str) -> None:
        """Use build-time lift contact heights during runtime attach."""
        self._prelift_contact_tops.clear()
        fallback=list(self.meta.get("lift_contact_tops_down_world",[]))
        if bool(self.cfg.get("runtime_safe_attach_enabled", True)):
            for idx,_ctrl in enumerate(self.controllers,1):
                if idx-1 < len(fallback):
                    z_top=float(fallback[idx-1])
                    self._prelift_contact_tops[idx]=z_top
                    print(f"[V2.16.6 SAFE PRELIFT] {reason} AMR{idx}: down_lift_top={z_top:.4f}m source=build-time meta")
            return
        for idx,ctrl in enumerate(self.controllers,1):
            z_top,source=_measure_lift_contact_top_world(self.stage,ctrl)
            if z_top is None and idx-1 < len(fallback):
                z_top=float(fallback[idx-1]); source="meta build-time /lift_plate/Plate"
            if z_top is not None:
                self._prelift_contact_tops[idx]=float(z_top)
                print(f"[CARGO CART V10 PRELIFT] {reason} AMR{idx}: down_lift_top={float(z_top):.4f}m source={source}")

    def _snap_visual_couplers_to_raised_lifts(self) -> None:
        """Size the visual coupler to the AFTER-LIFT position.

        V9 relied primarily on a bbox sampled after lift motion.  In Isaac/PhysX
        that sample can lag the articulation for a frame and reproduce the DOWN
        height. V10 instead uses the contact height captured before lift-up plus
        the AMR's existing upper-limit travel. The live raised measurement is only
        a sanity check. Square anchor blocks are never moved here; they stay high,
        above the LiDAR open band.
        """
        geom = self.cfg.get("geometry", {})
        adapter = geom.get("lift_adapter", {})
        overlap = float(geom.get("lift_receiver_contact_overlap_m", 0.0015))
        wear_t = float(geom.get("lift_receiver_wear_pad_thickness_m", 0.006))
        receiver_h = float(adapter.get("receiver_height_m", 0.028))
        post_contact_overlap = float(adapter.get("portal_post_contact_overlap_m", 0.0025))
        guide_h = float(adapter.get("guide_lip_height_m", 0.040))
        pad_t = float(geom.get("dock_pad_thickness_m", 0.018))
        top_beam_h = float(adapter.get("portal_top_beam_height_m", 0.055))
        underside = float(self.meta["underside"])
        top_beam_bottom = underside - top_beam_h
        fallback_down=list(self.meta.get("lift_contact_tops_down_world",[]))
        fallback_travel=list(self.meta.get("lift_travels_m",[]))

        cart_origin_z = float(self._cart_matrix().ExtractTranslation()[2])

        for idx, c in enumerate(self.controllers, 1):
            down_top=self._prelift_contact_tops.get(idx)
            if down_top is None and idx-1 < len(fallback_down):
                down_top=float(fallback_down[idx-1])
            travel=float(getattr(c,"geometry_cfg",{}).get("lift_upper_limit_m", fallback_travel[idx-1] if idx-1 < len(fallback_travel) else 0.035))
            if down_top is None:
                print(f"[CARGO CART V10 COUPLER WARNING] AMR{idx}: no pre-lift yellow plate height")
                continue

            expected_raised=float(down_top)+travel
            actual_raised,source=_measure_lift_contact_top_world(self.stage,c)
            # Expected AFTER-LIFT position is authoritative. Actual is diagnostic only.
            # If the live bbox is close, report it; if it is still near DOWN, do not
            # let it drag the receiver/coupler down into the pre-lift geometry.
            live_err=None if actual_raised is None else float(actual_raised)-expected_raised
            target_lift_top_world=expected_raised

            contact_z = target_lift_top_world - cart_origin_z - overlap
            receiver_center = contact_z + wear_t + receiver_h * 0.5
            receiver_top = contact_z + wear_t + receiver_h
            # V11.1: posts pass through ReceiverCrossHead and terminate at the
            # raised yellow lift contact plane, with a tiny visual overlap.
            post_bottom = contact_z - post_contact_overlap
            post_h = max(0.025, top_beam_bottom - post_bottom)
            post_z = post_bottom + post_h * 0.5

            base = f"{self.root_path}/BedStyleLiftCoupler_AMR{idx}"
            self._set_cube_local_z_and_height(f"{base}/ReceiverWearPad", contact_z + wear_t * 0.5)
            self._set_cube_local_z_and_height(f"{base}/ReceiverCrossHead", receiver_center)
            self._set_cube_local_z_and_height(f"{self.root_path}/DockPad_AMR{idx}", contact_z + wear_t + pad_t * 0.5)
            self._set_cube_local_z_and_height(f"{base}/GuideLipLeft", contact_z - guide_h * 0.5)
            self._set_cube_local_z_and_height(f"{base}/GuideLipRight", contact_z - guide_h * 0.5)
            # Only these two slim posts change length. In V11.1 they continue
            # through the receiver to the Lift-UP contact plane. UpperAnchorBlock
            # Left/Right remain high above the LiDAR sensor window.
            self._set_cube_local_z_and_height(f"{base}/PortalDropPostLeft", post_z, post_h)
            self._set_cube_local_z_and_height(f"{base}/PortalDropPostRight", post_z, post_h)

            self.app_update()
            try:
                wear = self.stage.GetPrimAtPath(f"{base}/ReceiverWearPad")
                mn, _mx = _world_bounds(wear)
                expected_gap=float(mn[2])-target_lift_top_world
                actual_text="n/a" if actual_raised is None else f"{float(actual_raised):.4f}m(err={live_err*1000.0:+.1f}mm)"
                print(
                    f"[CARGO CART V10 LIFT-UP FIT] AMR{idx}: down={float(down_top):.4f}m "
                    f"travel={travel*1000.0:.1f}mm expected_raised={expected_raised:.4f}m "
                    f"live_raised={actual_text} receiver_bottom={float(mn[2]):.4f}m "
                    f"expected_visual_gap={expected_gap*1000.0:+.1f}mm "
                    f"post_tip_overlap={post_contact_overlap*1000.0:.1f}mm"
                )
            except Exception as exc:
                print(f"[CARGO CART V10 LIFT-UP FIT] AMR{idx}: sized to expected raised height; verification unavailable: {exc}")

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

    def _update_measured_geometry_after_attach(self, reason: str = "attach") -> None:
        """Measure real AMR docking offsets in the current cart frame.

        A visual/ArUco docking can finish a few centimetres away from the nominal
        bay center.  For a cart-center twist (V,W), each AMR forward speed uses
        its measured lateral offset y_i: v_i = V - W*y_i.
        """
        try:
            inv=self._cart_matrix().GetInverse()
            measured_y=[]; measured_x=[]
            for idx,c in enumerate(self.controllers,1):
                center=_center_from_bounds(c.base_prim)
                local=inv.Transform(center)
                measured_x.append(float(local[0])); measured_y.append(float(local[1]))
                print(
                    f"[CARGO CART V11 GEOMETRY] {reason} AMR{idx}: "
                    f"cart_local_x={float(local[0]):+.4f}m cart_local_y={float(local[1]):+.4f}m"
                )
            if len(measured_y)==2:
                # Keep each robot's signed offset.  Do not force symmetry: this is
                # precisely the docking error the V11 kinematics should absorb.
                self._measured_lateral_offsets=measured_y
                self._measured_longitudinal_offsets=measured_x
                spacing=abs(measured_y[0]-measured_y[1])
                center_bias=0.5*(measured_y[0]+measured_y[1])
                print(
                    f"[CARGO CART V11 GEOMETRY] measured AMR spacing={spacing:.4f}m "
                    f"lateral center bias={center_bias:+.4f}m; cooperative speeds now use measured offsets"
                )
        except Exception as exc:
            print(f"[CARGO CART V11 GEOMETRY WARNING] keeping nominal dock_y: {exc}")

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
        # V10: record the actual DOWN yellow-plate height immediately before lift-up.
        # Receiver/post geometry will then target DOWN + original lift travel.
        self._capture_prelift_contact_tops("startup")
        # Raise only enough to meet the magnetic dock pads; the four cart casters keep carrying load.
        settle=float(self.cfg.get("startup_lift_settle_sec",1.1))
        for c in self.controllers:
            c._set_lift_raised(True,0.0)
        deadline=time.monotonic()+settle
        while time.monotonic()<deadline:
            self.app_update()
        for c in self.controllers:
            c.halt()
        # V10: after lift motion settles, size receiver/posts to DOWN + original lift travel
        # to the measured yellow-plate top. This guarantees near-zero visible gap.
        self._snap_visual_couplers_to_raised_lifts()
        for idx,(c,jp) in enumerate(zip(self.controllers,self.joints),1):
            _create_fixed_joint(self.stage,jp,_lift_body(c),self.cart_root,self.cfg)
            print(f"\a[CARGO CART START] AMR{idx} already CLACKED -> {jp}")
        self.attached=True; self.pending=False
        self._update_measured_geometry_after_attach("startup")
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
        # V10: K re-coupling also captures DOWN height before the lift moves.
        self._capture_prelift_contact_tops("K recouple")
        settle=float(self.cfg.get("lift_settle_sec",1.4))
        for c in self.controllers:
            c._set_lift_raised(True,settle)
        self.pending=True; self.deadline=time.monotonic()+settle
        print("[CARGO CART K] both lifts UP -> magnetic dock pads contacting; waiting to CLACK")

    def update(self)->None:
        if not self.pending or time.monotonic()<self.deadline: return
        was_playing=False
        try:
            was_playing=self._pause_for_usd_physics_edit("dual cart FixedJoint attach")
            if not bool(self.cfg.get("runtime_safe_skip_visual_coupler_refit",True)):
                self._snap_visual_couplers_to_raised_lifts()
            else:
                print("[V2.16.6 SAFE ATTACH] runtime visual coupler refit skipped")
            for idx,(ctrl,jp) in enumerate(zip(self.controllers,self.joints),1):
                body0=_lift_body(ctrl)
                _create_fixed_joint(self.stage,jp,body0,self.cart_root,self.cfg)
                print(f"\a[V2.16.6 CART MAGNET] AMR{idx} CLACK -> {jp}")
            self.pending=False; self.attached=True
            self._update_measured_geometry_after_attach("V2.16.6 runtime safe attach")
            print("[V2.16.6 COOPERATIVE CART MODE] ON - dual FixedJoints authored while physics paused")
        except Exception as exc:
            self.pending=False
            for ctrl in self.controllers: ctrl._set_lift_raised(False,0.0)
            print(f"[V2.16.6 CART ATTACH ERROR] coupling failed safely: {exc}")
        finally:
            try: self._resume_after_usd_physics_edit(was_playing,"dual cart FixedJoint attach")
            except Exception as exc: print(f"[V2.16.6 PHYSICS RESUME WARNING] {exc}")

    def release(self)->None:
        if self.pending: self.pending=False
        was_playing=False
        try:
            was_playing=self._pause_for_usd_physics_edit("dual cart FixedJoint release")
            for jp in self.joints:
                prim=self.stage.GetPrimAtPath(jp)
                if prim and prim.IsValid(): self.stage.RemovePrim(jp)
        finally:
            self._resume_after_usd_physics_edit(was_playing,"dual cart FixedJoint release")
        for ctrl in self.controllers:
            ctrl.halt(); ctrl._set_lift_raised(False,0.6)
        self.attached=False; self._last_center_v=0.0; self._last_center_w=0.0
        self.emergency_stop()
        print("[V2.16.6 CART RELEASE] FixedJoints removed safely; lifts DOWN")

    def commands(self,forward_key:float,yaw_key:float,speed_multiplier:float=1.0)->list[tuple[float,float,float]]:
        """Return rigid-transport commands for the two laterally separated AMRs.

        For a virtual cart-center twist (V, W), the forward speed at lateral
        offset y_i is V - W*y_i. Both robots share W. This keeps rotation about
        the cart center instead of commanding two unrelated turns.
        """
        v=float(forward_key)*self._manual_speed*float(speed_multiplier)
        w=float(yaw_key)*self._manual_turn*float(speed_multiplier)
        self._last_center_v=v; self._last_center_w=w
        ys=list(self._measured_lateral_offsets)
        return [(v-w*float(ys[0]),0.0,w),(v-w*float(ys[1]),0.0,w)]

    def commands_from_twist(self,v_mps:float,w_rad_s:float)->list[tuple[float,float,float]]:
        """Convert one virtual-cart Nav2 twist into commands for both AMRs."""
        vmax=float(self.cfg.get("nav2_max_linear_speed_mps",0.45))
        wmax=float(self.cfg.get("nav2_max_angular_speed_rad_s",0.35))
        v=max(-vmax,min(vmax,float(v_mps)))
        w=max(-wmax,min(wmax,float(w_rad_s)))
        self._last_center_v=v; self._last_center_w=w
        ys=list(self._measured_lateral_offsets)
        return [(v-w*float(ys[0]),0.0,w),(v-w*float(ys[1]),0.0,w)]

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
