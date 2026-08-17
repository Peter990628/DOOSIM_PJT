#!/usr/bin/env python3
"""Install a three-post / five-marker ArUco docking gate on the cooperative tray.

V2.4 gate geometry (viewed by AMRs approaching from tray local -X):

  LEFT OUTER POST        CENTER POST        RIGHT OUTER POST
      ID 40                  ID 44                ID 42
      ID 41                                       ID 43

The side-post marker center is deliberately placed at y=+/-2*dock_y while the
center marker is at y=0.  Therefore the midpoint of LEFT+CENTER is exactly the
AMR1 bay center (+dock_y), and the midpoint of CENTER+RIGHT is exactly the AMR2
bay center (-dock_y).  The lower side markers are redundant fallbacks if an upper
marker is temporarily occluded.

All carrier posts / marker cards are VISUAL ONLY: no collider or rigid-body API is
added, so hospital_total_08091221 cart physics, LiDAR collision geometry and lift
mechanics remain unchanged.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade


def _material(stage: Usd.Stage, path: str, texture: Path) -> UsdShade.Material:
    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    shader = UsdShade.Shader.Define(stage, Sdf.Path(path + "/PreviewSurface"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    tex = UsdShade.Shader.Define(stage, Sdf.Path(path + "/Texture"))
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(texture.resolve())))
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    reader = UsdShade.Shader.Define(stage, Sdf.Path(path + "/PrimvarReader"))
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def _quad(stage: Usd.Stage, path: str, size: float) -> UsdGeom.Mesh:
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    h = 0.5 * size
    mesh.CreatePointsAttr([
        Gf.Vec3f(-h, -h, 0.0), Gf.Vec3f(h, -h, 0.0),
        Gf.Vec3f(h, h, 0.0), Gf.Vec3f(-h, h, 0.0),
    ])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateExtentAttr([Gf.Vec3f(-h, -h, 0.0), Gf.Vec3f(h, h, 0.0)])
    mesh.CreateDoubleSidedAttr(True)
    pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
    )
    pv.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    prim = mesh.GetPrim()
    prim.SetCustomDataByKey("hospitalNonPhysicalVisual", True)
    prim.SetCustomDataByKey("trayArucoCard", True)
    return mesh


def _visual_cube(
    stage: Usd.Stage, path: str, size_xyz: tuple[float, float, float],
    pos_xyz: tuple[float, float, float], color: tuple[float, float, float],
) -> UsdGeom.Cube:
    cube = UsdGeom.Cube.Define(stage, Sdf.Path(path))
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*pos_xyz))
    xform.AddScaleOp().Set(Gf.Vec3f(*size_xyz))
    prim = cube.GetPrim()
    prim.SetCustomDataByKey("hospitalNonPhysicalVisual", True)
    prim.SetCustomDataByKey("trayArucoCarrier", True)
    return cube


def _basis(h: Gf.Vec3d, v: Gf.Vec3d, n: Gf.Vec3d, p: Gf.Vec3d) -> Gf.Matrix4d:
    return Gf.Matrix4d(
        h[0], h[1], h[2], 0.0,
        v[0], v[1], v[2], 0.0,
        n[0], n[1], n[2], 0.0,
        p[0], p[1], p[2], 1.0,
    )


def install_tray_aruco_markers(
    stage: Usd.Stage,
    project_root: Path,
    cfg: dict[str, Any],
    cart_controller: Any,
) -> list[str]:
    settings = cfg.get("tray_aruco_docking", {})
    if not bool(settings.get("enabled", False)) or cart_controller is None:
        return []

    root_path = str(cart_controller.meta["root_path"])
    cart_root = stage.GetPrimAtPath(root_path)
    if not cart_root or not cart_root.IsValid():
        print("[TRAY ARUCO GATE WARNING] cart root missing")
        return []

    marker_root_path = root_path + "/TrayArucoThreePostGate"
    old = stage.GetPrimAtPath(marker_root_path)
    if old and old.IsValid():
        stage.RemovePrim(marker_root_path)
    marker_root = UsdGeom.Xform.Define(stage, Sdf.Path(marker_root_path)).GetPrim()
    marker_root.SetCustomDataByKey("trayArucoThreePostGate", True)
    marker_root.SetCustomDataByKey("arucoDictionary", str(settings.get("dictionary", "DICT_4X4_50")))

    marker_size = float(settings.get("marker_size_m", 0.12))
    front_offset = float(settings.get("rack_front_offset_m", 0.012))
    underside = float(cart_controller.meta["underside"])
    length = float(cart_controller.meta["length"])
    width = float(cart_controller.meta["width"])
    dock_y = float(cart_controller.meta["dock_y"])

    # Geometric key: midpoint(outer post, center post) == each bay center.
    ideal_outer_y = 2.0 * abs(dock_y)
    outer_y = float(settings.get("outer_post_y_m", ideal_outer_y))
    if abs(outer_y - ideal_outer_y) > 1e-6:
        print(
            f"[TRAY ARUCO GATE WARNING] configured outer_y={outer_y:.3f} differs from "
            f"exact 2*dock_y={ideal_outer_y:.3f}; forcing exact bay-center geometry"
        )
        outer_y = ideal_outer_y

    # V2.4 FRONT-FACE FIX:
    # PRE_DOCK goals are generated at tray local -X (pre_x = -0.5*length - standoff),
    # so the AMRs approach the cart from the local -X side while looking toward +X.
    # The previous V2/V2.3 gate was authored on the +X face, which put the markers
    # behind the tray structure from the actual approach direction.  Put the complete
    # three-post gate on the TRUE APPROACH FACE at local -X.  The marker outward
    # normal stays -X so the printed face looks directly at the approaching cameras.
    marker_face_x = -0.5 * length - front_offset
    carrier_t = float(settings.get("carrier_thickness_m", 0.012))
    carrier_w = float(settings.get("carrier_width_m", 0.055))
    carrier_bottom = float(settings.get("carrier_bottom_z_m", 0.115))
    carrier_top = min(
        underside - float(settings.get("underdeck_clearance_m", 0.012)),
        float(settings.get("carrier_top_z_m", underside - 0.012)),
    )
    carrier_h = max(0.12, carrier_top - carrier_bottom)
    carrier_z = carrier_bottom + 0.5 * carrier_h
    carrier_x = marker_face_x + 0.5 * carrier_t

    # Marker centers: upper/lower redundancy on each side post + one shared center.
    half = 0.5 * marker_size
    z_margin = float(settings.get("marker_vertical_margin_m", 0.008))
    max_center_z = carrier_top - half - z_margin
    min_center_z = carrier_bottom + half + z_margin
    upper_z = min(float(settings.get("outer_upper_center_z_m", 0.360)), max_center_z)
    lower_z = max(float(settings.get("outer_lower_center_z_m", 0.200)), min_center_z)
    if upper_z - lower_z < marker_size + 0.010:
        mid = 0.5 * (upper_z + lower_z)
        upper_z = min(max_center_z, mid + 0.5 * (marker_size + 0.012))
        lower_z = max(min_center_z, mid - 0.5 * (marker_size + 0.012))
    center_z = float(settings.get("center_marker_center_z_m", 0.280))
    center_z = max(min_center_z, min(max_center_z, center_z))

    # Three visible carrier pillars. Visual-only, so no new physics/collision is introduced.
    carrier_color = tuple(float(v) for v in settings.get("carrier_rgb", [0.72, 0.74, 0.76]))
    for name, yy in (("LeftOuterPost", +outer_y), ("CenterPost", 0.0), ("RightOuterPost", -outer_y)):
        _visual_cube(
            stage, f"{marker_root_path}/{name}",
            (carrier_t, carrier_w, carrier_h),
            (carrier_x, yy, carrier_z), carrier_color,
        )

    # White back plates make the marker boundary stable under dark hospital lighting.
    backer_t = float(settings.get("backer_thickness_m", 0.006))
    backer_border = float(settings.get("backer_border_m", 0.016))
    backer_size = marker_size + 2.0 * backer_border
    backer_x = marker_face_x + 0.5 * backer_t

    textures = settings.get("textures", {})
    marker_layout = [
        (40, "LeftOuterUpper", +outer_y, upper_z, "AMR1_OUTER_PRIMARY"),
        (41, "LeftOuterLower", +outer_y, lower_z, "AMR1_OUTER_FALLBACK"),
        (44, "CenterShared", 0.0, center_z, "SHARED_CENTER"),
        (42, "RightOuterUpper", -outer_y, upper_z, "AMR2_OUTER_PRIMARY"),
        (43, "RightOuterLower", -outer_y, lower_z, "AMR2_OUTER_FALLBACK"),
    ]

    # Local marker axes: +X => cart -Y (image horizontal), +Y => cart +Z,
    # +Z surface normal => cart -X (faces approaching AMRs).
    hdir = Gf.Vec3d(0.0, -1.0, 0.0)
    vdir = Gf.Vec3d(0.0, 0.0, 1.0)
    ndir = Gf.Vec3d(-1.0, 0.0, 0.0)
    created: list[str] = []

    for marker_id, name, yy, zz, role in marker_layout:
        texture_rel = str(
            textures.get(str(marker_id), f"tray_overlay/markers/aruco_4x4_50_id_{marker_id}.png")
        )
        texture = project_root / texture_rel
        if not texture.exists():
            print(f"[TRAY ARUCO GATE WARNING] missing texture ID {marker_id}: {texture}")
            continue

        # Back plate sits immediately behind the marker, also render-only.
        _visual_cube(
            stage, f"{marker_root_path}/{name}_Backer",
            (backer_t, backer_size, backer_size),
            (backer_x, yy, zz), (0.96, 0.96, 0.96),
        )

        xform_path = f"{marker_root_path}/{name}_ID_{marker_id}"
        xf = UsdGeom.Xform.Define(stage, Sdf.Path(xform_path))
        local = _basis(hdir, vdir, ndir, Gf.Vec3d(marker_face_x - 0.0015, yy, zz))
        xformable = UsdGeom.Xformable(xf.GetPrim())
        xformable.ClearXformOpOrder()
        xformable.AddTransformOp(UsdGeom.XformOp.PrecisionDouble, "trayArucoPose").Set(local)
        mesh = _quad(stage, xform_path + "/Card", marker_size)
        mat = _material(stage, xform_path + "/Material", texture)
        UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(mat)
        prim = mesh.GetPrim()
        prim.SetCustomDataByKey("arucoDictionary", str(settings.get("dictionary", "DICT_4X4_50")))
        prim.SetCustomDataByKey("arucoId", marker_id)
        prim.SetCustomDataByKey("trayDockingRole", role)
        created.append(str(mesh.GetPath()))

    print(
        "[TRAY ARUCO GATE READY V2.4 FRONT] "
        f"ids=40/41 LEFT, 44 CENTER, 42/43 RIGHT; cards={len(created)} "
        f"outer_y=+/-{outer_y:.3f}m dock_y=+/-{dock_y:.3f}m"
    )
    print(
        "[TRAY ARUCO GATE GEOMETRY] "
        f"midpoint(left_outer,center)=+{0.5*outer_y:.3f}m, "
        f"midpoint(center,right_outer)=-{0.5*outer_y:.3f}m -> exact bay centers"
    )
    print(
        "[TRAY ARUCO GATE ROLE] AMR1 uses OUTER 40/41 + CENTER 44; "
        "AMR2 uses CENTER 44 + OUTER 42/43. Lower markers are occlusion fallbacks."
    )
    print(
        "[TRAY ARUCO GATE PHYSICS] front-face carrier posts/backers/cards are visual-only; "
        "no baseline cart collider/lift/receiver physics changed"
    )
    print(f"[TRAY ARUCO FRONT FACE] local_x={marker_face_x:.3f}m approach=local_-X marker_normal=-X")
    return created
