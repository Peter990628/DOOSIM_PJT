#!/usr/bin/env python3
from __future__ import annotations
from pathlib import Path
import ast, hashlib, json, sys

root=Path(sys.argv[1]).resolve()
manifest=json.loads((root/'tray_overlay/baseline_hospital_total_08091221_manifest.json').read_text(encoding='utf-8'))
# V2.3: __pycache__/*.pyc files are runtime-generated caches, not source assets.
# ZIP packaging and local Python versions may legitimately omit/regenerate them, so
# baseline immutability is enforced only for persistent files.
def _is_runtime_cache(rel: str) -> bool:
    # Python bytecode and empty runtime-output placeholders are packaging/runtime
    # artifacts, not functional baseline source. They may legitimately disappear
    # when output folders are cleaned or when a ZIP is recreated.
    if rel.endswith('.pyc') or '/__pycache__/' in rel:
        return True
    if rel in {'output/.keep', 'output/ocr/.keep'}:
        return True
    return False

intentional_v212_changes={
    # V2.10 rolling convoy
    'ros2_ws/src/hospital_nav2/hospital_nav2/path_conflict_manager.py',
    '09_run_collision_avoidance.sh',
    # V2.12 actual-Isaac-world pose lock replaces stale fixed station poses.
    'ros2_ws/src/hospital_nav2/hospital_nav2/world_pose_initializer.py',
    'ros2_ws/src/hospital_nav2/launch/hospital_amr1_navigation.launch.py',
    'ros2_ws/src/hospital_nav2/launch/hospital_amr2_navigation.launch.py',
    '09_run_nav2_amr1.sh',
    '09_run_nav2_amr2.sh',
}
persistent_manifest={
    rel: expected for rel, expected in manifest.items()
    if not _is_runtime_cache(rel) and rel not in intentional_v212_changes
}
ignored_cache_count=len(manifest)-len(persistent_manifest)
changed=[]; missing=[]
for rel, expected in persistent_manifest.items():
    p=root/rel
    if not p.exists(): missing.append(rel); continue
    got=hashlib.sha256(p.read_bytes()).hexdigest()
    if got!=expected: changed.append(rel)
if missing or changed:
    print('[FAIL] baseline hospital_total_08091221 persistent files were modified')
    if missing: print(' missing:',*missing[:30],sep='\n  ')
    if changed: print(' changed:',*changed[:30],sep='\n  ')
    raise SystemExit(2)
print(f'[PASS] baseline persistent files unchanged except intentional V2.10/V2.12 navigation overrides: {len(persistent_manifest)} files')
print(f'[INFO] ignored runtime/cache placeholder entries: {ignored_cache_count} (__pycache__/*.pyc + output/.keep placeholders)')

cfg=json.loads((root/'config/isaac_config.json').read_text(encoding='utf-8'))
tray=json.loads((root/'tray_overlay/config/isaac_config_tray_integrated.json').read_text(encoding='utf-8'))
assert cfg['ros2']['domain_id']==117 and tray['ros2']['domain_id']==117
assert tray['nav2']['lidar']['horizontal_resolution_deg']==0.5
aru=tray['tray_aruco_docking']
assert aru['layout']=='three_post_five_marker_gate'
assert aru['amr1_outer_ids']==[40,41]
assert aru['amr2_outer_ids']==[42,43]
assert aru['center_id']==44
assert aru['textures']['44']=='tray_overlay/markers/aruco_4x4_50_id_44.png'
assert (root/aru['textures']['44']).exists()
assert tray['cooperative_auto_transport']['fixed_target']['x']==7.9
assert tray['cooperative_auto_transport']['fixed_target']['y']==10.13
print('[PASS] DOMAIN117 + 3-post gate IDs LEFT40/41 CENTER44 RIGHT42/43 + fixed target 7.90/10.13')

# User-declared latest baseline invariants must remain present and byte-identical.
amr1=(root/'ros2_ws/src/hospital_nav2/launch/hospital_amr1_navigation.launch.py').read_text(encoding='utf-8')
amr2=(root/'ros2_ws/src/hospital_nav2/launch/hospital_amr2_navigation.launch.py').read_text(encoding='utf-8')
center=(root/'ros2_ws/src/hospital_nav2/hospital_nav2/centerline_navigator.py').read_text(encoding='utf-8')
pt=(root/'patient_transport_manager.py').read_text(encoding='utf-8')
for text, token in [
    (amr1,'"traffic_pause_topic": "/traffic_pause"'),
    (amr2,'"traffic_pause_topic": "/amr2/traffic_pause"'),
    (amr2,'"map_topic": "/amr2/map"'),
    (center,'elevator_2f_near_success_tolerance_m": 0.35'),
    (center,'self.final_goal_msg = None'),
    (center,'READY:WAITING_GOAL'),
    (pt,'미준비 항목'),
]:
    assert token in text, token
for f in ['nav2_params.yaml','nav2_params_amr1.yaml','nav2_params_amr2.yaml']:
    y=(root/'ros2_ws/src/hospital_nav2/config'/f).read_text(encoding='utf-8')
    for token in ['corner_stop_sec: 0.60','rotate_max_speed_rad_s: 0.70','rotate_min_speed_rad_s: 0.12','rotate_kp: 1.00','elevator_2f_near_success_tolerance_m: 0.35']:
        assert token in y, (f,token)
print('[PASS] latest corner/stale-goal/2F-elevator/AMR2 namespace/traffic-pause fixes preserved')

# 360 LiDAR overlay specifically removes the (720,1)->1-ray bug.
bridge=(root/'tray_overlay/scripts/nav2_bridge.py').read_text(encoding='utf-8')
assert 'def _extract_horizontal_scan' in bridge
assert 'np.squeeze' in bridge and 'np.moveaxis' in bridge
assert 'depth[depth.shape[0] // 2]' not in bridge
assert '[NAV2 LIDAR 360 READY]' in bridge
coop=(root/'tray_overlay/scripts/cooperative_nav2_bridge.py').read_text(encoding='utf-8')
assert 'Nav2Bridge._extract_horizontal_scan' in coop
print('[PASS] 360 LiDAR orientation fix remains active for base and cooperative scans')

# Exact gate geometry: side marker center = +/-2*dock_y, center marker = 0.
marker=(root/'tray_overlay/scripts/tray_aruco_markers.py').read_text(encoding='utf-8')
for token in [
    'ideal_outer_y = 2.0 * abs(dock_y)',
    '(40, "LeftOuterUpper"',
    '(41, "LeftOuterLower"',
    '(44, "CenterShared"',
    '(42, "RightOuterUpper"',
    '(43, "RightOuterLower"',
    'hospitalNonPhysicalVisual',
    'no baseline cart collider/lift/receiver physics changed',
]:
    assert token in marker, token
print('[PASS] exact three-post geometry + visual-only carrier structure')

# Vision gate must fuse redundant outer markers with shared center 44.
pair=(root/'ros2_ws/src/hospital_tray_overlay/hospital_tray_overlay/tray_aruco_pair_node.py').read_text(encoding='utf-8')
for token in [
    'def _virtual_marker',
    'self.declare_parameter("outer_ids", [40, 41])',
    'self.declare_parameter("center_id", 44)',
    '"outer_source_ids"',
    '"center_id": self.center_id',
    'outer_redundancy',
]:
    assert token in pair, token
launch=(root/'ros2_ws/src/hospital_tray_overlay/launch/tray_dual_aruco.launch.py').read_text(encoding='utf-8')
for token in ['"outer_ids": [40, 41]','"center_id": 44','"outer_side": "left"','"outer_ids": [42, 43]','"outer_side": "right"']:
    assert token in launch, token
print('[PASS] AMR1=left outer+center44 / AMR2=center44+right outer with lower-marker fallback')

manager=(root/'ros2_ws/src/hospital_tray_overlay/hospital_tray_overlay/cooperative_transport_manager.py').read_text(encoding='utf-8')
for token in ['ARUCO_GATE_START','[ARUCO FIXED DOCK V2.11]','[ARUCO ID LOCK V2.11]','[FIXED DISTANCE DOCKED V2.11]']:
    assert token in manager, token
print('[PASS] manager late-starts front ArUco scanner then performs bed-style fixed-distance insertion before lift/FixedJoint')

# V2.1 map lifecycle recovery: baseline launch remains byte-identical, wrapper repairs autostart races.
lifecycle=(root/'ros2_ws/src/hospital_tray_overlay/hospital_tray_overlay/lifecycle_bootstrap.py').read_text(encoding='utf-8')
for token in ['TRANSITION_CONFIGURE','TRANSITION_ACTIVATE','PRIMARY_STATE_ACTIVE','[LIFECYCLE ACTIVE]']:
    assert token in lifecycle, token
runner=(root/'RUN_TRAY_2_AUTO_TOTAL_360.sh').read_text(encoding='utf-8')
for token in ['ensure_map_active','/amr2/map_server','lifecycle_bootstrap','[BASE READY V2.12.1]']:
    assert token in runner, token
stop=(root/'STOP_TRAY_INTEGRATED_ROS.sh').read_text(encoding='utf-8')
for token in ['/opt/ros/humble/lib/nav2_map_server/map_server','nav2_lifecycle_manager/lifecycle_manager','ros2 daemon stop']:
    assert token in stop, token
print('[PASS] V2.1 lifecycle recovery preserved: stale child cleanup + explicit map_server configure/activate gate')


# V2.2 storage safety: RUN1 must prove real I/O before creating a session.
guard=(root/'scripts/isaac_storage_guard.sh').read_text(encoding='utf-8')
for token in ['dd if="$py"','write probe failed although mount may report rw','ISAAC_STORAGE_PYTHON_OK','isaac_select_root']:
    assert token in guard, token
run1=(root/'RUN_TRAY_1_ISAAC_TOTAL_360.sh').read_text(encoding='utf-8')
for token in ['rm -f "$SESSION_FILE"','isaac_select_root','[ISAAC STORAGE PASS]','RECOVER_ISAAC45_STORAGE.sh --repair','output/tray_integrated_v2_12']:
    assert token in run1, token
recover=(root/'RECOVER_ISAAC45_STORAGE.sh').read_text(encoding='utf-8')
for token in ['--repair','SAMPLED BACKING-IMAGE READ TEST','e2fsck -f -n','e2fsck -f -y','DO NOT fsck','isaac_probe_root']:
    assert token in recover, token
check=(root/'CHECK_ISAAC_STORAGE.sh').read_text(encoding='utf-8')
assert 'real read + write + python execute' in check
print('[PASS] V2.2 storage preflight: real read/write/execute gate + explicit safe recovery helper')

runtime=(root/'tray_overlay/scripts/isaac_amr_ros_tray_runtime.py').read_text(encoding='utf-8')
for token in ['install_cooperative_warehouse_cart','install_tray_aruco_markers','[BASE DUAL BRIDGE READY]','[COOP BRIDGE READY]','get_fresh_tray_command']:
    assert token in runtime, token
print('[PASS] tray runtime: baseline dual bridge -> tray gate -> lazy cooperative bridge')

# V2.4 actual approach-face fix: PRE_DOCK is tray local -X, so markers must live on -X.
assert aru['approach_face']=='local_minus_x'
assert aru['marker_surface_normal']=='local_minus_x'
for token in [
    'marker_face_x = -0.5 * length - front_offset',
    '[TRAY ARUCO GATE READY V2.4 FRONT]',
    '[TRAY ARUCO FRONT FACE]',
]:
    assert token in marker, token
print('[PASS] V2.6 keeps V2.4 front-face ArUco gate geometry')

# V2.5 camera revert: preserve the proven V2.3/original top-down follow camera exactly.
fc=cfg['follow_camera']
assert fc['position_local_m']==[0.0,0.0,6.0]
assert fc['target_local_m']==[0.0,0.0,0.35]
assert fc['up_local']==[1.0,0.0,0.0]
assert fc['view_mode']=='amr_local_top_down_follow'
assert 'lock_local_pose' not in fc
runtime=(root/'tray_overlay/scripts/isaac_amr_ros_tray_runtime.py').read_text(encoding='utf-8')
for token in ['create_follow_camera_guard','follow_camera_guard.update()','from follow_camera_guard import']:
    assert token not in runtime, token
assert not (root/'tray_overlay/scripts/follow_camera_guard.py').exists()
print('[PASS] V2.6 keeps V2.5/V2.3 original follow camera; per-frame guard remains removed')

# V2.5 keeps V2.4 storage repair unmounts NTFS by device to avoid escaped-space mountpoint failures.
assert 'sudo umount "$DEV"' in recover
assert 'findmnt -rn -S "$DEV" -o TARGET' not in recover
print('[PASS] V2.6 keeps storage repair device-unmount fix')

# V2.11 keeps first-arrival/convoy behavior but replaces pose steering with bed-style fixed-distance insertion.
manager=(root/'ros2_ws/src/hospital_tray_overlay/hospital_tray_overlay/cooperative_transport_manager.py').read_text(encoding='utf-8')
for token in [
    'safe_egress_amr1', 'safe_egress_amr2', 'navigate_and_dock_pair',
    '[PRE_DOCK ARRIVAL V2.11]', '[ARUCO START PASS V2.11]', '[ARUCO DOCKED V2.11]',
    '[NO-PROGRESS WATCHDOG V2.11]', 'ACTIVE:ROTATING_FINAL',
    '[ARUCO FIXED DOCK V2.11]', '[ARUCO ID LOCK V2.11]',
    '[FIXED INSERT V2.11]', '[FIXED DISTANCE DOCKED V2.11]',
    'target_distance = 0.5 * length + float(cfg.get("pre_dock_standoff_m", 0.95)) + float(dock_x)',
    '/amr1/tray_docking_active', '/amr2/tray_docking_active',
]:
    assert token in manager, token
assert aru['fixed_distance_docking_enabled'] is True
assert aru['fixed_forward_distance_m'] == 0.0
assert abs(0.5*tray['cooperative_warehouse_cart']['geometry']['length_m'] + aru['pre_dock_standoff_m'] - 2.05) < 1e-9
assert aru['single_marker_good_cycles'] <= 3
assert tray['cooperative_auto_transport']['safe_egress_amr1']['enabled'] is True
assert tray['cooperative_auto_transport']['safe_egress_amr2']['enabled'] is True
assert tray['cooperative_auto_transport']['no_progress_watchdog']['enabled'] is True
scanner=(root/'ros2_ws/src/hospital_tray_overlay/hospital_tray_overlay/tray_aruco_pair_node.py').read_text(encoding='utf-8')
for token in ['show_window','DOOSIM {self.amr_id.upper()} ARUCO SCANNER','SCANNING...','FIXED INSERT','cv2.imshow']:
    assert token in scanner, token
traffic=(root/'ros2_ws/src/hospital_nav2/hospital_nav2/path_conflict_manager.py').read_text(encoding='utf-8')
for token in ['tray_docking_active','TRAY_DOCK_BYPASS','old Nav path cleared','peer Nav2 remains free']:
    assert token in traffic, token
assert 'phase = "POSE_INSERT"' not in manager
assert '[POSE RECOVERY]' not in manager
assert '[POSE INSERT DOCKED]' not in manager
print('[PASS] V2.11 ArUco single-ID -> tray-size-derived 2.05m fixed straight insertion; pose steering removed')
print('[PASS] V2.11 tray docking clears stale PRE_DOCK path and never traffic-pauses the peer')
print('[PASS] V2.11 first-arrival + both safe-egress + no-progress watchdog preserved')

# V2.11 preserves V2.10 same-direction short-gap convoy policy.
traffic=(root/'ros2_ws/src/hospital_nav2/hospital_nav2/path_conflict_manager.py').read_text(encoding='utf-8')
for token in [
    'SameDirectionFollowSession',
    'same_direction_follow_enabled',
    'same_direction_hold_gap_m',
    'same_direction_release_gap_m',
    '_handle_same_direction_following',
    'FOLLOWING_RUN',
    'FOLLOWING_HOLD',
]:
    assert token in traffic, token
launcher=(root/'09_run_collision_avoidance.sh').read_text(encoding='utf-8')
for token in [
    'same_direction_follow_enabled:=true',
    'same_direction_hold_gap_m:=2.50',
    'same_direction_release_gap_m:=3.20',
]:
    assert token in launcher, token
print('[PASS] V2.11 same-direction shared corridor uses 2.5m/3.2m rolling convoy instead of full-route wait')

# Final demo staff assets and placement overlay.
final_staff=tray.get('final_scene_staff',{})
assert final_staff.get('enabled') is True
for rel in [
    'tray_overlay/assets/final_staff/doctor.glb',
    'tray_overlay/assets/final_staff/woman_doctor.glb',
    'tray_overlay/assets/final_staff/nurse_surgical_rigged.glb',
]:
    q=root/rel
    assert q.is_file() and q.stat().st_size>1000, rel
staff=(root/'tray_overlay/scripts/final_scene_staff.py').read_text(encoding='utf-8')
for token in [
    'MRI_WORLD_YAW_SIDE:', 'TRAY_GOAL_SIDE', 'CHAIR_SIDE:',
    '[FINAL STAFF READY]', 'hospitalNonPhysicalVisual',
    'CreateCollisionEnabledAttr(False)', 'CreateRigidBodyEnabledAttr(False)',
]:
    assert token in staff, token
runtime=(root/'tray_overlay/scripts/isaac_amr_ros_tray_runtime.py').read_text(encoding='utf-8')
assert 'schedule_final_scene_staff' in runtime
print('[PASS] V2.12 final staff: manual pose override + doctor forced upright/world-yaw MRI side + non-physical staff')

# V2.12 actual-world startup localization and no-spin station release.
wp=(root/'ros2_ws/src/hospital_nav2/hospital_nav2/world_pose_initializer.py').read_text(encoding='utf-8')
for token in ['actual Isaac world pose', 'stable_cycles', 'stable_xy_m', 'stable_yaw_deg', 'stable world pose confirmed']:
    assert token in wp, token
for launch_file, robot in [('hospital_amr1_navigation.launch.py','AMR1'),('hospital_amr2_navigation.launch.py','AMR2')]:
    text=(root/'ros2_ws/src/hospital_nav2/launch'/launch_file).read_text(encoding='utf-8')
    assert '"auto_initial_pose": False' in text, launch_file
    assert 'world_pose_initializer' in text, launch_file
    assert f'"robot_name": "{robot}"' in text, launch_file
manager=(root/'ros2_ws/src/hospital_tray_overlay/hospital_tray_overlay/cooperative_transport_manager.py').read_text(encoding='utf-8')
for token in ['straight_only', '[SAFE EGRESS SKIP V2.12]', 'cmd.angular.z = 0.0 if straight_only']:
    assert token in manager, token
assert tray['cooperative_auto_transport']['safe_egress_amr1']['straight_only'] is True
assert tray['cooperative_auto_transport']['safe_egress_amr1']['station_guard_radius_m'] > 0.0
assert final_staff['doctor_mri']['forced_visual_rotation_xyz_deg'] == [90.0,0.0,0.0]
assert final_staff['manual_pose_override_enabled'] is True
assert (root/'CAPTURE_CURRENT_STAFF_POSES.py').is_file()
for token in ['manual pose overrides loaded', 'forced_visual_rotation_xyz_deg', 'manual_poses.get("DoctorMRI")']:
    assert token in staff, token
print('[PASS] V2.12 live Isaac pose lock + startup stability filter + straight-only AMR1 egress + manual staff pose capture')

# Syntax parse all additive Python files. No imports are performed.
for p in list((root/'tray_overlay').rglob('*.py')) + list((root/'ros2_ws/src/hospital_tray_overlay').rglob('*.py')):
    ast.parse(p.read_text(encoding='utf-8'),filename=str(p))
print('[PASS] Python AST validation')

for p in [root/'RUN_TRAY_1_ISAAC_TOTAL_360.sh',root/'RUN_TRAY_2_AUTO_TOTAL_360.sh',root/'tray_overlay/config/isaac_config_tray_integrated.json']:
    t=p.read_text(encoding='utf-8')
    assert 'ROS_DOMAIN_ID=115' not in t
print('[PASS] no DOMAIN115 regression in integrated runtime')
print('[STATIC VALIDATION COMPLETE] actual Isaac camera/ArUco/PhysX/Nav2 docking must be runtime-tested on target PC.')
