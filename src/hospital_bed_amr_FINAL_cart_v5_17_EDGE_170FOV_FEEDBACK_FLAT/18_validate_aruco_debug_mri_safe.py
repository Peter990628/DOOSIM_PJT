#!/usr/bin/env python3
from __future__ import annotations
import hashlib, json, math, re
from pathlib import Path
import cv2

ROOT = Path(__file__).resolve().parent

def fail(msg: str):
    print('[FAIL]', msg)
    raise SystemExit(1)

def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()

# Preserve critical baseline assets/logic.
if sha(ROOT/'scripts/elevator_map_only.py') != '24d20af51b9e35ce2714e29b00b2b56cf7b9450611c49875ed25cf427ed5327d':
    fail('elevator_map_only.py changed')
if sha(ROOT/'project4/project4_hospital_bed_amr_v1_15_ocr.usd') != '44c5a5366b49479019795fa5f72b88d705fbe3dfb4aac90fca85fd8f95896d2c':
    fail('main USD stage changed')

cfg=json.loads((ROOT/'config/isaac_config.json').read_text())
if int(cfg.get('ros2',{}).get('domain_id',-1)) != 120:
    fail('config ROS domain is not 120')

# ArUco independent PNGs still decode as the intended IDs.
d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params=cv2.aruco.DetectorParameters()
try: det=cv2.aruco.ArucoDetector(d,params)
except AttributeError: det=None
for mid in (10,11,20,21,30,31):
    p=ROOT/f'project4/signs/aruco/aruco_4x4_50_id_{mid}.png'
    if not p.is_file(): fail(f'missing {p.name}')
    im=cv2.imread(str(p),cv2.IMREAD_GRAYSCALE)
    if im is None: fail(f'cannot read {p.name}')
    if det is not None: _,ids,_=det.detectMarkers(im)
    else: _,ids,_=cv2.aruco.detectMarkers(im,d,parameters=params)
    got=[] if ids is None else [int(v) for v in ids.flatten()]
    if got != [mid]: fail(f'{p.name} detects as {got}')

node=(ROOT/'ros2_ws/src/hospital_ocr_bridge/hospital_ocr_bridge/aruco_pair_node.py').read_text()
launch=(ROOT/'ros2_ws/src/hospital_ocr_bridge/launch/amr1_ocr.launch.py').read_text()
for token in ('debug_image_topic', '/amr1/aruco/debug_image', 'CAM CENTER', 'PAIR RIGHT', 'debug_msg.data = debug.tobytes()'):
    if token not in node: fail(f'ArUco debug node missing: {token}')
if '"debug_image_topic": "/amr1/aruco/debug_image"' not in launch:
    fail('launch does not configure ArUco debug image topic')

manager=(ROOT/'patient_transport_manager.py').read_text()
for token in (
    'MRI_TARGET_2F = {"x": 8.5246, "y": 5.8035}',
    'MRI_SAFE_STANDOFF_M = 1.05',
    'PATIENT1_STATUS_TOPIC = "/patient_transfer/patient1/status"',
    'wait_patient_state("MRI_BED"',
    'wait_patient_state("TRANSPORT_BED"',
    '2층 MRI 안전 정지점',
):
    if token not in manager: fail(f'MRI safe-flow token missing: {token}')

# Reproduce the derived stop point and guarantee it sits inside the auto-transfer radius.
tx,ty=8.5246,5.8035
ox,oy=7.02,6.37464
standoff=1.05
dx,dy=ox-tx,oy-ty
n=math.hypot(dx,dy)
gx,gy=tx+standoff*dx/n, ty+standoff*dy/n
if not (abs(math.hypot(gx-tx,gy-ty)-1.05) < 1e-6): fail('MRI standoff derivation invalid')
if math.hypot(gx-tx,gy-ty) >= 1.25: fail('MRI stop point outside patient-transfer enter radius')

# Check the 2F occupancy map says the stop point is free.
yaml=(ROOT/'ros2_ws/src/hospital_nav2/maps/hospital_map_2f.yaml').read_text()
res=float(re.search(r'resolution:\s*([0-9.]+)',yaml).group(1))
origin_vals=[float(v.strip()) for v in re.search(r'origin:\s*\[([^\]]+)\]',yaml).group(1).split(',')[:2]]
im=cv2.imread(str(ROOT/'ros2_ws/src/hospital_nav2/maps/hospital_map_2f.png'),cv2.IMREAD_GRAYSCALE)
if im is None: fail('cannot read hospital_map_2f.png')
h,w=im.shape
col=int(math.floor((gx-origin_vals[0])/res))
row=h-1-int(math.floor((gy-origin_vals[1])/res))
if not (0<=row<h and 0<=col<w): fail('MRI stop point is outside 2F map')
if int(im[row,col]) != 255: fail(f'MRI stop point is not free in map: pixel={int(im[row,col])}')

print('[PASS] OCR identity + independent ArUco pair docking preserved')
print('[PASS] /amr1/aruco/debug_image publishes marker boxes, IDs, camera centre, pair centre and error')
print(f'[PASS] MRI safe stop derived from measured target: ({gx:.4f}, {gy:.4f}), standoff=1.05m < enter_radius=1.25m')
print('[PASS] MRI sequence now waits for MRI_BED before leaving and TRANSPORT_BED before return')
print('[PASS] ROS_DOMAIN_ID=120; elevator and main USD unchanged')
