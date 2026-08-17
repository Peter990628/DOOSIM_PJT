#!/usr/bin/env python3
from __future__ import annotations
import hashlib, json, sys
from pathlib import Path

ROOT=Path(__file__).resolve().parent
EXPECTED_NAMEPLATE={
 'nameplate_kim_seoul.png':'77d96cc1b1e51a905fdba47c708aa012d282a2a9244178bd22d4912edda4b1ef',
 'nameplate_park_incheon.png':'1c2e99fb8d8cf4229ddd5493f6577a16b9704066f85daeaa73693e175bae715a',
 'nameplate_seo_suwon.png':'047a1e6253b006bf8546414f802a4ca06dbe5992ff7869e623f18953e137b250',
}
EXPECTED_ELEVATOR='24d20af51b9e35ce2714e29b00b2b56cf7b9450611c49875ed25cf427ed5327d'
EXPECTED_STAGE='44c5a5366b49479019795fa5f72b88d705fbe3dfb4aac90fca85fd8f95896d2c'

def sha(p:Path)->str:
 return hashlib.sha256(p.read_bytes()).hexdigest()

def fail(msg:str):
 print('[FAIL]',msg); raise SystemExit(1)

for name,digest in EXPECTED_NAMEPLATE.items():
 p=ROOT/'project4/signs'/name
 if not p.is_file() or sha(p)!=digest: fail(f'original OCR nameplate changed: {name}')
if sha(ROOT/'scripts/elevator_map_only.py')!=EXPECTED_ELEVATOR: fail('elevator_map_only.py changed')
if sha(ROOT/'project4/project4_hospital_bed_amr_v1_15_ocr.usd')!=EXPECTED_STAGE: fail('main USD stage changed')

try:
 import cv2
except Exception as e:
 fail(f'OpenCV unavailable: {e}')
d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params=cv2.aruco.DetectorParameters()
try: det=cv2.aruco.ArucoDetector(d,params)
except AttributeError: det=None
for marker_id in (10,11,20,21,30,31):
 p=ROOT/f'project4/signs/aruco/aruco_4x4_50_id_{marker_id}.png'
 if not p.is_file(): fail(f'marker PNG missing: {p.name}')
 im=cv2.imread(str(p),cv2.IMREAD_GRAYSCALE)
 if im is None: fail(f'cannot read marker PNG: {p.name}')
 if det is not None: _,ids,_=det.detectMarkers(im)
 else: _,ids,_=cv2.aruco.detectMarkers(im,d,parameters=params)
 got=[] if ids is None else [int(x) for x in ids.flatten()]
 if got != [marker_id]: fail(f'{p.name} detects as {got}, expected [{marker_id}]')

cfg=json.loads((ROOT/'config/isaac_config.json').read_text())
if int(cfg.get('ros2',{}).get('domain_id',-1))!=120: fail('ROS_DOMAIN_ID config is not 120')
a=cfg.get('aruco_markers',{})
if not a.get('enabled'): fail('aruco_markers disabled')
pairs={x.get('patient'):(x.get('left_id'),x.get('right_id')) for x in a.get('beds',[])}
if pairs!={'김서울':(10,11),'박인천':(20,21),'서수원':(30,31)}: fail(f'ArUco pair map mismatch: {pairs}')

manager=(ROOT/'patient_transport_manager.py').read_text()
for token in ('ARUCO_RESULT_TOPIC = "/amr1/aruco/result"','ARUCO_PAIRS = {','OCR bbox','pair_center_x','[ArUco 자동 접근 완료]'):
 if token not in manager: fail(f'mission manager missing token: {token}')
if 'bbox_center_x=float' in manager: fail('mission manager still builds docking observation from OCR bbox')

isaac=(ROOT/'scripts/isaac_amr_ros.py').read_text()
if 'install_aruco_markers' not in isaac or '[ARUCO READY]' not in isaac: fail('Isaac ArUco card installer not integrated')
setup=(ROOT/'ros2_ws/src/hospital_ocr_bridge/setup.py').read_text()
if 'aruco_pair_node = hospital_ocr_bridge.aruco_pair_node:main' not in setup: fail('ArUco ROS entry point missing')
launch=(ROOT/'ros2_ws/src/hospital_ocr_bridge/launch/amr1_ocr.launch.py').read_text()
if 'executable="aruco_pair_node"' not in launch: fail('amr1 OCR launch does not start ArUco detector')

qr_files=[p for p in ROOT.rglob('*') if p.is_file() and 'qr_' in p.name.lower() and 'output' not in p.parts]
if qr_files: fail('QR artifacts unexpectedly present: '+', '.join(str(p.relative_to(ROOT)) for p in qr_files[:5]))

print('[PASS] supplied FINAL baseline preserved + six separate bed-parented ArUco PNG cards integrated')
print('[PASS] original 3 OCR nameplate PNGs unchanged; no QR artifacts')
print('[PASS] full demo keeps OCR identity check but docking centre now comes from paired ArUco')
print('[PASS] 김서울=10/11, 박인천=20/21, 서수원=30/31; ROS domain=120')
print('[PASS] elevator and main USD stage byte-identical to supplied baseline')
