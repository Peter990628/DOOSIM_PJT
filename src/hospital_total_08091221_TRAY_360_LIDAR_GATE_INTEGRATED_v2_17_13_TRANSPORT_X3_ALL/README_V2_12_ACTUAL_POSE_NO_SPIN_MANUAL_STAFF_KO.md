# V2.12 — 실제 Isaac Pose Lock + AMR1 시작 회전 억제 + 수동 사람 배치 보존

## 1. AMR1이 도킹스테이션에서 이상하게 회전한 원인
V2.11까지 AMR1 Nav2의 Pose Lock은 실제 Isaac Stage의 `/amr1/world_pose`가 아니라 과거 고정값
`x=-45.0467, y=31.8558, yaw=-1.566514`를 자동으로 사용했다.
AMR2도 `x=-47.2788, y=26.5713, yaw=0` 고정값을 사용했다.

Isaac의 odom은 실제 시작 위치/방향을 기준으로 상대 이동을 내보내므로, Stage에서 AMR 위치나 yaw가 조금이라도 달라지면
Nav2 map->odom과 실제 병원 월드 좌표가 회전/이동 불일치 상태가 된다. 이 상태에서 CenterlineNavigator가 첫 구간의 방향을 맞추려고 하면
도킹스테이션에서 필요 이상으로 회전하거나 RViz의 센서/경로 표현이 맵 밖으로 틀어져 보일 수 있다.

## 2. V2.12 수정
- AMR1: `/amr1/world_pose`를 6회 연속 안정 확인한 뒤 그 좌표/방향을 `/initialpose`로 자동 입력한다.
- AMR2: `/amr2/world_pose`를 동일하게 사용한다.
- 기존 하드코딩 초기 Pose는 최종 실행 launch에서 사용하지 않는다.
- PhysX가 시작 직후 흔들리는 동안 잘못 잠그지 않도록 안정 필터를 추가했다.
- AMR1 safe-egress는 0.12m `straight_only`: `angular.z=0`으로 고정한다.
- AMR1이 과거 도킹스테이션 반경 0.90m 밖에 이미 수동 배치되어 있으면 safe-egress 자체를 건너뛴다.

## 3. 의사 모델이 누웠던 원인
`doctor.glb`의 원본 bbox는 대략 X=194.09, Y=181.78, Z=35.96이다.
T-pose의 팔 폭 X가 실제 키 방향 Y보다 조금 길기 때문에 기존 bbox 자동 판정은 X를 세로로 세우는 회전을 선택할 수 있었다.
V2.12에서는 DoctorMRI만 `forced_visual_rotation_xyz_deg=[90,0,0]`으로 고정하여 local +Y 몸통 방향을 world +Z로 세운다.
MRI 옆 자동 배치도 imported table의 기울어진 local axis 대신 fixed MRI의 world-Z yaw를 기준으로 계산한다.

## 4. 지금 Isaac에서 직접 옮겨 놓은 사람 위치를 그대로 저장하는 방법
현재 원하는 위치로 이미 사람을 옮겨 놓은 Isaac 창이 아직 열려 있다면, V2.12 폴더를 먼저 설치한 뒤
Isaac Sim의 Script Editor에서 V2.12 루트의 `CAPTURE_CURRENT_STAFF_POSES.py`를 한 번 실행한다.

저장 파일:
`tray_overlay/config/manual_staff_poses.json`

다음 V2.12 실행부터 DoctorMRI / WomanDoctorTrayGoal / NurseDesk의 캡처된 root world pose가 자동 배치보다 우선한다.

## 5. 보존된 기능
- V2.10 same-direction convoy: 2.5m HOLD / 3.2m RELEASE
- V2.11 first-arrival ArUco
- expected ArUco ID 하나 안정 인식 후 고정거리 직진 도킹
- 현재 tray 2.20m + PRE_DOCK 0.95m 기준 자동 2.05m 직진
- tray docking traffic bypass
- dual lift + FixedJoint
- cooperative Nav2
- 360 LiDAR
- 기존 follow camera
- Domain 117
