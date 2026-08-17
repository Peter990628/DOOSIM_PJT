# V2.4 — 트레이 전면 ArUco + Follow Camera 고정

기준본: `hospital_total_08091221`

이번 버전은 기존 병원 기능을 변경하지 않고 overlay에만 다음을 적용합니다.

1. **트레이 ArUco 위치 수정**
   - PRE_DOCK은 트레이 로컬 `-X` 쪽에서 접근합니다.
   - V2/V2.3은 마커 게이트가 `+X` 면에 있어 실제 접근 방향 기준으로 트레이 뒤쪽에 가려질 수 있었습니다.
   - V2.4는 전체 3기둥/5마커를 **로컬 -X 전면**으로 이동합니다.
   - 마커 표면 normal은 `-X`를 유지해서 접근 중인 AMR 카메라를 정면으로 봅니다.
   - 배치: `40/41 | 44 | 42/43`
   - AMR1 도킹 중심 = LEFT + CENTER 중점, AMR2 = CENTER + RIGHT 중점.
   - 기둥/백플레이트/마커는 visual-only라 트레이 collider/lift/FixedJoint는 변경하지 않습니다.

2. **Follow Camera 수정**
   - 기존 overlay에는 AMR1 바로 위 `6.0m` top-down 카메라가 들어 있어 병원 벽/천장/층 구조를 통과해 보이는 현상이 생길 수 있었습니다.
   - 새 위치: AMR1 로컬 기준 **뒤 -3.2m / 위 +2.1m**.
   - 시선: AMR 본체 중심 `z=0.35m`.
   - world up: `+Z`.
   - Camera Prim은 기존 규칙대로 `/World/AMR1/base_link/FollowCameraRig/FollowCamera` 아래 유지합니다.
   - 새 `follow_camera_guard.py`가 매 프레임 로컬 카메라 transform을 다시 고정해, viewport 조작으로 camera prim이 이동해도 바로 복원합니다.

3. **SSD 복구 스크립트 보완**
   - `/media/.../Seagate Expansion Drive`처럼 공백이 포함된 desktop mount path를 `findmnt -r`가 `\\x20`으로 출력하던 문제를 피합니다.
   - V2.4는 mount point 문자열 대신 `/dev/sda1` 같은 **장치 자체를 unmount**합니다.

## 정적 검증 결과

- hospital_total 영속 원본 파일 213개 SHA-256 동일
- DOMAIN 117 유지
- 최신 traffic pause / AMR2 namespace / stale goal / 2F elevator / corner rotation 유지
- 360 LiDAR overlay 유지
- ArUco 전면(local -X) 배치 검사 통과
- Follow Camera -3.2m / +2.1m 및 runtime pose-lock 검사 통과
- Python AST / JSON / bash syntax 검사 통과

> Isaac/PhysX/카메라 렌더링/실제 ArUco 인식은 대상 PC에서 runtime 확인이 필요합니다.

## 실행

최초 1회:

```bash
cd ~/hospital_bed_amr_projects/hospital_total_08091221_TRAY_360_LIDAR_GATE_INTEGRATED_v2_4
chmod +x ./*.sh scripts/*.sh tray_overlay/scripts/*.py
./00_SETUP_TRAY_360_INTEGRATED.sh
```

터미널 1:

```bash
cd ~/hospital_bed_amr_projects/hospital_total_08091221_TRAY_360_LIDAR_GATE_INTEGRATED_v2_4
export ROS_DOMAIN_ID=117
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
./RUN_TRAY_1_ISAAC_TOTAL_360.sh
```

Isaac에서 확인할 로그:

```text
[TRAY ARUCO GATE READY V2.4 FRONT]
[TRAY ARUCO FRONT FACE] local_x=...
[FOLLOW CAMERA GUARD READY V2.4]
```

Isaac PLAY 후 터미널 2:

```bash
cd ~/hospital_bed_amr_projects/hospital_total_08091221_TRAY_360_LIDAR_GATE_INTEGRATED_v2_4
export ROS_DOMAIN_ID=117
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
./RUN_TRAY_2_AUTO_TOTAL_360.sh
```
