# hospital_total_08091221 + Tray 360 LiDAR + 3-Post ArUco Gate V2

## 기준 원칙
이 통합본의 기준은 `hospital_total_08091221`이다. 기준본 231개 파일은 SHA-256 기준 byte-identical로 유지한다.

유지 대상:
- ROS_DOMAIN_ID=117
- `/home/peter-msi/isaacsim-5.1.0` 기본 Isaac 경로 + 현재 C3 `/mnt/isaac45/isaacsim_5.1` fallback
- AMR2 namespace / `/amr2/map` / `/amr2/traffic_pause`
- AMR1 `/traffic_pause`
- 2층 엘리베이터에만 0.35 m 근접 성공
- stale goal 제거
- 목표 없는 AMR2 resume 후 `READY:WAITING_GOAL`
- 코너 회전: stop 0.60 s, max 0.70 rad/s, min 0.12 rad/s, KP 1.00
- OCR / MRI / 환자 / 침대 도킹 / 엘리베이터 / 기존 path_conflict_manager

## V2 추가 기능
1. PhysX LiDAR `(720,1)` 계열 데이터가 1-ray로 잘리던 문제 수정. `/scan`, `/amr2/scan`은 실제 360도 scan을 runtime precheck한다.
2. 트레이에 **3개의 시각적 마커 기둥 + 5개의 ArUco marker**를 설치한다.
3. 기둥/마커는 render-only라 cart/lift/collider physics를 바꾸지 않는다.
4. PRE_DOCK 이후에만 ArUco node를 late launch한다.
5. 도킹 완료 후 기존 yellow lift + dual FixedJoint를 체결하고, 그때 cooperative Nav2를 lazy launch한다.

## 정확한 ArUco 배치
AMR가 트레이 뒤쪽(local -X)에서 접근해 보는 정면 기준:

```text
      LEFT OUTER POST          CENTER POST          RIGHT OUTER POST

          [ ID 40 ]                                  [ ID 42 ]
              |                    [ ID 44 ]              |
          [ ID 41 ]                                  [ ID 43 ]

                 AMR1 BAY                     AMR2 BAY
```

- Left outer post: ID 40(상), ID 41(하/fallback)
- Center post: ID 44(shared)
- Right outer post: ID 42(상), ID 43(하/fallback)
- Dictionary: `DICT_4X4_50`
- marker size: 0.12 m

### 핵심 기하 설계
트레이 실제 bay center가 `+/-dock_y`일 때 outer marker post를 정확히 `+/-2*dock_y`에 둔다.

따라서:

```text
AMR1 visual target = midpoint(LEFT OUTER, CENTER) = +dock_y
AMR2 visual target = midpoint(CENTER, RIGHT OUTER) = -dock_y
```

즉 카메라에서 두 marker 사이의 중점을 화면 중앙에 맞추면 각 AMR의 실제 도킹 bay center와 일치한다.

### 마커 인식 방식
- AMR1: ID40/41을 하나의 `virtual left outer post`로 합성 + ID44 center
- AMR2: ID44 center + ID42/43을 하나의 `virtual right outer post`로 합성
- 상/하 두 outer marker가 모두 보이면 중심을 평균하여 흔들림을 줄인다.
- 상단 marker가 가려지면 하단 marker 하나로 계속 tracking한다.
- pair midpoint의 `center_error_px`가 직접 final ingress steering에 사용된다.

## 실행
### 최초 1회
```bash
cd ~/hospital_bed_amr_projects/hospital_total_08091221_TRAY_360_LIDAR_GATE_INTEGRATED_v2
chmod +x ./*.sh scripts/*.sh tray_overlay/scripts/*.py
./00_SETUP_TRAY_360_INTEGRATED.sh
```

### Terminal 1 - Isaac
```bash
cd ~/hospital_bed_amr_projects/hospital_total_08091221_TRAY_360_LIDAR_GATE_INTEGRATED_v2
export ROS_DOMAIN_ID=117
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
./RUN_TRAY_1_ISAAC_TOTAL_360.sh
```

Isaac이 뜨면 PLAY. 로그에서 다음을 확인:

```text
[BASE BRIDGE READY] AMR1
[BASE BRIDGE READY] AMR2
[BASE DUAL BRIDGE READY]
[TRAY ARUCO GATE READY V2] ids=40/41 LEFT, 44 CENTER, 42/43 RIGHT
[NAV2 LIDAR 360 READY] ...
```

### Terminal 2 - 자동 시나리오
```bash
cd ~/hospital_bed_amr_projects/hospital_total_08091221_TRAY_360_LIDAR_GATE_INTEGRATED_v2
export ROS_DOMAIN_ID=117
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
./RUN_TRAY_2_AUTO_TOTAL_360.sh
```

흐름:

```text
360 LiDAR precheck
 -> AMR1 latest Nav2
 -> AMR2 latest Nav2
 -> latest path_conflict_manager
 -> AMR2 short safe egress
 -> dual PRE_DOCK
 -> three-post ArUco gate late launch
 -> AMR1 left+center visual alignment
 -> AMR2 center+right visual alignment
 -> dual yellow lift + dual FixedJoint
 -> cooperative bridge/Nav2
 -> target (7.90, 10.13)
 -> attached hold
```

## ArUco 상태 직접 확인
PRE_DOCK 이후 ArUco node가 실행된 상태에서 새 터미널:

```bash
cd ~/hospital_bed_amr_projects/hospital_total_08091221_TRAY_360_LIDAR_GATE_INTEGRATED_v2
./CHECK_TRAY_ARUCO_GATE.sh
```

정상 예:

```text
state: PAIR
center_id: 44
outer_source_ids: [40,41]   # AMR1
```

또는

```text
state: PAIR
center_id: 44
outer_source_ids: [42,43]   # AMR2
```

## 주의
정적 검증에서는 기준본 무결성, Python/Shell/JSON/YAML, marker layout, 360 LiDAR 변환 코드를 검사했다. 실제 카메라 시야, 조명, PhysX, Nav2 및 ArUco final docking 성공 여부는 대상 Isaac Sim PC에서 runtime 시험이 필요하다.
