# hospital_total_08091221 실행 안내

이 폴더는 Isaac Sim 5.1에서 병원 환경과 AMR 2대를 구동하고, ROS 2 Humble의 Nav2·OCR·ArUco·환자 침대 이송·MRI·엘리베이터·교통 충돌 회피를 함께 실행하는 통합 프로젝트입니다.

## 이 노트북 기준

- 프로젝트: `/home/peter-msi/hospital/hospital_total_08091221`
- Isaac Sim: `/home/peter-msi/isaacsim-5.1.0`
- ROS 2: Humble
- ROS Domain: `115`
- OCR Python: `/home/peter-msi/.venvs/hospital_ocr_ros310`

활성 실행 스크립트와 설정은 위 값으로 맞춰져 있습니다. Isaac 터미널에서는 `/opt/ros/humble/setup.bash`를 직접 source하지 마십시오.

## 주요 구성

- `scripts/isaac_amr_ros.py`: Isaac Stage, AMR 물리·센서·ROS bridge, 도킹, MRI·엘리베이터 런타임
- `patient_transport_manager.py`: 김서울·박인천·서수원 전체 환자 이송 순서
- `ros2_ws/src/hospital_nav2`: AMR1/AMR2 Nav2, 중앙선 경로, pose lock, 복도·경로 충돌 관리
- `ros2_ws/src/hospital_ocr_bridge`: OCR 노드와 환자별 mission launch
- `scripts/aruco_pair_node.py`: 환자별 좌우 ArUco pair 검출
- `patient_mri_transfer`: 세 환자의 침대↔MRI 이동 확장
- `project4`: 병원 USD Stage와 관련 에셋

환자별 ArUco ID와 도킹 후 고정 진입 거리는 다음과 같습니다.

| 환자 | ArUco ID | 진입 거리 |
|---|---:|---:|
| 김서울 | 10 / 11 | 3.3280 m |
| 박인천 | 20 / 21 | 3.5221 m |
| 서수원 | 30 / 31 | 3.1554 m |

ArUco pair가 보이면 이를 우선해 yaw와 좌우 중심을 맞춥니다. 마커가 없거나 오래되면 OCR 이름표 bbox 중심으로 정렬하는 fallback이 있으므로 마커가 없다고 반드시 멈추지는 않습니다. 깊이는 측정하지 않으며 마지막 도킹 진입은 위 고정거리입니다.

## 이번 버전의 핵심 변경

1층 엘리베이터 문이 열린 뒤 바로 직진하지 않고, 먼저 AMR과 결합 침대의 방향을 월드 `+Y`, 즉 `+90°`로 맞춥니다.

```text
WAITING_1F_DOOR_OPEN
→ ROTATING_1F_ENTRY
→ yaw 오차 ±2° 이내
→ DRIVING_IN
```

로그에서 `[1F ELEVATOR YAW] start=...`, `aligned ... target=1.570796`을 확인할 수 있습니다. 진입 중 yaw 폐루프나 문 중심 X 보정, 무진행 자동 후진은 아직 없습니다.

## 최초 1회 또는 소스 변경 후

```bash
cd /home/peter-msi/hospital/hospital_total_08091221
./02_build_ros_ws.sh
./check_project.sh
```

OCR 가상환경이 없을 때만 `./01_install_ocr_ros.sh`, Nav2 패키지가 없을 때만 `./07_install_nav2.sh`를 실행합니다.

## 권장 듀얼 실행 순서

기존 프로세스가 남아 있다면 먼저 실행합니다.

```bash
cd /home/peter-msi/hospital/hospital_total_08091221
./00_stop_all.sh
```

각 명령은 서로 다른 터미널에서 순서대로 실행합니다.

```bash
# 터미널 1
./03_run_isaac.sh

# 터미널 2
./09_run_nav2_amr1.sh

# 터미널 3
./09_run_nav2_amr2.sh

# 터미널 4
./09_run_collision_avoidance.sh

# 터미널 5
./04_run_ocr_mission_1.sh

# 터미널 6
./04_run_ocr_mission_2.sh
```

마지막 두 스크립트는 `1=김서울`, `2=박인천`, `3=서수원` 선택 메뉴를 표시합니다. 직접 지정하려면 다음처럼 실행합니다.

```bash
./04_run_ocr_mission_1.sh 1
./04_run_ocr_mission_2.sh 2
```

같은 환자는 파일 잠금으로 AMR 두 대에 동시에 지정되지 않습니다. 두 AMR을 충전소에서 거의 동시에 출발시키지 말고, 첫 AMR이 충전 구역을 빠져나간 뒤 두 번째 미션을 시작하십시오.

## AMR1 단독 시험

```bash
# 터미널 1
./03_run_isaac.sh

# 터미널 2
./09_run_nav2_amr1.sh

# 터미널 3
./09_run_collision_avoidance.sh

# 터미널 4
./04_run_ocr_mission_1.sh 1
```

공간 예약이 활성화되어 있으므로 단독 시험에서도 충돌/예약 관리자를 반드시 실행해야
합니다. 실행하지 않으면 AMR1은 침대 결합 후 병실 측 staging 위치에서 승인 메시지를 기다립니다.

## 상태 확인

```bash
./10_check_nav2_topics.sh
ros2 topic echo /traffic_conflict/status
ros2 topic info /traffic_pause
ros2 topic info /amr2/traffic_pause
```

standalone Nav2 두 개를 실행할 때 충돌 관리 지도는 `AMR1=/map`, `AMR2=/amr2/map`입니다. `09_run_nav2_dual.sh`를 사용하는 공유-map 시험에서는 다음처럼 실행합니다.

```bash
AMR2_MAP_TOPIC=/map ./09_run_collision_avoidance.sh
```

## 특수동작 공간 기반 정지

OCR/ArUco 접근, 결합·해제, MRI 강제 전후진, 엘리베이터 동작이 시작됐다는 이유만으로 다른 AMR을 전역 정지하지 않습니다. 기본값은 다음과 같습니다.

- 서로 다른 층: 상대 AMR 계속 이동
- 같은 층, 중심거리 5m 초과: 상대 AMR 계속 이동
- 같은 층, 중심거리 5m 이내: 먼저 특수동작을 시작한 AMR 우선, 상대 정지
- 정지 후 중심거리 6m 초과: 상대 재개
- 층을 모르거나 world pose가 1초 이상 오래됨: 기존 방식처럼 안전 정지

실행 중 다음 상태를 확인합니다.

```bash
ros2 topic echo /traffic_conflict/status
```

`SPECIAL_SPATIAL_CLEAR`이면 특수동작 중이어도 공간이 떨어져 있어 둘 다 이동할 수 있고, `SPECIAL_SPATIAL_HOLD`이면 같은 층의 안전거리 안이라 한 대가 대기합니다.

시험 중 거리만 바꾸려면 소스 수정 없이 실행할 수 있습니다.

```bash
SPECIAL_TRIGGER_DISTANCE_M=4.0 SPECIAL_RELEASE_DISTANCE_M=5.0 ./09_run_collision_avoidance.sh
```

기존 전역 정지와 A/B 비교하려면 다음처럼 실행합니다.

```bash
SPECIAL_SPATIAL_ENABLED=false ./09_run_collision_avoidance.sh
```

이 단계는 AMR 중심거리 기반 1차 완화입니다. 침대 결합 상태의 실제 footprint와 미래 swept footprint 교환은 아직 구현하지 않았으므로 좁은 복도에서 두 AMR의 교행을 허용하는 기능은 아닙니다.

## 침대 이송 경로 공간 예약

침대를 결합한 AMR은 좁은 복도와 엘리베이터에 들어가기 전에
`LOADED_TRANSPORT_ROUTE`를 예약합니다. 이 예약은 1층 좁은 복도, 엘리베이터,
목적 층 출구와 2층 공용 통로를 하나의 배타 구역으로 묶는 보수적인 1차 구현입니다.

- 이미 예약을 점유한 AMR은 중간에 선점되지 않습니다.
- 두 요청이 거의 동시에 들어오면 `TO_MRI_LOADED`가 `FROM_MRI_LOADED`보다 우선입니다.
- 경로가 비어 있으면 별도 대기점을 경유하지 않고 엘리베이터 목표로 바로 이동합니다.
- 예약을 받지 못한 출발 AMR은 침대 결합 후의 병실 측 staging 위치에서 스스로 정지합니다.
- 예약을 받지 못한 복귀 AMR은 MRI 검사 대기 위치에서 스스로 정지합니다.
- AMR1이 1층 하차를 마쳤을 때 AMR2가 `TO_MRI_LOADED` 예약 queue에 있으면, AMR1만
  로비 대기점 `(-29.7245, 19.3880, -1.5814rad)`으로 이동해 현재 예약을 넘깁니다.
- AMR1은 AMR2의 엘리베이터 상태가 `RIDING_TO_2F` 이후로 확인될 때까지 대기점에서
  zero velocity를 유지합니다. AMR2가 실제로 1층을 떠난 뒤에만 병실 복귀를 재개합니다.
- AMR2 요청이 없으면 AMR1은 대기점을 경유하지 않습니다. 대기점 이동 중 요청이
  사라져도 예약을 해제하지 않고 기존 병실 복귀를 계속합니다.
- AMR2가 예약을 받은 뒤 엘리베이터에서 실패하거나 240초 안에 상승 상태가 확인되지
  않으면 AMR1은 안전 대기점에 남고 예약을 강제로 선점하지 않습니다.
- 상대 AMR이 아직 OCR 위치 이동·환자 확인·결합 준비 중이면 예약만으로 전역 정지하지 않습니다.
- 두 AMR의 실제 Nav 경로가 겹치거나 같은 층에서 특수동작 안전거리에 들어오면 기존 충돌 관리가 별도로 개입합니다.
- MRI 검사 대기 위치에 도달하거나 병실에 침대를 돌려놓고 빠져나온 뒤에만 예약을 해제합니다.
- owner heartbeat가 2초 이상 끊기면 물리 구역이 비었다고 추정하지 않고 두 AMR을 정지합니다.

예약 상태는 다음 명령으로 확인합니다.

```bash
ros2 topic echo /traffic_reservation/status
```

정상 흐름의 주요 상태는 `GRANTED → OCCUPIED → RELEASED → CLEAR`입니다.
1층 인계가 작동하면 AMR1 미션 로그에서
`[1F 예약 인계 준비] → [1F 예약 인계 대기] → [1F 예약 인계 완료]`를 확인합니다.
`OWNER_STALE_FAILSAFE_HOLD`이면 예약 owner 미션 프로세스가 종료됐거나 통신이
끊긴 상태이므로 자동 재출발시키지 말고 실제 AMR 위치를 확인한 뒤 전체 프로세스를
재시작하십시오. 첫 시험에서는 기존 권장 실행 순서를 그대로 사용하고, 두 번째 미션은
첫 AMR이 침대를 결합하고 병실을 빠져나가기 시작할 때 실행합니다.

## 주의사항

- 미션 스크립트가 OCR·ArUco·환자 이송 매니저를 함께 시작하므로 `04_run_ocr_dual.sh`나 `13_run_patient_transport.sh`를 중복 실행하지 마십시오.
- 결합 후에도 Nav2 footprint는 AMR 본체 크기만 사용하며 침대 전체 크기는 반영하지 않습니다.
- 엘리베이터 진입은 강제 직선 주행입니다. 주변 장애물과 문틀 접촉을 직접 확인하십시오.
- `ROTATING_1F_ENTRY`는 진입 전 각도를 맞추지만, 침대가 AMR에 비스듬히 결합된 상대각도까지 자동 보정하지는 않습니다.
- MRI 완료 `K`는 OCR/미션 터미널에 포커스를 둔 상태에서 누르십시오. Isaac 창에서 `K`를 누르면 선택 Prim을 MRI 대상으로 저장합니다.
- OCR 판정은 시연 지속성을 우선한 완화된 규칙이며 실제 환자 신원 확인용 안전 시스템이 아닙니다.
- 종료는 각 터미널에서 `Ctrl+C` 후, 남은 프로세스가 있으면 `./00_stop_all.sh`를 실행합니다.

## 선택 기능: 트레이 협동 운송

`hospital_total_08091221_TRAY_360_LIDAR_GATE_INTEGRATED_v2_17_13_TRANSPORT_X3_ALL`의 핵심 기능을 선택 실행 모드로 통합했습니다. 기존 GUI 환자 이송은 그대로 유지되며, 트레이 모드는 별도 실행기를 사용합니다.

설치·실행·안전 주의사항은 `README_TRAY_FINAL_INTEGRATION_KO.md`를 참고하십시오.
