# hospital_total_08091221 실행 안내

이 폴더는 Isaac Sim 5.1에서 병원 환경과 AMR 2대를 구동하고, ROS 2 Humble의 Nav2·OCR·ArUco·환자 침대 이송·MRI·엘리베이터·교통 충돌 회피를 함께 실행하는 통합 프로젝트입니다.

## 이 노트북 기준

- 프로젝트: `/home/peter-msi/hospital/hospital_total_08091221`
- Isaac Sim: `/home/peter-msi/isaacsim-5.1.0`
- ROS 2: Humble
- ROS Domain: `117`
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
./04_run_ocr_mission_1.sh 1
```

1층 엘리베이터 방향 수정은 먼저 이 단독 구성으로 확인하는 것이 좋습니다.

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

## 주의사항

- 미션 스크립트가 OCR·ArUco·환자 이송 매니저를 함께 시작하므로 `04_run_ocr_dual.sh`나 `13_run_patient_transport.sh`를 중복 실행하지 마십시오.
- 결합 후에도 Nav2 footprint는 AMR 본체 크기만 사용하며 침대 전체 크기는 반영하지 않습니다.
- 엘리베이터 진입은 강제 직선 주행입니다. 주변 장애물과 문틀 접촉을 직접 확인하십시오.
- `ROTATING_1F_ENTRY`는 진입 전 각도를 맞추지만, 침대가 AMR에 비스듬히 결합된 상대각도까지 자동 보정하지는 않습니다.
- MRI 완료 `K`는 OCR/미션 터미널에 포커스를 둔 상태에서 누르십시오. Isaac 창에서 `K`를 누르면 선택 Prim을 MRI 대상으로 저장합니다.
- OCR 판정은 시연 지속성을 우선한 완화된 규칙이며 실제 환자 신원 확인용 안전 시스템이 아닙니다.
- 종료는 각 터미널에서 `Ctrl+C` 후, 남은 프로세스가 있으면 `./00_stop_all.sh`를 실행합니다.
