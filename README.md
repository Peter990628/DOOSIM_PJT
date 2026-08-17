# DOOSIM 병원 AMR 통합 프로젝트

Isaac Sim 병원 디지털 트윈에서 2대의 AMR이 환자 침대를 인식·결합·이송하고, 좁은 복도와 엘리베이터를 공유하도록 구현한 ROS 2 통합 프로젝트입니다. 별도 모드로 2대의 AMR이 트레이를 공동 운송하는 기능도 포함합니다.

## 제출물 핵심 구성

- `project4/`: 병원 시뮬레이션 USD와 표지·이름표 에셋
- `scripts/isaac_amr_ros.py`: Isaac Sim 메인 실행 코드
- `ros2_ws/src/hospital_nav2/`: 듀얼 Nav2, 중앙선 주행, 충돌 회피·공간 예약
- `ros2_ws/src/hospital_ocr_bridge/`: 환자 이름표 OCR
- `ros2_ws/src/hospital_tray_overlay/`: 360° LiDAR·ArUco·협동 트레이 운송
- `patient_transport_manager.py`: 환자 침대 이송 시나리오 관리
- `app.py`, `templates/`, `static/`: 관제 GUI

## 문서 바로가기

- [운영환경 및 장비 목록](src/협동3_문서모음/docs/01_운영환경_및_장비목록.md)
- [설치 및 실행 가이드](src/협동3_문서모음/docs/02_설치_및_실행_가이드.md)
- [소스·에셋 구성 명세](src/협동3_문서모음/docs/03_소스_및_에셋_구성.md)
- [검증 및 문제 해결](src/협동3_문서모음/docs/04_검증_및_문제해결.md)
- [기존 상세 실행 설명](README_KO.md)
- [트레이 협동 운송 설명](README_TRAY_FINAL_INTEGRATION_KO.md)

## 시스템 설계 및 플로우차트

- [시스템 아키텍처](src/협동3_문서모음/두심이%20시스템%20설계도(메인,서브).drawio.png) ([원본 drawio](src/협동3_문서모음/두심이%20시스템%20설계도(메인,서브).drawio))
- [1차 시나리오 플로우차트 (메인)](src/협동3_문서모음/메인시나리오%20플로우차트.drawio.png) ([원본 drawio](src/협동3_문서모음/메인시나리오%20플로우차트.drawio))
- [2차 시나리오 플로우차트 (서브)](src/협동3_문서모음/서브%20시나리오%20플로우차트.drawio.png) ([원본 drawio](src/협동3_문서모음/서브%20시나리오%20플로우차트.drawio))

## 운영환경

| 항목 | 값 |
|---|---|
| 운영체제 | Ubuntu 22.04 LTS |
| ROS 2 | Humble |
| 시뮬레이터 | Isaac Sim 5.1 |
| Python | ROS 측 3.10, Isaac 내장 Python |
| ROS Domain ID | 115 |
| RMW | `rmw_fastrtps_cpp` |
| 주 운용 층 | 1층 |
| GUI | Flask 기반 웹 GUI |
| OCR | PaddleOCR 3.1.1 |
| 영상처리 | OpenCV contrib 4.10.0.84 |

## 실제 사용 장비 목록

| 장비 | 수량 | 사양 | 용도 |
|---|---:|---|---|
| Isaac Sim 실행 노트북 | 2대 | RAM 32GB, NVIDIA GeForce RTX 5080, VRAM 16GB | Isaac Sim 병원 환경 실행, AMR 시뮬레이션, ROS 2 및 GUI 구동 |
| TP-Link 유선 네트워크 허브 | 1대 | 유선 LAN 연결 지원 | 두 노트북 간 ROS 2 DDS 통신망 구성 |
| LAN 케이블 | 2개 | 이더넷 케이블 | 각 노트북과 TP-Link 허브 연결 |

두 대의 노트북을 TP-Link 유선 네트워크 허브에 연결하여 동일한 로컬 네트워크를 구성하였습니다. 양쪽 노트북은 `ROS_DOMAIN_ID=115`와 `rmw_fastrtps_cpp`를 동일하게 설정하여 AMR 상태, 센서 데이터, Nav2 명령 및 GUI 정보를 주고받았습니다.

## 의존성 설치

GUI 및 OCR 실행에 필요한 Python 의존성은 다음 파일에 정리되어 있습니다.

- `requirements.txt`: Flask GUI 의존성
- `requirements_ocr_ros.txt`: PaddleOCR, OpenCV, NumPy 등 OCR 의존성

GUI 의존성 설치:

```bash
python3 -m pip install -r requirements.txt
```

OCR 의존성은 전용 설치 스크립트를 사용합니다.

```bash
./01_install_ocr_ros.sh
```

ROS 2 및 Nav2 관련 패키지는 pip가 아닌 Ubuntu/ROS 패키지 관리자를 통해 설치합니다.

## 빠른 실행

최초 1회:

```bash
cd <압축을_푼_경로>/final
./01_install_ocr_ros.sh   # OCR을 사용할 때만
./07_install_nav2.sh      # Nav2가 설치되지 않았을 때만
./02_build_ros_ws.sh
./check_project.sh
```

기본 시연은 서로 다른 터미널에서 아래 순서로 실행합니다.

```bash
./03_run_isaac.sh
./09_run_nav2_amr1.sh
./09_run_nav2_amr2.sh
./09_run_collision_avoidance.sh
./10_run_gui.sh
```

GUI에서 AMR과 환자를 선택해 미션을 시작합니다. GUI 없이 실행하려면 `./04_run_ocr_mission_1.sh 1`처럼 실행합니다. 환자 번호는 `1=김서울`, `2=박인천`, `3=서수원`입니다.

Isaac Sim 설치 위치가 기본값과 다르면 실행 전에 지정합니다.

```bash
export ISAAC_SIM_DIR=/실제/isaacsim/경로
```

## 네트워크 공통 설정

두 노트북에서 다음 값을 동일하게 사용합니다.

```bash
export ROS_DOMAIN_ID=115
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

## 종료

각 터미널에서 `Ctrl+C`로 종료한 뒤 남은 프로세스가 있으면 다음을 실행합니다.

```bash
./00_stop_all.sh
```

트레이 모드는 `./STOP_TRAY_INTEGRATED_ROS.sh`를 사용합니다.

## 안전 및 적용 범위

- 본 프로젝트는 시뮬레이션·교육용 프로토타입입니다.
- OCR은 시연 지속성을 위한 완화 판정을 포함하므로 실제 환자 신원 확인에 사용할 수 없습니다.
- 엘리베이터 진입과 일부 도킹 구간은 직접 속도 명령을 사용하므로 시연 중 충돌 여부를 관찰해야 합니다.
- 실제 하드웨어 적용 전 속도, footprint, 제동거리, 통신 장애 대응을 별도로 검증해야 합니다.
