# final + V2.17.13 트레이 통합

이 폴더는 기존 `final`의 GUI 병원 이송 기능을 그대로 유지하면서, 별도의 트레이 협동 운송 모드를 추가한 통합본입니다.

## 유지된 기능

- GUI 실행과 환자 MRI 이송 시나리오
- ROS Domain 115
- 기존 공간예약, 대기점, 엘리베이터 및 병실 복귀 로직
- 기존 OCR 완화 판정 설정
- 동료 PC의 기본 Isaac Sim 경로

## 추가된 트레이 기능

- AMR1/AMR2용 실제 360도 LaserScan 변환
- 트레이의 3개 마커 기둥과 ArUco ID 40~44
- AMR1: ID 40/41과 중앙 ID 44의 중점 정렬
- AMR2: 중앙 ID 44와 ID 42/43의 중점 정렬
- 도킹 중 오래된 Nav2 PRE_DOCK 경로를 충돌 판단에서 제외
- 양쪽 리프트 상승 및 FixedJoint 결합
- 결합된 두 AMR과 트레이의 협동 운송
- V2.17.13 전용 `tray_cmd_vel` 직접 도킹과 X3 운송 시연

트레이 기능은 기존 GUI 환자 이송 명령에 자동으로 섞이지 않습니다. 같은 `final` 폴더에서 선택적으로 실행하는 별도 모드이므로 기존 GUI 시연을 깨뜨리지 않습니다.

## 최초 1회 빌드

```bash
cd ~/hospital/final
./00_SETUP_TRAY_360_INTEGRATED.sh
```

Isaac Sim이 다른 위치에 있다면 실행 전에만 지정합니다.

```bash
export ISAAC_SIM_DIR=/실제/isaacsim/경로
```

## 모드 A: 출발 위치부터 트레이까지 자동 이동

터미널 1:

```bash
cd ~/hospital/final
./RUN_TRAY_1_ISAAC_TOTAL_360.sh
```

터미널 2:

```bash
cd ~/hospital/final
./RUN_TRAY_2_AUTO_TOTAL_360.sh
```

흐름은 `각 AMR Nav2 이동 → ArUco 정렬 → 직선 도킹 → 리프트/FixedJoint 결합 → 협동 운송`입니다.

## 모드 B: V2.17.13 고속 시연

터미널 1:

```bash
cd ~/hospital/final
./RUN_V217_1_ISAAC_SCAN_READY.sh
```

터미널 2:

```bash
cd ~/hospital/final
./RUN_V217_2_TRUE_ARUCO_DOCK_TRANSPORT.sh
```

이 모드는 두 AMR을 트레이 마커 관측 위치에 배치하고, 실제 ArUco pair를 찾은 뒤 직접 도킹하여 트레이를 운송합니다. 병원 환자 이송 전체 시나리오가 아니라 트레이 기능 자체를 보여 주는 시연 모드입니다.

V2.17.13은 결합 후 최고 요청 속도가 1.98m/s이고 회전도 크게 올라간 설정입니다. 넓은 공간에서 먼저 시험하고, 병원 좁은 복도나 실제 하드웨어에는 그대로 적용하지 마십시오.

## 확인 명령

```bash
./CHECK_360_LIDAR_RUNTIME.sh
./CHECK_TRAY_ARUCO_GATE.sh
```

정상 ArUco 결과는 AMR1이 `40/41 + 44`, AMR2가 `44 + 42/43` 조합으로 `state=PAIR`를 출력합니다.

## 종료

```bash
./STOP_TRAY_INTEGRATED_ROS.sh
```

이 스크립트는 트레이/Nav2/RViz ROS 프로세스를 정리하고 Isaac Sim은 유지합니다.
