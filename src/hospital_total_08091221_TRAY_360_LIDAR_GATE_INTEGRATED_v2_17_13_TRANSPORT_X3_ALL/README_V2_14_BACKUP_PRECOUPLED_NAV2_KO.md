# V2.14 BACKUP PRE-COUPLED NAV2

발표 백업용 독립 실행 경로입니다. 기존 자동 도킹, ArUco, traffic manager 파일은 수정하지 않습니다.

## 동작
1. 트레이 시작 위치 (-22.69, 11.03, yaw 0)에서 시작
2. Isaac physics 시작 직후 AMR1/AMR2 Lift UP + dual FixedJoint
3. 하나의 cooperative_base_link + dual 360 LiDAR bridge 생성
4. Terminal2 실행 시 RViz 자동 실행
5. cooperative Nav2가 사용자 스크린샷 목표 (7.7732, 6.329, yaw 0)로 자동 이동
6. BACKUP Nav2에서만 live LiDAR obstacle layer를 비활성화해 사소한 접촉 때문에 정지하지 않음. 정적 지도 planner는 유지.
7. RViz `/coop/wheel_telemetry_markers`에 AMR1/AMR2의 서로 다른 바퀴 각속도를 실시간 표시

## 운동학 표시
트레이 중심 명령이 V, W이고 각 AMR 횡방향 오프셋이 y_i일 때:
- v_i = V - W y_i
- FL/RL = (v_i - L W) / r
- FR/RR = (v_i + L W) / r

현재 r=0.075m, L=0.5825m, y1=+0.425m, y2=-0.425m.
따라서 회전 시 두 AMR의 병진속도와 네 바퀴의 목표 각속도가 서로 달라집니다. 이 값이 RViz와 Terminal2에 실시간 표시됩니다.

## 실행
최초 1회: `./00_SETUP_TRAY_360_INTEGRATED.sh`

Terminal1: `./RUN_BACKUP_1_ISAAC_PRECOUPLED.sh`

Terminal2 (Isaac PLAY 후): `./RUN_BACKUP_2_RVIZ_NAV2_AUTO_GOAL.sh`
