# V2.17.2 BED-STYLE ARUCO HANDOFF

V2.17의 문제:
실제 pair는 정상 검출됐지만, 카메라가 트레이 밑으로 가까워질수록
마커가 가려지는데도 d≈0.58m까지 pair를 요구해서 PAIR LOST에 정지했습니다.

V2.17.2:
- 시작 시 두 AMR 모두 실제 pair 필수
- 약 0.25m 동안 실제 ArUco center/yaw visual servo
- 각 AMR이 정렬되면 실제 world pose/yaw 저장
- 이후 마커가 가려지는 것은 정상으로 처리
- 저장된 ArUco 정렬 yaw + odom으로 남은 거리 직진 삽입
- 두 AMR 진행차 3cm 이상이면 앞선 AMR 감속
- 9초 fake fallback 없음
- 초기 ALIGN 스냅 없음
- 물리적으로 못 들어가면 정지하고 attach하지 않음

실행:
Terminal 1:
./RUN_V216_1_ISAAC_SCAN_READY_EXTERNAL_SAFE.sh

Terminal 2:
./RUN_V217_2_TRUE_ARUCO_DOCK_TRANSPORT.sh
