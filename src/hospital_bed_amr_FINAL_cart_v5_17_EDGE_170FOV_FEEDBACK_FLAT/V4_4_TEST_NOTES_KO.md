# V4.4 Corner State + Path Lock 테스트

목표:
1. 작은 LiDAR/Costmap 변화로 /plan이 흔들려도 현재 segment를 즉시 버리지 않는다.
2. 큰 방향 변화(코너)를 감지하면 DRIVE -> STOP -> ROTATE -> DRIVE 상태로 전환한다.
3. 회전 중 최종 /trolley/cmd_vel의 linear.x는 0이어야 한다.

기본값:
- corner_detect_angle_deg: 28 deg
- corner_stop_distance_m: 0.18 m
- corner_exit_angle_deg: 2 deg
- replan_min_interval_sec: 1.5 s
- replan_lateral_threshold_m: 0.25 m
- replan_heading_threshold_deg: 12 deg

주의:
- 이번 버전은 Stage-1 주행 안정화용이다.
- 책상/자판기와 벽을 서로 다른 종류의 cost로 분리하는 기능은 아직 넣지 않았다. 그 부분은 Stage-2 Clearance-aware/Structural-center 설계에서 처리한다.
