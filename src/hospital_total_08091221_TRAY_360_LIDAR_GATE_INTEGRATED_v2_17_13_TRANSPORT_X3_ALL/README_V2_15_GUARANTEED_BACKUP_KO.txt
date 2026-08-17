V2.15 GUARANTEED BACKUP 핵심

현재 V2.14.2 정지 원인:
- RUN_BACKUP_2_RVIZ_NAV2_SAFE_GOAL.sh가 map_probe 다음에
  /coopnav/initial_pose_locked bool_probe를 기다린다.
- 실제 로그는 [MAP READY] 이후 coop bridge/LiDAR만 출력되고
  [BACKUP SAFE] INITIAL STRAIGHT가 나오지 않았다.
- 즉 /coop/cmd_vel이 0인 이유는 주행 명령 전에 pose-lock gate에서 막힌 것이다.

V2.15 해결:
1) pose_lock_localizer를 백업 실행에서 제거.
2) known START (-22.69,11.03,0deg)를 static map->coop_odom으로 사용.
3) Nav2 planner 대신 controller_server FollowPath에 검증된 rounded-L 경로 직접 전달.
4) live obstacle layer / traffic / ArUco는 이 백업 실행에서 command owner가 아님.
5) Nav2가 10초 안에 실제로 움직이지 않으면 direct odometry follower 자동 전환.
6) direct follower도 실행 불가하면 마지막 time-only /coop/cmd_vel 경로 자동 전환.
7) RViz와 wheel telemetry는 계속 유지.

경로:
START (-22.69,11.03)
 -> 직진
 -> 1.0m 반경 우회전 arc
 -> GOAL (7.7732,6.329)

실행:
Terminal1
  ./RUN_BACKUP_1_ISAAC_PRECOUPLED.sh
Terminal2
  ./RUN_BACKUP_2_GUARANTEED_V2_15.sh
