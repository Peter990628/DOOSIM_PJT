# V2.14.1 BACKUP INITIAL STRAIGHT HOTFIX

백업 실행 전용 수정이다. 원본 자동도킹/Nav2/traffic 파일은 수정하지 않는다.

문제 원인:
- backup_precoupled_nav2.yaml의 `rotate_before_segment: true` 때문에 centerline_navigator가 첫 경로 조각 방향으로 출발 즉시 제자리 회전했다.
- start=(-22.69,11.03), final=(7.7732,6.329)은 직선 방위도 약 -8.8deg라 시작 yaw 0deg와 완전히 같지 않다.

수정:
1. 백업 설정에서만 `rotate_before_segment: false`.
2. Nav2 goal 전 `/coop/cmd_vel`에 `linear.x=0.20`, `angular.z=0`을 직접 보내 3.0m를 반드시 직진.
3. 3.0m 후 정지하고 기존 cooperative Nav2에 최종 목표를 전달.
4. 원본 RUN_TRAY / path_conflict_manager / cooperative_transport_manager는 변경하지 않음.

실행은 기존과 동일:
- Terminal1: `./RUN_BACKUP_1_ISAAC_PRECOUPLED.sh`
- Terminal2: `./RUN_BACKUP_2_RVIZ_NAV2_AUTO_GOAL.sh`
