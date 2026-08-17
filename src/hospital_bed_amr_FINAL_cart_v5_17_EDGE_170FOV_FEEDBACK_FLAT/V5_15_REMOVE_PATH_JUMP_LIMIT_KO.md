# V5.15 - 재계획 경로 0.20m 연속성 제한 제거

V5.14에서 추가했던 `replan_max_path_jump_m=0.20` 및 `_limit_replan_path_jump()`을 완전히 제거했다.

유지:
- FollowPath 시작 후 2.0초 dynamic-replan grace period
- NEW full-scan obstacle 3회 연속 확인 후 재계획
- start_lock_m=0.15
- 코너 중앙화 최소 50%
- 0.25m tracking deviation 재계획
- explicit /spin pre-alignment
- ROS_DOMAIN_ID=120

의도:
- 재계획 시 Clearance Optimizer가 계산한 lateral shift를 0.20m로 잘라 이전 경로에 붙이는 부작용 제거
- 벽에서 충분히 떨어져야 하는 경우 +0.3~0.6m shift도 그대로 FollowPath에 전달
