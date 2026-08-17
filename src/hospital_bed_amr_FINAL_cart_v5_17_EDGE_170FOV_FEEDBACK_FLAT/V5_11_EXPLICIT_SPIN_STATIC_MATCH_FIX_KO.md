# V5.11 수정
- 출발 전 방향 정렬을 RotationShim에 맡기지 않고 `trolley_clearance_navigator`가 `/spin` Action을 직접 호출.
- 초록 경로 초기 진행방향과 trolley_base yaw 차이가 15도 이상이면 제자리 회전 완료 후 FollowPath 전송.
- `/spin` 성공 전에는 FollowPath를 보내지 않음.
- static map 벽 셀 주변 0.30 m를 map/LiDAR matching 영역으로 간주하여 벽 경계 오차를 NEW dynamic obstacle로 오인하지 않도록 수정.
- controller_server는 plain MPPI로 복원.
- ROS_DOMAIN_ID=120 유지.
