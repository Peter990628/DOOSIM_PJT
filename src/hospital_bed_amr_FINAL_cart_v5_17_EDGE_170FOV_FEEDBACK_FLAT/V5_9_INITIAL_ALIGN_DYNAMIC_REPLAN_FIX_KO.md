# V5.9 초기 방향 정렬 + 동적 재계획 수정

- FollowPath 앞에 Nav2 RotationShimController 추가
  - 초기 경로 방향 오차가 15도 이상이면 제자리 회전 후 MPPI로 넘김
  - 회전 속도 1.20 rad/s, 최대 각가속도 2.4 rad/s^2
- 조건부 재계획은 min-clearance / inflation 감소로 취소하지 않음
  - /map에 없는 NEW full-scan 장애물이 남은 경로와 겹칠 때만 재계획
- full /trolley/scan에서 trolley 자체에 가까운 self-return 제거
- scan point 중 /map의 기존 정적 벽에 해당하는 점은 동적 장애물에서 제외
- Clearance optimizer 실패 시, 새 동적 장애물과 충돌하지 않는 한 State-Lattice raw fallback 경로를 실제 주행에 허용
- ROS_DOMAIN_ID=120 유지
