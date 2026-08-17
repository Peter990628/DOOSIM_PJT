# V5.5 사용자 결정 반영

- 새 LiDAR/Costmap 장애물이 남은 swept path와 충돌하면 즉시 재계획.
- Goal 최종 yaw는 반드시 보존. 위치 도달 후 필요하면 제자리 회전으로 맞춤.
- 넓은 공간에서는 State Lattice 원본(최단거리) 유지 성향.
- 좁은 복도/벽 근접 구간에서는 좌우 clearance 균형을 강하게 적용.
- Nav2 내부 velocity smoother 체인은 건드리지 않고 최종 /cmd_vel만 /trolley/cmd_vel로 relay.
- 기존 외부 API /trolley/center_goal, /trolley/scan, /trolley/odom, /trolley/cmd_vel 유지.
- LaserScan을 scan timestamp의 TF로 map 변환.
- global costmap update/publish 4 Hz.
