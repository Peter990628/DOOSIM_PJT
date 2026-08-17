# V4.6 테스트 노트

이번 버전은 두 가지를 동시에 분리해서 검증한다.

1. **누적 코너 검출**
   - 직선 주행 중 `dominant-heading recovery`로 멈췄다 가는 동작을 제거했다.
   - 여러 작은 경로 각도 변화가 누적되어 실제 코너가 되는 경우를 검출한다.
   - 코너 시작점 약 0.8 m 전에 `STOP -> ROTATE -> DRIVE`를 수행한다.
   - 회전 최소 각속도 0.12 rad/s와 consumed-corner 보호는 유지한다.

2. **Global planning용 전방 LiDAR 분리**
   - `/trolley/scan`: 360도 전체 스캔. Local Costmap은 그대로 사용한다.
   - `/trolley/scan_front`: 전방 160도만 남긴 스캔. Global Costmap의 VoxelLayer만 사용한다.
   - ROS LaserScan 기준 0도가 전방이므로 -80도~+80도를 사용한다. 사용자가 말한 전방 10~170도 범위와 같은 160도 전방 부채꼴 의미다.
   - 뒤쪽 책상/자판기/벽의 동적 LiDAR marking이 Smac global path를 밀어내는 영향을 줄이는 실험이다.

주의: Static Layer에 원래 들어 있는 고정 벽/구조물은 여전히 Global Costmap에 남는다. 이번 front scan은 LiDAR 기반 VoxelLayer 입력만 제한한다.
