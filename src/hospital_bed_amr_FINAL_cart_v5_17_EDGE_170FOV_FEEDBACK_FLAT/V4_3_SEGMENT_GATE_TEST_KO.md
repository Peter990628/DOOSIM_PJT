# V4.3 Segment Heading Gate 테스트

목적: V4.2의 robot->lookahead chord 각도 문제를 제거한다.

- 목표 yaw는 로봇에서 먼 path point를 직접 바라보는 각도가 아니라 **Global Path 자체의 로컬 tangent**에서 계산한다.
- tangent 오차가 3도 이상이면 `linear.x=0`으로 강제하고 제자리 회전한다.
- 1도 이내로 정렬되면 전진을 허용한다.
- 전진 중에는 DWB의 큰 `angular.z`를 그대로 통과시키지 않고, 로컬 tangent 유지에 필요한 작은 보정(`max 0.08 rad/s`)만 허용한다.
- 따라서 코너 너머 점을 직선으로 바라보며 대각선으로 코너를 자르는 현상을 줄이는 것이 이번 테스트의 목적이다.

실행:
1. `chmod +x *.sh`
2. `./03_run_isaac.sh`
3. 다른 터미널에서 `./31_run_nav2_trolley_smac.sh`
4. 확인: `./34_check_segment_heading_gate.sh`
5. 실시간 최종 명령: `ros2 topic echo /trolley/cmd_vel`

주의: 이것은 아직 최종 코너 검출 상태기계가 아니라, V4.2의 잘못된 heading 기준을 수정하는 디버깅 버전이다.
