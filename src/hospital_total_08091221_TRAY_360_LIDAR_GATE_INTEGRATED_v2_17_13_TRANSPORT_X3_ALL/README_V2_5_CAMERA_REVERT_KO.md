# V2.5 — Follow Camera 원복 + 전면 ArUco 유지

## 이번 변경
- V2.4의 rear-chase camera(-3.2m, +2.1m)를 제거했습니다.
- `follow_camera_guard.py` 및 매 프레임 transform 재기록을 완전히 제거했습니다.
- Follow Camera를 V2.3/original 설정으로 정확히 복원했습니다.
  - body: `/World/AMR1/base_link`
  - position_local_m: `[0.0, 0.0, 6.0]`
  - target_local_m: `[0.0, 0.0, 0.35]`
  - up_local: `[1.0, 0.0, 0.0]`
- V2.4에서 수정한 트레이 전면 ArUco 위치는 유지합니다.
- 360 LiDAR, AMR2 map lifecycle recovery, storage repair/device unmount fix, hospital_total 최신 기능은 유지합니다.

## 카메라 흔들림 원인 분석
코드상 FollowCamera는 `/World/AMR1/base_link` 바로 아래 child입니다. 따라서 AMR1의 실제 PhysX rigid-body transform을 필터 없이 그대로 상속합니다. 바퀴 접촉, 코너 회전, 충돌/관성 때문에 base_link에 작은 Z/pitch/roll/yaw 변화가 생기면 카메라도 같은 순간 변화를 받습니다. 별도 smoothing/interpolation이 없기 때문에 viewport에서는 이것이 흔들림으로 보일 수 있습니다.

V2.4에서는 이 문제를 막으려고 매 프레임 카메라 local transform을 다시 `Set()`했는데, 이 방식은 오히려 PhysX가 부모 transform을 갱신하는 루프와 USD camera transform 재-authoring을 동시에 수행하게 되어 화면 떨림/점프를 악화시킬 수 있습니다. 그래서 V2.5에서는 그 guard를 완전히 제거했습니다.

또한 기존 카메라는 AMR 위 6m의 top-down 구조라 AMR의 작은 각도 변화도 화면 전체 회전으로 바로 보입니다. 특히 Nav2가 코너에서 yaw를 자주 보정하면 카메라가 회전하는 느낌이 커질 수 있습니다. 이것은 현재 원복 버전의 구조적 특성이며 이번 버전에서는 의도적으로 수정하지 않았습니다.

## 실행
`./00_SETUP_TRAY_360_INTEGRATED.sh` 완료 후 터미널1에서 `./RUN_TRAY_1_ISAAC_TOTAL_360.sh`, Isaac PLAY 후 터미널2에서 `./RUN_TRAY_2_AUTO_TOTAL_360.sh`.
