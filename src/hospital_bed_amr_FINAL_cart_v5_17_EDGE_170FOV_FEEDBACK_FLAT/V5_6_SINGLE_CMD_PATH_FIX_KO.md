# V5.6 단일 속도 명령 경로 수정

- AMR1 Nav2 bridge의 `/cmd_vel` **구독만 비활성화**했습니다.
- AMR1의 `/odom`, `/scan`, `/tf`, `/clock` 발행은 그대로 유지합니다.
- 트롤리 자율주행 최종 명령 체인은 다음 하나만 사용합니다.
  `MPPI -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel -> trolley_cmd_vel_relay -> /trolley/cmd_vel -> Isaac`
- Isaac의 트롤리 제어 입력 토픽 `/trolley/cmd_vel`은 변경하지 않았습니다.
- `/trolley/center_goal`, `/trolley/raw_plan`, `/trolley/clearance_plan`, `/trolley/scan`, `/trolley/odom`도 변경하지 않았습니다.
