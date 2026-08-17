# V2.17.12 ACTUAL X3 DIRECT DRIVE

V2.17.11에서 속도 상수는 올라갔지만 실제 체감속도가 거의 변하지 않았던 핵심 원인:
1. 도킹 명령이 일반 /cmd_vel 경로를 사용.
2. Isaac 물리 본체의 linear_accel_mps2가 2.0 그대로.
3. command 값만 커지고 실제 rigid-body 속도 상승은 기존 가속도에 의해 완화됨.

V2.17.12:
- AMR1: /amr1/tray_cmd_vel
- AMR2: /amr2/tray_cmd_vel
  전용 Isaac tray-direct 채널 사용. 이 채널은 main runtime에서 Nav2보다 우선순위가 높음.
- physical acceleration:
  linear 2.0 -> 6.0 m/s²
  lateral 6.0 -> 18.0 m/s²
  angular 10.0 -> 30.0 rad/s²
- docking commands:
  0.90 / 0.75 / 0.60 / 0.45 / 0.30 m/s
- ArUco correction clamp +/-0.42 rad/s 유지
- micro creep 0.075m/s 유지
- lift settle 0.48s 유지
- stop packet delay 약 0.30s -> 0.05s
- cart command burst 약 0.25s -> 0.06s
- scanner warmup 2.0s -> 0.7s

실제 확인:
output/v217_true_aruco/motion_signal.log 에
[DIRECT COMMAND] 과 [ACTUAL SPEED]가 동시에 출력됨.
