# V2.17.11 FULL TURBO X3 PRE-DRIVE

ArUco 검출/정렬부터 실제 결합 후 이송이 출발하는 순간까지 전체 준비 구간을 고속화.

변경:
- ArUco angular correction clamp: ±0.14 -> ±0.42 rad/s
- ArUco stable frames: 5 -> 2
- docking insert:
  0.300/0.250/0.200/0.150/0.100
  -> 0.900/0.750/0.600/0.450/0.300 m/s
- insertion yaw clamp: ±0.070 -> ±0.210 rad/s
- dual-AMR sync correction: 0.030 -> 0.090 m/s
- micro-creep: 0.025 -> 0.075 m/s
- micro-creep yaw clamp: ±0.05 -> ±0.15 rad/s
- micro-creep timeout: 4.0 -> 2.0 s
- cooperative cart lift settle: 1.40 -> 0.48 s
- attach result wait: 4.5 -> 1.6 s
- cooperative bridge max linear: 0.45 -> 0.72 m/s
  따라서 기존 /coop/cmd_vel 0.66m/s 요청이 실제로 clamp되지 않음.

유지:
- 실제 ArUco full pair 필수
- calibrated attach threshold 2.90m
- 3cm micro-creep x 최대 3
- no ALIGN fake snap
- always-on LiDAR
- RViz map-ready
