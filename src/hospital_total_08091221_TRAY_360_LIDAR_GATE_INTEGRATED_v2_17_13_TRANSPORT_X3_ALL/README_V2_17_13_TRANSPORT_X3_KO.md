# V2.17.13 TRANSPORT X3 ALL

V2.17.12의 실제 3배속 도킹을 유지하면서, 결합 이후 트레이 이송 구간도 전부 3배속화.

## 실제 이송 속도
이전:
- FAST 0.66 m/s
- MID 0.24 m/s
- TIGHT 0.10 m/s
- recovery reverse -0.08 m/s

V2.17.13:
- FAST 1.98 m/s
- MID 0.72 m/s
- TIGHT 0.30 m/s
- recovery reverse -0.24 m/s

## 회전
- transport W gain: 2.70 -> 8.10
- 실제 cooperative angular clamp: 0.35 -> 1.05 rad/s
- cart 내부 angular clamp: 0.50 -> 1.50 rad/s

## 모든 clamp 제거
- CooperativeNav2Bridge linear: 0.72 -> 2.16 m/s
- Cart commands_from_twist linear: 0.72 -> 2.16 m/s
- Root controller max linear: 2.40 m/s 유지 (1.98 요청보다 큼)

## 수동 cooperative 움직임도 3배
- linear 0.42 -> 1.26 m/s
- angular 0.42 -> 1.26 rad/s

도킹 V2.17.12:
- direct tray_cmd_vel
- physical acceleration x3
- ArUco / LiDAR / RViz
모두 그대로 유지.
