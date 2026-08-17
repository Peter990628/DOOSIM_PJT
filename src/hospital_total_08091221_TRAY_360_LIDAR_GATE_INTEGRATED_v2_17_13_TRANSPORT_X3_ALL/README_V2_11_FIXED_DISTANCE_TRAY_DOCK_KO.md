# V2.11 — 침대 도킹 방식의 트레이 고정거리 자동도킹

## 왜 바꿨나
V2.9/V2.10의 트레이 최종 진입은 ArUco 이후 `POSE_INSERT`에서 트레이 world pose를 계속 따라가며 전진/회전/후퇴 recovery를 수행했다. 실제 로그에서는 PRE_DOCK 경로가 traffic manager에 남아 도킹 중 다시 `TRAFFIC_PAUSE`가 걸리는 현상과, 좁은 트레이 안에서 pose steering이 오래 반복되는 문제가 있었다.

## V2.11 동작
1. 두 AMR은 V2.10 convoy 방식으로 함께 PRE_DOCK까지 이동한다.
2. 먼저 도착한 AMR은 상대 AMR을 기다리지 않는다.
3. 자기에게 허용된 ArUco ID 하나(ID44 또는 자기 outer ID)를 3프레임 연속 확인한다.
4. 그 순간 Nav2 traffic 대상에서 제외하고 기존 PRE_DOCK path를 폐기한다. 상대 AMR Nav2는 멈추지 않는다.
5. 침대 도킹 `FORWARD_TARGET`과 같은 방식으로 시작 위치를 저장한다.
6. 이후 마커가 안 보여도 조향 없이 직진하며 실제 이동거리만 측정한다.
7. 현재 트레이 길이 2.20m, PRE_DOCK standoff 0.95m, 최종 dock_x=0 기준 자동 계산 거리는 `1.10 + 0.95 = 2.05m`이다.
8. 2.05m 완료 즉시 정지한다. 두 대 모두 완료되면 기존 dual lift + FixedJoint ATTACH를 수행한다.

## 핵심 로그
```text
[ARUCO ID LOCK V2.11] AMR2: ID44 stable 3/3 -> STRAIGHT 2.050m
[FIXED INSERT V2.11] AMR2: moved=.../2.050m
[FIXED DISTANCE DOCKED V2.11] AMR2: moved=2.05... ready for lift/FixedJoint
```

Traffic manager에서는 도킹 동안 다음처럼 보여야 한다.
```text
TRAY_DOCK_BYPASS ... peer Nav2 remains free
```
`POSE_INSERT`, `POSE RECOVERY`, `POSE INSERT DOCKED`는 V2.11에서 사용하지 않는다.
