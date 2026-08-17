# V2.9 단일 ArUco ID 자동 도킹

## 수정 이유
V2.8은 각 AMR이 PRE_DOCK에 도착한 뒤에도 완전한 2-post gate가 필요했다.
- AMR1: LEFT OUTER ID40/41 + CENTER ID44
- AMR2: CENTER ID44 + RIGHT OUTER ID42/43

따라서 카메라 화각에서 ID44만 선명하게 보여도 `WAIT GATE`에서 정지할 수 있었다.

## V2.9 동작
1. 각 AMR은 기존과 동일하게 자기 PRE_DOCK까지 Nav2로 이동한다.
2. 완전한 outer+center pair가 보이면 기존 V2.8 pair 정렬을 우선 사용한다.
3. pair가 안 보여도 **자기에게 허용된 ArUco ID 하나가 3프레임 연속 검출되면 즉시 LOCK**한다.
   - AMR1: ID44 또는 ID40/41
   - AMR2: ID44 또는 ID42/43
4. AMR 번호 자체가 좌/우 베이를 결정하므로, 단일 ID는 트레이 입구 인증 신호로 사용한다.
5. LOCK 직후 기존의 cart-local/world-pose 삽입 제어로 자동 전진/회전하여 최종 도킹 위치까지 들어간다.
6. 최종 pose tolerance가 안정적으로 만족되면 기존 Lift + FixedJoint 결합으로 넘어간다.

## 스캐너 표시
- pair 검출: `PAIR LOCKED`
- 단일 ID 검출: `SINGLE ID 44 DETECTED`, 상단 `SINGLE ID READY [44] -> AUTO DOCK`
- 미검출: `SCANNING...`

## 예상 핵심 로그
```text
[ARUCO SINGLE-ID LOCK V2.9] AMR2: ID44 stable 3/3 -> RIGHT bay -> automatic pose insertion
[POSE INSERT DOCKED] AMR2: ... ready for dual lift/FixedJoint
```

## 보존 사항
V2.8의 first-arrival 독립 ArUco launch/docking, AMR1/AMR2 safe-egress, PRE_DOCK traffic deadlock 수정, no-progress watchdog, dual lift/FixedJoint, cooperative Nav2는 그대로 유지한다.
