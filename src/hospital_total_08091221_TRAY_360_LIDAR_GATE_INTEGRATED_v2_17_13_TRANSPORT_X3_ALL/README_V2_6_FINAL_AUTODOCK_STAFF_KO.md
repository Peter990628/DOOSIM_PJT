# V2.6 FINAL — 자동 ArUco 결합 + 복도 협동이송 + 최종 의료진 배치

기준본: `hospital_total_08091221` 기능을 원본 그대로 보존하고 tray overlay만 확장한 최종 시연용 버전입니다.

## 이번 V2.6 핵심 수정

1. PRE_DOCK 완료 조건을 exact `SUCCEEDED`만 기다리지 않도록 수정했습니다.
   - 각 AMR이 PRE_DOCK 목표 0.30 m 이내 + yaw 18° 이내에 5 cycle 안정적으로 들어오면 즉시 ArUco 단계로 handoff합니다.
   - handoff된 AMR에는 fresh zero direct command를 계속 보내 stale Nav2 creep을 막습니다.

2. 트레이 전면 ArUco의 물리적 가시성 문제를 수정했습니다.
   - 전면 마커는 AMR이 기둥을 통과한 뒤 카메라 뒤로 가므로 최종 dock까지 계속 보일 수 없습니다.
   - V2.6은 `ArUco entrance gate -> pose insertion` 2단계 구조입니다.
   - 입구에서 40/41+44 또는 44+42/43으로 X/Yaw/중심을 맞춘 뒤, 마커 면을 통과하면 실제 cart/world pose로 트레이 중앙까지 계속 진입합니다.

3. 비홀로노믹 최종 정렬 recovery를 추가했습니다.
   - X는 맞았지만 Y/Yaw 오차가 남으면 약 0.22 m 후진합니다.
   - 다시 전진+회전으로 진입해 횡이동 없이 Y 오차를 줄입니다.
   - 최종 허용치: X 0.060 m, Y 0.040 m, yaw 5°, 6 cycle 안정.

4. 두 AMR가 dock되면 기존 자동 결합 체인을 그대로 실행합니다.
   - `ATTACH` 명령
   - 두 yellow lift 상승
   - AMR1 FixedJoint
   - AMR2 FixedJoint
   - cooperative bridge / cooperative Nav2
   - 최종 목적지 `(7.90, 10.13)`까지 복도 협동이송
   - 도착 후 tray 결합 유지

5. 최종 의료진 에셋 3개를 추가했습니다.
   - `doctor.glb`: MRI TableTop의 측면 축으로 1.35 m 떨어져 MRI 기계 옆에 배치. AMR의 MRI 종방향 동선과 겹치지 않도록 측면 배치.
   - `woman_doctor.glb`: 트레이 최종 목표에서 진행방향의 측면으로 1.95 m 떨어뜨려 배치. cart footprint와 겹치지 않게 함.
   - `nurse_surgical_rigged.glb`: 실제 desk/table cluster의 chair를 검색해서 의자 옆 0.48 m에 배치.
   - 세 인물은 모두 runtime visual-only / collision disabled / rigid body disabled입니다.

6. 카메라는 V2.5에서 복구한 기존 V2.3 방식 그대로입니다.
   - per-frame camera guard 없음.
   - 기존 top-down FollowCamera 동작 보존.

## 기대되는 자동 시나리오

`AMR1/AMR2 기존 Nav2 -> PathConflictManager yield/resume -> PRE_DOCK proximity handoff -> ArUco entrance alignment -> marker plane 통과 -> world-pose under-cart insertion -> 필요 시 back-out/re-entry -> AMR1/AMR2 docked -> dual lift -> dual FixedJoint -> cooperative Nav2 -> 복도 최종 목적지 -> 결합 유지`

## 주요 로그

정상 진행 시 아래 순서를 확인하세요.

```text
[PRE_DOCK PROXIMITY HANDOFF] AMR1 ...
[PRE_DOCK PROXIMITY HANDOFF] AMR2 ...
[PRE_DOCK COMPLETE] ... switching immediately to ArUco final docking
[ARUCO GATE DOCK] AMR1 ...
[ARUCO->POSE HANDOFF] AMR1 ...
[POSE INSERT DOCKED] AMR1 ...
[ARUCO GATE DOCK] AMR2 ...
[ARUCO->POSE HANDOFF] AMR2 ...
[POSE INSERT DOCKED] AMR2 ...
[CART] ATTACH request
[CARGO CART MAGNET] AMR1 CLACK ...
[CARGO CART MAGNET] AMR2 CLACK ...
[COOP GOAL] x=7.900 y=10.130 ...
[COOP COMPLETE] destination reached
```

정렬이 부족하면 다음과 같이 recovery가 보일 수 있습니다.

```text
[POSE RECOVERY] AMR1 attempt=1/5 ...
[POSE RECOVERY] AMR1 retreat complete ... re-insert
```

의료진은 Isaac 시작 후 다음 로그를 확인합니다.

```text
[FINAL STAFF] DoctorMRI anchor=MRI_SIDE:...
[FINAL STAFF] WomanDoctorTrayGoal anchor=TRAY_GOAL_SIDE ...
[FINAL STAFF] NurseDesk anchor=CHAIR_SIDE:...
[FINAL STAFF READY] ... all non-physical
```

## 검증 범위

정적 검증에서 다음을 확인했습니다.
- hospital_total 기준 persistent 213 files SHA-256 동일
- Python AST 통과
- 모든 shell `bash -n` 통과
- JSON parse 통과
- 360° LiDAR overlay 유지
- AMR1/AMR2 traffic pause / AMR2 namespace / 2F 0.35 m / stale goal / 코너 저속회전 보존
- 전면 5-marker gate 보존
- 최종 의료진 GLB 3개 포함

실제 Isaac Sim PhysX + ROS2/Nav2 주행 성공 여부는 대상 PC에서 런타임 확인이 필요합니다.
