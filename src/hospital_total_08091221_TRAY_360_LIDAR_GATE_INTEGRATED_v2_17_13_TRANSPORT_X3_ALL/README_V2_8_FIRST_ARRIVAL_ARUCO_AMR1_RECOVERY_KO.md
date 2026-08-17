# V2.8 — 첫 도착 AMR 즉시 ArUco 도킹 + AMR1 출차/무진행 복구

## 수정 목적
V2.7 실기 테스트에서 두 문제가 연결되어 나타났다.

1. AMR2가 트레이 PRE_DOCK까지 먼저 도착해 `SUCCEEDED`가 되어도 AMR1이 아직 도착하지 않으면 ArUco 노드가 시작되지 않았다.
2. AMR1은 시작 도킹 위치에서 해제된 뒤 초기 이동이 불안정하거나, `ACTIVE:SEGMENT_*` 상태인데 실제 위치가 진행하지 않는 경우가 있었다.

V2.8은 기존 hospital_total_08091221의 Nav2, path_conflict_manager, 코너 회전, OCR, 엘리베이터 코드를 수정하지 않고 tray overlay만 수정한다.

## 핵심 변경 1 — 두 AMR를 기다리지 않는다
V2.7:

AMR1 PRE_DOCK + AMR2 PRE_DOCK 모두 완료 -> ArUco launch -> AMR1 도킹 -> AMR2 도킹

V2.8:

AMR1/AMR2 동시 PRE_DOCK 출발 -> 먼저 도착한 AMR이 즉시 ArUco launch/도킹 -> 다른 AMR은 기존 Nav2 계속 진행 -> 두 번째 AMR 도착 즉시 도킹 -> ATTACH

따라서 AMR2가 먼저 도착한 경우 예상 순서는 다음과 같다.

```text
[PRE_DOCK ARRIVAL V2.8] AMR2: Nav2 SUCCEEDED...
[TRAY STATE] ARUCO_GATE_START: first PRE_DOCK arrival -> start dual scanner immediately
[LATE LAUNCH] tray_aruco ...
[ARUCO START PASS V2.8] scanner/debug windows ready
[TRAY STATE] ARUCO_DOCKING_AMR2: ...
[ARUCO GATE DOCK] AMR2 ...
...
[POSE INSERT DOCKED] AMR2 ...
[ARUCO DOCKED V2.8] AMR2; peer state=...
```

이때 AMR1의 Nav2 프로세스와 기존 path_conflict_manager는 종료하거나 교체하지 않는다.

## 핵심 변경 2 — OpenCV ArUco Scanner 창
첫 PRE_DOCK 도착 시 `tray_dual_aruco.launch.py`가 실행되고 두 개의 창을 만든다.

- `DOOSIM AMR1 ARUCO SCANNER`
- `DOOSIM AMR2 ARUCO SCANNER`

표시 내용:

- 카메라 중앙선
- 인식된 ArUco 테두리와 ID
- AMR1: outer 40/41 + center 44
- AMR2: center 44 + outer 42/43
- 움직이는 scanner line
- `SCANNING...` / `GATE LOCKED`
- visible marker ID 목록

GUI 창을 만들 수 없는 OpenCV/디스플레이 환경에서는 노드가 죽지 않고 창만 비활성화하며 기존 debug image topic은 계속 발행한다.

```text
/amr1/tray_aruco/debug_image
/amr2/tray_aruco/debug_image
```

## 핵심 변경 3 — AMR1 Safe Egress
V2.7에서는 AMR2에만 초기 상대좌표 safe-egress가 있었다.
V2.8에서는 AMR1에도 같은 원리의 짧은 출차 동작을 넣었다.

- 기준점: 실행 순간 실제 AMR world pose
- AMR1 기본 escape: 0.16 m
- 성공 판정: 0.10 m 이상 진행
- 짧은 direct tray command 후 기존 Nav2에 즉시 제어권 반환
- timeout은 non-fatal

설정:

```json
cooperative_auto_transport.safe_egress_amr1
cooperative_auto_transport.safe_egress_amr2
```

## 핵심 변경 4 — AMR1/AMR2 No-Progress Watchdog
`ACTIVE:SEGMENT_*`는 직선 이동 상태다. 이 상태에서 실제 world pose가 8초 동안 0.06 m 미만으로 움직이고 traffic이 FREE/READY라면 같은 PRE_DOCK 목표를 다시 publish한다.

CenterlineNavigator는 새 목표를 받으면 현재 FollowPath를 취소하고 현재 위치 기준으로 다시 계획하므로, Nav2 자체 코드를 수정하지 않고 stuck/replan을 유도한다.

다음 상태에서는 watchdog을 절대 작동시키지 않는다.

- `ACTIVE:ROTATING_*` — 제자리 회전이므로 위치가 안 변하는 것이 정상
- `PAUSED:*` — traffic manager가 의도적으로 정지
- `ACTIVE:PLANNING*`
- traffic `YIELDING`, `CONFLICT_DETECTED`, `CLEARANCE_DELAY` 등

예상 로그:

```text
[NO-PROGRESS WATCHDOG V2.8] AMR1 stalled 8.0s in ACTIVE:SEGMENT_...; reissue PRE_DOCK goal 1/3
```

## V2.7 교착 방지 규칙 유지
V2.7에서 해결했던 규칙은 그대로 유지한다.

- `ACTIVE:ROTATING_FINAL`을 `tray_cmd_vel=0`으로 지속 덮어쓰지 않는다.
- traffic YIELDING 중 proximity fallback을 강제로 사용하지 않는다.
- 정상 handoff는 Nav2 `SUCCEEDED` 우선이다.
- tight fallback은 traffic FREE/READY일 때만 허용한다.

## 실행
### 최초 1회
```bash
cd ~/hospital_bed_amr_projects/hospital_total_08091221_TRAY_360_LIDAR_GATE_INTEGRATED_v2_8
chmod +x ./*.sh scripts/*.sh tray_overlay/scripts/*.py
./00_SETUP_TRAY_360_INTEGRATED.sh
```

### 터미널 1 — Isaac Sim
```bash
cd ~/hospital_bed_amr_projects/hospital_total_08091221_TRAY_360_LIDAR_GATE_INTEGRATED_v2_8
./RUN_TRAY_1_ISAAC_TOTAL_360.sh
```
Isaac Sim 창이 뜨면 PLAY.

### 터미널 2 — Nav2 + 자동 트레이 미션
```bash
cd ~/hospital_bed_amr_projects/hospital_total_08091221_TRAY_360_LIDAR_GATE_INTEGRATED_v2_8
./RUN_TRAY_2_AUTO_TOTAL_360.sh
```

## 첫 테스트에서 볼 로그
AMR1 초기:
```text
[TRAY STATE] SAFE_EGRESS_AMR1
[SAFE EGRESS AMR1] ...
[SAFE EGRESS PASS] AMR1 ...
```

AMR2가 먼저 트레이 앞에 도착하는 경우:
```text
[PRE_DOCK ARRIVAL V2.8] AMR2
[ARUCO START PASS V2.8]
[TRAY STATE] ARUCO_DOCKING_AMR2
```

AMR1이 주행 중 멈춘 경우:
```text
[NO-PROGRESS WATCHDOG V2.8] AMR1 ... reissue PRE_DOCK goal ...
```

둘 다 도킹:
```text
[PRE_DOCK+ARUCO COMPLETE V2.8]
[TRAY STATE] LIFT_AND_ATTACH
```

## 정적 검증 결과
패키징 전에 `tray_overlay/check_integration.py`와 Python AST/py_compile/bash syntax 검사를 수행했다.

- hospital_total_08091221 persistent baseline 211개 SHA-256 byte-identical: PASS
- V2.7 PRE_DOCK traffic safety 유지: PASS
- AMR1/AMR2 safe-egress: PASS
- first-arrival ArUco dock 구조: PASS
- no-progress replan watchdog: PASS
- OpenCV scanner/debug image: PASS

실제 Isaac Sim의 물리·카메라·Nav2 동작은 사용자 PC에서 PLAY 후 런타임 테스트가 필요하다.
