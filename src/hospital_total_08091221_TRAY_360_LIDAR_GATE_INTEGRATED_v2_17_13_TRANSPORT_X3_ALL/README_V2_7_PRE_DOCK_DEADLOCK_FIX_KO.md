# V2.7 PRE_DOCK 교착상태 수정

실제 런타임에서 다음 상태가 확인되었습니다.

- AMR2: `ACTIVE:ROTATING_FINAL`, PRE_DOCK 약 0.19 m, yaw 약 16 deg
- AMR1: `PAUSED:TRAFFIC`, 약 28 m 남음
- traffic: `YIELDING`, winner=AMR2, loser=AMR1
- 기존 V2.6.x proximity handoff 카운터는 이미 임계치를 넘었고 AMR2에 `/amr2/tray_cmd_vel=0`을 계속 발행

Isaac runtime에서는 신선한 `tray_cmd_vel`이 Nav2 `cmd_vel`보다 우선하므로, AMR2가 최종 회전을 끝내지 못했습니다. 따라서 AMR2가 `SUCCEEDED`가 되지 않았고, `path_conflict_manager`는 AMR1의 traffic pause를 해제하지 못했습니다. ArUco 단계는 두 AMR PRE_DOCK 완료 후 시작하므로 도킹도 시작하지 않았습니다.

## V2.7 변경

1. 정상 PRE_DOCK 완료는 각 AMR의 기존 Nav2 `SUCCEEDED`를 우선합니다.
2. `YIELDING`, `CONFLICT_DETECTED`, `CLEARANCE_DELAY` 등 traffic 세션 중에는 proximity fallback을 금지합니다.
3. `ACTIVE:ROTATING_FINAL` 상태를 direct zero로 덮지 않습니다.
4. 한 AMR만 proximity handoff한 뒤 계속 `tray_cmd_vel=0`을 보내던 로직을 제거했습니다.
5. 물리적 fallback은 `traffic=FREE/READY`일 때만 허용합니다.
6. fallback 조건은 0.22 m 이하, yaw 3 deg 이하, 5 cycle이며, **두 AMR가 동시에 준비되었거나 다른 AMR가 이미 Nav2 SUCCEEDED인 경우에만** 적용됩니다.
7. 로그는 `handoff=1417/5`처럼 무한 증가하지 않고 `WAIT_TRAFFIC_YIELDING`, `DONE_NAV2`, `TIGHT_PAIR=n/5`로 표시됩니다.

## 기대 동작

AMR2 winner -> AMR1 pause -> AMR2 최종회전 완료 -> AMR2 SUCCEEDED -> path conflict release delay -> AMR1 resume -> AMR1 PRE_DOCK SUCCEEDED -> ArUco 40/41+44, 44+42/43 -> pose insertion/recovery -> dual lift + FixedJoint -> cooperative corridor transport.

기존 `hospital_total_08091221`의 Nav2, traffic manager, 코너 회전, stale goal, elevator, OCR 기능은 수정하지 않습니다.
