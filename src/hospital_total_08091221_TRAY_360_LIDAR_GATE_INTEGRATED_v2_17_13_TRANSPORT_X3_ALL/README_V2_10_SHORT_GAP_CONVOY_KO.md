# V2.10 — 같은 방향 2-AMR 짧은 간격 Convoy 주행

## 왜 수정했나
V2.9까지 path_conflict_manager는 두 AMR의 미래 centerline이 길게 겹치면 같은 방향 주행도 하나의 배타적 충돌구간으로 처리했다.
그 결과 AMR2가 winner가 되면 AMR1이 YIELDING에서 오래 기다리고, 경우에 따라 AMR2가 PRE_DOCK에 거의 도착한 뒤에야 AMR1이 재출발했다.

## V2.10 정책
- 반대방향/교차 충돌: 기존 우선권 회피 유지
- 같은 방향 + 실제 공통 centerline: `FOLLOWING_RUN` convoy 모드
- 두 AMR 모두 동시에 이동
- center-to-center 간격이 2.50m 이하가 되면 뒤 AMR만 잠깐 pause
- 간격이 3.20m 이상으로 회복되면 즉시 resume
- 앞 AMR은 정지시키지 않는다
- 기존 V2.9 first-arrival ArUco + single-ID auto dock + safe egress + no-progress watchdog 보존

## 기대 로그
```text
{"state":"FOLLOWING_RUN", ... "separation_m":3.45, "leader":"amr2", "follower":"amr1"}
```
간격이 가까워지면:
```text
{"state":"FOLLOWING_HOLD", ... "separation_m":2.42}
```
다시 벌어지면:
```text
{"state":"FOLLOWING_RUN", ... "separation_m":3.22}
```

즉 AMR1은 AMR2가 트레이에 도착할 때까지 기다리는 방식이 아니다.
