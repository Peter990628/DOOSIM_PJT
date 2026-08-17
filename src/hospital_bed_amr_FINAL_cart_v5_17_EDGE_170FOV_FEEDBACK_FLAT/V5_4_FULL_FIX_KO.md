# V5.4 FULL FIX — Clearance-DP + MPPI

## 목표
센터라인 없이 대형 트롤리(2.36 x 1.90 m)가 좌/우 여유와 Inflation을 직접 비교해 가능한 한 중앙의 안전 경로를 만들고, 경로의 SE(2) 자세를 보존한 채 추종한다.

## V5.3 대비 핵심 수정
1. **State Lattice yaw 보존**: raw `(x,y,yaw)`를 유지하며 제자리 회전 샘플도 삭제하지 않는다.
2. **전체 측면 Clearance**: 트롤리 좌/우 측면을 코너 포함 9개 종방향 지점에서 검사한다.
3. **Inflation = soft cost**: lethal/unknown만 충돌, Inflation은 L/R 비용 및 균형 계산에 사용한다.
4. **360° LiDAR 직접 사용**: `/trolley/scan`을 map 좌표 장애물 점으로 변환해 좌/우 동적 장애물과 footprint 충돌에도 사용한다.
5. **Dynamic Programming**: 각 점을 greedy하게 결정하지 않고 전체 shift sequence 비용을 최소화한다.
6. **Swept Footprint**: 연속 pose 사이를 6 cm / 4° 이하 간격으로 보간해 전체 트롤리 footprint 충돌을 검사한다.
7. **Dense Footprint**: 6 cm 간격으로 몸체 전체를 검사한다.
8. **Endpoint 정책 수정**: 시작 0.25 m만 완만히 고정하고 최종 goal pose만 hard-lock 한다.
9. **조건부 재계획**: 남은 경로에 새 lethal/scan 장애물이 생기거나 clearance가 크게 악화될 때만 재계획한다.
10. **Global Costmap publish 4 Hz**: custom optimizer가 최신 costmap을 빠르게 받도록 변경.
11. **MPPI Controller**: DWB 대신 DiffDrive MPPI + footprint-aware CostCritic으로 clearance path를 추종한다.
12. **Velocity Smoother 실제 연결**: `/trolley/cmd_vel_nav -> velocity_smoother -> /trolley/cmd_vel`.
13. **Lattice primitive pinning**: 실행 시작 시 differential-drive JSON 1개를 결정해 launch에 고정한다.
14. **Stale build 제거**: 실행 스크립트가 `hospital_nav2`의 이전 build/install cache를 지우고 새로 빌드한다.

## 경로 토픽
- 빨강: `/trolley/raw_plan`
- 초록: `/trolley/clearance_plan`

RViz는 두 경로가 시작부터 자동 표시되며 Local Plan은 기본 숨김 상태다.

## 실행
```bash
./03_run_isaac.sh
```
다른 터미널:
```bash
./42_run_nav2_trolley_clearance_balance.sh
```
확인:
```bash
./43_check_clearance_balance.sh
```

## 성공 로그
Goal 입력 후 다음과 비슷한 로그가 나와야 한다.
```text
DPCLR[...]
CLEARANCE RESULT | ... | DP+swept=OK
FollowPath accepted CLEARANCE-DP path.
```
새 장애물이 남은 경로를 위험하게 만들면:
```text
CONDITIONAL REPLAN: ...
```
가 출력된다.
