# V5.2 Explicit L/R Clearance Balance Demo

목적: 단순 Inflation 튜닝이 아니라, 트롤리 좌/우 여유를 직접 계산해 경로를 중앙 쪽으로 보정한다.

## 실제 계산
각 경로 pose에서 트롤리 진행방향 기준으로:
- dL, dR: 트롤리 좌/우 측면 끝에서 고비용 장애물까지 거리
- cL, cR: 좌/우 side corridor의 평균 inflation cost

후보 lateral shift s에 대해:

J = w_balance * |dL-dR|/range
  + w_min_clearance / min(dL,dR)
  + w_inflation_sum * (cL+cR)
  + w_inflation_balance * |cL-cR|
  + w_deviation * |s|
  + w_shift_smooth * |s-s_prev|

충돌 후보는 트롤리 2.36 x 1.90 m rectangle footprint로 제거한다.

## 실행
터미널1:
./03_run_isaac.sh

터미널2:
./42_run_nav2_trolley_clearance_balance.sh

확인:
./43_check_clearance_balance.sh

RViz에서 비교용 Path를 추가:
- /trolley/raw_plan      : State Lattice 원본
- /trolley/clearance_plan: 좌우 clearance 보정 결과

2D Goal Pose는 기존 /trolley/center_goal 인터페이스를 사용한다.

## 성공 판정
1. raw_plan보다 clearance_plan이 좌우 공간 균형이 좋아야 함
2. 터미널에 `LR imbalance A->B`에서 B가 A보다 작아야 함
3. `min clearance A->B`에서 B가 A보다 커지거나 최소한 유지되어야 함
4. FollowPath가 clearance_plan을 accepted 해야 함
