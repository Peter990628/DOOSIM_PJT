# V5 Smac State Lattice baseline test

목적은 **코너 상태기계 없이 Planner 자체가 트롤리의 x/y/yaw와 실제 polygon footprint를 고려했을 때 코너 진입 경로가 좋아지는지** 확인하는 것입니다.

## 유지
- V4.6 전방 LiDAR 필터: global costmap = `/trolley/scan_front`
- local costmap = `/trolley/scan` 360°
- 트롤리 polygon footprint = ±1.18 m x ±0.95 m
- 기존 DWB controller
- `/trolley/center_goal` -> NavigateToPose 인터페이스

## 제거 / 비활성
- `trolley_heading_gate`
- PRETURN / dominant segment / cumulative corner detection
- 강제 STOP -> ROTATE 상태기계
- centerline

## 변경
- `SmacPlanner2D` -> `SmacPlannerLattice`
- planner가 SE(2) 상태(x, y, yaw)와 polygon footprint collision checking을 사용하도록 변경
- V5 launch가 설치된 `nav2_smac_planner`에서 5 cm differential lattice JSON을 자동 탐색함. 없으면 Nav2 기본 lattice를 사용함.

## 테스트 순서
터미널 1:
```bash
./03_run_isaac.sh
```

터미널 2:
```bash
./38_run_nav2_trolley_lattice.sh
```

터미널 3 (Goal 찍기 전):
```bash
./39_check_trolley_lattice.sh
```

그 다음 RViz에서 **이전과 동일한 코너를 통과하는 Goal**을 찍습니다.

## 이번 테스트에서 볼 것
1. 최초 `/plan`이 코너 안쪽 벽에 바짝 붙는지
2. 코너 진입 전에 yaw가 자연스럽게 바뀌는 경로가 생성되는지
3. 트롤리 footprint가 회전할 공간을 확보하는지
4. V4.6처럼 gate가 경로를 뒤에서 재해석하지 않아도 통과하는지
5. 뒤쪽 자판기/책상 영향 감소가 유지되는지

이번 버전은 아직 자체 clearance planner나 조건부 재계획 알고리즘을 넣지 않았습니다. State Lattice 자체의 효과를 먼저 분리해서 보는 baseline입니다.


V5.1에서는 40_run_nav2_trolley_clearance_lattice.sh 를 우선 사용하세요.
