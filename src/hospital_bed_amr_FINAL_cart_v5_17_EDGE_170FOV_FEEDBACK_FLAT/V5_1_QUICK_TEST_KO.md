# V5.1 대형 트롤리 Clearance-biased State Lattice 빠른 테스트

## 목적
센터라인 없이 `map + LiDAR + 실제 trolley footprint`를 이용해 코너 진입 위치와 yaw가 포함된 SE(2) 경로를 생성하는지 확인한다.

이번 버전은 완성형 자체 planner가 아니라 **Chapter 2 영상 확보용 1차 구현**이다.

### 적용
- SmacPlannerLattice
- trolley polygon footprint: 약 2.36 m x 1.90 m
- Global costmap: `/trolley/scan_front`
- Local costmap: `/trolley/scan` 360도
- Global inflation radius: 1.45 m
- Global inflation cost scaling: 1.8
- State Lattice cost penalty: 3.2
- V4.x Heading Gate / PRETURN / Corner Detector: 사용 안 함
- Centerline: 사용 안 함

## 실행
터미널 1:
```bash
./03_run_isaac.sh
```

터미널 2:
```bash
./40_run_nav2_trolley_clearance_lattice.sh
```

터미널 3:
```bash
./41_check_clearance_lattice.sh
```

## 테스트 1 — 코너
RViz에서 기존에 문제가 있던 동일한 ㄱ자 코너 너머에 Goal을 찍는다.

확인할 것:
1. 보라색/Global Path가 벽에 바짝 붙는지
2. 코너 직전부터 yaw가 계획되어 있는지
3. 트롤리가 별도 PRETURN 상태기계 없이 코너에 진입하는지

## 테스트 2 — 장애물 추가
주행 전에 혹은 주행 중 앞쪽에 박스/장애물을 둔다.
Global costmap에 장애물이 잡힌 뒤 `/plan`이 우회 방향으로 바뀌는지 촬영한다.

## 실패 시 가장 먼저 볼 로그
`SmacPlannerLattice`, `lattice`, `primitive`, `No valid path`가 포함된 planner_server 로그를 그대로 복사한다.
