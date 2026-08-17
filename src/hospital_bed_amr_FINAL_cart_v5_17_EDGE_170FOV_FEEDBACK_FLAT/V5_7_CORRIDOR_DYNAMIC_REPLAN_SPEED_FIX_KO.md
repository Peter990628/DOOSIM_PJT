# V5.7 변경사항

- 정적 벽/고정 장애물: 전체 Global Costmap(Static Map 포함)으로 시야 밖 구간도 clearance 계산
- 동적 장애물: full `/trolley/scan`으로 현재 경로 영향을 감지하고 조건부 재계획
- 초록 경로 미표시 방지: optimizer 실패 시 raw State-Lattice를 `/trolley/clearance_plan`에 fallback publish
- 넓은 공간: raw 최단경로 이탈 penalty 및 추가 path-length penalty 강화
- 좁은 직선 복도: 좌우 clearance 중앙화 강화
- 코너: 원본 State-Lattice yaw 변화가 큰 구간에서 중앙화 약화/비활성화
- lateral shift 연속성 강화: max shift step 0.25 m
- 속도 상향: MPPI vx_max 0.65 m/s, wz_max 1.20 rad/s; velocity smoother도 동일 한계
- 기존 외부 토픽 유지: `/trolley/center_goal`, `/trolley/raw_plan`, `/trolley/clearance_plan`, `/trolley/cmd_vel`
