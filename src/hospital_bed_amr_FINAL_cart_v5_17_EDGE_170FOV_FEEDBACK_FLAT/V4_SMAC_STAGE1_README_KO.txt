V4 STAGE-1 — Smac2D + Inflation Cost baseline (TEST PENDING)
============================================================
목표
- 기존 하드코딩 centerline 생성기를 사용하지 않는다.
- RViz의 /trolley/center_goal 인터페이스는 유지한다.
- goal_forwarder가 해당 Goal을 Nav2 NavigateToPose에 그대로 전달한다.
- Global Planner는 Navfn 대신 SmacPlanner2D를 사용한다.
- Smac가 기존 Global Costmap의 Inflation Cost를 경로 비용으로 활용한다.

핵심 변경
1) 새 config
   ros2_ws/src/hospital_nav2/config/nav2_params_trolley_smac.yaml
   - plugin: nav2_smac_planner/SmacPlanner2D
   - cost_travel_multiplier: 2.0
   - V3의 Global Inflation 값은 일단 그대로 유지:
       inflation_radius: 0.30
       cost_scaling_factor: 2.2

2) 새 Goal forwarder
   ros2_ws/src/hospital_nav2/hospital_nav2/trolley_goal_forwarder.py
   /trolley/center_goal -> /navigate_to_pose Action
   ※ 경로 생성/센터라인 계산을 하지 않음.

3) 새 launch
   hospital_trolley_smac_navigation.launch.py
   - centerline_navigator_trolley 미실행
   - trolley_goal_forwarder 실행

4) 새 RViz
   trolley_smac_navigation.rviz
   - 기존 /trolley/center_goal 도구 유지
   - 경로 표시는 /trolley/centerline_path 대신 /plan

실행 예정 순서
터미널 1: ./03_run_isaac.sh
터미널 2: ./31_run_nav2_trolley_smac.sh
터미널 3: ./32_check_trolley_smac.sh

중요
- 현재는 이론 기반으로 미리 만든 TEST-PENDING 버전이다.
- 첫 테스트에서는 기존 inflation 값을 그대로 두어 "Planner만 Navfn -> Smac" 효과를 분리해서 본다.
- 중앙 성향이 약하면 다음 실험에서 Global Costmap inflation_radius / cost_scaling_factor를 넓고 완만한 potential field로 조정한다.
- Custom Center-Bias Optimizer와 MPPI는 아직 넣지 않았다. 이것이 Stage-1 baseline이다.
