# V5.13 Clearance 경로 추종 강화

핵심 변경:
- start_lock_m: 1.00 -> 0.15 m
- 급한 코너에서도 clearance centering 최소 50% 유지
- clearance 목적함수 강화: balance 16, min-clearance 8, inflation-balance 10
- raw path 고착 완화: deviation 1.0, extra-path-length 5.0
- 실제 trolley 중심이 green path에서 0.25 m 초과 이탈하면 current pose 기준 재계획
- 이탈 판정은 2회 연속 확인 후 동작, replan cooldown 1.5 s 유지
- NEW LiDAR obstacle가 남은 경로를 침범할 때 재계획하는 기존 정책 유지
- explicit /spin 출발 전 제자리 정렬 유지
- ROS_DOMAIN_ID=120 유지

정적 검증 대상:
1. Python syntax / launch syntax
2. declared parameter와 launch override 일치
3. min-clearance 자체는 replan trigger가 아님
4. corner factor가 0으로 내려가지 않음
5. current-pose 재계획은 ComputePathToPose use_start=False를 사용
