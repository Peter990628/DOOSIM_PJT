# V5.14 시작 경로 튐 안정화

- 새 FollowPath 수락 후 2.0초 동안 conditional replan 금지
- NEW full-scan obstacle는 3회 연속 검출될 때만 cancel/replan
- 연속 재계획 경로의 공간 점프를 최대 0.20m로 제한(제한 pose가 안전할 때만)
- 기존 explicit /spin, corner centering >= 50%, tracking error 0.25m replan 유지
- ROS_DOMAIN_ID=120 유지
