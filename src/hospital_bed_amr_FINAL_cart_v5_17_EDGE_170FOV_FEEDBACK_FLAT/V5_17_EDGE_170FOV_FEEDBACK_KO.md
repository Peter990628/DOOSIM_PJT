# V5.17 Footprint Edge + Forward 170deg Feedback

- 경로 내부 인접 lateral shift 최대 변화: 0.15 m
- 기존 second-difference shift smoothing 유지
- 현재 트롤리 clearance: 4점이 아니라 앞/뒤/좌/우 **전체 edge를 다점 샘플링**
- 예측 clearance: trolley_base 기준 전방 170도(-85~+85)를 5개 sector로 평가
  - left-side / front-left / front / front-right / right-side
- sector 거리값은 단일 최소값 대신 10 percentile을 사용해 1-point LiDAR noise 억제
- LiDAR 거리값은 센터가 아니라 직사각형 footprint 경계에서의 여유거리로 환산
- 정상 코너 벽 하나만 보고 재계획하지 않도록: forward sector 위험 + near-body edge 위험이 함께 있거나, static left/right edge imbalance가 지속될 때만 wall replan
- wall feedback cadence 0.5 s x 3 = 약 1.5 s, replan cooldown 1.5 s 유지
- explicit /spin, dynamic obstacle 3x confirmation, path deviation replan, Domain 120 유지
