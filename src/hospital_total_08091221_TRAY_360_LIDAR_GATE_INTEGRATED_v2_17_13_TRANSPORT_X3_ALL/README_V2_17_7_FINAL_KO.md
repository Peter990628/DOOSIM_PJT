# V2.17.7 FINAL

최종 도킹:
- 명목 3.10m를 강제로 밀지 않음.
- 3.00m에서 멈춘 뒤 실제 cart capture 조건으로 ATTACH 시도.
- 실패하면 3cm씩 최대 3번 동기 micro-creep.
- 최대 이동거리 3.09m.
- ALIGN snap 없음.
- cart가 실제 distance/yaw capture 조건을 통과해야만 결합.

RViz:
- map_server ACTIVE 확인
- 실제 /map OccupancyGrid 수신 확인
- 그 다음 RViz 시작
- scenario 실패해도 RViz/map은 자동 종료하지 않음
