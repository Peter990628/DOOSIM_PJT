# V2.17.10 DOCKING 2X SPEED

V2.17.9의 안정된 one-shot ArUco / early physical attach 구조를 유지하면서
ArUco lock 이후 트레이 밑으로 진입하는 직선 도킹 속도만 약 2배로 올렸습니다.

이전:
0.150 / 0.125 / 0.100 / 0.075 / 0.055 m/s

V2.17.10:
0.300 / 0.250 / 0.200 / 0.150 / 0.100 m/s

유지:
- 실제 FULL PAIR ArUco lock
- 2.90m에서 physical ATTACH 시도
- 실패 시 +0.03m micro-creep x 최대 3회
- ALIGN snap 없음
- Always-on LiDAR
- map-ready RViz
- 결합 후 transport 0.66m/s request
