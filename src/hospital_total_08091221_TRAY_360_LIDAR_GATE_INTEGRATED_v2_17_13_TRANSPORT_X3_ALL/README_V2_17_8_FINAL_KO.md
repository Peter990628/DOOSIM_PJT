# V2.17.8 NO TIMEOUT + ALWAYS LIDAR

## 삽입 실패 원인
V2.17.7 로그에서 AMR는 lat≈0, sync≈1mm로 계속 정상 전진 중이었지만
기존 150초 hard timeout이 먼저 종료했습니다.

V2.17.8:
- 전체 삽입 hard timeout 완전 제거
- 8초간 >=1cm 실제 진행이 없을 때만 STALL 실패
- 계속 조금이라도 전진하면 끝까지 진행
- calibrated target 3.00m와 physical ATTACH handshake는 그대로

## LiDAR가 안 보인 원인
RViz는 /coop/scan_left, /coop/scan_right를 보고 있었습니다.
하지만 CooperativeNav2Bridge는 cart_controller.attached == True가 된 뒤에만
lazy-create됩니다. 따라서 도킹/삽입 중에는 /coop/scan_* 자체가 존재하지 않습니다.

Isaac의 원본:
- AMR1 /scan
- AMR2 /amr2/scan

은 시작부터 살아 있으므로 V2.17.8은 raw LaserScan과 각 AMR /world_pose를 결합하여
map-frame PointCloud2를 항상 생성합니다.

출력:
- /viz/amr1_lidar_points
- /viz/amr2_lidar_points

이 PointCloud2는 frame_id=map이므로 cooperative TF/attach 상태와 무관하게
도킹 전/중/후 항상 RViz에서 표시됩니다.
