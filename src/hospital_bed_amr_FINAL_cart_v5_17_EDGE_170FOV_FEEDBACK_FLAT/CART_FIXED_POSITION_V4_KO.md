# Cooperative Warehouse Cart V4 - 사용자 실측 좌표 고정

- 음료수 자판기 월드 좌표: X=-32.4262, Y=16.2500
- 리셉션 데스크 월드 좌표: X=-24.9950, Y=6.5000
- 카트 중심: X=-28.7106, Y=11.3750
- 카트 Yaw: -52.6863 deg (두 기준점을 잇는 복도 축과 평행)
- 위치 계산에서 desk/elevator prim 검색, bounding-box 추정, 자동 오프셋을 사용하지 않음.
- 기존 AMR1/AMR2는 physics 시작 전에 카트의 두 dock 위치로 이동하며 기존 주차 위치에 별도 AMR를 생성하지 않음.
- 시작 후 dual lift + fixed joint 자동 결합 및 W/S/A/D cooperative synchronization 로직은 V3에서 유지.
