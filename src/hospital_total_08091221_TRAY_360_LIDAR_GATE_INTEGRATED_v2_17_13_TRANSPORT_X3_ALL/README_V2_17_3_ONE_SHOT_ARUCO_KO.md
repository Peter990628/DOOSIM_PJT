# V2.17.3 ONE-SHOT ARUCO FIXED INSERT

- 시작 위치를 기존보다 0.50m 더 뒤로 이동.
- 트레이 앞면과 카메라 거리 2.00m.
- 두 AMR 모두 실제 FULL PAIR을 정지 상태에서 5프레임 연속 확인.
- 중심/yaw가 맞으면 그 순간 실제 world pose/yaw 저장.
- 이후 카메라/ArUco는 더 이상 필요 없음.
- 트레이 반길이 1.10m + 앞면 standoff 2.00m = 정확히 3.10m 직진.
- 두 AMR 진행차 2.5cm 이상이면 앞선 AMR 감속.
- 9초 fake fallback 없음.
- 초기 ALIGN snap 없음.
- lateral/yaw drift가 커지면 attach하지 않고 실패.
