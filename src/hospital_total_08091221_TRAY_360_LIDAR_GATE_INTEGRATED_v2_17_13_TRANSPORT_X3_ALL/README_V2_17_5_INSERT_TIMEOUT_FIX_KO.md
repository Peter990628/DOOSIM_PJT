# V2.17.5 INSERT TIMEOUT FIX

V2.17.4 실행 로그에서:
- AMR1/AMR2 lateral error = 거의 0.000m
- sync error = 약 0.002~0.003m
- 두 AMR 모두 계속 실제 전진 중
- 3.10m 목표 중 약 2.08m까지 진행
- 그러나 기존 52초 hard timeout이 먼저 만료되어 rc=15로 종료

수정:
- fixed insertion 목표거리 3.10m 유지
- 속도 프로파일 유지
- hard timeout 52초 -> 150초
- 8초간 1cm 이상의 실제 진행이 없을 때만 STALL 실패
- 계속 움직이는 동안에는 중간 timeout으로 실패하지 않음
- lateral/yaw/sync 안전 제한 유지
- V2.17.4 OUTBOARD CASTERS 유지
- ArUco one-shot lock 유지
