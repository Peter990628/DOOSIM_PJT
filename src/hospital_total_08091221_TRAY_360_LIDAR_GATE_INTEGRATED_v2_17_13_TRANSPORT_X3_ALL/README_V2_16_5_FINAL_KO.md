# V2.16.5 FINAL DEMO STABLE

이 폴더는 패치 누적본이 아니라 V2.16 기준 전체본에 최신 시연 변경사항을 직접 통합한 버전입니다.

## 최종 시나리오
1. 트레이 앞면 기준 1.50 m 위치에서 AMR1/AMR2 시작.
2. 좌우 도킹 Y와 yaw는 최종 결합 중심과 동일하며 뒤로만 떨어져 있음.
3. 양쪽 ArUco scanner가 실제 ID를 확인. 실패 시 시연용 fallback은 유지.
4. 두 AMR은 lateral/yaw 보정 없이 2.60 m 직진-only.
5. Lift UP + dual FixedJoint.
6. 결합 후 실제 cart world pose 기반 cooperative transport.
7. 직선 최대 요청 속도 0.66 m/s.
8. attached=false 단일 프레임으로 종료하지 않고 2초 grace 후 자동 ATTACH 복구.
9. 목적지 의료진은 트레이 통로 밖 world (9.50, 7.00)에 배치.
10. 문제 있던 MRI T-pose doctor asset은 정상 upright 의료진 asset으로 교체.

## 실행
Terminal 1:
```bash
./RUN_V216_1_ISAAC_SCAN_READY_EXTERNAL_SAFE.sh
```

Isaac Stage가 정상 로드되면 PLAY.

Terminal 2:
```bash
./RUN_V216_2_SCAN_STRAIGHT_ATTACH_TRANSPORT.sh
```

## 정상 핵심 로그
- `[V2.16 SCAN READY PASS]`
- `[V2.16 ARUCO PASS]` 또는 fallback
- `[V2.16 STRAIGHT INSERT PASS]`
- `[V2.16 ATTACH PASS]`
- `[V2.16.5 STEP4] FAST TRANSPORT x3 + ATTACH GUARD`
- 직선에서 `V=+0.66`
- 필요시 `[V2.16.5 ATTACH RECOVERY PASS]`
- 최종 `[V2.16.5 TRANSPORT SUCCESS]`
