# V2.13 DEMO FORCE SUCCESS (추가 실행경로)

이 모드는 원본 로직을 수정하지 않는다.

그대로 보존되는 파일:
- `RUN_TRAY_2_AUTO_TOTAL_360.sh`
- `hospital_nav2/path_conflict_manager.py`
- `hospital_tray_overlay/cooperative_transport_manager.py`
- `centerline_navigator.py`
- Isaac runtime / cart controller

추가된 별도 실행경로 `RUN_TRAY_2_DEMO_FORCE_SUCCESS.sh`만 다음과 같이 동작한다.

1. AMR1/AMR2 기존 독립 Nav2 실행.
2. 이 실행에서만 `path_conflict_manager`를 시작하지 않는다. 따라서 두 AMR이 같이 주행한다.
3. 첫 PRE_DOCK 도착 시 기존 ArUco scanner를 실행한다.
4. ID44 또는 해당 AMR의 outer ID가 보이면 짧게 직진하는 모습을 보여준다.
5. 직진 중 물리 충돌/정체가 발생해도 거리 부족으로 미션 실패시키지 않는다.
6. 두 AMR 준비 후 기존 트레이 컨트롤러에 이미 있던 `ALIGN(G)` 명령을 호출한다.
7. `ALIGN(G)`는 Physics를 잠깐 pause하고 AMR1/2를 트레이 좌/우 정확한 dock center에 배치한 뒤 resume한다.
8. `ATTACH(K)`를 수행한다. 실패 시 ALIGN→ATTACH를 최대 3회 반복한다.
9. 결합 이후에는 기존 cooperative Nav2를 그대로 사용한다.

즉, **ArUco 화면은 시연 증거**, 최종 결합 위치는 **기존 ALIGN/G 기능**이 보증한다.

## 실행
Terminal 1은 기존과 동일:

```bash
./RUN_TRAY_1_ISAAC_TOTAL_360.sh
```

Isaac PLAY 후 Terminal 2만 아래를 사용:

```bash
./RUN_TRAY_2_DEMO_FORCE_SUCCESS.sh
```

일반 모드로 돌아가려면 기존 파일을 그대로 실행하면 된다:

```bash
./RUN_TRAY_2_AUTO_TOTAL_360.sh
```
