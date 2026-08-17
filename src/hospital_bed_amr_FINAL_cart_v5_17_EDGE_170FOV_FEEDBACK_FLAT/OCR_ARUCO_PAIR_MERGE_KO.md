# FINAL 기준본 + OCR 신원확인 + 독립 ArUco 중심 도킹

이 프로젝트는 사용자가 제공한 `FINAL_ROUNDTRIP_JITTERFIX_EXIT5M_BOTH_OCR_FAST_2F_HEIGHT_LOCK_domain120`를 기준으로 한다.
기존 엘리베이터, MRI 왕복, 2F 높이 잠금, 퇴출 5m, Nav2, 자석 결합 등은 유지하고 아래 기능만 추가/교체했다.

- 기존 이름표 PNG 3개: 변경 없음
- QR: 사용하지 않음
- 김서울 침대: 독립 PNG ArUco 10 / 11
- 박인천 침대: 독립 PNG ArUco 20 / 21
- 서수원 침대: 독립 PNG ArUco 30 / 31
- 각 마커는 `ArUcoMarkers` 아래 침대 Root 자식이므로 침대 이동/결합/엘리베이터 이동 시 같이 움직임
- Collider/RigidBody 없는 render-only 카드
- 카드 0.16m, 실제 마커 0.14m, 이름표 가장자리와 카드 사이 0.025m

## 전체 시연의 변경된 도킹 단계

기존: OCR 신원확인 + OCR bbox 중심정렬 -> 3.328m 전진 -> 자석 결합

현재: OCR 신원확인 -> OCR 추적 종료 -> ArUco 좌/우 pair 중점 중심정렬 -> 기존 3.328m 전진 -> 자석 결합

즉 OCR 문자는 환자 확인에만 사용하며 `bbox_center_x/y`는 도킹 위치 제어에 사용하지 않는다.

## 실행

1. `./02_build_ros_ws.sh`
2. 터미널1 `./03_run_isaac.sh`
3. 터미널2 `./04_run_ocr_amr1.sh`
4. 터미널3 `./09_run_nav2_amr1.sh`
5. 확인 `./15_check_aruco_runtime.sh`
6. 전체 시연 `./13_run_patient_transport.sh 1`

정상적으로 김서울 침대 정면에서 `/amr1/aruco/result`의 `visible_ids`에 10,11이 함께 나타나고 `pairs.김서울.pair_center_x`가 생성되어야 한다.
