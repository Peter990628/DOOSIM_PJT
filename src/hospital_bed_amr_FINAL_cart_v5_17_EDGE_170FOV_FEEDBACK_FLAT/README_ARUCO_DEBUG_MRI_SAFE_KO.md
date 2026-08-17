# OCR + 독립 ArUco Debug + 2F MRI 안전정지 통합본

## 역할 분리
- OCR: 김서울 이름/생년월일 신원 확인만 사용
- ArUco 10/11: 침대 중심 정렬 좌표
- `/amr1/aruco/debug_image`: ID 박스, 좌우 마커 연결선, pair center, camera center, px 오차 표시
- MRI: 측정된 MRI TableTop XY `(8.5246, 5.8035)` 기준으로 기존 접근 방향을 유지하면서 1.05 m 떨어진 안전 정지점으로 Nav2 이동
- 환자 이동: 1차 MRI 도착 뒤 `/patient_transfer/patient1/status == MRI_BED` 확인 전에는 이탈하지 않음. 재진입 뒤 `TRANSPORT_BED` 확인 전에는 엘리베이터로 복귀하지 않음.

## 첫 1회
```bash
./02_build_ros_ws.sh
./18_validate_aruco_debug_mri_safe.py
```

## 실행
터미널 1:
```bash
./03_run_isaac.sh
```
터미널 2:
```bash
./04_run_ocr_amr1.sh
```
터미널 3:
```bash
./09_run_nav2_amr1.sh
```
터미널 4:
```bash
./13_run_patient_transport.sh 1
```

## ArUco 화면 보기
터미널 5:
```bash
./17_view_aruco_debug.sh
```
`rqt_image_view`에서 `/amr1/aruco/debug_image`를 선택한다.

## ROS
모든 실행 스크립트/실행 설정은 `ROS_DOMAIN_ID=120` 기준이다.
