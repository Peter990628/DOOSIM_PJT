# V2.6.1 setup placeholder fix

V2.6에서 `output/.keep`, `output/ocr/.keep` 두 개의 빈 placeholder가 ZIP/로컬 정리 과정에서 빠지면 baseline integrity check가 실패하던 문제를 수정했습니다.

- 기존 hospital_total 기능/소스 검증은 유지합니다.
- `__pycache__/*.pyc`와 두 `output/*.keep` placeholder만 런타임/패키징 산출물로 제외합니다.
- 360 LiDAR, AMR2 map lifecycle 복구, 전면 ArUco gate, 자동 도킹/리프트/dual FixedJoint/cooperative Nav2, staff 배치, 기존 FollowCamera 동작은 V2.6과 동일합니다.
- setup 성공 후 `ros2_ws/install/setup.bash`가 생성되어야 합니다.

실행:
```bash
chmod +x ./*.sh scripts/*.sh tray_overlay/scripts/*.py
./00_SETUP_TRAY_360_INTEGRATED.sh
```
마지막에 `[DONE] setup/build complete`를 확인하세요.
