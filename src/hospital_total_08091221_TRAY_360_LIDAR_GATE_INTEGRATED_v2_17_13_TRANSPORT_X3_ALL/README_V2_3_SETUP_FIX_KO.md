# V2.3 setup fix

V2.2의 `check_integration.py`가 기준본 manifest 안의 `__pycache__/*.pyc`까지 필수 원본 파일로 검사해, ZIP에서 캐시 파일이 빠진 경우 setup이 빌드 전에 종료되는 문제가 있었습니다.

V2.3은 소스/설정/자산 등 영속 파일 213개에 대해서만 SHA-256 원본 보존을 검사하고, Python이 실행 시 재생성하는 18개의 `__pycache__/*.pyc` 항목은 무결성 검사에서 제외합니다. 기존 hospital_total 기능 코드는 변경하지 않습니다.

정상 setup 완료 기준은 `ros2_ws/install/setup.bash`가 생성되고 마지막에 `[DONE] setup/build complete`가 출력되는 것입니다.
