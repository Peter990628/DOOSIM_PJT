# V2.15.1 External SSD Safe Backup

이 버전은 기존 외장 SSD의 Isaac Sim 5.1을 그대로 사용한다.

핵심 수정:
1. `/mnt/isaac45`의 300GiB ext4 이미지는 read-only 유지.
2. Isaac Kit가 설치 폴더의 `kit/cache`에 직접 쓰는 문제를 해결하기 위해,
   실행 중에만 내장 프로젝트의 writable cache를 `kit/cache` 위에 bind mount.
3. XDG cache/config/data, TMPDIR, Python pycache도 내장 SSD의 output으로 redirect.
4. backup stage loader가 고정 120 frame 대신 HospitalMap/AMR1/AMR2가 실제 compose될 때까지 최대 120초 대기.
5. `check_integration.py`의 오래된 `[BASE READY V2.11]` assertion을 현재 `[BASE READY V2.12.1]`로 수정.
6. 정상 프로젝트/자동도킹/traffic 원본 실행 로직은 변경하지 않음. 백업 실행 경로만 사용.

실행은 `START_HERE_V2_15_1_KO.txt` 참고.
