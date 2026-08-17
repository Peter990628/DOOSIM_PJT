# V2.2 — Isaac 저장장치 I/O 사전검사 + 안전 복구

이 버전은 V2.1의 기능을 그대로 유지하면서, `/mnt/isaac45`가 `findmnt`에서는 `rw`로 보여도 실제로는 `Input/output error`가 나는 상태에서 Isaac을 실행하려던 문제를 막습니다.

## 바뀐 점

`RUN_TRAY_1_ISAAC_TOTAL_360.sh`는 세션 파일을 만들기 전에 다음을 실제로 수행합니다.

1. `python.sh` 데이터 읽기
2. `/mnt/isaac45` 계열이면 실제 임시 파일 쓰기/삭제
3. Isaac 번들 Python 실행: `ISAAC_STORAGE_PYTHON_OK`

셋 중 하나라도 실패하면 Isaac을 실행하지 않고 종료합니다. 따라서 깨진 loop/ext4를 `-x python.sh` 같은 메타데이터 검사만 통과해서 잘못 선택하는 상황을 막습니다.

## 평상시 실행

```bash
cd ~/hospital_bed_amr_projects/hospital_total_08091221_TRAY_360_LIDAR_GATE_INTEGRATED_v2_2

export ROS_DOMAIN_ID=117
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

./CHECK_ISAAC_STORAGE.sh
./RUN_TRAY_1_ISAAC_TOTAL_360.sh
```

정상이면 먼저 `[ISAAC STORAGE PASS]`가 나오고 그 뒤에만 `[SESSION]`이 생성됩니다.

터미널 2:

```bash
cd ~/hospital_bed_amr_projects/hospital_total_08091221_TRAY_360_LIDAR_GATE_INTEGRATED_v2_2
export ROS_DOMAIN_ID=117
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
./RUN_TRAY_2_AUTO_TOTAL_360.sh
```

## `/mnt/isaac45`가 I/O 오류일 때

비파괴 검사:

```bash
./RECOVER_ISAAC45_STORAGE.sh --check
```

복구가 필요하고 외장 SSD 연결이 안정적인 것을 확인했다면:

```bash
./RECOVER_ISAAC45_STORAGE.sh --repair
```

스크립트가 `REPAIR` 입력을 요구합니다. 자동 확인을 생략하려면 명시적으로:

```bash
./RECOVER_ISAAC45_STORAGE.sh --repair --yes
```

복구 모드는 내부 ext4를 마운트한 상태에서는 `e2fsck`를 실행하지 않습니다. 먼저 `/mnt/isaac45`를 unmount하고 stale loop를 detach한 뒤, 외부 `isaac45_ext4.img`의 여러 위치를 실제로 읽습니다. 이 단계에서 I/O 오류가 하나라도 나면 **fsck를 수행하지 않고 중단**합니다. 그 경우 USB 케이블/포트/외장 드라이브/NTFS 계층부터 해결해야 합니다.

## 그대로 보존되는 것

- `hospital_total_08091221` 보호 원본 231개 파일
- ROS_DOMAIN_ID=117
- AMR1/AMR2 traffic pause 및 AMR2 namespace/map
- 2층 elevator 0.35m near-success
- stale goal 제거 및 AMR2 WAITING_GOAL 수정
- 코너 0.60s / 0.70rad/s / 0.12rad/s / KP 1.0
- V2.1 map_server lifecycle 자동 configure/activate
- 360° LiDAR 약 720 ray 변환
- 트레이 3기둥 ArUco 40/41 | 44 | 42/43
- Dual Lift / FixedJoint / cooperative Nav2

## 주의

`RUN_TRAY_1`은 파일시스템을 자동 수리하지 않습니다. 실제 수리는 반드시 사용자가 `RECOVER_ISAAC45_STORAGE.sh --repair`를 직접 실행한 경우에만 시작합니다.
