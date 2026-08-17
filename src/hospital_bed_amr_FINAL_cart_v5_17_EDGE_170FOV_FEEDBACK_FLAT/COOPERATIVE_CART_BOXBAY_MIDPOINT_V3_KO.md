# Cooperative Warehouse Cart V3

## 이번 수정 핵심
- 카트는 테이블 프레임이 아니라 **박스형 본체**이며, 하부에 AMR1/AMR2용 직사각형 통로 2개만 파인 형태입니다.
- 카트 중심은 **1층 엘리베이터 중심과 Office_desk 중심의 정확한 XY 중점**입니다. 자판기 기준 이동/보정은 제거했습니다.
- 갈색 Shipper 박스는 0.22 m가 높이가 되도록 눕혀서 4x4, 2단으로 32개 적재합니다.
- 기존 /World/AMR1, /World/AMR2를 새로 복제하지 않습니다. Physics 시작 전에 그 두 Prim 자체를 카트 하부 두 Bay로 이동합니다. 따라서 기존 주차 위치에 별도 AMR가 남지 않아야 합니다.
- Timeline 시작 후에는 위치 재텔레포트를 하지 않고 Lift 상승 -> Dual FixedJoint만 수행합니다.
- 결합 뒤 W/S/A/D는 두 AMR에 카트 중심 기준 차등 속도를 계산하며 cart velocity sync assist도 유지합니다.

## 실행
```bash
./22_check_boxbay_midpoint_v3.sh
./03_run_isaac.sh
```

정상 시작 로그:
- `[CARGO CART POSITION] EXACT MIDPOINT selected`
- `[CARGO CART START] moved existing /World/AMR1 ... BEFORE physics`
- `[CARGO CART START] moved existing /World/AMR2 ... BEFORE physics`
- `[CARGO CART READY] BOX BODY: two carved AMR bays ...`
- `[COOPERATIVE CART MODE] READY AT START`
