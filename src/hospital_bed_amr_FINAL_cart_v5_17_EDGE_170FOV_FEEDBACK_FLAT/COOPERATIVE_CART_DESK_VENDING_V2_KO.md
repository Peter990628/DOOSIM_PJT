# Cooperative Warehouse Cart V2 — Desk/Vending Pre-coupled Start

## 시작 상태
- 기존 병원 맵 유지.
- `Office_desk`와 `MI_DrinksMachine`를 런타임에서 찾아 데스크 앞/자판기 쪽 열린 공간을 자동 선택.
- 4개 caster가 하중을 지지하는 카트, 갈색 Shipper 박스 적재.
- AMR1/AMR2는 같은 방향으로 카트 하부에 정렬.
- Isaac 조작이 시작되기 전에 두 lift를 올리고 FixedJoint 2개를 생성하여 **이미 결합된 상태**로 시작.

## 조작
- W/S: 전체 전진/후진
- A/D: 카트 중심 기준 동기 회전
- SPACE: 긴급 정지
- J: Dual Joint 해제 + lift down
- G/K: 해제 후 개발용 재정렬/재결합

## 동기화
AMR1/AMR2는 카트 중심 twist `(V, W)`에서 각 지지점 속도를 계산하며, cart RigidBody에도 같은 중심 속도/각속도를 보조 입력한다. FixedJoint가 실제 결합을 유지하고 velocity sync assist가 caster drag에 의한 지연/비틀림을 줄인다.
