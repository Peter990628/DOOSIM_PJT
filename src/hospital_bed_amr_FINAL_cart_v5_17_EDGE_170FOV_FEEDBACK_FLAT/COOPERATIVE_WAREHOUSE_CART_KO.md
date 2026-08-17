# Cooperative Warehouse Cart V1

## 목적
기존 병원 맵/엘리베이터/OCR/ArUco/Nav2/침대 결합 기능은 유지하고, 1층 엘리베이터 앞 복도에 창고형 대형 물류 카트를 추가한다.

참조 형태는 사용자가 제공한 `WAREHOUSE DISPLAY` 사진처럼 **원래부터 4개의 보조 캐스터로 지지되는 낮은 카트**이다.
AMR 전용 외부 포드나 돌출 홈은 없다. AMR1/AMR2가 정상적인 카트 상판 아래의 비어 있는 중앙 공간으로 같은 방향으로 들어간다.

## V1 기계 설계
- 카트 외곽: 2.20 m (길이) x 1.70 m (폭)
- 상판: 전체 면적을 덮는 일반 화물 적재 Deck
- 보조 바퀴: 네 모서리 4개 Warehouse Caster
- V1 물리: 보이는 캐스터 Wheel + 저마찰 구형 Contact Proxy
  - 복잡한 swivel+axle 8개 Joint 대신 안정적인 시연용 passive caster 모델
  - 카트 RigidBody와 화물의 수직 하중은 4개 contact proxy가 지면에 전달
- AMR: 기존 `/World/AMR1`, `/World/AMR2` 그대로 사용
- AMR 방향: 두 대 모두 같은 방향
- AMR 차체 사이 간격: 약 0.12 m
- 두 AMR 외곽 대비 카트 좌/우 여유: 약 0.05 m씩
- 카트 전체 등가 질량: 75 kg cart + 140 kg cargo = 약 215 kg

## 왜 Lift가 카트 전체를 들지 않는가
이 카트의 4개 캐스터는 결합 전/후 모두 바닥에 닿는다.
AMR Lift는 카트를 들어 올리는 용도가 아니라 **하부 Magnetic Dock Pad에 접촉하고 FixedJoint를 걸기 위한 35 mm 결합 스트로크**로 사용한다.

상판은 AMR 위에 열린 공간을 확보하도록 높이고, AMR Lift 높이에는 작은 Dock Pad 2개가 상판에서 매달리는 구조다.
외부에서 보면 별도 AMR 수납 포드가 아니라 하나의 정상적인 창고형 카트다.

## 박스
`cargo_cart_assets/source/medi_m.glb`의 갈색 Shipper/Object_0를 Isaac Asset Converter로 변환하여 사용한다.
기본 32개: 1층 5x4 + 2층 4x3.
변환 실패 시에만 동일 크기 갈색 proxy box로 fallback한다.

## 배치
가능하면 실제 엘리베이터 `Side_Lift_Anim_29`의 1층 위치를 읽고 복도 방향으로 약 3.4 m 떨어진 위치에 자동 배치한다.
엘리베이터 위치를 읽지 못하면 config의 fallback 좌표를 사용한다.

위치는 `config/isaac_config.json -> cooperative_warehouse_cart -> placement`에서 수정 가능하다.

## 키
기존 침대 C/X 키는 건드리지 않는다.

- `G`: 개발용 — AMR1/AMR2를 카트 하부 DockPoint에 정확히 정렬
- `K`: 두 AMR Lift 상승 -> 약 1.4 s 안정화 -> FixedJoint 2개 생성
- `J`: 카트 FixedJoint 2개 제거 -> 두 Lift 하강
- 결합 후 `W/S`: 전체 카트 전진/후진
- 결합 후 `A/D`: 트레이 중심 기준 좌/우 회전
- `SPACE`: 긴급 정지

## Cooperative 운동학
카트 중심의 목표 속도를 `(V, W)`라고 하면, 좌우 AMR의 y 오프셋을 각각 `+d`, `-d`로 두고:

- AMR1 forward = V - W*d
- AMR2 forward = V + W*d
- AMR1/AMR2 angular = W

따라서 제자리 회전에서는 한 AMR는 상대적으로 후진, 다른 AMR는 전진하여 카트 중심을 기준으로 회전한다.
카트 RigidBody 자체에는 평상시 구동 속도를 직접 넣지 않는다. 카트는 4개의 passive caster를 굴리며 AMR 두 대가 밀고 간다.

## 첫 테스트 순서
1. `./20_check_cooperative_warehouse_cart.sh`
2. `./03_run_isaac.sh`
3. Isaac Viewport 클릭
4. `G`
5. `K`
6. 로그에서 `COOPERATIVE CART MODE ON` 확인
7. `W`, `S`, `A`, `D` 테스트
8. `J`로 해제

## 안전 조건
- 침대와 이미 Magnetic FixedJoint 상태인 AMR가 있으면 카트 K 결합을 거부한다.
- 결합 대기 중 두 AMR는 정지한다.
- 카트 결합 모드에서는 Nav2/자동 접근보다 Cooperative 명령이 우선한다.
- 기존 병원 침대 magnetic joint path와 새 cart joint path는 완전히 별도다.
