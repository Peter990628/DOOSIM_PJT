# V5.3 Clearance Balance — lethal-only collision fix

핵심 수정:
- 후보 경로 collision reject 기준: cost >= 100만 충돌 처리
- Inflation(0~99)은 후보를 삭제하지 않고 좌/우 clearance / inflation cost 계산에만 사용
- 따라서 오른쪽 inflation cost가 높으면 왼쪽 후보가 살아남아 실제로 선택될 수 있음
- unknown은 내부에서 100으로 처리하므로 여전히 안전상 충돌 취급

확인 로그:
- `reverted`가 V5.2보다 크게 감소해야 함
- `avg_shift`, `max_shift`가 0에 가깝지 않아야 함
- RViz: Raw State Lattice Plan(빨강)과 Clearance Balanced Plan(초록)이 분리되어 보여야 함
