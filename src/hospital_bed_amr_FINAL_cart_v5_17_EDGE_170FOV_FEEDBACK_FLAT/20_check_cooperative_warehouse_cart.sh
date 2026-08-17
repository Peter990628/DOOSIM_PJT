#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 - <<'PY' "$ROOT"
import json, pathlib, py_compile, sys
root=pathlib.Path(sys.argv[1])
required=[
 root/'scripts/cooperative_warehouse_cart.py',
 root/'scripts/isaac_amr_ros.py',
 root/'config/isaac_config.json',
 root/'cargo_cart_assets/source/medi_m.glb',
 root/'project4/project4_hospital_bed_amr_v1_15_ocr.usd',
]
for p in required:
    assert p.exists(), f'missing: {p}'
py_compile.compile(str(root/'scripts/cooperative_warehouse_cart.py'),doraise=True)
py_compile.compile(str(root/'scripts/isaac_amr_ros.py'),doraise=True)
cfg=json.loads((root/'config/isaac_config.json').read_text(encoding='utf-8'))
c=cfg['cooperative_warehouse_cart']; g=c['geometry']
assert c['enabled'] is True
assert abs(g['length_m']-2.20)<1e-6
assert abs(g['width_m']-1.70)<1e-6
assert abs(g['amr_body_gap_m']-0.12)<1e-6
combined=2*g['amr_width_m']+g['amr_body_gap_m']
clearance=(g['width_m']-combined)/2
assert clearance>0.04, clearance
src=(root/'scripts/isaac_amr_ros.py').read_text(encoding='utf-8')
for token in ['install_cooperative_warehouse_cart','KeyboardInput.G','KeyboardInput.K','KeyboardInput.J','cooperative_commands']:
    assert token in src, token
print(f'[CHECK] cart={g["length_m"]:.2f} x {g["width_m"]:.2f} m')
print(f'[CHECK] two AMRs combined={combined:.2f} m, body gap={g["amr_body_gap_m"]:.2f} m, side clearance={clearance:.3f} m each')
print(f'[CHECK] equivalent loaded mass={c["cart_mass_kg"]+c["equivalent_cargo_mass_kg"]:.1f} kg')
print('[CHECK] original hospital stage preserved; runtime cart injected in existing map')
print('[PASS] cooperative warehouse cart static validation complete')
PY
