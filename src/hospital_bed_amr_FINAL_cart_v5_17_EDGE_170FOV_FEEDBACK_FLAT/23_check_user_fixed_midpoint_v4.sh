#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
python3 - <<'PY'
import json, pathlib, py_compile
r=pathlib.Path.cwd()
cfg=json.loads((r/'config/isaac_config.json').read_text())['cooperative_warehouse_cart']
p=cfg['placement']
assert p['mode']=='fixed_user_world_midpoint'
v=p['vending_world_xy_m']; d=p['desk_world_xy_m']; f=p['fixed_world_xyz_m']
mx=(v[0]+d[0])/2; my=(v[1]+d[1])/2
assert abs(f[0]-mx)<1e-9 and abs(f[1]-my)<1e-9
assert abs(f[0]-(-28.7106))<1e-6 and abs(f[1]-11.3750)<1e-6
code=(r/'scripts/cooperative_warehouse_cart.py').read_text()
assert 'EXACT USER MIDPOINT' in code
assert 'USER FIXED anchors' in code
py_compile.compile(str(r/'scripts/cooperative_warehouse_cart.py'), doraise=True)
py_compile.compile(str(r/'scripts/isaac_amr_ros.py'), doraise=True)
print(f"[CHECK] vending = ({v[0]:.4f}, {v[1]:.4f})")
print(f"[CHECK] desk    = ({d[0]:.4f}, {d[1]:.4f})")
print(f"[CHECK] midpoint= ({f[0]:.4f}, {f[1]:.4f})")
print(f"[CHECK] yaw     = {p['fixed_yaw_deg']:.2f} deg")
print('[PASS] V4 placement uses the user-fixed world midpoint only')
print('[PASS] V4 Python static checks complete')
PY
