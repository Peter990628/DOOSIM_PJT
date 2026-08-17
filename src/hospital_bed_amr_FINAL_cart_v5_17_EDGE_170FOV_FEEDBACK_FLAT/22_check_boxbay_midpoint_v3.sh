#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
python3 -m py_compile "$ROOT/scripts/cooperative_warehouse_cart.py" "$ROOT/scripts/isaac_amr_ros.py"
python3 - <<'PY2' "$ROOT/config/isaac_config.json"
import json,sys
c=json.load(open(sys.argv[1]))['cooperative_warehouse_cart']
assert c['enabled'] and c['start_coupled'] and c['sync_assist_enabled']
assert c['placement']['mode']=='elevator_desk_exact_midpoint'
g=c['geometry']; assert g['bay_side_wall_thickness_m']>0 and g['bay_center_wall_thickness_m']>0
b=c['cargo']['box_size_m']; assert abs(b[2]-0.22)<1e-9 and c['cargo']['count']==32
print('[PASS] exact elevator/desk midpoint placement configured')
print('[PASS] box body with two carved AMR bays configured')
print('[PASS] cartons configured laid-flat, 4x4x2')
print('[PASS] start coupled + synchronized cooperative drive configured')
PY2
grep -q '_prephysics_move_controller_to_world_dock' "$ROOT/scripts/cooperative_warehouse_cart.py"
grep -q 'moved existing .* BEFORE physics' "$ROOT/scripts/cooperative_warehouse_cart.py"
grep -q 'two carved AMR bays' "$ROOT/scripts/cooperative_warehouse_cart.py"
echo '[PASS] V3 static checks complete'
