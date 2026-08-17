#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
python3 -m py_compile "$ROOT/scripts/cooperative_warehouse_cart.py" "$ROOT/scripts/isaac_amr_ros.py"
python3 - <<'PY' "$ROOT/config/isaac_config.json"
import json,sys
c=json.load(open(sys.argv[1]))['cooperative_warehouse_cart']
assert c['enabled'] and c['start_coupled'] and c['sync_assist_enabled']
assert c['placement']['mode']=='desk_vending_auto'
assert 'Office_desk' in c['placement']['desk_name_tokens']
assert 'MI_DrinksMachine' in c['placement']['vending_name_tokens']
print('[PASS] desk/vending auto placement configured')
print('[PASS] pre-coupled startup configured')
print('[PASS] cooperative velocity sync assist configured')
PY
grep -q 'initialize_start_coupled' "$ROOT/scripts/cooperative_warehouse_cart.py"
grep -q 'apply_sync_assist' "$ROOT/scripts/cooperative_warehouse_cart.py"
grep -q 'cart_controller.initialize_start_coupled()' "$ROOT/scripts/isaac_amr_ros.py"
grep -q 'cart_controller.apply_sync_assist()' "$ROOT/scripts/isaac_amr_ros.py"
echo '[PASS] V2 static checks complete'
