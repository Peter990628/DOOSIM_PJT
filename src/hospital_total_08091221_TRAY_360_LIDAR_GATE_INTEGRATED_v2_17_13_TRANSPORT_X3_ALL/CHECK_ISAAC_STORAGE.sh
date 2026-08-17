#!/usr/bin/env bash
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/scripts/isaac_storage_guard.sh"

echo '============================================================'
echo '[ISAAC STORAGE CHECK V2.2] real read + write + python execute'
echo '============================================================'

if [[ -n "${ISAAC_SIM_DIR:-}" ]]; then
  CANDIDATES=("$ISAAC_SIM_DIR")
else
  CANDIDATES=(/home/peter-msi/isaacsim-5.1.0 /mnt/isaac45/isaacsim_5.1)
fi

ok=0
for c in "${CANDIDATES[@]}"; do
  [[ -e "$c/python.sh" ]] || { echo "[MISS] $c/python.sh"; continue; }
  echo "[CHECK] $c"
  findmnt -T "$c/python.sh" 2>/dev/null || true
  if isaac_probe_root "$c"; then
    echo "[PASS] $c"
    ok=1
  else
    echo "[FAIL] $c"
    echo "       $isaac_storage_last_error"
  fi
  echo
 done

if (( ok == 0 )); then
  echo '[BLOCKED] No healthy Isaac Sim installation was found.'
  echo '[NEXT] For the /mnt/isaac45 loop-image layout:'
  echo '       ./RECOVER_ISAAC45_STORAGE.sh --check'
  echo '       ./RECOVER_ISAAC45_STORAGE.sh --repair'
  exit 2
fi

echo '[READY] At least one Isaac Sim installation passed real I/O probes.'
