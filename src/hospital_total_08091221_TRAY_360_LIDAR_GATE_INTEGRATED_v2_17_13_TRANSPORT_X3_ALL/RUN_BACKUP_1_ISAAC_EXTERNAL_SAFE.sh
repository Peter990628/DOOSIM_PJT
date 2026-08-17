#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_SIM_DIR="/mnt/isaac45/isaacsim_5.1"
PY="$ISAAC_SIM_DIR/python.sh"
KIT_CACHE_TARGET="$ISAAC_SIM_DIR/kit/cache"
CACHE_ROOT="$ROOT/output/isaac_external_safe_runtime"
KIT_CACHE_SOURCE="$CACHE_ROOT/kit_cache"
SESSION_OUT="$ROOT/output/backup_precoupled_v2_14"
SESSION_FILE="$SESSION_OUT/current_session_id"
LOG="$CACHE_ROOT/terminal1.log"

mkdir -p "$CACHE_ROOT" "$KIT_CACHE_SOURCE" "$SESSION_OUT"
chmod -R u+rwX "$CACHE_ROOT" "$SESSION_OUT" 2>/dev/null || true

# Prepare the user's existing external storage. No write is made to the Seagate image.
"$ROOT/PREPARE_EXTERNAL_ISAAC45_SAFE.sh"

source "$ROOT/scripts/clean_isaac_env.sh"

# Verify bundled Python from the actual external Isaac install.
if ! dd if="$PY" of=/dev/null bs=64K count=1 status=none 2>/dev/null; then
  echo "[BLOCKED] real read failed: $PY"
  exit 40
fi
set +e
PYO=$(timeout 35 "$PY" -c 'print("ISAAC_EXTERNAL_PYTHON_OK")' 2>&1)
PYRC=$?
set -e
if [[ $PYRC -ne 0 || "$PYO" != *ISAAC_EXTERNAL_PYTHON_OK* ]]; then
  echo "[BLOCKED] external Isaac Python execution failed rc=$PYRC"
  echo "$PYO"
  exit 41
fi

# Critical V2.15.1 fix:
# Isaac/Kit writes material_cache.json under INSTALL/kit/cache even when
# XDG_CACHE_HOME is redirected. Since the 300GiB image is intentionally RO,
# bind a writable INTERNAL-SSD cache directory over only kit/cache.
[[ -d "$KIT_CACHE_TARGET" ]] || { echo "[ERROR] kit/cache target missing: $KIT_CACHE_TARGET"; exit 42; }

cleanup_cache_bind(){
  if mountpoint -q "$KIT_CACHE_TARGET"; then
    echo '[CLEANUP] unmounting writable Kit cache overlay'
    sudo umount "$KIT_CACHE_TARGET" 2>/dev/null || true
  fi
}
trap cleanup_cache_bind EXIT INT TERM

if mountpoint -q "$KIT_CACHE_TARGET"; then
  echo '[PREP] removing stale Kit cache bind mount'
  sudo umount "$KIT_CACHE_TARGET"
fi

echo '[PREP] binding INTERNAL writable cache onto external Isaac kit/cache'
echo "       source=$KIT_CACHE_SOURCE"
echo "       target=$KIT_CACHE_TARGET"
sudo mount --bind "$KIT_CACHE_SOURCE" "$KIT_CACHE_TARGET"

touch "$KIT_CACHE_TARGET/.v2151_write_test"
rm -f "$KIT_CACHE_TARGET/.v2151_write_test"
echo '[PASS] INSTALL/kit/cache write test succeeded through bind overlay'

export ISAAC_SIM_DIR
export ROS_DISTRO=humble ROS_DOMAIN_ID=117 RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" ROS_LOCALHOST_ONLY=0
INTERNAL_ROS_LIB="$ISAAC_SIM_DIR/exts/isaacsim.ros2.bridge/humble/lib"
[[ -d "$INTERNAL_ROS_LIB" ]] && export LD_LIBRARY_PATH="$INTERNAL_ROS_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
[[ -f "$HOME/.ros/fastdds_whitelist.xml" ]] && export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
export PYTHONPATH="$ROOT/tray_overlay/scripts:$ROOT/scripts${PYTHONPATH:+:$PYTHONPATH}"

# Everything that can be redirected is written to the INTERNAL project SSD.
mkdir -p "$CACHE_ROOT/cache" "$CACHE_ROOT/config" "$CACHE_ROOT/data" "$CACHE_ROOT/tmp" "$CACHE_ROOT/pycache"
export XDG_CACHE_HOME="$CACHE_ROOT/cache"
export XDG_CONFIG_HOME="$CACHE_ROOT/config"
export XDG_DATA_HOME="$CACHE_ROOT/data"
export TMPDIR="$CACHE_ROOT/tmp"
export PYTHONPYCACHEPREFIX="$CACHE_ROOT/pycache"

SESSION="$(date +%s)-$RANDOM-$RANDOM"
printf '%s\n' "$SESSION" > "$SESSION_FILE"
export TRAY_SESSION_ID="$SESSION" HOSPITAL_TRAY_PROJECT_ROOT="$ROOT"

echo '================================================================='
echo '[V2.15.1 EXTERNAL SSD SAFE BACKUP - TERMINAL 1]'
echo 'Isaac Sim : /mnt/isaac45/isaacsim_5.1  (existing external copy)'
echo 'Isaac data: READ ONLY'
echo 'Kit cache : writable bind overlay on INTERNAL SSD'
echo 'Project   : INTERNAL SSD'
echo 'START     : tray + AMR1 + AMR2 pre-coupled'
echo "SESSION=$SESSION"
echo "LOG=$LOG"
echo '================================================================='

set +e
"$PY" "$ROOT/tray_overlay/scripts/isaac_amr_ros_tray_backup_precoupled.py" \
  --project-root "$ROOT" \
  --config "$ROOT/tray_overlay/config/isaac_config_backup_precoupled.json" "$@" \
  2>&1 | tee "$LOG"
RC=${PIPESTATUS[0]}
set -e
exit "$RC"
