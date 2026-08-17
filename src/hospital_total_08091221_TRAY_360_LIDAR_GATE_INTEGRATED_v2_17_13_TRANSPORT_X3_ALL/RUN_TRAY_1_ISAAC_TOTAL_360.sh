#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$ROOT/output/tray_integrated_v2_12"
mkdir -p "$OUT"
SESSION_FILE="$OUT/current_session_id"
# Never let Terminal2 trust a session left behind by a failed storage launch.
rm -f "$SESSION_FILE"

source "$ROOT/scripts/clean_isaac_env.sh"
source "$ROOT/scripts/isaac_storage_guard.sh"

echo '============================================================'
echo '[ISAAC STORAGE PREFLIGHT V2.6]'
echo 'metadata is NOT trusted: python.sh is really read, the loop volume is write-probed,'
echo 'and Isaac bundled Python must execute before a session is created.'
echo '============================================================'

if ! ISAAC_SIM_DIR="$(isaac_select_root)"; then
  echo >&2
  echo '[BLOCKED] No healthy Isaac Sim installation passed the real I/O preflight.' >&2
  echo '[INFO] Current /mnt/isaac45 state:' >&2
  findmnt -T /mnt/isaac45/isaacsim_5.1/python.sh 2>/dev/null >&2 || true
  sudo -n losetup -l 2>/dev/null | grep -F 'isaac45_ext4.img' >&2 || true
  echo >&2
  echo '[SAFE NEXT STEP]' >&2
  echo '  ./CHECK_ISAAC_STORAGE.sh' >&2
  echo '  ./RECOVER_ISAAC45_STORAGE.sh --check' >&2
  echo '  ./RECOVER_ISAAC45_STORAGE.sh --repair' >&2
  echo '[NOTE] RUN_TRAY_1 will never auto-fsck a drive.' >&2
  exit 40
fi
export ISAAC_SIM_DIR
PYTHON_SH="$ISAAC_SIM_DIR/python.sh"
echo "[ISAAC STORAGE PASS] $ISAAC_SIM_DIR"

export ROS_DISTRO=humble
export ROS_DOMAIN_ID=117
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY=0
INTERNAL_ROS_LIB="$ISAAC_SIM_DIR/exts/isaacsim.ros2.bridge/humble/lib"
[[ -d "$INTERNAL_ROS_LIB" ]] && export LD_LIBRARY_PATH="$INTERNAL_ROS_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
[[ -f "$HOME/.ros/fastdds_whitelist.xml" ]] && export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
# Critical: use the fixed copy of CURRENT hospital_total nav2_bridge before baseline scripts path.
export PYTHONPATH="$ROOT/tray_overlay/scripts:$ROOT/scripts${PYTHONPATH:+:$PYTHONPATH}"

SESSION="$(date +%s)-$RANDOM-$RANDOM"
printf '%s\n' "$SESSION" > "$SESSION_FILE"
export TRAY_SESSION_ID="$SESSION"
export HOSPITAL_TRAY_PROJECT_ROOT="$ROOT"

echo '============================================================'
echo '[ISAAC V2.12 FINAL] hospital_total_08091221 + AUTO-DOCK TRAY + 360 LIDAR + FINAL STAFF'
echo '             + MAP LIFECYCLE RECOVERY + STORAGE I/O PREFLIGHT'
echo "[ISAAC] dir=$ISAAC_SIM_DIR"
echo '[DOMAIN] 117 (latest hospital_total preserved)'
echo '[LIDAR] fixed bridge expects ~720 rays for /scan and /amr2/scan'
echo '[TRAFFIC] latest hospital_total traffic-pause wiring preserved'
echo '[ARUCO GATE] LEFT 40/41 | CENTER 44 | RIGHT 42/43 (visual-only carriers)'
echo '[DOCKING] PRE_DOCK -> single expected ArUco ID -> fixed straight tray insertion -> auto dual attach'
echo '[STAFF] manual captured poses override auto placement; DoctorMRI forced upright +90deg X (non-physical)'
echo '[STORAGE] real read/write/python execution PASSED before session creation'
echo "[SESSION] $SESSION"
echo '============================================================'

# If exec itself fails, invalidate the session so Terminal2 cannot start against a dead Isaac process.
if ! exec "$PYTHON_SH" "$ROOT/tray_overlay/scripts/isaac_amr_ros_tray_runtime.py" \
  --project-root "$ROOT" \
  --config "$ROOT/tray_overlay/config/isaac_config_tray_integrated.json" \
  "$@"; then
  rm -f "$SESSION_FILE"
  exit 41
fi
