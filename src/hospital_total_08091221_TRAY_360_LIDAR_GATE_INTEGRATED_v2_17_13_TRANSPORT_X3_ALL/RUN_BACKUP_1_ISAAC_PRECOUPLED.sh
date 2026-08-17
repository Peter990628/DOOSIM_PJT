#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$ROOT/output/backup_precoupled_v2_14"; mkdir -p "$OUT"; SESSION_FILE="$OUT/current_session_id"; rm -f "$SESSION_FILE"
source "$ROOT/scripts/clean_isaac_env.sh"
source "$ROOT/scripts/isaac_storage_guard.sh"
ISAAC_SIM_DIR="$(isaac_select_root)" || { echo '[BLOCKED] healthy Isaac Sim not found'; exit 40; }
export ISAAC_SIM_DIR ROS_DISTRO=humble ROS_DOMAIN_ID=117 RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" ROS_LOCALHOST_ONLY=0
INTERNAL_ROS_LIB="$ISAAC_SIM_DIR/exts/isaacsim.ros2.bridge/humble/lib"; [[ -d "$INTERNAL_ROS_LIB" ]] && export LD_LIBRARY_PATH="$INTERNAL_ROS_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
[[ -f "$HOME/.ros/fastdds_whitelist.xml" ]] && export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
export PYTHONPATH="$ROOT/tray_overlay/scripts:$ROOT/scripts${PYTHONPATH:+:$PYTHONPATH}"
SESSION="$(date +%s)-$RANDOM-$RANDOM"; printf '%s\n' "$SESSION" > "$SESSION_FILE"; export TRAY_SESSION_ID="$SESSION" HOSPITAL_TRAY_PROJECT_ROOT="$ROOT"
echo '================================================================='
echo '[V2.14 BACKUP ISAAC] PRE-COUPLED START'
echo 'START: tray (-22.69, 11.03), AMR1+AMR2 already under tray'
echo 'COUPLING: automatic Lift UP + dual FixedJoint immediately after PLAY'
echo 'NO ArUco / NO individual traffic manager / NO docking wait in this backup path'
echo "SESSION=$SESSION"
echo '================================================================='
exec "$ISAAC_SIM_DIR/python.sh" "$ROOT/tray_overlay/scripts/isaac_amr_ros_tray_backup_precoupled.py" \
  --project-root "$ROOT" --config "$ROOT/tray_overlay/config/isaac_config_backup_precoupled.json" "$@"
