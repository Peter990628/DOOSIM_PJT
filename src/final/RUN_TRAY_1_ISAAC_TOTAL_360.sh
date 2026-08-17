#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$ROOT/output/tray_integrated"
SESSION_FILE="$OUT/current_session_id"
mkdir -p "$OUT"
rm -f "$SESSION_FILE"

# final이 배포되는 동료 PC의 기존 Isaac 경로를 기본값으로 유지합니다.
ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release}"
export ISAAC_SIM_DIR
PYTHON_SH="$ISAAC_SIM_DIR/python.sh"
[[ -x "$PYTHON_SH" ]] || {
  echo "[ERROR] Isaac Sim python.sh not found: $PYTHON_SH" >&2
  echo '[FIX] 다른 위치라면 ISAAC_SIM_DIR을 export한 뒤 다시 실행하세요.' >&2
  exit 40
}

source "$ROOT/scripts/clean_isaac_env.sh"
export ROS_DISTRO=humble
export ROS_DOMAIN_ID=115
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY=0
INTERNAL_ROS_LIB="$ISAAC_SIM_DIR/exts/isaacsim.ros2.bridge/humble/lib"
[[ -d "$INTERNAL_ROS_LIB" ]] && export LD_LIBRARY_PATH="$INTERNAL_ROS_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
[[ -f "$HOME/.ros/fastdds_whitelist.xml" ]] && export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
export PYTHONPATH="$ROOT/tray_overlay/scripts:$ROOT/scripts${PYTHONPATH:+:$PYTHONPATH}"

SESSION="$(date +%s)-$RANDOM-$RANDOM"
printf '%s\n' "$SESSION" > "$SESSION_FILE"
export TRAY_SESSION_ID="$SESSION"
export HOSPITAL_TRAY_PROJECT_ROOT="$ROOT"

echo '============================================================'
echo '[TRAY AUTO / TERMINAL 1] Isaac + tray + 360 LiDAR'
echo "[ISAAC] $ISAAC_SIM_DIR"
echo '[DOMAIN] ROS_DOMAIN_ID=115'
echo '[ARUCO] AMR1=40/41+44, AMR2=44+42/43'
echo "[SESSION] $SESSION"
echo '============================================================'

if ! exec "$PYTHON_SH" "$ROOT/tray_overlay/scripts/isaac_amr_ros_tray_runtime.py" \
  --project-root "$ROOT" \
  --config "$ROOT/tray_overlay/config/isaac_config_tray_integrated.json" \
  "$@"; then
  rm -f "$SESSION_FILE"
  exit 41
fi
