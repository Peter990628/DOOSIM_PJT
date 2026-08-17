#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION_OUT="$ROOT/output/v216_scan_straight"
SESSION_FILE="$SESSION_OUT/current_session_id"
LOG_DIR="$ROOT/output/v217_true_aruco"
mkdir -p "$SESSION_OUT" "$LOG_DIR"
rm -f "$SESSION_FILE"

# V2.17.13 전용 시연 모드입니다. 외장 SSD bind-mount 없이 final의 Isaac 경로를 사용합니다.
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
echo '[V2.17.13 / TERMINAL 1] scan-ready Isaac demo'
echo '[MODE] 두 AMR을 트레이 마커 관측 위치에 배치해 실제 ArUco 도킹을 시연합니다.'
echo '[SPEED] 도킹 가속 및 결합 후 운송 X3 설정'
echo "[ISAAC] $ISAAC_SIM_DIR"
echo '[DOMAIN] ROS_DOMAIN_ID=115'
echo "[SESSION] $SESSION"
echo '============================================================'

if ! exec "$PYTHON_SH" "$ROOT/tray_overlay/scripts/isaac_amr_ros_tray_scan_straight_v216.py" \
  --project-root "$ROOT" \
  --config "$ROOT/tray_overlay/config/isaac_config_scan_straight_v216.json" \
  "$@"; then
  rm -f "$SESSION_FILE"
  exit 41
fi
