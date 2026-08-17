#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

resolve_isaac_dir() {
  local candidate
  if [[ -n "${ISAAC_SIM_DIR:-}" && -x "${ISAAC_SIM_DIR}/python.sh" ]]; then
    printf '%s\n' "$ISAAC_SIM_DIR"
    return 0
  fi
  for candidate in \
    "/mnt/isaac45/isaacsim_5.1" \
    "$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release" \
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"; do
    if [[ -x "$candidate/python.sh" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  return 1
}

ISAAC_SIM_DIR="$(resolve_isaac_dir || true)"
if [[ -z "$ISAAC_SIM_DIR" ]]; then
  echo "[ERROR] Isaac Sim 5.1 python.sh를 찾지 못했습니다." >&2
  echo "[EXPECTED] /mnt/isaac45/isaacsim_5.1/python.sh" >&2
  echo "[HINT] 외장 SSD 이미지가 마운트되어 있는지 확인하세요: ls -l /mnt/isaac45/isaacsim_5.1/python.sh" >&2
  echo "[HINT] 다른 위치라면: ISAAC_SIM_DIR=/path/to/isaacsim_5.1 ./03_run_isaac.sh" >&2
  exit 1
fi
export ISAAC_SIM_DIR
PYTHON_SH="$ISAAC_SIM_DIR/python.sh"

source "$ROOT/scripts/clean_isaac_env.sh"
export ROS_DISTRO=humble
export ROS_DOMAIN_ID=120
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY=0
INTERNAL_ROS_LIB="$ISAAC_SIM_DIR/exts/isaacsim.ros2.bridge/humble/lib"
if [[ -d "$INTERNAL_ROS_LIB" ]]; then
  export LD_LIBRARY_PATH="$INTERNAL_ROS_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
if [[ -f "$HOME/.ros/fastdds_whitelist.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
fi

echo "[ISAAC] Python-only standalone app"
echo "[ISAAC] ISAAC_SIM_DIR=$ISAAC_SIM_DIR"
echo "[ISAAC] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[ISAAC] Do NOT source /opt/ros/humble in this terminal"
exec "$PYTHON_SH" "$ROOT/scripts/isaac_amr_ros.py" \
  --project-root "$ROOT" \
  --config "$ROOT/config/isaac_config.json" \
  "$@"
