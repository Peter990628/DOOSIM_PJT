#!/usr/bin/env bash
# Same latest hospital_total runtime, but with only the 360-ray bridge fix; no tray mission.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -n "${ISAAC_SIM_DIR:-}" ]]; then :
elif [[ -x /home/peter-msi/isaacsim-5.1.0/python.sh ]]; then ISAAC_SIM_DIR=/home/peter-msi/isaacsim-5.1.0
elif [[ -x /mnt/isaac45/isaacsim_5.1/python.sh ]]; then ISAAC_SIM_DIR=/mnt/isaac45/isaacsim_5.1
else ISAAC_SIM_DIR=/home/peter-msi/isaacsim-5.1.0; fi
PYTHON_SH="$ISAAC_SIM_DIR/python.sh"
[[ -x "$PYTHON_SH" ]] || { echo "[ERROR] Isaac Sim python.sh not found: $PYTHON_SH" >&2; exit 1; }
source "$ROOT/scripts/clean_isaac_env.sh"
export ROS_DISTRO=humble ROS_DOMAIN_ID=117 RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" ROS_LOCALHOST_ONLY=0
INTERNAL_ROS_LIB="$ISAAC_SIM_DIR/exts/isaacsim.ros2.bridge/humble/lib"
[[ -d "$INTERNAL_ROS_LIB" ]] && export LD_LIBRARY_PATH="$INTERNAL_ROS_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$ROOT/tray_overlay/scripts:$ROOT/scripts${PYTHONPATH:+:$PYTHONPATH}"
exec "$PYTHON_SH" "$ROOT/scripts/isaac_amr_ros.py" --project-root "$ROOT" --config "$ROOT/config/isaac_config.json" "$@"
