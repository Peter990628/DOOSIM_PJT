#!/usr/bin/env bash
# Remove Isaac Sim libraries before using the system ROS 2 Python 3.10 environment.
# The project normally uses Isaac Sim 5.1 at /mnt/isaac45/isaacsim_5.1.
if [[ -z "${ISAAC_SIM_DIR:-}" ]]; then
  if [[ -d "/mnt/isaac45/isaacsim_5.1" ]]; then
    ISAAC_SIM_DIR="/mnt/isaac45/isaacsim_5.1"
  elif [[ -d "$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release" ]]; then
    ISAAC_SIM_DIR="$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
  else
    ISAAC_SIM_DIR="/mnt/isaac45/isaacsim_5.1"
  fi
fi
export ISAAC_SIM_DIR
filter_path() {
  local input="${1:-}" item out=""
  IFS=':' read -r -a items <<< "$input"
  for item in "${items[@]}"; do
    [[ -z "$item" ]] && continue
    case "$item" in
      "$ISAAC_SIM_DIR"|"$ISAAC_SIM_DIR"/*) continue ;;
    esac
    out="${out:+$out:}$item"
  done
  printf '%s' "$out"
}
export PYTHONPATH="$(filter_path "${PYTHONPATH:-}")"
export LD_LIBRARY_PATH="$(filter_path "${LD_LIBRARY_PATH:-}")"
export PATH="$(filter_path "${PATH:-}")"
unset PYTHONHOME || true
