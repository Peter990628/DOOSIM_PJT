#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$ROOT/ros2_ws"

source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
set +u
export ROS_DOMAIN_ID=120
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY=0
if [[ -f "$HOME/.ros/fastdds_whitelist.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
fi

if ! ros2 pkg prefix nav2_smac_planner >/dev/null 2>&1; then
  echo "[ERROR] nav2_smac_planner is not installed."
  echo "Install with: sudo apt install ros-humble-nav2-smac-planner"
  exit 2
fi

cd "$WS"
echo "[BUILD] hospital_nav2 V5 State Lattice baseline"
colcon build --packages-select hospital_nav2 --symlink-install
set +u
source "$WS/install/setup.bash"
set +u

printf '%s\n' \
"============================================================" \
"[TROLLEY NAV2 V5] Smac State Lattice baseline" \
"[CENTERLINE] DISABLED" \
"[V4 CORNER GATE] DISABLED" \
"[PLANNER] nav2_smac_planner/SmacPlannerLattice" \
"[STATE] x + y + yaw / polygon footprint collision checking" \
"[FOOTPRINT] 2.36 m x 1.90 m polygon (existing config)" \
"[GLOBAL SENSOR] /trolley/scan_front (forward 160 deg)" \
"[LOCAL SENSOR] /trolley/scan (full 360 deg)" \
"[CMD PIPE] Nav2 -> /trolley/cmd_vel -> Isaac" \
"[TEST] Compare the FIRST generated /plan at the same corner" \
"============================================================"

exec ros2 launch hospital_nav2 hospital_trolley_lattice_navigation.launch.py
