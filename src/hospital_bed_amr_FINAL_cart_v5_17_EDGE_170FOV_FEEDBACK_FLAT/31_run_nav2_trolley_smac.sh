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
echo "[BUILD] hospital_nav2 (V4.2 Smac + STRICT Heading Gate)"
colcon build --packages-select hospital_nav2 --symlink-install
set +u
source "$WS/install/setup.bash"
set +u

echo "============================================================"
echo "[TROLLEY NAV2 V4.2] Smac2D + STRICT rotate-before-drive Heading Gate"
echo "[CENTERLINE] DISABLED for this launch"
echo "[CMD PIPE] Nav2 -> /trolley/cmd_vel_raw -> Heading Gate -> /trolley/cmd_vel -> Isaac"
echo "[GATE] >3 deg: linear.x=0, rotate only / <=1 deg: drive allowed"
echo "[GOAL] /trolley/center_goal -> NavigateToPose"
echo "[PLANNER] nav2_smac_planner/SmacPlanner2D"
echo "[COST MULTIPLIER] 2.0"
echo "[GLOBAL INFLATION] unchanged from V3 (radius=0.30, scaling=2.2)"
echo "[ORDER] Keep ./03_run_isaac.sh running, then use RViz 2D Goal Pose"
echo "============================================================"
exec ros2 launch hospital_nav2 hospital_trolley_smac_navigation.launch.py
