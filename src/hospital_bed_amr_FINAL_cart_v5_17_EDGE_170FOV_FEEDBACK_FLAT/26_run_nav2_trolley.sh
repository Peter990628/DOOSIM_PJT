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

# This ZIP adds a new launch/config file, so rebuild hospital_nav2 automatically.
cd "$WS"
echo "[BUILD] hospital_nav2 (trolley Nav2 files)"
colcon build --packages-select hospital_nav2 --symlink-install
set +u
source "$WS/install/setup.bash"
set +u

echo "============================================================"
echo "[TROLLEY NAV2] Ground-truth localization first validation"
echo "[ROS_DOMAIN_ID] $ROS_DOMAIN_ID"
echo "[EXPECT] /trolley/scan, /trolley/odom, /trolley/cmd_vel"
echo "[TF] map -> trolley_odom -> trolley_base -> trolley_lidar"
echo "[ORDER] Keep ./03_run_isaac.sh running, then use RViz 2D Goal Pose"
echo "============================================================"
exec ros2 launch hospital_nav2 hospital_trolley_navigation.launch.py
