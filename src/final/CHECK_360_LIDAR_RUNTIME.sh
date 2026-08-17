#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/scripts/clean_ros_env.sh"
set +u; source /opt/ros/humble/setup.bash; source "$ROOT/ros2_ws/install/setup.bash"; set -u
export ROS_DOMAIN_ID=115 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
ros2 run hospital_tray_overlay scan_probe --topic /scan --timeout 20 --min-rays 600 --min-span-deg 350
ros2 run hospital_tray_overlay scan_probe --topic /amr2/scan --timeout 20 --min-rays 600 --min-span-deg 350
