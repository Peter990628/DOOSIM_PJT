#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
set +u
export ROS_DOMAIN_ID=120

echo "[TEST] Forward virtual-trolley command for ~1.5 sec"
timeout 1.5 ros2 topic pub -r 10 /trolley/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.20, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" || true

echo "[STOP]"
timeout 0.6 ros2 topic pub -r 10 /trolley/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" || true
