#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
set +u
export ROS_DOMAIN_ID=120
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

printf '\n=== planner plugin ===\n'
ros2 param get /planner_server GridBased.plugin || true
printf '\n=== Smac center-cost multiplier ===\n'
ros2 param get /planner_server GridBased.cost_travel_multiplier || true
printf '\n=== centerline node should be absent ===\n'
ros2 node list | grep centerline && echo '[WARN] centerline node is running' || echo '[OK] no centerline node'
printf '\n=== direct goal forwarder ===\n'
ros2 node list | grep trolley_goal_forwarder || true
printf '\n=== path topic ===\n'
ros2 topic info /plan || true
printf '\n=== trolley command ===\n'
ros2 topic info /trolley/cmd_vel -v || true
