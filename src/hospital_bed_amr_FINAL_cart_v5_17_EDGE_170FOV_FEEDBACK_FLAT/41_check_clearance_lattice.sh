#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=120

echo '===== PLANNER PLUGIN ====='
ros2 param get /planner_server GridBased.plugin || true
echo '===== COST PENALTY ====='
ros2 param get /planner_server GridBased.cost_penalty || true
echo '===== GLOBAL INFLATION ====='
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius || true
ros2 param get /global_costmap/global_costmap inflation_layer.cost_scaling_factor || true
echo '===== TROLLEY FOOTPRINT ====='
ros2 param get /global_costmap/global_costmap footprint || true
echo '===== SENSOR INPUT ====='
ros2 topic info /trolley/scan_front || true
echo '===== PATH ====='
ros2 topic info /plan || true
echo '===== CMD ====='
ros2 topic info /trolley/cmd_vel || true
