#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-120}"

echo "=== TROLLEY LIDAR ROS CHECK ==="
ros2 topic list | grep -E '^/trolley/scan$|^/tf$' || true
echo
ros2 topic info /trolley/scan || true
echo
printf 'Rate (5 sec):\n'
timeout 5 ros2 topic hz /trolley/scan || true
echo
printf 'One scan header/range summary:\n'
timeout 5 ros2 topic echo /trolley/scan --once --field header || true
