#!/usr/bin/env bash
set +e

echo '[CLEANUP] stopping tray + hospital Nav2 ROS processes; Isaac is preserved'

# Overlay / launch parents.
pkill -TERM -f 'cooperative_transport_manager' 2>/dev/null
pkill -TERM -f 'tray_aruco_pair_node' 2>/dev/null
pkill -TERM -f 'cooperative_cart_nav.launch.py' 2>/dev/null
pkill -TERM -f 'path_conflict_manager' 2>/dev/null
pkill -TERM -f 'hospital_amr1_navigation.launch.py' 2>/dev/null
pkill -TERM -f 'hospital_amr2_navigation.launch.py' 2>/dev/null
sleep 1

# Child processes can survive a killed ros2 launch parent.  They are the source of
# stale /amr2/map_server and lifecycle-manager races seen in V2.
for pat in \
  '/opt/ros/humble/lib/nav2_map_server/map_server' \
  '/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager' \
  '/opt/ros/humble/lib/nav2_controller/controller_server' \
  '/opt/ros/humble/lib/nav2_smoother/smoother_server' \
  '/opt/ros/humble/lib/nav2_planner/planner_server' \
  '/opt/ros/humble/lib/nav2_behaviors/behavior_server' \
  '/opt/ros/humble/lib/nav2_bt_navigator/bt_navigator' \
  '/opt/ros/humble/lib/nav2_waypoint_follower/waypoint_follower' \
  '/opt/ros/humble/lib/nav2_velocity_smoother/velocity_smoother' \
  '/hospital_nav2/centerline_navigator' \
  '/hospital_nav2/pose_lock_localizer' \
  '/hospital_nav2/path_conflict_manager' \
  '/robot_state_publisher/robot_state_publisher' \
  '/rviz2/rviz2'
do
  pkill -TERM -f "$pat" 2>/dev/null
 done
sleep 2

# Escalate only remaining known ROS children; never match Isaac/python.sh.
for pat in \
  '/opt/ros/humble/lib/nav2_map_server/map_server' \
  '/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager' \
  '/opt/ros/humble/lib/nav2_controller/controller_server' \
  '/opt/ros/humble/lib/nav2_smoother/smoother_server' \
  '/opt/ros/humble/lib/nav2_planner/planner_server' \
  '/opt/ros/humble/lib/nav2_behaviors/behavior_server' \
  '/opt/ros/humble/lib/nav2_bt_navigator/bt_navigator' \
  '/opt/ros/humble/lib/nav2_waypoint_follower/waypoint_follower' \
  '/opt/ros/humble/lib/nav2_velocity_smoother/velocity_smoother' \
  '/hospital_nav2/centerline_navigator' \
  '/hospital_nav2/pose_lock_localizer' \
  '/hospital_nav2/path_conflict_manager'
do
  pkill -KILL -f "$pat" 2>/dev/null
 done

# Refresh ros2 CLI discovery after stale participant cleanup. This does not stop Isaac.
if command -v ros2 >/dev/null 2>&1; then
  timeout 5 ros2 daemon stop >/dev/null 2>&1 || true
  sleep 1
  timeout 5 ros2 daemon start >/dev/null 2>&1 || true
fi
sleep 1

echo '[DONE] stale Nav2/map/lifecycle/RViz children cleaned; Isaac was NOT stopped.'
