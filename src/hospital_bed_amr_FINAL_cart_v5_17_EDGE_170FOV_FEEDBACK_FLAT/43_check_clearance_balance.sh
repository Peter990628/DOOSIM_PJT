#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=120
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo '===== V5.13 NODES ====='
timeout 3 ros2 node list | grep -E 'trolley_clearance_navigator|trolley_goal_forwarder|velocity_smoother|trolley_cmd_vel_relay' || true

echo '===== PLANNER / CONTROLLER ====='
timeout 3 ros2 param get /planner_server GridBased.plugin || true
timeout 3 ros2 param get /controller_server FollowPath.plugin || true
timeout 3 ros2 param get /controller_server FollowPath.PathAlignCritic.use_path_orientations || true

echo '===== COSTMAP ====='
timeout 3 ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius || true
timeout 3 ros2 param get /global_costmap/global_costmap update_frequency || true
timeout 3 ros2 param get /global_costmap/global_costmap publish_frequency || true
timeout 3 ros2 param get /global_costmap/global_costmap footprint || true

echo '===== CLEARANCE / DP / SWEPT ====='
for p in w_balance w_min_clearance w_inflation_balance w_deviation w_shift_smooth w_extra_path_length corner_soften_yaw_deg corner_disable_yaw_deg corner_min_center_factor corridor_narrow_clearance_m corridor_open_clearance_m corridor_center_gain start_lock_m trolley_half_width_m collision_cost_threshold obstacle_cost_threshold footprint_sample_step_m swept_linear_step_m optimize_yaw_spacing_deg max_shift_step_m enable_conditional_replan replan_check_period_sec replan_cooldown_sec replan_start_grace_sec dynamic_obstacle_confirmations replan_path_deviation_m replan_deviation_confirmations; do
  timeout 3 ros2 param get /trolley_clearance_navigator "$p" || true
done

echo '===== SPEED LIMITS ====='
timeout 3 ros2 param get /controller_server FollowPath.vx_max || true
timeout 3 ros2 param get /controller_server FollowPath.wz_max || true
timeout 3 ros2 param get /velocity_smoother max_velocity || true
timeout 3 ros2 param get /velocity_smoother max_accel || true

echo '===== PATH TOPICS ====='
timeout 3 ros2 topic info /trolley/raw_plan -v || true
timeout 3 ros2 topic info /trolley/clearance_plan -v || true

echo '===== VELOCITY CHAIN ====='
echo '-- controller raw / smoother input --'
timeout 3 ros2 topic info /cmd_vel_nav -v || true
echo '-- smoother final --'
timeout 3 ros2 topic info /cmd_vel -v || true
echo '-- relay final to Isaac --'
timeout 3 ros2 topic info /trolley/cmd_vel -v || true


echo '===== ISAAC COMMAND OWNERSHIP ====='
CMD_INFO="$(timeout 3 ros2 topic info /cmd_vel -v 2>/dev/null || true)"
printf '%s\n' "$CMD_INFO"
if printf '%s\n' "$CMD_INFO" | grep -q 'Node name: hospital_amr_isaac_bridge'; then
  echo '[ERROR] hospital_amr_isaac_bridge is still subscribed to /cmd_vel -- duplicate command path remains.'
else
  echo '[OK] Isaac is NOT subscribed to /cmd_vel.'
fi
TROLLEY_INFO="$(timeout 3 ros2 topic info /trolley/cmd_vel -v 2>/dev/null || true)"
if printf '%s\n' "$TROLLEY_INFO" | grep -q 'Node name: hospital_amr_isaac_bridge'; then
  echo '[OK] Isaac subscribes to /trolley/cmd_vel.'
else
  echo '[ERROR] Isaac subscription to /trolley/cmd_vel not found.'
fi

echo '===== SENSOR INPUTS ====='
timeout 3 ros2 topic info /trolley/scan -v || true
timeout 3 ros2 topic info /trolley/scan_front -v || true

echo '===== EXPECTATION ====='
echo 'Goal: /trolley/center_goal'
echo 'Raw:  /trolley/raw_plan (State Lattice SE2)'
echo 'Safe: /trolley/clearance_plan -> explicit /spin pre-align -> MPPI FollowPath'
echo 'Static walls come from /map/global costmap; FULL /trolley/scan is filtered to NEW dynamic obstacles for replanning.'
echo 'Open space: raw shortest-route bias. Narrow corridor: aggressive L/R centering. Corners keep at least 50% centering pressure.'
echo 'Green path is always published; optimizer failure falls back visually to the raw lattice path.'
echo 'Initial heading: explicit /spin aligns in place when error >15deg, then MPPI follows. Final goal yaw remains preserved.'
echo 'Isaac must NOT subscribe to /cmd_vel; it must subscribe only to /trolley/cmd_vel for trolley Nav2.'
echo 'trolley_goal_forwarder and trolley_heading_gate must NOT be running.'

echo "===== DOMAIN ====="
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
if [[ "${ROS_DOMAIN_ID:-}" == "120" ]]; then
  echo "[OK] ROS_DOMAIN_ID=120"
else
  echo "[FAIL] ROS_DOMAIN_ID is not 120"
fi

echo "===== V5.14 PREALIGN / STABLE REPLAN ====="
ros2 param get /trolley_clearance_navigator initial_align_threshold_deg || true
ros2 param get /trolley_clearance_navigator initial_heading_sample_distance_m || true
ros2 param get /trolley_clearance_navigator static_match_radius_m || true
ros2 action info /spin || true

ros2 param get /trolley_clearance_navigator start_lock_m || true
ros2 param get /trolley_clearance_navigator corner_min_center_factor || true
ros2 param get /trolley_clearance_navigator replan_path_deviation_m || true
ros2 param get /trolley_clearance_navigator replan_deviation_confirmations || true
