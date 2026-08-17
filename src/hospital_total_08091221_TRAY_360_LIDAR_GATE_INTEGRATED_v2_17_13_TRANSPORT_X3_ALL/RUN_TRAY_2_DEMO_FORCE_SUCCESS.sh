#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$ROOT/ros2_ws/install/setup.bash" ]]; then
  echo '[ERROR] ROS workspace is not built.'
  echo '[FIX] Run ./00_SETUP_TRAY_360_INTEGRATED.sh first.'
  exit 1
fi

SESSION_FILE="$ROOT/output/tray_integrated_v2_12/current_session_id"
[[ -s "$SESSION_FILE" ]] || SESSION_FILE="$ROOT/output/tray_integrated_v2_6/current_session_id"
[[ -s "$SESSION_FILE" ]] || SESSION_FILE="$ROOT/output/tray_integrated_v2/current_session_id"
[[ -s "$SESSION_FILE" ]] || { echo '[ERROR] Terminal1 session missing. Start RUN_TRAY_1_ISAAC_TOTAL_360.sh first.'; exit 2; }
SESSION="$(cat "$SESSION_FILE")"

source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID=117 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
[[ -f "$HOME/.ros/fastdds_whitelist.xml" ]] && export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"

LOG="$ROOT/output/tray_demo_force_success"; mkdir -p "$LOG"
"$ROOT/STOP_TRAY_INTEGRATED_ROS.sh" || true
sleep 2

echo "[DEMO PRECHECK] Isaac session=$SESSION"
timeout 130 ros2 run hospital_tray_overlay runtime_probe --session "$SESSION" --timeout 120
timeout 35 ros2 run hospital_tray_overlay scan_probe --topic /scan --timeout 30 --min-rays 600 --min-span-deg 350
timeout 35 ros2 run hospital_tray_overlay scan_probe --topic /amr2/scan --timeout 30 --min-rays 600 --min-span-deg 350
echo '[DEMO PRECHECK PASS] real 360 scans healthy.'

P1=''; P2=''
show(){ echo "---------------- $1 ----------------"; [[ -f "$2" ]] && tail -n 180 "$2" || true; }
alive(){ [[ -n "${1:-}" ]] && kill -0 "$1" 2>/dev/null; }
ensure_map_active(){
  local node="$1" pid="$2" label="$3" log="$4"
  timeout 55 ros2 run hospital_tray_overlay lifecycle_bootstrap --node "$node" --timeout 45 --watch-pid "$pid" || { show "$label" "$log"; return 21; }
}
stage_amr(){
  local idx="$1" launch="$2" map_node="$3" map_topic="$4" lock_topic="$5" status_topic="$6" log="$7"
  echo "[DEMO STAGE] AMR${idx} normal Nav2"
  "$launch" >"$log" 2>&1 & local pid=$!
  if [[ "$idx" == 1 ]]; then P1="$pid"; else P2="$pid"; fi
  sleep 1
  alive "$pid" || { echo "[FAIL] AMR${idx} launch exited"; show "AMR${idx}" "$log"; return 11; }
  ensure_map_active "$map_node" "$pid" "AMR${idx}" "$log"
  timeout 155 ros2 run hospital_tray_overlay map_probe --topic "$map_topic" --timeout 140 --watch-pid "$pid" || { show "AMR${idx}" "$log"; return 12; }
  timeout 115 ros2 run hospital_tray_overlay bool_probe --topic "$lock_topic" --timeout 100 --watch-pid "$pid" || { show "AMR${idx}" "$log"; return 13; }
  timeout 75 ros2 run hospital_tray_overlay string_probe --topic "$status_topic" --timeout 60 --expect-prefix READY --watch-pid "$pid" || { show "AMR${idx}" "$log"; return 14; }
  echo "[DEMO PASS] AMR${idx} ready"
}

stage_amr 1 "$ROOT/09_run_nav2_amr1.sh" /map_server /map /initial_pose_locked /center_goal/status "$LOG/amr1_nav.log"
stage_amr 2 "$ROOT/09_run_nav2_amr2.sh" /amr2/map_server /amr2/map /amr2/initial_pose_locked /amr2/center_goal/status "$LOG/amr2_nav.log"

# DEMO ONLY: do NOT start 09_run_collision_avoidance.sh. Clear any old latched pause once.
ros2 topic pub --once --qos-durability transient_local /traffic_pause std_msgs/msg/Bool '{data: false}' >/dev/null 2>&1 || true
ros2 topic pub --once --qos-durability transient_local /amr2/traffic_pause std_msgs/msg/Bool '{data: false}' >/dev/null 2>&1 || true

echo '================================================================='
echo '[DEMO FORCE SUCCESS READY]'
echo '- Original RUN_TRAY_2_AUTO_TOTAL_360.sh is UNCHANGED.'
echo '- Original path_conflict_manager is UNCHANGED and simply NOT launched in this demo runner.'
echo '- Both AMRs can move together; no software traffic pause in this run.'
echo '- Final tray docking: ArUco visual proof -> short straight preview -> EXISTING ALIGN/G exact snap.'
echo '- Physics collision/stall during preview is NON-FATAL; ALIGN handles exact capture pose.'
echo '- Attach retries ALIGN + Lift + FixedJoint up to 3 times.'
echo '================================================================='

set +e
python3 "$ROOT/scripts/demo_force_tray_mission.py" --config "$ROOT/tray_overlay/config/isaac_config_tray_demo_force.json" 2>&1 | tee "$LOG/demo_mission.log"
RC=${PIPESTATUS[0]}
set -e
if (( RC != 0 )); then
  echo "[DEMO MISSION EXIT] code=$RC"
  echo '[NOTE] AMR Nav2 processes are intentionally left alive for inspection.'
  show AMR1 "$LOG/amr1_nav.log"
  show AMR2 "$LOG/amr2_nav.log"
  echo "[LOG] $LOG/demo_mission.log"
  exit "$RC"
fi
echo '[DEMO COMPLETE] force-success tray docking + normal cooperative transport complete.'
