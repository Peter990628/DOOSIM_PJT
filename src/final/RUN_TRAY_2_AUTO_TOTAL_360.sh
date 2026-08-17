#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$ROOT/ros2_ws/install/setup.bash" ]]; then
  echo '[ERROR] ROS workspace is not built: ros2_ws/install/setup.bash is missing.'
  echo '[FIX] Run ./00_SETUP_TRAY_360_INTEGRATED.sh and wait for [DONE] setup/build complete.'
  exit 1
fi
SESSION_FILE="$ROOT/output/tray_integrated/current_session_id"
[[ -s "$SESSION_FILE" ]] || { echo '[ERROR] Terminal1 session missing. Start RUN_TRAY_1_ISAAC_TOTAL_360.sh first.'; exit 2; }
SESSION="$(cat "$SESSION_FILE")"
source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID=115 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
[[ -f "$HOME/.ros/fastdds_whitelist.xml" ]] && export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
LOG="$ROOT/output/tray_integrated"; mkdir -p "$LOG"
"$ROOT/STOP_TRAY_INTEGRATED_ROS.sh" || true
sleep 2

echo "[PRECHECK] current Isaac session=$SESSION"
timeout 130 ros2 run hospital_tray_overlay runtime_probe --session "$SESSION" --timeout 120
timeout 35 ros2 run hospital_tray_overlay scan_probe --topic /scan --timeout 30 --min-rays 600 --min-span-deg 350
timeout 35 ros2 run hospital_tray_overlay scan_probe --topic /amr2/scan --timeout 30 --min-rays 600 --min-span-deg 350
echo '[PRECHECK PASS] AMR1/AMR2 real 360 scans are healthy.'

P1=''; P2=''; P3=''
show(){ echo "---------------- $1 ----------------"; [[ -f "$2" ]] && tail -n 220 "$2" || true; }
alive(){ [[ -n "${1:-}" ]] && kill -0 "$1" 2>/dev/null; }

# Preserved V2.1 lifecycle fix: do not assume lifecycle_manager autostart succeeded.
# The observed failure was /amr2/map_server == unconfigured[1], which prevents
# /amr2/map and leaves the global costmap at its 5x5m default.
ensure_map_active(){
  local node="$1" pid="$2" label="$3" log="$4"
  echo "[LIFECYCLE GATE] ensuring $node ACTIVE before any map/costmap readiness check"
  if ! timeout 55 ros2 run hospital_tray_overlay lifecycle_bootstrap --node "$node" --timeout 45 --watch-pid "$pid"; then
    echo "[FAIL] $label map_server did not reach ACTIVE"
    show "$label" "$log"
    return 21
  fi
}

stage_amr(){
  local idx="$1" launch="$2" map_node="$3" map_topic="$4" lock_topic="$5" status_topic="$6" log="$7"
  echo "[STAGE $idx] latest hospital_total AMR${idx} Nav2"
  "$launch" >"$log" 2>&1 & local pid=$!
  if [[ "$idx" == 1 ]]; then P1="$pid"; else P2="$pid"; fi
  sleep 1
  alive "$pid" || { echo "[FAIL] AMR${idx} launch exited"; show "AMR${idx}" "$log"; return 11; }

  ensure_map_active "$map_node" "$pid" "AMR${idx}" "$log" || return $?

  timeout 155 ros2 run hospital_tray_overlay map_probe --topic "$map_topic" --timeout 140 --watch-pid "$pid" || { show "AMR${idx}" "$log"; return 12; }
  timeout 115 ros2 run hospital_tray_overlay bool_probe --topic "$lock_topic" --timeout 100 --watch-pid "$pid" || { show "AMR${idx}" "$log"; return 13; }
  timeout 75 ros2 run hospital_tray_overlay string_probe --topic "$status_topic" --timeout 60 --expect-prefix READY --watch-pid "$pid" || { show "AMR${idx}" "$log"; return 14; }
  echo "[PASS] AMR${idx} base Nav2 ready: map_server ACTIVE + map + pose lock + centerline READY"
}

stage_amr 1 "$ROOT/09_run_nav2_amr1.sh" /map_server /map /initial_pose_locked /center_goal/status "$LOG/amr1_nav.log"
stage_amr 2 "$ROOT/09_run_nav2_amr2.sh" /amr2/map_server /amr2/map /amr2/initial_pose_locked /amr2/center_goal/status "$LOG/amr2_nav.log"

echo '[STAGE 3] latest hospital_total path_conflict_manager (unchanged)'
"$ROOT/09_run_collision_avoidance.sh" >"$LOG/path_conflict.log" 2>&1 & P3=$!
sleep 1
alive "$P3" || { echo '[FAIL] path conflict manager exited'; show TRAFFIC "$LOG/path_conflict.log"; exit 31; }
timeout 75 ros2 run hospital_tray_overlay string_probe --topic /traffic_conflict/status --timeout 60 --watch-pid "$P3" || { show TRAFFIC "$LOG/path_conflict.log"; exit 32; }
timeout 30 ros2 run hospital_tray_overlay traffic_probe --timeout 20 || { show TRAFFIC "$LOG/path_conflict.log"; exit 33; }

echo '============================================================'
echo '[BASE READY] both map_servers ACTIVE + final의 공간예약/충돌회피 로직 유지'
echo '[RVIZ] two RViz windows are expected: one per independent AMR before docking.'
echo '[MISSION V2.12.1] both AMRs travel together with short convoy gap -> first PRE_DOCK arrival immediate single-ID ArUco dock -> dual attach -> cooperative Nav2'
echo '[STAFF] doctor=MRI side, woman_doctor=tray-goal side, nurse=desk-chair side (all non-physical)'
echo '============================================================'
set +e
ros2 run hospital_tray_overlay cooperative_transport_manager --config "$ROOT/tray_overlay/config/isaac_config_tray_integrated.json"
RC=$?
set -e
if (( RC != 0 )); then
  echo "[MISSION EXIT] code=$RC. Base Nav2/traffic are left alive for diagnosis."
  show AMR1 "$LOG/amr1_nav.log"; show AMR2 "$LOG/amr2_nav.log"; show TRAFFIC "$LOG/path_conflict.log"
  exit "$RC"
fi
echo '[COMPLETE] tray mission complete; tray remains attached at target.'
