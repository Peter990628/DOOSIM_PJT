#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[[ -f "$ROOT/ros2_ws/install/setup.bash" ]] || { echo "[ERROR] 먼저 ./02_build_ros_ws.sh 실행" >&2; exit 1; }
source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set +u
export ROS_DOMAIN_ID=117
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
AMR1_MAP_TOPIC="${AMR1_MAP_TOPIC:-/map}"
AMR2_MAP_TOPIC="${AMR2_MAP_TOPIC:-/amr2/map}"
if [[ -f "$HOME/.ros/fastdds_whitelist.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
fi

pkill -f path_conflict_manager 2>/dev/null || true
sleep 0.3

echo "============================================================"
echo "[TRAFFIC] AMR1/AMR2 실제 centerline path 충돌 회피"
echo "[DOMAIN] ROS_DOMAIN_ID=117"
echo "[MAP] AMR1=$AMR1_MAP_TOPIC AMR2=$AMR2_MAP_TOPIC"
echo "[RULE 1] OCR/결합/해체/강제직선/엘리베이터 특수동작 AMR = 절대 우선"
echo "[RULE 2] 반대방향/교차 path는 기존 우선권 회피 유지"
echo "[CONVOY V2.10] 같은 방향 공통경로는 두 AMR 동시주행; 약 2.5m 이내에서 뒤차만 잠시 정지"
echo "[CONVOY V2.10] 간격이 3.2m 이상 벌어지면 뒤차 즉시 재출발"
echo "[RESUME] 특수동작 종료 또는 충돌구간 해제 시 기존 최종 목표 재계산"
echo "[STATUS] ros2 topic echo /traffic_conflict/status"
echo "============================================================"
exec ros2 run hospital_nav2 path_conflict_manager --ros-args \
  -p overlap_distance_m:=1.0 \
  -p hold_trigger_distance_m:=4.0 \
  -p release_clearance_m:=1.2 \
  -p release_delay_sec:=2.0 \
  -p tie_distance_m:=0.35 \
  -p same_direction_follow_enabled:=true \
  -p same_direction_dot_threshold:=0.65 \
  -p same_direction_lateral_limit_m:=1.40 \
  -p same_direction_hold_gap_m:=2.50 \
  -p same_direction_release_gap_m:=3.20 \
  -p same_direction_min_longitudinal_m:=0.30 \
  -p amr1_map_topic:="$AMR1_MAP_TOPIC" \
  -p amr2_map_topic:="$AMR2_MAP_TOPIC"
