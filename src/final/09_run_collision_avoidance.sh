#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[[ -f "$ROOT/ros2_ws/install/setup.bash" ]] || { echo "[ERROR] 먼저 ./02_build_ros_ws.sh 실행" >&2; exit 1; }
source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set +u
export ROS_DOMAIN_ID=115
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
AMR1_MAP_TOPIC="${AMR1_MAP_TOPIC:-/map}"
AMR2_MAP_TOPIC="${AMR2_MAP_TOPIC:-/amr2/map}"
SPECIAL_SPATIAL_ENABLED="${SPECIAL_SPATIAL_ENABLED:-true}"
SPECIAL_TRIGGER_DISTANCE_M="${SPECIAL_TRIGGER_DISTANCE_M:-5.0}"
SPECIAL_RELEASE_DISTANCE_M="${SPECIAL_RELEASE_DISTANCE_M:-6.0}"
SPECIAL_POSE_STALE_SEC="${SPECIAL_POSE_STALE_SEC:-1.0}"
RESERVATION_ENABLED="${RESERVATION_ENABLED:-true}"
RESERVATION_ARBITRATION_SEC="${RESERVATION_ARBITRATION_SEC:-0.4}"
RESERVATION_STALE_SEC="${RESERVATION_STALE_SEC:-2.0}"
if [[ -f "$HOME/.ros/fastdds_whitelist.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
fi

pkill -f path_conflict_manager 2>/dev/null || true
sleep 0.3

echo "============================================================"
echo "[TRAFFIC] AMR1/AMR2 실제 centerline path 충돌 회피"
echo "[DOMAIN] ROS_DOMAIN_ID=115"
echo "[MAP] AMR1=$AMR1_MAP_TOPIC AMR2=$AMR2_MAP_TOPIC"
echo "[RULE 1] 특수동작은 같은 층 + 중심거리 ${SPECIAL_TRIGGER_DISTANCE_M}m 이내에서만 상대 정지"
echo "[SPECIAL] ${SPECIAL_TRIGGER_DISTANCE_M}m 정지 / ${SPECIAL_RELEASE_DISTANCE_M}m 재개 / pose stale ${SPECIAL_POSE_STALE_SEC}s fail-safe"
echo "[RULE 2] 그 외에는 같은 층 + 미래 path 1.0m 이내 겹침 -> 우선권 결정"
echo "[RESERVATION] loaded route=$RESERVATION_ENABLED, arbitration=${RESERVATION_ARBITRATION_SEC}s, stale=${RESERVATION_STALE_SEC}s"
echo "[PRIORITY] 현재 owner > TO_MRI_LOADED > FROM_MRI_LOADED"
echo "[YIELD] 후순위 AMR은 충돌구간 약 4m 전 정지"
echo "[RESUME] 특수동작 종료 또는 선행 AMR 충돌구간 통과 후 기존 최종 목표 재계산"
echo "[STATUS] ros2 topic echo /traffic_conflict/status"
echo "[RESERVATION STATUS] ros2 topic echo /traffic_reservation/status"
echo "============================================================"
exec ros2 run hospital_nav2 path_conflict_manager --ros-args \
  -p overlap_distance_m:=1.0 \
  -p hold_trigger_distance_m:=4.0 \
  -p release_clearance_m:=1.2 \
  -p release_delay_sec:=2.0 \
  -p tie_distance_m:=0.35 \
  -p amr1_map_topic:="$AMR1_MAP_TOPIC" \
  -p amr2_map_topic:="$AMR2_MAP_TOPIC" \
  -p special_spatial_enabled:="$SPECIAL_SPATIAL_ENABLED" \
  -p special_trigger_distance_m:="$SPECIAL_TRIGGER_DISTANCE_M" \
  -p special_release_distance_m:="$SPECIAL_RELEASE_DISTANCE_M" \
  -p special_pose_stale_sec:="$SPECIAL_POSE_STALE_SEC" \
  -p reservation_enabled:="$RESERVATION_ENABLED" \
  -p reservation_arbitration_sec:="$RESERVATION_ARBITRATION_SEC" \
  -p reservation_stale_sec:="$RESERVATION_STALE_SEC"
