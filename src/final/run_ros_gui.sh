#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -f "$ROOT/scripts/clean_ros_env.sh" ]]; then
  source "$ROOT/scripts/clean_ros_env.sh"
fi
set +u
source /opt/ros/humble/setup.bash
set +u
if [[ ! -f "$ROOT/ros2_ws/install/setup.bash" ]]; then
  echo "[ERROR] ROS workspace가 빌드되지 않았습니다. 먼저 ./02_build_ros_ws.sh 를 실행하세요." >&2
  exit 1
fi
set +u
source "$ROOT/ros2_ws/install/setup.bash"
set +u

export ROS_DOMAIN_ID=115
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export GUI_AMR_01_POSE_TOPIC="${GUI_AMR_01_POSE_TOPIC:-/amr1/world_pose}"
export GUI_AMR_02_POSE_TOPIC="${GUI_AMR_02_POSE_TOPIC:-/amr2/world_pose}"
export GUI_POSE_SAMPLE_INTERVAL_SEC="${GUI_POSE_SAMPLE_INTERVAL_SEC:-1.0}"

if ! python3 -c 'import flask, rclpy' >/dev/null 2>&1; then
  echo "[ERROR] python3에서 flask 또는 rclpy를 불러올 수 없습니다." >&2
  echo "        Flask가 없으면: sudo apt install python3-flask" >&2
  exit 1
fi

echo "[DOOSIM GUI] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[DOOSIM GUI] RX: $GUI_AMR_01_POSE_TOPIC, $GUI_AMR_02_POSE_TOPIC"
echo "[DOOSIM GUI] MOVE: GUI가 04_run_ocr_mission_1.sh / 2.sh를 직접 실행"
echo "[DOOSIM GUI] RETURN: /amr1/inspection/complete, /amr2/inspection/complete"
cd "$ROOT"
exec python3 app.py
