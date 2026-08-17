#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[[ -f "$ROOT/ros2_ws/install/setup.bash" ]] || { echo "[ERROR] 먼저 ./02_build_ros_ws.sh 실행" >&2; exit 1; }
source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set +u
export ROS_DOMAIN_ID=120
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
if [[ -f "$HOME/.ros/fastdds_whitelist.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
fi
echo "============================================================"
echo "[NAV2 DUAL] AMR1 원본 주행 + AMR2 동일 복제 주행"
echo "[DOMAIN] ROS_DOMAIN_ID=120"
echo "[AMR1] /center_goal /cmd_vel /odom /scan"
echo "[AMR2] /amr2/center_goal /amr2/cmd_vel /amr2/odom /amr2/scan"
echo "[AMR2 POSE] fixed x=-47.2788 y=26.5713 yaw=0.0 자동 초기 Pose 잠금"
echo "[CORRIDOR] 같은 좁은 복도 경로가 겹치면 선행 AMR 우선, 이탈 3초 후 대기 AMR 원래 목표 재발행"
echo "============================================================"
exec ros2 launch hospital_nav2 hospital_dual_navigation.launch.py
