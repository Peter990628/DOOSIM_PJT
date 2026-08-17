#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
[[ -f "$ROOT/ros2_ws/install/setup.bash" ]] || { echo '[ERROR] ./02_build_ros_ws.sh 먼저 실행하세요.' >&2; exit 1; }
source "$ROOT/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=120
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY=0
if [[ -f "$HOME/.ros/fastdds_whitelist.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
fi
TOPIC=/amr1/aruco/debug_image
echo "[ArUco DEBUG] $TOPIC"
echo "[INFO] rqt_image_view가 열리면 위 토픽을 선택하세요."
exec ros2 run rqt_image_view rqt_image_view
