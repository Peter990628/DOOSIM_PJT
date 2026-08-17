#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/humble/setup.bash
[[ -f "$ROOT/ros2_ws/install/setup.bash" ]] || { echo '[ERROR] ./02_build_ros_ws.sh 먼저 실행'; exit 1; }
source "$ROOT/ros2_ws/install/setup.bash"
set +u
export ROS_DOMAIN_ID=120
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY=0

echo '=== OCR + ArUco topics ==='
TOPICS="$(ros2 topic list 2>/dev/null || true)"
for T in /amr1/camera/front/color/image_raw /amr1/ocr/result /amr1/aruco/result /amr1/aruco/debug_image /amr1/magnet/status; do
  if grep -Fxq "$T" <<<"$TOPICS"; then echo "[OK] $T"; else echo "[MISS] $T"; fi
done

echo
echo '=== one paired-ArUco result ==='
if grep -Fxq /amr1/aruco/result <<<"$TOPICS"; then
  timeout 4 ros2 topic echo /amr1/aruco/result --once || true
else
  echo '[INFO] ./04_run_ocr_amr1.sh 를 먼저 실행하세요.'
fi

echo
echo '=== ArUco debug view ==='
echo '실시간 박스/ID/중앙선 보기: ./17_view_aruco_debug.sh'
