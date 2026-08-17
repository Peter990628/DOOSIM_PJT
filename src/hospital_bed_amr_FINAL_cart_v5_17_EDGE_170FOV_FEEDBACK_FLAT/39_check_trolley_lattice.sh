#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
set +u
export ROS_DOMAIN_ID=120
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ -f "$ROOT/ros2_ws/install/setup.bash" ]]; then
  source "$ROOT/ros2_ws/install/setup.bash"
fi

echo "=== V5 STATE LATTICE CHECK ==="
echo

echo "[1] Planner plugin"
ros2 param get /planner_server GridBased.plugin || true

echo
echo "[2] Planner lattice file"
ros2 param get /planner_server GridBased.lattice_filepath || true

echo
echo "[3] Final cmd_vel topology (V5 should NOT contain trolley_heading_gate)"
ros2 topic info /trolley/cmd_vel -v || true

echo
echo "[4] Heading/corner gate process check"
if ros2 node list | grep -qx '/trolley_heading_gate'; then
  echo "[FAIL] /trolley_heading_gate is running. Stop old V4.6 Nav2 terminals first."
else
  echo "[OK] /trolley_heading_gate is not running."
fi

echo
echo "[5] Scan split"
ros2 topic info /trolley/scan_front -v || true

echo
echo "[6] Plan topics"
ros2 topic info /plan -v || true

echo
echo "[7] Optional lattice debug topics"
ros2 topic list | grep -E '^/(planned_footprints|expansions)$' || true
