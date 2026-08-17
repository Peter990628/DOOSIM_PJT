#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$ROOT/ros2_ws"

source "$ROOT/scripts/clean_ros_env.sh"
set +u
source /opt/ros/humble/setup.bash
set +u
export ROS_DOMAIN_ID=120
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY=0
if [[ -f "$HOME/.ros/fastdds_whitelist.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
fi

if ! ros2 pkg prefix nav2_smac_planner >/dev/null 2>&1; then
  echo "[ERROR] nav2_smac_planner not found"
  echo "sudo apt install ros-humble-nav2-smac-planner"
  exit 2
fi

echo "[CHECK] differential State Lattice primitive"
PRIM=$(find "$(ros2 pkg prefix nav2_smac_planner)/share/nav2_smac_planner" -type f -name '*.json' 2>/dev/null | grep -i diff | head -n 1 || true)
if [[ -z "$PRIM" ]]; then
  echo "[ERROR] differential-drive lattice primitive JSON not found."
  echo "Run: find /opt/ros/humble/share/nav2_smac_planner -type f -name '*.json'"
  exit 3
fi
echo "[OK] primitive: $PRIM"

cd "$WS"
echo "[BUILD] hospital_nav2 V5.1 clearance-lattice demo"
colcon build --packages-select hospital_nav2 --symlink-install
set +u
source "$WS/install/setup.bash"
set +u

cat <<'EOF'
============================================================
[V5.1 CLEARANCE DEMO]
Planner       : SmacPlannerLattice (x,y,yaw)
Centerline    : OFF
Corner gate   : OFF
Footprint     : trolley polygon 2.36m x 1.90m
Global sensor : /trolley/scan_front
Local sensor  : /trolley/scan (360 deg)
Global inflate: radius 1.45m / scale 1.8
Cost penalty  : 3.2  (stronger high-cost avoidance)
TEST          : RViz goal -> inspect /plan -> run same L-corner
============================================================
EOF

exec ros2 launch hospital_nav2 hospital_trolley_lattice_navigation.launch.py
