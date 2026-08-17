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

for pkg in nav2_smac_planner nav2_mppi_controller nav2_velocity_smoother; do
  if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "[ERROR] $pkg not found"
    echo "sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup"
    exit 2
  fi
done

# Pin one deterministic differential-drive primitive for this whole run.
echo "[CHECK] pinning differential State Lattice primitive"
SMAC_SHARE="$(ros2 pkg prefix nav2_smac_planner)/share/nav2_smac_planner"
PRIM=$(python3 - "$SMAC_SHARE" <<'PYSEL'
from pathlib import Path
import json, sys
share = Path(sys.argv[1])
items = [p for p in share.rglob('*.json') if 'diff' in str(p).lower()]
if not items:
    raise SystemExit(1)
def score(p):
    t=str(p).lower()
    return (0 if ('5cm' in t or '0.05' in t or '5_cm' in t) else 1,
            0 if ('0.5m' in t or '0.5_m' in t or '0.5' in t) else 1, str(p))
p=sorted(items,key=score)[0]
json.loads(p.read_text())
print(p)
PYSEL
) || true
if [[ -z "$PRIM" ]]; then
  echo "[ERROR] valid differential-drive lattice primitive JSON not found."
  exit 3
fi
export HOSPITAL_LATTICE_FILE="$PRIM"
echo "[OK] pinned primitive: $HOSPITAL_LATTICE_FILE"

cd "$WS"
echo "[CLEAN] removing stale hospital_nav2 build/install cache only"
rm -rf build/hospital_nav2 install/hospital_nav2

echo "[BUILD] hospital_nav2 V5.7"
colcon build --packages-select hospital_nav2 --symlink-install
set +u
source "$WS/install/setup.bash"
set +u

cat <<'EOF'
============================================================
[V5.7 CLEARANCE-DP / MPPI DEMO]
Planner          : SmacPlannerLattice (raw x,y,yaw preserved)
Centerline       : OFF
Corner gate      : OFF
Clearance logic  : adaptive: shortest raw route in open space, strong L/R centering in corridors
Optimization     : Dynamic Programming across full lateral-shift sequence
Collision        : 6 cm full-footprint + swept-footprint between samples
Dynamic obstacles: full 360 /trolley/scan also used by custom optimizer
Replanning       : immediate when new LiDAR/costmap obstacle intersects remaining path
Controller       : MPPI (DiffDrive, footprint-aware CostCritic)
Velocity path    : MPPI -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel -> relay -> /trolley/cmd_vel
Raw path         : /trolley/raw_plan
Optimized path   : /trolley/clearance_plan -> FollowPath
RViz             : raw/optimized paths preconfigured
============================================================
EOF

exec ros2 launch hospital_nav2 hospital_trolley_lattice_navigation.launch.py
