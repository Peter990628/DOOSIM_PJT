#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-120}"

echo "=== /clock endpoint check ==="
ros2 topic info /clock -v | sed -n '1,80p'

echo
echo "=== live stamp delta (single Python process, 5 samples) ==="
python3 - <<'PY'
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

rclpy.init()
n=Node('trolley_clock_sync_check')
q=QoSProfile(depth=10)
q.reliability=ReliabilityPolicy.BEST_EFFORT
q.durability=DurabilityPolicy.VOLATILE
state={'clock':None,'scan':None,'last_scan':None,'count':0}

def ccb(m):
    state['clock']=m.clock.sec + m.clock.nanosec*1e-9

def scb(m):
    s=m.header.stamp.sec + m.header.stamp.nanosec*1e-9
    state['scan']=s
    if state['clock'] is not None and s != state['last_scan']:
        print(f"clock={state['clock']:.6f} scan={s:.6f} delta={state['clock']-s:+.6f}s")
        state['last_scan']=s
        state['count']+=1

n.create_subscription(Clock,'/clock',ccb,q)
n.create_subscription(LaserScan,'/trolley/scan',scb,q)
while rclpy.ok() and state['count'] < 5:
    rclpy.spin_once(n, timeout_sec=1.0)
n.destroy_node(); rclpy.shutdown()
PY
