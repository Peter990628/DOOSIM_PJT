#!/usr/bin/env python3
from __future__ import annotations
import argparse, math, time
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, String

class AutoGoal(Node):
    def __init__(self,x,y,yaw_deg):
        super().__init__('backup_precoupled_auto_goal')
        self.x=x; self.y=y; self.yaw=math.radians(yaw_deg); self.lock=False; self.status=''
        q=QoSProfile(depth=10); q.reliability=ReliabilityPolicy.RELIABLE; q.durability=DurabilityPolicy.TRANSIENT_LOCAL
        self.pub=self.create_publisher(PoseStamped,'/coop/center_goal',10)
        self.create_subscription(Bool,'/coopnav/initial_pose_locked',lambda m:setattr(self,'lock',bool(m.data)),q)
        self.create_subscription(String,'/coop/center_goal/status',lambda m:setattr(self,'status',str(m.data)),q)
    def send(self):
        m=PoseStamped(); m.header.frame_id='map'; m.header.stamp=self.get_clock().now().to_msg(); m.pose.position.x=self.x; m.pose.position.y=self.y
        m.pose.orientation.z=math.sin(self.yaw/2); m.pose.orientation.w=math.cos(self.yaw/2); self.pub.publish(m)

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--x',type=float,required=True); ap.add_argument('--y',type=float,required=True); ap.add_argument('--yaw-deg',type=float,default=0.0); ap.add_argument('--timeout',type=float,default=300.0); a=ap.parse_args()
    rclpy.init(); n=AutoGoal(a.x,a.y,a.yaw_deg)
    start=time.monotonic()
    print(f'[BACKUP GOAL] waiting cooperative pose lock -> ({a.x:.4f},{a.y:.4f}) yaw={a.yaw_deg:.1f}deg')
    while rclpy.ok() and time.monotonic()-start<90:
        rclpy.spin_once(n,timeout_sec=0.1)
        if n.lock and (n.status.startswith('READY') or n.status==''):
            break
    if not n.lock:
        print('[BACKUP GOAL ERROR] pose lock not ready'); n.destroy_node(); rclpy.shutdown(); raise SystemExit(2)
    attempts=0; deadline=time.monotonic()+a.timeout; last_send=0.0
    while rclpy.ok() and time.monotonic()<deadline:
        rclpy.spin_once(n,timeout_sec=0.1)
        now=time.monotonic()
        if attempts==0 or ((n.status.startswith('FAILED') or n.status.startswith('CANCELED')) and now-last_send>2.0):
            attempts+=1; n.send(); last_send=now
            print(f'[BACKUP GOAL SEND] attempt={attempts} target=({a.x:.4f},{a.y:.4f})')
        if n.status.startswith('SUCCEEDED'):
            print('[BACKUP NAV2 SUCCESS] screenshot goal reached; tray remains attached')
            n.destroy_node(); rclpy.shutdown(); return
        if now-last_send>30.0 and attempts<4 and not n.status.startswith('ACTIVE'):
            attempts+=1; n.send(); last_send=now; print(f'[BACKUP GOAL RE-SEND] attempt={attempts}')
    print(f'[BACKUP NAV2 TIMEOUT] last_status={n.status}; Nav2/RViz left running for manual 2D Goal Pose')
    n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
