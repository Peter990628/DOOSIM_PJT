#!/usr/bin/env python3
"""V2.15 direct odometry fallback.

Uses only /coop/odom and /coop/cmd_vel. It follows the same known-clear rounded
L route as the Nav2 primary. It ignores LiDAR collision stops and traffic logic.
This is BACKUP ONLY and does not modify normal project behavior.
"""
from __future__ import annotations
import math, sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

START=(-22.69,11.03); GOAL=(7.7732,6.329); R=1.0
ARC_START=(GOAL[0]-R,START[1]); C=(GOAL[0]-R,START[1]-R); ARC_END=(GOAL[0],START[1]-R)


def norm(a): return math.atan2(math.sin(a),math.cos(a))
def clamp(v,a,b): return max(a,min(b,v))
def yaw_from_q(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

def path_points(sp=0.08):
    pts=[]
    d1=ARC_START[0]-START[0]; n=max(2,int(math.ceil(d1/sp)))
    for i in range(n+1): pts.append((START[0]+d1*i/n,START[1]))
    na=max(12,int(math.ceil((math.pi*R/2)/sp)))
    for i in range(1,na+1):
        ang=math.pi/2-(math.pi/2)*i/na
        pts.append((C[0]+R*math.cos(ang),C[1]+R*math.sin(ang)))
    d2=ARC_END[1]-GOAL[1]; n2=max(2,int(math.ceil(d2/sp)))
    for i in range(1,n2+1): pts.append((GOAL[0],ARC_END[1]-d2*i/n2))
    return pts
PATH=path_points()

class Follower(Node):
    def __init__(self):
        super().__init__('backup_direct_fallback_v215')
        self.odom=None
        self.create_subscription(Odometry,'/coop/odom',self.cb,30)
        self.pub=self.create_publisher(Twist,'/coop/cmd_vel',30)
    def cb(self,m):
        # map->coop_odom is fixed to START, therefore map position = START + odom translation
        q=m.pose.pose.orientation
        self.odom=(START[0]+float(m.pose.pose.position.x),START[1]+float(m.pose.pose.position.y),yaw_from_q(q))
    def stop(self):
        z=Twist()
        for _ in range(20): self.pub.publish(z); time.sleep(0.03)

def spin(n,t=0.05):
    try: rclpy.spin_once(n,timeout_sec=t); return True
    except (ExternalShutdownException,KeyboardInterrupt): return False

def nearest_idx(x,y,begin=0):
    end=min(len(PATH),begin+220)
    best=begin; bd=1e9
    for i in range(begin,end):
        d=(PATH[i][0]-x)**2+(PATH[i][1]-y)**2
        if d<bd: bd=d; best=i
    return best

def main():
    rclpy.init(); n=Follower(); print('=================================================================',flush=True)
    print('[V2.15 DIRECT FALLBACK] independent odometry pure-pursuit',flush=True)
    print('Collision/traffic/ArUco are not command owners in this backup.',flush=True)
    print('Route: straight -> 1m right arc -> nurse goal.',flush=True)
    print('=================================================================',flush=True)
    t0=time.monotonic()
    while n.odom is None and time.monotonic()-t0<10:
        if not spin(n): return 81
    if n.odom is None:
        print('[DIRECT FALLBACK ERROR] /coop/odom unavailable',flush=True); return 31
    idx=0; last_print=0; deadline=time.monotonic()+220
    try:
        while time.monotonic()<deadline:
            if not spin(n): return 82
            if n.odom is None: continue
            x,y,yaw=n.odom
            if math.hypot(GOAL[0]-x,GOAL[1]-y)<0.22:
                n.stop(); print(f'[DIRECT FALLBACK SUCCESS] map≈({x:.3f},{y:.3f})',flush=True); return 0
            idx=nearest_idx(x,y,max(0,idx-10))
            look=min(len(PATH)-1,idx+12)
            tx,ty=PATH[look]
            desired=math.atan2(ty-y,tx-x); err=norm(desired-yaw)
            # Slow only for tight heading error; otherwise keep a visible but stable demo speed.
            if abs(err)>1.05: v=0.03
            elif abs(err)>0.55: v=0.12
            else: v=0.30
            w=clamp(1.15*err,-0.24,0.24)
            cmd=Twist(); cmd.linear.x=v; cmd.angular.z=w; n.pub.publish(cmd)
            now=time.monotonic()
            if now-last_print>1.0:
                print(f'[DIRECT] map=({x:+.2f},{y:+.2f}) yaw={math.degrees(yaw):+.1f}deg '
                      f'path={idx}/{len(PATH)-1} V={v:+.2f} W={w:+.2f}',flush=True)
                last_print=now
        n.stop(); print('[DIRECT FALLBACK TIMEOUT] 220s',flush=True); return 32
    finally:
        try: n.stop(); n.destroy_node()
        except Exception: pass
        try:
            if rclpy.ok(): rclpy.shutdown()
        except Exception: pass

if __name__=='__main__': sys.exit(main())
