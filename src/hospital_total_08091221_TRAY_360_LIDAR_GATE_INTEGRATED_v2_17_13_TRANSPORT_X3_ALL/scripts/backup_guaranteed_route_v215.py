#!/usr/bin/env python3
"""V2.15 BACKUP: known-clear FollowPath route with automatic fallback trigger.

Primary path is a deterministic rounded L route on the hospital occupancy map:
  START (-22.69, 11.03, 0deg)
  straight east
  1.0 m radius right arc
  GOAL  (7.7732, 6.329)

It sends the path directly to Nav2 controller_server FollowPath. No planner,
pose-lock localizer, ArUco, traffic manager, or live obstacle layer is required.
If the action server is unavailable, the goal is rejected, or the vehicle never
starts moving, this program exits non-zero and the shell runner switches to the
independent direct odometry follower.
"""
from __future__ import annotations
import math, sys, time
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

START=(-22.69,11.03)
GOAL=(7.7732,6.329)
RADIUS=1.0
ARC_START=(GOAL[0]-RADIUS, START[1])
ARC_CENTER=(GOAL[0]-RADIUS, START[1]-RADIUS)
ARC_END=(GOAL[0], START[1]-RADIUS)


def yaw_q(yaw: float):
    return math.sin(yaw/2.0), math.cos(yaw/2.0)


def route_points(spacing=0.10):
    pts=[]
    d1=max(0.0, ARC_START[0]-START[0]); n1=max(2,int(math.ceil(d1/spacing)))
    for i in range(n1+1):
        t=i/n1; pts.append((START[0]+t*d1, START[1], 0.0))
    arc_len=math.pi*RADIUS/2.0; na=max(8,int(math.ceil(arc_len/spacing)))
    for i in range(1,na+1):
        t=i/na; ang=math.pi/2.0-t*math.pi/2.0
        x=ARC_CENTER[0]+RADIUS*math.cos(ang)
        y=ARC_CENTER[1]+RADIUS*math.sin(ang)
        yaw=ang-math.pi/2.0
        pts.append((x,y,yaw))
    d2=max(0.0, ARC_END[1]-GOAL[1]); n2=max(2,int(math.ceil(d2/spacing)))
    for i in range(1,n2+1):
        t=i/n2; pts.append((GOAL[0], ARC_END[1]-t*d2, -math.pi/2.0))
    return pts


class RouteNode(Node):
    def __init__(self):
        super().__init__('backup_guaranteed_route_v215')
        qos=QoSProfile(depth=1)
        qos.reliability=ReliabilityPolicy.RELIABLE
        qos.durability=DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub=self.create_publisher(Path,'/coop/centerline_path',qos)
        self.odom=None; self.last_cmd=(0.0,0.0); self.last_motion=time.monotonic()
        self.create_subscription(Odometry,'/coop/odom',self._odom,20)
        self.create_subscription(Twist,'/coop/cmd_vel',self._cmd,20)
        self.client=ActionClient(self,FollowPath,'/coopnav/follow_path')
    def _odom(self,msg):
        self.odom=(float(msg.pose.pose.position.x),float(msg.pose.pose.position.y))
    def _cmd(self,msg):
        self.last_cmd=(float(msg.linear.x),float(msg.angular.z))
    def make_path(self):
        msg=Path(); msg.header.frame_id='map'; msg.header.stamp=self.get_clock().now().to_msg()
        for x,y,yaw in route_points():
            p=PoseStamped(); p.header=msg.header; p.pose.position.x=x; p.pose.position.y=y
            p.pose.orientation.z,p.pose.orientation.w=yaw_q(yaw); msg.poses.append(p)
        return msg
    def publish_path(self,path):
        path.header.stamp=self.get_clock().now().to_msg()
        for p in path.poses: p.header.stamp=path.header.stamp
        for _ in range(8): self.path_pub.publish(path); time.sleep(0.08)


def spin_once_safe(node, timeout=0.1):
    try:
        rclpy.spin_once(node,timeout_sec=timeout); return True
    except (ExternalShutdownException, KeyboardInterrupt):
        return False


def main():
    rclpy.init(); n=RouteNode(); path=n.make_path()
    print('=================================================================',flush=True)
    print('[V2.15 PRIMARY] NAV2 FOLLOWPATH - GUARANTEED KNOWN ROUTE',flush=True)
    print(f'START={START}  ARC_START={ARC_START}  GOAL={GOAL}',flush=True)
    print('No pose-lock wait. No planner dependency. Live obstacle stop disabled.',flush=True)
    print('If this does not move, shell automatically starts DIRECT FALLBACK.',flush=True)
    print('=================================================================',flush=True)
    n.publish_path(path)
    # Wait briefly for odom so the controller has a live robot base.
    t0=time.monotonic()
    while n.odom is None and time.monotonic()-t0<12.0:
        if not spin_once_safe(n,0.1):
            n.destroy_node(); return 81
    if n.odom is None:
        print('[V2.15 PRIMARY FAIL] /coop/odom unavailable -> fallback',flush=True)
        n.destroy_node(); rclpy.shutdown(); return 21
    if not n.client.wait_for_server(timeout_sec=20.0):
        print('[V2.15 PRIMARY FAIL] /coopnav/follow_path unavailable -> fallback',flush=True)
        n.destroy_node(); rclpy.shutdown(); return 22
    goal=FollowPath.Goal(); goal.path=path; goal.controller_id='FollowPath'; goal.goal_checker_id='general_goal_checker'
    try:
        fut=n.client.send_goal_async(goal)
        while not fut.done():
            if not spin_once_safe(n,0.1): return 82
        gh=fut.result()
        if gh is None or not gh.accepted:
            print('[V2.15 PRIMARY FAIL] FollowPath rejected -> fallback',flush=True); return 23
        print('[V2.15 PRIMARY] FollowPath accepted. Waiting for real motion...',flush=True)
        result_fut=gh.get_result_async()
        initial=n.odom; started=False; start_wait=time.monotonic(); last_xy=initial; last_change=time.monotonic()
        while not result_fut.done():
            if not spin_once_safe(n,0.1): return 83
            if n.odom is not None:
                moved=math.hypot(n.odom[0]-initial[0],n.odom[1]-initial[1])
                step=math.hypot(n.odom[0]-last_xy[0],n.odom[1]-last_xy[1])
                if step>0.025: last_xy=n.odom; last_change=time.monotonic()
                if moved>0.06 or abs(n.last_cmd[0])>0.04 or abs(n.last_cmd[1])>0.05:
                    if not started:
                        print(f'[V2.15 PRIMARY MOTION PASS] odom_moved={moved:.3f} cmd={n.last_cmd}',flush=True)
                    started=True
            if not started and time.monotonic()-start_wait>10.0:
                print('[V2.15 PRIMARY FAIL] Nav2 accepted but no motion for 10s -> cancel + fallback',flush=True)
                try: gh.cancel_goal_async()
                except Exception: pass
                return 24
            if started and time.monotonic()-last_change>18.0:
                print('[V2.15 PRIMARY FAIL] Nav2 stalled for 18s -> cancel + fallback from current odom',flush=True)
                try: gh.cancel_goal_async()
                except Exception: pass
                return 25
        res=result_fut.result()
        status=getattr(res,'status',None)
        print(f'[V2.15 PRIMARY RESULT] action_status={status}',flush=True)
        # GoalStatus.STATUS_SUCCEEDED == 4
        return 0 if status==4 else 26
    except Exception as exc:
        print(f'[V2.15 PRIMARY EXCEPTION] {type(exc).__name__}: {exc} -> fallback',flush=True)
        return 27
    finally:
        try: n.destroy_node()
        except Exception: pass
        try:
            if rclpy.ok(): rclpy.shutdown()
        except Exception: pass

if __name__=='__main__':
    sys.exit(main())
