#!/usr/bin/env python3
from __future__ import annotations
import argparse, math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_q(q):
    # planar yaw from quaternion
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)

class StraightDeparture(Node):
    def __init__(self):
        super().__init__('backup_precoupled_straight_departure')
        self.pose = None
        self.pub = self.create_publisher(Twist, '/coop/cmd_vel', 20)
        self.create_subscription(Odometry, '/coop/odom', self._odom, 30)
    def _odom(self, msg):
        self.pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            yaw_from_q(msg.pose.pose.orientation),
        )
    def command(self, v):
        m=Twist(); m.linear.x=float(v); m.angular.z=0.0; self.pub.publish(m)
    def stop(self, seconds=0.8):
        end=time.monotonic()+seconds
        while rclpy.ok() and time.monotonic()<end:
            self.command(0.0); rclpy.spin_once(self, timeout_sec=0.02); time.sleep(0.03)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--distance', type=float, default=3.0)
    ap.add_argument('--speed', type=float, default=0.20)
    ap.add_argument('--timeout', type=float, default=30.0)
    a=ap.parse_args()
    rclpy.init(); n=StraightDeparture()
    wait_deadline=time.monotonic()+8.0
    while rclpy.ok() and n.pose is None and time.monotonic()<wait_deadline:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.pose is None:
        print('[BACKUP STRAIGHT ERROR] /coop/odom not received')
        n.destroy_node(); rclpy.shutdown(); raise SystemExit(2)
    sx,sy,syaw=n.pose
    print('=================================================================')
    print('[BACKUP STRAIGHT DEPARTURE V2.14.1]')
    print(f'start=({sx:.3f},{sy:.3f}) yaw={math.degrees(syaw):+.1f}deg')
    print(f'command: linear.x={a.speed:.3f} m/s, angular.z=0.000 rad/s')
    print(f'target straight distance={a.distance:.2f}m')
    print('NO Nav2 rotation is allowed during this departure segment.')
    print('=================================================================')
    deadline=time.monotonic()+a.timeout
    last_print=0.0
    while rclpy.ok() and time.monotonic()<deadline:
        rclpy.spin_once(n, timeout_sec=0.02)
        if n.pose is None: continue
        x,y,yaw=n.pose
        d=math.hypot(x-sx,y-sy)
        if d >= a.distance:
            n.stop(1.0)
            print(f'[BACKUP STRAIGHT PASS] moved={d:.3f}m; now hand off to Nav2')
            n.destroy_node(); rclpy.shutdown(); return
        n.command(abs(a.speed))
        now=time.monotonic()
        if now-last_print>0.75:
            print(f'[BACKUP STRAIGHT] moved={d:.2f}/{a.distance:.2f}m yaw={math.degrees(yaw):+.1f}deg W=0.000')
            last_print=now
        time.sleep(0.03)
    n.stop(1.0)
    x,y,yaw=n.pose if n.pose else (sx,sy,syaw)
    d=math.hypot(x-sx,y-sy)
    print(f'[BACKUP STRAIGHT TIMEOUT] moved={d:.3f}m; continue to Nav2 without abort')
    n.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
