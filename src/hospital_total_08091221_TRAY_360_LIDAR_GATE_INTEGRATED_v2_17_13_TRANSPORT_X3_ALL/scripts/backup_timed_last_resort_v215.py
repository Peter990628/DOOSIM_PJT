#!/usr/bin/env python3
"""Last resort: publish a time-based known route with no subscriptions/spin."""
import math,time,sys
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

def run(pub,v,w,duration,rate=20):
    m=Twist(); m.linear.x=v; m.angular.z=w; dt=1.0/rate
    t=time.monotonic(); nxt=0
    while time.monotonic()-t<duration:
        pub.publish(m)
        s=int(time.monotonic()-t)
        if s!=nxt:
            print(f'[LAST RESORT] V={v:+.2f} W={w:+.2f} t={s}/{duration:.1f}s',flush=True); nxt=s
        time.sleep(dt)

def main():
    rclpy.init(); n=Node('backup_timed_last_resort_v215'); p=n.create_publisher(Twist,'/coop/cmd_vel',30); time.sleep(1.0)
    # From the known pre-coupled START only. Used only if Nav2 primary and odom fallback never moved.
    d1=(7.7732-1.0)-(-22.69); v1=0.30
    d2=(11.03-1.0)-6.329; v2=0.24
    print('[V2.15 LAST RESORT] time-only route from known START; no spin/subscription.',flush=True)
    run(p,v1,0.0,d1/v1)
    run(p,0.18,-0.18,(math.pi/2)/0.18)
    run(p,v2,0.0,d2/v2)
    z=Twist()
    for _ in range(30): p.publish(z); time.sleep(0.03)
    n.destroy_node(); rclpy.shutdown(); print('[V2.15 LAST RESORT COMPLETE]',flush=True); return 0
if __name__=='__main__': sys.exit(main())
