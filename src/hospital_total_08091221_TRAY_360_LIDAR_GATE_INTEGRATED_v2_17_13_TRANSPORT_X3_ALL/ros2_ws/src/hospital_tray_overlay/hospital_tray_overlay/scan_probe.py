#!/usr/bin/env python3
from __future__ import annotations
import argparse, math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan

class Probe(Node):
    def __init__(self, topic: str):
        super().__init__('tray_scan_probe_' + topic.strip('/').replace('/','_'))
        self.topic=topic; self.msg=None
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(LaserScan, topic, self._cb, qos)
    def _cb(self,msg): self.msg=msg

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--topic',required=True)
    ap.add_argument('--timeout',type=float,default=20.0)
    ap.add_argument('--min-rays',type=int,default=600)
    ap.add_argument('--min-span-deg',type=float,default=350.0)
    a=ap.parse_args()
    rclpy.init(); n=Probe(a.topic); end=time.monotonic()+a.timeout; rc=2
    try:
        last=0.0
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=0.2)
            m=n.msg
            if m is not None:
                rays=len(m.ranges)
                span=abs(float(m.angle_increment))*max(0,rays-1)*180.0/math.pi
                finite=sum(1 for x in m.ranges if math.isfinite(float(x)))
                if rays>=a.min_rays and span>=a.min_span_deg:
                    print(f'[SCAN 360 READY] topic={a.topic} rays={rays} span={span:.1f}deg finite={finite} inc={m.angle_increment:.6f}')
                    rc=0; break
                if time.monotonic()-last>1.0:
                    print(f'[SCAN WAIT] topic={a.topic} rays={rays} span={span:.1f}deg need>={a.min_rays}/{a.min_span_deg:.1f}')
                    last=time.monotonic()
        if rc:
            print(f'[SCAN FAIL] topic={a.topic} did not become a full 360 scan')
    finally:
        n.destroy_node(); rclpy.shutdown()
    raise SystemExit(rc)
if __name__=='__main__': main()
