#!/usr/bin/env python3
from __future__ import annotations
import argparse,json,time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan

class Probe(Node):
    def __init__(self, session:str):
        super().__init__('tray_runtime_probe')
        self.session=session; self.status=None; self.clock=False; self.scan1=False; self.scan2=False
        lat=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        vol=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(String,'/tray/runtime_status',self.on_status,lat)
        self.create_subscription(Clock,'/clock',lambda m:setattr(self,'clock',True),vol)
        self.create_subscription(LaserScan,'/scan',lambda m:setattr(self,'scan1',True),vol)
        self.create_subscription(LaserScan,'/amr2/scan',lambda m:setattr(self,'scan2',True),vol)
    def on_status(self,msg):
        try:
            d=json.loads(msg.data)
            if str(d.get('session_id'))==self.session: self.status=d
        except Exception: pass
    def ok(self):
        d=self.status or {}
        return bool(d.get('amr1_bridge')) and bool(d.get('amr2_bridge')) and bool(d.get('cart_ready')) and self.clock and self.scan1 and self.scan2

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--session',required=True); ap.add_argument('--timeout',type=float,default=90.0); a=ap.parse_args()
    rclpy.init(); n=Probe(a.session); end=time.monotonic()+a.timeout; last=0
    rc=2
    try:
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=0.2)
            if n.ok():
                print('[RUNTIME READY] session='+a.session+' bridges=AMR1+AMR2 cart=YES clock=YES scan=YES amr2_scan=YES')
                print('[RUNTIME STATUS] '+json.dumps(n.status,ensure_ascii=False))
                rc=0; break
            if time.monotonic()-last>2:
                s=n.status or {}
                print(f"[RUNTIME WAIT] session_match={n.status is not None} bridge1={s.get('amr1_bridge',False)} bridge2={s.get('amr2_bridge',False)} cart={s.get('cart_ready',False)} clock={n.clock} scan={n.scan1} amr2_scan={n.scan2}")
                last=time.monotonic()
        if rc:
            print('[RUNTIME FAIL] current-session Isaac dual bridge/sensors not ready')
    finally:
        n.destroy_node(); rclpy.shutdown()
    raise SystemExit(rc)
if __name__=='__main__': main()
