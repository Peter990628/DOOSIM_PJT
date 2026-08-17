#!/usr/bin/env python3
from __future__ import annotations
import argparse,time
import rclpy
from rclpy.node import Node

class Probe(Node):
    def __init__(self): super().__init__('tray_traffic_wiring_probe')
    def names(self,topic):
        try:
            infos=self.get_subscriptions_info_by_topic(topic)
            return sorted({f'{x.node_namespace.rstrip("/")}/{x.node_name}'.replace('//','/') for x in infos})
        except Exception:
            return []

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--timeout',type=float,default=20.0); a=ap.parse_args()
    rclpy.init(); n=Probe(); end=time.monotonic()+a.timeout; rc=2; last=0.0
    try:
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=0.2)
            a1=n.names('/traffic_pause'); a2=n.names('/amr2/traffic_pause')
            ok1=any('centerline_navigator' in x for x in a1)
            ok2=any('centerline_navigator' in x for x in a2)
            if ok1 and ok2:
                print('[TRAFFIC WIRING READY] /traffic_pause -> '+','.join(a1))
                print('[TRAFFIC WIRING READY] /amr2/traffic_pause -> '+','.join(a2))
                rc=0; break
            if time.monotonic()-last>1.5:
                print(f'[TRAFFIC WAIT] amr1_centerline={ok1} nodes={a1} | amr2_centerline={ok2} nodes={a2}')
                last=time.monotonic()
        if rc: print('[TRAFFIC FAIL] both centerline_navigator pause subscriptions were not visible')
    finally:
        n.destroy_node(); rclpy.shutdown()
    raise SystemExit(rc)
if __name__=='__main__': main()
