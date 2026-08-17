#!/usr/bin/env python3
from __future__ import annotations
import argparse, os, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool


def alive(pid:int)->bool:
    if pid <= 0: return True
    try: os.kill(pid,0); return True
    except ProcessLookupError: return False
    except PermissionError: return True


class N(Node):
    def __init__(self,t):
        super().__init__('bool_ready_probe'); self.okv=False; self.rx=False
        q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool,t,self.cb,q)
    def cb(self,m):
        self.rx=True; self.okv=bool(m.data)


def main():
    p=argparse.ArgumentParser(); p.add_argument('--topic',required=True); p.add_argument('--timeout',type=float,default=90); p.add_argument('--watch-pid',type=int,default=0); a=p.parse_args()
    rclpy.init(args=[]); n=N(a.topic); end=time.monotonic()+max(.1,a.timeout); last=0.0; rc=2
    try:
        while rclpy.ok() and time.monotonic()<end and not n.okv:
            if a.watch_pid and not alive(a.watch_pid):
                print(f'[BOOL FAIL] watched process pid={a.watch_pid} exited while waiting for {a.topic}'); return 3
            rclpy.spin_once(n,timeout_sec=.2)
            now=time.monotonic()
            if now-last>=2.0 and not n.okv:
                print(f'[BOOL WAIT] topic={a.topic} rx={n.rx} value={n.okv} process_alive={alive(a.watch_pid)}'); last=now
        if n.okv: print('[BOOL READY] '+a.topic+'=true'); rc=0
        else: print('[BOOL FAIL] '+a.topic+' did not become true')
        return rc
    finally:
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__': raise SystemExit(main())
