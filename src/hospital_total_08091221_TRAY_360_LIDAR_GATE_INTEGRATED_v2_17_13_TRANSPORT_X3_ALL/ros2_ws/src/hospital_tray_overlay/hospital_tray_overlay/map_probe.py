#!/usr/bin/env python3
from __future__ import annotations
import argparse, os, time
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


def alive(pid:int)->bool:
    if pid <= 0: return True
    try: os.kill(pid,0); return True
    except ProcessLookupError: return False
    except PermissionError: return True


def parse_args():
    p=argparse.ArgumentParser(description='Wait for a valid OccupancyGrid without lifecycle/ros2-CLI assumptions.')
    p.add_argument('--topic',default='/map'); p.add_argument('--timeout',type=float,default=90.0); p.add_argument('--min-cells',type=int,default=100); p.add_argument('--watch-pid',type=int,default=0)
    return p.parse_args()


class Probe(Node):
    def __init__(self,topic,min_cells):
        super().__init__('tray_overlay_map_probe'); self.min_cells=min_cells; self.ok=False; self.msg=None; self.rx=False
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,topic,self.cb,qos)
    def cb(self,msg):
        self.rx=True; cells=int(msg.info.width)*int(msg.info.height)
        if msg.info.width>0 and msg.info.height>0 and len(msg.data)>=self.min_cells and cells==len(msg.data): self.ok=True; self.msg=msg


def main():
    a=parse_args(); rclpy.init(args=[]); n=Probe(a.topic,a.min_cells); end=time.monotonic()+max(.1,a.timeout); last=0.0
    try:
        while rclpy.ok() and time.monotonic()<end and not n.ok:
            if a.watch_pid and not alive(a.watch_pid):
                print(f'[MAP FAIL] watched process pid={a.watch_pid} exited while waiting for {a.topic}'); return 3
            rclpy.spin_once(n,timeout_sec=.20)
            now=time.monotonic()
            if now-last>=2.0 and not n.ok:
                print(f'[MAP WAIT] topic={a.topic} rx={n.rx} process_alive={alive(a.watch_pid)}'); last=now
        if n.ok:
            m=n.msg; print(f'[MAP READY] topic={a.topic} frame={m.header.frame_id} size={m.info.width}x{m.info.height} resolution={m.info.resolution:.3f} cells={len(m.data)}'); return 0
        print(f'[MAP TIMEOUT] topic={a.topic} timeout={a.timeout:.1f}s'); return 2
    finally:
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__': raise SystemExit(main())
