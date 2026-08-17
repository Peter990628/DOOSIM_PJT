#!/usr/bin/env python3
import json
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class ActualSpeedMonitor(Node):
    def __init__(self):
        super().__init__('v21712_actual_speed_monitor')
        self.last_cmd={}
        self.pose=[None,None]
        self.last_pose=[None,None]
        self.create_subscription(Twist,'/amr1/tray_cmd_vel',lambda m:self.cmd_cb(0,m),20)
        self.create_subscription(Twist,'/amr2/tray_cmd_vel',lambda m:self.cmd_cb(1,m),20)
        self.create_subscription(Twist,'/coop/cmd_vel',self.coop_cb,20)
        self.create_subscription(String,'/amr1/world_pose',lambda m:self.pose_cb(0,m),20)
        self.create_subscription(String,'/amr2/world_pose',lambda m:self.pose_cb(1,m),20)
        print('[V2.17.12 SPEED PROOF] direct tray command + measured world speed monitor',flush=True)

    def cmd_cb(self,i,m):
        now=time.monotonic()
        if now-self.last_cmd.get(f'c{i}',0.0)>0.35:
            print(
                f'[DIRECT COMMAND] AMR{i+1} tray_cmd_vel '
                f'V={m.linear.x:+.3f} W={m.angular.z:+.3f}',
                flush=True
            )
            self.last_cmd[f'c{i}']=now

    def coop_cb(self,m):
        now=time.monotonic()
        if now-self.last_cmd.get('coop',0.0)>0.5:
            print(
                f'[COOP COMMAND] /coop/cmd_vel V={m.linear.x:+.3f} W={m.angular.z:+.3f}',
                flush=True
            )
            self.last_cmd['coop']=now

    def pose_cb(self,i,m):
        try:
            d=json.loads(m.data)
            x=float(d['x']); y=float(d['y'])
        except Exception:
            return
        now=time.monotonic()
        prev=self.last_pose[i]
        self.last_pose[i]=(x,y,now)
        if prev is None:
            return
        dt=now-prev[2]
        if dt<0.15:
            return
        v=math.hypot(x-prev[0],y-prev[1])/max(dt,1e-6)
        if now-self.last_cmd.get(f'p{i}',0.0)>0.5:
            print(f'[ACTUAL SPEED] AMR{i+1} world={v:.3f} m/s',flush=True)
            self.last_cmd[f'p{i}']=now

def main():
    rclpy.init()
    n=ActualSpeedMonitor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=='__main__':
    main()
