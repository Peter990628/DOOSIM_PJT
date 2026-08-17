#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class Monitor(Node):
    def __init__(self):
        super().__init__('v2179_motion_signal_monitor')
        self.last={}
        self.create_subscription(Twist,'/cmd_vel',lambda m:self.cb('AMR1 /cmd_vel',m),20)
        self.create_subscription(Twist,'/amr2/cmd_vel',lambda m:self.cb('AMR2 /amr2/cmd_vel',m),20)
        self.create_subscription(Twist,'/coop/cmd_vel',lambda m:self.cb('COOP /coop/cmd_vel',m),20)
        self.create_subscription(String,'/coop/cart/status',self.cart_cb,20)
        print('[V2.17.9 SIGNAL MONITOR] listening',flush=True)

    def cb(self,name,m):
        now=time.monotonic()
        if now-self.last.get(name,0.0)>0.5:
            print(f'[MOTION SIGNAL] {name}: V={m.linear.x:+.3f} m/s W={m.angular.z:+.3f} rad/s',flush=True)
            self.last[name]=now

    def cart_cb(self,m):
        now=time.monotonic()
        if now-self.last.get('cart',0.0)>1.0:
            print(f'[CART STATUS] {m.data[:280]}',flush=True)
            self.last['cart']=now

def main():
    rclpy.init()
    n=Monitor()
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
