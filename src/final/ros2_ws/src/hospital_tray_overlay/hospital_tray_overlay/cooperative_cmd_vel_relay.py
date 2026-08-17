#!/usr/bin/env python3
"""Relay Nav2's standard /cmd_vel output to the cooperative cart input."""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CooperativeCmdVelRelay(Node):
    def __init__(self):
        super().__init__('cooperative_cmd_vel_relay')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/coop/cmd_vel')
        inp=str(self.get_parameter('input_topic').value)
        out=str(self.get_parameter('output_topic').value)
        self.pub=self.create_publisher(Twist,out,20)
        self.sub=self.create_subscription(Twist,inp,self._cb,20)
        self.get_logger().info(f'Nav2 velocity relay: {inp} -> {out}')

    def _cb(self,msg:Twist):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node=CooperativeCmdVelRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
