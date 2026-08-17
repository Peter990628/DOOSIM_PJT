#!/usr/bin/env python3
"""Relay Nav2's final smoothed /cmd_vel to Isaac's stable /trolley/cmd_vel API."""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class TrolleyCmdVelRelay(Node):
    def __init__(self):
        super().__init__('trolley_cmd_vel_relay')
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self.pub = self.create_publisher(Twist, '/trolley/cmd_vel', qos)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self._cb, qos)
        self.get_logger().info('Relay active: /cmd_vel -> /trolley/cmd_vel')

    def _cb(self, msg: Twist):
        # Preserve cooperative-drive contract: no lateral command.
        msg.linear.y = 0.0
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrolleyCmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
