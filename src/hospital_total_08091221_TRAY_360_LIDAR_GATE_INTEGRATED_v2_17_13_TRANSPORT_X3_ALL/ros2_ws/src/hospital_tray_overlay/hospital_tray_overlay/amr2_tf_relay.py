#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_msgs.msg import TFMessage

class Amr2TfRelay(Node):
    def __init__(self):
        super().__init__("amr2_tf_display_relay")
        self.pub=self.create_publisher(TFMessage,"/tf",100)
        q=QoSProfile(depth=100); q.reliability=ReliabilityPolicy.RELIABLE; q.durability=DurabilityPolicy.VOLATILE
        self.sub=self.create_subscription(TFMessage,"/amr2/tf",self.pub.publish,q)
        qs=QoSProfile(depth=100); qs.reliability=ReliabilityPolicy.RELIABLE; qs.durability=DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_s=self.create_publisher(TFMessage,"/tf_static",qs)
        self.sub_s=self.create_subscription(TFMessage,"/amr2/tf_static",self.pub_s.publish,qs)
        self.get_logger().info("display-only relay: /amr2/tf -> /tf, /amr2/tf_static -> /tf_static")

def main(args=None):
    rclpy.init(args=args); n=Amr2TfRelay()
    try: rclpy.spin(n)
    finally:
        n.destroy_node(); rclpy.shutdown()
if __name__=="__main__": main()
