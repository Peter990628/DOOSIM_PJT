#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


def norm(a):
    return math.atan2(math.sin(a), math.cos(a))


class TrolleyFrontScanFilter(Node):
    """Publish only the forward 160-degree sector for GLOBAL obstacle marking.

    ROS LaserScan convention is 0 rad = forward.  User-requested 10..170 deg
    frontal sector is therefore represented as -80..+80 deg around the robot's
    forward axis.  Samples outside the sector are set to +inf, preserving scan
    indexing/geometry.
    """
    def __init__(self):
        super().__init__('trolley_front_scan_filter')
        self.declare_parameter('input_topic', '/trolley/scan')
        self.declare_parameter('output_topic', '/trolley/scan_front')
        self.declare_parameter('front_half_angle_deg', 80.0)
        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.half = math.radians(float(self.get_parameter('front_half_angle_deg').value))
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(LaserScan, self.output_topic, qos)
        self.create_subscription(LaserScan, self.input_topic, self.cb, qos)
        self.get_logger().info('Front scan active: %.1f deg total (%.1f..%.1f deg ROS angle)' %
                               (math.degrees(2*self.half), -math.degrees(self.half), math.degrees(self.half)))

    def cb(self, msg):
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        ranges = list(msg.ranges)
        for i in range(len(ranges)):
            a = norm(msg.angle_min + i * msg.angle_increment)
            if abs(a) > self.half:
                ranges[i] = float('inf')
        out.ranges = ranges
        if msg.intensities:
            out.intensities = list(msg.intensities)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    n = TrolleyFrontScanFilter()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
