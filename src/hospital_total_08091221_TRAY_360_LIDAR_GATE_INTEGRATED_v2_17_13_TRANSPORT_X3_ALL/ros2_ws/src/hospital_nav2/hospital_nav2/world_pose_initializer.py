#!/usr/bin/env python3
"""Publish an initial pose from the *actual* Isaac world pose until pose lock succeeds.

V2.12 replaces stale hard-coded docking-station coordinates.  A short stability
filter prevents the pose lock from capturing the robot while PhysX is still
settling at startup.
"""
from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class WorldPoseInitializer(Node):
    def __init__(self) -> None:
        super().__init__('world_pose_initializer')
        self.declare_parameter('robot_name', 'AMR')
        self.declare_parameter('world_pose_topic', '/amr1/world_pose')
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('lock_topic', '/initial_pose_locked')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_period_sec', 0.2)
        self.declare_parameter('fixed_pose_enabled', False)
        self.declare_parameter('fixed_x', 0.0)
        self.declare_parameter('fixed_y', 0.0)
        self.declare_parameter('fixed_yaw', 0.0)
        self.declare_parameter('stable_cycles', 6)
        self.declare_parameter('stable_xy_m', 0.008)
        self.declare_parameter('stable_yaw_deg', 1.0)

        self.robot_name = str(self.get_parameter('robot_name').value)
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter('initialpose_topic').value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('world_pose_topic').value),
            self._on_world_pose,
            20,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('lock_topic').value),
            self._on_lock,
            latched,
        )
        self.fixed_pose_enabled = bool(self.get_parameter('fixed_pose_enabled').value)
        self.latest: dict | None = None
        self.prev: dict | None = None
        self.stable_count = 0
        self.need_stable = max(1, int(self.get_parameter('stable_cycles').value))
        self.stable_xy = max(0.0, float(self.get_parameter('stable_xy_m').value))
        self.stable_yaw = math.radians(max(0.0, float(self.get_parameter('stable_yaw_deg').value)))
        if self.fixed_pose_enabled:
            self.latest = {
                'x': float(self.get_parameter('fixed_x').value),
                'y': float(self.get_parameter('fixed_y').value),
                'yaw': float(self.get_parameter('fixed_yaw').value),
            }
            self.stable_count = self.need_stable
        self.locked = False
        self.reported_ready = False
        period = max(0.1, float(self.get_parameter('publish_period_sec').value))
        self.create_timer(period, self._tick)
        if self.fixed_pose_enabled:
            self.get_logger().info(
                f"{self.robot_name} fixed initial pose waiting: x={self.latest['x']:.4f}, "
                f"y={self.latest['y']:.4f}, yaw={math.degrees(self.latest['yaw']):.1f}deg"
            )
        else:
            self.get_logger().info(
                f"{self.robot_name} actual Isaac world pose -> automatic initial pose; "
                f"wait stable {self.need_stable} samples"
            )

    def _on_world_pose(self, msg: String) -> None:
        if self.fixed_pose_enabled or self.locked:
            return
        try:
            payload = json.loads(msg.data)
            cur = {
                'x': float(payload['x']),
                'y': float(payload['y']),
                'yaw': float(payload['yaw']),
            }
        except Exception as exc:
            self.get_logger().warning(f'{self.robot_name} invalid world pose: {exc}')
            return

        if self.prev is None:
            self.stable_count = 1
        else:
            dxy = math.hypot(cur['x'] - self.prev['x'], cur['y'] - self.prev['y'])
            dyaw = abs(wrap(cur['yaw'] - self.prev['yaw']))
            if dxy <= self.stable_xy and dyaw <= self.stable_yaw:
                self.stable_count += 1
            else:
                self.stable_count = 1
                self.reported_ready = False
        self.prev = cur
        self.latest = cur

    def _on_lock(self, msg: Bool) -> None:
        self.locked = bool(msg.data)
        if self.locked:
            self.get_logger().info(f'{self.robot_name} actual-world initial pose locked')

    def _tick(self) -> None:
        if self.locked or self.latest is None or self.stable_count < self.need_stable:
            return
        if not self.reported_ready:
            self.reported_ready = True
            self.get_logger().info(
                f"{self.robot_name} stable world pose confirmed: "
                f"x={self.latest['x']:.4f}, y={self.latest['y']:.4f}, "
                f"yaw={math.degrees(self.latest['yaw']):.1f}deg -> pose lock"
            )
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = str(self.get_parameter('frame_id').value)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.latest['x']
        msg.pose.pose.position.y = self.latest['y']
        yaw = self.latest['yaw']
        msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorldPoseInitializer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
