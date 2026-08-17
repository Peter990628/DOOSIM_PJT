#!/usr/bin/env python3
"""Standalone ROS 2 bridge for the runtime cooperative trolley.

Publishes:
- /trolley/scan       sensor_msgs/msg/LaserScan
- /trolley/odom       nav_msgs/msg/Odometry
- /tf                 map -> trolley_odom -> trolley_base -> trolley_lidar

Subscribes are handled in isaac_amr_ros.py so Nav2 /trolley/cmd_vel can be
fed into the cooperative cart controller.

For the first Nav2 validation this bridge intentionally uses Isaac ground-truth
cart pose.  map->trolley_odom is identity and trolley_odom->trolley_base uses
absolute Isaac world XY/yaw.  This avoids mixing localization problems with
cooperative-control problems.  AMCL can replace the map->odom source later.
"""
from __future__ import annotations

import math
import time
from typing import Any

import numpy as np
from isaacsim.sensors.physx import _range_sensor
from pxr import Gf, Usd, UsdGeom, UsdPhysics


def _set_stamp(stamp: Any, seconds: float) -> None:
    safe = max(0.0, float(seconds))
    sec = int(safe)
    nanosec = int(round((safe - sec) * 1_000_000_000.0))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    stamp.sec = sec
    stamp.nanosec = nanosec


def _normalize_angle(v: float) -> float:
    return math.atan2(math.sin(v), math.cos(v))


def _yaw_from_matrix(matrix: Gf.Matrix4d) -> float:
    q = matrix.ExtractRotationQuat()
    im = q.GetImaginary()
    x, y, z, w = float(im[0]), float(im[1]), float(im[2]), float(q.GetReal())
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class TrolleyLidarRosBridge:
    def __init__(
        self,
        stage: Usd.Stage,
        node: Any,
        cart_root_path: str,
        lidar_path: str,
        cfg: dict[str, Any],
    ) -> None:
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan
        from tf2_msgs.msg import TFMessage

        self.stage = stage
        self.node = node
        self.cfg = dict(cfg)
        self.cart_root_path = str(cart_root_path)
        self.lidar_path = str(lidar_path)
        self.LaserScan = LaserScan
        self.Odometry = Odometry
        self.Clock = Clock
        self.TransformStamped = TransformStamped
        self.TFMessage = TFMessage

        self.scan_topic = str(self.cfg.get("scan_topic", "/trolley/scan"))
        self.odom_topic = str(self.cfg.get("odom_topic", "/trolley/odom"))
        self.map_frame = str(self.cfg.get("map_frame", "map"))
        self.odom_frame = str(self.cfg.get("odom_frame", "trolley_odom"))
        self.base_frame = str(self.cfg.get("base_frame", "trolley_base"))
        self.scan_frame = str(self.cfg.get("scan_frame", "trolley_lidar"))
        self.tf_topic = str(self.cfg.get("tf_topic", "/tf"))
        self.clock_topic = str(self.cfg.get("clock_topic", "/clock"))
        self.publish_ground_truth_map_tf = bool(self.cfg.get("publish_ground_truth_map_tf", True))

        self.scan_hz = max(1.0, float(self.cfg.get("scan_publish_hz", 10.0)))
        self.odom_hz = max(1.0, float(self.cfg.get("odom_publish_hz", 30.0)))
        self.scan_period = 1.0 / self.scan_hz
        self.odom_period = 1.0 / self.odom_hz
        self.last_scan = -1.0
        self.last_odom = -1.0
        self.latest_clock_time: float | None = None
        self.warned = False

        self.min_range = float(self.cfg.get("min_range_m", 0.15))
        self.max_range = float(self.cfg.get("max_range_m", 12.0))
        translation = list(self.cfg.get("translation", [0.0, 0.0, 1.0]))
        while len(translation) < 3:
            translation.append(0.0)
        self.sensor_translation = tuple(float(v or 0.0) for v in translation[:3])

        self.cart_root = self.stage.GetPrimAtPath(self.cart_root_path)
        if not self.cart_root or not self.cart_root.IsValid():
            raise RuntimeError(f"Trolley root missing: {self.cart_root_path}")
        self.cart_rb = UsdPhysics.RigidBodyAPI(self.cart_root)

        # Canonical trolley QoS.  LaserScan favors fresh sensor samples; odom
        # favors reliable delivery.  TF stays on its existing queue to avoid
        # changing the proven transform transport behavior in this test build.
        try:
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

            scan_qos = QoSProfile(depth=5)
            scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            scan_qos.durability = DurabilityPolicy.VOLATILE

            odom_qos = QoSProfile(depth=10)
            odom_qos.reliability = ReliabilityPolicy.RELIABLE
            odom_qos.durability = DurabilityPolicy.VOLATILE
        except Exception:
            scan_qos = 5
            odom_qos = 10

        self.scan_pub = node.create_publisher(LaserScan, self.scan_topic, scan_qos)
        self.odom_pub = node.create_publisher(Odometry, self.odom_topic, odom_qos)
        self.tf_pub = node.create_publisher(TFMessage, self.tf_topic, 30)

        # Use the exact same ROS simulation clock that Nav2 consumes.  The
        # trolley bridge is created several seconds after the AMR Nav2 bridge,
        # so an independent time.monotonic() origin makes scan/odom/TF stamps
        # lag /clock by that creation delay.  Subscribe to /clock instead.
        try:
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            clock_qos = QoSProfile(depth=10)
            clock_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            clock_qos.durability = DurabilityPolicy.VOLATILE
        except Exception:
            clock_qos = 10
        self.clock_sub = node.create_subscription(
            Clock, self.clock_topic, self._on_clock, clock_qos
        )

        self.lidar = _range_sensor.acquire_lidar_sensor_interface()

        self.prev_pose: tuple[float, float, float] | None = None
        self.prev_pose_time: float | None = None

        print(f"[TROLLEY ROS] CLOCK source={self.clock_topic}")
        print(f"[TROLLEY ROS] PUB {self.scan_topic} frame={self.scan_frame} source={self.lidar_path}")
        print(f"[TROLLEY ROS] PUB {self.odom_topic} frame={self.odom_frame}->{self.base_frame}")
        if self.publish_ground_truth_map_tf:
            print(f"[TROLLEY ROS] GT TF {self.map_frame}->{self.odom_frame} identity")
        print(f"[TROLLEY ROS] TF {self.base_frame}->{self.scan_frame} xyz={self.sensor_translation}")

    def _on_clock(self, msg: Any) -> None:
        self.latest_clock_time = (
            float(msg.clock.sec) + float(msg.clock.nanosec) * 1.0e-9
        )

    def _sim_time(self) -> float | None:
        return self.latest_clock_time

    def publish(self) -> None:
        sim_time = self._sim_time()
        if sim_time is None:
            return
        # Handle simulation Stop/Play or a clock reset cleanly.
        if self.last_odom >= 0.0 and sim_time < self.last_odom:
            self.last_odom = -1.0
            self.last_scan = -1.0
            self.prev_pose = None
            self.prev_pose_time = None
        if self.last_odom < 0.0 or sim_time - self.last_odom >= self.odom_period:
            self._publish_odom_and_tf(sim_time)
            self.last_odom = sim_time
        if self.last_scan < 0.0 or sim_time - self.last_scan >= self.scan_period:
            self._publish_scan(sim_time)
            self.last_scan = sim_time

    def _cart_pose(self) -> tuple[float, float, float]:
        matrix = UsdGeom.Xformable(self.cart_root).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        p = matrix.ExtractTranslation()
        return float(p[0]), float(p[1]), _yaw_from_matrix(matrix)

    def _publish_odom_and_tf(self, sim_time: float) -> None:
        x, y, yaw = self._cart_pose()
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)

        vx = 0.0
        vy = 0.0
        wz = 0.0
        if self.prev_pose is not None and self.prev_pose_time is not None:
            dt = max(1.0e-4, sim_time - self.prev_pose_time)
            dx = x - self.prev_pose[0]
            dy = y - self.prev_pose[1]
            c = math.cos(yaw)
            s = math.sin(yaw)
            # World displacement -> trolley-local velocity.
            vx = (c * dx + s * dy) / dt
            vy = (-s * dx + c * dy) / dt
            wz = _normalize_angle(yaw - self.prev_pose[2]) / dt
        self.prev_pose = (x, y, yaw)
        self.prev_pose_time = sim_time

        odom = self.Odometry()
        _set_stamp(odom.header.stamp, sim_time)
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.02
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = 0.02
        odom.twist.covariance[35] = 0.03
        self.odom_pub.publish(odom)

        transforms = []
        if self.publish_ground_truth_map_tf:
            map_tf = self.TransformStamped()
            _set_stamp(map_tf.header.stamp, sim_time)
            map_tf.header.frame_id = self.map_frame
            map_tf.child_frame_id = self.odom_frame
            map_tf.transform.rotation.w = 1.0
            transforms.append(map_tf)

        base_tf = self.TransformStamped()
        _set_stamp(base_tf.header.stamp, sim_time)
        base_tf.header.frame_id = self.odom_frame
        base_tf.child_frame_id = self.base_frame
        base_tf.transform.translation.x = x
        base_tf.transform.translation.y = y
        base_tf.transform.rotation.z = qz
        base_tf.transform.rotation.w = qw
        transforms.append(base_tf)

        sensor_tf = self.TransformStamped()
        _set_stamp(sensor_tf.header.stamp, sim_time)
        sensor_tf.header.frame_id = self.base_frame
        sensor_tf.child_frame_id = self.scan_frame
        sensor_tf.transform.translation.x = self.sensor_translation[0]
        sensor_tf.transform.translation.y = self.sensor_translation[1]
        sensor_tf.transform.translation.z = self.sensor_translation[2]
        sensor_tf.transform.rotation.w = 1.0
        transforms.append(sensor_tf)

        self.tf_pub.publish(self.TFMessage(transforms=transforms))

    def _publish_scan(self, sim_time: float) -> None:
        try:
            depth = np.asarray(self.lidar.get_linear_depth_data(self.lidar_path), dtype=np.float32)
            azimuth = np.asarray(self.lidar.get_azimuth_data(self.lidar_path), dtype=np.float32)
            if depth.size == 0:
                return

            # Keep the long axis as the horizontal scan axis. PhysX may return
            # (N,1), (1,N), (V,N), or (N,V) depending on sensor configuration.
            if depth.ndim == 2:
                if depth.shape[0] == 1:
                    depth = depth[0, :]
                elif depth.shape[1] == 1:
                    depth = depth[:, 0]
                elif depth.shape[1] >= depth.shape[0]:
                    depth = depth[depth.shape[0] // 2, :]
                else:
                    depth = depth[:, depth.shape[1] // 2]
            depth = depth.reshape(-1)

            if azimuth.size == depth.size:
                angles = azimuth.reshape(-1).astype(np.float64)
                if np.nanmax(np.abs(angles)) > 7.0:
                    angles = np.deg2rad(angles)
                angles = np.arctan2(np.sin(angles), np.cos(angles))
                order = np.argsort(angles)
                angles = angles[order]
                ranges = depth[order]
                angle_min = float(angles[0])
                angle_max = float(angles[-1])
                angle_increment = float((angle_max - angle_min) / max(1, len(ranges) - 1))
            else:
                ranges = depth
                angle_min = -math.pi
                angle_max = math.pi
                angle_increment = float((angle_max - angle_min) / max(1, len(ranges) - 1))

            valid = np.isfinite(ranges) & (ranges >= self.min_range) & (ranges <= self.max_range)
            clean = np.where(valid, ranges, np.inf).astype(np.float32)

            msg = self.LaserScan()
            _set_stamp(msg.header.stamp, sim_time)
            msg.header.frame_id = self.scan_frame
            msg.angle_min = angle_min
            msg.angle_max = angle_max
            msg.angle_increment = angle_increment
            msg.time_increment = 0.0
            msg.scan_time = self.scan_period
            msg.range_min = self.min_range
            msg.range_max = self.max_range
            msg.ranges = clean.tolist()
            self.scan_pub.publish(msg)
        except Exception as exc:
            if not self.warned:
                print(f"[TROLLEY ROS WARNING] scan not ready yet: {exc}")
                self.warned = True
