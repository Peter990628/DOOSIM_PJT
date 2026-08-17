#!/usr/bin/env python3
"""Single-Nav2 bridge for the pre-coupled two-AMR warehouse cart.

The existing per-AMR Nav2Bridge instances still own the two PhysX lidars.  This
bridge reuses those physical sensors but exposes one virtual mobile base:

  /coop/cmd_vel  -> virtual cart center twist
  /coop/odom     -> coop_odom -> cooperative_base_link
  /coop/scan_left, /coop/scan_right -> two observation sources

Each scan is expressed in a cooperative lidar frame whose transform is published
relative to the cart center.  Hits on the cart itself are removed from the scan.
"""
from __future__ import annotations

import math
import time
from typing import Any

import numpy as np
from nav2_bridge import Nav2Bridge
from pxr import Gf, Usd, UsdGeom


def _normalize_angle(v: float) -> float:
    return math.atan2(math.sin(v), math.cos(v))


def _yaw_from_matrix(matrix: Gf.Matrix4d) -> float:
    q = matrix.ExtractRotationQuat()
    qw = float(q.GetReal())
    qx, qy, qz = (float(x) for x in q.GetImaginary())
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _set_stamp(stamp: Any, seconds: float) -> None:
    safe = max(0.0, float(seconds))
    sec = int(safe)
    nsec = int(round((safe - sec) * 1_000_000_000.0))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    stamp.sec = sec
    stamp.nanosec = nsec


class CooperativeNav2Bridge:
    def __init__(
        self,
        stage: Usd.Stage,
        cart_controller: Any,
        source_bridges: list[Any],
        node: Any,
        cfg: dict[str, Any],
    ) -> None:
        from geometry_msgs.msg import TransformStamped, Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        from tf2_msgs.msg import TFMessage

        if len(source_bridges) < 2:
            raise RuntimeError("CooperativeNav2Bridge requires the two AMR lidar bridges")

        self.stage = stage
        self.cart = cart_controller
        self.sources = source_bridges[:2]
        self.node = node
        self.cfg = cfg
        self.Twist = Twist
        self.Odometry = Odometry
        self.LaserScan = LaserScan
        self.TransformStamped = TransformStamped
        self.TFMessage = TFMessage

        self.cmd_vel_topic = str(cfg.get("cmd_vel_topic", "/coop/cmd_vel"))
        self.odom_topic = str(cfg.get("odom_topic", "/coop/odom"))
        self.scan_topics = [
            str(cfg.get("scan_left_topic", "/coop/scan_left")),
            str(cfg.get("scan_right_topic", "/coop/scan_right")),
        ]
        self.odom_frame = str(cfg.get("odom_frame", "coop_odom"))
        self.base_frame = str(cfg.get("base_frame", "cooperative_base_link"))
        self.scan_frames = [
            str(cfg.get("scan_left_frame", "coop_lidar_left")),
            str(cfg.get("scan_right_frame", "coop_lidar_right")),
        ]
        self.tf_topic = str(cfg.get("tf_topic", "/tf"))
        self.command_timeout = float(cfg.get("command_timeout_sec", 0.5))
        self.max_linear = float(cfg.get("max_linear_speed_mps", 0.45))
        self.max_angular = float(cfg.get("max_angular_speed_rad_s", 0.35))
        self.odom_period = 1.0 / max(1.0, float(cfg.get("odom_publish_hz", 30.0)))
        self.scan_period = 1.0 / max(1.0, float(cfg.get("scan_publish_hz", 10.0)))
        self.self_filter_margin = float(cfg.get("self_filter_margin_m", 0.06))
        self.self_filter_max_z = float(cfg.get("self_filter_max_z_m", 0.50))

        self.latest_command = (0.0, 0.0, 0.0)
        self.last_command_wall_time = -1.0
        self.clock_start_wall_time = time.monotonic()
        self.last_odom_sim_time = -1.0
        self.last_scan_sim_time = -1.0

        self.cmd_subscription = node.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 20)
        self.odom_publisher = node.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_publisher = node.create_publisher(TFMessage, self.tf_topic, 30)
        self.scan_publishers = [
            node.create_publisher(LaserScan, self.scan_topics[0], 10),
            node.create_publisher(LaserScan, self.scan_topics[1], 10),
        ]

        initial = self._cart_world_matrix()
        self.initial_position = initial.ExtractTranslation()
        self.initial_yaw = _yaw_from_matrix(initial)

        self._raw_anchor = (0.0, 0.0, 0.0)
        self._odom_anchor = (0.0, 0.0, 0.0)
        self._odom_output = (0.0, 0.0, 0.0)
        self._was_moving = False

        print(f"[COOP NAV2 SUB] {self.cmd_vel_topic}")
        print(f"[COOP NAV2 PUB] {self.odom_topic} frame={self.odom_frame}->{self.base_frame}")
        print(f"[COOP NAV2 PUB] {self.scan_topics[0]} frame={self.scan_frames[0]} source={self.sources[0].lidar_path}")
        print(f"[COOP NAV2 PUB] {self.scan_topics[1]} frame={self.scan_frames[1]} source={self.sources[1].lidar_path}")
        print("[COOP NAV2] dual lidar self-filter enabled; one virtual cart base drives both AMRs")

    def _cart_world_matrix(self) -> Gf.Matrix4d:
        return UsdGeom.Xformable(self.cart.cart_root).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _lidar_world_matrix(self, index: int) -> Gf.Matrix4d:
        prim = self.stage.GetPrimAtPath(self.sources[index].lidar_path)
        if not prim or not prim.IsValid():
            raise RuntimeError(f"Missing lidar prim: {self.sources[index].lidar_path}")
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _on_cmd_vel(self, msg: Any) -> None:
        vx = max(-self.max_linear, min(self.max_linear, float(msg.linear.x)))
        wz = max(-self.max_angular, min(self.max_angular, float(msg.angular.z)))
        self.latest_command = (vx, 0.0, wz)
        self.last_command_wall_time = time.monotonic()

    def get_fresh_command(self) -> tuple[float, float, float] | None:
        if self.last_command_wall_time < 0.0:
            return None
        if time.monotonic() - self.last_command_wall_time > self.command_timeout:
            return None
        return self.latest_command

    def _sim_time(self) -> float:
        # Use AMR1 bridge's clock epoch so /clock, cooperative TF, odom and scans
        # are stamped on the same simulation timeline even if cart construction took seconds.
        try:
            return float(self.sources[0]._sim_time())
        except Exception:
            return max(0.0, time.monotonic() - self.clock_start_wall_time)

    def _relative_pose(self) -> tuple[float, float, float]:
        m = self._cart_world_matrix()
        p = m.ExtractTranslation()
        yaw_world = _yaw_from_matrix(m)
        dx = float(p[0] - self.initial_position[0])
        dy = float(p[1] - self.initial_position[1])
        c, s = math.cos(self.initial_yaw), math.sin(self.initial_yaw)
        x = c * dx + s * dy
        y = -s * dx + c * dy
        return x, y, _normalize_angle(yaw_world - self.initial_yaw)

    @staticmethod
    def _pose_delta(anchor: tuple[float,float,float], current: tuple[float,float,float]) -> tuple[float,float,float]:
        ax,ay,ayaw=anchor; cx,cy,cyaw=current
        dx,dy=cx-ax,cy-ay; c,s=math.cos(ayaw),math.sin(ayaw)
        return c*dx+s*dy, -s*dx+c*dy, _normalize_angle(cyaw-ayaw)

    @staticmethod
    def _pose_compose(anchor: tuple[float,float,float], delta: tuple[float,float,float]) -> tuple[float,float,float]:
        ax,ay,ayaw=anchor; dx,dy,dyaw=delta; c,s=math.cos(ayaw),math.sin(ayaw)
        return ax+c*dx-s*dy, ay+s*dx+c*dy, _normalize_angle(ayaw+dyaw)

    def _stable_pose(self) -> tuple[float,float,float]:
        raw=self._relative_pose()
        fresh=self.get_fresh_command()
        moving = fresh is not None and (abs(fresh[0]) > 0.004 or abs(fresh[2]) > 0.006)
        if moving:
            if not self._was_moving:
                self._raw_anchor=raw; self._odom_anchor=self._odom_output; self._was_moving=True
            self._odom_output=self._pose_compose(self._odom_anchor,self._pose_delta(self._raw_anchor,raw))
        else:
            if self._was_moving:
                self._odom_output=self._pose_compose(self._odom_anchor,self._pose_delta(self._raw_anchor,raw))
            self._raw_anchor=raw; self._odom_anchor=self._odom_output; self._was_moving=False
        return self._odom_output

    def _base_to_lidar_tf(self, index: int, sim_time: float) -> Any:
        base = self._cart_world_matrix()
        lidar = self._lidar_world_matrix(index)
        base_inv = base.GetInverse()
        lp = base_inv.Transform(lidar.ExtractTranslation())
        yaw_rel = _normalize_angle(_yaw_from_matrix(lidar) - _yaw_from_matrix(base))
        t = self.TransformStamped()
        _set_stamp(t.header.stamp, sim_time)
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.scan_frames[index]
        t.transform.translation.x = float(lp[0])
        t.transform.translation.y = float(lp[1])
        t.transform.translation.z = float(lp[2])
        t.transform.rotation.z = math.sin(yaw_rel * 0.5)
        t.transform.rotation.w = math.cos(yaw_rel * 0.5)
        return t

    def _publish_odom_tf(self, sim_time: float) -> None:
        x,y,yaw=self._stable_pose()
        qz,qw=math.sin(yaw*0.5),math.cos(yaw*0.5)
        cmd=self.get_fresh_command() or (0.0,0.0,0.0)

        odom=self.Odometry()
        _set_stamp(odom.header.stamp,sim_time)
        odom.header.frame_id=self.odom_frame
        odom.child_frame_id=self.base_frame
        odom.pose.pose.position.x=x; odom.pose.pose.position.y=y
        odom.pose.pose.orientation.z=qz; odom.pose.pose.orientation.w=qw
        odom.twist.twist.linear.x=float(cmd[0]); odom.twist.twist.angular.z=float(cmd[2])
        odom.pose.covariance[0]=0.025; odom.pose.covariance[7]=0.025; odom.pose.covariance[35]=0.05
        self.odom_publisher.publish(odom)

        base_tf=self.TransformStamped()
        _set_stamp(base_tf.header.stamp,sim_time)
        base_tf.header.frame_id=self.odom_frame
        base_tf.child_frame_id=self.base_frame
        base_tf.transform.translation.x=x; base_tf.transform.translation.y=y
        base_tf.transform.rotation.z=qz; base_tf.transform.rotation.w=qw
        self.tf_publisher.publish(self.TFMessage(transforms=[base_tf,self._base_to_lidar_tf(0,sim_time),self._base_to_lidar_tf(1,sim_time)]))

    def _scan_arrays(self, source: Any) -> tuple[np.ndarray,float,float,float] | None:
        interface=source.lidar_interface
        depth=np.asarray(interface.get_linear_depth_data(source.lidar_path),dtype=np.float32)
        az=np.asarray(interface.get_azimuth_data(source.lidar_path),dtype=np.float32)
        if depth.size == 0:
            return None
        res_deg=float(source.cfg.get("lidar",{}).get("horizontal_resolution_deg",0.5))
        expected=max(2,int(round(360.0/max(1e-6,res_deg))))
        ranges,angles=Nav2Bridge._extract_horizontal_scan(depth,az,expected)
        if ranges.size == 0:
            return None
        if angles is not None and angles.size == ranges.size:
            if np.nanmax(np.abs(angles)) > 7.0:
                angles=np.deg2rad(angles)
            angles=np.arctan2(np.sin(angles),np.cos(angles))
            order=np.argsort(angles); angles=angles[order]; ranges=ranges[order]
            amin=float(angles[0]); amax=float(angles[-1]); inc=float((amax-amin)/max(1,len(ranges)-1))
        else:
            amin=-math.pi; inc=float((2.0*math.pi)/max(1,len(ranges))); amax=float(amin+inc*max(0,len(ranges)-1))
        return ranges, amin, amax, inc

    def _filter_self_hits(self,index:int,ranges:np.ndarray,angle_min:float,angle_increment:float)->np.ndarray:
        source_cfg=self.sources[index].cfg.get("lidar",{})
        rmin=float(source_cfg.get("min_range_m",0.15)); rmax=float(source_cfg.get("max_range_m",12.0))
        clean=np.where(np.isfinite(ranges)&(ranges>=rmin)&(ranges<=rmax),ranges,np.inf).astype(np.float32)
        finite=np.where(np.isfinite(clean))[0]
        if finite.size == 0:
            return clean
        lidar_m=self._lidar_world_matrix(index)
        cart_inv=self._cart_world_matrix().GetInverse()
        half_l=0.5*float(self.cart.meta.get("length",2.20))+self.self_filter_margin
        half_w=0.5*float(self.cart.meta.get("width",1.74))+self.self_filter_margin
        for j in finite.tolist():
            r=float(clean[j]); a=float(angle_min+j*angle_increment)
            wp=lidar_m.Transform(Gf.Vec3d(r*math.cos(a),r*math.sin(a),0.0))
            cp=cart_inv.Transform(wp)
            if abs(float(cp[0])) <= half_l and abs(float(cp[1])) <= half_w and float(cp[2]) <= self.self_filter_max_z:
                clean[j]=np.inf
        return clean

    def _publish_scan(self,index:int,sim_time:float)->None:
        arr=self._scan_arrays(self.sources[index])
        if arr is None:
            return
        ranges,amin,amax,inc=arr
        clean=self._filter_self_hits(index,ranges,amin,inc)
        source_cfg=self.sources[index].cfg.get("lidar",{})
        msg=self.LaserScan()
        _set_stamp(msg.header.stamp,sim_time)
        msg.header.frame_id=self.scan_frames[index]
        msg.angle_min=amin; msg.angle_max=amax; msg.angle_increment=inc
        msg.time_increment=0.0; msg.scan_time=float(self.scan_period)
        msg.range_min=float(source_cfg.get("min_range_m",0.15)); msg.range_max=float(source_cfg.get("max_range_m",12.0))
        msg.ranges=clean.tolist(); msg.intensities=[]
        self.scan_publishers[index].publish(msg)

    def publish(self)->None:
        sim_time=self._sim_time()
        if self.last_odom_sim_time < 0.0 or sim_time-self.last_odom_sim_time >= self.odom_period:
            self._publish_odom_tf(sim_time); self.last_odom_sim_time=sim_time
        if self.last_scan_sim_time < 0.0 or sim_time-self.last_scan_sim_time >= self.scan_period:
            self._publish_scan(0,sim_time); self._publish_scan(1,sim_time); self.last_scan_sim_time=sim_time

    def close(self)->None:
        try: self.node.destroy_subscription(self.cmd_subscription)
        except Exception: pass
        for pub in [self.odom_publisher,self.tf_publisher,*self.scan_publishers]:
            try: self.node.destroy_publisher(pub)
            except Exception: pass
