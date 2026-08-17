#!/usr/bin/env python3
"""Always-on RViz LiDAR visualizer for V2.17.8.

Why this exists:
- /coop/scan_left and /coop/scan_right are created lazily only AFTER cart ATTACH.
- During ArUco docking / insertion those topics do not exist.
- Raw Isaac scans /scan and /amr2/scan already exist from startup.

This node combines each raw LaserScan with /amr1/world_pose or /amr2/world_pose,
transforms the valid points directly into Isaac/map world coordinates, and
publishes PointCloud2 messages whose frame_id is 'map'.  RViz therefore needs
no per-AMR TF chain to show LiDAR before attachment.
"""
import json
import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String


class RawLidarMapPoints(Node):
    def __init__(self):
        super().__init__('v217_raw_lidar_map_points')

        self.pose=[None,None]
        self.last_log=[0.0,0.0]

        self.pub=[
            self.create_publisher(PointCloud2,'/viz/amr1_lidar_points',qos_profile_sensor_data),
            self.create_publisher(PointCloud2,'/viz/amr2_lidar_points',qos_profile_sensor_data),
        ]

        self.create_subscription(
            String,'/amr1/world_pose',lambda m:self.pose_cb(0,m),20
        )
        self.create_subscription(
            String,'/amr2/world_pose',lambda m:self.pose_cb(1,m),20
        )
        self.create_subscription(
            LaserScan,'/scan',lambda m:self.scan_cb(0,m),qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan,'/amr2/scan',lambda m:self.scan_cb(1,m),qos_profile_sensor_data
        )

        print('[V2.17.8 LIDAR VIZ] raw /scan + /amr2/scan -> map-frame PointCloud2',flush=True)
        print('[V2.17.8 LIDAR VIZ] outputs: /viz/amr1_lidar_points /viz/amr2_lidar_points',flush=True)

    def pose_cb(self,i,msg):
        try:
            d=json.loads(msg.data)
            self.pose[i]=(
                float(d['x']),
                float(d['y']),
                float(d.get('yaw',0.0)),
                float(d.get('z',0.0)),
            )
        except Exception:
            pass

    @staticmethod
    def cloud(points, stamp):
        msg=PointCloud2()
        msg.header.stamp=stamp
        msg.header.frame_id='map'
        msg.height=1
        msg.width=len(points)
        msg.is_bigendian=False
        msg.is_dense=True
        msg.fields=[
            PointField(name='x',offset=0,datatype=PointField.FLOAT32,count=1),
            PointField(name='y',offset=4,datatype=PointField.FLOAT32,count=1),
            PointField(name='z',offset=8,datatype=PointField.FLOAT32,count=1),
        ]
        msg.point_step=12
        msg.row_step=12*len(points)
        msg.data=b''.join(struct.pack('<fff',float(x),float(y),float(z)) for x,y,z in points)
        return msg

    def scan_cb(self,i,scan):
        pose=self.pose[i]
        if pose is None:
            return

        px,py,yaw,pz=pose
        cy=math.cos(yaw)
        sy=math.sin(yaw)

        # Visual height only; top-down RViz uses XY.
        z=max(0.08,pz+0.32)
        points=[]

        # Keep every valid ray. Typical 360 scan is ~720 points.
        angle=float(scan.angle_min)
        rmin=max(0.02,float(scan.range_min))
        rmax=float(scan.range_max) if math.isfinite(scan.range_max) else 50.0

        for r in scan.ranges:
            rr=float(r)
            if math.isfinite(rr) and rmin <= rr <= rmax:
                lx=rr*math.cos(angle)
                ly=rr*math.sin(angle)

                # Sensor is mounted aligned with AMR base in this project.
                wx=px + cy*lx - sy*ly
                wy=py + sy*lx + cy*ly
                points.append((wx,wy,z))
            angle += float(scan.angle_increment)

        if not points:
            return

        self.pub[i].publish(self.cloud(points,scan.header.stamp))

        now=time.monotonic()
        if now-self.last_log[i] > 2.0:
            print(
                f'[V2.17.8 LIDAR LIVE] AMR{i+1} '
                f'raw_topic={"/scan" if i==0 else "/amr2/scan"} '
                f'points={len(points)} world=({px:+.2f},{py:+.2f})',
                flush=True
            )
            self.last_log[i]=now


def main():
    rclpy.init()
    node=RawLidarMapPoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__=='__main__':
    main()
