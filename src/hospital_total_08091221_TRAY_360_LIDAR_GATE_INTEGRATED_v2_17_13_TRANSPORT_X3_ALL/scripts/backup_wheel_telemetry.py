#!/usr/bin/env python3
from __future__ import annotations
import math
import sys
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

class WheelTelemetry(Node):
    def __init__(self):
        super().__init__('backup_wheel_telemetry')
        self.declare_parameter('wheel_radius_m', 0.075)
        self.declare_parameter('wheel_lever_m', 0.5825)
        self.declare_parameter('lateral_offset_m', 0.425)
        self.r=float(self.get_parameter('wheel_radius_m').value)
        self.L=float(self.get_parameter('wheel_lever_m').value)
        self.y=float(self.get_parameter('lateral_offset_m').value)
        self.v=0.0; self.w=0.0
        self.sub=self.create_subscription(Twist,'/coop/cmd_vel',self.cb,20)
        self.pub=self.create_publisher(MarkerArray,'/coop/wheel_telemetry_markers',10)
        self.timer=self.create_timer(0.10,self.tick)
        self.print_timer=self.create_timer(0.50,self.print_dashboard)
        self.get_logger().info('V2.14 wheel telemetry: /coop/cmd_vel -> RViz MarkerArray + terminal dashboard')

    def cb(self,msg):
        self.v=float(msg.linear.x); self.w=float(msg.angular.z)

    def calc(self,y):
        vi=self.v-self.w*y
        # Runtime formula with vy=0: FL/RL=(vi-L*w)/r, FR/RR=(vi+L*w)/r
        left=(vi-self.L*self.w)/self.r
        right=(vi+self.L*self.w)/self.r
        return vi, {'FL':left,'FR':right,'RL':left,'RR':right}

    def marker(self,mid,text,x,y,z,scale):
        m=Marker(); m.header.frame_id='cooperative_base_link'; m.header.stamp=self.get_clock().now().to_msg()
        m.ns='wheel_telemetry'; m.id=mid; m.type=Marker.TEXT_VIEW_FACING; m.action=Marker.ADD
        m.pose.position.x=x; m.pose.position.y=y; m.pose.position.z=z; m.pose.orientation.w=1.0
        m.scale.z=scale; m.color.r=0.96; m.color.g=0.96; m.color.b=0.96; m.color.a=1.0
        m.text=text; m.lifetime.sec=0; return m

    @staticmethod
    def fmt(ws):
        rpm={k:v*60.0/(2.0*math.pi) for k,v in ws.items()}
        return (f"FL {ws['FL']:+5.2f} | FR {ws['FR']:+5.2f} rad/s\\n"
                f"RL {ws['RL']:+5.2f} | RR {ws['RR']:+5.2f} rad/s\\n"
                f"rpm L {rpm['FL']:+5.1f} / R {rpm['FR']:+5.1f}")

    def tick(self):
        v1,w1=self.calc(+self.y); v2,w2=self.calc(-self.y)
        arr=MarkerArray()
        arr.markers.append(self.marker(0,
            f"COOPERATIVE NAV2 WHEEL KINEMATICS\\nV={self.v:+.3f} m/s   W={self.w:+.3f} rad/s\\n"
            f"v_i = V - W*y_i   |   wheel omega=(v_i +/- L*W)/r",
            0.0,0.0,1.65,0.22))
        arr.markers.append(self.marker(1,f"AMR1  y=+{self.y:.3f}m  v1={v1:+.3f}m/s\\n"+self.fmt(w1),0.0,+0.62,1.12,0.17))
        arr.markers.append(self.marker(2,f"AMR2  y=-{self.y:.3f}m  v2={v2:+.3f}m/s\\n"+self.fmt(w2),0.0,-0.62,1.12,0.17))
        self.pub.publish(arr)

    def print_dashboard(self):
        v1,w1=self.calc(+self.y); v2,w2=self.calc(-self.y)
        sys.stdout.write('\033[2J\033[H')
        sys.stdout.write('DOOSIM / V2.14 BACKUP - DUAL AMR WHEEL ANGULAR VELOCITY\n')
        sys.stdout.write('==========================================================\n')
        sys.stdout.write(f'Cart center: V={self.v:+.3f} m/s  W={self.w:+.3f} rad/s\n')
        sys.stdout.write(f'Equation: v_i = V - W*y_i,  omega_L/R=(v_i +/- L*W)/r\n\n')
        for name,vi,ws in [('AMR1',v1,w1),('AMR2',v2,w2)]:
            sys.stdout.write(f'{name}: v={vi:+.3f} m/s | FL={ws["FL"]:+.2f} FR={ws["FR"]:+.2f} RL={ws["RL"]:+.2f} RR={ws["RR"]:+.2f} rad/s\n')
        sys.stdout.flush()

def main():
    rclpy.init(); n=WheelTelemetry()
    try: rclpy.spin(n)
    finally: n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
