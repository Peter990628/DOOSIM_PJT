#!/usr/bin/env python3
from __future__ import annotations
import json, math, sys, time
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

START_CART=(-22.69,11.03); GOAL=(7.7732,6.329)
CART_LENGTH=2.20; MARKER_STANDOFF=1.50
INSERT_DISTANCE=0.5*CART_LENGTH+MARKER_STANDOFF
INSERT_SPEED=0.13; DOCK_Y=0.425
WHEEL_R=0.075; WHEEL_L=0.5825

# V2.16.5 FINAL DEMO transport tuning.
# ArUco scan and straight insertion remain intentionally slow/stable.
TRANSPORT_V_FAST=0.66
TRANSPORT_V_MID=0.24
TRANSPORT_V_TIGHT=0.10
TRANSPORT_W_GAIN=2.70
TRANSPORT_W_MAX=0.44
DETACH_GRACE_SEC=2.0
DETACH_RECOVERY_WAIT_SEC=3.5


def norm(a): return math.atan2(math.sin(a),math.cos(a))
def clamp(v,lo,hi): return max(lo,min(hi,v))
def yaw_q(y): return math.sin(y*0.5),math.cos(y*0.5)
def safe_spin(n,t=0.05):
    try: rclpy.spin_once(n,timeout_sec=t); return True
    except (ExternalShutdownException,KeyboardInterrupt): return False

def make_safe_route(spacing=0.08):
    pts=[]; x0,y0=START_CART; x1,y1=-17.0,10.60
    n=max(2,int(math.ceil((x1-x0)/spacing)))
    for i in range(n+1):
        t=i/n; s=3*t*t-2*t*t*t; ds=6*t-6*t*t
        x=x0+(x1-x0)*t; y=y0+(y1-y0)*s
        yaw=math.atan2((y1-y0)*ds,(x1-x0)); pts.append((x,y,yaw))
    R=1.40; arc_start=(GOAL[0]-R,y1)
    d=max(0.0,arc_start[0]-x1); n=max(2,int(math.ceil(d/spacing)))
    for i in range(1,n+1): pts.append((x1+d*i/n,y1,0.0))
    cx,cy=arc_start[0],y1-R; n=max(16,int(math.ceil((math.pi*R/2)/spacing)))
    for i in range(1,n+1):
        a=math.pi/2-(math.pi/2)*(i/n)
        pts.append((cx+R*math.cos(a),cy+R*math.sin(a),a-math.pi/2))
    d=max(0.0,cy-GOAL[1]); n=max(2,int(math.ceil(d/spacing)))
    for i in range(1,n+1): pts.append((GOAL[0],cy-d*i/n,-math.pi/2))
    return pts
SAFE_PATH=make_safe_route()

class Demo(Node):
    def __init__(self):
        super().__init__('v216_scan_straight_attach_transport')
        self.amr_pose=[None,None]; self.scan_good=[0,0]; self.scan_ids=[[],[]]
        self.cart=None; self.cart_wall=-1.0
        self.cmd1=self.create_publisher(Twist,'/cmd_vel',20)
        self.cmd2=self.create_publisher(Twist,'/amr2/cmd_vel',20)
        self.coop=self.create_publisher(Twist,'/coop/cmd_vel',30)
        self.cartcmd=self.create_publisher(String,'/coop/cart/command',10)
        qos=rclpy.qos.QoSProfile(depth=1); qos.reliability=rclpy.qos.ReliabilityPolicy.RELIABLE; qos.durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub=self.create_publisher(Path,'/coop/v216_safe_path',qos)
        self.mark_pub=self.create_publisher(MarkerArray,'/coop/v216_status_markers',10)
        self.create_subscription(String,'/amr1/world_pose',lambda m:self.pose_cb(0,m),20)
        self.create_subscription(String,'/amr2/world_pose',lambda m:self.pose_cb(1,m),20)
        self.create_subscription(String,'/amr1/tray_aruco/result',lambda m:self.scan_cb(0,m),20)
        self.create_subscription(String,'/amr2/tray_aruco/result',lambda m:self.scan_cb(1,m),20)
        self.create_subscription(String,'/coop/cart/status',self.cart_cb,20)
        self.publish_path()
    def pose_cb(self,i,m):
        try:
            d=json.loads(m.data); self.amr_pose[i]=(float(d['x']),float(d['y']),float(d.get('yaw',0.0)))
        except Exception: pass
    def scan_cb(self,i,m):
        try: d=json.loads(m.data)
        except Exception: return
        expected={40,41,44} if i==0 else {42,43,44}; vis={int(v) for v in d.get('visible_ids',[])}
        good=bool(vis & expected) and str(d.get('state','')).upper() in {'SINGLE','PAIR'}
        self.scan_ids[i]=sorted(vis); self.scan_good[i]=self.scan_good[i]+1 if good else 0
    def cart_cb(self,m):
        try:
            d=json.loads(m.data); cp=d.get('cart_pose') or {}
            if all(k in cp for k in ('x','y','yaw')):
                self.cart=(float(cp['x']),float(cp['y']),float(cp['yaw']),bool(d.get('attached',False))); self.cart_wall=time.monotonic()
        except Exception: pass
    def publish_path(self):
        p=Path(); p.header.frame_id='map'; p.header.stamp=self.get_clock().now().to_msg()
        for x,y,yaw in SAFE_PATH:
            s=PoseStamped(); s.header=p.header; s.pose.position.x=x; s.pose.position.y=y; s.pose.orientation.z,s.pose.orientation.w=yaw_q(yaw); p.poses.append(s)
        for _ in range(6): self.path_pub.publish(p); time.sleep(0.02)
    def two(self,v1,v2):
        a=Twist(); a.linear.x=float(v1); b=Twist(); b.linear.x=float(v2); self.cmd1.publish(a); self.cmd2.publish(b)
    def stop_two(self):
        for _ in range(15): self.two(0,0); time.sleep(0.02)
    def cart_json(self,c,rid):
        m=String(); m.data=json.dumps({'command':c,'request_id':rid,'timestamp':time.time()},separators=(',',':'))
        for _ in range(5): self.cartcmd.publish(m); time.sleep(0.05)
    def wheel(self,v,w):
        def one(y):
            vi=v-w*y; return vi,(vi-WHEEL_L*w)/WHEEL_R,(vi+WHEEL_L*w)/WHEEL_R
        return one(+DOCK_Y),one(-DOCK_Y)
    def markers(self,phase,v=0.0,w=0.0,extra=''):
        if self.cart: cx,cy=self.cart[0],self.cart[1]
        else: cx,cy=START_CART
        now=self.get_clock().now().to_msg(); arr=MarkerArray()
        def mk(i,text,yoff,z,scale,color):
            m=Marker(); m.header.frame_id='map'; m.header.stamp=now; m.ns='v216_status'; m.id=i; m.type=Marker.TEXT_VIEW_FACING; m.action=Marker.ADD
            m.pose.position.x=cx; m.pose.position.y=cy+yoff; m.pose.position.z=z; m.pose.orientation.w=1.0; m.scale.z=scale
            m.color.r,m.color.g,m.color.b,m.color.a=*color,1.0; m.text=text; return m
        a1,a2=self.wheel(v,w)
        arr.markers=[mk(0,f'V2.16.5 {phase}\\n{extra}',0,1.75,0.30,(1.0,0.05,0.55)),
                     mk(1,f'AMR1 v={a1[0]:+.3f}\\nL={a1[1]:+.2f} R={a1[2]:+.2f} rad/s',+0.85,1.20,0.24,(1.0,0.05,0.55)),
                     mk(2,f'AMR2 v={a2[0]:+.3f}\\nL={a2[1]:+.2f} R={a2[2]:+.2f} rad/s',-0.85,1.20,0.24,(1.0,0.05,0.55))]
        self.mark_pub.publish(arr)
    def wait_poses(self,timeout=12):
        t=time.monotonic()
        while time.monotonic()-t<timeout:
            if not safe_spin(self): return False
            if self.amr_pose[0] and self.amr_pose[1]: return True
        return False
    def scan_phase(self):
        print('[V2.16 STEP1] ARUCO SCAN ONLY - NO STEERING',flush=True); t=time.monotonic()
        while time.monotonic()-t<9.0:
            if not safe_spin(self): return False
            self.markers('ARUCO SCAN',extra=f'AMR1={self.scan_ids[0]}  AMR2={self.scan_ids[1]}')
            if self.scan_good[0]>=2 and self.scan_good[1]>=2:
                print(f'[V2.16 ARUCO PASS] AMR1={self.scan_ids[0]} AMR2={self.scan_ids[1]}',flush=True); return True
        print(f'[V2.16 ARUCO FALLBACK] deterministic scan-ready pose; AMR1={self.scan_ids[0]} AMR2={self.scan_ids[1]}',flush=True); return True
    def insert_phase(self):
        if not self.wait_poses(): return False
        start=[self.amr_pose[0],self.amr_pose[1]]; done=[False,False]; t0=time.monotonic(); last=0
        print(f'[V2.16 STEP2] STRAIGHT ONLY {INSERT_DISTANCE:.2f}m | V={INSERT_SPEED:.2f} W=0.000 Y=0.000',flush=True)
        while time.monotonic()-t0<36.0:
            if not safe_spin(self,0.03): return False
            moved=[]
            for i in range(2):
                p=self.amr_pose[i]
                if p:
                    heading=float(start[i][2])
                    dx=p[0]-start[i][0]; dy=p[1]-start[i][1]
                    moved.append(dx*math.cos(heading)+dy*math.sin(heading))
                else:
                    moved.append(0.0)
                if moved[i]>=INSERT_DISTANCE-0.025: done[i]=True
            self.two(0 if done[0] else INSERT_SPEED,0 if done[1] else INSERT_SPEED)
            if time.monotonic()-last>0.6:
                print(f'[INSERT] AMR1={moved[0]:.3f}/{INSERT_DISTANCE:.2f} AMR2={moved[1]:.3f}/{INSERT_DISTANCE:.2f} W=0',flush=True); last=time.monotonic()
            if all(done): self.stop_two(); print('[V2.16 STRAIGHT INSERT PASS]',flush=True); return True
        self.stop_two(); print('[V2.16 INSERT TIMEOUT] attach will use one exact ALIGN fallback if needed',flush=True); return False
    def attach_phase(self):
        print('[V2.16 STEP3] LIFT + DUAL FIXEDJOINT',flush=True); self.cart_json('ATTACH','v216_attach_primary'); t=time.monotonic()
        while time.monotonic()-t<5:
            safe_spin(self)
            if self.cart and self.cart[3]: print('[V2.16 ATTACH PASS] no alignment fallback',flush=True); return True
        print('[V2.16 ATTACH FALLBACK] one exact ALIGN -> ATTACH; no iterative side steering',flush=True)
        self.cart_json('ALIGN','v216_align_once'); time.sleep(0.6); self.cart_json('ATTACH','v216_attach_fallback'); t=time.monotonic()
        while time.monotonic()-t<6:
            safe_spin(self)
            if self.cart and self.cart[3]: print('[V2.16 ATTACH PASS FALLBACK]',flush=True); return True
        return False
    def nearest(self,x,y,idx):
        lo=max(0,idx-20); hi=min(len(SAFE_PATH),idx+160); best=lo; bd=1e9
        for i in range(lo,hi):
            d=(SAFE_PATH[i][0]-x)**2+(SAFE_PATH[i][1]-y)**2
            if d<bd: bd=d; best=i
        return best,math.sqrt(bd)
    def _wait_cart_attached(self, timeout=3.5):
        t=time.monotonic()
        while time.monotonic()-t < timeout:
            if not safe_spin(self,0.04):
                print('[V2.16.5 FAIL REASON] ROS context shutdown while waiting for cart attach',flush=True)
                return False
            if self.cart and self.cart[3]:
                return True
        return False

    def _recover_cart_attach(self):
        print('[V2.16.5 ATTACH RECOVERY] STOP -> ATTACH retry',flush=True)
        z=Twist()
        for _ in range(15):
            self.coop.publish(z)
            time.sleep(0.02)

        self.cart_json('ATTACH','v2165_transport_reattach')
        if self._wait_cart_attached(DETACH_RECOVERY_WAIT_SEC):
            print('[V2.16.5 ATTACH RECOVERY PASS] ATTACH restored',flush=True)
            return True

        print('[V2.16.5 ATTACH RECOVERY] one exact ALIGN -> ATTACH',flush=True)
        self.cart_json('ALIGN','v2165_transport_align_once')
        time.sleep(0.7)
        self.cart_json('ATTACH','v2165_transport_reattach_after_align')
        if self._wait_cart_attached(5.0):
            print('[V2.16.5 ATTACH RECOVERY PASS] ALIGN + ATTACH restored',flush=True)
            return True

        print('[V2.16.5 FAIL REASON] cart attachment could not be restored',flush=True)
        return False

    def transport(self):
        print('[V2.16.5 STEP4] FAST TRANSPORT x3 + ATTACH GUARD',flush=True)
        idx=0
        rec=0
        last_prog=None
        lastprint=0
        deadline=time.monotonic()+360
        detach_since=None

        while time.monotonic()<deadline:
            if not safe_spin(self,0.03):
                print('[V2.16.5 FAIL REASON] safe_spin returned false / ROS shutdown',flush=True)
                return False

            if not self.cart or time.monotonic()-self.cart_wall>1.2:
                continue

            x,y,yaw,attached=self.cart

            # One stale/transient false frame must not abort a successful demo.
            if not attached:
                if detach_since is None:
                    detach_since=time.monotonic()
                    print(
                        f'[V2.16.5 ATTACH GUARD] DETACHED status at '
                        f'actual=({x:+.2f},{y:+.2f}); hold before judging',
                        flush=True
                    )
                z=Twist()
                self.coop.publish(z)

                if time.monotonic()-detach_since < DETACH_GRACE_SEC:
                    continue

                if not self._recover_cart_attach():
                    return False

                detach_since=None
                last_prog=(x,y,time.monotonic())
                continue

            detach_since=None

            gd=math.hypot(GOAL[0]-x,GOAL[1]-y)
            if gd<0.28:
                z=Twist()
                for _ in range(20):
                    self.coop.publish(z)
                print(f'[V2.16.5 TRANSPORT SUCCESS] actual=({x:.3f},{y:.3f})',flush=True)
                return True

            idx,cross=self.nearest(x,y,idx)
            look=min(len(SAFE_PATH)-1,idx+11)
            tx,ty,_=SAFE_PATH[look]
            err=norm(math.atan2(ty-y,tx-x)-yaw)

            v=TRANSPORT_V_TIGHT if abs(err)>0.75 else (
                TRANSPORT_V_MID if abs(err)>0.40 or cross>0.35
                else TRANSPORT_V_FAST
            )
            w=clamp(TRANSPORT_W_GAIN*err,-TRANSPORT_W_MAX,TRANSPORT_W_MAX)

            m=Twist()
            m.linear.x=v
            m.angular.z=w
            self.coop.publish(m)
            self.markers(
                'FAST TRANSPORT x3',v,w,
                f'actual=({x:+.2f},{y:+.2f}) cross={cross:.2f}m'
            )

            now=time.monotonic()
            if last_prog is None:
                last_prog=(x,y,now)
            elif math.hypot(x-last_prog[0],y-last_prog[1])>0.08:
                last_prog=(x,y,now)
            elif now-last_prog[2]>4.5:
                if rec>=4:
                    print(
                        f'[V2.16.5 FAIL REASON] physical stall persisted after {rec} recoveries '
                        f'at actual=({x:+.2f},{y:+.2f}) cross={cross:.2f}',
                        flush=True
                    )
                    return False

                rec+=1
                print(f'[V2.16.5 STALL RECOVERY {rec}/4] short reverse -> resume',flush=True)
                z=Twist()
                for _ in range(10):
                    self.coop.publish(z)
                rev=Twist()
                rev.linear.x=-0.08
                tr=time.monotonic()
                while time.monotonic()-tr<0.8:
                    self.coop.publish(rev)
                    time.sleep(0.05)
                last_prog=(x,y,time.monotonic())

            if now-lastprint>0.55:
                a1,a2=self.wheel(v,w)
                print(
                    f'[TRANSPORT] actual=({x:+.2f},{y:+.2f}) '
                    f'yaw={math.degrees(yaw):+.1f} cross={cross:.2f} '
                    f'V={v:+.2f} W={w:+.2f} | '
                    f'AMR1 L/R={a1[1]:+.2f}/{a1[2]:+.2f} '
                    f'AMR2 L/R={a2[1]:+.2f}/{a2[2]:+.2f}',
                    flush=True
                )
                lastprint=now

        print('[V2.16.5 FAIL REASON] transport deadline exceeded',flush=True)
        return False

def main():
    rclpy.init(); n=Demo()
    try:
        if not n.wait_poses(): return 10
        if not n.scan_phase(): return 11
        n.insert_phase()
        if not n.attach_phase(): return 12
        if not n.transport(): return 13
        return 0
    except KeyboardInterrupt: return 130
    finally:
        try: n.stop_two()
        except: pass
        try:
            z=Twist(); [n.coop.publish(z) for _ in range(5)]
        except: pass
        try: n.destroy_node()
        except: pass
        try:
            if rclpy.ok(): rclpy.shutdown()
        except: pass
if __name__=='__main__': sys.exit(main())
