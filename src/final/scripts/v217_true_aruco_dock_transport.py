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
CART_LENGTH=2.20; MARKER_STANDOFF=2.00
INSERT_DISTANCE=0.5*CART_LENGTH+MARKER_STANDOFF
CALIBRATED_DOCK_TRAVEL=2.90
MICRO_CREEP_STEP=0.03
MICRO_CREEP_RETRIES=3
INSERT_SPEED=0.13; DOCK_Y=0.425
WHEEL_R=0.075; WHEEL_L=0.5825

# V2.16.5 FINAL DEMO transport tuning.
# ArUco scan and straight insertion remain intentionally slow/stable.
TRANSPORT_V_FAST=1.98
TRANSPORT_V_MID=0.72
TRANSPORT_V_TIGHT=0.30
TRANSPORT_W_GAIN=8.10
TRANSPORT_W_MAX=1.05
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
        self.scan_pair=[None,None]; self.scan_wall=[-1.0,-1.0]; self.scan_width=[640.0,640.0]
        self.visual_baseline=[None,None]; self.visual_handoff=[None,None]
        self.cart=None; self.cart_wall=-1.0
        self.cmd1=self.create_publisher(Twist,'/amr1/tray_cmd_vel',20)
        self.cmd2=self.create_publisher(Twist,'/amr2/tray_cmd_vel',20)
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
        self.scan_ids[i]=[int(v) for v in d.get('visible_ids',[])]
        pair=d.get('pair') if str(d.get('state','')).upper()=='PAIR' else None
        if isinstance(pair,dict):
            self.scan_pair[i]=dict(pair); self.scan_wall[i]=time.monotonic(); self.scan_width[i]=float(d.get('image_width',640) or 640)
            self.scan_good[i]+=1
        else:
            self.scan_pair[i]=None; self.scan_good[i]=0
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
    def two(self,v1,v2,w1=0.0,w2=0.0):
        a=Twist(); a.linear.x=float(v1); a.angular.z=float(w1)
        b=Twist(); b.linear.x=float(v2); b.angular.z=float(w2)
        self.cmd1.publish(a); self.cmd2.publish(b)
    def stop_two(self):
        for _ in range(5): self.two(0,0); time.sleep(0.01)
    def cart_json(self,c,rid):
        m=String(); m.data=json.dumps({'command':c,'request_id':rid,'timestamp':time.time()},separators=(',',':'))
        for _ in range(3): self.cartcmd.publish(m); time.sleep(0.02)
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
        arr.markers=[mk(0,f'V2.17.3 {phase}\\n{extra}',0,1.75,0.30,(1.0,0.05,0.55)),
                     mk(1,f'AMR1 v={a1[0]:+.3f}\\nL={a1[1]:+.2f} R={a1[2]:+.2f} rad/s',+0.85,1.20,0.24,(1.0,0.05,0.55)),
                     mk(2,f'AMR2 v={a2[0]:+.3f}\\nL={a2[1]:+.2f} R={a2[2]:+.2f} rad/s',-0.85,1.20,0.24,(1.0,0.05,0.55))]
        self.mark_pub.publish(arr)
    def wait_poses(self,timeout=12):
        t=time.monotonic()
        while time.monotonic()-t<timeout:
            if not safe_spin(self): return False
            if self.amr_pose[0] and self.amr_pose[1]: return True
        return False
    def pair_fresh(self,i,max_age=0.45):
        return self.scan_pair[i] is not None and (time.monotonic()-self.scan_wall[i])<=max_age
    def pair_values(self,i):
        p=self.scan_pair[i] or {}; w=max(1.0,self.scan_width[i])
        epx=float(p.get('center_error_px',9999.0)); en=epx/(0.5*w)
        yaw=float(p.get('yaw_error_ratio',0.0)); side=max(1.0,float(p.get('mean_side_px',1.0)))
        return epx,en,yaw,side
    def visual_w(self,i):
        epx,en,yaw,_=self.pair_values(i)
        # Target to camera-right => robot must turn right => negative angular.z.
        return clamp(-1.02*en-1.26*yaw,-0.42,0.42)
    def scan_phase(self):
        print('[V2.17.3 STEP1] ONE-SHOT TRUE ARUCO LOCK - BOTH AMRs REQUIRED',flush=True)
        print('[V2.17.3 INFO] ArUco is used only at the farther start pose.',flush=True)
        if not self.wait_poses(4.0):
            return False

        stable=[0,0]
        lock=[None,None]
        deadline=time.monotonic()+90.0
        last=0.0

        while time.monotonic()<deadline:
            if not safe_spin(self,0.025):
                return False

            cmds=[(0.0,0.0),(0.0,0.0)]
            status=[]

            for i in range(2):
                if lock[i] is not None:
                    status.append(f'AMR{i+1}:LOCKED')
                    continue

                if not self.pair_fresh(i,max_age=0.70):
                    stable[i]=0
                    cmds[i]=(0.0,0.0)
                    status.append(f'AMR{i+1}:WAIT FULL PAIR')
                    continue

                epx,en,yaw,side=self.pair_values(i)
                aligned=(abs(epx)<=18.0 and abs(yaw)<=0.12)

                if aligned:
                    stable[i]+=1
                else:
                    stable[i]=0

                if stable[i]>=2:
                    p=self.amr_pose[i]
                    lock[i]={
                        'pose':(float(p[0]),float(p[1]),float(p[2])),
                        'epx':float(epx),
                        'yaw_visual':float(yaw),
                        'side_px':float(side),
                    }
                    cmds[i]=(0.0,0.0)
                    print(
                        f'[V2.17.3 ARUCO LOCK] AMR{i+1} '
                        f'e={epx:+.0f}px yaw={yaw:+.3f} '
                        f'pose=({p[0]:+.3f},{p[1]:+.3f},{math.degrees(p[2]):+.2f}deg)',
                        flush=True
                    )
                else:
                    # Only rotate for final one-shot alignment. Do not creep forward.
                    cmds[i]=(0.0,self.visual_w(i))

                status.append(
                    f'AMR{i+1}:PAIR e={epx:+.0f}px yaw={yaw:+.3f} '
                    f'stable={stable[i]}/2'
                )

            self.two(cmds[0][0],cmds[1][0],cmds[0][1],cmds[1][1])

            if time.monotonic()-last>0.45:
                print('[ONE-SHOT ARUCO] '+' | '.join(status),flush=True)
                self.markers('ONE-SHOT ARUCO LOCK',extra=' | '.join(status))
                last=time.monotonic()

            if lock[0] is not None and lock[1] is not None:
                self.stop_two()
                self.visual_handoff=[dict(lock[0]),dict(lock[1])]
                print(
                    '[V2.17.3 ARUCO LOCK PASS] BOTH real pairs locked. '
                    'Camera is no longer required.',
                    flush=True
                )
                return True

        self.stop_two()
        print('[V2.17.3 ARUCO LOCK FAIL] NO fallback / NO attach',flush=True)
        return False

    def visual_approach_phase(self):
        if not self.visual_handoff[0] or not self.visual_handoff[1]:
            return False
        print(
            '[V2.17.3 STEP2] ARUCO HANDOFF COMPLETE -> fixed-distance straight insertion',
            flush=True
        )
        return True

    def insert_phase(self):
        print('[V2.17.12 STEP3] ACTUAL X3 DIRECT DRIVE -> EARLY PHYSICAL CAPTURE',flush=True)
        if not self.visual_handoff[0] or not self.visual_handoff[1]:
            return False

        starts=[self.visual_handoff[0]['pose'],self.visual_handoff[1]['pose']]
        targets=[CALIBRATED_DOCK_TRAVEL,CALIBRATED_DOCK_TRAVEL]
        done=[False,False]
        last=0.0

        # V2.17.5: timeout must never kill a robot that is still making progress.
        # Keep a separate physical-stall watchdog.
        last_progress_time=time.monotonic()
        last_progress_sum=0.0
        STALL_TIMEOUT_SEC=8.0

        print(
            f'[V2.17.3 INSERT PLAN] distance={INSERT_DISTANCE:.2f}m '
            f'(tray half={0.5*CART_LENGTH:.2f} + standoff={MARKER_STANDOFF:.2f})',
            flush=True
        )

        while rclpy.ok():
            if not safe_spin(self,0.025):
                return False

            moved=[0.0,0.0]
            lateral=[0.0,0.0]
            cmd=[(0.0,0.0),(0.0,0.0)]

            for i in range(2):
                p=self.amr_pose[i]
                st=starts[i]
                if p is None or st is None:
                    continue

                heading=float(st[2])
                dx=p[0]-st[0]
                dy=p[1]-st[1]
                prog=dx*math.cos(heading)+dy*math.sin(heading)
                lat=-dx*math.sin(heading)+dy*math.cos(heading)
                moved[i]=prog
                lateral[i]=lat
                remain=targets[i]-prog

                if remain<=0.025:
                    done[i]=True
                    cmd[i]=(0.0,0.0)
                    continue

                yaw_err=norm(heading-p[2])
                w=clamp(2.80*yaw_err,-0.210,0.210)

                # V2.17.11 FULL TURBO x3 pre-drive.
                if remain>1.60:
                    v=0.900
                elif remain>0.80:
                    v=0.750
                elif remain>0.35:
                    v=0.600
                elif remain>0.14:
                    v=0.450
                else:
                    v=0.300

                if abs(lat)>0.11:
                    self.stop_two()
                    print(
                        f'[V2.17.3 INSERT ABORT] AMR{i+1} lateral drift={lat:+.3f}m; NO attach',
                        flush=True
                    )
                    return False
                if abs(yaw_err)>math.radians(9.0):
                    self.stop_two()
                    print(
                        f'[V2.17.3 INSERT ABORT] AMR{i+1} yaw drift={math.degrees(yaw_err):+.1f}deg; NO attach',
                        flush=True
                    )
                    return False

                cmd[i]=(v,w)

            sync=moved[0]-moved[1]
            if not done[0] and not done[1]:
                if sync>0.025:
                    cmd[0]=(max(0.060,cmd[0][0]-0.090),cmd[0][1])
                elif sync<-0.025:
                    cmd[1]=(max(0.060,cmd[1][0]-0.090),cmd[1][1])

            # Refresh stall watchdog whenever the two AMRs make >=1cm aggregate progress.
            progress_sum=max(0.0,moved[0])+max(0.0,moved[1])
            now_watch=time.monotonic()
            if progress_sum-last_progress_sum >= 0.010:
                last_progress_sum=progress_sum
                last_progress_time=now_watch
            elif now_watch-last_progress_time > STALL_TIMEOUT_SEC:
                self.stop_two()
                print(
                    f'[V2.17.8 INSERT STALL] no >=1cm physical progress for '
                    f'{STALL_TIMEOUT_SEC:.1f}s; A1={moved[0]:.3f} A2={moved[1]:.3f}; NO attach',
                    flush=True
                )
                return False

            self.two(cmd[0][0],cmd[1][0],cmd[0][1],cmd[1][1])

            if time.monotonic()-last>0.42:
                print(
                    f'[FIXED INSERT] '
                    f'AMR1={moved[0]:.3f}/{targets[0]:.3f} lat={lateral[0]:+.3f} | '
                    f'AMR2={moved[1]:.3f}/{targets[1]:.3f} lat={lateral[1]:+.3f} | '
                    f'sync={sync:+.3f}m',
                    flush=True
                )
                self.markers(
                    'FIXED INSERT AFTER ARUCO',
                    extra=f'A1 {moved[0]:.2f}/{targets[0]:.2f}m | A2 {moved[1]:.2f}/{targets[1]:.2f}m'
                )
                last=time.monotonic()

            if all(done):
                self.stop_two()
                print(
                    '[V2.17.9 EARLY CAPTURE WINDOW PASS] REAL ArUco lock -> exact fixed-distance insertion complete',
                    flush=True
                )
                return True

    def _wait_attached(self, timeout=1.6):
        t=time.monotonic()
        while time.monotonic()-t<timeout:
            if not safe_spin(self,0.04):
                return False
            if self.cart and self.cart[3]:
                return True
        return False

    def _micro_creep_pair(self, distance_m):
        starts=[self.amr_pose[0],self.amr_pose[1]]
        if starts[0] is None or starts[1] is None:
            return False
        deadline=time.monotonic()+2.0
        done=[False,False]
        while time.monotonic()<deadline:
            if not safe_spin(self,0.025):
                return False
            moved=[0.0,0.0]
            cmd=[(0.0,0.0),(0.0,0.0)]
            for i in range(2):
                p=self.amr_pose[i]; st=starts[i]
                if p is None or st is None:
                    continue
                heading=float(self.visual_handoff[i]['pose'][2])
                dx=p[0]-st[0]; dy=p[1]-st[1]
                prog=dx*math.cos(heading)+dy*math.sin(heading)
                moved[i]=prog
                if prog>=distance_m-0.004:
                    done[i]=True
                    continue
                yaw_err=norm(heading-p[2])
                cmd[i]=(0.075,clamp(2.8*yaw_err,-0.15,0.15))
            sync=moved[0]-moved[1]
            if not done[0] and not done[1]:
                if sync>0.012:
                    cmd[0]=(0.036,cmd[0][1])
                elif sync<-0.012:
                    cmd[1]=(0.036,cmd[1][1])
            self.two(cmd[0][0],cmd[1][0],cmd[0][1],cmd[1][1])
            if all(done):
                self.stop_two()
                print(f'[V2.17.7 MICRO CREEP PASS] A1={moved[0]:.3f} A2={moved[1]:.3f}',flush=True)
                return True
        self.stop_two()
        print('[V2.17.7 MICRO CREEP FAIL] no fake attach',flush=True)
        return False

    def attach_phase(self):
        print('[V2.17.12 STEP4] FAST ATTACH -> IMMEDIATE COOP START',flush=True)
        for attempt in range(MICRO_CREEP_RETRIES+1):
            self.stop_two()
            self.cart_json('ATTACH',f'v2177_attach_{attempt}')
            if self._wait_attached(1.6):
                print(f'[V2.17.12 ATTACH PASS] physical capture accepted on attempt {attempt+1}',flush=True)
                return True
            if attempt>=MICRO_CREEP_RETRIES:
                break
            print(f'[V2.17.11 ATTACH RETRY] creep +{MICRO_CREEP_STEP:.2f}m',flush=True)
            if not self._micro_creep_pair(MICRO_CREEP_STEP):
                return False
        self.stop_two()
        print('[V2.17.7 ATTACH FAIL] physical capture rejected; NO ALIGN / NO fake attach',flush=True)
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
        print('[V2.17.13 TRANSPORT X3] /coop/cmd_vel FAST=1.98m/s MID=0.72 TIGHT=0.30',flush=True)
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
                'FULL TRANSPORT X3',v,w,
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
                rev.linear.x=-0.24
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
        if not n.visual_approach_phase(): return 14
        if not n.insert_phase(): return 15
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
