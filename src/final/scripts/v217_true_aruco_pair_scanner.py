#!/usr/bin/env python3
from __future__ import annotations
import json, math, time
from typing import Any
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

def image_to_bgr(msg: Image) -> np.ndarray:
    enc=str(msg.encoding).lower(); h,w=int(msg.height),int(msg.width)
    ch={"rgb8":3,"bgr8":3,"rgba8":4,"bgra8":4,"mono8":1}.get(enc)
    if ch is None: raise RuntimeError(f"unsupported image encoding: {msg.encoding}")
    raw=np.frombuffer(msg.data,dtype=np.uint8); rows=raw.reshape(h,int(msg.step))[:,:w*ch]
    if ch==1: return cv2.cvtColor(rows.reshape(h,w),cv2.COLOR_GRAY2BGR)
    im=rows.reshape(h,w,ch)
    if enc=="rgb8": return cv2.cvtColor(im,cv2.COLOR_RGB2BGR)
    if enc=="rgba8": return cv2.cvtColor(im,cv2.COLOR_RGBA2BGR)
    if enc=="bgra8": return cv2.cvtColor(im,cv2.COLOR_BGRA2BGR)
    return im.copy()

def marker_side(pts):
    pts=np.asarray(pts,dtype=np.float32).reshape(4,2)
    return float(sum(np.linalg.norm(pts[(i+1)%4]-pts[i]) for i in range(4))/4.0)

def virtual_marker(data, ids):
    visible=[data[i] for i in ids if i in data]
    visible_ids=[i for i in ids if i in data]
    if not visible: return None
    return {
        'center_x':float(sum(v['center_x'] for v in visible)/len(visible)),
        'center_y':float(sum(v['center_y'] for v in visible)/len(visible)),
        'side_px':float(sum(v['side_px'] for v in visible)/len(visible)),
        'source_ids':visible_ids,
        'source_count':len(visible_ids),
    }

class Scanner(Node):
    def __init__(self):
        super().__init__('v217_true_aruco_scanner')
        self.declare_parameter('amr_id','amr1')
        self.declare_parameter('image_topic','/amr1/camera/front/color/image_raw')
        self.declare_parameter('result_topic','/amr1/tray_aruco/result')
        self.declare_parameter('debug_image_topic','/amr1/tray_aruco/debug_image')
        self.declare_parameter('show_window',True)
        self.amr=str(self.get_parameter('amr_id').value)
        self.image_topic=str(self.get_parameter('image_topic').value)
        self.result_topic=str(self.get_parameter('result_topic').value)
        self.debug_topic=str(self.get_parameter('debug_image_topic').value)
        self.show=bool(self.get_parameter('show_window').value)
        if self.amr=='amr1': self.outer_ids=[40,41]; self.outer_side='left'
        else: self.outer_ids=[42,43]; self.outer_side='right'
        self.center_id=44; self.window=f'DOOSIM {self.amr.upper()} TRUE ARUCO DOCK'
        dic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        p=cv2.aruco.DetectorParameters(); p.adaptiveThreshWinSizeMin=3; p.adaptiveThreshWinSizeMax=53; p.adaptiveThreshWinSizeStep=4
        p.minMarkerPerimeterRate=0.008; p.maxMarkerPerimeterRate=4.0; p.minCornerDistanceRate=0.01; p.minDistanceToBorder=2
        p.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX; p.cornerRefinementWinSize=5; p.cornerRefinementMaxIterations=50
        if hasattr(p,'detectInvertedMarker'): p.detectInvertedMarker=True
        try: self.det=cv2.aruco.ArucoDetector(dic,p); self.dic=None; self.params=None
        except AttributeError: self.det=None; self.dic=dic; self.params=p
        self.pub=self.create_publisher(String,self.result_topic,20)
        self.debug=self.create_publisher(Image,self.debug_topic,qos_profile_sensor_data)
        self.create_subscription(Image,self.image_topic,self.cb,qos_profile_sensor_data)
        self.last=0.0
        self.get_logger().info(f'[{self.amr}] V2.17 PAIR-REQUIRED scanner outer={self.outer_ids} center=44')
    def detect(self,gray):
        def one(img):
            if self.det is not None: return self.det.detectMarkers(img)
            return cv2.aruco.detectMarkers(img,self.dic,parameters=self.params)
        obs={}
        def consume(img,scale):
            corners,ids,_=one(img)
            if ids is None: return
            for c,rid in zip(corners,ids.flatten()):
                mid=int(rid)
                if mid not in set(self.outer_ids+[44]): continue
                pts=np.asarray(c,dtype=np.float32).reshape(4,2)/float(scale); side=marker_side(pts)
                if mid not in obs or side>obs[mid][0]: obs[mid]=(side,pts)
        consume(gray,1.0)
        ids=set(obs)
        if not (44 in ids and bool(ids & set(self.outer_ids))):
            clahe=cv2.createCLAHE(clipLimit=2.0,tileGridSize=(8,8)).apply(gray); consume(clahe,1.0)
            enlarged=cv2.resize(clahe,None,fx=2.0,fy=2.0,interpolation=cv2.INTER_CUBIC); consume(enlarged,2.0)
        return {k:v[1] for k,v in obs.items()}
    def cb(self,msg):
        now=time.monotonic()
        if now-self.last<1/15: return
        self.last=now
        try:
            frame=image_to_bgr(msg); gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY); detected=self.detect(gray)
        except Exception as e:
            self.get_logger().warning(str(e)); return
        data={}
        for mid,pts in detected.items():
            cen=pts.mean(axis=0); data[mid]={'center_x':float(cen[0]),'center_y':float(cen[1]),'side_px':marker_side(pts),'corners':pts}
        outer=virtual_marker(data,self.outer_ids); center=virtual_marker(data,[44]); pair=None; h,w=frame.shape[:2]
        if outer is not None and center is not None:
            if self.outer_side=='left': left,right=outer,center
            else: left,right=center,outer
            lx,ly=float(left['center_x']),float(left['center_y']); rx,ry=float(right['center_x']),float(right['center_y'])
            ls=max(1.0,float(left['side_px'])); rs=max(1.0,float(right['side_px'])); mean=0.5*(ls+rs)
            pair={'outer_source_ids':list(outer['source_ids']),'center_id':44,
                  'left_center_x':lx,'left_center_y':ly,'right_center_x':rx,'right_center_y':ry,
                  'pair_center_x':0.5*(lx+rx),'pair_center_y':0.5*(ly+ry),
                  'center_error_px':0.5*(lx+rx)-0.5*w,'pair_spacing_px':float(math.hypot(rx-lx,ry-ly)),
                  'left_side_px':ls,'right_side_px':rs,'mean_side_px':mean,
                  'yaw_error_ratio':float(math.log(rs/ls)),
                  'size_error_ratio':float((rs-ls)/mean),
                  'line_angle_deg':float(math.degrees(math.atan2(ry-ly,rx-lx)))}
        dbg=frame.copy(); cx=int(w/2); cv2.line(dbg,(cx,0),(cx,h-1),(255,255,0),2)
        for mid,d in data.items():
            pts=np.round(d['corners']).astype(np.int32).reshape((-1,1,2)); cv2.polylines(dbg,[pts],True,(0,255,0),2)
            x,y=np.round(d['corners'].mean(axis=0)).astype(int); cv2.putText(dbg,f'ID {mid}',(x-25,max(18,y-10)),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,0),2)
        if pair:
            pcx,pcy=int(pair['pair_center_x']),int(pair['pair_center_y']); cv2.circle(dbg,(pcx,pcy),9,(255,0,255),2)
            text=f"PAIR LOCKED  err={pair['center_error_px']:+.1f}px yaw={pair['yaw_error_ratio']:+.3f}"
            col=(0,255,0)
        else:
            text=f"WAIT FULL PAIR outer={self.outer_ids} + center=44"; col=(0,80,255)
        sweep=int((time.monotonic()*140)%max(1,h)); cv2.line(dbg,(0,sweep),(w-1,sweep),(255,220,80),1)
        cv2.rectangle(dbg,(0,h-42),(w,h),(10,10,10),-1); cv2.putText(dbg,f'{self.amr.upper()} {text}',(8,h-14),cv2.FONT_HERSHEY_SIMPLEX,.52,col,2)
        if self.show:
            try: cv2.namedWindow(self.window,cv2.WINDOW_NORMAL); cv2.resizeWindow(self.window,760,520); cv2.imshow(self.window,dbg); cv2.waitKey(1)
            except Exception: self.show=False
        dm=Image(); dm.header=msg.header; dm.height=h; dm.width=w; dm.encoding='bgr8'; dm.is_bigendian=0; dm.step=w*3; dm.data=dbg.tobytes(); self.debug.publish(dm)
        out=String(); out.data=json.dumps({'state':'PAIR' if pair else 'WAIT_PAIR','amr':self.amr,'timestamp':time.time(),
            'image_width':w,'image_height':h,'visible_ids':sorted(data),'outer_ids':self.outer_ids,'center_id':44,'pair':pair},separators=(',',':'))
        self.pub.publish(out)
    def destroy_node(self):
        try: cv2.destroyWindow(self.window)
        except Exception: pass
        return super().destroy_node()

def main():
    rclpy.init(); n=Scanner()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()
