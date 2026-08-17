#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_ros import Buffer, TransformListener, TransformException


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))


class TrolleyHeadingGate(Node):
    """V4.6: cumulative-corner controller with stable straight driving.

    Key changes from V4.4
    - Straight driving no longer performs repeated dominant-heading recovery stops.
    - Corners are detected from cumulative heading change over a forward window.
    - The first meaningful turn onset becomes the pre-turn reference point.
    - A consumed-corner guard prevents immediate re-detection after rotation.
    - Rotation has a guaranteed minimum angular speed and a stall boost.
    """

    DRIVE = 'DRIVE'
    ROTATE = 'ROTATE'

    def __init__(self):
        super().__init__('trolley_heading_gate')

        for name, default in [
            ('input_topic', '/trolley/cmd_vel_raw'), ('output_topic', '/trolley/cmd_vel'),
            ('plan_topic', '/plan'), ('global_frame', 'map'), ('base_frame', 'trolley_base')]:
            self.declare_parameter(name, default)

        # Dominant-segment / corner detection
        self.declare_parameter('dominant_segment_span_m', 0.80)
        self.declare_parameter('corner_detect_angle_deg', 35.0)
        self.declare_parameter('corner_onset_angle_deg', 12.0)
        self.declare_parameter('corner_scan_ahead_m', 3.0)
        self.declare_parameter('corner_target_lookahead_m', 0.90)
        self.declare_parameter('preturn_trigger_distance_m', 0.80)
        self.declare_parameter('corner_exit_angle_deg', 2.0)
        self.declare_parameter('consumed_corner_radius_m', 1.00)

        # Straight heading stabilization (do not react to tiny tangent noise)
        self.declare_parameter('drive_enter_angle_deg', 10.0)
        self.declare_parameter('drive_exit_angle_deg', 3.0)
        self.declare_parameter('drive_heading_kp', 1.0)
        self.declare_parameter('max_drive_angular_speed', 0.06)

        # Rotation robustness
        self.declare_parameter('rotate_kp', 1.6)
        self.declare_parameter('min_rotate_speed', 0.12)
        self.declare_parameter('max_rotate_speed', 0.35)
        self.declare_parameter('rotation_stall_time_sec', 1.0)
        self.declare_parameter('rotation_stall_yaw_deg', 0.5)
        self.declare_parameter('rotation_stall_boost_speed', 0.18)

        # Path lock
        self.declare_parameter('replan_min_interval_sec', 2.0)
        self.declare_parameter('replan_lateral_threshold_m', 0.30)
        self.declare_parameter('replan_heading_threshold_deg', 15.0)
        self.declare_parameter('new_goal_endpoint_threshold_m', 0.40)

        gp = lambda n: self.get_parameter(n).value
        self.input_topic = str(gp('input_topic')); self.output_topic = str(gp('output_topic'))
        self.plan_topic = str(gp('plan_topic')); self.global_frame = str(gp('global_frame')); self.base_frame = str(gp('base_frame'))
        self.seg_span = float(gp('dominant_segment_span_m'))
        self.corner_detect = math.radians(float(gp('corner_detect_angle_deg')))
        self.corner_onset = math.radians(float(gp('corner_onset_angle_deg')))
        self.corner_scan = float(gp('corner_scan_ahead_m'))
        self.corner_target_lookahead = float(gp('corner_target_lookahead_m'))
        self.preturn_dist = float(gp('preturn_trigger_distance_m'))
        self.corner_exit = math.radians(float(gp('corner_exit_angle_deg')))
        self.consumed_radius = float(gp('consumed_corner_radius_m'))
        self.drive_enter = math.radians(float(gp('drive_enter_angle_deg')))
        self.drive_exit = math.radians(float(gp('drive_exit_angle_deg')))
        self.drive_kp = float(gp('drive_heading_kp')); self.max_drive_w = float(gp('max_drive_angular_speed'))
        self.rotate_kp = float(gp('rotate_kp')); self.min_rotate = float(gp('min_rotate_speed')); self.max_rotate = float(gp('max_rotate_speed'))
        self.stall_time = float(gp('rotation_stall_time_sec')); self.stall_yaw = math.radians(float(gp('rotation_stall_yaw_deg')))
        self.stall_boost = float(gp('rotation_stall_boost_speed'))
        self.replan_dt = float(gp('replan_min_interval_sec')); self.replan_lat = float(gp('replan_lateral_threshold_m'))
        self.replan_heading = math.radians(float(gp('replan_heading_threshold_deg'))); self.new_goal_end = float(gp('new_goal_endpoint_threshold_m'))

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(Twist, self.output_topic, qos)
        self.create_subscription(Twist, self.input_topic, self._on_cmd, qos)
        self.create_subscription(Path, self.plan_topic, self._on_plan, qos)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.locked_plan = None
        self.last_plan_accept = 0.0
        self.state = self.DRIVE
        self.heading_hold = False
        self.rotate_target = None
        self.active_corner_xy = None
        self.consumed_corner_xy = None
        self.last_mode = None
        self.last_warn_ns = 0
        self.rot_ref_time = None
        self.rot_ref_yaw = None

        self.get_logger().info(
            'V4.6 CUMULATIVE CORNER active | seg=%.2fm corner>=%.1fdeg preturn=%.2fm | rotate min=%.2f max=%.2f'
            % (self.seg_span, math.degrees(self.corner_detect), self.preturn_dist, self.min_rotate, self.max_rotate))

    @staticmethod
    def _xy(pp):
        p = pp.pose.position
        return p.x, p.y

    def _pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.05))
        except TransformException as e:
            now = self.get_clock().now().nanoseconds
            if now - self.last_warn_ns > 2_000_000_000:
                self.get_logger().warning(f'TF unavailable: {e}')
                self.last_warn_ns = now
            return None
        t = tf.transform.translation; q = tf.transform.rotation
        return t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _nearest(self, path, rx, ry):
        return min(range(len(path.poses)), key=lambda i: (path.poses[i].pose.position.x-rx)**2 + (path.poses[i].pose.position.y-ry)**2)

    def _walk_index(self, path, start, distance, direction=1):
        pts = path.poses
        i = max(0, min(start, len(pts)-1)); acc = 0.0
        while True:
            j = i + direction
            if j < 0 or j >= len(pts):
                return i
            x0,y0 = self._xy(pts[i]); x1,y1 = self._xy(pts[j])
            acc += math.hypot(x1-x0, y1-y0)
            i = j
            if acc >= distance:
                return i

    def _segment_heading_centered(self, path, center_i, span):
        if len(path.poses) < 2:
            return None
        half = max(0.15, span * 0.5)
        i0 = self._walk_index(path, center_i, half, -1)
        i1 = self._walk_index(path, center_i, half, +1)
        x0,y0 = self._xy(path.poses[i0]); x1,y1 = self._xy(path.poses[i1])
        if math.hypot(x1-x0, y1-y0) < 0.10:
            return None
        return math.atan2(y1-y0, x1-x0)

    def _forward_heading(self, path, start_i, span=None):
        span = self.seg_span if span is None else span
        i1 = self._walk_index(path, start_i, span, +1)
        x0,y0 = self._xy(path.poses[start_i]); x1,y1 = self._xy(path.poses[i1])
        if math.hypot(x1-x0, y1-y0) < 0.10:
            return None
        return math.atan2(y1-y0, x1-x0)

    def _distance_along(self, path, i0, i1):
        if i1 <= i0: return 0.0
        d = 0.0
        x0,y0 = self._xy(path.poses[i0])
        for j in range(i0+1, min(i1+1, len(path.poses))):
            x1,y1 = self._xy(path.poses[j]); d += math.hypot(x1-x0, y1-y0); x0,y0=x1,y1
        return d

    def _corner_is_consumed(self, x, y):
        if self.consumed_corner_xy is None:
            return False
        return math.hypot(x-self.consumed_corner_xy[0], y-self.consumed_corner_xy[1]) < self.consumed_radius

    def _find_corner(self, path, nearest_i):
        """Detect a real corner from cumulative heading change, not one noisy tangent.

        1) Lock a base heading from the current forward dominant segment.
        2) Walk forward and look for the first place the local dominant heading
           starts to depart from the base by corner_onset.
        3) Confirm that farther ahead the total departure reaches corner_detect.
        4) Use a farther post-corner segment as the rotate target.
        """
        base = self._forward_heading(path, nearest_i, self.seg_span)
        if base is None:
            return None

        pts = path.poses
        traveled = 0.0
        prev = self._xy(pts[nearest_i])
        onset_i = None

        for j in range(nearest_i + 1, len(pts) - 1):
            cur = self._xy(pts[j])
            traveled += math.hypot(cur[0] - prev[0], cur[1] - prev[1])
            prev = cur
            if traveled > self.corner_scan:
                break
            if traveled < 0.25:
                continue

            local_h = self._forward_heading(path, j, max(0.45, self.seg_span * 0.65))
            if local_h is None:
                continue
            departure = abs(normalize_angle(local_h - base))

            if onset_i is None and departure >= self.corner_onset:
                onset_i = j

            if onset_i is not None and departure >= self.corner_detect:
                cx, cy = self._xy(pts[onset_i])
                if self._corner_is_consumed(cx, cy):
                    return None

                target_i = self._walk_index(path, j, self.corner_target_lookahead, +1)
                next_h = self._forward_heading(path, j, self.corner_target_lookahead)
                if next_h is None:
                    next_h = local_h
                total_turn = abs(normalize_angle(next_h - base))
                if total_turn < self.corner_detect:
                    total_turn = departure
                return onset_i, next_h, total_turn
        return None

    def _sig(self, path, rx, ry):
        i = self._nearest(path, rx, ry)
        px,py = self._xy(path.poses[i]); h = self._forward_heading(path, i)
        return math.hypot(px-rx, py-ry), h

    def _on_plan(self, msg):
        if len(msg.poses) < 2: return
        if self.locked_plan is None:
            self.locked_plan = msg; self.last_plan_accept = time.monotonic(); self.get_logger().info('PATH_LOCK: first plan accepted'); return
        old_end=self._xy(self.locked_plan.poses[-1]); new_end=self._xy(msg.poses[-1])
        if math.hypot(new_end[0]-old_end[0], new_end[1]-old_end[1]) >= self.new_goal_end:
            self.locked_plan=msg; self.last_plan_accept=time.monotonic(); self.state=self.DRIVE; self.rotate_target=None
            self.active_corner_xy=None; self.consumed_corner_xy=None; self.get_logger().info('PATH_LOCK: new goal accepted'); return
        # Never rewrite the path while a corner rotation is active.
        if self.state == self.ROTATE: return
        if time.monotonic()-self.last_plan_accept < self.replan_dt: return
        pose=self._pose()
        if pose is None: return
        rx,ry,_=pose; a=self._sig(self.locked_plan,rx,ry); b=self._sig(msg,rx,ry)
        if a is None or b is None: return
        lat=abs(a[0]-b[0]); dh=0.0 if a[1] is None or b[1] is None else abs(normalize_angle(b[1]-a[1]))
        if lat >= self.replan_lat or dh >= self.replan_heading:
            self.locked_plan=msg; self.last_plan_accept=time.monotonic()
            self.get_logger().info('PATH_LOCK: meaningful replan accepted (lat=%.2f, heading=%.1fdeg)'%(lat,math.degrees(dh)))

    def _mode(self, s):
        if s != self.last_mode:
            self.get_logger().info(s); self.last_mode=s

    def _rotate_cmd(self, err, ryaw):
        now=time.monotonic()
        if self.rot_ref_time is None:
            self.rot_ref_time=now; self.rot_ref_yaw=ryaw
        boost=False
        if now-self.rot_ref_time >= self.stall_time:
            moved=abs(normalize_angle(ryaw-self.rot_ref_yaw))
            boost = moved < self.stall_yaw
            self.rot_ref_time=now; self.rot_ref_yaw=ryaw
        w=clamp(self.rotate_kp*err,-self.max_rotate,self.max_rotate)
        floor=self.stall_boost if boost else self.min_rotate
        if abs(err)>self.corner_exit and abs(w)<floor:
            w=math.copysign(floor,err)
        out=Twist(); out.angular.z=w
        return out

    def _on_cmd(self, raw):
        if self.locked_plan is None or len(self.locked_plan.poses)<2:
            self.pub.publish(raw); return
        pose=self._pose()
        if pose is None:
            self.pub.publish(raw); return
        rx,ry,ryaw=pose; path=self.locked_plan; ni=self._nearest(path,rx,ry)

        # ROTATE is fully latched to one dominant next-segment heading.
        if self.state == self.ROTATE and self.rotate_target is not None:
            err=normalize_angle(self.rotate_target-ryaw)
            if abs(err) <= self.corner_exit:
                self.state=self.DRIVE; self.heading_hold=False; self.consumed_corner_xy=self.active_corner_xy
                self.active_corner_xy=None; self.rotate_target=None; self.rot_ref_time=None; self.rot_ref_yaw=None
                self._mode('CORNER: rotation complete -> DRIVE (corner consumed)')
            else:
                self._mode('CORNER: STOP + ROTATE dominant heading (linear.x=0)')
                self.pub.publish(self._rotate_cmd(err,ryaw)); return

        corner=self._find_corner(path,ni)
        if corner is not None:
            ci,next_yaw,turn=corner; dist=self._distance_along(path,ni,ci)
            if dist <= self.preturn_dist:
                self.state=self.ROTATE; self.rotate_target=next_yaw; self.active_corner_xy=self._xy(path.poses[ci]); self.rot_ref_time=None; self.rot_ref_yaw=None
                err=normalize_angle(next_yaw-ryaw)
                self._mode('CORNER: dominant turn %.1fdeg at %.2fm -> PRETURN STOP + ROTATE to %.1fdeg' % (math.degrees(turn),dist,math.degrees(next_yaw)))
                self.pub.publish(self._rotate_cmd(err,ryaw)); return

        # V4.6: do NOT stop/rotate for small straight-line heading changes.
        # That recovery loop was the source of DRIVE->ROTATE oscillation whenever
        # LiDAR/costmap replanning slightly changed the path. Only a confirmed
        # cumulative corner is allowed to force STOP+ROTATE.
        out=Twist()
        out.linear.x=max(0.0, raw.linear.x)
        out.linear.y=0.0
        out.angular.z=raw.angular.z
        self.heading_hold=False
        self._mode('STRAIGHT: DRIVE (no heading-recovery stop)')
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args); node=TrolleyHeadingGate()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
