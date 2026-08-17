#!/usr/bin/env python3
"""Integrated collision-avoidance + cooperative tray transport mission.

Baseline behavior is inherited from the user's integrated hospital project.
Only the tray mission is added:
  1. Tray starts alone at the fixed screenshot pose (-22.69, 11.03).
  2. AMR1 and AMR2 each perform a small guarded safe-egress from their start bays.
  3. AMR1/AMR2 receive ordinary PUBLIC center goals, so the existing
     the hospital_total_08091221 latest baseline CenterlineNavigator stacks stay unchanged; the uploaded final path_conflict_manager remains the traffic arbiter.
  4. The existing path_conflict_manager may publish /traffic_pause or
     /amr2/traffic_pause.  CenterlineNavigator cancels/replans automatically.
  5. V2.11 does not wait for both robots at PRE_DOCK. The first arrival immediately
     scans one expected ArUco ID and performs a bed-style fixed-distance straight
     insertion while the peer keeps navigating.
  6. A translation no-progress watchdog reissues the same center goal when an ACTIVE
     straight segment is physically stalled while traffic is FREE/READY.
  7. Both lifts attach the cart with two FixedJoints.
  7. Cooperative Nav2 transports the attached tray to (7.90, 10.13).
  8. Stop and keep the tray attached.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time
import uuid
import subprocess
import os

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import LaserScan


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", default="config/isaac_config.json")
    return p.parse_args()


def fixed_target(cfg: dict) -> tuple[float, float, float]:
    t = cfg["cooperative_auto_transport"]["fixed_target"]
    return float(t["x"]), float(t["y"]), math.radians(float(t.get("yaw_deg", 0.0)))


class Manager(Node):
    def __init__(self, cfg: dict) -> None:
        super().__init__("integrated_collision_tray_transport_manager")
        self.cfg = cfg
        self.dock_cfg = cfg.get("tray_aruco_docking", {})
        self.auto_cfg = cfg.get("cooperative_auto_transport", {})

        latched = QoSProfile(depth=10)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Goals are sent to the original AMR1/AMR2 centerline topics. The hospital_total_08091221 latest baseline navigation and path_conflict_manager are not replaced.
        self.goal_topics = ["/center_goal", "/amr2/center_goal"]
        self.status_topics = ["/center_goal/status", "/amr2/center_goal/status"]
        self.goal_pubs = [
            self.create_publisher(PoseStamped, self.goal_topics[0], 10),
            self.create_publisher(PoseStamped, self.goal_topics[1], 10),
        ]
        self.nav_status = ["", ""]
        self.create_subscription(String, self.status_topics[0], lambda m: self._status_cb(0, m), latched)
        self.create_subscription(String, self.status_topics[1], lambda m: self._status_cb(1, m), latched)

        self.world_pose: list[tuple[float, float, float] | None] = [None, None]
        self.create_subscription(String, "/amr1/world_pose", lambda m: self._world_cb(0, m), 20)
        self.create_subscription(String, "/amr2/world_pose", lambda m: self._world_cb(1, m), 20)

        # V2.12 guarded start-bay egress for BOTH AMRs.  These consume the same
        # 360-degree LaserScan topics already used by the independent Nav2 stacks.
        self.front_clearance = [float("inf"), float("inf")]
        self.scan_rx = [0.0, 0.0]
        self.create_subscription(LaserScan, "/scan", lambda m: self._scan_cb(0, m), 20)
        self.create_subscription(LaserScan, "/amr2/scan", lambda m: self._scan_cb(1, m), 20)

        self.pose_locked = [False, False]
        self.create_subscription(Bool, "/initial_pose_locked", lambda m: self._lock_cb(0, m), latched)
        self.create_subscription(Bool, "/amr2/initial_pose_locked", lambda m: self._lock_cb(1, m), latched)

        self.traffic_status = ""
        self.create_subscription(String, "/traffic_conflict/status", self._traffic_cb, latched)

        # Dedicated final-ingress commands.  Isaac gives these fresh commands
        # priority over normal Nav2 but only for ~0.35 s.
        self.tray_cmd_topics = ["/amr1/tray_cmd_vel", "/amr2/tray_cmd_vel"]
        self.tray_cmd_pubs = [
            self.create_publisher(Twist, self.tray_cmd_topics[0], 20),
            self.create_publisher(Twist, self.tray_cmd_topics[1], 20),
        ]

        # V2.11: tell path_conflict_manager that this AMR has left Nav2 and is in
        # dedicated tray ingress. Unlike special_motion_active, this bypass does NOT
        # pause the peer; the other AMR is allowed to keep driving to PRE_DOCK.
        self.tray_docking_active_pubs = [
            self.create_publisher(Bool, "/amr1/tray_docking_active", latched),
            self.create_publisher(Bool, "/amr2/tray_docking_active", latched),
        ]
        self.tray_docking_active = [False, False]

        self.aruco = [None, None]
        self.aruco_rx = [0.0, 0.0]
        self.create_subscription(String, "/amr1/tray_aruco/result", lambda m: self._aruco_cb(0, m), 20)
        self.create_subscription(String, "/amr2/tray_aruco/result", lambda m: self._aruco_cb(1, m), 20)

        self.cart_status: dict = {}
        self.cart_rx = 0.0
        cart_cmd_topic = str(self.dock_cfg.get("cart_command_topic", "/coop/cart/command"))
        cart_status_topic = str(self.dock_cfg.get("cart_status_topic", "/coop/cart/status"))
        self.cart_command_pub = self.create_publisher(String, cart_cmd_topic, 10)
        self.create_subscription(String, cart_status_topic, self._cart_cb, 20)

        self.coop_goal_pub = self.create_publisher(PoseStamped, "/coop/center_goal", 10)
        self.coop_initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/coopnav/initialpose", 10)
        self.coop_pose_locked = False
        self.create_subscription(Bool, "/coopnav/initial_pose_locked", self._coop_lock_cb, latched)
        self.coop_cmd_pub = self.create_publisher(Twist, "/coop/cmd_vel", 20)
        self.coop_status = ""
        self.create_subscription(String, "/coop/center_goal/status", self._coop_status_cb, latched)

        self.state_pub = self.create_publisher(String, "/coop/transport/state", latched)
        self.runtime_status: dict = {}
        self.create_subscription(String, "/tray/runtime_status", self._runtime_status_cb, latched)
        self.child_processes: list[subprocess.Popen] = []
        self.child_logs = []
        self.aruco_started = False

    def _runtime_status_cb(self, msg: String) -> None:
        try: self.runtime_status = json.loads(msg.data)
        except Exception: pass

    def _status_cb(self, i: int, msg: String) -> None:
        self.nav_status[i] = str(msg.data)

    def _lock_cb(self, i: int, msg: Bool) -> None:
        self.pose_locked[i] = bool(msg.data)

    def _traffic_cb(self, msg: String) -> None:
        self.traffic_status = str(msg.data)

    def set_tray_docking_active(self, i: int, active: bool, force: bool = False) -> None:
        active = bool(active)
        if not force and self.tray_docking_active[i] == active:
            return
        self.tray_docking_active[i] = active
        msg = Bool(); msg.data = active
        self.tray_docking_active_pubs[i].publish(msg)
        print(f"[TRAY TRAFFIC BYPASS V2.11] AMR{i+1} active={active}")

    def _world_cb(self, i: int, msg: String) -> None:
        try:
            p = json.loads(msg.data)
            self.world_pose[i] = (float(p["x"]), float(p["y"]), float(p["yaw"]))
        except Exception:
            pass

    def _scan_cb(self, i: int, msg: LaserScan) -> None:
        cfg = self.auto_cfg.get(f"safe_egress_amr{i+1}", {})
        half = math.radians(float(cfg.get("front_lidar_half_angle_deg", 15.0)))
        best = float("inf")
        angle = float(msg.angle_min)
        inc = float(msg.angle_increment)
        rmin = float(msg.range_min)
        rmax = float(msg.range_max)
        for rng in msg.ranges:
            rr = float(rng)
            if abs(wrap(angle)) <= half and math.isfinite(rr) and rmin <= rr <= rmax:
                best = min(best, rr)
            angle += inc
        self.front_clearance[i] = best
        self.scan_rx[i] = time.monotonic()

    def _aruco_cb(self, i: int, msg: String) -> None:
        try:
            self.aruco[i] = json.loads(msg.data)
            self.aruco_rx[i] = time.monotonic()
        except Exception:
            pass

    def _cart_cb(self, msg: String) -> None:
        try:
            self.cart_status = json.loads(msg.data)
            self.cart_rx = time.monotonic()
        except Exception:
            pass

    def _coop_status_cb(self, msg: String) -> None:
        self.coop_status = str(msg.data)

    def _coop_lock_cb(self, msg: Bool) -> None:
        self.coop_pose_locked = bool(msg.data)

    def pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        return msg

    def set_state(self, state: str, detail: str = "") -> None:
        msg = String()
        msg.data = json.dumps(
            {"state": state, "detail": detail, "timestamp": time.time()},
            ensure_ascii=False, separators=(",", ":"),
        )
        self.state_pub.publish(msg)
        print(f"[TRAY STATE] {state}{': ' + detail if detail else ''}")

    def cart_pose(self) -> tuple[float, float, float] | None:
        p = self.cart_status.get("cart_pose") if isinstance(self.cart_status, dict) else None
        if not isinstance(p, dict):
            return None
        try:
            return float(p["x"]), float(p["y"]), float(p["yaw"])
        except Exception:
            return None

    def cart_geometry(self) -> tuple[float, float, float, float]:
        g = self.cfg["cooperative_warehouse_cart"]["geometry"]
        length = float(g["length_m"])
        width = float(g["width_m"])
        side = float(g["bay_side_wall_thickness_m"])
        center = float(g["bay_center_wall_thickness_m"])
        bay_w = (width - 2.0 * side - center) * 0.5
        dock_y = (center + bay_w) * 0.5
        dock_x = float(g.get("dock_x_m", 0.0))
        return length, width, dock_x, dock_y

    @staticmethod
    def local_to_world(cart: tuple[float, float, float], lx: float, ly: float) -> tuple[float, float]:
        x, y, yaw = cart
        c, s = math.cos(yaw), math.sin(yaw)
        return x + c * lx - s * ly, y + s * lx + c * ly

    @staticmethod
    def world_to_local(cart: tuple[float, float, float], x: float, y: float) -> tuple[float, float]:
        cx, cy, yaw = cart
        dx, dy = x - cx, y - cy
        c, s = math.cos(yaw), math.sin(yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def wait_ready(self, timeout: float = 75.0) -> bool:
        print("[TRAY READY] integrated Nav2 + collision avoidance + ArUco + cooperative bridge 확인")
        deadline = time.monotonic() + timeout
        stable_since = None
        last = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.10)
            conditions = {
                "world": all(p is not None for p in self.world_pose),
                "pose_lock": all(self.pose_locked),
                "cart": (time.monotonic() - self.cart_rx) < 2.0,
                "goal_sub": all(pub.get_subscription_count() > 0 for pub in self.goal_pubs),
                "status_msg": all(bool(v) for v in self.nav_status),
                "traffic_msg": bool(self.traffic_status),
                "tray_direct_sub": all(self.count_subscribers(t) > 0 for t in self.tray_cmd_topics),
                "cart_cmd_sub": self.cart_command_pub.get_subscription_count() > 0,
                "runtime": bool(self.runtime_status.get("amr1_bridge")) and bool(self.runtime_status.get("amr2_bridge")) and bool(self.runtime_status.get("cart_ready")),
            }
            ready = all(conditions.values())
            if ready:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= 1.0:
                    print("[TRAY READY] 모든 인터페이스 1초 안정 확인")
                    return True
            else:
                stable_since = None
            now = time.monotonic()
            if now - last > 2.0:
                print("[TRAY WAIT] " + " ".join(f"{k}={v}" for k, v in conditions.items()))
                last = now
        return False

    def safe_egress_amr(self, i: int) -> bool:
        """Small non-fatal relative escape before handing authority to Nav2.

        V2.12 applies the same mechanism to AMR1 and AMR2.  The actual world pose
        at maneuver start is the reference, so stale station coordinates cannot
        make this primitive run a long distance.
        """
        cfg = self.auto_cfg.get(f"safe_egress_amr{i+1}", {})
        if not bool(cfg.get("enabled", True)):
            return True
        deadline_pose = time.monotonic() + 5.0
        while rclpy.ok() and self.world_pose[i] is None and time.monotonic() < deadline_pose:
            rclpy.spin_once(self, timeout_sec=0.1)
        p0 = self.world_pose[i]
        if p0 is None:
            print(f"[SAFE EGRESS SKIP] AMR{i+1} world pose unavailable; Nav2 remains authoritative")
            return True

        x0, y0, yaw0 = p0
        # V2.12: safe-egress is a docking-station-only primitive.  If the user has
        # manually moved an AMR elsewhere in the Stage, do not push it 12/14 cm
        # just because the mission started.  Nav2 will start from the live pose.
        guard = float(cfg.get("station_guard_radius_m", 0.0))
        if guard > 0.0 and "station_x" in cfg and "station_y" in cfg:
            ds = math.hypot(x0 - float(cfg["station_x"]), y0 - float(cfg["station_y"]))
            if ds > guard:
                print(
                    f"[SAFE EGRESS SKIP V2.12] AMR{i+1} is {ds:.2f}m from configured station "
                    f"(guard={guard:.2f}m); preserve manually placed start pose"
                )
                return True

        c, ss = math.cos(yaw0), math.sin(yaw0)
        direction = -1.0 if bool(cfg.get("reverse", False)) else 1.0
        target = abs(float(cfg.get("distance_m", 0.14)))
        success = abs(float(cfg.get("min_success_progress_m", max(0.08, target - 0.04))))
        soft = abs(float(cfg.get("soft_continue_progress_m", max(0.05, success - 0.03))))
        total_timeout = float(cfg.get("timeout_s", 8.0))
        retries = max(1, int(cfg.get("retry_count", 2)))
        speeds = [abs(float(cfg.get("speed_mps", 0.12))), abs(float(cfg.get("retry_speed_mps", 0.13)))]
        stop_range = float(cfg.get("front_stop_range_m", 0.55))
        lateral_abort = float(cfg.get("lateral_abort_m", 0.20))
        yaw_abort = math.radians(float(cfg.get("yaw_abort_deg", 15.0)))
        max_w = abs(float(cfg.get("max_yaw_speed_rad_s", 0.18)))
        yaw_kp = abs(float(cfg.get("yaw_kp", 1.2)))
        straight_only = bool(cfg.get("straight_only", False))

        def metrics():
            p = self.world_pose[i]
            if p is None:
                return None
            dx, dy = p[0] - x0, p[1] - y0
            forward = c * dx + ss * dy
            lateral = -ss * dx + c * dy
            return direction * forward, lateral, wrap(p[2] - yaw0)

        self.set_state(f"SAFE_EGRESS_AMR{i+1}", f"relative escape success={success:.2f}m")
        per = max(2.0, total_timeout / retries)
        for a in range(retries):
            end = time.monotonic() + per
            v = direction * speeds[min(a, len(speeds) - 1)]
            print(
                f"[SAFE EGRESS AMR{i+1}] attempt={a+1}/{retries} "
                f"actual_start=({x0:.3f},{y0:.3f}) yaw={math.degrees(yaw0):.1f} v={v:+.2f}"
            )
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.04)
                m = metrics()
                if m is None:
                    continue
                prog, lat, ye = m
                if prog >= success:
                    self.stop_tray_direct(i, 8)
                    print(f"[SAFE EGRESS PASS] AMR{i+1} relative progress={prog:.3f}m")
                    return True
                if abs(lat) > lateral_abort or abs(ye) > yaw_abort:
                    print(f"[SAFE EGRESS WARN] AMR{i+1} drift lat={lat:.3f} yaw={math.degrees(ye):.1f}")
                    break
                # Front clearance is meaningful only for a forward egress.  Reverse
                # egress can be enabled explicitly in config without misusing front scan.
                if direction > 0.0 and math.isfinite(self.front_clearance[i]) and self.front_clearance[i] < stop_range:
                    print(f"[SAFE EGRESS WARN] AMR{i+1} front={self.front_clearance[i]:.2f}m; hand back to Nav2")
                    break
                cmd = Twist()
                cmd.linear.x = v
                # V2.12 AMR1 station release can be forced straight-only.  The old
                # yaw-hold controller could visibly spin a robot whose startup pose
                # was still settling or whose physical yaw did not match the stale
                # hard-coded map pose.
                cmd.angular.z = 0.0 if straight_only else clamp(-yaw_kp * ye, -max_w, max_w)
                self.tray_cmd_pubs[i].publish(cmd)
            self.stop_tray_direct(i, 8)
            time.sleep(0.25)

        m = metrics()
        prog = m[0] if m else 0.0
        tag = "SOFT PASS" if prog >= soft else "NONFATAL"
        print(f"[SAFE EGRESS {tag}] AMR{i+1} {prog:.3f}m; continue original Nav2")
        return True

    def safe_egress_amr1(self) -> bool:
        return self.safe_egress_amr(0)

    def safe_egress_amr2(self) -> bool:
        return self.safe_egress_amr(1)

    def _traffic_state_name(self) -> str:
        """Return the current path-conflict state in a compact uppercase form."""
        raw = (self.traffic_status or "").strip()
        if not raw:
            return "UNKNOWN"
        try:
            payload = json.loads(raw)
            state = str(payload.get("state", "")).strip().upper()
            return state or "UNKNOWN"
        except Exception:
            return raw.split(":", 1)[0].strip().upper() or "UNKNOWN"

    def _traffic_is_free_for_predock_fallback(self) -> bool:
        # Proximity fallback is intentionally forbidden while the baseline traffic
        # arbiter owns a YIELDING/CLEARANCE session.  In that state the priority
        # AMR must be allowed to finish its Nav2 final rotation and report
        # SUCCEEDED so path_conflict_manager can release the paused AMR.
        return self._traffic_state_name() in {"FREE", "READY"}

    def ensure_aruco_started(self, timeout: float = 60.0) -> bool:
        """Late-start the scanner exactly once, then wait for both result topics."""
        if self.aruco_started:
            return True
        self.set_state("ARUCO_GATE_START", "first PRE_DOCK arrival -> start dual scanner immediately")
        self.start_child_launch("tray_aruco", "hospital_tray_overlay", "tray_dual_aruco.launch.py")
        if not self.wait_publishers(["/amr1/tray_aruco/result", "/amr2/tray_aruco/result"], timeout):
            print("[ARUCO START FAIL] result publishers not ready")
            return False
        self.aruco_started = True
        print("[ARUCO START PASS V2.11] scanner/debug windows ready")
        return True

    def navigate_and_dock_pair(
        self,
        goals: list[tuple[float, float, float]],
        cart_home: tuple[float, float, float],
        dock_y: float,
        dock_x: float,
        timeout: float = 420.0,
    ) -> bool:
        """V2.11 asynchronous PRE_DOCK handoff.

        Both baseline Nav2 stacks are dispatched together.  The first robot that
        reaches PRE_DOCK starts ArUco and docks immediately; the other robot keeps
        navigating under the unchanged path_conflict_manager.  This removes the
        V2.7 all(done) barrier that made AMR2 sit silently at the tray whenever AMR1
        was delayed.
        """
        self.nav_status = ["", ""]
        start = list(self.world_pose)
        for i, (x, y, yaw) in enumerate(goals):
            self.goal_pubs[i].publish(self.pose(x, y, yaw))
            print(f"[PRE_DOCK GOAL V2.11] AMR{i+1}: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}deg")

        arrived = [False, False]
        docked = [False, False]
        arrival_order: list[int] = []
        prox_stable = [0, 0]
        handoff_dist = float(self.dock_cfg.get("pre_dock_handoff_distance_m", 0.22))
        handoff_yaw = math.radians(float(self.dock_cfg.get("pre_dock_handoff_yaw_deg", 3.0)))
        handoff_need = max(1, int(self.dock_cfg.get("pre_dock_handoff_stable_cycles", 5)))

        wd = self.auto_cfg.get("no_progress_watchdog", {})
        wd_enabled = bool(wd.get("enabled", True))
        wd_timeout = float(wd.get("timeout_s", 8.0))
        wd_min_move = float(wd.get("min_progress_m", 0.06))
        wd_max_replans = max(0, int(wd.get("max_replans_per_amr", 3)))
        wd_cooldown = float(wd.get("replan_cooldown_s", 4.0))
        progress_pose = list(self.world_pose)
        progress_time = [time.monotonic(), time.monotonic()]
        replan_count = [0, 0]
        last_replan = [-999.0, -999.0]

        deadline = time.monotonic() + timeout
        last = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.08)
            traffic_state = self._traffic_state_name()
            traffic_free = self._traffic_is_free_for_predock_fallback()
            now = time.monotonic()

            # Normal Nav2 completion is always the preferred handoff.
            for i, st in enumerate(self.nav_status):
                if st.startswith("FAILED") and not arrived[i]:
                    print(f"[PRE_DOCK FAIL] AMR{i+1}: {st}")
                    return False
                if st.startswith("SUCCEEDED") and not arrived[i]:
                    arrived[i] = True
                    prox_stable[i] = handoff_need
                    arrival_order.append(i)
                    self.stop_tray_direct(i, 2)
                    print(f"[PRE_DOCK ARRIVAL V2.11] AMR{i+1}: Nav2 {st}; no longer waiting for peer")

            # Tight fallback remains traffic-safe.  Unlike V2.7, one robot may use
            # it if the peer has already arrived/docked; otherwise both must be tight.
            for i, st in enumerate(self.nav_status):
                if arrived[i]:
                    continue
                p = self.world_pose[i]
                if p is None or not traffic_free or st.startswith("PAUSED"):
                    prox_stable[i] = 0
                    continue
                gx, gy, gyaw = goals[i]
                remain = math.hypot(gx - p[0], gy - p[1])
                yaw_err = abs(wrap(gyaw - p[2]))
                prox_stable[i] = min(handoff_need, prox_stable[i] + 1) if (remain <= handoff_dist and yaw_err <= handoff_yaw) else 0

            for i in range(2):
                if arrived[i] or prox_stable[i] < handoff_need or not traffic_free:
                    continue
                peer = 1 - i
                peer_ready = arrived[peer] or prox_stable[peer] >= handoff_need
                if not peer_ready:
                    continue
                arrived[i] = True
                arrival_order.append(i)
                self.stop_tray_direct(i, 2)
                print(f"[PRE_DOCK TIGHT HANDOFF V2.11] AMR{i+1}: traffic={traffic_state}")

            # Translation-only stall watchdog.  ROTATING_* is intentionally excluded
            # because zero positional progress is correct during an in-place turn.
            if wd_enabled and traffic_free:
                for i in range(2):
                    if arrived[i]:
                        continue
                    st = self.nav_status[i]
                    p = self.world_pose[i]
                    if p is None:
                        continue
                    if progress_pose[i] is None:
                        progress_pose[i] = p
                        progress_time[i] = now
                    dprog = math.hypot(p[0] - progress_pose[i][0], p[1] - progress_pose[i][1])
                    if dprog >= wd_min_move:
                        progress_pose[i] = p
                        progress_time[i] = now
                    active_translation = st.startswith("ACTIVE:SEGMENT_")
                    if not active_translation:
                        # Do not age the timer through PLANNING, ROTATING or traffic pause.
                        progress_pose[i] = p
                        progress_time[i] = now
                        continue
                    stalled = (now - progress_time[i]) >= wd_timeout
                    can_replan = replan_count[i] < wd_max_replans and (now - last_replan[i]) >= wd_cooldown
                    if stalled and can_replan:
                        replan_count[i] += 1
                        last_replan[i] = now
                        progress_pose[i] = p
                        progress_time[i] = now
                        gx, gy, gyaw = goals[i]
                        print(
                            f"[NO-PROGRESS WATCHDOG V2.11] AMR{i+1} stalled {wd_timeout:.1f}s "
                            f"in {st}; reissue PRE_DOCK goal {replan_count[i]}/{wd_max_replans}"
                        )
                        self.goal_pubs[i].publish(self.pose(gx, gy, gyaw))

            # Dock in real arrival order.  dock_one spins this node, so the peer's
            # Nav2/traffic state continues to update while the first robot docks.
            next_dock = next((i for i in arrival_order if arrived[i] and not docked[i]), None)
            if next_dock is not None:
                if not self.ensure_aruco_started():
                    return False
                self.set_state(
                    f"ARUCO_DOCKING_AMR{next_dock+1}",
                    "V2.11 stable ArUco ID -> tray-size fixed-distance straight insertion; peer Nav2 continues",
                )
                self.set_tray_docking_active(next_dock, True, force=True)
                if not self.dock_one(next_dock, cart_home, dock_y, dock_x):
                    self.set_tray_docking_active(next_dock, False, force=True)
                    print(f"[ARUCO DOCK FAIL V2.11] AMR{next_dock+1}")
                    return False
                docked[next_dock] = True
                print(f"[ARUCO DOCKED V2.11] AMR{next_dock+1}; peer state={self.nav_status[1-next_dock] or 'WAIT'}")
                if all(docked):
                    self.set_tray_docking_active(0, False, force=True)
                    self.set_tray_docking_active(1, False, force=True)
                    print("[PRE_DOCK+ARUCO COMPLETE V2.11] both AMRs fixed-distance docked")
                    return True
                continue

            if now - last > 1.0:
                parts = []
                for i, (gx, gy, gyaw) in enumerate(goals):
                    p = self.world_pose[i]
                    if p is None:
                        parts.append(f"AMR{i+1}:NO_POSE status={self.nav_status[i] or 'WAIT'}")
                        continue
                    remain = math.hypot(gx - p[0], gy - p[1])
                    yaw_e = abs(wrap(gyaw - p[2]))
                    moved = math.hypot(p[0] - start[i][0], p[1] - start[i][1]) if start[i] is not None else 0.0
                    phase = "DOCKED" if docked[i] else ("ARRIVED_WAIT_ARUCO" if arrived[i] else f"TIGHT={prox_stable[i]}/{handoff_need}")
                    parts.append(
                        f"AMR{i+1}:status={self.nav_status[i] or 'WAIT'} remain={remain:.2f}m "
                        f"final_yaw_error={math.degrees(yaw_e):.1f}deg phase={phase} moved={moved:.2f}m replans={replan_count[i]}"
                    )
                traffic = self.traffic_status[:160] if self.traffic_status else "READY"
                print("[PRE_DOCK RUN V2.11] " + " | ".join(parts) + f" | traffic={traffic}")
                last = now

        print(
            f"[PRE_DOCK/ARUCO FAIL V2.11] timeout arrived={arrived} docked={docked} "
            f"status={self.nav_status} traffic={self._traffic_state_name()}"
        )
        return False

    def navigate_pair(self, goals: list[tuple[float, float, float]], timeout: float = 360.0) -> bool:
        """Drive both AMRs to PRE_DOCK without stealing control from traffic/Nav2.

        V2.7 fixes the V2.6 deadlock observed at runtime:
          * a priority AMR near PRE_DOCK was previously marked handed-off at
            0.30 m / 18 deg and then continuously fed tray_cmd_vel=0;
          * the direct zero overrode Nav2's final rotation, so the winner never
            reached SUCCEEDED;
          * path_conflict_manager therefore never released the yielding AMR.

        The normal path is now strictly Nav2 SUCCEEDED for each AMR.  A physical
        proximity fallback remains, but only as a PAIR-SAFE fallback when traffic
        is FREE/READY and every unfinished AMR is already very tightly aligned.
        No fresh direct-zero stream is published while one robot is merely waiting
        for the other.
        """
        self.nav_status = ["", ""]
        start = list(self.world_pose)
        for i, (x, y, yaw) in enumerate(goals):
            self.goal_pubs[i].publish(self.pose(x, y, yaw))
            print(f"[PRE_DOCK GOAL] AMR{i+1}: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}deg")

        done = [False, False]
        prox_stable = [0, 0]
        handoff_dist = float(self.dock_cfg.get("pre_dock_handoff_distance_m", 0.22))
        handoff_yaw = math.radians(float(self.dock_cfg.get("pre_dock_handoff_yaw_deg", 3.0)))
        handoff_need = max(1, int(self.dock_cfg.get("pre_dock_handoff_stable_cycles", 5)))
        deadline = time.monotonic() + timeout
        last = 0.0

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.08)
            traffic_state = self._traffic_state_name()
            traffic_free = self._traffic_is_free_for_predock_fallback()

            # First let the baseline Nav2 statuses finish naturally.  In particular
            # ACTIVE:ROTATING_FINAL must never be overridden by tray_cmd_vel=0 while
            # traffic is YIELDING.
            for i, st in enumerate(self.nav_status):
                if st.startswith("SUCCEEDED") and not done[i]:
                    done[i] = True
                    prox_stable[i] = handoff_need
                    self.stop_tray_direct(i, 2)  # short one-shot settle only
                    print(f"[PRE_DOCK STATUS COMPLETE] AMR{i+1}: {st}")
                elif st.startswith("FAILED") and not done[i]:
                    print(f"[PRE_DOCK FAIL] AMR{i+1}: {st}")
                    self.stop_tray_direct(0); self.stop_tray_direct(1)
                    return False

            # Tight physical fallback.  It is deliberately disabled during any
            # conflict/yield/clearance session and while a robot is traffic-paused.
            # It also never hands off one active robot by itself: the pair must be
            # simultaneously ready (or the other robot already finished normally).
            for i, st in enumerate(self.nav_status):
                if done[i]:
                    continue
                p = self.world_pose[i]
                if p is None or not traffic_free or st.startswith("PAUSED"):
                    prox_stable[i] = 0
                    continue
                gx, gy, gyaw = goals[i]
                remain = math.hypot(gx - p[0], gy - p[1])
                yaw_err = abs(wrap(gyaw - p[2]))
                if remain <= handoff_dist and yaw_err <= handoff_yaw:
                    prox_stable[i] = min(handoff_need, prox_stable[i] + 1)
                else:
                    prox_stable[i] = 0

            pair_fallback_ready = traffic_free and all(
                done[i] or prox_stable[i] >= handoff_need for i in range(2)
            )
            if not all(done) and pair_fallback_ready:
                for i in range(2):
                    if done[i]:
                        continue
                    done[i] = True
                    self.stop_tray_direct(i, 2)
                    p = self.world_pose[i]
                    gx, gy, gyaw = goals[i]
                    remain = math.hypot(gx - p[0], gy - p[1]) if p is not None else float("nan")
                    yaw_err = abs(wrap(gyaw - p[2])) if p is not None else float("nan")
                    print(
                        f"[PRE_DOCK PAIR PROXIMITY FALLBACK] AMR{i+1}: "
                        f"traffic={traffic_state} remain={remain:.3f}m "
                        f"yaw_err={math.degrees(yaw_err):.1f}deg"
                    )

            if all(done):
                # Both normal Nav2 goals are complete (or the pair-safe fallback has
                # completed both at once), so it is now safe to seize final-ingress
                # control for ArUco docking.
                self.stop_tray_direct(0, 4)
                self.stop_tray_direct(1, 4)
                print(
                    "[PRE_DOCK COMPLETE V2.7] both AMRs finished without traffic/Nav2 "
                    "control contention; switching to ArUco final docking"
                )
                return True

            now = time.monotonic()
            if now - last > 1.0:
                parts = []
                for i, (gx, gy, gyaw) in enumerate(goals):
                    p = self.world_pose[i]
                    if p is None:
                        parts.append(f"AMR{i+1}:NO_POSE status={self.nav_status[i] or 'WAIT'}")
                        continue
                    remain = math.hypot(gx - p[0], gy - p[1])
                    yaw_e = abs(wrap(gyaw - p[2]))
                    moved = 0.0
                    if start[i] is not None:
                        moved = math.hypot(p[0] - start[i][0], p[1] - start[i][1])
                    if done[i]:
                        handoff_text = "DONE_NAV2"
                    elif not traffic_free:
                        handoff_text = f"WAIT_TRAFFIC_{traffic_state}"
                    elif self.nav_status[i].startswith("PAUSED"):
                        handoff_text = "WAIT_TRAFFIC_PAUSE"
                    else:
                        handoff_text = f"TIGHT_PAIR={prox_stable[i]}/{handoff_need}"
                    parts.append(
                        f"AMR{i+1}:status={self.nav_status[i] or 'WAIT'} remain={remain:.2f}m "
                        f"yaw={math.degrees(yaw_e):.1f}deg handoff={handoff_text} "
                        f"moved={moved:.2f}m"
                    )
                traffic = self.traffic_status[:160] if self.traffic_status else "READY"
                print("[PRE_DOCK RUN V2.7] " + " | ".join(parts) + f" | traffic={traffic}")
                last = now

        print(f"[PRE_DOCK FAIL] timeout status={self.nav_status} traffic={self._traffic_state_name()}")
        return False

    def stop_tray_direct(self, i: int, repeats: int = 5) -> None:
        z = Twist()
        for _ in range(repeats):
            if not rclpy.ok():
                break
            self.tray_cmd_pubs[i].publish(z)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.025)

    def dock_one(self, i: int, cart_home: tuple[float, float, float], dock_y: float, dock_x: float) -> bool:
        """V2.11 bed-style tray dock: ArUco ID -> fixed straight distance -> stop.

        This intentionally removes the V2.6~V2.10 world-pose insertion/recovery
        controller.  PRE_DOCK already selects the correct bay and yaw.  ArUco is
        used only to authenticate the tray entrance; then the AMR measures its own
        travelled distance exactly like the proven hospital-bed FORWARD_TARGET
        state machine.  No marker pair is required and marker visibility is not
        required after motion starts.
        """
        cfg = self.dock_cfg
        outer_ids = [int(v) for v in (
            cfg.get("amr1_outer_ids", cfg.get("amr1_ids", [40, 41])) if i == 0
            else cfg.get("amr2_outer_ids", cfg.get("amr2_ids", [42, 43]))
        )]
        center_id = int(cfg.get("center_id", 44))
        allowed_ids = set(outer_ids + [center_id])
        single_need = max(1, int(cfg.get("single_marker_good_cycles", 3)))
        stale_limit = float(cfg.get("aruco_stale_s", 0.7))

        length, _width, _dock_x_cfg, _dock_y_cfg = self.cart_geometry()
        configured_distance = float(cfg.get("fixed_forward_distance_m", 0.0))
        if configured_distance > 0.0:
            target_distance = configured_distance
            distance_source = "config"
        else:
            # PRE_DOCK local x = -length/2 - standoff, final dock x = dock_x.
            # Current geometry: 1.10 + 0.95 + 0.00 = 2.05 m.
            target_distance = 0.5 * length + float(cfg.get("pre_dock_standoff_m", 0.95)) + float(dock_x)
            distance_source = "tray geometry"
        target_distance = max(0.20, target_distance)
        speed = max(0.03, abs(float(cfg.get("fixed_forward_speed_mps", 0.16))))
        timeout = max(8.0, float(cfg.get("fixed_forward_timeout_s", 32.0)))
        settle_s = max(0.0, float(cfg.get("fixed_forward_settle_s", 0.55)))

        lane = "LEFT" if i == 0 else "RIGHT"
        print(
            f"[ARUCO FIXED DOCK V2.11] AMR{i+1} {lane} bay allowed={sorted(allowed_ids)} "
            f"distance={target_distance:.3f}m ({distance_source}) speed={speed:.3f}m/s"
        )

        deadline = time.monotonic() + timeout
        lock_id: int | None = None
        good = 0
        start_pose: tuple[float, float, float] | None = None
        last_print = 0.0

        # Keep traffic bypass asserted for this whole direct-motion phase.
        self.set_tray_docking_active(i, True, force=True)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.03)
            pose = self.world_pose[i]
            if pose is None:
                self.stop_tray_direct(i, 1)
                continue

            if start_pose is None:
                payload = self.aruco[i]
                fresh = payload is not None and (time.monotonic() - self.aruco_rx[i]) <= stale_limit
                visible = payload.get("visible_ids", []) if fresh and isinstance(payload, dict) else []
                candidates = sorted({int(v) for v in visible}.intersection(allowed_ids))
                candidate = center_id if center_id in candidates else (int(candidates[0]) if candidates else None)

                if candidate is None:
                    lock_id = None
                    good = 0
                    self.stop_tray_direct(i, 1)
                    if time.monotonic() - last_print > 0.8:
                        print(f"[ARUCO WAIT V2.11] AMR{i+1} expected={sorted(allowed_ids)} visible={visible}")
                        last_print = time.monotonic()
                    continue

                if lock_id == candidate:
                    good += 1
                else:
                    lock_id = candidate
                    good = 1

                self.stop_tray_direct(i, 1)
                if good < single_need:
                    continue

                start_pose = pose
                print(
                    f"[ARUCO ID LOCK V2.11] AMR{i+1}: ID{lock_id} stable {good}/{single_need} -> "
                    f"STRAIGHT {target_distance:.3f}m; marker no longer required"
                )
                continue

            moved = math.hypot(pose[0] - start_pose[0], pose[1] - start_pose[1])
            if moved >= target_distance:
                self.stop_tray_direct(i, 10)
                if settle_s > 0.0:
                    time.sleep(settle_s)
                live_cart = self.cart_pose() or cart_home
                lx, ly = self.world_to_local(live_cart, pose[0], pose[1])
                desired_y = dock_y if i == 0 else -dock_y
                print(
                    f"[FIXED DISTANCE DOCKED V2.11] AMR{i+1}: moved={moved:.3f}/{target_distance:.3f}m "
                    f"local=({lx:+.3f},{ly:+.3f}) bay_y={desired_y:+.3f}; ready for lift/FixedJoint"
                )
                return True

            cmd = Twist(); cmd.linear.x = speed; cmd.angular.z = 0.0
            self.tray_cmd_pubs[i].publish(cmd)
            if time.monotonic() - last_print > 0.45:
                print(
                    f"[FIXED INSERT V2.11] AMR{i+1}: ID{lock_id} moved={moved:.3f}/{target_distance:.3f}m "
                    f"remaining={max(0.0,target_distance-moved):.3f}m cmd=({speed:+.3f},+0.000)"
                )
                last_print = time.monotonic()

        self.stop_tray_direct(i, 10)
        print(f"[FIXED DISTANCE DOCK FAIL V2.11] AMR{i+1}: timeout; ID={lock_id} good={good}/{single_need}")
        return False

    def start_child_launch(self, name: str, package: str, launch_file: str) -> bool:
        project_root = Path(self.cfg.get("_project_root", "."))
        logdir = project_root / "output" / "tray_integrated_v2_9"; logdir.mkdir(parents=True, exist_ok=True)
        fh = open(logdir / f"{name}.log", "w", encoding="utf-8")
        p = subprocess.Popen(["ros2","launch",package,launch_file], stdout=fh, stderr=subprocess.STDOUT, start_new_session=True)
        self.child_processes.append(p); self.child_logs.append(fh)
        print(f"[LATE LAUNCH] {name} pid={p.pid}")
        return True

    def wait_publishers(self, topics: list[str], timeout: float) -> bool:
        end=time.monotonic()+timeout; last=0.0
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.1)
            missing=[t for t in topics if self.count_publishers(t)<=0]
            if not missing: return True
            if time.monotonic()-last>2: print("[LATE WAIT] missing="+",".join(missing)); last=time.monotonic()
        return False

    def cart_command(self, command: str, attached: bool, timeout: float = 12.0) -> bool:
        request_id = f"tray-{command.lower()}-{uuid.uuid4().hex[:8]}"
        msg = String()
        msg.data = json.dumps({"command": command, "request_id": request_id, "timestamp": time.time()}, separators=(",", ":"))
        self.cart_command_pub.publish(msg)
        print(f"[CART] {command} request")
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if bool(self.cart_status.get("attached", False)) == attached and not bool(self.cart_status.get("pending", False)):
                print(f"[CART] {'ATTACHED' if attached else 'DETACHED'} y_offsets={self.cart_status.get('measured_lateral_offsets_m', [])}")
                return True
        print(f"[CART FAIL] {command}: {self.cart_status}")
        return False

    def stop_coop(self, repeats: int = 10) -> None:
        z = Twist()
        for _ in range(repeats):
            if not rclpy.ok():
                break
            self.coop_cmd_pub.publish(z)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.025)

    def lock_cooperative_pose_from_cart(self, timeout: float = 8.0) -> bool:
        p=self.cart_pose()
        if p is None:
            print("[COOP POSE WARN] cart pose unavailable")
            return False
        self.coop_pose_locked=False
        deadline=time.monotonic()+timeout
        print(f"[COOP POSE] lock actual attached cart pose x={p[0]:.3f} y={p[1]:.3f} yaw={math.degrees(p[2]):.1f}deg")
        while rclpy.ok() and time.monotonic()<deadline:
            msg=PoseWithCovarianceStamped(); msg.header.frame_id="map"; msg.header.stamp=self.get_clock().now().to_msg()
            msg.pose.pose.position.x=p[0]; msg.pose.pose.position.y=p[1]
            msg.pose.pose.orientation.z=math.sin(p[2]*0.5); msg.pose.pose.orientation.w=math.cos(p[2]*0.5)
            self.coop_initialpose_pub.publish(msg)
            rclpy.spin_once(self,timeout_sec=0.10)
            if self.coop_pose_locked:
                print("[COOP POSE] locked")
                return True
            time.sleep(0.10)
        print("[COOP POSE FAIL] cooperative pose lock timeout")
        return False

    def coop_navigate(self, x: float, y: float, yaw: float, timeout: float = 360.0) -> bool:
        self.coop_status = ""
        self.coop_goal_pub.publish(self.pose(x, y, yaw))
        print(f"[COOP GOAL] x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}deg")
        active = False
        deadline = time.monotonic() + timeout
        last = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.10)
            if self.coop_status.startswith("ACTIVE"):
                active = True
            if active and self.coop_status.startswith("SUCCEEDED"):
                print("[COOP COMPLETE] destination reached")
                return True
            if self.coop_status.startswith("FAILED"):
                print(f"[COOP FAIL] {self.coop_status}")
                return False
            if time.monotonic() - last > 1.2:
                print(f"[COOP RUN] {self.coop_status or 'WAIT'}")
                last = time.monotonic()
        return False


def main() -> int:
    args = parse_args()
    root = Path(__file__).resolve().parent
    config_path = (root / args.config).resolve() if not Path(args.config).is_absolute() else Path(args.config)
    cfg = json.loads(config_path.read_text(encoding="utf-8"))
    cfg["_project_root"] = str(config_path.parent.parent.parent)

    rclpy.init(args=[])
    node = Manager(cfg)
    try:
        if not node.wait_ready():
            print("[MISSION FAIL] integrated runtime not ready")
            return 2
        cart_home = node.cart_pose()
        if cart_home is None:
            print("[MISSION FAIL] cart pose unavailable")
            return 2
        length, _width, dock_x, dock_y = node.cart_geometry()
        standoff = float(node.dock_cfg.get("pre_dock_standoff_m", 0.85))
        pre_x = -0.5 * length - standoff
        pre_goals = []
        for dy in (dock_y, -dock_y):
            wx, wy = node.local_to_world(cart_home, pre_x, dy)
            pre_goals.append((wx, wy, cart_home[2]))

        tx, ty, tyaw = fixed_target(cfg)

        # V2.11: release BOTH AMRs from their start bays before Nav2 dispatch.
        # These primitives are short and non-fatal; the normal Nav2 stack remains
        # authoritative immediately afterward.
        if not node.safe_egress_amr1():
            node.set_state("FAILED", "AMR1 safe egress failed")
            return 8
        if not node.safe_egress_amr2():
            node.set_state("FAILED", "AMR2 safe egress failed")
            return 8

        print("\n============= HOSPITAL_TOTAL_08091221 + FINAL TRAY GATE V2.11 =============")
        print(f"TRAY START  = ({cart_home[0]:.3f}, {cart_home[1]:.3f}, {math.degrees(cart_home[2]):.1f}deg)")
        print(f"FINAL GOAL  = ({tx:.3f}, {ty:.3f}, {math.degrees(tyaw):.1f}deg)")
        print("TRAFFIC     = hospital_total_08091221 latest baseline path_conflict_manager unchanged")
        print("ArUco gate  = LEFT 40/41 | CENTER 44 | RIGHT 42/43")
        print("END         = corridor transport to x=7.9/y=10.13, tray remains attached")
        print("===============================================================\n")

        node.set_state(
            "DISPATCH_AMRS",
            "V2.11 independent PRE_DOCK handoff: first arrival scans/docks immediately; peer Nav2 continues",
        )
        if not node.navigate_and_dock_pair(pre_goals, cart_home, dock_y, dock_x):
            node.set_state("FAILED", "V2.11 pre-dock navigation / immediate ArUco docking failed")
            return 3

        node.set_state("LIFT_AND_ATTACH", "dual yellow lift + dual FixedJoint")
        if not node.cart_command("ATTACH", True):
            node.set_state("FAILED", "cart attach failed")
            return 6

        node.set_state("COOPERATIVE_RUNTIME", "wait lazy Isaac cooperative bridge after attach")
        if not node.wait_publishers(["/coop/odom","/coop/scan_left","/coop/scan_right"], 30.0):
            node.set_state("FAILED", "Isaac cooperative bridge not ready after attach"); return 11
        node.start_child_launch("coop_nav", "hospital_tray_overlay", "cooperative_cart_nav.launch.py")
        if not node.wait_publishers(["/coop/center_goal/status","/coopnav/initial_pose_locked"], 90.0):
            node.set_state("FAILED", "cooperative Nav2 launch not ready"); return 12

        node.set_state("COOPERATIVE_READY", "lock cooperative map pose from actual attached cart")
        if not node.lock_cooperative_pose_from_cart():
            node.set_state("FAILED", "cooperative pose lock failed")
            return 9
        node.set_state("COOPERATIVE_NAV", f"fixed destination=({tx:.2f},{ty:.2f})")
        if not node.coop_navigate(tx, ty, tyaw):
            node.stop_coop()
            node.set_state("FAILED", "cooperative navigation failed")
            return 7

        node.stop_coop()
        node.set_state("TRANSPORT_COMPLETE", "x=7.9 destination reached; tray stays attached")
        print("\n[COMPLETE V2.11] dual safe-egress -> independent PRE_DOCK -> first-arrival ArUco scanner/dock -> dual attach -> corridor cooperative transport")
        print("[HOLD] tray remains attached at final position x=7.90, y=10.13")
        return 0
    finally:
        if rclpy.ok():
            node.stop_tray_direct(0, 2); node.stop_tray_direct(1, 2); node.stop_coop(3)
        for p in getattr(node, "child_processes", []):
            try:
                if p.poll() is None: os.killpg(p.pid, 15)
            except Exception: pass
        for fh in getattr(node, "child_logs", []):
            try: fh.close()
            except Exception: pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
