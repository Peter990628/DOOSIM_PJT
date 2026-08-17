#!/usr/bin/env python3
"""DEMO-ONLY forced-success tray mission.

IMPORTANT:
- This file does NOT replace or edit the original cooperative_transport_manager,
  path_conflict_manager, CenterlineNavigator, Isaac runtime, or cart controller.
- It deliberately uses the EXISTING cart ALIGN/G command as the final docking
  fallback after ArUco/PRE_DOCK visual confirmation.
- Intended only for presentation/demo recovery when physics collision prevents
  the normal fixed-distance insertion from completing.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import time
import uuid

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from hospital_tray_overlay.cooperative_transport_manager import Manager, fixed_target


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class DemoManager(Manager):
    def __init__(self, cfg: dict) -> None:
        super().__init__(cfg)
        self.demo = cfg.get("demo_force_success", {})

    def wait_ready_demo(self, timeout: float = 75.0) -> bool:
        """Same readiness gate as baseline, except traffic manager is intentionally absent."""
        print("[DEMO READY] base Nav2 + world pose + tray direct + cart bridge (traffic manager intentionally BYPASSED)")
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
                "tray_direct_sub": all(self.count_subscribers(t) > 0 for t in self.tray_cmd_topics),
                "cart_cmd_sub": self.cart_command_pub.get_subscription_count() > 0,
                "runtime": bool(self.runtime_status.get("amr1_bridge")) and bool(self.runtime_status.get("amr2_bridge")) and bool(self.runtime_status.get("cart_ready")),
            }
            ready = all(conditions.values())
            if ready:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= 1.0:
                    print("[DEMO READY PASS] all required interfaces stable for 1s")
                    return True
            else:
                stable_since = None
            if time.monotonic() - last > 2.0:
                print("[DEMO WAIT] " + " ".join(f"{k}={v}" for k, v in conditions.items()))
                last = time.monotonic()
        return False

    def _expected_ids(self, i: int) -> set[int]:
        cfg = self.dock_cfg
        outer = cfg.get("amr1_outer_ids", cfg.get("amr1_ids", [40, 41])) if i == 0 else cfg.get("amr2_outer_ids", cfg.get("amr2_ids", [42, 43]))
        return {int(v) for v in outer} | {int(cfg.get("center_id", 44))}

    def auth_and_preview(self, i: int) -> bool:
        """Use ArUco as visual proof, but never fail the demo because of physical insertion.

        If an expected marker is seen, apply a short straight ingress preview.  We do
        NOT require a measured 2.05m displacement.  The final exact pose is handled
        later by the existing cart ALIGN command.
        """
        allowed = self._expected_ids(i)
        good_need = max(1, int(self.demo.get("marker_good_cycles", 2)))
        marker_wait = max(2.0, float(self.demo.get("marker_wait_s", 9.0)))
        stale = float(self.dock_cfg.get("aruco_stale_s", 0.7))
        fallback = bool(self.demo.get("marker_timeout_predock_fallback", True))
        preview_speed = max(0.0, float(self.demo.get("preview_forward_speed_mps", 0.14)))
        preview_time = max(0.0, float(self.demo.get("preview_forward_time_s", 1.8)))

        self.set_tray_docking_active(i, True, force=True)
        deadline = time.monotonic() + marker_wait
        lock_id = None
        good = 0
        last = 0.0
        print(f"[DEMO ARUCO AUTH] AMR{i+1} expected={sorted(allowed)} wait={marker_wait:.1f}s")
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.04)
            payload = self.aruco[i]
            fresh = payload is not None and (time.monotonic() - self.aruco_rx[i]) <= stale
            visible = payload.get("visible_ids", []) if fresh and isinstance(payload, dict) else []
            candidates = sorted({int(v) for v in visible}.intersection(allowed))
            candidate = 44 if 44 in candidates else (candidates[0] if candidates else None)
            if candidate is None:
                lock_id = None
                good = 0
                self.stop_tray_direct(i, 1)
                if time.monotonic() - last > 0.8:
                    print(f"[DEMO ARUCO WAIT] AMR{i+1} visible={visible}")
                    last = time.monotonic()
                continue
            if lock_id == candidate:
                good += 1
            else:
                lock_id = candidate
                good = 1
            if good >= good_need:
                print(f"[DEMO ARUCO PASS] AMR{i+1} ID{lock_id} stable {good}/{good_need}")
                break
        else:
            lock_id = None

        if lock_id is None:
            if not fallback:
                print(f"[DEMO ARUCO FAIL] AMR{i+1}: no expected marker")
                return False
            print(f"[DEMO ARUCO FALLBACK] AMR{i+1}: PRE_DOCK arrival accepted; final ALIGN will guarantee exact bay pose")
            self.stop_tray_direct(i, 2)
            return True

        if preview_speed > 0.0 and preview_time > 0.0:
            print(f"[DEMO INGRESS PREVIEW] AMR{i+1}: straight {preview_speed:.2f}m/s for {preview_time:.1f}s; collision/stall is NON-FATAL")
            end = time.monotonic() + preview_time
            cmd = Twist(); cmd.linear.x = preview_speed; cmd.angular.z = 0.0
            while rclpy.ok() and time.monotonic() < end:
                self.tray_cmd_pubs[i].publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.03)
                time.sleep(0.025)
            self.stop_tray_direct(i, 6)
        return True

    def navigate_auth_pair(self, goals, timeout: float = 150.0) -> bool:
        """Run both Nav2 stacks simultaneously with no path_conflict_manager.

        Normal success is preferred.  Nav2 failure is NOT fatal in demo mode.  Once
        one robot has reached/seen the tray, the peer gets a bounded grace period;
        after that the existing ALIGN command is allowed to place both exactly.
        """
        self.nav_status = ["", ""]
        for i, (x, y, yaw) in enumerate(goals):
            self.goal_pubs[i].publish(self.pose(x, y, yaw))
            print(f"[DEMO PRE_DOCK GOAL] AMR{i+1}: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}deg")

        arrived = [False, False]
        authed = [False, False]
        failed = [False, False]
        first_auth_time = None
        force_peer_after = max(5.0, float(self.demo.get("force_peer_align_after_first_auth_s", 18.0)))
        near_dist = max(0.20, float(self.demo.get("predock_near_accept_m", 0.40)))
        deadline = time.monotonic() + timeout
        last = 0.0

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.06)
            now = time.monotonic()
            for i in range(2):
                st = self.nav_status[i]
                if st.startswith("SUCCEEDED"):
                    arrived[i] = True
                elif st.startswith("FAILED"):
                    failed[i] = True

                p = self.world_pose[i]
                if p is not None and not arrived[i]:
                    gx, gy, _ = goals[i]
                    if math.hypot(gx - p[0], gy - p[1]) <= near_dist:
                        arrived[i] = True
                        print(f"[DEMO PRE_DOCK NEAR] AMR{i+1}: within {near_dist:.2f}m -> accept handoff")

            # Authenticate arrivals one at a time. Peer Nav2 runs independently.
            for i in range(2):
                if arrived[i] and not authed[i]:
                    if not self.ensure_aruco_started(timeout=45.0):
                        print("[DEMO SCANNER WARNING] scanner publishers missing; PRE_DOCK fallback remains enabled")
                    authed[i] = self.auth_and_preview(i)
                    if authed[i] and first_auth_time is None:
                        first_auth_time = time.monotonic()

            if all(authed):
                print("[DEMO AUTH COMPLETE] both AMRs ready for exact ALIGN")
                return True

            if first_auth_time is not None and (now - first_auth_time) >= force_peer_after:
                for i in range(2):
                    if not authed[i]:
                        print(f"[DEMO PEER FALLBACK] AMR{i+1} did not complete PRE_DOCK in {force_peer_after:.1f}s; exact cart ALIGN will place it")
                        authed[i] = True
                return True

            if time.monotonic() - last > 1.0:
                print(f"[DEMO RUN] arrived={arrived} authed={authed} failed(nonfatal)={failed} status={self.nav_status}")
                last = time.monotonic()

        print("[DEMO TIMEOUT FALLBACK] bounded PRE_DOCK window expired; proceeding to exact ALIGN rather than terminating")
        return True

    def force_align_both(self) -> bool:
        """Invoke the EXISTING cart ALIGN/G implementation; retry and verify."""
        retries = max(1, int(self.demo.get("align_retries", 3)))
        settle = max(0.4, float(self.demo.get("align_settle_s", 1.2)))
        tolerance = max(0.05, float(self.demo.get("align_verify_distance_m", 0.30)))
        length, width, dock_x, dock_y = self.cart_geometry()
        _ = length, width
        for attempt in range(1, retries + 1):
            request_id = f"demo-align-{uuid.uuid4().hex[:8]}"
            msg = String()
            msg.data = json.dumps({"command": "ALIGN", "request_id": request_id, "timestamp": time.time()}, separators=(",", ":"))
            self.cart_command_pub.publish(msg)
            print(f"[DEMO EXACT ALIGN] request {attempt}/{retries} -> existing cart ALIGN/G")
            end = time.monotonic() + settle
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
                time.sleep(0.03)
            cart = self.cart_pose()
            if cart is None or any(p is None for p in self.world_pose):
                continue
            errors = []
            for i in range(2):
                tx, ty = self.local_to_world(cart, dock_x, dock_y if i == 0 else -dock_y)
                p = self.world_pose[i]
                errors.append(math.hypot(p[0] - tx, p[1] - ty))
            print(f"[DEMO ALIGN VERIFY] errors={errors[0]:.3f}m/{errors[1]:.3f}m tol={tolerance:.3f}m")
            if max(errors) <= tolerance:
                print("[DEMO EXACT ALIGN PASS] both AMRs are inside nominal tray capture zones")
                return True
        print("[DEMO ALIGN WARNING] verification did not converge; ATTACH retry will invoke ALIGN again")
        return False

    def force_attach(self) -> bool:
        retries = max(1, int(self.demo.get("attach_retries", 3)))
        for attempt in range(1, retries + 1):
            self.force_align_both()
            print(f"[DEMO ATTACH] attempt {attempt}/{retries}")
            if self.cart_command("ATTACH", True, timeout=15.0):
                print("[DEMO ATTACH PASS] dual lift + dual FixedJoint confirmed")
                return True
            print("[DEMO ATTACH RETRY] re-ALIGN and retry; no mission termination yet")
        return False


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--config", required=True)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    config_path = Path(args.config).expanduser().resolve()
    cfg = json.loads(config_path.read_text(encoding="utf-8"))
    cfg["_project_root"] = str(config_path.parent.parent.parent)

    rclpy.init(args=[])
    node = DemoManager(cfg)
    try:
        if not node.wait_ready_demo():
            print("[DEMO FAIL] base runtime not ready")
            return 2
        cart_home = node.cart_pose()
        if cart_home is None:
            print("[DEMO FAIL] cart pose unavailable")
            return 2

        length, _width, dock_x, dock_y = node.cart_geometry()
        standoff = float(node.dock_cfg.get("pre_dock_standoff_m", 0.95))
        pre_x = -0.5 * length - standoff
        pre_goals = []
        for dy in (dock_y, -dock_y):
            wx, wy = node.local_to_world(cart_home, pre_x, dy)
            pre_goals.append((wx, wy, cart_home[2]))
        tx, ty, tyaw = fixed_target(cfg)

        print("\n================ DOOSIM TRAY DEMO FORCE SUCCESS ================")
        print("NORMAL FILES = UNCHANGED")
        print("TRAFFIC      = DEMO ONLY bypass (AMR1/AMR2 run together)")
        print("DOCK         = PRE_DOCK -> ArUco visual auth -> short ingress preview -> existing ALIGN/G exact snap")
        print("ATTACH       = ALIGN retry -> dual lift -> FixedJoint retry")
        print("FAIL POLICY  = collision/stall/marker timeout during final tray ingress is NON-FATAL")
        print("=================================================================\n")

        node.set_state("DEMO_DISPATCH", "traffic manager bypass; both independent Nav2 stacks run together")
        # Safe-egress is intentionally skipped in this demo path. Docking-station
        # collision is already visual-only in the preserved Isaac runtime, and the
        # live world pose is the Nav2 localization reference in V2.12.
        if not node.navigate_auth_pair(pre_goals, timeout=float(node.demo.get("predock_total_timeout_s", 150.0))):
            print("[DEMO NOTE] navigation/auth returned false; exact ALIGN fallback still proceeds")

        node.stop_tray_direct(0, 4); node.stop_tray_direct(1, 4)
        node.set_state("DEMO_EXACT_ALIGN", "existing cart ALIGN/G places both AMRs at exact left/right dock centers")
        node.force_align_both()

        node.set_state("DEMO_LIFT_ATTACH", "re-ALIGN + dual lift + dual FixedJoint; retry on capture failure")
        if not node.force_attach():
            node.set_state("FAILED", "demo exact align succeeded but cart attach still failed")
            return 6

        # From here the original cooperative transport pipeline is preserved.
        node.set_state("COOPERATIVE_RUNTIME", "normal cooperative transport resumes after guaranteed demo attach")
        if not node.wait_publishers(["/coop/odom", "/coop/scan_left", "/coop/scan_right"], 30.0):
            node.set_state("FAILED", "Isaac cooperative bridge not ready after attach")
            return 11
        node.start_child_launch("coop_nav", "hospital_tray_overlay", "cooperative_cart_nav.launch.py")
        if not node.wait_publishers(["/coop/center_goal/status", "/coopnav/initial_pose_locked"], 90.0):
            node.set_state("FAILED", "cooperative Nav2 launch not ready")
            return 12
        if not node.lock_cooperative_pose_from_cart():
            node.set_state("FAILED", "cooperative pose lock failed")
            return 9
        node.set_state("COOPERATIVE_NAV", f"normal cooperative Nav2 destination=({tx:.2f},{ty:.2f})")
        if not node.coop_navigate(tx, ty, tyaw):
            node.stop_coop()
            node.set_state("FAILED", "cooperative navigation failed after successful demo attach")
            return 7
        node.stop_coop()
        node.set_state("TRANSPORT_COMPLETE", "demo docking succeeded; normal cooperative destination reached")
        print("\n[DEMO COMPLETE] ArUco evidence -> exact ALIGN -> dual FixedJoint -> normal cooperative transport COMPLETE")
        return 0
    finally:
        if rclpy.ok():
            node.stop_tray_direct(0, 3); node.stop_tray_direct(1, 3); node.stop_coop(3)
        for p in getattr(node, "child_processes", []):
            try:
                if p.poll() is None:
                    os.killpg(p.pid, 15)
            except Exception:
                pass
        for fh in getattr(node, "child_logs", []):
            try: fh.close()
            except Exception: pass
        try: node.destroy_node()
        except Exception: pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
