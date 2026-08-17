#!/usr/bin/env python3
"""Two-AMR path conflict coordinator.

This node does not replace either robot's Nav2 stack.  It only watches the two
already-generated centerline paths and publishes a per-robot traffic pause Bool.
A tiny optional pause hook in centerline_navigator cancels the current FollowPath
segment, holds zero velocity, and replans the same final goal after release.

Rules:
* A robot doing non-Nav special motion (OCR/coupling/forced motion/elevator) has priority
  only when both robots are on the same floor and spatially close. Unknown/stale
  floor or pose data falls back to the legacy safe global pause.
* Otherwise compare paths only when both robots are on the same loaded floor.
* Future centerline points closer than overlap_distance_m form the next conflict zone.
* A robot already inside the conflict zone has priority.
* Otherwise the robot with the shorter path distance to the conflict entry wins.
  Near ties use AMR1 deterministically.
* The loser is paused only as it approaches the zone (hold_trigger_distance_m).
* After the winner passes the conflict zone, keep a release delay, then resume.
* Actual AMR1/AMR2 world-pose distance is used only as a backup/early-stop guard:
  one priority AMR always keeps moving, only the yielding AMR is paused, and the
  yielding AMR automatically resumes after the priority AMR has passed.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
import time
from typing import Optional

import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


@dataclass
class RobotState:
    name: str
    path: list[tuple[float, float]] = field(default_factory=list)
    pose: Optional[tuple[float, float]] = None
    pose_stamp: float = 0.0
    floor: str = "unknown"
    nav_active: bool = False
    path_stamp: float = 0.0
    status: str = ""


@dataclass
class ConflictSession:
    winner: str
    loser: str
    winner_start: int
    winner_end: int
    loser_start: int
    loser_end: int
    created: float
    loser_paused: bool = False
    release_started: Optional[float] = None


@dataclass
class PhysicalYieldSession:
    winner: str
    loser: str
    created: float
    opening_since: Optional[float] = None


@dataclass
class RouteReservationRequest:
    robot: str
    request_id: str
    phase: str
    priority: int
    first_seen: float
    last_seen: float


class PathConflictManager(Node):
    def __init__(self) -> None:
        super().__init__("path_conflict_manager")
        self.declare_parameter("overlap_distance_m", 1.00)
        self.declare_parameter("hold_trigger_distance_m", 4.00)
        self.declare_parameter("release_clearance_m", 1.20)
        self.declare_parameter("release_delay_sec", 2.00)
        self.declare_parameter("tie_distance_m", 0.35)
        self.declare_parameter("path_sample_step", 2)
        self.declare_parameter("amr1_map_topic", "/map")
        self.declare_parameter("amr2_map_topic", "/amr2/map")
        # centerline_path is intentionally latched for the whole active goal.
        # Do not expire a valid route only because the trip lasts >30 seconds.
        self.declare_parameter("path_stale_sec", 30.0)  # kept only for launch compatibility
        self.declare_parameter("special_stale_sec", 1.0)
        self.declare_parameter("special_spatial_enabled", True)
        self.declare_parameter("special_trigger_distance_m", 5.0)
        self.declare_parameter("special_release_distance_m", 6.0)
        self.declare_parameter("special_pose_stale_sec", 1.0)
        self.declare_parameter("head_on_enabled", False)
        self.declare_parameter("head_on_trigger_distance_m", 1.25)
        self.declare_parameter("head_on_dot_threshold", -0.70)
        self.declare_parameter("head_on_lateral_distance_m", 1.0)
        self.declare_parameter("head_on_forward_distance_m", 1.0)
        self.declare_parameter("head_on_timeout_sec", 25.0)

        # Backup guard using the ACTUAL Isaac world positions.  It never pauses both
        # AMRs: the existing priority AMR keeps moving and only the loser yields.
        self.declare_parameter("physical_guard_enabled", True)
        self.declare_parameter("physical_trigger_distance_m", 5.0)
        self.declare_parameter("physical_release_distance_m", 6.0)
        self.declare_parameter("physical_closing_rate_mps", 0.08)
        self.declare_parameter("physical_closing_confirm_sec", 0.20)
        self.declare_parameter("physical_opening_confirm_sec", 0.40)
        self.declare_parameter("physical_pose_stale_sec", 1.0)
        self.declare_parameter("reservation_enabled", True)
        self.declare_parameter("reservation_arbitration_sec", 0.40)
        self.declare_parameter("reservation_stale_sec", 2.0)

        self.overlap_distance = max(0.3, float(self.get_parameter("overlap_distance_m").value))
        self.hold_trigger_distance = max(0.5, float(self.get_parameter("hold_trigger_distance_m").value))
        self.release_clearance = max(0.2, float(self.get_parameter("release_clearance_m").value))
        self.release_delay = max(0.0, float(self.get_parameter("release_delay_sec").value))
        self.tie_distance = max(0.0, float(self.get_parameter("tie_distance_m").value))
        self.sample_step = max(1, int(self.get_parameter("path_sample_step").value))
        self.amr1_map_topic = str(self.get_parameter("amr1_map_topic").value).strip() or "/map"
        self.amr2_map_topic = str(self.get_parameter("amr2_map_topic").value).strip() or "/amr2/map"
        self.path_stale_sec = max(2.0, float(self.get_parameter("path_stale_sec").value))
        self.special_stale_sec = max(0.5, float(self.get_parameter("special_stale_sec").value))
        self.special_spatial_enabled = bool(self.get_parameter("special_spatial_enabled").value)
        self.special_trigger_distance = max(
            1.0, float(self.get_parameter("special_trigger_distance_m").value)
        )
        self.special_release_distance = max(
            self.special_trigger_distance + 0.2,
            float(self.get_parameter("special_release_distance_m").value),
        )
        self.special_pose_stale = max(
            0.3, float(self.get_parameter("special_pose_stale_sec").value)
        )
        self.head_on_enabled = bool(self.get_parameter("head_on_enabled").value)
        self.head_on_trigger_distance = max(0.8, float(self.get_parameter("head_on_trigger_distance_m").value))
        self.head_on_dot_threshold = min(-0.10, float(self.get_parameter("head_on_dot_threshold").value))
        self.head_on_lateral_distance = max(0.1, float(self.get_parameter("head_on_lateral_distance_m").value))
        self.head_on_forward_distance = max(0.1, float(self.get_parameter("head_on_forward_distance_m").value))
        self.head_on_timeout = max(5.0, float(self.get_parameter("head_on_timeout_sec").value))
        self.physical_guard_enabled = bool(self.get_parameter("physical_guard_enabled").value)
        self.physical_trigger_distance = max(1.0, float(self.get_parameter("physical_trigger_distance_m").value))
        self.physical_release_distance = max(
            self.physical_trigger_distance + 0.5,
            float(self.get_parameter("physical_release_distance_m").value),
        )
        self.physical_closing_rate = max(0.0, float(self.get_parameter("physical_closing_rate_mps").value))
        self.physical_closing_confirm = max(0.0, float(self.get_parameter("physical_closing_confirm_sec").value))
        self.physical_opening_confirm = max(0.0, float(self.get_parameter("physical_opening_confirm_sec").value))
        self.physical_pose_stale = max(0.3, float(self.get_parameter("physical_pose_stale_sec").value))
        self.reservation_enabled = bool(self.get_parameter("reservation_enabled").value)
        self.reservation_arbitration_sec = max(
            0.0, float(self.get_parameter("reservation_arbitration_sec").value)
        )
        self.reservation_stale_sec = max(
            1.0, float(self.get_parameter("reservation_stale_sec").value)
        )

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.robots = {
            "amr1": RobotState("amr1"),
            "amr2": RobotState("amr2"),
        }
        self.pause_pub = {
            "amr1": self.create_publisher(Bool, "/traffic_pause", latched),
            "amr2": self.create_publisher(Bool, "/amr2/traffic_pause", latched),
        }
        self.special_pause_pub = {
            "amr1": self.create_publisher(Bool, "/amr1/special_motion_pause", latched),
            "amr2": self.create_publisher(Bool, "/amr2/special_motion_pause", latched),
        }
        # Direct, agreed passing maneuver. Nav2 is paused first; Isaac then executes
        # local-frame E 1m -> forward 1m -> Q 1m for BOTH robots.
        self.maneuver_pub = {
            "amr1": self.create_publisher(String, "/amr1/traffic_maneuver/command", latched),
            "amr2": self.create_publisher(String, "/amr2/traffic_maneuver/command", latched),
        }
        self.status_pub = self.create_publisher(String, "/traffic_conflict/status", latched)
        self.reservation_status_pub = self.create_publisher(
            String, "/traffic_reservation/status", latched
        )
        self.reservation_grant_pub = {
            "amr1": self.create_publisher(String, "/amr1/route_reservation/grant", latched),
            "amr2": self.create_publisher(String, "/amr2/route_reservation/grant", latched),
        }

        self.create_subscription(Path, "/centerline_path", lambda m: self._on_path("amr1", m), latched)
        self.create_subscription(Path, "/amr2/centerline_path", lambda m: self._on_path("amr2", m), latched)
        self.create_subscription(String, "/amr1/world_pose", lambda m: self._on_pose("amr1", m), 20)
        self.create_subscription(String, "/amr2/world_pose", lambda m: self._on_pose("amr2", m), 20)
        self.create_subscription(String, "/center_goal/status", lambda m: self._on_nav_status("amr1", m), latched)
        self.create_subscription(String, "/amr2/center_goal/status", lambda m: self._on_nav_status("amr2", m), latched)
        self.create_subscription(
            OccupancyGrid, self.amr1_map_topic, lambda m: self._on_map("amr1", m), latched
        )
        self.create_subscription(
            OccupancyGrid, self.amr2_map_topic, lambda m: self._on_map("amr2", m), latched
        )
        self.create_subscription(Bool, "/amr1/special_motion_active", lambda m: self._on_special("amr1", m), 10)
        self.create_subscription(Bool, "/amr2/special_motion_active", lambda m: self._on_special("amr2", m), 10)
        self.create_subscription(String, "/amr1/traffic_maneuver/status", lambda m: self._on_maneuver_status("amr1", m), 10)
        self.create_subscription(String, "/amr2/traffic_maneuver/status", lambda m: self._on_maneuver_status("amr2", m), 10)
        self.create_subscription(
            String,
            "/amr1/route_reservation/request",
            lambda m: self._on_reservation_request("amr1", m),
            20,
        )
        self.create_subscription(
            String,
            "/amr2/route_reservation/request",
            lambda m: self._on_reservation_request("amr2", m),
            20,
        )

        # Tray final ingress uses direct local velocity commands instead of Nav2.
        # Clear the old PRE_DOCK path while docking so it cannot pause the docking
        # robot or the peer through a stale path-conflict decision.
        self.tray_docking_active = {"amr1": False, "amr2": False}
        self.create_subscription(
            Bool,
            "/amr1/tray_docking_active",
            lambda m: self._on_tray_docking("amr1", m),
            10,
        )
        self.create_subscription(
            Bool,
            "/amr2/tray_docking_active",
            lambda m: self._on_tray_docking("amr2", m),
            10,
        )

        self.special_active = {"amr1": False, "amr2": False}
        self.special_stamp = {"amr1": 0.0, "amr2": 0.0}
        self.special_since = {"amr1": float("inf"), "amr2": float("inf")}
        self.special_owner: Optional[str] = None
        self.special_spatial_hold = False

        self.passing_request_id = ""
        self.passing_started = 0.0
        self.passing_command_sent = False
        self.passing_status = {"amr1": "", "amr2": ""}
        self.passing_cooldown_until = 0.0

        self.session: Optional[ConflictSession] = None
        self.physical_session: Optional[PhysicalYieldSession] = None
        self.last_physical_separation: Optional[float] = None
        self.last_physical_stamp = 0.0
        self.physical_closing_since: Optional[float] = None
        self.last_pause = {"amr1": None, "amr2": None}
        self.last_special_pause = {"amr1": None, "amr2": None}
        self.last_status = ""
        self.reservation_owner: Optional[RouteReservationRequest] = None
        self.reservation_pending: dict[str, RouteReservationRequest] = {}
        self.last_reservation_status = ""
        self.reservation_release_pending: Optional[tuple[str, str]] = None
        self.create_timer(0.10, self._tick)
        self._set_pause("amr1", False)
        self._set_pause("amr2", False)
        self._set_special_pause("amr1", False)
        self._set_special_pause("amr2", False)
        self._publish_status("READY", "실제 AMR1/AMR2 centerline path 비교 대기")
        self.get_logger().info(
            f"경로 충돌 회피 준비: overlap<={self.overlap_distance:.2f}m, "
            f"hold_trigger={self.hold_trigger_distance:.1f}m, release_delay={self.release_delay:.1f}s, "
            f"special-spatial={'ON' if self.special_spatial_enabled else 'LEGACY_GLOBAL'} "
            f"{self.special_trigger_distance:.1f}m->{self.special_release_distance:.1f}m, "
            f"head-on-pass={'ON' if self.head_on_enabled else 'OFF'}, "
            f"physical-guard={'ON' if self.physical_guard_enabled else 'OFF'} "
            f"{self.physical_trigger_distance:.1f}m->{self.physical_release_distance:.1f}m, "
            f"loaded-route-reservation={'ON' if self.reservation_enabled else 'OFF'} "
            f"stale={self.reservation_stale_sec:.1f}s, "
            f"maps={self.amr1_map_topic}/{self.amr2_map_topic}"
        )

    def _publish_reservation_grant(
        self, robot: str, request_id: str, granted: bool, phase: str, detail: str
    ) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "robot": robot,
                "request_id": request_id,
                "resource": "LOADED_TRANSPORT_ROUTE",
                "granted": bool(granted),
                "phase": phase,
                "detail": detail,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.reservation_grant_pub[robot].publish(msg)

    def _publish_reservation_status(self, state: str, detail: str, **extra) -> None:
        payload = {
            "state": state,
            "resource": "LOADED_TRANSPORT_ROUTE",
            "detail": detail,
            **extra,
        }
        text = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        if text == self.last_reservation_status:
            return
        self.last_reservation_status = text
        msg = String()
        msg.data = text
        self.reservation_status_pub.publish(msg)

    def _on_reservation_request(self, robot: str, msg: String) -> None:
        if not self.reservation_enabled:
            return
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if str(payload.get("robot", robot)).strip().lower() != robot:
            return
        request_id = str(payload.get("request_id", "")).strip()
        action = str(payload.get("action", "")).strip().upper()
        if not request_id or action not in {"ACQUIRE", "HEARTBEAT", "RELEASE"}:
            return
        now = time.monotonic()
        owner = self.reservation_owner
        if action == "RELEASE":
            self.reservation_pending.pop(robot, None)
            if owner is not None and owner.robot == robot and owner.request_id == request_id:
                self._publish_reservation_grant(
                    robot, request_id, False, owner.phase, "explicit safe-point release"
                )
                self.reservation_owner = None
                self.reservation_release_pending = (robot, owner.phase)
                self._publish_reservation_status(
                    "RELEASED", f"{robot.upper()} released loaded transport route", robot=robot
                )
            return

        phase = str(payload.get("phase", "UNKNOWN")).strip().upper() or "UNKNOWN"
        priority = int(payload.get("priority", 0))
        if owner is not None and owner.robot == robot and owner.request_id == request_id:
            owner.last_seen = now
            return
        previous = self.reservation_pending.get(robot)
        first_seen = previous.first_seen if previous and previous.request_id == request_id else now
        self.reservation_pending[robot] = RouteReservationRequest(
            robot=robot,
            request_id=request_id,
            phase=phase,
            priority=priority,
            first_seen=first_seen,
            last_seen=now,
        )

    def _handle_route_reservation(self, now: float) -> bool:
        """Apply one non-preemptive reservation to all loaded narrow-route motion.

        The mission managers request this before entering the shared corridor/elevator
        chain and release it only after reaching a known safe endpoint.  A stale owner
        is deliberately held instead of automatically released: a dead mission process
        does not prove that its physical AMR/bed has cleared the bottleneck.
        """
        if not self.reservation_enabled:
            return False

        for robot, request in list(self.reservation_pending.items()):
            if now - request.last_seen > self.reservation_stale_sec:
                self.reservation_pending.pop(robot, None)

        owner = self.reservation_owner
        # A queued robot cannot inherit the route merely because RELEASE arrived.
        # The following tick must first verify the former owner's physical clearance.
        if owner is None and self.reservation_release_pending is not None:
            return False
        if owner is None and self.reservation_pending:
            ready = [
                request
                for request in self.reservation_pending.values()
                if now - request.first_seen >= self.reservation_arbitration_sec
            ]
            if ready:
                # Higher phase priority wins simultaneous requests.  Arrival time and
                # AMR name make the result deterministic.  Existing owners are never
                # preempted by a later, higher-priority request.
                owner = sorted(
                    ready,
                    key=lambda request: (-request.priority, request.first_seen, request.robot),
                )[0]
                self.reservation_owner = owner
                self.reservation_pending.pop(owner.robot, None)
                self._publish_reservation_grant(
                    owner.robot,
                    owner.request_id,
                    True,
                    owner.phase,
                    "exclusive loaded transport route granted",
                )
                self._publish_reservation_status(
                    "GRANTED",
                    f"{owner.robot.upper()} owns corridor + elevator + destination exit",
                    owner=owner.robot,
                    phase=owner.phase,
                    priority=owner.priority,
                    waiting=sorted(self.reservation_pending),
                )

        owner = self.reservation_owner
        if owner is None:
            return False

        stale = now - owner.last_seen > self.reservation_stale_sec
        if stale:
            self._set_pause("amr1", True)
            self._set_pause("amr2", True)
            self._set_special_pause("amr1", True)
            self._set_special_pause("amr2", True)
            self._publish_reservation_status(
                "OWNER_STALE_FAILSAFE_HOLD",
                "owner heartbeat lost; physical route clearance is unknown, hold both AMRs",
                owner=owner.robot,
                phase=owner.phase,
                stale_s=round(now - owner.last_seen, 2),
            )
            return True

        self._publish_reservation_status(
            "OCCUPIED",
            f"{owner.robot.upper()} owns loaded route; other AMR may prepare outside the gate",
            owner=owner.robot,
            phase=owner.phase,
            priority=owner.priority,
            waiting=sorted(self.reservation_pending),
        )
        # The reservation gates mission-level entry, not every motion of the other
        # robot.  A queued mission stops itself at its staging pose.  Before it asks
        # for the route it may continue OCR/pickup preparation, while the existing
        # path/special/physical conflict logic below remains active.
        return False

    @staticmethod
    def _floor_from_map(msg: OccupancyGrid) -> str:
        w, h = int(msg.info.width), int(msg.info.height)
        if (w, h) == (1528, 841):
            return "1f"
        if (w, h) == (1512, 841):
            return "2f"
        return f"map:{w}x{h}"

    def _on_map(self, robot: str, msg: OccupancyGrid) -> None:
        floor = self._floor_from_map(msg)
        if self.robots[robot].floor != floor:
            self.robots[robot].floor = floor
            self.get_logger().info(f"{robot.upper()} floor={floor}")
            # A floor transition invalidates any old cross-floor conflict session.
            if self.session is not None:
                self._clear_session("FLOOR_CHANGED")
            if self.physical_session is not None:
                self._clear_physical_session("FLOOR_CHANGED")
            self._reset_physical_measurement()

    def _on_path(self, robot: str, msg: Path) -> None:
        if self.tray_docking_active.get(robot, False):
            self.robots[robot].path = []
            self.robots[robot].nav_active = False
            return
        pts = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]
        self.robots[robot].path = pts
        self.robots[robot].path_stamp = time.monotonic()
        if len(pts) >= 2:
            self.robots[robot].nav_active = True

    def _on_pose(self, robot: str, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self.robots[robot].pose = (float(data["x"]), float(data["y"]))
            self.robots[robot].pose_stamp = time.monotonic()
        except Exception:
            return

    def _on_tray_docking(self, robot: str, msg: Bool) -> None:
        active = bool(msg.data)
        self.tray_docking_active[robot] = active
        if active:
            state = self.robots[robot]
            state.nav_active = False
            state.path = []
            state.status = "TRAY_DOCKING"
            self.get_logger().info(
                f"{robot.upper()} tray docking bypass ON; old Nav path cleared"
            )
        else:
            self.get_logger().info(f"{robot.upper()} tray docking bypass OFF")

    def _on_special(self, robot: str, msg: Bool) -> None:
        now = time.monotonic()
        active = bool(msg.data)
        if active and not self.special_active[robot]:
            self.special_since[robot] = now
        if not active:
            self.special_since[robot] = float("inf")
        self.special_active[robot] = active
        self.special_stamp[robot] = now

    def _special_now(self, robot: str, now: float) -> bool:
        return bool(
            self.special_active[robot]
            and (now - self.special_stamp[robot]) <= self.special_stale_sec
        )

    def _special_spatial_conflict(
        self, now: float, winner: str, loser: str
    ) -> tuple[bool, str, Optional[float]]:
        """Return whether special motion must pause the other robot.

        This first-stage spatial gate intentionally uses actual Isaac world-pose
        center distance rather than pretending the current static Nav2 footprint
        includes the attached bed.  Unknown floor/pose data is fail-safe and keeps
        the legacy global pause.  A separate release distance prevents chatter.
        """
        if not self.special_spatial_enabled:
            return True, "LEGACY_GLOBAL", None

        owner = self.robots[winner]
        other = self.robots[loser]
        if owner.floor == "unknown" or other.floor == "unknown":
            return True, "UNKNOWN_FLOOR_FAILSAFE", None
        if owner.floor != other.floor:
            return False, f"DIFFERENT_FLOORS:{owner.floor}/{other.floor}", None
        if owner.pose is None or other.pose is None:
            return True, "MISSING_POSE_FAILSAFE", None
        if (
            now - owner.pose_stamp > self.special_pose_stale
            or now - other.pose_stamp > self.special_pose_stale
        ):
            return True, "STALE_POSE_FAILSAFE", None

        separation = math.hypot(
            other.pose[0] - owner.pose[0],
            other.pose[1] - owner.pose[1],
        )
        threshold = (
            self.special_release_distance
            if self.special_spatial_hold
            else self.special_trigger_distance
        )
        if separation <= threshold:
            return True, "SAME_FLOOR_NEAR", separation
        return False, "SAME_FLOOR_CLEAR", separation

    def _on_maneuver_status(self, robot: str, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if str(payload.get("request_id", "")) != self.passing_request_id:
            return
        self.passing_status[robot] = str(payload.get("state", "")).upper()

    @staticmethod
    def _unit_heading(path: list[tuple[float, float]], index: int) -> Optional[tuple[float, float]]:
        if len(path) < 2:
            return None
        i0 = max(0, min(int(index), len(path) - 1))
        i1 = min(len(path) - 1, i0 + 6)
        if i1 == i0:
            i0 = max(0, i0 - 6)
        dx = float(path[i1][0] - path[i0][0])
        dy = float(path[i1][1] - path[i0][1])
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return None
        return dx / norm, dy / norm

    def _head_on_geometry(self, a: RobotState, b: RobotState) -> tuple[bool, float, float]:
        if not self.head_on_enabled or a.pose is None or b.pose is None:
            return False, float("inf"), 1.0
        ai = self._nearest_index(a.path, a.pose)
        bi = self._nearest_index(b.path, b.pose)
        ha = self._unit_heading(a.path, ai)
        hb = self._unit_heading(b.path, bi)
        if ha is None or hb is None:
            return False, float("inf"), 1.0
        ax, ay = a.pose
        bx, by = b.pose
        dx, dy = bx - ax, by - ay
        separation = math.hypot(dx, dy)
        if separation < 1e-6:
            return True, separation, -1.0
        ux, uy = dx / separation, dy / separation
        heading_dot = ha[0] * hb[0] + ha[1] * hb[1]
        # Both robots must actually face toward each other, not merely cross at an intersection.
        a_faces_b = ha[0] * ux + ha[1] * uy
        b_faces_a = -(hb[0] * ux + hb[1] * uy)
        head_on = bool(
            heading_dot <= self.head_on_dot_threshold
            and a_faces_b >= 0.55
            and b_faces_a >= 0.55
        )
        return head_on, separation, heading_dot

    def _publish_maneuver_command(self, robot: str, command: str) -> None:
        msg = String()
        msg.data = json.dumps({
            "request_id": self.passing_request_id,
            "command": command,
            "robot": robot,
            "lateral_distance_m": self.head_on_lateral_distance,
            "forward_distance_m": self.head_on_forward_distance,
        }, ensure_ascii=False, separators=(",", ":"))
        self.maneuver_pub[robot].publish(msg)

    def _start_head_on_passing(self, now: float, separation: float, heading_dot: float) -> None:
        self.session = None
        self.passing_request_id = f"headon-{int(time.time() * 1000)}"
        self.passing_started = now
        self.passing_command_sent = False
        self.passing_status = {"amr1": "", "amr2": ""}
        # First cancel/pause BOTH Nav2 controllers. Direct motion starts only after
        # both centerline navigators report PAUSED:TRAFFIC.
        self._set_pause("amr1", True)
        self._set_pause("amr2", True)
        self._publish_status(
            "HEAD_ON_PAUSE",
            "head-on single-path encounter: pause both Nav2 before agreed right-side pass",
            request_id=self.passing_request_id,
            separation_m=round(separation, 2),
            heading_dot=round(heading_dot, 3),
        )

    def _handle_head_on_passing(self, now: float) -> bool:
        if not self.passing_request_id:
            return False
        self._set_pause("amr1", True)
        self._set_pause("amr2", True)

        elapsed = now - self.passing_started
        failed = [r for r, state in self.passing_status.items() if state.startswith("FAILED") or state.startswith("REJECTED")]
        if failed or elapsed > self.head_on_timeout:
            if self.passing_command_sent:
                self._publish_maneuver_command("amr1", "CANCEL")
                self._publish_maneuver_command("amr2", "CANCEL")
            # Fail-safe: never silently resume into the same face-to-face conflict.
            self._publish_status(
                "HEAD_ON_FAILED_HOLD",
                "passing maneuver failed/timed out; both AMRs remain traffic-paused for safety",
                request_id=self.passing_request_id,
                failed=failed,
                elapsed_s=round(elapsed, 1),
            )
            return True

        if not self.passing_command_sent:
            both_paused = all(self.robots[r].status.startswith("PAUSED:TRAFFIC") for r in ("amr1", "amr2"))
            if not both_paused:
                return True
            self._publish_maneuver_command("amr1", "START")
            self._publish_maneuver_command("amr2", "START")
            self.passing_command_sent = True
            self._publish_status(
                "HEAD_ON_PASSING",
                "both Nav2 paused -> both execute E 1m, forward 1m, Q 1m",
                request_id=self.passing_request_id,
            )
            return True

        if all(self.passing_status[r] == "COMPLETE" for r in ("amr1", "amr2")):
            request_id = self.passing_request_id
            self.passing_request_id = ""
            self.passing_command_sent = False
            self.passing_status = {"amr1": "", "amr2": ""}
            self.passing_cooldown_until = now + 2.0
            # Releasing traffic_pause makes each centerline navigator replan the
            # SAME final goal from its new lane position.
            self._set_pause("amr1", False)
            self._set_pause("amr2", False)
            self._publish_status(
                "HEAD_ON_COMPLETE",
                "agreed pass complete; both AMRs replan their original final goals",
                request_id=request_id,
            )
            return True
        return True

    def _on_nav_status(self, robot: str, msg: String) -> None:
        status = str(msg.data)
        state = self.robots[robot]
        if self.tray_docking_active.get(robot, False):
            state.status = "TRAY_DOCKING"
            state.nav_active = False
            state.path = []
            return
        state.status = status
        if status.startswith("ACTIVE") or status.startswith("PAUSED"):
            state.nav_active = True
        elif status.startswith("SUCCEEDED") or status.startswith("FAILED") or status.startswith("READY"):
            state.nav_active = False
            if status.startswith("SUCCEEDED") or status.startswith("FAILED"):
                state.path = []

    def _set_pause(self, robot: str, pause: bool) -> None:
        pause = bool(pause)
        if self.last_pause[robot] == pause:
            return
        msg = Bool()
        msg.data = pause
        self.pause_pub[robot].publish(msg)
        self.last_pause[robot] = pause
        self.get_logger().warning(f"{robot.upper()} TRAFFIC_PAUSE={pause}") if pause else self.get_logger().info(
            f"{robot.upper()} TRAFFIC_PAUSE={pause}"
        )

    def _set_special_pause(self, robot: str, pause: bool) -> None:
        pause = bool(pause)
        if self.last_special_pause[robot] == pause:
            return
        msg = Bool()
        msg.data = pause
        self.special_pause_pub[robot].publish(msg)
        self.last_special_pause[robot] = pause
        if pause:
            self.get_logger().warning(f"{robot.upper()} SPECIAL_MOTION_PAUSE=True")
        else:
            self.get_logger().info(f"{robot.upper()} SPECIAL_MOTION_PAUSE=False")

    def _publish_status(self, state: str, detail: str, **extra) -> None:
        payload = {"state": state, "detail": detail, **extra}
        text = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    @staticmethod
    def _nearest_index(path: list[tuple[float, float]], pose: Optional[tuple[float, float]]) -> int:
        if not path or pose is None:
            return 0
        px, py = pose
        best_i = 0
        best_d2 = float("inf")
        for i, (x, y) in enumerate(path):
            d2 = (x - px) ** 2 + (y - py) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        return best_i

    @staticmethod
    def _distance_along(path: list[tuple[float, float]], start: int, end: int) -> float:
        if not path or end <= start:
            return 0.0
        end = min(end, len(path) - 1)
        start = max(0, start)
        total = 0.0
        for i in range(start + 1, end + 1):
            total += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        return total

    def _find_conflict(self, a: RobotState, b: RobotState) -> Optional[tuple[int, int, int, int]]:
        if len(a.path) < 2 or len(b.path) < 2:
            return None
        ai0 = self._nearest_index(a.path, a.pose)
        bi0 = self._nearest_index(b.path, b.pose)
        threshold = self.overlap_distance
        cell = threshold

        # Hash remaining B path points so the comparison stays cheap even for long paths.
        buckets: dict[tuple[int, int], list[tuple[int, float, float]]] = {}
        for j in range(bi0, len(b.path), self.sample_step):
            x, y = b.path[j]
            key = (math.floor(x / cell), math.floor(y / cell))
            buckets.setdefault(key, []).append((j, x, y))

        pairs: list[tuple[int, int]] = []
        threshold2 = threshold * threshold
        for i in range(ai0, len(a.path), self.sample_step):
            x, y = a.path[i]
            gx, gy = math.floor(x / cell), math.floor(y / cell)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for j, bx, by in buckets.get((gx + dx, gy + dy), ()): 
                        if (x - bx) ** 2 + (y - by) ** 2 <= threshold2:
                            pairs.append((i, j))
        if not pairs:
            return None

        # Do not merge disconnected future intersections into one huge hold zone.
        # Select only the nearest contiguous conflict island on A's remaining path.
        a_indices = sorted({i for i, _ in pairs})
        max_gap = max(2, self.sample_step * 3)
        groups: list[list[int]] = []
        current = [a_indices[0]]
        for idx in a_indices[1:]:
            if idx - current[-1] <= max_gap:
                current.append(idx)
            else:
                groups.append(current)
                current = [idx]
        groups.append(current)
        group = min(groups, key=lambda g: self._distance_along(a.path, ai0, g[0]))
        a_start, a_end = group[0], group[-1]
        island_pairs = [(i, j) for i, j in pairs if a_start <= i <= a_end]
        return (
            a_start,
            a_end,
            min(j for _, j in island_pairs),
            max(j for _, j in island_pairs),
        )

    def _current_progress(self, robot: str) -> int:
        state = self.robots[robot]
        return self._nearest_index(state.path, state.pose)

    def _select_priority(
        self,
        a: RobotState,
        b: RobotState,
        a_start: int,
        a_end: int,
        b_start: int,
        b_end: int,
    ) -> tuple[str, str, float, float]:
        ai = self._nearest_index(a.path, a.pose)
        bi = self._nearest_index(b.path, b.pose)
        a_inside = a_start <= ai <= a_end
        b_inside = b_start <= bi <= b_end
        da = self._distance_along(a.path, ai, a_start) if ai < a_start else 0.0
        db = self._distance_along(b.path, bi, b_start) if bi < b_start else 0.0

        if a_inside and not b_inside:
            return "amr1", "amr2", da, db
        if b_inside and not a_inside:
            return "amr2", "amr1", da, db
        if a_inside and b_inside:
            # Late detection fallback: let the robot closer to leaving finish first.
            a_exit = self._distance_along(a.path, ai, a_end)
            b_exit = self._distance_along(b.path, bi, b_end)
            if b_exit + self.tie_distance < a_exit:
                return "amr2", "amr1", da, db
            return "amr1", "amr2", da, db
        # Outside the conflict island, the AMR that already owns the loaded route
        # keeps priority.  This does not preempt a robot physically inside the zone;
        # the occupant checks above intentionally remain stronger.
        reservation_owner = self.reservation_owner
        if reservation_owner is not None:
            if time.monotonic() - reservation_owner.last_seen <= self.reservation_stale_sec:
                if reservation_owner.robot == "amr2":
                    return "amr2", "amr1", da, db
                return "amr1", "amr2", da, db
        if db + self.tie_distance < da:
            return "amr2", "amr1", da, db
        return "amr1", "amr2", da, db

    def _loser_distance_to_entry(self, session: ConflictSession) -> float:
        state = self.robots[session.loser]
        idx = self._nearest_index(state.path, state.pose)
        entry = session.loser_start
        if idx >= entry:
            return 0.0
        return self._distance_along(state.path, idx, entry)

    def _winner_cleared(self, session: ConflictSession) -> bool:
        state = self.robots[session.winner]
        if not state.nav_active and not state.path:
            return True
        if not state.path:
            return True
        idx = self._nearest_index(state.path, state.pose)
        if idx <= session.winner_end:
            return False
        # Require extra path progress after the last shared point.
        after = self._distance_along(state.path, session.winner_end, idx)
        return after >= self.release_clearance

    def _clear_session(self, reason: str) -> None:
        self._set_pause("amr1", False)
        self._set_pause("amr2", False)
        self.session = None
        self._publish_status("FREE", reason)

    def _reset_physical_measurement(self) -> None:
        self.last_physical_separation = None
        self.last_physical_stamp = 0.0
        self.physical_closing_since = None

    def _clear_physical_session(self, reason: str) -> None:
        ps = self.physical_session
        if ps is None:
            return
        self._set_pause(ps.loser, False)
        self._set_pause(ps.winner, False)
        self.physical_session = None
        self.physical_closing_since = None
        self._publish_status(
            "PHYSICAL_CLEAR",
            reason,
            winner=ps.winner,
            loser=ps.loser,
        )

    def _physical_metrics(self, now: float, a: RobotState, b: RobotState) -> Optional[tuple[float, float]]:
        if a.pose is None or b.pose is None:
            return None
        if (now - a.pose_stamp) > self.physical_pose_stale or (now - b.pose_stamp) > self.physical_pose_stale:
            return None
        separation = math.hypot(b.pose[0] - a.pose[0], b.pose[1] - a.pose[1])
        closing_rate = 0.0
        if self.last_physical_separation is not None and self.last_physical_stamp > 0.0:
            dt = now - self.last_physical_stamp
            if dt > 1e-3:
                # Positive = approaching, negative = moving apart.
                closing_rate = (self.last_physical_separation - separation) / dt
        self.last_physical_separation = separation
        self.last_physical_stamp = now
        return separation, closing_rate

    def _make_conflict_session_from_paths(self, now: float, a: RobotState, b: RobotState) -> bool:
        conflict = self._find_conflict(a, b)
        if conflict is None:
            return False
        a_start, a_end, b_start, b_end = conflict
        winner, loser, da, db = self._select_priority(a, b, a_start, a_end, b_start, b_end)
        self.session = ConflictSession(
            winner=winner,
            loser=loser,
            winner_start=a_start if winner == "amr1" else b_start,
            winner_end=a_end if winner == "amr1" else b_end,
            loser_start=b_start if loser == "amr2" else a_start,
            loser_end=b_end if loser == "amr2" else a_end,
            created=now,
        )
        self._publish_status(
            "CONFLICT_DETECTED",
            f"{winner.upper()} priority / {loser.upper()} yields before shared path",
            floor=a.floor,
            amr1_distance_to_conflict_m=round(da, 2),
            amr2_distance_to_conflict_m=round(db, 2),
            winner=winner,
            loser=loser,
        )
        return True

    def _handle_physical_guard(self, now: float, a: RobotState, b: RobotState) -> bool:
        """Backup using actual world positions. Never pauses both AMRs.

        Returns True only while a fallback physical-only yield owns traffic control.
        A normal path ConflictSession keeps its original release logic.
        """
        if not self.physical_guard_enabled:
            return False

        metrics = self._physical_metrics(now, a, b)
        if metrics is None:
            # Do not deadlock the scenario because a pose message disappeared.
            # Fall back to the original path-based coordinator.
            if self.physical_session is not None:
                self._clear_physical_session("world pose stale/missing -> return to original path control")
            self._reset_physical_measurement()
            return False
        separation, closing_rate = metrics

        # If the original path coordinator already has a priority, actual distance
        # only makes the SAME loser stop earlier/more reliably. Release remains the
        # original winner-cleared logic below.
        if self.session is not None:
            self.physical_closing_since = None
            if separation <= self.physical_trigger_distance and not self.session.loser_paused:
                self._set_pause(self.session.winner, False)
                self._set_pause(self.session.loser, True)
                self.session.loser_paused = True
                self._publish_status(
                    "YIELDING_PHYSICAL_BACKUP",
                    f"actual distance {separation:.2f}m -> keep {self.session.winner.upper()} moving / "
                    f"pause {self.session.loser.upper()}",
                    separation_m=round(separation, 2),
                    winner=self.session.winner,
                    loser=self.session.loser,
                )
            return False

        ps = self.physical_session
        if ps is not None:
            # The priority robot ALWAYS remains free. Only the loser is held.
            self._set_pause(ps.winner, False)
            self._set_pause(ps.loser, True)

            # If the priority robot finished its Nav goal, there is nothing left to
            # wait for. Resume the yielding AMR immediately so the scenario continues.
            if not self.robots[ps.winner].nav_active:
                self._clear_physical_session(f"{ps.winner.upper()} goal finished -> {ps.loser.upper()} resume")
                return False

            # Normal pass: after the winner has gone by, separation grows again.
            # Confirm that opening trend briefly, then release the loser automatically.
            if separation >= self.physical_release_distance and closing_rate < -0.03:
                if ps.opening_since is None:
                    ps.opening_since = now
                elif now - ps.opening_since >= self.physical_opening_confirm:
                    self._clear_physical_session(
                        f"{ps.winner.upper()} passed; distance {separation:.2f}m opening -> "
                        f"{ps.loser.upper()} automatic resume"
                    )
                    return False
            else:
                ps.opening_since = None
            return True

        # Physical-only fallback is only for two active Nav missions on the same floor.
        # Special motions were already handled above and keep their original priority.
        if not a.nav_active or not b.nav_active:
            self.physical_closing_since = None
            return False

        if separation <= self.physical_trigger_distance and closing_rate >= self.physical_closing_rate:
            if self.physical_closing_since is None:
                self.physical_closing_since = now
                return False
            if now - self.physical_closing_since < self.physical_closing_confirm:
                return False

            # First try the ORIGINAL path priority. If path detection was merely one
            # timer tick late, create the normal ConflictSession and keep all original
            # priority/release behavior.
            if self._make_conflict_session_from_paths(now, a, b):
                session = self.session
                if session is not None:
                    self._set_pause(session.winner, False)
                    self._set_pause(session.loser, True)
                    session.loser_paused = True
                    self._publish_status(
                        "YIELDING_PHYSICAL_BACKUP",
                        f"actual distance {separation:.2f}m confirmed -> {session.winner.upper()} keeps moving / "
                        f"{session.loser.upper()} pauses",
                        separation_m=round(separation, 2),
                        winner=session.winner,
                        loser=session.loser,
                    )
                self.physical_closing_since = None
                return False

            # Path overlap was genuinely missed. Prefer the fresh loaded-route owner;
            # otherwise keep the existing deterministic AMR1 tie convention.
            reservation_owner = self.reservation_owner
            if (
                reservation_owner is not None
                and now - reservation_owner.last_seen <= self.reservation_stale_sec
            ):
                winner = reservation_owner.robot
                loser = "amr2" if winner == "amr1" else "amr1"
            else:
                winner, loser = "amr1", "amr2"
            self.physical_session = PhysicalYieldSession(winner=winner, loser=loser, created=now)
            self._set_pause(winner, False)
            self._set_pause(loser, True)
            self.physical_closing_since = None
            self.get_logger().warning(
                f"실제거리 백업회피: AMR1-AMR2={separation:.2f}m closing={closing_rate:.2f}m/s "
                f"-> {winner.upper()} 계속 진행 / {loser.upper()} 일시정지"
            )
            self._publish_status(
                "PHYSICAL_YIELD",
                "path overlap missed; actual world distance backup selected one fixed priority AMR",
                floor=a.floor,
                separation_m=round(separation, 2),
                closing_rate_mps=round(closing_rate, 2),
                winner=winner,
                loser=loser,
            )
            return True

        self.physical_closing_since = None
        return False

    def _tick(self) -> None:
        now = time.monotonic()
        a = self.robots["amr1"]
        b = self.robots["amr2"]

        # Direct tray ingress is intentionally outside Nav2.  It must not inherit
        # a stale PRE_DOCK path, reservation pause bit, or special-motion pause.
        # The physical docking controller owns safety during this short phase.
        tray_docking = [
            robot
            for robot in ("amr1", "amr2")
            if self.tray_docking_active.get(robot, False)
        ]
        if tray_docking:
            self.session = None
            self.physical_session = None
            self._reset_physical_measurement()
            self._set_pause("amr1", False)
            self._set_pause("amr2", False)
            self._set_special_pause("amr1", False)
            self._set_special_pause("amr2", False)
            self._publish_status(
                "TRAY_DOCK_BYPASS",
                f"{','.join(robot.upper() for robot in tray_docking)} direct tray ingress; peer Nav2 remains free",
                docking=tray_docking,
            )
            return

        # A mission-level loaded-route reservation is stronger than reactive path
        # overlap and special-motion arbitration.  It is granted before the owner
        # enters the narrow corridor/elevator chain, so the other AMR waits before
        # both robots can meet inside a bottleneck.
        if self._handle_route_reservation(now):
            return

        # Do not blindly clear the reservation's old pause bit.  Release the former
        # waiter only after fresh world poses prove that the two physical bodies are
        # on different floors or outside the configured safety radius, and only when
        # no independent special/path/physical hold is active.
        if self.reservation_release_pending is not None:
            released_robot, released_phase = self.reservation_release_pending
            special_now = any(self._special_now(robot, now) for robot in ("amr1", "amr2"))
            poses_fresh = all(
                self.robots[robot].pose is not None
                and now - self.robots[robot].pose_stamp <= self.physical_pose_stale
                for robot in ("amr1", "amr2")
            )
            physically_clear = False
            separation = None
            if poses_fresh:
                if a.floor != "unknown" and b.floor != "unknown" and a.floor != b.floor:
                    physically_clear = True
                else:
                    separation = math.hypot(
                        b.pose[0] - a.pose[0],  # type: ignore[index]
                        b.pose[1] - a.pose[1],  # type: ignore[index]
                    )
                    physically_clear = separation >= self.physical_release_distance
            if (
                physically_clear
                and not special_now
                and not self.passing_request_id
                and self.session is None
                and self.physical_session is None
            ):
                self._set_pause("amr1", False)
                self._set_pause("amr2", False)
                self._set_special_pause("amr1", False)
                self._set_special_pause("amr2", False)
                self.reservation_release_pending = None
                self._publish_reservation_status(
                    "CLEAR",
                    "explicit release plus physical clearance confirmed",
                    released_robot=released_robot,
                    phase=released_phase,
                    separation_m=None if separation is None else round(separation, 2),
                )
            else:
                self._publish_reservation_status(
                    "RELEASE_CLEARANCE_HOLD",
                    "release received but physical clearance/safety state is not yet confirmed",
                    released_robot=released_robot,
                    phase=released_phase,
                    poses_fresh=poses_fresh,
                    separation_m=None if separation is None else round(separation, 2),
                )
                # Keep the next reservation blocked, but continue into the ordinary
                # path/special/physical checks so unrelated safe motion is not frozen.

        # Spatial priority for non-Nav special motion. The other AMR pauses only
        # on the same floor and inside the configured safety distance. Unknown or
        # stale localization deliberately falls back to the legacy global pause.
        special = [r for r in ("amr1", "amr2") if self._special_now(r, now)]
        if special:
            if self.special_owner not in special:
                self.special_owner = min(special, key=lambda r: self.special_since[r])
                self.special_spatial_hold = False
            winner = self.special_owner
            loser = "amr2" if winner == "amr1" else "amr1"
            conflict, reason, separation = self._special_spatial_conflict(now, winner, loser)
            self.session = None
            if self.physical_session is not None:
                self.physical_session = None
            self._reset_physical_measurement()
            if conflict:
                self.special_spatial_hold = True
                self._set_pause(winner, False)
                self._set_pause(loser, True)
                # Nav pause alone cannot stop direct OCR/ArUco/forced/elevator motion.
                self._set_special_pause(winner, False)
                self._set_special_pause(loser, True)
                self._publish_status(
                    "SPECIAL_SPATIAL_HOLD",
                    f"{winner.upper()} special motion is spatially relevant; {loser.upper()} waits",
                    winner=winner,
                    loser=loser,
                    reason=reason,
                    floor=self.robots[winner].floor,
                    separation_m=None if separation is None else round(separation, 2),
                )
            else:
                was_held = self.special_spatial_hold
                self.special_spatial_hold = False
                self._set_pause("amr1", False)
                self._set_pause("amr2", False)
                self._set_special_pause("amr1", False)
                self._set_special_pause("amr2", False)
                self._publish_status(
                    "SPECIAL_SPATIAL_CLEAR",
                    f"{winner.upper()} special motion is spatially separate; both AMRs may continue",
                    winner=winner,
                    other=loser,
                    reason=reason,
                    floor=self.robots[winner].floor,
                    separation_m=None if separation is None else round(separation, 2),
                    resumed=was_held,
                )
            return
        if self.special_owner is not None:
            previous = self.special_owner
            self.special_owner = None
            self.special_spatial_hold = False
            self.session = None
            self._set_pause("amr1", False)
            self._set_pause("amr2", False)
            self._set_special_pause("amr1", False)
            self._set_special_pause("amr2", False)
            self._publish_status("FREE", f"{previous.upper()} special motion finished; paused Nav/special motion resumed")
            return

        if self._handle_head_on_passing(now):
            return

        # Never compare coordinates from different floor maps.
        if a.floor == "unknown" or b.floor == "unknown" or a.floor != b.floor:
            if self.physical_session is not None:
                self._clear_physical_session(f"DIFFERENT_FLOORS:{a.floor}/{b.floor}")
            self._reset_physical_measurement()
            if self.session is not None or self.last_pause["amr1"] or self.last_pause["amr2"]:
                self._clear_session(f"DIFFERENT_FLOORS:{a.floor}/{b.floor}")
            return

        # Actual world-position backup. It may pause ONLY one AMR and always has an
        # automatic release path, so it cannot intentionally end the scenario in a
        # both-stopped deadlock.
        if self._handle_physical_guard(now, a, b):
            return

        if not a.nav_active or not b.nav_active:
            if self.session is not None:
                # If the winner completed its route, this will release below.
                if self._winner_cleared(self.session):
                    if self.session.release_started is None:
                        self.session.release_started = now
                    elif now - self.session.release_started >= self.release_delay:
                        self._clear_session("WINNER_FINISHED")
                return
            return

        if self.session is None:
            conflict = self._find_conflict(a, b)
            if conflict is None:
                self._publish_status("FREE", f"same floor {a.floor}, future paths do not overlap")
                return
            a_start, a_end, b_start, b_end = conflict
            head_on, separation, heading_dot = self._head_on_geometry(a, b)
            if head_on:
                if now < self.passing_cooldown_until:
                    return
                if separation <= self.head_on_trigger_distance:
                    self._start_head_on_passing(now, separation, heading_dot)
                    return
                # It is the same corridor but they are still far apart. Keep normal
                # Nav2 running until the agreed pass trigger distance is reached.
                self._publish_status(
                    "HEAD_ON_APPROACHING",
                    "opposite-direction same-path encounter detected; waiting for pass trigger distance",
                    separation_m=round(separation, 2),
                    trigger_m=round(self.head_on_trigger_distance, 2),
                    heading_dot=round(heading_dot, 3),
                )
                return
            winner, loser, da, db = self._select_priority(a, b, a_start, a_end, b_start, b_end)
            self.session = ConflictSession(
                winner=winner,
                loser=loser,
                winner_start=a_start if winner == "amr1" else b_start,
                winner_end=a_end if winner == "amr1" else b_end,
                loser_start=b_start if loser == "amr2" else a_start,
                loser_end=b_end if loser == "amr2" else a_end,
                created=now,
            )
            self._publish_status(
                "CONFLICT_DETECTED",
                f"{winner.upper()} priority / {loser.upper()} yields before shared path",
                floor=a.floor,
                amr1_distance_to_conflict_m=round(da, 2),
                amr2_distance_to_conflict_m=round(db, 2),
                winner=winner,
                loser=loser,
            )
            self.get_logger().warning(
                f"충돌 경로 감지({a.floor}): winner={winner.upper()}, loser={loser.upper()}, "
                f"AMR1 entry≈{da:.2f}m, AMR2 entry≈{db:.2f}m"
            )

        session = self.session
        if session is None:
            return

        loser_distance = self._loser_distance_to_entry(session)
        if not session.loser_paused and loser_distance <= self.hold_trigger_distance:
            self._set_pause(session.loser, True)
            self._set_pause(session.winner, False)
            session.loser_paused = True
            self._publish_status(
                "YIELDING",
                f"{session.loser.upper()} paused before conflict; {session.winner.upper()} passes",
                floor=a.floor,
                winner=session.winner,
                loser=session.loser,
                loser_distance_to_conflict_m=round(loser_distance, 2),
            )

        if self._winner_cleared(session):
            if session.release_started is None:
                session.release_started = now
                self._publish_status(
                    "CLEARANCE_DELAY",
                    f"{session.winner.upper()} cleared; wait {self.release_delay:.1f}s before release",
                    winner=session.winner,
                    loser=session.loser,
                )
            elif now - session.release_started >= self.release_delay:
                self._set_pause(session.loser, False)
                self._set_pause(session.winner, False)
                self.get_logger().info(
                    f"충돌 구간 해제: {session.winner.upper()} 통과 완료 -> {session.loser.upper()} 재출발"
                )
                self.session = None
                self._publish_status("FREE", "conflict cleared; yielded AMR resumed")
        else:
            session.release_started = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathConflictManager()
    try:
        rclpy.spin(node)
    finally:
        try:
            node._set_pause("amr1", False)
            node._set_pause("amr2", False)
            node._set_special_pause("amr1", False)
            node._set_special_pause("amr2", False)
            if node.passing_request_id:
                node._publish_maneuver_command("amr1", "CANCEL")
                node._publish_maneuver_command("amr2", "CANCEL")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
