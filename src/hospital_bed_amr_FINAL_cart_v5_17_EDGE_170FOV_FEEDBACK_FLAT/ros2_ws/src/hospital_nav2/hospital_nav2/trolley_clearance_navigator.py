#!/usr/bin/env python3
"""V5.9 clearance-balanced trolley navigation.

Pipeline
--------
/trolley/center_goal
  -> Nav2 ComputePathToPose (Smac State Lattice, raw SE(2) path)
  -> project-specific DP clearance optimizer (preserves x/y/yaw structure)
  -> /trolley/clearance_plan
  -> Nav2 FollowPath (Rotation Shim -> MPPI)

Design goals
------------
* Preserve State Lattice yaw, including in-place rotation samples.
* Explicitly balance LEFT/RIGHT clearance from the full trolley footprint.
* Treat inflation as a soft cost, not a collision.
* Use the full 360-degree trolley scan as an additional dynamic-obstacle source.
* Optimize the whole lateral-shift sequence with dynamic programming.
* Validate pose footprints and swept footprint between path samples.
* Replan immediately when a newly sensed full-scan obstacle intersects the remaining path.
* In open space preserve the shortest raw route; in narrow corridors strongly center the trolley.
"""
from __future__ import annotations

import copy
import math
import time
from typing import Optional

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath, Spin
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_yaw(q) -> float:
    return math.atan2(
        2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)),
        1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2),
    )


def yaw_to_quat(yaw: float):
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


class ClearanceNavigator(Node):
    def __init__(self) -> None:
        super().__init__('trolley_clearance_navigator')

        defaults = {
            'goal_topic': '/trolley/center_goal',
            'costmap_topic': '/global_costmap/costmap',
            'static_map_topic': '/map',
            'full_scan_topic': '/trolley/scan',
            'raw_path_topic': '/trolley/raw_plan',
            'optimized_path_topic': '/trolley/clearance_plan',
            'compute_path_action': '/compute_path_to_pose',
            'follow_path_action': '/follow_path',
            'spin_action': '/spin',
            'planner_id': 'GridBased',
            'global_frame': 'map',
            'base_frame': 'trolley_base',
            'static_occupied_threshold': 65,
            'scan_self_filter_margin_m': 0.10,
            # A LiDAR return within this radius of a known static occupied cell is
            # treated as the same mapped wall, not a new dynamic obstacle.
            'static_match_radius_m': 0.30,
            # Explicit pre-alignment before FollowPath. This is implemented with
            # Nav2 Humble's /spin behavior action, not RotationShim.
            'initial_align_threshold_deg': 15.0,
            'initial_heading_sample_distance_m': 0.55,
            'initial_spin_timeout_sec': 12.0,
            # Trolley Nav2 footprint = 2.36 m x 1.90 m.
            'trolley_half_length_m': 1.18,
            'trolley_half_width_m': 0.95,
            # Clearance sensing from the footprint side edges.
            'side_scan_m': 2.30,
            'side_step_m': 0.05,
            'side_longitudinal_samples': 9,
            # Candidate offsets around each raw SE(2) sample.
            'max_lateral_shift_m': 1.20,
            'candidate_step_m': 0.10,
            'optimize_spacing_m': 0.20,
            'optimize_yaw_spacing_deg': 7.5,
            'max_shift_step_m': 0.15,
            # 100 == lethal/unknown only in the published OccupancyGrid convention.
            'collision_cost_threshold': 100,
            # Used only for measuring an obstacle/inflation edge for dL/dR.
            'obstacle_cost_threshold': 90,
            'footprint_sample_step_m': 0.06,
            'swept_linear_step_m': 0.06,
            'swept_angular_step_deg': 4.0,
            # Project-specific objective weights.
            'w_balance': 16.0,
            'w_min_clearance': 8.0,
            'w_inflation_sum': 3.0,
            'w_inflation_balance': 10.0,
            'w_deviation': 1.00,
            'w_shift_smooth': 3.0,
            'w_shift_accel': 4.0,
            'w_extra_path_length': 5.0,
            'corner_soften_yaw_deg': 12.0,
            'corner_disable_yaw_deg': 28.0,
            'corner_min_center_factor': 0.50,
            # Adaptive policy: shortest-path bias in open space, strong centering in corridors.
            'corridor_narrow_clearance_m': 0.95,
            'corridor_open_clearance_m': 1.55,
            'corridor_center_gain': 2.40,
            # Start is gently anchored; only the final goal pose is hard locked.
            'start_lock_m': 0.15,
            # Final approach policy: keep corridor clearance until a safe yaw staging pose.
            'goal_rotation_clearance_margin_m': 0.12,
            'goal_staging_min_m': 0.55,
            'goal_staging_max_m': 1.60,
            'goal_staging_step_m': 0.15,
            'goal_rotation_step_deg': 4.0,
            # Conditional replanning, not continuous replanning.
            'enable_conditional_replan': True,
            'replan_check_period_sec': 0.40,
            'replan_cooldown_sec': 1.5,
            'replan_start_grace_sec': 2.0,
            'dynamic_obstacle_confirmations': 3,
            'replan_check_stride': 2,
            'replan_path_deviation_m': 0.25,
            'replan_deviation_confirmations': 2,
            # Wall-clearance feedback: 0.5 s x 3 confirmations ~= 1.5 s response.
            'wall_replan_check_period_sec': 0.50,
            'wall_replan_confirmations': 3,
            'wall_replan_min_clearance_m': 0.30,
            'wall_replan_imbalance_m': 0.35,
            # Full-body / forward-field feedback. Use the complete footprint perimeter
            # for present clearance and the forward 170 deg LiDAR field for look-ahead.
            'forward_fov_deg': 170.0,
            'forward_sector_percentile': 10.0,
            'front_sector_clearance_m': 0.55,
            'front_diagonal_clearance_m': 0.40,
            'side_sector_clearance_m': 0.30,
            'edge_replan_min_clearance_m': 0.30,
            'debug_every_n': 8,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        gp = lambda n: self.get_parameter(n).value

        self.goal_topic = str(gp('goal_topic'))
        self.costmap_topic = str(gp('costmap_topic'))
        self.static_map_topic = str(gp('static_map_topic'))
        self.full_scan_topic = str(gp('full_scan_topic'))
        self.raw_path_topic = str(gp('raw_path_topic'))
        self.opt_path_topic = str(gp('optimized_path_topic'))
        self.planner_id = str(gp('planner_id'))
        self.global_frame = str(gp('global_frame'))
        self.base_frame = str(gp('base_frame'))
        self.static_occ_thr = int(gp('static_occupied_threshold'))
        self.scan_self_margin = max(0.0, float(gp('scan_self_filter_margin_m')))
        self.static_match_radius = max(0.0, float(gp('static_match_radius_m')))
        self.initial_align_threshold = math.radians(max(0.0, float(gp('initial_align_threshold_deg'))))
        self.initial_heading_sample_distance = max(0.10, float(gp('initial_heading_sample_distance_m')))
        self.initial_spin_timeout = max(2.0, float(gp('initial_spin_timeout_sec')))
        self.half_l = float(gp('trolley_half_length_m'))
        self.half_w = float(gp('trolley_half_width_m'))
        self.side_scan = float(gp('side_scan_m'))
        self.side_step = max(0.025, float(gp('side_step_m')))
        self.side_long_n = max(5, int(gp('side_longitudinal_samples')))
        self.max_shift = max(0.0, float(gp('max_lateral_shift_m')))
        self.candidate_step = max(0.05, float(gp('candidate_step_m')))
        self.spacing = max(0.05, float(gp('optimize_spacing_m')))
        self.yaw_spacing = math.radians(max(1.0, float(gp('optimize_yaw_spacing_deg'))))
        self.max_shift_step = max(self.candidate_step, float(gp('max_shift_step_m')))
        self.collision_thr = int(gp('collision_cost_threshold'))
        self.obstacle_thr = int(gp('obstacle_cost_threshold'))
        self.footprint_step = max(0.04, float(gp('footprint_sample_step_m')))
        self.swept_linear_step = max(0.04, float(gp('swept_linear_step_m')))
        self.swept_angular_step = math.radians(max(1.0, float(gp('swept_angular_step_deg'))))
        self.w_balance = float(gp('w_balance'))
        self.w_clear = float(gp('w_min_clearance'))
        self.w_infl_sum = float(gp('w_inflation_sum'))
        self.w_infl_bal = float(gp('w_inflation_balance'))
        self.w_dev = float(gp('w_deviation'))
        self.w_smooth = float(gp('w_shift_smooth'))
        self.w_shift_accel = max(0.0, float(gp('w_shift_accel')))
        self.w_path_extra = max(0.0, float(gp('w_extra_path_length')))
        self.corner_soften_yaw = math.radians(max(0.0, float(gp('corner_soften_yaw_deg'))))
        self.corner_disable_yaw = math.radians(max(float(gp('corner_soften_yaw_deg')) + 1.0, float(gp('corner_disable_yaw_deg'))))
        self.corner_min_center_factor = max(0.0, min(1.0, float(gp('corner_min_center_factor'))))
        self.corridor_narrow = max(0.10, float(gp('corridor_narrow_clearance_m')))
        self.corridor_open = max(self.corridor_narrow + 0.10, float(gp('corridor_open_clearance_m')))
        self.corridor_gain = max(1.0, float(gp('corridor_center_gain')))
        self.start_lock = max(0.0, float(gp('start_lock_m')))
        self.goal_rot_margin = max(0.0, float(gp('goal_rotation_clearance_margin_m')))
        self.goal_stage_min = max(0.20, float(gp('goal_staging_min_m')))
        self.goal_stage_max = max(self.goal_stage_min, float(gp('goal_staging_max_m')))
        self.goal_stage_step = max(0.05, float(gp('goal_staging_step_m')))
        self.goal_rot_step = math.radians(max(1.0, float(gp('goal_rotation_step_deg'))))
        self.enable_replan = bool(gp('enable_conditional_replan'))
        self.replan_period = max(0.25, float(gp('replan_check_period_sec')))
        self.replan_cooldown = max(0.5, float(gp('replan_cooldown_sec')))
        self.replan_start_grace = max(0.0, float(gp('replan_start_grace_sec')))
        self.dynamic_obstacle_confirmations = max(1, int(gp('dynamic_obstacle_confirmations')))
        self.replan_stride = max(1, int(gp('replan_check_stride')))
        self.replan_path_deviation = max(0.05, float(gp('replan_path_deviation_m')))
        self.replan_deviation_confirmations = max(1, int(gp('replan_deviation_confirmations')))
        self.wall_replan_period = max(0.10, float(gp('wall_replan_check_period_sec')))
        self.wall_replan_confirmations = max(1, int(gp('wall_replan_confirmations')))
        self.wall_replan_min_clearance = max(0.05, float(gp('wall_replan_min_clearance_m')))
        self.wall_replan_imbalance = max(0.05, float(gp('wall_replan_imbalance_m')))
        self.forward_fov = math.radians(max(30.0, min(179.0, float(gp('forward_fov_deg')))))
        self.forward_sector_percentile = max(1.0, min(40.0, float(gp('forward_sector_percentile'))))
        self.front_sector_clearance = max(0.10, float(gp('front_sector_clearance_m')))
        self.front_diag_clearance = max(0.10, float(gp('front_diagonal_clearance_m')))
        self.side_sector_clearance = max(0.10, float(gp('side_sector_clearance_m')))
        self.edge_replan_min_clearance = max(0.05, float(gp('edge_replan_min_clearance_m')))
        self.debug_every_n = max(1, int(gp('debug_every_n')))

        # QoS: global costmap is latched/transient; LaserScan is sensor QoS.
        cost_qos = QoSProfile(depth=1)
        cost_qos.reliability = ReliabilityPolicy.RELIABLE
        cost_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        goal_qos = QoSProfile(depth=1)
        goal_qos.reliability = ReliabilityPolicy.RELIABLE
        goal_qos.durability = DurabilityPolicy.VOLATILE
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(OccupancyGrid, self.costmap_topic, self._on_costmap, cost_qos)
        self.create_subscription(OccupancyGrid, self.static_map_topic, self._on_static_map, cost_qos)
        self.create_subscription(LaserScan, self.full_scan_topic, self._on_scan, scan_qos)
        self.create_subscription(PoseStamped, self.goal_topic, self._on_goal, goal_qos)
        self.raw_pub = self.create_publisher(Path, self.raw_path_topic, 10)
        self.opt_pub = self.create_publisher(Path, self.opt_path_topic, 10)

        self.compute_client = ActionClient(self, ComputePathToPose, str(gp('compute_path_action')))
        self.follow_client = ActionClient(self, FollowPath, str(gp('follow_path_action')))
        self.spin_client = ActionClient(self, Spin, str(gp('spin_action')))

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.map_msg: Optional[OccupancyGrid] = None
        self.grid: Optional[np.ndarray] = None
        self.static_map_msg: Optional[OccupancyGrid] = None
        self.static_grid: Optional[np.ndarray] = None
        self.static_near_occ_grid: Optional[np.ndarray] = None
        self.scan_points_map = np.empty((0, 2), dtype=np.float64)
        # All valid LiDAR returns in trolley_base coordinates (static + dynamic).
        # This is used only for forward-field clearance feedback, not dynamic classification.
        self.scan_points_base_all = np.empty((0, 2), dtype=np.float64)
        self.pending_goal: Optional[PoseStamped] = None
        self.active_goal: Optional[PoseStamped] = None
        self.follow_handle = None
        self.spin_handle = None
        self.plan_request_active = False
        self.current_path: Optional[Path] = None
        self.current_baseline_min_clear = 0.0
        self.last_replan_wall_time = -1e9
        self.follow_start_wall_time = -1e9
        self._replan_reason = ''
        self.deviation_violation_count = 0
        self.dynamic_obstacle_violation_count = 0
        self.wall_clearance_violation_count = 0
        self.last_wall_check_wall_time = -1e9

        # Precompute dense footprint samples once. This includes the complete body area,
        # corners included, at a spacing close to the 5 cm costmap resolution.
        xs = np.arange(-self.half_l, self.half_l + 0.5 * self.footprint_step, self.footprint_step)
        ys = np.arange(-self.half_w, self.half_w + 0.5 * self.footprint_step, self.footprint_step)
        gx, gy = np.meshgrid(xs, ys, indexing='ij')
        self.fp_lx = gx.ravel()
        self.fp_ly = gy.ravel()
        self.long_samples = np.linspace(-self.half_l, self.half_l, self.side_long_n)
        self.side_distances = np.arange(
            self.side_step, self.side_scan + 0.5 * self.side_step, self.side_step
        )
        self.candidate_values = np.arange(
            -self.max_shift, self.max_shift + 0.5 * self.candidate_step, self.candidate_step
        )

        if self.enable_replan:
            self.create_timer(self.replan_period, self._conditional_replan_check)

        self.get_logger().info(
            'V5.17 CLEARANCE DP active | explicit /spin prealign | 2s start grace | 3x dynamic confirmation | '
            'wall-clearance feedback 0.5s x3 | full footprint-edge + forward-170deg sectors | second-difference shift smoothing | '
            f'footprint-step={self.footprint_step:.2f}m | swept-check | conditional-replan | '
            f'shift=+/-{self.max_shift:.2f}m'
        )

    # ----------------------------- inputs -----------------------------
    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.grid = np.asarray(msg.data, dtype=np.int16).reshape(
            int(msg.info.height), int(msg.info.width)
        )

    def _on_static_map(self, msg: OccupancyGrid) -> None:
        self.static_map_msg = msg
        self.static_grid = np.asarray(msg.data, dtype=np.int16).reshape(
            int(msg.info.height), int(msg.info.width)
        )
        # Dilate known static occupied cells only for LiDAR classification. This is NOT
        # used as a navigation collision layer; it merely tolerates map/LiDAR boundary
        # quantization so mapped walls are not mislabeled as NEW obstacles.
        occ = self.static_grid >= self.static_occ_thr
        radius_cells = int(math.ceil(self.static_match_radius / max(float(msg.info.resolution), 1e-6)))
        if radius_cells <= 0:
            self.static_near_occ_grid = occ.copy()
        else:
            h, w = occ.shape
            pad = radius_cells
            padded = np.pad(occ, pad, mode='constant', constant_values=False)
            dil = np.zeros_like(occ, dtype=bool)
            rr = radius_cells * radius_cells
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    if dx * dx + dy * dy > rr:
                        continue
                    y0 = pad + dy
                    x0 = pad + dx
                    dil |= padded[y0:y0+h, x0:x0+w]
            self.static_near_occ_grid = dil
        occ_count = int(np.count_nonzero(occ))
        self.get_logger().info(
            f'STATIC MAP READY | {int(msg.info.width)}x{int(msg.info.height)} '
            f'res={float(msg.info.resolution):.3f}m | occupied={occ_count} | '
            f'match_radius={self.static_match_radius:.2f}m'
        )

    def _static_values_at_arrays(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        if self.static_grid is None or self.static_map_msg is None:
            return np.full(np.shape(xs), -1, dtype=np.int16)
        info = self.static_map_msg.info
        res = float(info.resolution)
        cols = np.floor((xs - float(info.origin.position.x)) / res).astype(np.int64)
        rows = np.floor((ys - float(info.origin.position.y)) / res).astype(np.int64)
        valid = (rows >= 0) & (cols >= 0) & (rows < int(info.height)) & (cols < int(info.width))
        out = np.full(rows.shape, -1, dtype=np.int16)
        if np.any(valid):
            out[valid] = self.static_grid[rows[valid], cols[valid]].astype(np.int16)
        return out

    def _static_near_occupied_at_arrays(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        if self.static_near_occ_grid is None or self.static_map_msg is None:
            return np.zeros(np.shape(xs), dtype=bool)
        info = self.static_map_msg.info
        res = float(info.resolution)
        cols = np.floor((xs - float(info.origin.position.x)) / res).astype(np.int64)
        rows = np.floor((ys - float(info.origin.position.y)) / res).astype(np.int64)
        valid = (rows >= 0) & (cols >= 0) & (rows < int(info.height)) & (cols < int(info.width))
        out = np.ones(rows.shape, dtype=bool)  # outside map is never treated as a new obstacle
        if np.any(valid):
            out[valid] = self.static_near_occ_grid[rows[valid], cols[valid]]
        return out

    def _on_scan(self, msg: LaserScan) -> None:
        """Cache the full 360-degree scan as obstacle points in map coordinates."""
        if not msg.ranges:
            return
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        angles = float(msg.angle_min) + np.arange(len(ranges), dtype=np.float64) * float(msg.angle_increment)
        valid = np.isfinite(ranges)
        valid &= ranges >= max(float(msg.range_min), 0.05)
        valid &= ranges <= float(msg.range_max)
        if not np.any(valid):
            self.scan_points_map = np.empty((0, 2), dtype=np.float64)
            self.scan_points_base_all = np.empty((0, 2), dtype=np.float64)
            return
        r = ranges[valid]
        a = angles[valid]
        sx = r * np.cos(a)
        sy = r * np.sin(a)
        # Reject returns from the trolley itself / very-near mounted geometry. These are
        # never dynamic obstacles and previously could make future raw poses look unsafe.
        self_hit = (np.abs(sx) <= self.half_l + self.scan_self_margin) & (np.abs(sy) <= self.half_w + self.scan_self_margin)
        sx = sx[~self_hit]
        sy = sy[~self_hit]
        if sx.size == 0:
            self.scan_points_map = np.empty((0, 2), dtype=np.float64)
            self.scan_points_base_all = np.empty((0, 2), dtype=np.float64)
            return
        source_frame = msg.header.frame_id or 'trolley_lidar'
        try:
            stamp_is_zero = int(msg.header.stamp.sec) == 0 and int(msg.header.stamp.nanosec) == 0
            tf_time = Time() if stamp_is_zero else Time.from_msg(msg.header.stamp)
            # Keep a complete, unclassified forward field in trolley_base coordinates.
            # This sees known walls and new obstacles alike, which is exactly what the
            # near-body look-ahead feedback needs.
            tf_base = self.tf_buffer.lookup_transform(
                self.base_frame, source_frame, tf_time, timeout=Duration(seconds=0.12)
            )
            tb = tf_base.transform.translation
            yb = quat_to_yaw(tf_base.transform.rotation)
            cb, sb = math.cos(yb), math.sin(yb)
            bx = float(tb.x) + cb * sx - sb * sy
            by = float(tb.y) + sb * sx + cb * sy
            self.scan_points_base_all = np.column_stack((bx, by))

            tf = self.tf_buffer.lookup_transform(
                self.global_frame, source_frame, tf_time, timeout=Duration(seconds=0.12)
            )
        except TransformException:
            return
        t = tf.transform.translation
        yaw = quat_to_yaw(tf.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        wx = float(t.x) + c * sx - s * sy
        wy = float(t.y) + s * sx + c * sy
        # Static walls are already represented over the full route by /map and the global
        # costmap. Keep only scan returns that fall in static-map free/unknown-free space as
        # NEW dynamic obstacles. This prevents known corridor walls from retriggering replans.
        if self.static_grid is not None and self.static_map_msg is not None:
            sv = self._static_values_at_arrays(wx, wy)
            near_static = self._static_near_occupied_at_arrays(wx, wy)
            # NEW obstacle = return in known-free space AND not within the map/LiDAR
            # matching radius of any mapped occupied wall. This removes the false
            # dynamic-wall detections that previously caused HOLD/replan loops.
            dynamic = (sv >= 0) & (sv < self.static_occ_thr) & (~near_static)
            self.scan_points_map = np.column_stack((wx[dynamic], wy[dynamic])) if np.any(dynamic) else np.empty((0, 2), dtype=np.float64)
        else:
            # Until the static map is available, do not classify every wall as dynamic.
            self.scan_points_map = np.empty((0, 2), dtype=np.float64)

    def _on_goal(self, msg: PoseStamped) -> None:
        if self.grid is None or self.map_msg is None:
            self.get_logger().error(f'No {self.costmap_topic} yet. Goal ignored.')
            return
        self.active_goal = msg
        self.pending_goal = msg
        if self.spin_handle is not None and self.spin_handle.status in (
            GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING
        ):
            self.get_logger().info('New goal: cancel current pre-alignment Spin first.')
            f = self.spin_handle.cancel_goal_async()
            f.add_done_callback(lambda _: self._request_plan())
        elif self.follow_handle is not None and self.follow_handle.status in (
            GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING
        ):
            self.get_logger().info('New goal: cancel current FollowPath first.')
            f = self.follow_handle.cancel_goal_async()
            f.add_done_callback(lambda _: self._request_plan())
        elif not self.plan_request_active:
            self._request_plan()

    # ----------------------------- planning -----------------------------
    def _request_plan(self) -> None:
        if self.plan_request_active:
            return
        if self.pending_goal is None:
            return
        if not self.compute_client.wait_for_server(timeout_sec=1.5):
            self.get_logger().error('ComputePathToPose action server not ready.')
            return
        goal_pose = self.pending_goal
        self.pending_goal = None
        req = ComputePathToPose.Goal()
        req.goal = goal_pose
        req.planner_id = self.planner_id
        req.use_start = False
        self.plan_request_active = True
        self.get_logger().info('Requesting raw State-Lattice path...')
        fut = self.compute_client.send_goal_async(req)
        fut.add_done_callback(self._on_compute_goal)

    def _on_compute_goal(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self.plan_request_active = False
            self.get_logger().error(f'ComputePathToPose goal exception: {exc}')
            return
        if handle is None or not handle.accepted:
            self.plan_request_active = False
            self.get_logger().error('ComputePathToPose goal rejected.')
            return
        handle.get_result_async().add_done_callback(self._on_compute_result)

    def _on_compute_result(self, future) -> None:
        self.plan_request_active = False
        try:
            wrapped = future.result()
            raw = wrapped.result.path
        except Exception as exc:
            self.get_logger().error(f'ComputePathToPose result exception: {exc}')
            return
        if len(raw.poses) < 2:
            self.get_logger().error('Raw path is empty/too short.')
            return

        self.raw_pub.publish(raw)
        try:
            opt, baseline_min = self._optimize_path(raw)
            fallback_used = False
        except Exception as exc:
            # Never make the green path disappear. Publish the State-Lattice path as a
            # visible fallback. Follow it only when the CURRENT static-map + full-scan
            # safety check says it is still safe; otherwise request a fresh global plan.
            self.get_logger().error(f'Clearance optimizer failed: {exc}')
            opt = raw
            fallback_used = True
            _, baseline_min = self._path_safety(raw)
            self.get_logger().warn('CLEARANCE FALLBACK: publishing raw State-Lattice as green path.')

        # Keep corridor centering to the end, but move any large final yaw rotation
        # to a collision-checked staging pose when the goal itself is too close to a wall.
        opt = self._append_safe_final_approach(opt, raw)
        self.opt_pub.publish(opt)
        self.current_path = opt
        self.current_baseline_min_clear = baseline_min
        self.deviation_violation_count = 0
        self.last_replan_wall_time = time.monotonic()

        # IMPORTANT: never HOLD a valid State-Lattice fallback before motion solely from
        # a single full-scan classification.  The global planner already validated the
        # static geometry, while scan/map edge quantization can still create false
        # "new obstacle" points near L-corners.  Start FollowPath and let the conditional
        # replan logic react only after motion is active and the replan cooldown has passed.
        # This preserves dynamic-obstacle replanning without deadlocking long/L-corner goals.
        if fallback_used:
            if self._dynamic_scan_path_collision(opt):
                self.get_logger().warn(
                    'Fallback path currently overlaps full-scan NEW-obstacle candidates, '
                    'but startup HOLD is disabled; following raw State-Lattice and '
                    'conditional replan will verify after motion starts.'
                )
            else:
                self.get_logger().warn(
                    'Fallback raw State-Lattice is trusted for known static geometry and will be followed.'
                )
        self._prealign_then_follow(opt)

    # -------------------------- costmap helpers --------------------------
    def _costs_at_arrays(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        if self.grid is None or self.map_msg is None:
            return np.full(np.shape(xs), 100, dtype=np.int16)
        info = self.map_msg.info
        res = float(info.resolution)
        cols = np.floor((xs - float(info.origin.position.x)) / res).astype(np.int64)
        rows = np.floor((ys - float(info.origin.position.y)) / res).astype(np.int64)
        valid = (
            (rows >= 0) & (cols >= 0) &
            (rows < int(info.height)) & (cols < int(info.width))
        )
        out = np.full(rows.shape, 100, dtype=np.int16)
        if np.any(valid):
            vals = self.grid[rows[valid], cols[valid]].astype(np.int16)
            vals[vals < 0] = 100
            out[valid] = np.clip(vals, 0, 100)
        return out

    def _pose_collision(self, x: float, y: float, yaw: float) -> bool:
        """Full trolley body collision against lethal/unknown + full-scan points.

        Inflation values below collision_thr are *not* collision; they remain soft costs.
        """
        c, s = math.cos(yaw), math.sin(yaw)
        wx = x + c * self.fp_lx - s * self.fp_ly
        wy = y + s * self.fp_lx + c * self.fp_ly
        if np.any(self._costs_at_arrays(wx, wy) >= self.collision_thr):
            return True

        if self.scan_points_map.size:
            dx = self.scan_points_map[:, 0] - x
            dy = self.scan_points_map[:, 1] - y
            lx = c * dx + s * dy
            ly = -s * dx + c * dy
            # Small 2 cm guard prevents exact-boundary numerical grazing.
            if np.any((np.abs(lx) <= self.half_l + 0.02) & (np.abs(ly) <= self.half_w + 0.02)):
                return True
        return False

    def _swept_collision(self, p0, p1) -> bool:
        x0, y0, a0 = p0
        x1, y1, a1 = p1
        dist = math.hypot(x1 - x0, y1 - y0)
        da = norm_angle(a1 - a0)
        steps = max(
            1,
            int(math.ceil(dist / self.swept_linear_step)),
            int(math.ceil(abs(da) / self.swept_angular_step)),
        )
        for k in range(1, steps + 1):
            t = k / float(steps)
            x = x0 + (x1 - x0) * t
            y = y0 + (y1 - y0) * t
            a = norm_angle(a0 + da * t)
            if self._pose_collision(x, y, a):
                return True
        return False

    def _side_metrics(self, x: float, y: float, yaw: float):
        """Return (dL, dR, cL, cR) from the *full side edges* of the trolley."""
        c, s = math.cos(yaw), math.sin(yaw)
        nx, ny = -s, c

        # Costmap inflation sampled along the complete left/right edges, corners included.
        def costmap_side(sign: float):
            lx = np.repeat(self.long_samples, len(self.side_distances))
            d = np.tile(self.side_distances, len(self.long_samples))
            wx = x + c * lx + sign * nx * (self.half_w + d)
            wy = y + s * lx + sign * ny * (self.half_w + d)
            cv = self._costs_at_arrays(wx, wy).reshape(self.side_long_n, -1)
            # Distance is nearest high-cost ring from any point along the side edge.
            hit_cols = np.where(np.max(cv, axis=0) >= self.obstacle_thr)[0]
            hit = self.side_scan if len(hit_cols) == 0 else float(self.side_distances[int(hit_cols[0])])
            # Mean normalized inflation, slightly emphasizing nearer cells.
            weights = 1.0 / (1.0 + self.side_distances / max(self.side_scan, 1e-6))
            weighted = (cv.astype(np.float64) / 100.0) * weights[None, :]
            mean_cost = float(np.mean(weighted))
            return hit, mean_cost

        d_l, c_l = costmap_side(+1.0)
        d_r, c_r = costmap_side(-1.0)

        # Full 360 scan directly supplies dynamic left/right clearance around the body.
        if self.scan_points_map.size:
            dx = self.scan_points_map[:, 0] - x
            dy = self.scan_points_map[:, 1] - y
            local_x = c * dx + s * dy
            local_y = -s * dx + c * dy
            longitudinal = np.abs(local_x) <= self.half_l + 0.10
            left = longitudinal & (local_y > self.half_w)
            right = longitudinal & (local_y < -self.half_w)
            if np.any(left):
                d_l = min(d_l, float(np.min(local_y[left] - self.half_w)))
            if np.any(right):
                d_r = min(d_r, float(np.min(-local_y[right] - self.half_w)))

        return max(0.0, d_l), max(0.0, d_r), c_l, c_r

    # -------------------------- path helpers --------------------------
    @staticmethod
    def _raw_arrays(path: Path):
        pts = np.asarray([[p.pose.position.x, p.pose.position.y] for p in path.poses], dtype=np.float64)
        yaws = np.asarray([quat_to_yaw(p.pose.orientation) for p in path.poses], dtype=np.float64)
        return pts, yaws

    def _resample_indices(self, pts: np.ndarray, yaws: np.ndarray) -> list[int]:
        """Resample by translation OR yaw change so in-place lattice rotations survive."""
        if len(pts) <= 2:
            return list(range(len(pts)))
        out = [0]
        acc_dist = 0.0
        acc_yaw = 0.0
        for i in range(1, len(pts)):
            acc_dist += float(np.linalg.norm(pts[i] - pts[i - 1]))
            acc_yaw += abs(norm_angle(float(yaws[i] - yaws[i - 1])))
            if acc_dist >= self.spacing or acc_yaw >= self.yaw_spacing:
                out.append(i)
                acc_dist = 0.0
                acc_yaw = 0.0
        if out[-1] != len(pts) - 1:
            out.append(len(pts) - 1)
        return out

    def _start_scale(self, from_start: float) -> float:
        if self.start_lock <= 1e-6:
            return 1.0
        return max(0.0, min(1.0, from_start / self.start_lock))

    def _unary_cost(self, x: float, y: float, yaw: float, shift: float, straight_factor: float = 1.0):
        if self._pose_collision(x, y, yaw):
            return None
        d_l, d_r, c_l, c_r = self._side_metrics(x, y, yaw)
        min_clear = min(d_l, d_r)
        balance = abs(d_l - d_r) / max(self.side_scan, 1e-3)
        clear_pen = 1.0 / max(min_clear, 0.06)
        infl_sum = c_l + c_r
        infl_bal = abs(c_l - c_r)
        dev = abs(shift) / max(self.max_shift, 1e-3)

        # Adaptive corridor policy:
        # - open space: corridor_factor -> 0, so raw shortest path is preferred by deviation cost
        # - narrow corridor / wall nearby: corridor_factor -> 1, strongly balance L/R clearance
        corridor_factor = (self.corridor_open - min_clear) / max(
            self.corridor_open - self.corridor_narrow, 1e-6
        )
        corridor_factor = max(0.0, min(1.0, corridor_factor))
        # Keep clearance centering active through corners. The corner factor may soften
        # lateral pressure, but V5.13 never lets it fall below the configured floor.
        corridor_factor *= max(0.0, min(1.0, straight_factor))
        center_gain = 1.0 + (self.corridor_gain - 1.0) * corridor_factor
        narrow_center_pressure = 0.0
        if corridor_factor > 0.65:
            # In a real corridor, a raw pose that hugs one wall should not win merely
            # because shift=0 is short. This term is zero in open space.
            narrow_center_pressure = 6.0 * corridor_factor * balance
        J = (
            self.w_balance * center_gain * corridor_factor * balance
            + self.w_clear * corridor_factor * clear_pen
            + self.w_infl_sum * infl_sum
            + self.w_infl_bal * center_gain * corridor_factor * infl_bal
            + self.w_dev * (1.0 + 7.0 * (1.0 - corridor_factor)) * dev
            + narrow_center_pressure
        )
        return J, d_l, d_r, c_l, c_r

    def _optimize_path(self, raw: Path):
        raw_pts_all, raw_yaws_all = self._raw_arrays(raw)
        idx = self._resample_indices(raw_pts_all, raw_yaws_all)
        pts = raw_pts_all[idx]
        yaws = raw_yaws_all[idx]  # IMPORTANT: preserve State Lattice yaw.
        n = len(pts)
        if n < 2:
            return raw, 0.0

        seglen = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        from_start = np.concatenate([[0.0], np.cumsum(seglen)])

        # Build candidate states per sample. Final goal position is hard locked to raw.
        states = []
        raw_metrics = []
        for i in range(n):
            x0, y0 = float(pts[i, 0]), float(pts[i, 1])
            yaw = float(yaws[i])
            nx, ny = -math.sin(yaw), math.cos(yaw)
            raw_metrics.append(self._side_metrics(x0, y0, yaw))

            # Detect corners from the original State-Lattice yaw change. Centering is
            # strongest on straights but remains active at turns (minimum floor).
            local_dyaw = 0.0
            if i > 0:
                local_dyaw = max(local_dyaw, abs(norm_angle(float(yaws[i] - yaws[i - 1]))))
            if i + 1 < n:
                local_dyaw = max(local_dyaw, abs(norm_angle(float(yaws[i + 1] - yaws[i]))))
            if local_dyaw <= self.corner_soften_yaw:
                straight_factor = 1.0
            elif local_dyaw >= self.corner_disable_yaw:
                straight_factor = self.corner_min_center_factor
            else:
                t = (local_dyaw - self.corner_soften_yaw) / max(
                    self.corner_disable_yaw - self.corner_soften_yaw, 1e-6
                )
                straight_factor = 1.0 - t * (1.0 - self.corner_min_center_factor)

            # Never translate the center during a State-Lattice in-place rotation.
            # The raw lattice already chose a collision-feasible rotation pose; moving it
            # laterally while yaw changes would turn a pure rotation into a small circle.
            rotation_only = False
            if i > 0:
                raw_step = float(np.linalg.norm(pts[i] - pts[i - 1]))
                raw_dyaw = abs(norm_angle(float(yaws[i] - yaws[i - 1])))
                rotation_only = raw_step < 0.03 and raw_dyaw > math.radians(2.0)

            if i == n - 1 or rotation_only:
                allowed = np.asarray([0.0])
            else:
                scale = self._start_scale(float(from_start[i]))
                allowed = np.unique(np.round(self.candidate_values * scale, 6))

            row = []
            for shift in allowed:
                shift = float(shift)
                x = x0 + nx * shift
                y = y0 + ny * shift
                score = self._unary_cost(x, y, yaw, shift, straight_factor=straight_factor)
                if score is None:
                    continue
                row.append({
                    'shift': shift, 'x': x, 'y': y, 'yaw': yaw,
                    'unary': float(score[0]), 'metrics': score[1:],
                })
            if not row:
                # Keep the optimized path drawable whenever possible. State Lattice already
                # validated the raw pose against the global map, so use shift=0 as a local
                # fallback when our extra lateral candidates are over-constrained. A truly
                # unsafe raw pose (e.g. a new LiDAR obstacle occupying the body) is escalated
                # to the outer fallback/replan path instead of being followed blindly.
                if not self._pose_collision(x0, y0, yaw):
                    m = self._side_metrics(x0, y0, yaw)
                    row.append({
                        'shift': 0.0, 'x': x0, 'y': y0, 'yaw': yaw,
                        'unary': 1000.0, 'metrics': m, 'forced_raw': True,
                    })
                    self.get_logger().warn(
                        f'CLEARANCE FALLBACK sample {i}/{n-1}: use raw State-Lattice pose.'
                    )
                else:
                    raise RuntimeError(
                        f'No safe candidate at sample {i}/{n-1}; raw pose also intersects current scan/costmap.'
                    )
            states.append(row)

        # Dynamic programming across the full path, not greedy point-by-point selection.
        prev_cost = np.full(len(states[0]), np.inf, dtype=np.float64)
        prev_cost[:] = [st['unary'] for st in states[0]]
        backrefs = [np.full(len(states[0]), -1, dtype=np.int32)]
        # Last lateral-shift slope of the best path ending at each state. This lets the
        # first-order DP penalize abrupt changes in shift slope (approx. second difference)
        # without expanding the state space to predecessor pairs.
        prev_delta = np.zeros(len(states[0]), dtype=np.float64)

        for i in range(1, n):
            cur = states[i]
            prv = states[i - 1]
            cur_cost = np.full(len(cur), np.inf, dtype=np.float64)
            cur_back = np.full(len(cur), -1, dtype=np.int32)
            cur_delta = np.zeros(len(cur), dtype=np.float64)
            for j, st in enumerate(cur):
                for k, pst in enumerate(prv):
                    if not np.isfinite(prev_cost[k]):
                        continue
                    if abs(st['shift'] - pst['shift']) > self.max_shift_step + 1e-6:
                        continue
                    if self._swept_collision(
                        (pst['x'], pst['y'], pst['yaw']),
                        (st['x'], st['y'], st['yaw']),
                    ):
                        continue
                    shift_delta = st['shift'] - pst['shift']
                    shift_transition = self.w_smooth * abs(shift_delta) / max(self.candidate_step, 1e-3)
                    # Penalize sudden changes in lateral shift slope. This smooths the
                    # entry into a corner instead of allowing 0.0 -> 0.4 -> 0.6 m jumps.
                    shift_accel = self.w_shift_accel * abs(shift_delta - prev_delta[k]) / max(self.candidate_step, 1e-3)
                    raw_dist = float(np.linalg.norm(pts[i] - pts[i - 1]))
                    candidate_dist = math.hypot(st['x'] - pst['x'], st['y'] - pst['y'])
                    extra_dist = max(0.0, candidate_dist - raw_dist)
                    path_transition = self.w_path_extra * extra_dist
                    transition = shift_transition + shift_accel + path_transition
                    cost = prev_cost[k] + st['unary'] + transition
                    if cost < cur_cost[j]:
                        cur_cost[j] = cost
                        cur_back[j] = k
                        cur_delta[j] = shift_delta
            if not np.any(np.isfinite(cur_cost)):
                raise RuntimeError(
                    f'No swept-collision-free DP transition at sample {i}/{n-1}; path needs replanning.'
                )
            prev_cost = cur_cost
            prev_delta = cur_delta
            backrefs.append(cur_back)

        j = int(np.argmin(prev_cost))
        chosen = [None] * n
        for i in range(n - 1, -1, -1):
            chosen[i] = states[i][j]
            if i > 0:
                j = int(backrefs[i][j])
                if j < 0:
                    raise RuntimeError('Broken DP back-reference.')

        # Build output preserving every selected lattice yaw exactly.
        out = Path()
        out.header = raw.header
        stamp = self.get_clock().now().to_msg()
        out.header.stamp = stamp
        shifts = []
        opt_metrics = []
        for i, st in enumerate(chosen):
            p = PoseStamped()
            p.header.frame_id = raw.header.frame_id or self.global_frame
            p.header.stamp = stamp
            p.pose.position.x = float(st['x'])
            p.pose.position.y = float(st['y'])
            qx, qy, qz, qw = yaw_to_quat(float(st['yaw']))
            p.pose.orientation.x = qx
            p.pose.orientation.y = qy
            p.pose.orientation.z = qz
            p.pose.orientation.w = qw
            out.poses.append(p)
            shifts.append(float(st['shift']))
            opt_metrics.append(st['metrics'])
            if i % self.debug_every_n == 0:
                d_l, d_r, c_l, c_r = st['metrics']
                self.get_logger().info(
                    f'DPCLR[{i:03d}] shift={st["shift"]:+.2f}m '
                    f'dL={d_l:.2f} dR={d_r:.2f} cL={c_l:.2f} cR={c_r:.2f}'
                )

        # Final orientation is already the State Lattice / user goal orientation, no abrupt overwrite.
        raw_balance = float(np.mean([abs(a - b) for a, b, _, _ in raw_metrics]))
        opt_balance = float(np.mean([abs(a - b) for a, b, _, _ in opt_metrics]))
        raw_min_mean = float(np.mean([min(a, b) for a, b, _, _ in raw_metrics]))
        opt_min_mean = float(np.mean([min(a, b) for a, b, _, _ in opt_metrics]))
        opt_min_abs = float(min(min(a, b) for a, b, _, _ in opt_metrics))
        self.get_logger().info(
            f'CLEARANCE RESULT | samples={n} avg_shift={np.mean(np.abs(shifts)):.2f}m '
            f'max_shift={np.max(np.abs(shifts)):.2f}m | '
            f'LR imbalance {raw_balance:.2f}->{opt_balance:.2f}m | '
            f'mean min-clear {raw_min_mean:.2f}->{opt_min_mean:.2f}m | '
            f'absolute min-clear={opt_min_abs:.2f}m | DP+swept=OK'
        )
        return out, opt_min_abs

    # -------------------------- final approach / rotation safety --------------------------
    def _rotation_collision(self, x: float, y: float, yaw0: float, yaw1: float) -> bool:
        """Check the full trolley footprint while rotating in place from yaw0 to yaw1."""
        da = norm_angle(yaw1 - yaw0)
        steps = max(1, int(math.ceil(abs(da) / self.goal_rot_step)))
        for k in range(steps + 1):
            a = norm_angle(yaw0 + da * (k / float(steps)))
            if self._pose_collision(x, y, a):
                return True
            d_l, d_r, _, _ = self._side_metrics(x, y, a)
            if min(d_l, d_r) < self.goal_rot_margin:
                return True
        return False

    def _append_safe_final_approach(self, path: Path, raw: Path) -> Path:
        """Keep the final yaw, but avoid doing a large in-place rotation against a wall.

        If the final raw goal can be reached with the existing path safely, leave it alone.
        If the final yaw change would require an unsafe rotation near a wall, find a staging
        pose behind the goal along the final heading. The path then: approach staging ->
        rotate safely at staging -> drive the final short segment straight into the goal.
        """
        if len(path.poses) < 2 or not raw.poses:
            return path
        goal = raw.poses[-1]
        gx = float(goal.pose.position.x)
        gy = float(goal.pose.position.y)
        gyaw = quat_to_yaw(goal.pose.orientation)
        prev = path.poses[-2]
        pyaw = quat_to_yaw(prev.pose.orientation)
        yaw_need = abs(norm_angle(gyaw - pyaw))
        if yaw_need < math.radians(12.0):
            return path

        # If rotating at the goal is safe, preserve the ordinary optimized path.
        if not self._rotation_collision(gx, gy, pyaw, gyaw):
            return path

        # Search behind the goal along final heading so the last segment is straight.
        c, ss = math.cos(gyaw), math.sin(gyaw)
        distances = np.arange(self.goal_stage_min, self.goal_stage_max + 0.5 * self.goal_stage_step, self.goal_stage_step)
        for d in distances:
            sx = gx - float(d) * c
            sy = gy - float(d) * ss
            # Use the approach direction from the preceding optimized pose at staging.
            approach_yaw = math.atan2(sy - float(prev.pose.position.y), sx - float(prev.pose.position.x))
            if self._pose_collision(sx, sy, approach_yaw):
                continue
            if self._rotation_collision(sx, sy, approach_yaw, gyaw):
                continue
            if self._swept_collision((sx, sy, gyaw), (gx, gy, gyaw)):
                continue

            out = Path()
            out.header = path.header
            # Drop final goal pose if present, then append explicit staging sequence.
            out.poses = list(path.poses[:-1])
            stamp = self.get_clock().now().to_msg()
            def mk(x, y, yaw):
                ps = PoseStamped()
                ps.header.frame_id = path.header.frame_id or self.global_frame
                ps.header.stamp = stamp
                ps.pose.position.x = float(x)
                ps.pose.position.y = float(y)
                qx, qy, qz, qw = yaw_to_quat(float(yaw))
                ps.pose.orientation.x = qx; ps.pose.orientation.y = qy
                ps.pose.orientation.z = qz; ps.pose.orientation.w = qw
                return ps
            out.poses.append(mk(sx, sy, approach_yaw))
            out.poses.append(mk(sx, sy, gyaw))
            out.poses.append(mk(gx, gy, gyaw))
            self.get_logger().warn(
                f'FINAL YAW STAGING: unsafe rotation at goal; stage {float(d):.2f}m before goal, rotate, then final straight.'
            )
            return out

        self.get_logger().warn(
            'FINAL YAW STAGING: no safe staging pose found; keep State-Lattice final approach and rely on MPPI footprint safety.'
        )
        return path

    def _static_side_clearance(self, x: float, y: float, yaw: float):
        """Left/right clearance from the trolley side edges to STATIC mapped walls only.

        Unlike _side_metrics(), this does not use inflation costs or dynamic scan points,
        so it cannot recreate the old false 0.05 m inflation-trigger loop.
        """
        if self.static_grid is None or self.static_map_msg is None:
            return self.side_scan, self.side_scan
        c, s = math.cos(yaw), math.sin(yaw)
        nx, ny = -s, c

        def side(sign: float) -> float:
            lx = np.repeat(self.long_samples, len(self.side_distances))
            d = np.tile(self.side_distances, len(self.long_samples))
            wx = x + c * lx + sign * nx * (self.half_w + d)
            wy = y + s * lx + sign * ny * (self.half_w + d)
            sv = self._static_values_at_arrays(wx, wy).reshape(self.side_long_n, -1)
            # Unknown/outside-map values are ignored here; this feedback is specifically
            # for known static hospital walls.
            hit_cols = np.where(np.max(sv, axis=0) >= self.static_occ_thr)[0]
            if len(hit_cols) == 0:
                return self.side_scan
            return float(self.side_distances[int(hit_cols[0])])

        return side(+1.0), side(-1.0)

    def _static_edge_clearances(self, x: float, y: float, yaw: float):
        """Clearance from all four footprint edges to known static occupied cells.

        Each edge is sampled along its full length (not only the 4 corners), then rays are
        cast outward at side_step spacing. Returns (front, left, right, rear).
        """
        if self.static_grid is None or self.static_map_msg is None:
            return (self.side_scan, self.side_scan, self.side_scan, self.side_scan)
        c, s = math.cos(yaw), math.sin(yaw)
        fx, fy = c, s
        lxv, lyv = -s, c
        edge_long = np.linspace(-self.half_l, self.half_l, self.side_long_n)
        edge_lat = np.linspace(-self.half_w, self.half_w, self.side_long_n)
        dists = self.side_distances

        def nearest(edge_x, edge_y, nx, ny):
            ex = np.repeat(edge_x, len(dists))
            ey = np.repeat(edge_y, len(dists))
            dd = np.tile(dists, len(edge_x))
            wx = ex + nx * dd
            wy = ey + ny * dd
            sv = self._static_values_at_arrays(wx, wy).reshape(len(edge_x), -1)
            hit_cols = np.where(np.max(sv, axis=0) >= self.static_occ_thr)[0]
            return self.side_scan if len(hit_cols) == 0 else float(dists[int(hit_cols[0])])

        # front/rear edge points span lateral axis; left/right span longitudinal axis.
        front_x = x + fx * self.half_l + lxv * edge_lat
        front_y = y + fy * self.half_l + lyv * edge_lat
        rear_x = x - fx * self.half_l + lxv * edge_lat
        rear_y = y - fy * self.half_l + lyv * edge_lat
        left_x = x + fx * edge_long + lxv * self.half_w
        left_y = y + fy * edge_long + lyv * self.half_w
        right_x = x + fx * edge_long - lxv * self.half_w
        right_y = y + fy * edge_long - lyv * self.half_w
        return (
            nearest(front_x, front_y, fx, fy),
            nearest(left_x, left_y, lxv, lyv),
            nearest(right_x, right_y, -lxv, -lyv),
            nearest(rear_x, rear_y, -fx, -fy),
        )

    def _forward_sector_clearances(self):
        """Robust clearance over the forward ~170 deg LiDAR field.

        Five sectors are used: left-side, front-left, front, front-right, right-side.
        Distances are measured from the rectangular footprint boundary along each ray,
        and a low percentile is used instead of a single minimum to reject one-point noise.
        """
        pts = self.scan_points_base_all
        far = self.side_scan
        if pts.size == 0:
            return {'left_side': far, 'front_left': far, 'front': far, 'front_right': far, 'right_side': far}
        x = pts[:, 0]; y = pts[:, 1]
        r = np.hypot(x, y)
        a = np.arctan2(y, x)
        half = 0.5 * self.forward_fov
        in_fov = (a >= -half) & (a <= half) & (x > 0.0)
        if not np.any(in_fov):
            return {'left_side': far, 'front_left': far, 'front': far, 'front_right': far, 'right_side': far}
        x=x[in_fov]; y=y[in_fov]; r=r[in_fov]; a=a[in_fov]
        ca=np.maximum(np.abs(np.cos(a)), 1e-6)
        sa=np.maximum(np.abs(np.sin(a)), 1e-6)
        # Distance from trolley center to rectangle boundary on each LiDAR ray.
        body_r = np.minimum(self.half_l / ca, self.half_w / sa)
        clear = np.maximum(0.0, r - body_r)
        deg = np.degrees(a)
        # Positive angles are trolley-left.
        sectors = {
            'left_side': (45.0, 85.0),
            'front_left': (15.0, 45.0),
            'front': (-15.0, 15.0),
            'front_right': (-45.0, -15.0),
            'right_side': (-85.0, -45.0),
        }
        out = {}
        for name,(lo,hi) in sectors.items():
            m=(deg >= lo) & (deg <= hi)
            out[name] = far if not np.any(m) else float(np.percentile(clear[m], self.forward_sector_percentile))
        return out

    # -------------------------- conditional replanning --------------------------
    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, Time(), timeout=Duration(seconds=0.06)
            )
            return (
                float(tf.transform.translation.x),
                float(tf.transform.translation.y),
                quat_to_yaw(tf.transform.rotation),
            )
        except TransformException:
            return None

    def _robot_xy(self):
        pose = self._robot_pose()
        if pose is None:
            return None
        return pose[0], pose[1]

    def _remaining_path_start_index(self, path: Path) -> int:
        xy = self._robot_xy()
        if xy is None or not path.poses:
            return 0
        pts = np.asarray([[p.pose.position.x, p.pose.position.y] for p in path.poses])
        d2 = (pts[:, 0] - xy[0]) ** 2 + (pts[:, 1] - xy[1]) ** 2
        return int(np.argmin(d2))

    def _path_safety(self, path: Path):
        if not path.poses:
            return True, 0.0
        start = self._remaining_path_start_index(path)
        poses = path.poses[start::self.replan_stride]
        min_clear = self.side_scan
        prev = None
        for ps in poses:
            x = float(ps.pose.position.x)
            y = float(ps.pose.position.y)
            yaw = quat_to_yaw(ps.pose.orientation)
            if self._pose_collision(x, y, yaw):
                return True, 0.0
            if prev is not None and self._swept_collision(prev, (x, y, yaw)):
                return True, 0.0
            d_l, d_r, _, _ = self._side_metrics(x, y, yaw)
            min_clear = min(min_clear, d_l, d_r)
            prev = (x, y, yaw)
        return False, float(min_clear)

    def _dynamic_scan_path_collision(self, path: Path) -> bool:
        """True only when a NEW (not static-map) full-scan obstacle intersects remaining path."""
        if self.scan_points_map.size == 0 or not path.poses:
            return False
        start = self._remaining_path_start_index(path)
        for ps in path.poses[start::self.replan_stride]:
            x = float(ps.pose.position.x); y = float(ps.pose.position.y)
            yaw = quat_to_yaw(ps.pose.orientation)
            c, ss = math.cos(yaw), math.sin(yaw)
            dx = self.scan_points_map[:, 0] - x
            dy = self.scan_points_map[:, 1] - y
            lx = c * dx + ss * dy
            ly = -ss * dx + c * dy
            if np.any((np.abs(lx) <= self.half_l + 0.08) & (np.abs(ly) <= self.half_w + 0.08)):
                return True
        return False

    @staticmethod
    def _point_segment_distance(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
        vx = bx - ax
        vy = by - ay
        vv = vx * vx + vy * vy
        if vv <= 1e-12:
            return math.hypot(px - ax, py - ay)
        t = ((px - ax) * vx + (py - ay) * vy) / vv
        t = max(0.0, min(1.0, t))
        qx = ax + t * vx
        qy = ay + t * vy
        return math.hypot(px - qx, py - qy)

    def _path_lateral_error(self, path: Path) -> Optional[float]:
        """Cross-track distance from trolley center to the current optimized path."""
        xy = self._robot_xy()
        if xy is None or not path.poses:
            return None
        if len(path.poses) == 1:
            p = path.poses[0].pose.position
            return math.hypot(xy[0] - float(p.x), xy[1] - float(p.y))
        nearest = self._remaining_path_start_index(path)
        # Include one segment behind the nearest sample so sparse 0.2 m path sampling
        # cannot create a false cross-track error between two valid path poses.
        first = max(0, nearest - 1)
        best = float('inf')
        for i in range(first, len(path.poses) - 1):
            a = path.poses[i].pose.position
            b = path.poses[i + 1].pose.position
            d = self._point_segment_distance(
                xy[0], xy[1], float(a.x), float(a.y), float(b.x), float(b.y)
            )
            if d < best:
                best = d
        return float(best)

    def _trigger_replan(self, reason: str) -> None:
        self.get_logger().warn(f'CONDITIONAL REPLAN: {reason}')
        self.last_replan_wall_time = time.monotonic()
        self.deviation_violation_count = 0
        self.dynamic_obstacle_violation_count = 0
        self.wall_clearance_violation_count = 0
        self.pending_goal = self.active_goal
        h = self.follow_handle
        if h is None:
            return
        f = h.cancel_goal_async()
        f.add_done_callback(lambda _: self._request_plan())

    def _conditional_replan_check(self) -> None:
        if not self.enable_replan or self.current_path is None or self.active_goal is None:
            return
        if self.plan_request_active:
            return
        if time.monotonic() - self.last_replan_wall_time < self.replan_cooldown:
            return
        if self.follow_handle is None or self.follow_handle.status not in (
            GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING
        ):
            return

        # Startup grace: do not replace a freshly accepted path while the controller is
        # settling and scan/static-map edge classifications are still changing.
        if time.monotonic() - self.follow_start_wall_time < self.replan_start_grace:
            self.dynamic_obstacle_violation_count = 0
            self.deviation_violation_count = 0
            self.wall_clearance_violation_count = 0
            return

        # Trigger A: genuinely new LiDAR obstacle on the remaining swept route.
        # Require three consecutive observations before cancelling FollowPath. This blocks
        # one-frame scan/map edge false positives while preserving persistent obstacles.
        if self._dynamic_scan_path_collision(self.current_path):
            self.dynamic_obstacle_violation_count += 1
            if self.dynamic_obstacle_violation_count >= self.dynamic_obstacle_confirmations:
                self._trigger_replan(
                    f'NEW full-scan obstacle intersects remaining path for '
                    f'{self.dynamic_obstacle_confirmations} consecutive checks'
                )
                return
        else:
            self.dynamic_obstacle_violation_count = 0

        # Trigger C: STATIC wall-clearance feedback after corners / during tracking.
        # Check at 0.5 s cadence and require 3 consecutive violations (~1.5 s total).
        # This is deliberately based on the static occupancy map, not inflation cost, so
        # a high inflation ring cannot recreate the old immediate-cancel loop.
        now_wall = time.monotonic()
        if now_wall - self.last_wall_check_wall_time >= self.wall_replan_period:
            self.last_wall_check_wall_time = now_wall
            pose = self._robot_pose()
            if pose is not None:
                d_front, d_left, d_right, d_rear = self._static_edge_clearances(*pose)
                sectors = self._forward_sector_clearances()
                side_imbalance = abs(d_left - d_right)
                edge_bad = min(d_front, d_left, d_right) < self.edge_replan_min_clearance
                sector_bad = (
                    sectors['front'] < self.front_sector_clearance
                    or sectors['front_left'] < self.front_diag_clearance
                    or sectors['front_right'] < self.front_diag_clearance
                    or sectors['left_side'] < self.side_sector_clearance
                    or sectors['right_side'] < self.side_sector_clearance
                )
                # Do not replan just because one forward ray sees a normal corner wall.
                # A forward-field warning must agree with near-body edge risk, OR the
                # left/right static edge geometry must be persistently unbalanced.
                wall_bad = edge_bad or side_imbalance > self.wall_replan_imbalance
                if sector_bad and min(d_front, d_left, d_right) < max(0.55, self.edge_replan_min_clearance + 0.20):
                    wall_bad = True
                if wall_bad:
                    self.wall_clearance_violation_count += 1
                    if self.wall_clearance_violation_count >= self.wall_replan_confirmations:
                        self._trigger_replan(
                            f'EDGE+170FOV clearance feedback edges(F/L/R/B)='
                            f'{d_front:.2f}/{d_left:.2f}/{d_right:.2f}/{d_rear:.2f}m; '
                            f'sectors(LS/FL/F/FR/RS)='
                            f'{sectors["left_side"]:.2f}/{sectors["front_left"]:.2f}/{sectors["front"]:.2f}/'
                            f'{sectors["front_right"]:.2f}/{sectors["right_side"]:.2f}m for '
                            f'{self.wall_replan_confirmations} checks (~{self.wall_replan_period*self.wall_replan_confirmations:.1f}s)'
                        )
                        return
                else:
                    self.wall_clearance_violation_count = 0

        # Trigger B: feedback correction. If the physical trolley drifts away from the
        # green path, re-run State Lattice + clearance optimization from the CURRENT pose.
        # Require consecutive violations to avoid replanning on TF/path discretization noise.
        err = self._path_lateral_error(self.current_path)
        if err is None:
            return
        if err <= self.replan_path_deviation:
            self.deviation_violation_count = 0
            return
        self.deviation_violation_count += 1
        if self.deviation_violation_count < self.replan_deviation_confirmations:
            return
        self._trigger_replan(
            f'path tracking error {err:.2f}m > {self.replan_path_deviation:.2f}m '
            f'for {self.replan_deviation_confirmations} consecutive checks'
        )

    # ---------------------- explicit initial pre-alignment ----------------------
    def _current_base_yaw(self) -> Optional[float]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, Time(), timeout=Duration(seconds=0.20)
            )
            return quat_to_yaw(tf.transform.rotation)
        except TransformException as exc:
            self.get_logger().warn(f'PREALIGN skipped: base TF unavailable: {exc}')
            return None

    def _initial_path_heading(self, path: Path) -> Optional[float]:
        if len(path.poses) < 2:
            return None
        x0 = float(path.poses[0].pose.position.x)
        y0 = float(path.poses[0].pose.position.y)
        last_x, last_y = x0, y0
        accum = 0.0
        for ps in path.poses[1:]:
            x = float(ps.pose.position.x)
            y = float(ps.pose.position.y)
            accum += math.hypot(x - last_x, y - last_y)
            last_x, last_y = x, y
            if accum >= self.initial_heading_sample_distance:
                if math.hypot(x - x0, y - y0) > 0.05:
                    return math.atan2(y - y0, x - x0)
        # For a very short path, use the last translated pose. If the path is pure
        # in-place rotation, preserve its SE(2) orientation instead.
        xe = float(path.poses[-1].pose.position.x)
        ye = float(path.poses[-1].pose.position.y)
        if math.hypot(xe - x0, ye - y0) > 0.05:
            return math.atan2(ye - y0, xe - x0)
        return quat_to_yaw(path.poses[-1].pose.orientation)

    def _prealign_then_follow(self, path: Path) -> None:
        current_yaw = self._current_base_yaw()
        desired_yaw = self._initial_path_heading(path)
        if current_yaw is None or desired_yaw is None:
            self._send_follow_path(path)
            return
        delta = norm_angle(desired_yaw - current_yaw)
        deg = math.degrees(delta)
        if abs(delta) < self.initial_align_threshold:
            self.get_logger().info(f'PREALIGN not needed: heading error={deg:+.1f}deg')
            self._send_follow_path(path)
            return
        if not self.spin_client.wait_for_server(timeout_sec=1.5):
            self.get_logger().error('PREALIGN failed: /spin action server not ready; holding motion.')
            return
        goal = Spin.Goal()
        goal.target_yaw = float(delta)  # Humble Spin uses relative rotation angle.
        goal.time_allowance.sec = int(math.ceil(self.initial_spin_timeout))
        goal.time_allowance.nanosec = 0
        self.get_logger().info(
            f'PREALIGN START: rotate {deg:+.1f}deg in place before FollowPath.'
        )
        fut = self.spin_client.send_goal_async(goal)
        fut.add_done_callback(lambda f, p=path: self._on_spin_goal(f, p))

    def _on_spin_goal(self, future, path: Path) -> None:
        try:
            h = future.result()
        except Exception as exc:
            self.get_logger().error(f'PREALIGN Spin goal exception: {exc}')
            return
        if h is None or not h.accepted:
            self.get_logger().error('PREALIGN Spin rejected; holding motion.')
            return
        self.spin_handle = h
        h.get_result_async().add_done_callback(lambda f, handle=h, p=path: self._on_spin_result(f, handle, p))

    def _on_spin_result(self, future, handle, path: Path) -> None:
        try:
            wrapped = future.result()
            status = wrapped.status
            self.get_logger().info(f'PREALIGN FINISHED status={status}')
        except Exception as exc:
            self.get_logger().error(f'PREALIGN Spin result exception: {exc}')
            if self.spin_handle is handle:
                self.spin_handle = None
            return
        if self.spin_handle is handle:
            self.spin_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('PREALIGN OK -> sending CLEARANCE-DP FollowPath.')
            self._send_follow_path(path)
        elif status == GoalStatus.STATUS_CANCELED and self.pending_goal is not None:
            if not self.plan_request_active:
                self._request_plan()
        else:
            self.get_logger().warn('PREALIGN did not succeed; holding motion for safety.')

    # ----------------------------- following -----------------------------
    def _send_follow_path(self, path: Path) -> None:
        if not self.follow_client.wait_for_server(timeout_sec=1.5):
            self.get_logger().error('FollowPath action server not ready.')
            return
        g = FollowPath.Goal()
        g.path = path
        try:
            g.controller_id = 'FollowPath'
            g.goal_checker_id = 'general_goal_checker'
        except Exception:
            pass
        self.follow_client.send_goal_async(g).add_done_callback(self._on_follow_goal)

    def _on_follow_goal(self, future) -> None:
        try:
            h = future.result()
        except Exception as exc:
            self.get_logger().error(f'FollowPath goal exception: {exc}')
            return
        if h is None or not h.accepted:
            self.get_logger().error('FollowPath rejected optimized path.')
            return
        self.follow_handle = h
        self.follow_start_wall_time = time.monotonic()
        self.dynamic_obstacle_violation_count = 0
        self.deviation_violation_count = 0
        self.wall_clearance_violation_count = 0
        self.last_wall_check_wall_time = time.monotonic()
        self.get_logger().info(
            f'FollowPath accepted CLEARANCE-DP path. Replan grace={self.replan_start_grace:.1f}s; '
            f'dynamic confirmation={self.dynamic_obstacle_confirmations}x; '
            f'wall feedback={self.wall_replan_period:.1f}s x{self.wall_replan_confirmations}.'
        )
        h.get_result_async().add_done_callback(lambda fut, handle=h: self._on_follow_result(fut, handle))

    def _on_follow_result(self, future, handle) -> None:
        try:
            wrapped = future.result()
            self.get_logger().info(f'FollowPath finished status={wrapped.status}')
        except Exception as exc:
            self.get_logger().error(f'FollowPath result exception: {exc}')
        # A cancelled older goal may finish after a new FollowPath was already accepted.
        # Do not erase the newer handle in that race.
        if self.follow_handle is handle:
            self.follow_handle = None
        if self.pending_goal is not None and not self.plan_request_active:
            self._request_plan()


def main(args=None):
    rclpy.init(args=args)
    node = ClearanceNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
