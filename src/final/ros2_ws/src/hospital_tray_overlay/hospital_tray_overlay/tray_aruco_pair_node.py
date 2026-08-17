#!/usr/bin/env python3
"""Detect a three-post ArUco gate for cooperative tray docking.

AMR1 uses the LEFT OUTER post (IDs 40/41) plus shared CENTER ID 44.
AMR2 uses shared CENTER ID 44 plus the RIGHT OUTER post (IDs 42/43).

If both upper/lower markers on an outer post are visible, their image centers are
averaged into one virtual outer-post observation.  This removes vertical-bias from
the pair center and makes the visual target correspond to the physical bay center.
If only one outer marker is visible, docking continues with that marker as fallback.
"""
from __future__ import annotations

import json
import math
import time
from typing import Any

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


def _image_to_bgr(msg: Image) -> np.ndarray:
    enc = str(msg.encoding).lower()
    h, w = int(msg.height), int(msg.width)
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(enc)
    if channels is None:
        raise RuntimeError(f"unsupported image encoding: {msg.encoding}")
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    rows = raw.reshape(h, int(msg.step))[:, : w * channels]
    if channels == 1:
        return cv2.cvtColor(rows.reshape(h, w), cv2.COLOR_GRAY2BGR)
    image = rows.reshape(h, w, channels)
    if enc == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if enc == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if enc == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image.copy()


def _marker_side_px(points: np.ndarray) -> float:
    pts = points.reshape(4, 2).astype(np.float32)
    return float(sum(np.linalg.norm(pts[(i + 1) % 4] - pts[i]) for i in range(4)) / 4.0)


def _virtual_marker(marker_data: dict[int, dict[str, Any]], ids: list[int]) -> dict[str, Any] | None:
    visible = [marker_data[i] for i in ids if i in marker_data]
    visible_ids = [i for i in ids if i in marker_data]
    if not visible:
        return None
    # Averaging upper+lower markers makes one virtual outer-post center at the same
    # vertical level as the shared center marker. With one visible marker, it is a
    # graceful fallback instead of a hard stop.
    return {
        "center_x": float(sum(float(v["center_x"]) for v in visible) / len(visible)),
        "center_y": float(sum(float(v["center_y"]) for v in visible) / len(visible)),
        "side_px": float(sum(float(v["side_px"]) for v in visible) / len(visible)),
        "source_ids": visible_ids,
        "source_count": len(visible_ids),
    }


class TrayArucoPairNode(Node):
    def __init__(self) -> None:
        super().__init__("tray_aruco_pair_node")
        self.declare_parameter("amr_id", "amr1")
        self.declare_parameter("image_topic", "/amr1/camera/front/color/image_raw")
        self.declare_parameter("result_topic", "/amr1/tray_aruco/result")
        self.declare_parameter("debug_image_topic", "/amr1/tray_aruco/debug_image")
        self.declare_parameter("outer_ids", [40, 41])
        self.declare_parameter("center_id", 44)
        self.declare_parameter("outer_side", "left")  # left for AMR1, right for AMR2
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("publish_hz", 15.0)
        self.declare_parameter("show_window", True)
        self.declare_parameter("window_width", 760)
        self.declare_parameter("window_height", 520)

        self.amr_id = str(self.get_parameter("amr_id").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.result_topic = str(self.get_parameter("result_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.outer_ids = [int(v) for v in self.get_parameter("outer_ids").value]
        self.center_id = int(self.get_parameter("center_id").value)
        self.outer_side = str(self.get_parameter("outer_side").value).strip().lower()
        if self.outer_side not in ("left", "right"):
            raise RuntimeError("outer_side must be left or right")
        self.publish_period = 1.0 / max(1.0, float(self.get_parameter("publish_hz").value))
        self.show_window = bool(self.get_parameter("show_window").value)
        self.window_width = max(320, int(self.get_parameter("window_width").value))
        self.window_height = max(240, int(self.get_parameter("window_height").value))
        self.window_name = f"DOOSIM {self.amr_id.upper()} ARUCO SCANNER"
        self.window_failed = False

        dictionary_name = str(self.get_parameter("dictionary").value)
        dictionary_id = getattr(cv2.aruco, dictionary_name, cv2.aruco.DICT_4X4_50)
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        params = cv2.aruco.DetectorParameters()
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 35
        params.adaptiveThreshWinSizeStep = 8
        params.minMarkerPerimeterRate = 0.012
        params.maxMarkerPerimeterRate = 4.0
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        try:
            self.detector = cv2.aruco.ArucoDetector(dictionary, params)
            self.legacy_dictionary = None
            self.legacy_params = None
        except AttributeError:
            self.detector = None
            self.legacy_dictionary = dictionary
            self.legacy_params = params

        self.last_publish = -1.0
        self.pub = self.create_publisher(String, self.result_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, qos_profile_sensor_data)
        self.create_subscription(Image, self.image_topic, self._on_image, qos_profile_sensor_data)
        self.get_logger().info(
            f"[{self.amr_id}] tray 3-post ArUco gate ready: outer={self.outer_ids} "
            f"center={self.center_id} outer_side={self.outer_side} RGB={self.image_topic}"
        )

    def _detect(self, gray: np.ndarray):
        if self.detector is not None:
            return self.detector.detectMarkers(gray)
        return cv2.aruco.detectMarkers(gray, self.legacy_dictionary, parameters=self.legacy_params)

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if self.last_publish > 0.0 and now - self.last_publish < self.publish_period:
            return
        self.last_publish = now
        try:
            frame = _image_to_bgr(msg)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self._detect(gray)
        except Exception as exc:
            self.get_logger().warning(f"tray ArUco image processing failed: {exc}")
            return

        marker_data: dict[int, dict[str, Any]] = {}
        drawn: list[tuple[int, np.ndarray]] = []
        if ids is not None:
            for marker_corners, raw_id in zip(corners, ids.flatten()):
                marker_id = int(raw_id)
                pts = np.asarray(marker_corners, dtype=np.float32).reshape(4, 2)
                center = pts.mean(axis=0)
                marker_data[marker_id] = {
                    "center_x": float(center[0]),
                    "center_y": float(center[1]),
                    "side_px": _marker_side_px(pts),
                }
                drawn.append((marker_id, pts.copy()))

        h, w = frame.shape[:2]
        outer = _virtual_marker(marker_data, self.outer_ids)
        center = _virtual_marker(marker_data, [self.center_id])
        pair: dict[str, Any] | None = None
        if outer is not None and center is not None:
            if self.outer_side == "left":
                left, right = outer, center
                left_role, right_role = "OUTER", "CENTER"
            else:
                left, right = center, outer
                left_role, right_role = "CENTER", "OUTER"

            lx, ly = float(left["center_x"]), float(left["center_y"])
            rx, ry = float(right["center_x"]), float(right["center_y"])
            center_x = 0.5 * (lx + rx)
            center_y = 0.5 * (ly + ry)
            size_mean = max(1.0, 0.5 * (float(left["side_px"]) + float(right["side_px"])))
            pair = {
                "left_role": left_role,
                "right_role": right_role,
                "left_source_ids": list(left.get("source_ids", [])),
                "right_source_ids": list(right.get("source_ids", [])),
                "outer_source_ids": list(outer.get("source_ids", [])),
                "center_id": self.center_id,
                "left_center_x": lx,
                "left_center_y": ly,
                "right_center_x": rx,
                "right_center_y": ry,
                "pair_center_x": center_x,
                "pair_center_y": center_y,
                "center_error_px": center_x - float(w) * 0.5,
                "pair_spacing_px": float(math.hypot(rx - lx, ry - ly)),
                "left_side_px": float(left["side_px"]),
                "right_side_px": float(right["side_px"]),
                "mean_side_px": size_mean,
                "size_error_ratio": float((float(right["side_px"]) - float(left["side_px"])) / size_mean),
                "line_angle_deg": float(math.degrees(math.atan2(ry - ly, rx - lx))),
                "outer_redundancy": int(outer.get("source_count", 1)),
            }

        debug = frame.copy()
        cam_cx = int(round(w * 0.5))
        cv2.line(debug, (cam_cx, 0), (cam_cx, h - 1), (255, 255, 0), 2)
        expected = set(self.outer_ids + [self.center_id])
        for marker_id, pts in drawn:
            poly = np.round(pts).astype(np.int32).reshape((-1, 1, 2))
            color = (0, 255, 0) if marker_id in expected else (0, 180, 255)
            cv2.polylines(debug, [poly], True, color, 2, cv2.LINE_AA)
            cx, cy = np.round(pts.mean(axis=0)).astype(int)
            cv2.putText(debug, f"ID {marker_id}", (cx - 24, max(16, cy - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
        single_candidates = sorted(set(marker_data.keys()).intersection(expected))
        if pair is not None:
            pcx, pcy = int(round(pair["pair_center_x"])), int(round(pair["pair_center_y"]))
            cv2.circle(debug, (pcx, pcy), 8, (0, 255, 0), 2)
            cv2.putText(
                debug,
                f"GATE outer={pair['outer_source_ids']} + center={self.center_id} err={pair['center_error_px']:+.1f}px",
                (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (0, 255, 0), 2, cv2.LINE_AA,
            )
        elif single_candidates:
            cv2.putText(
                debug, f"SINGLE ID READY {single_candidates} -> FIXED INSERT", (8, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.56, (0, 255, 255), 2, cv2.LINE_AA,
            )
        else:
            cv2.putText(
                debug, f"WAIT ID outer={self.outer_ids} center={self.center_id}", (8, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.56, (0, 120, 255), 2, cv2.LINE_AA,
            )

        # V2.8 demo scanner overlay: this is visual feedback only and does not
        # change the detector or docking control.  The moving scan line makes it
        # immediately obvious that the camera/ArUco node is alive.
        sweep_y = int((time.monotonic() * 120.0) % max(1, h))
        cv2.line(debug, (0, sweep_y), (w - 1, sweep_y), (255, 220, 80), 1, cv2.LINE_AA)
        if pair is not None:
            state_text = "PAIR LOCKED"
            state_color = (0, 255, 0)
        elif single_candidates:
            state_text = f"SINGLE ID {single_candidates[0]} -> FIXED INSERT"
            state_color = (0, 255, 255)
        else:
            state_text = "SCANNING..."
            state_color = (0, 180, 255)
        cv2.rectangle(debug, (0, h - 38), (w, h), (12, 12, 12), -1)
        cv2.putText(
            debug, f"{self.amr_id.upper()}  {state_text}  visible={sorted(marker_data.keys())}",
            (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.58, state_color, 2, cv2.LINE_AA,
        )

        if self.show_window and not self.window_failed:
            try:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, self.window_width, self.window_height)
                cv2.imshow(self.window_name, debug)
                cv2.waitKey(1)
            except Exception as exc:
                self.window_failed = True
                self.get_logger().warning(
                    f"OpenCV scanner window disabled ({exc}); debug image topic remains available: {self.debug_image_topic}"
                )

        debug_msg = Image()
        debug_msg.header = msg.header
        debug_msg.height = int(h)
        debug_msg.width = int(w)
        debug_msg.encoding = "bgr8"
        debug_msg.is_bigendian = 0
        debug_msg.step = int(w * 3)
        debug_msg.data = debug.tobytes()
        self.debug_pub.publish(debug_msg)

        out = String()
        out.data = json.dumps({
            "state": "PAIR" if pair is not None else ("SINGLE" if single_candidates else "SEARCHING"),
            "amr": self.amr_id,
            "timestamp": time.time(),
            "image_width": int(w),
            "image_height": int(h),
            "visible_ids": sorted(marker_data.keys()),
            "single_candidate_ids": single_candidates,
            "outer_ids": self.outer_ids,
            "center_id": self.center_id,
            "outer_side": self.outer_side,
            "pair": pair,
        }, separators=(",", ":"))
        self.pub.publish(out)


    def destroy_node(self):
        if self.show_window and not self.window_failed:
            try:
                cv2.destroyWindow(self.window_name)
                cv2.waitKey(1)
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrayArucoPairNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
