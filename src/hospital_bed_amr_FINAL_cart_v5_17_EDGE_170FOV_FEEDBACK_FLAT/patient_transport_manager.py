#!/usr/bin/env python3
"""김서울 1명 전용 통합 시나리오.

1 입력 -> 기존 OCR Pose Nav2 이동 -> OCR은 환자 신원만 검증
-> 독립 ArUco 10/11 중점으로 침대 중심 정렬 -> 3.328 m 전진
-> C와 동일한 마그네틱 결합 -> 3.000 m 강제 직선 후진
-> 엘리베이터 2F -> MRI 왕복 -> 엘리베이터 1F -> 병실까지 자동 복귀.

중요: OCR 판정은 hospital_ocr_bridge가 담당하지만, OCR bbox는 도킹 위치 제어에
사용하지 않는다. 환자 확인 후 좌/우 독립 ArUco pair의 중점을 침대 중심으로 사용한다.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import shlex
import sys
import time
import uuid


def _restart_with_ros_environment() -> None:
    if os.environ.get("KIMSEOUL_MISSION_ROS_READY") == "1":
        return
    root = Path(__file__).resolve().parent
    ros_setup = Path("/opt/ros/humble/setup.bash")
    ws_setup = root / "ros2_ws" / "install" / "setup.bash"
    if not ros_setup.is_file():
        raise SystemExit("[오류] ROS 2 Humble setup.bash가 없습니다.")
    if not ws_setup.is_file():
        raise SystemExit("[오류] 먼저 ./02_build_ros_ws.sh를 실행하세요.")
    command = " ".join([
        f"source {shlex.quote(str(ros_setup))}",
        f"&& source {shlex.quote(str(ws_setup))}",
        "&& export ROS_DOMAIN_ID=120",
        "&& export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}",
        "&& export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}",
        "&& export KIMSEOUL_MISSION_ROS_READY=1",
        "&& if [ -f \"$HOME/.ros/fastdds_whitelist.xml\" ]; then export FASTRTPS_DEFAULT_PROFILES_FILE=\"$HOME/.ros/fastdds_whitelist.xml\"; fi",
        "&& exec python3",
        shlex.quote(str(Path(__file__).resolve())),
        *(shlex.quote(arg) for arg in sys.argv[1:]),
    ])
    os.execv("/bin/bash", ["bash", "-lc", command])


_restart_with_ros_environment()

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.srv import LoadMap
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


PATIENT_NAME = "김서울"
PATIENT_BIRTH_DATE = "2000-11-02"

AMR1_INITIAL_POSE = {"x": -45.0467, "y": 31.8558, "yaw": -1.566514}
SEOUL_OCR_POSE = {"x": -43.6377, "y": 11.4166, "yaw": -1.565538}
ELEVATOR = {"x": -26.208667755126953, "y": 22.487224578857422}
# Measured MRI TableTop world XY from the fixed runtime MRI target.  The old hard-coded
# navigation goal (7.02, 6.37464) was 1.61 m away, outside the 1.25 m patient-transfer
# trigger radius.  Derive a reproducible safe stop point 1.05 m away along the same
# approach direction so Nav2 stops before the MRI target while the bed is definitely
# inside the automatic transfer zone.
MRI_TARGET_2F = {"x": 8.5246, "y": 5.8035}
MRI_OLD_APPROACH_2F = {"x": 7.02, "y": 6.37464}
MRI_SAFE_STANDOFF_M = 1.05
_mri_dx = MRI_OLD_APPROACH_2F["x"] - MRI_TARGET_2F["x"]
_mri_dy = MRI_OLD_APPROACH_2F["y"] - MRI_TARGET_2F["y"]
_mri_norm = max(1e-9, math.hypot(_mri_dx, _mri_dy))
MRI_2F = {
    "x": MRI_TARGET_2F["x"] + MRI_SAFE_STANDOFF_M * _mri_dx / _mri_norm,
    "y": MRI_TARGET_2F["y"] + MRI_SAFE_STANDOFF_M * _mri_dy / _mri_norm,
}
MRI_CLEAR_DISTANCE_M = 11.0
MRI_CLEAR_2F = {"x": MRI_2F["x"], "y": MRI_2F["y"] + MRI_CLEAR_DISTANCE_M}
MRI_TRANSFER_TIMEOUT_S = 8.0
MRI_RETURN_TIMEOUT_S = 8.0
APPROACH_DISTANCE_M = 3.328
REVERSE_DISTANCE_M = 3.000
REVERSE_SPEED_MPS = 0.22
ELEVATOR_RETURN_REVERSE_DISTANCE_M = 10.0
ELEVATOR_RETURN_REVERSE_SPEED_MPS = 0.25
ELEVATOR_RETURN_EXIT_DISTANCE_M = 5.0
ELEVATOR_RETURN_EXIT_SPEED_MPS = 0.25

CENTER_GOAL_TOPIC = "/center_goal"
CENTER_STATUS_TOPIC = "/center_goal/status"
INITIAL_LOCK_TOPIC = "/initial_pose_locked"
AUTO_COMMAND_TOPIC = "/amr1/auto_approach/command"
ALIGN_STATUS_TOPIC = "/amr1/align/status"
OCR_REQUEST_TOPIC = "/amr1/ocr/request"
OCR_RESULT_TOPIC = "/amr1/ocr/result"
OCR_CONTROL_TOPIC = "/amr1/ocr/control"
ARUCO_RESULT_TOPIC = "/amr1/aruco/result"
ARUCO_PAIRS = {"김서울": (10, 11), "박인천": (20, 21), "서수원": (30, 31)}
ARUCO_X_TOLERANCE_PX = 12.0
ARUCO_STABLE_MESSAGES = 2
ARUCO_YAW_THRESHOLD_PX = 70.0
ARUCO_LATERAL_KP = 1.6
ARUCO_MAX_LATERAL_MPS = 0.10
ARUCO_YAW_KP = 0.9
ARUCO_MAX_YAW_RAD_S = 0.18
ARUCO_MIN_YAW_RAD_S = 0.05
ARUCO_RESULT_STALE_SEC = 0.75
APPROACH_SPEED_MPS = 0.32
MAGNET_COMMAND_TOPIC = "/amr1/magnet/command"
MAGNET_STATUS_TOPIC = "/amr1/magnet/status"
CMD_VEL_TOPIC = "/cmd_vel"
ELEVATOR_ARRIVAL_TOPIC = "/elevator/amr_arrived"
ELEVATOR_STATUS_TOPIC = "/elevator/status"
ELEVATOR_MAP_ACK_TOPIC = "/elevator/map_switch_ack"
ELEVATOR_FINAL_TOPIC = "/amr1/elevator/arrived"
MAP_LOAD_SERVICE = "/map_server/load_map"
PATIENT1_STATUS_TOPIC = "/patient_transfer/patient1/status"


class KimSeoulMission(Node):
    def __init__(self) -> None:
        super().__init__("kimseoul_patient_transport_manager")

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.goal_pub = self.create_publisher(PoseStamped, CENTER_GOAL_TOPIC, 10)
        self.auto_pub = self.create_publisher(String, AUTO_COMMAND_TOPIC, 10)
        self.ocr_request_pub = self.create_publisher(String, OCR_REQUEST_TOPIC, 10)
        self.ocr_control_pub = self.create_publisher(String, OCR_CONTROL_TOPIC, 10)
        self.magnet_pub = self.create_publisher(String, MAGNET_COMMAND_TOPIC, 10)
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 20)
        self.elevator_arrival_pub = self.create_publisher(String, ELEVATOR_ARRIVAL_TOPIC, 10)
        self.elevator_map_ack_pub = self.create_publisher(String, ELEVATOR_MAP_ACK_TOPIC, 10)
        self.map_client = self.create_client(LoadMap, MAP_LOAD_SERVICE)

        self.create_subscription(String, CENTER_STATUS_TOPIC, self._on_nav, latched)
        self.create_subscription(String, ALIGN_STATUS_TOPIC, self._on_align, 10)
        self.create_subscription(String, OCR_RESULT_TOPIC, self._on_ocr_result, 10)
        self.create_subscription(String, ARUCO_RESULT_TOPIC, self._on_aruco_result, 10)
        self.create_subscription(String, MAGNET_STATUS_TOPIC, self._on_magnet, 10)
        self.create_subscription(Bool, INITIAL_LOCK_TOPIC, self._on_lock, latched)
        self.create_subscription(String, ELEVATOR_STATUS_TOPIC, self._on_elevator_status, 10)
        self.create_subscription(Bool, ELEVATOR_FINAL_TOPIC, self._on_elevator_final, latched)
        self.create_subscription(String, PATIENT1_STATUS_TOPIC, self._on_patient_status, 10)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.initial_locked = False
        self.nav_status = ""
        self.align_payload: dict = {}
        self.magnet_payload: dict = {}
        self.elevator_payload: dict = {}
        self.elevator_final = False
        self.ocr_payload: dict = {}
        self.aruco_payload: dict = {}
        self.aruco_received_at = 0.0
        self.patient_status_payload: dict = {}

    def _on_nav(self, msg: String) -> None:
        self.nav_status = str(msg.data)

    def _on_align(self, msg: String) -> None:
        try:
            self.align_payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.align_payload = {"state": "ERROR", "reason": "invalid align status JSON"}

    def _on_ocr_result(self, msg: String) -> None:
        try:
            self.ocr_payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

    def _on_aruco_result(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if str(payload.get("amr", "amr1")) != "amr1":
            return
        self.aruco_payload = payload
        self.aruco_received_at = time.monotonic()

    def _on_magnet(self, msg: String) -> None:
        try:
            self.magnet_payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

    def _on_lock(self, msg: Bool) -> None:
        self.initial_locked = bool(msg.data)

    def _on_elevator_status(self, msg: String) -> None:
        try:
            self.elevator_payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

    def _on_elevator_final(self, msg: Bool) -> None:
        self.elevator_final = bool(msg.data)

    def _on_patient_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if str(payload.get("id", "")).lower() not in {"patient1", ""}:
            return
        self.patient_status_payload = payload

    def wait_patient_state(self, expected: str, timeout: float, label: str) -> bool:
        expected = str(expected)
        end = time.monotonic() + timeout
        last_print = 0.0
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            state = str(self.patient_status_payload.get("state", ""))
            if state == expected:
                print(f"[MRI 상태 확인] {label}: state={state}")
                return True
            now = time.monotonic()
            if now - last_print > 0.8:
                print(f"[MRI 상태 대기] {label}: current={state or 'NO_STATUS'} expected={expected}")
                last_print = now
        print(f"[MRI 실패] {label}: {timeout:.1f}초 안에 patient state={expected} 확인 못함")
        return False

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self) -> None:
        msg = Twist()
        for _ in range(8):
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)

    def wait_for_system(self, timeout: float = 45.0) -> bool:
        print("[준비] 자동 초기 Pose, Nav2, Isaac, OCR + ArUco launch 연결을 확인합니다.")
        end = time.monotonic() + timeout
        last = 0.0
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            nav_ready = self.goal_pub.get_subscription_count() > 0 and not self.nav_status.startswith("ACTIVE")
            isaac_ready = self.auto_pub.get_subscription_count() > 0
            magnet_ready = self.magnet_pub.get_subscription_count() > 0
            ocr_ready = self.count_subscribers(OCR_REQUEST_TOPIC) > 0
            aruco_ready = self.count_publishers(ARUCO_RESULT_TOPIC) > 0
            tf_ready = self.lookup_pose("map", "base_link") is not None
            if self.initial_locked and nav_ready and isaac_ready and magnet_ready and ocr_ready and aruco_ready and tf_ready:
                print("[준비 완료] 자동 Pose + AMR1 Nav2 + Isaac + OCR 신원확인 + ArUco 중심검출 연결 완료")
                return True
            now = time.monotonic()
            if now - last > 2.0:
                print(
                    "[대기] "
                    f"pose={self.initial_locked}, nav={nav_ready}, tf={tf_ready}, "
                    f"isaac={isaac_ready}, magnet={magnet_ready}, ocr_launch={ocr_ready}, aruco={aruco_ready}"
                )
                last = now
        print("[실패] 시스템 준비 제한시간 초과")
        print("       OCR 터미널에 OCR model ready와 paired ArUco ready가 둘 다 보이는지 확인하세요.")
        return False

    def lookup_pose(self, parent: str, child: str):
        try:
            tf = self.tf_buffer.lookup_transform(parent, child, Time())
        except TransformException:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return float(t.x), float(t.y), float(yaw)

    def make_goal(self, target: dict[str, float]) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(target["x"])
        msg.pose.position.y = float(target["y"])
        if "yaw" in target:
            yaw = float(target["yaw"])
        else:
            pose = self.lookup_pose("map", "base_link")
            yaw = float(pose[2]) if pose is not None else 0.0
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        return msg

    def navigate(self, label: str, target: dict[str, float], timeout: float = 240.0) -> bool:
        if self.goal_pub.get_subscription_count() < 1:
            print(f"[Nav2 실패] {CENTER_GOAL_TOPIC} 구독자가 없습니다.")
            return False
        self.nav_status = ""
        goal = self.make_goal(target)
        self.goal_pub.publish(goal)
        print(
            f"[Nav2 시작] {label}: "
            f"x={target['x']:.4f}, y={target['y']:.4f}"
            + (f", yaw={target['yaw']:.6f}" if "yaw" in target else ", final yaw 강제 없음")
        )
        active_seen = False
        end = None if timeout <= 0.0 else time.monotonic() + timeout
        last_print = 0.0
        while rclpy.ok() and (end is None or time.monotonic() < end):
            rclpy.spin_once(self, timeout_sec=0.1)
            status = self.nav_status
            if status.startswith("ACTIVE"):
                active_seen = True
            if active_seen and status == "SUCCEEDED":
                self.stop()
                pose = self.lookup_pose("map", "base_link")
                if pose:
                    print(f"[Nav2 완료] {label}: 현재=({pose[0]:.4f}, {pose[1]:.4f}, {pose[2]:.6f})")
                else:
                    print(f"[Nav2 완료] {label}")
                return True
            if status.startswith("FAILED"):
                self.stop()
                print(f"[Nav2 실패] {label}: {status}")
                return False
            now = time.monotonic()
            if now - last_print > 2.0:
                pose = self.lookup_pose("map", "base_link")
                if pose:
                    remain = math.hypot(float(target["x"]) - pose[0], float(target["y"]) - pose[1])
                    print(f"[Nav2 이동] {label}: 상태={status or 'WAIT'}, 남은 거리≈{remain:.2f}m")
                last_print = now
        self.stop()
        print(f"[Nav2 실패] {label} 제한시간 초과")
        return False

    def start_ocr_approach(self, timeout: float = 120.0) -> bool:
        """OCR verifies identity; paired ArUco supplies the bed centre for docking.

        This deliberately ignores OCR bbox_center_x/bbox_center_y.  After a
        VERIFIED identity, only the physical left/right ArUco pair is used to
        centre the AMR.  The already-proven 3.328 m forward leg is preserved.
        """
        if self.ocr_request_pub.get_subscription_count() < 1:
            print("[OCR 실패] /amr1/ocr/request 구독자가 없습니다.")
            return False
        if self.count_publishers(ARUCO_RESULT_TOPIC) < 1:
            print("[ArUco 실패] /amr1/aruco/result publisher가 없습니다.")
            return False

        request_id = f"kimseoul-{uuid.uuid4().hex[:10]}"
        self.ocr_payload = {}
        req = String()
        req.data = json.dumps(
            {
                "protocol_version": 1,
                "command": "VERIFY_AND_TRACK",
                "request_id": request_id,
                "amr": "amr1",
                "expected_name": PATIENT_NAME,
                "expected_birth_date": PATIENT_BIRTH_DATE,
                "frames_to_check": 2,
                "candidates": [{"name": PATIENT_NAME, "birth_date": PATIENT_BIRTH_DATE}],
                "timestamp": time.time(),
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.ocr_request_pub.publish(req)
        print(f"[OCR 신원확인] {PATIENT_NAME} {PATIENT_BIRTH_DATE} 확인 요청")

        verify_end = time.monotonic() + min(60.0, timeout)
        verified = False
        last_state = ""
        while rclpy.ok() and time.monotonic() < verify_end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if str(self.ocr_payload.get("request_id", "")) != request_id:
                continue
            state = str(self.ocr_payload.get("state", ""))
            if state and state != last_state:
                print(f"[OCR 상태] {state}")
                last_state = state
            if state in {"REJECTED", "ERROR"}:
                print(f"[OCR 실패] {self.ocr_payload.get('reason', state)}")
                self.stop()
                return False
            if state in {"VERIFIED", "TRACKING"} and bool(self.ocr_payload.get("verified", False)):
                if str(self.ocr_payload.get("selected_name", "")) != PATIENT_NAME:
                    print("[OCR 실패] 이름 불일치")
                    return False
                if str(self.ocr_payload.get("selected_birth_date", "")) != PATIENT_BIRTH_DATE:
                    print("[OCR 실패] 생년월일 불일치")
                    return False
                verified = True
                break
        if not verified:
            print("[OCR 실패] 신원확인 제한시간 초과")
            self.stop()
            return False

        print("[OCR 성공] 환자 신원 확인 완료 — 이제 OCR bbox는 위치 제어에 사용하지 않습니다.")
        control = String()
        control.data = json.dumps(
            {
                "protocol_version": 1,
                "action": "STOP_TRACKING",
                "request_id": request_id,
                "amr": "amr1",
                "reason": "identity verified; switch to paired ArUco centre",
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.ocr_control_pub.publish(control)

        expected_left, expected_right = ARUCO_PAIRS[PATIENT_NAME]
        print(
            f"[ArUco 중심정렬] {PATIENT_NAME} 침대 pair={expected_left}/{expected_right} "
            "두 마커 중점을 카메라 중심으로 맞춥니다."
        )
        stable = 0
        wrong_pair_count = 0
        last_msg_stamp = None
        last_direction = ""
        align_end = time.monotonic() + max(20.0, timeout - min(60.0, timeout))
        while rclpy.ok() and time.monotonic() < align_end:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            if not self.aruco_payload or now - self.aruco_received_at > ARUCO_RESULT_STALE_SEC:
                self.stop()
                continue
            stamp = self.aruco_payload.get("timestamp")
            pair = self.aruco_payload.get("pairs", {}).get(PATIENT_NAME)
            visible = self.aruco_payload.get("visible_ids", [])
            complete = [str(x) for x in self.aruco_payload.get("complete_patients", [])]
            if pair is None:
                self.stop()
                wrong = [name for name in complete if name != PATIENT_NAME]
                if wrong:
                    wrong_pair_count += 1
                    if wrong_pair_count >= 5:
                        print(f"[ArUco 실패] OCR={PATIENT_NAME}, 카메라에서 다른 침대 pair={wrong} 검출")
                        return False
                else:
                    wrong_pair_count = 0
                if stamp != last_msg_stamp:
                    print(f"\r[ArUco 대기] expected={expected_left}/{expected_right}, visible={visible}", end="", flush=True)
                    last_msg_stamp = stamp
                continue
            wrong_pair_count = 0

            try:
                error_px = float(pair["center_error_px"])
                image_width = float(self.aruco_payload["image_width"])
            except (KeyError, TypeError, ValueError):
                self.stop()
                continue
            is_new = stamp != last_msg_stamp
            if is_new:
                last_msg_stamp = stamp

            if abs(error_px) <= ARUCO_X_TOLERANCE_PX:
                self.stop()
                if is_new:
                    stable += 1
                    print(
                        f"\r[ArUco CENTER] error={error_px:+.1f}px "
                        f"stable={stable}/{ARUCO_STABLE_MESSAGES}",
                        end="",
                        flush=True,
                    )
                if stable >= ARUCO_STABLE_MESSAGES:
                    print("\n[ArUco 중심정렬 완료] 좌/우 마커 중점 = 침대 도킹 중심선")
                    break
                continue

            if is_new:
                stable = 0
            normalized = error_px / max(1.0, image_width * 0.5)
            cmd = Twist()
            if abs(error_px) > ARUCO_YAW_THRESHOLD_PX:
                raw_w = -ARUCO_YAW_KP * normalized
                raw_w = max(-ARUCO_MAX_YAW_RAD_S, min(ARUCO_MAX_YAW_RAD_S, raw_w))
                if 0.0 < abs(raw_w) < ARUCO_MIN_YAW_RAD_S:
                    raw_w = math.copysign(ARUCO_MIN_YAW_RAD_S, raw_w)
                cmd.angular.z = raw_w
                direction = "ROTATE_LEFT" if raw_w > 0 else "ROTATE_RIGHT"
            else:
                raw_y = -ARUCO_LATERAL_KP * normalized
                raw_y = max(-ARUCO_MAX_LATERAL_MPS, min(ARUCO_MAX_LATERAL_MPS, raw_y))
                cmd.linear.y = raw_y
                direction = "Q-equivalent" if raw_y > 0 else "E-equivalent"
            self.cmd_pub.publish(cmd)
            if is_new and direction != last_direction:
                print(
                    f"\n[ArUco ALIGN] {direction}: pair_center={float(pair['pair_center_x']):.1f}px "
                    f"screen_center={image_width*0.5:.1f}px error={error_px:+.1f}px"
                )
                last_direction = direction
        else:
            self.stop()
            print("\n[ArUco 실패] 중심정렬 제한시간 초과")
            return False

        self.stop()
        self.spin_for(0.25)
        print(f"[ArUco 기준 전진] 검증된 기존 거리 {APPROACH_DISTANCE_M:.3f}m 전진 시작")
        start_pose = self.lookup_pose("odom", "base_link")
        if start_pose is None:
            print("[ArUco 전진 실패] odom->base_link TF 없음")
            return False
        forward_end = time.monotonic() + 30.0
        last_print = 0.0
        cmd = Twist()
        cmd.linear.x = APPROACH_SPEED_MPS
        while rclpy.ok() and time.monotonic() < forward_end:
            rclpy.spin_once(self, timeout_sec=0.02)
            current = self.lookup_pose("odom", "base_link")
            if current is None:
                self.stop()
                continue
            moved = math.hypot(current[0] - start_pose[0], current[1] - start_pose[1])
            if moved >= APPROACH_DISTANCE_M:
                self.stop()
                print(f"\n[ArUco 자동 접근 완료] OCR 신원확인 + ArUco pair 중심정렬 + {moved:.3f}m 전진")
                return True
            self.cmd_pub.publish(cmd)
            now = time.monotonic()
            if now - last_print > 0.5:
                print(f"\r[ArUco 전진] {moved:.3f}/{APPROACH_DISTANCE_M:.3f}m", end="", flush=True)
                last_print = now
            time.sleep(0.03)
        self.stop()
        print("\n[ArUco 전진 실패] 제한시간 초과")
        return False

    def lock_bed(self) -> bool:
        if self.magnet_pub.get_subscription_count() < 1:
            print("[결합 실패] Isaac magnet command 구독자가 없습니다.")
            return False
        for attempt in range(1, 4):
            request_id = f"lock-{uuid.uuid4().hex[:10]}"
            self.magnet_payload = {}
            msg = String()
            msg.data = json.dumps(
                {"command": "C", "request_id": request_id},
                separators=(",", ":"),
            )
            self.magnet_pub.publish(msg)
            print(f"[C 결합 {attempt}/3] C 키와 동일한 LOCK 명령 전송")
            end = time.monotonic() + 3.0
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.magnet_payload.get("request_id") != request_id:
                    continue
                if bool(self.magnet_payload.get("success")):
                    print(f"[결합 성공] {self.magnet_payload.get('attached_bed', '')}")
                    print("[자동 리프트] R 상승과 동일하게 침대 캐스터를 바닥에서 분리합니다.")
                    print("[Nav2 footprint] AMR 정사각형 0.74m x 0.74m 적용, 침대 확장 footprint는 사용하지 않습니다.")
                    return True
                print(f"[결합 응답 실패] {self.magnet_payload.get('state', '')}")
                break
            time.sleep(0.3)
        print("[결합 실패 종료]")
        return False

    def forced_reverse(self, distance_m: float = REVERSE_DISTANCE_M) -> bool:
        start = self.lookup_pose("odom", "base_link")
        if start is None:
            print("[후진 실패] odom->base_link TF 없음")
            return False
        print(
            f"[강제 직선 후진] {distance_m:.3f}m, linear.x={-REVERSE_SPEED_MPS:.2f}, "
            "linear.y=0, angular.z=0"
        )
        cmd = Twist()
        cmd.linear.x = -REVERSE_SPEED_MPS
        last_print = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            current = self.lookup_pose("odom", "base_link")
            if current is None:
                self.cmd_pub.publish(Twist())
                continue
            moved = math.hypot(current[0] - start[0], current[1] - start[1])
            if moved >= distance_m:
                self.stop()
                print(f"[강제 후진 완료] 실제 이동={moved:.3f}m")
                return True
            self.cmd_pub.publish(cmd)
            now = time.monotonic()
            if now - last_print > 0.5:
                print(f"\r[강제 후진] {moved:.3f}/{distance_m:.3f}m", end="", flush=True)
                last_print = now
            time.sleep(0.03)
        self.stop()
        print("\n[강제 후진 중단] ROS 종료")
        return False

    def forced_forward(self, distance_m: float = REVERSE_DISTANCE_M) -> bool:
        start = self.lookup_pose("odom", "base_link")
        if start is None:
            print("[전진 실패] odom->base_link TF 없음")
            return False
        print(
            f"[강제 직선 전진] {distance_m:.3f}m, linear.x={REVERSE_SPEED_MPS:.2f}, "
            "linear.y=0, angular.z=0"
        )
        cmd = Twist()
        cmd.linear.x = REVERSE_SPEED_MPS
        last_print = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            current = self.lookup_pose("odom", "base_link")
            if current is None:
                self.cmd_pub.publish(Twist())
                continue
            moved = math.hypot(current[0] - start[0], current[1] - start[1])
            if moved >= distance_m:
                self.stop()
                print(f"[강제 전진 완료] 실제 이동={moved:.3f}m")
                return True
            self.cmd_pub.publish(cmd)
            now = time.monotonic()
            if now - last_print > 0.5:
                print(f"\r[강제 전진] {moved:.3f}/{distance_m:.3f}m", end="", flush=True)
                last_print = now
            time.sleep(0.03)
        self.stop()
        print("\n[강제 전진 중단] ROS 종료")
        return False

    def capture_map_pose(self, label: str) -> dict[str, float] | None:
        pose = self.lookup_pose("map", "base_link")
        if pose is None:
            print(f"[위치 저장 실패] {label}: map->base_link TF 없음")
            return None
        saved = {"x": pose[0], "y": pose[1], "yaw": pose[2]}
        print(
            f"[위치 저장] {label}: x={saved['x']:.4f}, y={saved['y']:.4f}, "
            f"yaw={saved['yaw']:.6f}"
        )
        return saved

    def switch_to_floor_map(self, floor: str, request_id: str) -> bool:
        floor = str(floor).strip().lower()
        if floor not in {"1f", "2f"}:
            reason = f"invalid floor map request: {floor}"
            print(f"[맵 전환 실패] {reason}")
            self._publish_map_ack(request_id, False, floor, reason)
            return False
        map_yaml = (
            Path(__file__).resolve().parent
            / f"ros2_ws/src/hospital_nav2/maps/hospital_map_{floor}.yaml"
        )
        if not map_yaml.is_file():
            reason = f"{floor.upper()} map missing: {map_yaml}"
            print(f"[맵 전환 실패] {reason}")
            self._publish_map_ack(request_id, False, floor, reason)
            return False
        if not self.map_client.wait_for_service(timeout_sec=10.0):
            reason = f"service unavailable: {MAP_LOAD_SERVICE}"
            print(f"[맵 전환 실패] {reason}")
            self._publish_map_ack(request_id, False, floor, reason)
            return False
        req = LoadMap.Request()
        req.map_url = str(map_yaml)
        future = self.map_client.call_async(req)
        end = time.monotonic() + 20.0
        while rclpy.ok() and time.monotonic() < end and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done() or future.result() is None:
            reason = f"{floor.upper()} LoadMap timeout/no response"
            print(f"[맵 전환 실패] {reason}")
            self._publish_map_ack(request_id, False, floor, reason)
            return False
        response = future.result()
        success = int(response.result) == int(getattr(response, "RESULT_SUCCESS", 0))
        reason = "" if success else f"LoadMap result={int(response.result)}"
        print(f"[맵 전환] -> {floor.upper()} {'완료' if success else '실패'}: {map_yaml}")
        self._publish_map_ack(request_id, success, floor, reason)
        return success

    def _publish_map_ack(
        self,
        request_id: str,
        success: bool,
        floor: str,
        reason: str = "",
    ) -> None:
        msg = String()
        msg.data = json.dumps({
            "request_id": request_id,
            "success": bool(success),
            "floor": floor,
            "reason": reason,
        }, ensure_ascii=False, separators=(",", ":"))
        self.elevator_map_ack_pub.publish(msg)

    def run_elevator_sequence(self, direction: str = "up", timeout: float = 240.0) -> bool:
        if self.elevator_arrival_pub.get_subscription_count() < 1:
            print("[엘리베이터 실패] Isaac /elevator/amr_arrived 구독자가 없습니다.")
            return False

        direction = str(direction).strip().lower()
        if direction not in {"up", "down"}:
            print(f"[엘리베이터 실패] invalid direction: {direction}")
            return False

        is_up = direction == "up"
        request_id = f"elevator-{direction}-{uuid.uuid4().hex[:10]}"
        self.elevator_payload = {}
        self.elevator_final = False
        command = {
            "command": "START_UP" if is_up else "START_DOWN",
            "request_id": request_id,
            "robot": "amr1",
            "floor_from": "1f" if is_up else "2f",
            "floor_to": "2f" if is_up else "1f",
        }
        if not is_up:
            command.update({
                "return_entry_distance_m": ELEVATOR_RETURN_REVERSE_DISTANCE_M,
                "return_entry_speed_mps": ELEVATOR_RETURN_REVERSE_SPEED_MPS,
                "return_exit_distance_m": ELEVATOR_RETURN_EXIT_DISTANCE_M,
                "return_exit_speed_mps": ELEVATOR_RETURN_EXIT_SPEED_MPS,
            })
        msg = String()
        msg.data = json.dumps(command, separators=(",", ":"))
        self.elevator_arrival_pub.publish(msg)
        if is_up:
            print("[엘리베이터 상승 시작] 기존 자동 탑승 -> 2F -> 기존 5m 하차")
        else:
            print(
                "[엘리베이터 하강 시작] 2F 시작점에서 10m 후진 탑승 -> "
                "1F 하강 -> 5m 전진 하차"
            )

        end = time.monotonic() + timeout
        last_state = ""
        map_switched = False
        sequence_started = False
        expected_floor = "2f" if is_up else "1f"
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            payload = self.elevator_payload
            payload_request = str(payload.get("request_id", ""))
            if payload_request and payload_request != request_id:
                continue
            state = str(payload.get("state", ""))
            if payload_request == request_id and state:
                sequence_started = True
            if state and state != last_state:
                print(f"[엘리베이터 상태] {state}")
                last_state = state
            if state == "MAP_SWITCH_REQUIRED" and not map_switched:
                requested_floor = str(payload.get("floor", expected_floor)).strip().lower() or expected_floor
                if not self.switch_to_floor_map(requested_floor, request_id):
                    return False
                map_switched = True
            if state == "FAILED":
                print(f"[엘리베이터 실패] {payload.get('reason', '')}")
                return False
            if sequence_started and (state == "COMPLETE" or self.elevator_final):
                if is_up:
                    print("[엘리베이터 상승 완료] 2층 맵 + 5m 하차 완료")
                else:
                    print("[엘리베이터 하강 완료] 1층 맵 + 4m 전진 하차 완료")
                return True
        print(f"[엘리베이터 실패] {direction} 자동 시퀀스 제한시간 초과")
        return False

    def run(self) -> bool:
        print("===== 김서울 침대 이송 최종 왕복 테스트 =====")
        print(
            "자동 시작 Pose: "
            f"({AMR1_INITIAL_POSE['x']}, {AMR1_INITIAL_POSE['y']}, {AMR1_INITIAL_POSE['yaw']})"
        )
        if not self.wait_for_system():
            return False
        if not self.navigate("김서울 OCR/ArUco 사전 위치", SEOUL_OCR_POSE):
            return False
        self.stop()
        print("[카메라 안정화] 정지 상태로 0.5초 대기")
        self.spin_for(0.5)
        if not self.start_ocr_approach():
            return False
        self.stop()
        self.spin_for(0.8)
        if not self.lock_bed():
            return False
        self.stop()
        print("[자동 리프트 대기] 목표 높이 0.035m 상승을 위해 3.5초 대기")
        self.spin_for(3.5)
        if not self.forced_reverse():
            return False
        self.stop()
        self.spin_for(1.5)

        room_return_staging = self.capture_map_pose("병실 복귀용 3m 전진 시작 위치")
        if room_return_staging is None:
            return False

        if not self.navigate("엘리베이터 앞(X/Y만, 방향 강제 없음)", ELEVATOR, timeout=0.0):
            return False
        self.stop()
        if not self.run_elevator_sequence("up"):
            return False
        self.stop()
        self.spin_for(1.5)

        elevator_2f_nav_start = self.capture_map_pose("2층 자율주행 시작 위치")
        if elevator_2f_nav_start is None:
            return False

        target_dist = math.hypot(MRI_2F["x"] - MRI_TARGET_2F["x"], MRI_2F["y"] - MRI_TARGET_2F["y"])
        print(
            f"[2F MRI 안전 이동] measured MRI target=({MRI_TARGET_2F['x']:.4f},{MRI_TARGET_2F['y']:.4f}), "
            f"Nav2 stop=({MRI_2F['x']:.4f},{MRI_2F['y']:.4f}), standoff={target_dist:.2f}m"
        )
        if not self.navigate("2층 MRI 안전 정지점", MRI_2F, timeout=0.0):
            return False
        self.stop()
        print("[MRI 안전 정지] MRI 기계 목표점으로 더 들어가지 않고 정지. 환자 Bed->MRI 완료 상태를 확인합니다.")
        if not self.wait_patient_state("MRI_BED", MRI_TRANSFER_TIMEOUT_S, "Bed -> MRI"):
            return False

        print(
            f"[MRI 이탈] X={MRI_CLEAR_2F['x']:.5f} 유지, "
            f"Y={MRI_2F['y']:.5f} -> {MRI_CLEAR_2F['y']:.5f} (+{MRI_CLEAR_DISTANCE_M:.1f}m)"
        )
        if not self.navigate("MRI에서 Y축 +11m 이탈", MRI_CLEAR_2F, timeout=0.0):
            return False
        self.stop()

        if not self.navigate("MRI실 2차 안전 복귀", MRI_2F, timeout=0.0):
            return False
        self.stop()
        print("[MRI 2차 안전 정지] MRI -> 운반 침대 환자 복귀 상태를 확인합니다.")
        if not self.wait_patient_state("TRANSPORT_BED", MRI_RETURN_TIMEOUT_S, "MRI -> Bed"):
            return False
        print("[MRI 왕복 확인 완료] 환자가 운반 침대로 돌아온 것을 확인하고 엘리베이터로 복귀합니다.")

        print("[복귀] 저장해 둔 2층 자율주행 시작 위치로 돌아갑니다.")
        if not self.navigate("2층 엘리베이터 복귀 시작점", elevator_2f_nav_start, timeout=0.0):
            return False
        self.stop()
        self.spin_for(1.0)

        if not self.run_elevator_sequence("down"):
            return False
        self.stop()
        self.spin_for(1.5)

        print("[1F 복귀] 처음 침대 결합 후 3m 후진이 끝났던 위치로 돌아갑니다.")
        if not self.navigate("병실 복귀 대기 위치", room_return_staging, timeout=0.0):
            return False
        self.stop()
        self.spin_for(0.8)
        if not self.forced_forward(REVERSE_DISTANCE_M):
            return False
        self.stop()
        print(
            "[전체 성공] 김서울 병실 -> 엘리베이터 -> MRI -> Y축 11m 이탈 -> "
            "MRI 재진입 3초 -> 엘리베이터 하강 -> 병실 복귀 완료"
        )
        return True



def select_patient(argument: str | None) -> bool:
    if argument is not None:
        return argument.strip() in {"1", PATIENT_NAME}
    print("\n===== 이송할 환자를 선택하세요 =====")
    print("1. 김서울")
    try:
        selected = input("번호 입력 (1): ").strip()
    except EOFError:
        return False
    return selected == "1"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("patient", nargs="?", help="1 또는 김서울")
    args = parser.parse_args()
    if not select_patient(args.patient):
        print("[종료] 현재 테스트는 1. 김서울만 지원합니다.")
        return 2

    rclpy.init()
    node = KimSeoulMission()
    try:
        return 0 if node.run() else 1
    except KeyboardInterrupt:
        print("\n[사용자 중단] AMR 정지")
        return 130
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
