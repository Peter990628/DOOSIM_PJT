"""DOOSIM GUI ROS 2 bridge.

실제 GUI 통합 경로:
- RX: /amr1/world_pose, /amr2/world_pose (std_msgs/String JSON)
- RETURN: /amr1/inspection/complete, /amr2/inspection/complete (std_srvs/Trigger)
- MOVE: 실제 04_run_ocr_mission_1.sh / 2.sh 프로세스를 GUI가 직접 실행
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path

from mission_runtime import MissionRuntime
from pose_config import POSE_SAMPLE_INTERVAL_SEC

WORLD_POSE_ACTIVE_TIMEOUT_SEC = float(os.getenv("GUI_WORLD_POSE_ACTIVE_TIMEOUT_SEC", "2.5"))
ROS_IMPORT_ERROR = None

try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
except Exception as exc:  # pragma: no cover
    rclpy = None
    ROS_IMPORT_ERROR = str(exc)


def robot_config():
    return {
        "AMR-01": {"pose_topic": os.getenv("GUI_AMR_01_POSE_TOPIC", "/amr1/world_pose"), "ros_id": "amr1"},
        "AMR-02": {"pose_topic": os.getenv("GUI_AMR_02_POSE_TOPIC", "/amr2/world_pose"), "ros_id": "amr2"},
    }


def world_pose_qos_profile():
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def parse_world_pose_payload(data):
    payload = json.loads(data) if isinstance(data, str) else dict(data)
    x = float(payload["x"]); y = float(payload["y"])
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("world_pose x/y는 유한한 숫자여야 합니다.")
    return {"x": x, "y": y, "yaw": float(payload.get("yaw", 0.0) or 0.0), **({"z": float(payload["z"])} if payload.get("z") is not None else {})}


class HospitalRosNode(Node):
    def __init__(self, service):
        super().__init__("doosim_gui_bridge")
        self.service = service
        self._lock = threading.RLock()
        self._pose_diag = {}
        self._return_clients = {}
        for amr_name, cfg in robot_config().items():
            topic = cfg["pose_topic"]
            self._pose_diag[amr_name] = {
                "topic": topic, "publisher_count": 0, "received_count": 0,
                "processed_count": 0, "error_count": 0, "last_error": None,
                "last_received_at": None, "age_sec": None, "active": False,
                "streaming_confirmed": False, "last_pose": None, "type_compatible": True,
            }
            self.create_subscription(String, topic, lambda msg, a=amr_name: self._on_pose(a, msg), world_pose_qos_profile())
            ros_id = cfg["ros_id"]
            self._return_clients[amr_name] = self.create_client(Trigger, f"/{ros_id}/inspection/complete")
        self.create_timer(0.5, self._refresh_diagnostics)

    def _on_pose(self, amr_name, msg):
        now = time.time()
        with self._lock:
            d = self._pose_diag[amr_name]
            d["received_count"] += 1; d["last_received_at"] = now
        try:
            pose = parse_world_pose_payload(msg.data)
            self.service.update_pose(amr_name, pose["x"], pose["y"], pose.get("yaw", 0.0))
            with self._lock:
                d = self._pose_diag[amr_name]
                d["processed_count"] += 1; d["last_pose"] = pose; d["last_error"] = None
                d["streaming_confirmed"] = d["processed_count"] >= 2
        except Exception as exc:
            with self._lock:
                d = self._pose_diag[amr_name]
                d["error_count"] += 1; d["last_error"] = str(exc)

    def _refresh_diagnostics(self):
        now = time.time()
        with self._lock:
            for amr_name, cfg in robot_config().items():
                d = self._pose_diag[amr_name]
                d["publisher_count"] = self.count_publishers(cfg["pose_topic"])
                last = d.get("last_received_at")
                d["age_sec"] = round(now-last, 3) if last else None
                d["active"] = bool(last and now-last <= WORLD_POSE_ACTIVE_TIMEOUT_SEC)

    def trigger_return(self, amr_name, timeout=3.0):
        client = self._return_clients.get(amr_name)
        if client is None:
            raise RuntimeError(f"지원하지 않는 AMR입니다: {amr_name}")
        if not client.wait_for_service(timeout_sec=timeout):
            ros_id = robot_config()[amr_name]["ros_id"]
            raise RuntimeError(f"/{ros_id}/inspection/complete 서비스가 없습니다. 실제 미션이 MRI 검사 대기 단계인지 확인하세요.")
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            if future.done():
                result = future.result()
                if result is None or not result.success:
                    raise RuntimeError(getattr(result, "message", "검사완료 서비스 호출 실패"))
                return getattr(result, "message", "accepted")
            time.sleep(0.02)
        raise RuntimeError("검사완료 서비스 응답 timeout")

    def diagnostics(self):
        self._refresh_diagnostics()
        with self._lock:
            return {name: dict(v) for name, v in self._pose_diag.items()}


class RosManager:
    def __init__(self, service):
        self.service = service
        self.node = None; self.executor = None; self.thread = None; self.error = None
        self.runtime = MissionRuntime(Path(__file__).resolve().parent, service)

    def start(self):
        if rclpy is None:
            self.error = f"ROS 2 Python 패키지를 불러오지 못했습니다: {ROS_IMPORT_ERROR}"
            return
        try:
            if not rclpy.ok(): rclpy.init(args=None)
            self.node = HospitalRosNode(self.service)
            self.executor = MultiThreadedExecutor(num_threads=4)
            self.executor.add_node(self.node)
            self.thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.thread.start()
        except Exception as exc:
            self.error = str(exc)

    def stop(self):
        self.runtime.stop_all()
        if self.executor: self.executor.shutdown()
        if self.node: self.node.destroy_node()
        if rclpy and rclpy.ok(): rclpy.shutdown()

    def is_scenario_mode(self):
        return True

    def navigate(self, amr_name, floor, room):
        # HospitalService의 기존 인터페이스 호환용. 실제 Nav2 goal은 mission process가 소유합니다.
        return True

    def start_mission(self, amr_name, patient_name):
        if not self.node:
            raise RuntimeError("ROS 2 GUI bridge가 연결되지 않았습니다.")
        pose = self.node.diagnostics().get(amr_name, {})
        if int(pose.get("publisher_count") or 0) < 1:
            raise RuntimeError(f"{pose.get('topic')} publisher가 없습니다. Isaac/ROS bridge 실행 상태를 확인하세요.")
        if not pose.get("active"):
            raise RuntimeError(f"{pose.get('topic')} 최신 world_pose를 아직 수신하지 못했습니다.")
        return self.runtime.start(amr_name, patient_name)

    def trigger_return(self, amr_name):
        if not self.node:
            raise RuntimeError("ROS 2 GUI bridge가 연결되지 않았습니다.")
        return self.node.trigger_return(amr_name)

    def status(self):
        pose = self.node.diagnostics() if self.node else {
            a: {"topic": c["pose_topic"], "publisher_count": 0, "received_count": 0, "processed_count": 0,
                "error_count": 0, "last_error": None, "last_received_at": None, "age_sec": None,
                "active": False, "streaming_confirmed": False, "last_pose": None, "type_compatible": True}
            for a,c in robot_config().items()
        }
        return {
            "enabled": True,
            "mode": "actual_mission",
            "connected": bool(self.node and not self.error),
            "ready": bool(self.node and not self.error),
            "error": self.error,
            "robots": robot_config(),
            "world_pose": pose,
            "mission_runtime": self.runtime.status(),
            "gui_rx_topics": [c["pose_topic"] for c in robot_config().values()],
            "gui_tx_services": [f"/{c['ros_id']}/inspection/complete" for c in robot_config().values()],
            "pose_sample_interval_sec": POSE_SAMPLE_INTERVAL_SEC,
        }
