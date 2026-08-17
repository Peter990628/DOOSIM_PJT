#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String


def process_alive(pid: int) -> bool:
    if pid <= 0:
        return True
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


class Probe(Node):
    def __init__(self, topic: str):
        super().__init__("string_ready_probe")
        self.value = None
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, topic, self._cb, qos)

    def _cb(self, msg: String) -> None:
        self.value = str(msg.data)


def main() -> int:
    p = argparse.ArgumentParser(description="Wait for a real latched std_msgs/String message, without ros2 CLI graph polling.")
    p.add_argument("--topic", required=True)
    p.add_argument("--timeout", type=float, default=60.0)
    p.add_argument("--expect-prefix", default="", help="Optional required message prefix, e.g. READY")
    p.add_argument("--watch-pid", type=int, default=0, help="Fail immediately if the launched stack process exits")
    a = p.parse_args()

    rclpy.init(args=[])
    node = Probe(a.topic)
    deadline = time.monotonic() + max(0.1, a.timeout)
    last_print = 0.0
    rc = 2
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if a.watch_pid and not process_alive(a.watch_pid):
                print(f"[STRING FAIL] watched process pid={a.watch_pid} exited while waiting for {a.topic}")
                return 3
            rclpy.spin_once(node, timeout_sec=0.20)
            if node.value is not None:
                if not a.expect_prefix or node.value.startswith(a.expect_prefix):
                    print(f"[STRING READY] {a.topic}={node.value}")
                    rc = 0
                    break
                print(f"[STRING WAIT] {a.topic} got '{node.value}', waiting prefix '{a.expect_prefix}'")
            now = time.monotonic()
            if now - last_print >= 2.0:
                print(f"[STRING WAIT] topic={a.topic} rx={node.value is not None} process_alive={process_alive(a.watch_pid)}")
                last_print = now
        if rc:
            print(f"[STRING TIMEOUT] topic={a.topic} timeout={a.timeout:.1f}s last={node.value!r}")
        return rc
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
