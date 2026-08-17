#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


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


class LifecycleBootstrap(Node):
    def __init__(self, target: str):
        safe = target.strip('/').replace('/', '_') or 'node'
        super().__init__(f'tray_lifecycle_bootstrap_{safe}')
        self.target = '/' + target.strip('/')
        self.get_cli = self.create_client(GetState, f'{self.target}/get_state')
        self.change_cli = self.create_client(ChangeState, f'{self.target}/change_state')

    def wait_services(self, deadline: float, watch_pid: int) -> bool:
        last = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            if watch_pid and not process_alive(watch_pid):
                print(f'[LIFECYCLE FAIL] watched process pid={watch_pid} exited before services for {self.target}')
                return False
            get_ok = self.get_cli.wait_for_service(timeout_sec=0.20)
            change_ok = self.change_cli.wait_for_service(timeout_sec=0.20)
            if get_ok and change_ok:
                return True
            now = time.monotonic()
            if now - last >= 2.0:
                print(f'[LIFECYCLE WAIT] node={self.target} get_state={get_ok} change_state={change_ok}')
                last = now
        return False

    def get_state_id(self, timeout: float = 2.0) -> tuple[int, str] | None:
        req = GetState.Request()
        fut = self.get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done() or fut.result() is None:
            return None
        st = fut.result().current_state
        return int(st.id), str(st.label)

    def transition(self, transition_id: int, label: str, timeout: float = 3.0) -> bool:
        req = ChangeState.Request()
        req.transition.id = int(transition_id)
        fut = self.change_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done() or fut.result() is None:
            print(f'[LIFECYCLE WARN] {self.target}: {label} service timeout')
            return False
        ok = bool(fut.result().success)
        print(f'[LIFECYCLE TRANSITION] node={self.target} action={label} success={ok}')
        return ok


def parse_args():
    p = argparse.ArgumentParser(
        description='Ensure a ROS2 lifecycle node reaches ACTIVE. Designed to recover map_server autostart races without editing the baseline launch.'
    )
    p.add_argument('--node', required=True, help='Fully qualified lifecycle node, e.g. /amr2/map_server')
    p.add_argument('--timeout', type=float, default=45.0)
    p.add_argument('--watch-pid', type=int, default=0)
    return p.parse_args()


def main() -> int:
    a = parse_args()
    rclpy.init(args=[])
    node = LifecycleBootstrap(a.node)
    deadline = time.monotonic() + max(1.0, a.timeout)
    last = 0.0
    try:
        if not node.wait_services(deadline, a.watch_pid):
            print(f'[LIFECYCLE TIMEOUT] services unavailable for {node.target}')
            return 2

        while rclpy.ok() and time.monotonic() < deadline:
            if a.watch_pid and not process_alive(a.watch_pid):
                print(f'[LIFECYCLE FAIL] watched process pid={a.watch_pid} exited while activating {node.target}')
                return 3

            state = node.get_state_id()
            if state is None:
                time.sleep(0.20)
                continue
            sid, label = state

            if sid == State.PRIMARY_STATE_ACTIVE:
                print(f'[LIFECYCLE ACTIVE] node={node.target} state={label}[{sid}]')
                return 0

            # The baseline lifecycle manager normally performs these transitions.
            # If DDS discovery/autostart races leave map_server unconfigured, recover it here.
            if sid == State.PRIMARY_STATE_UNCONFIGURED:
                node.transition(Transition.TRANSITION_CONFIGURE, 'configure')
                time.sleep(0.35)
                continue

            if sid == State.PRIMARY_STATE_INACTIVE:
                node.transition(Transition.TRANSITION_ACTIVATE, 'activate')
                time.sleep(0.35)
                continue

            # Transitional/final states: wait instead of issuing an invalid transition.
            now = time.monotonic()
            if now - last >= 1.5:
                print(f'[LIFECYCLE WAIT] node={node.target} state={label}[{sid}]')
                last = now
            time.sleep(0.20)

        state = node.get_state_id()
        print(f'[LIFECYCLE TIMEOUT] node={node.target} last_state={state}')
        return 4
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
