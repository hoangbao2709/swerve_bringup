#!/usr/bin/env python3
"""Check several ROS 2 lifecycle nodes in one finite DDS participant."""

from __future__ import annotations

import argparse
import time

import rclpy
from lifecycle_msgs.srv import GetState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class Probe(Node):
    def __init__(self, names: list[str]) -> None:
        super().__init__('lifecycle_probe')
        self._lifecycle_clients = {
            name: self.create_client(GetState, f'/{name.lstrip("/")}/get_state')
            for name in names
        }

    def run(self, timeout: float) -> dict[str, int | None]:
        deadline = time.monotonic() + timeout
        futures = {}
        states: dict[str, int | None] = {}
        while rclpy.ok() and time.monotonic() < deadline:
            for name, client in self._lifecycle_clients.items():
                if states.get(name) == 3:
                    continue
                future = futures.get(name)
                if future is not None and future.done():
                    try:
                        states[name] = int(future.result().current_state.id)
                    except Exception:
                        states[name] = None
                    futures.pop(name, None)
                    future = None
                if future is None and client.service_is_ready():
                    futures[name] = client.call_async(GetState.Request())
            if all(states.get(name) == 3 for name in self._lifecycle_clients):
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        for name in self._lifecycle_clients:
            if name not in states:
                states[name] = None
        return states


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=20.0)
    parser.add_argument('nodes', nargs='+')
    args = parser.parse_args()
    if args.timeout <= 0.0:
        parser.error('--timeout must be positive')

    rclpy.init()
    node = Probe(args.nodes)
    try:
        try:
            states = node.run(args.timeout)
        except ExternalShutdownException:
            states = {name: None for name in args.nodes}
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    passed = True
    for name in args.nodes:
        state = states.get(name)
        if state == 3:
            print(f'OK: /{name.lstrip("/")} active [3]')
        else:
            print(f'FAIL: /{name.lstrip("/")} lifecycle state={state}')
            passed = False
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
