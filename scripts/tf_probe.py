#!/usr/bin/env python3
"""Resolve several TF pairs with one finite buffer and DDS participant."""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class Probe(Node):
    def __init__(self, pairs: list[tuple[str, str]]) -> None:
        super().__init__('tf_probe')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pairs = pairs

    def run(self, timeout: float) -> set[tuple[str, str]]:
        deadline = time.monotonic() + timeout
        resolved: set[tuple[str, str]] = set()
        while rclpy.ok() and time.monotonic() < deadline and len(resolved) < len(self.pairs):
            for target, source in self.pairs:
                if (target, source) in resolved:
                    continue
                try:
                    self.buffer.lookup_transform(target, source, rclpy.time.Time())
                    resolved.add((target, source))
                except (TransformException, RuntimeError):
                    pass
            rclpy.spin_once(self, timeout_sec=0.1)
        return resolved


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=15.0)
    parser.add_argument('frames', nargs='+', help='target/source frame pairs')
    args = parser.parse_args()
    if args.timeout <= 0.0:
        parser.error('--timeout must be positive')
    if len(args.frames) % 2:
        parser.error('frames must contain target/source pairs')
    pairs = list(zip(args.frames[::2], args.frames[1::2]))

    rclpy.init()
    node = Probe(pairs)
    try:
        try:
            resolved = node.run(args.timeout)
        except ExternalShutdownException:
            resolved = set()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    passed = True
    for target, source in pairs:
        if (target, source) in resolved:
            print(f'OK: {target} -> {source}')
        else:
            print(f'FAIL: {target} -> {source}')
            passed = False
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
