#!/usr/bin/env python3
"""Receive one real sample from a finite set of ROS topics.

This is intentionally a single rclpy process. Spawning one ``ros2 topic
echo`` process per topic makes a slow Gazebo VM spend most of the probe window
on DDS discovery and can starve the simulator. The probe still requires a
callback for every requested topic; graph membership is never used as a
substitute for data.
"""

from __future__ import annotations

import argparse
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan, PointCloud2


TOPIC_TYPES = {
    '/clock': Clock,
    '/joint_states': JointState,
    '/odom': Odometry,
    '/odometry/filtered': Odometry,
    '/lidar/points': PointCloud2,
    '/lidar/points_filtered': PointCloud2,
    '/scan': LaserScan,
    '/map': OccupancyGrid,
}


def qos_for(topic: str) -> QoSProfile:
    if topic == '/map':
        return QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class Probe(Node):
    def __init__(self, topics: list[str]) -> None:
        super().__init__('ros_data_probe')
        self.seen = {topic: False for topic in topics}
        self._probe_subscriptions = []
        for topic in topics:
            message_type = TOPIC_TYPES[topic]
            self._probe_subscriptions.append(
                self.create_subscription(
                    message_type,
                    topic,
                    lambda _message, topic=topic: self.seen.__setitem__(topic, True),
                    qos_for(topic),
                )
            )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=15.0)
    parser.add_argument('topics', nargs='+', choices=sorted(TOPIC_TYPES))
    args = parser.parse_args()
    if args.timeout <= 0.0:
        parser.error('--timeout must be positive')

    rclpy.init()
    node = Probe(args.topics)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline and not all(node.seen.values()):
            rclpy.spin_once(node, timeout_sec=0.1)
    except ExternalShutdownException:
        # A caller may stop the finite probe while DDS is inside a wait set.
        # Keep the result deterministic and avoid a misleading traceback.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    passed = True
    for topic in args.topics:
        if node.seen[topic]:
            print(f'OK: {topic} message')
        else:
            print(f'FAIL: {topic} no message received')
            passed = False
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
