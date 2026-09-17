#!/usr/bin/env python3
"""Live A->B Nav2 acceptance test for the simulated/real swerve interface.

The test only prints PASS when the goal was accepted, the robot moved, Nav2
returned SUCCEEDED, and the final map-frame error is within tolerance.
"""
import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class NavigationTest(Node):
    def __init__(self, distance_m, tolerance, timeout):
        super().__init__('navigation_live_test')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.distance_m = distance_m
        self.tolerance = tolerance
        self.timeout = timeout
        self.start = None
        self.goal = None
        self.last_feedback = None
        self.last_feedback_log = 0.0
        self.moved = False
        self.accepted = False
        self.status = None
        self.result_event = False
        self.odom_start = None
        self.odom_last = None
        self.create_subscription(Odometry, '/odom', self.odom_cb, 20)

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
            return (t.translation.x, t.translation.y, yaw(t.rotation))
        except TransformException:
            return None

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.odom_last = (p.x, p.y)
        if self.odom_start is None:
            self.odom_start = self.odom_last
        if distance(self.odom_start, self.odom_last) > 0.10:
            self.moved = True

    def feedback(self, message):
        self.last_feedback = float(message.feedback.distance_remaining)
        now = time.monotonic()
        if now - self.last_feedback_log >= 1.0:
            self.get_logger().info('distance_remaining=%.3f m' % self.last_feedback)
            self.last_feedback_log = now

    def send(self):
        if not self.client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError('/navigate_to_pose action server unavailable')
        deadline = time.monotonic() + 15.0
        while self.start is None and time.monotonic() < deadline:
            self.start = self.pose()
            time.sleep(0.1)
        if self.start is None:
            raise RuntimeError('TF map->base_footprint unavailable')
        x, y, heading = self.start
        self.goal = (x + self.distance_m * math.cos(heading),
                     y + self.distance_m * math.sin(heading), heading)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = self.goal[:2]
        goal.pose.pose.orientation.z = math.sin(heading / 2.0)
        goal.pose.pose.orientation.w = math.cos(heading / 2.0)
        self.get_logger().info('start=(%.3f, %.3f), goal=(%.3f, %.3f)' %
                               (x, y, self.goal[0], self.goal[1]))
        future = self.client.send_goal_async(goal, feedback_callback=self.feedback)
        while not future.done() and rclpy.ok():
            time.sleep(0.05)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.accepted = False
            return
        self.accepted = True
        self.get_logger().info('goal ACCEPTED')
        result_future = handle.get_result_async()
        deadline = time.monotonic() + self.timeout
        while not result_future.done() and time.monotonic() < deadline and rclpy.ok():
            time.sleep(0.1)
        if not result_future.done():
            handle.cancel_goal_async()
            self.status = GoalStatus.STATUS_UNKNOWN
            self.get_logger().error('timeout after %.1f s' % self.timeout)
        else:
            self.status = result_future.result().status
            self.get_logger().info('action status=%d' % self.status)
        self.result_event = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--distance', type=float, default=1.0,
                        help='goal distance along current heading in metres')
    parser.add_argument('--tolerance', type=float, default=0.30)
    parser.add_argument('--timeout', type=float, default=120.0)
    args, ros_args = parser.parse_known_args()
    if not 1.0 <= args.distance <= 3.0:
        print('ERROR: --distance must be between 1 and 3 m', file=sys.stderr)
        return 2
    rclpy.init(args=ros_args)
    node = NavigationTest(args.distance, args.tolerance, args.timeout)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    import threading
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        node.start = node.pose()
        node.send()
        final = node.pose()
        error = distance(final, node.goal) if final and node.goal else float('inf')
        print('start pose: %s' % (node.start,))
        print('goal: %.3f %.3f' % node.goal[:2] if node.goal else 'goal: unavailable')
        print('goal accepted: %s' % node.accepted)
        print('robot moved (>0.10 m odom): %s' % node.moved)
        print('final error: %.3f m' % error)
        passed = (node.accepted and node.moved and
                  node.status == GoalStatus.STATUS_SUCCEEDED and error <= args.tolerance)
        print('PASS' if passed else 'FAIL')
        return 0 if passed else 1
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
