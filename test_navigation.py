#!/usr/bin/env python3
"""Live A->B Nav2 acceptance test for the simulated/real swerve interface.

The test only prints PASS when the goal was accepted, the robot moved, Nav2
returned SUCCEEDED, and both final map-frame position and heading satisfy the
configured goal checker.  It also records the live goal-checker parameters and
the final command so a position-vs-heading completion failure is diagnosable.
"""
import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformException, TransformListener


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def normalize_angle(value):
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(value), math.cos(value))


def status_name(status):
    names = {
        GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
        GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
        GoalStatus.STATUS_EXECUTING: 'EXECUTING',
        GoalStatus.STATUS_CANCELING: 'CANCELING',
        GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
        GoalStatus.STATUS_CANCELED: 'CANCELED',
        GoalStatus.STATUS_ABORTED: 'ABORTED',
    }
    return names.get(status, 'STATUS_%s' % status)


class NavigationTest(Node):
    def __init__(self, distance_m, tolerance, timeout):
        super().__init__('navigation_live_test', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        if not self.get_parameter('use_sim_time').value:
            raise RuntimeError('FAIL: navigation test use_sim_time is false')
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
        self.cmd_vel_count = 0
        self.cmd_vel_nonzero_count = 0
        self.last_cmd_vel = None
        self.goal_checker_parameters = {}
        self.goal_checker_client = self.create_client(
            GetParameters, '/controller_server/get_parameters')
        self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 20)

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

    def cmd_vel_cb(self, msg):
        self.cmd_vel_count += 1
        linear = (float(msg.linear.x), float(msg.linear.y), float(msg.linear.z))
        angular = (float(msg.angular.x), float(msg.angular.y), float(msg.angular.z))
        self.last_cmd_vel = (linear, angular)
        if max(abs(value) for value in (*linear, *angular)) > 1.0e-4:
            self.cmd_vel_nonzero_count += 1

    def read_goal_checker_parameters(self, timeout=10.0):
        names = (
            'general_goal_checker.xy_goal_tolerance',
            'general_goal_checker.yaw_goal_tolerance',
            'general_goal_checker.stateful',
        )
        if not self.goal_checker_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('controller_server/get_parameters unavailable')
            return False
        request = GetParameters.Request()
        request.names = list(names)
        future = self.goal_checker_client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            time.sleep(0.05)
        if not future.done():
            self.get_logger().error('controller_server parameter query timed out')
            return False
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - middleware failure path
            self.get_logger().error('controller_server parameter query failed: %s' % exc)
            return False
        if response is None or len(response.values) != len(names):
            self.get_logger().error('controller_server returned incomplete goal checker parameters')
            return False
        for name, value in zip(names, response.values):
            if value.type == ParameterType.PARAMETER_DOUBLE:
                self.goal_checker_parameters[name] = float(value.double_value)
            elif value.type == ParameterType.PARAMETER_BOOL:
                self.goal_checker_parameters[name] = bool(value.bool_value)
            else:
                self.goal_checker_parameters[name] = None
        self.get_logger().info(
            'goal checker parameters: xy_goal_tolerance=%.6f, '
            'yaw_goal_tolerance=%.6f, stateful=%s' % (
                self.goal_checker_parameters.get(names[0], float('nan')),
                self.goal_checker_parameters.get(names[1], float('nan')),
                self.goal_checker_parameters.get(names[2], '<unset>'),
            ))
        return all(self.goal_checker_parameters.get(name) is not None for name in names)

    def wait_for_pose(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        current = None
        while time.monotonic() < deadline and rclpy.ok():
            current = self.pose()
            if current is not None:
                return current
            time.sleep(0.1)
        return current

    def feedback(self, message):
        self.last_feedback = float(message.feedback.distance_remaining)
        now = time.monotonic()
        if now - self.last_feedback_log >= 1.0:
            self.get_logger().info('distance_remaining=%.3f m' % self.last_feedback)
            self.last_feedback_log = now

    def send(self):
        if not self.client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError('/navigate_to_pose action server unavailable')
        self.read_goal_checker_parameters()
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
        self.get_logger().info(
            'start=(%.3f, %.3f, yaw=%.6f), goal=(%.3f, %.3f, yaw=%.6f)' %
            (x, y, heading, self.goal[0], self.goal[1], self.goal[2]))
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
        start_sim = self.get_clock().now().nanoseconds * 1e-9
        wall_deadline = time.monotonic() + max(300.0, self.timeout * 10.0)
        while (not result_future.done() and
               self.get_clock().now().nanoseconds * 1e-9 - start_sim < self.timeout and
               time.monotonic() < wall_deadline and rclpy.ok()):
            time.sleep(0.1)
        if not result_future.done():
            handle.cancel_goal_async()
            self.status = GoalStatus.STATUS_UNKNOWN
            self.get_logger().error('timeout after %.1f s' % self.timeout)
        else:
            self.status = result_future.result().status
            self.get_logger().info('action status=%d (%s)' %
                                   (self.status, status_name(self.status)))
        self.result_event = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--distance', type=float, default=1.0,
                        help='goal distance along current heading in metres')
    parser.add_argument('--tolerance', type=float, default=0.05,
                        help='maximum final XY error for acceptance in metres')
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
        final = node.wait_for_pose()
        xy_error = distance(final, node.goal) if final and node.goal else float('inf')
        yaw_error = (normalize_angle(final[2] - node.goal[2])
                     if final and node.goal else float('inf'))
        yaw_error_abs = abs(yaw_error)
        xy_goal_tolerance = node.goal_checker_parameters.get(
            'general_goal_checker.xy_goal_tolerance')
        yaw_goal_tolerance = node.goal_checker_parameters.get(
            'general_goal_checker.yaw_goal_tolerance')
        print('start pose: %s' % (node.start,))
        print('goal: %.3f %.3f' % node.goal[:2] if node.goal else 'goal: unavailable')
        print('goal yaw: %.6f rad' % node.goal[2] if node.goal else 'goal yaw: unavailable')
        print('final map->base_footprint pose: %s' % (final,))
        print('final map->base_footprint yaw: %.6f rad' % final[2]
              if final else 'final map->base_footprint yaw: unavailable')
        print('current XY error: %.6f m' % xy_error)
        print('final yaw error: %.6f rad (abs=%.6f rad)' % (yaw_error, yaw_error_abs))
        print('goal checker xy_goal_tolerance: %s' % xy_goal_tolerance)
        print('goal checker yaw_goal_tolerance: %s' % yaw_goal_tolerance)
        print('goal checker stateful: %s' %
              node.goal_checker_parameters.get('general_goal_checker.stateful'))
        print('goal accepted: %s' % node.accepted)
        print('robot moved (>0.10 m odom): %s' % node.moved)
        print('cmd_vel samples: %d (nonzero=%d)' %
              (node.cmd_vel_count, node.cmd_vel_nonzero_count))
        print('final /cmd_vel: %s' % (node.last_cmd_vel,))
        print('action status: %s (%s)' %
              (node.status, status_name(node.status) if node.status is not None else 'NONE'))
        yaw_configured = (isinstance(yaw_goal_tolerance, (float, int)) and
                          math.isfinite(yaw_goal_tolerance))
        passed = (node.accepted and node.moved and
                  node.status == GoalStatus.STATUS_SUCCEEDED and
                  xy_error <= args.tolerance and
                  yaw_configured and yaw_error_abs <= yaw_goal_tolerance)
        print('PASS' if passed else 'FAIL')
        return 0 if passed else 1
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
