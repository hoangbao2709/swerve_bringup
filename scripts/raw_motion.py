#!/usr/bin/env python3
"""Raw motion primitive evaluator; no Nav2 goals and no GT injection."""
import csv
import math
import os
import time

import rclpy
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformException, TransformListener


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


CASES = {
    'X+': (0.2, 0.0, 0.0), 'X-': (-0.2, 0.0, 0.0),
    'Y+': (0.0, 0.2, 0.0), 'Y-': (0.0, -0.2, 0.0),
    'yaw+': (0.0, 0.0, 0.2), 'yaw-': (0.0, 0.0, -0.2),
}


class Raw(Node):
    def __init__(self):
        super().__init__('raw_motion_evaluator')
        self.gt = self.odom = self.local = self.imu = None
        self.steer = self.drive = self.joints = self.last_stamp = None
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.create_subscription(Float64MultiArray, '/steering_controller/commands', lambda m: setattr(self, 'steer', list(m.data)), 10)
        self.create_subscription(Float64MultiArray, '/drive_controller/commands', lambda m: setattr(self, 'drive', list(m.data)), 10)
        self.create_subscription(JointState, '/joint_states', lambda m: setattr(self, 'joints', dict(zip(m.name, m.position))), 10)
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.state = self.create_client(GetEntityState, '/get_entity_state')

    def odom_cb(self, msg):
        p = msg.pose.pose; self.odom = (p.position.x, p.position.y, yaw(p.orientation))

    def imu_cb(self, msg):
        self.imu = (msg.angular_velocity.z,)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def get_gt(self):
        if not self.state.service_is_ready():
            return None
        req = GetEntityState.Request(); req.name = 'swerve_base'; req.reference_frame = 'world'
        future = self.state.call_async(req); deadline = time.monotonic() + 0.5
        while not future.done() and time.monotonic() < deadline: rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done() or not future.result().success: return None
        p = future.result().state.pose; return (p.position.x, p.position.y, yaw(p.orientation))

    def local_pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
            return (t.translation.x, t.translation.y, yaw(t.rotation))
        except TransformException: return None

    def run(self, name, command, duration=8.0):
        self.spin(1.0)
        start_gt = start_odom = start_local = None
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and (start_gt is None or start_odom is None or start_local is None):
            self.spin(0.1)
            start_gt, start_odom, start_local = self.get_gt(), self.odom, self.local_pose()
        trace = []; end = time.monotonic() + duration
        while time.monotonic() < end:
            msg = Twist(); msg.linear.x, msg.linear.y, msg.angular.z = command; self.pub.publish(msg)
            gt = self.get_gt(); loc = self.local_pose(); now = self.get_clock().now().nanoseconds * 1e-9
            trace.append({'case': name, 'sim_time': now, 'gt_x': gt[0] if gt else None, 'gt_y': gt[1] if gt else None, 'gt_yaw': gt[2] if gt else None,
                          'odom_x': self.odom[0] if self.odom else None, 'odom_y': self.odom[1] if self.odom else None, 'odom_yaw': self.odom[2] if self.odom else None,
                          'local_x': loc[0] if loc else None, 'local_y': loc[1] if loc else None, 'local_yaw': loc[2] if loc else None,
                          'cmd_x': command[0], 'cmd_y': command[1], 'cmd_yaw': command[2], 'steer': repr(self.steer), 'drive': repr(self.drive), 'imu_wz': self.imu[0] if self.imu else None})
            self.spin(0.08)
        self.pub.publish(Twist()); self.spin(0.5)
        finish_gt = finish_odom = finish_local = None
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and (finish_gt is None or finish_odom is None or finish_local is None):
            self.spin(0.1)
            finish_gt, finish_odom, finish_local = self.get_gt(), self.odom, self.local_pose()
        def delta(a, b): return (a[0] - b[0], a[1] - b[1], wrap(a[2] - b[2])) if a and b else (None, None, None)
        gt_d, odom_d, local_d = delta(finish_gt, start_gt), delta(finish_odom, start_odom), delta(finish_local, start_local)
        expected = (command[0] * duration, command[1] * duration, command[2] * duration)
        direction_correct = False
        if gt_d and all(value is not None for value in gt_d):
            if abs(command[0]) > 0:
                direction_correct = gt_d[0] > 0 if command[0] > 0 else gt_d[0] < 0
            elif abs(command[1]) > 0:
                direction_correct = gt_d[1] > 0 if command[1] > 0 else gt_d[1] < 0
            else:
                direction_correct = gt_d[2] > 0 if command[2] > 0 else gt_d[2] < 0
        result = {'case': name, 'duration_s': duration, 'gt_dx': gt_d[0], 'gt_dy': gt_d[1], 'gt_dyaw': gt_d[2], 'odom_dx': odom_d[0], 'odom_dy': odom_d[1], 'odom_dyaw': odom_d[2], 'local_dx': local_d[0], 'local_dy': local_d[1], 'local_dyaw': local_d[2], 'expected_dx': expected[0], 'expected_dy': expected[1], 'expected_dyaw': expected[2], 'direction_correct': direction_correct}
        return result, trace


def main():
    name = os.environ.get('ACCEPTANCE_CASE', os.environ.get('ACCEPTANCE_CASE_INDEX', 'X+'))
    if name not in CASES: raise SystemExit(f'unknown raw case {name}')
    outdir = os.environ.get('ACCEPTANCE_CASE_DIR', 'artifacts/raw_motion')
    os.makedirs(outdir, exist_ok=True)
    rclpy.init(); node = Raw()
    try: result, trace = node.run(name, CASES[name])
    finally: node.destroy_node(); rclpy.shutdown()
    with open(os.path.join(outdir, 'raw_motion.csv'), 'w', newline='', encoding='utf-8') as stream:
        fields = list(trace[0].keys()) if trace else list(result.keys()); writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows(trace)
    with open(os.path.join(outdir, 'raw_motion_result.csv'), 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=result.keys()); writer.writeheader(); writer.writerow(result)
    print(result, flush=True)
    return 0 if result['direction_correct'] else 1


if __name__ == '__main__': raise SystemExit(main())
