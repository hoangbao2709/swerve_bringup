#!/usr/bin/env python3
"""Raw motion evaluator with ROS-time duration and command-to-contact tracing."""
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
WHEEL_RADIUS = 0.0675
MODULE_X = 0.300042


class Raw(Node):
    def __init__(self):
        super().__init__('raw_motion_evaluator')
        self.gt = self.odom = self.imu = self.local = None
        self.steer = self.drive = self.joints = None
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 20)
        self.create_subscription(Float64MultiArray, '/steering_controller/commands', self.steer_cb, 20)
        self.create_subscription(Float64MultiArray, '/drive_controller/commands', self.drive_cb, 20)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 20)
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.state = self.create_client(GetEntityState, '/get_entity_state')

    def odom_cb(self, msg):
        p = msg.pose.pose
        self.odom = {'x': p.position.x, 'y': p.position.y, 'yaw': yaw(p.orientation),
                     'vx': msg.twist.twist.linear.x, 'vy': msg.twist.twist.linear.y,
                     'wz': msg.twist.twist.angular.z}

    def imu_cb(self, msg):
        self.imu = {'wz': msg.angular_velocity.z, 'yaw': yaw(msg.orientation)}

    def steer_cb(self, msg): self.steer = list(msg.data)
    def drive_cb(self, msg): self.drive = list(msg.data)

    def joint_cb(self, msg):
        self.joints = {'position': dict(zip(msg.name, msg.position)),
                       'velocity': dict(zip(msg.name, msg.velocity))}

    def get_gt(self):
        if not self.state.service_is_ready(): return None
        req = GetEntityState.Request(); req.name = 'swerve_base'; req.reference_frame = 'world'
        future = self.state.call_async(req); deadline = time.monotonic() + 0.2
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.005)
        if not future.done() or not future.result().success: return None
        state = future.result().state; p = state.pose; t = state.twist
        return {'x': p.position.x, 'y': p.position.y, 'yaw': yaw(p.orientation),
                'vx_world': t.linear.x, 'vy_world': t.linear.y, 'wz': t.angular.z}

    def local_pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
            return {'x': t.translation.x, 'y': t.translation.y, 'yaw': yaw(t.rotation)}
        except TransformException: return None

    def snapshot(self): return self.get_gt(), self.odom, self.local_pose()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end: rclpy.spin_once(self, timeout_sec=0.01)

    def run(self, name, command, duration=8.0):
        self.spin_for(1.0); start = None; deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            self.spin_for(0.05); start = self.snapshot()
            if all(start) and self.get_clock().now().nanoseconds > 0: break
        if not start or not all(start): raise RuntimeError('missing GT/odom/local pose before raw motion')
        start_sim = self.get_clock().now().nanoseconds * 1e-9; start_wall = time.monotonic(); trace = []
        watchdog = start_wall + max(60.0, duration * 12.0)
        while time.monotonic() < watchdog:
            now_sim = self.get_clock().now().nanoseconds * 1e-9; elapsed_sim = now_sim - start_sim
            if elapsed_sim >= duration: break
            msg = Twist(); msg.linear.x, msg.linear.y, msg.angular.z = command; self.pub.publish(msg)
            self.spin_for(0.02); gt, odom, loc = self.snapshot()
            pos = self.joints.get('position', {}) if self.joints else {}; vel = self.joints.get('velocity', {}) if self.joints else {}
            trace.append({'case': name, 'sim_time': now_sim, 'elapsed_sim_time': elapsed_sim,
                'elapsed_wall_time': time.monotonic() - start_wall,
                'gt_x': gt['x'] if gt else None, 'gt_y': gt['y'] if gt else None, 'gt_yaw': gt['yaw'] if gt else None,
                'gt_vx_world': gt['vx_world'] if gt else None, 'gt_vy_world': gt['vy_world'] if gt else None, 'gt_wz': gt['wz'] if gt else None,
                'odom_x': odom['x'] if odom else None, 'odom_y': odom['y'] if odom else None, 'odom_yaw': odom['yaw'] if odom else None,
                'odom_vx': odom['vx'] if odom else None, 'odom_vy': odom['vy'] if odom else None, 'odom_wz': odom['wz'] if odom else None,
                'local_x': loc['x'] if loc else None, 'local_y': loc['y'] if loc else None, 'local_yaw': loc['yaw'] if loc else None,
                'cmd_x': command[0], 'cmd_y': command[1], 'cmd_yaw': command[2],
                'steer_cmd_front': self.steer[0] if self.steer and len(self.steer) > 0 else None,
                'steer_cmd_rear': self.steer[1] if self.steer and len(self.steer) > 1 else None,
                'drive_cmd_front': self.drive[0] if self.drive and len(self.drive) > 0 else None,
                'drive_cmd_rear': self.drive[1] if self.drive and len(self.drive) > 1 else None,
                'steer_pos_front': pos.get('steer_front_joint'), 'steer_pos_rear': pos.get('steer_rear_joint'),
                'wheel_vel_front': vel.get('wheel_front_drive_joint'), 'wheel_vel_rear': vel.get('wheel_rear_drive_joint'),
                'imu_wz': self.imu['wz'] if self.imu else None, 'imu_yaw': self.imu['yaw'] if self.imu else None})
        command_elapsed_sim = min(duration, max(0.0, self.get_clock().now().nanoseconds * 1e-9 - start_sim))
        self.pub.publish(Twist()); self.spin_for(0.5); finish = self.snapshot()
        elapsed_sim = command_elapsed_sim; elapsed_wall = time.monotonic() - start_wall

        def delta(a, b): return (a['x'] - b['x'], a['y'] - b['y'], wrap(a['yaw'] - b['yaw'])) if a and b else (None, None, None)
        gt_d, odom_d, local_d = delta(finish[0], start[0]), delta(finish[1], start[1]), delta(finish[2], start[2])
        expected = (command[0] * elapsed_sim, command[1] * elapsed_sim, command[2] * elapsed_sim)
        gt_disp = math.hypot(gt_d[0], gt_d[1]) if gt_d[0] is not None else None; expected_disp = math.hypot(expected[0], expected[1])
        active = trace[max(0, len(trace) * 3 // 4):]
        def mean(key):
            vals = [float(r[key]) for r in active if r.get(key) is not None and math.isfinite(float(r[key]))]
            return sum(vals) / len(vals) if vals else None
        expected_wheel = math.hypot(command[0], command[1]) / WHEEL_RADIUS if command[2] == 0 else abs(command[2] * MODULE_X) / WHEEL_RADIUS
        actual_wheel = (abs(mean('wheel_vel_front') or 0.0) + abs(mean('wheel_vel_rear') or 0.0)) / 2.0
        result = {'case': name, 'cmd_linear': math.hypot(command[0], command[1]), 'cmd_wz': command[2],
            'elapsed_sim_time': elapsed_sim, 'elapsed_wall_time': elapsed_wall, 'rtf': elapsed_sim / elapsed_wall if elapsed_wall else None,
            'gt_dx': gt_d[0], 'gt_dy': gt_d[1], 'gt_dyaw': gt_d[2], 'odom_dx': odom_d[0], 'odom_dy': odom_d[1], 'odom_dyaw': odom_d[2],
            'local_dx': local_d[0], 'local_dy': local_d[1], 'local_dyaw': local_d[2], 'expected_dx': expected[0], 'expected_dy': expected[1], 'expected_dyaw': expected[2],
            'actual_displacement': gt_disp, 'expected_displacement': expected_disp,
            'velocity_efficiency': gt_disp / expected_disp if expected_disp > 1e-9 else abs(gt_d[2] / expected[2]) if abs(expected[2]) > 1e-9 else None,
            'steady_gt_body_speed': math.hypot(mean('gt_vx_world') or 0.0, mean('gt_vy_world') or 0.0), 'steady_gt_wz': mean('gt_wz'),
            'steady_odom_speed': math.hypot(mean('odom_vx') or 0.0, mean('odom_vy') or 0.0), 'command_wheel_rad_s': expected_wheel,
            'actual_wheel_rad_s': actual_wheel, 'wheel_surface_m_s': actual_wheel * WHEEL_RADIUS,
            'traction_ratio': (mean('gt_vx_world') or 0.0) / (actual_wheel * WHEEL_RADIUS) if actual_wheel > 1e-9 else None,
            'steer_error': max(abs((mean('steer_pos_front') or 0.0) - (mean('steer_cmd_front') or 0.0)), abs((mean('steer_pos_rear') or 0.0) - (mean('steer_cmd_rear') or 0.0))),
            'direction_correct': bool(gt_d[0] * command[0] > 0 if abs(command[0]) > 0 else gt_d[1] * command[1] > 0 if abs(command[1]) > 0 else gt_d[2] * command[2] > 0)}
        return result, trace


def main():
    name = os.environ.get('ACCEPTANCE_CASE', 'X+')
    if name not in CASES: raise SystemExit(f'unknown raw case {name}')
    outdir = os.environ.get('ACCEPTANCE_CASE_DIR', 'artifacts/raw_motion'); os.makedirs(outdir, exist_ok=True)
    rclpy.init(); node = Raw()
    try: result, trace = node.run(name, CASES[name])
    finally: node.destroy_node(); rclpy.shutdown()
    fields = list(trace[0].keys()) if trace else list(result.keys())
    with open(os.path.join(outdir, 'raw_motion.csv'), 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows(trace)
    with open(os.path.join(outdir, 'raw_motion_result.csv'), 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=result.keys()); writer.writeheader(); writer.writerow(result)
    print(result, flush=True); return 0 if result['direction_correct'] else 1


if __name__ == '__main__': raise SystemExit(main())
