#!/usr/bin/env python3
"""Raw motion evaluator with ROS-time duration and command-to-contact tracing."""
import csv
import json
import math
import os
import time
import argparse

import rclpy
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

def roll(q): return math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
def pitch(q): return math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


CASES = {
    'X+': (0.2, 0.0, 0.0), 'X-': (-0.2, 0.0, 0.0),
    'Y+': (0.0, 0.2, 0.0), 'Y-': (0.0, -0.2, 0.0),
    'yaw+': (0.0, 0.0, 0.2), 'yaw-': (0.0, 0.0, -0.2),
}
WHEEL_RADIUS = 0.0675
MODULE_X = 0.300042
YAW_MIN_MEANINGFUL_RAD = 0.05
YAW_RATIO_TOLERANCE = 0.15


def finite_values(rows, key):
    return [float(row[key]) for row in rows
            if row.get(key) not in (None, '') and math.isfinite(float(row[key]))]


def trapezoid(rows, value_key):
    """Integrate a trace signal in simulation time, ignoring invalid spans."""
    total = 0.0
    for before, after in zip(rows, rows[1:]):
        try:
            dt = float(after['sim_time']) - float(before['sim_time'])
            a, b = float(before[value_key]), float(after[value_key])
        except (KeyError, TypeError, ValueError):
            continue
        if dt > 0.0 and math.isfinite(a) and math.isfinite(b):
            total += 0.5 * (a + b) * dt
    return total


def unwrapped_heading_delta(rows, heading_key):
    """Endpoint heading change without the +/-pi discontinuity.

    This is deliberately distinct from a rate integral: GT/odom heading is an
    orientation measurement, while encoder and IMU yaw are rate integrals.
    """
    values = finite_values(rows, heading_key)
    return sum(wrap(after - before) for before, after in zip(values, values[1:])) if len(values) > 1 else None


def yaw_report_from_trace(trace, command, clock_verified=True):
    """Produce named yaw metrics; used both live and to repair old artifacts."""
    active = trace[max(0, len(trace) * 3 // 4):]
    def mean(key):
        values = finite_values(active, key)
        return sum(values) / len(values) if values else None
    gt_yaw = unwrapped_heading_delta(trace, 'gt_yaw')
    odom_yaw = unwrapped_heading_delta(trace, 'odom_yaw')
    encoder_yaw = trapezoid(trace, 'encoder_wz')
    imu_yaw = trapezoid(trace, 'imu_wz')
    ratio = encoder_yaw / gt_yaw if gt_yaw is not None and abs(gt_yaw) > 1e-9 else None
    yaw_meaningful = gt_yaw is not None and abs(gt_yaw) >= YAW_MIN_MEANINGFUL_RAD
    direction = bool(gt_yaw is not None and gt_yaw * command[2] > 0.0)
    ratio_ok = ratio is not None and abs(ratio - 1.0) <= YAW_RATIO_TOLERANCE
    return {
        'steady_gt_wz': mean('gt_wz'),
        'steady_encoder_wz': mean('encoder_wz'),
        'steady_odom_wz': mean('odom_wz'),
        'steady_imu_wz': mean('imu_wz'),
        'integrated_gt_yaw': gt_yaw,
        'integrated_encoder_yaw': encoder_yaw,
        'integrated_odom_yaw': odom_yaw,
        'integrated_imu_yaw': imu_yaw,
        'encoder_to_gt_yaw_ratio': ratio,
        'odom_to_gt_yaw_ratio': odom_yaw / gt_yaw if gt_yaw is not None and abs(gt_yaw) > 1e-9 else None,
        'imu_to_gt_yaw_error': imu_yaw - gt_yaw if gt_yaw is not None else None,
        'gt_yaw_magnitude_meaningful': yaw_meaningful,
        'direction_correct': direction,
        'physics_acceptance_pass': bool(clock_verified and direction and yaw_meaningful and ratio_ok),
        'yaw_ratio_acceptance_tolerance': YAW_RATIO_TOLERANCE,
    }


class Raw(Node):
    def __init__(self):
        super().__init__('raw_motion_evaluator', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        if not self.get_parameter('use_sim_time').value:
            raise RuntimeError('FAIL: raw evaluator use_sim_time is false')
        self.gt = self.odom = self.imu = self.local = None
        self.steer = self.drive = self.joints = None
        self.clock_samples = []
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 20)
        self.create_subscription(Float64MultiArray, '/steering_controller/commands', self.steer_cb, 20)
        self.create_subscription(Float64MultiArray, '/drive_controller/commands', self.drive_cb, 20)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 20)
        clock_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Clock, '/clock', self.clock_cb, clock_qos)
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.state = self.create_client(GetEntityState, '/get_entity_state')

    def clock_cb(self, msg):
        value = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.clock_samples.append((value, time.monotonic()))
        self.clock_samples = self.clock_samples[-20:]

    def wait_for_sim_clock(self):
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if len(self.clock_samples) >= 3:
                first = self.clock_samples[0][0]
                if self.clock_samples[-1][0] > first and self.get_clock().now().nanoseconds > 0:
                    result = {'first_clock': first, 'second_clock': self.clock_samples[-1][0],
                            'clock_delta': self.clock_samples[-1][0] - first,
                            'wall_delta': self.clock_samples[-1][1] - self.clock_samples[0][1]}
                    self.get_logger().info('use_sim_time=true first_clock=%.9f second_clock=%.9f clock_delta=%.9f wall_delta=%.9f measured_RTF=%.6f' %
                                           (result['first_clock'], result['second_clock'], result['clock_delta'], result['wall_delta'],
                                            result['clock_delta'] / result['wall_delta'] if result['wall_delta'] else 0.0))
                    return result
        raise RuntimeError('FAIL: /clock did not advance while use_sim_time=true')

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
        return {'x': p.position.x, 'y': p.position.y, 'z': p.position.z,
                'roll': roll(p.orientation), 'pitch': pitch(p.orientation), 'yaw': yaw(p.orientation),
                'vx_world': t.linear.x, 'vy_world': t.linear.y, 'wz': t.angular.z}

    @staticmethod
    def encoder_body(steer, wheel):
        rows = ((1.0, 0.0, 0.0), (0.0, 1.0, MODULE_X),
                (1.0, 0.0, 0.0), (0.0, 1.0, -MODULE_X))
        rhs = (WHEEL_RADIUS * wheel[0] * math.cos(steer[0]),
               WHEEL_RADIUS * wheel[0] * math.sin(steer[0]),
               WHEEL_RADIUS * wheel[1] * math.cos(steer[1]),
               WHEEL_RADIUS * wheel[1] * math.sin(steer[1]))
        normal = [[0.0] * 3 for _ in range(3)]; nrhs = [0.0] * 3
        for row, value in zip(rows, rhs):
            for i in range(3):
                nrhs[i] += row[i] * value
                for j in range(3): normal[i][j] += row[i] * row[j]
        # Gaussian elimination for the tiny overdetermined least-squares system.
        for col in range(3):
            pivot = max(range(col, 3), key=lambda r: abs(normal[r][col]))
            if abs(normal[pivot][col]) < 1e-12: return (0.0, 0.0, 0.0)
            normal[col], normal[pivot] = normal[pivot], normal[col]; nrhs[col], nrhs[pivot] = nrhs[pivot], nrhs[col]
            d = normal[col][col]; normal[col] = [x / d for x in normal[col]]; nrhs[col] /= d
            for r in range(3):
                if r == col: continue
                k = normal[r][col]; normal[r] = [normal[r][j] - k * normal[col][j] for j in range(3)]; nrhs[r] -= k * nrhs[col]
        return tuple(nrhs)

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
        clock_validation = self.wait_for_sim_clock()
        self.spin_for(1.0); start = None; deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            self.spin_for(0.05); start = self.snapshot()
            if all(start) and self.get_clock().now().nanoseconds > 0: break
        if not start or not all(start): raise RuntimeError('missing GT/odom/local pose before raw motion')
        start_sim = self.get_clock().now().nanoseconds * 1e-9; start_wall = time.monotonic(); trace = []
        publish_times = []
        watchdog = start_wall + max(60.0, duration * 12.0)
        while time.monotonic() < watchdog:
            now_sim = self.get_clock().now().nanoseconds * 1e-9; elapsed_sim = now_sim - start_sim
            if elapsed_sim >= duration: break
            msg = Twist(); msg.linear.x, msg.linear.y, msg.angular.z = command; self.pub.publish(msg)
            publish_times.append(now_sim)
            self.spin_for(0.02); gt, odom, loc = self.snapshot()
            pos = self.joints.get('position', {}) if self.joints else {}; vel = self.joints.get('velocity', {}) if self.joints else {}
            actual_steer = (float(pos.get('steer_front_joint', 0.0)), float(pos.get('steer_rear_joint', 0.0)))
            actual_wheel = (float(vel.get('wheel_front_drive_joint', 0.0)), float(vel.get('wheel_rear_drive_joint', 0.0)))
            enc_vx, enc_vy, enc_wz = self.encoder_body(actual_steer, actual_wheel)
            trace.append({'case': name, 'sim_time': now_sim, 'elapsed_sim_time': elapsed_sim,
                'elapsed_wall_time': time.monotonic() - start_wall,
                'cmd_publish_gap_sim': now_sim - publish_times[-2] if len(publish_times) > 1 else None,
                'gt_x': gt['x'] if gt else None, 'gt_y': gt['y'] if gt else None, 'gt_z': gt['z'] if gt else None,
                'gt_roll': gt['roll'] if gt else None, 'gt_pitch': gt['pitch'] if gt else None, 'gt_yaw': gt['yaw'] if gt else None,
                'gt_vx_world': gt['vx_world'] if gt else None, 'gt_vy_world': gt['vy_world'] if gt else None, 'gt_wz': gt['wz'] if gt else None,
                'encoder_vx': enc_vx, 'encoder_vy': enc_vy, 'encoder_wz': enc_wz,
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
                'drive_error_front': (float(self.drive[0]) - actual_wheel[0]) if self.drive and len(self.drive) > 0 else None,
                'drive_error_rear': (float(self.drive[1]) - actual_wheel[1]) if self.drive and len(self.drive) > 1 else None,
                'imu_wz': self.imu['wz'] if self.imu else None, 'imu_yaw': self.imu['yaw'] if self.imu else None})
        command_elapsed_sim = min(duration, max(0.0, self.get_clock().now().nanoseconds * 1e-9 - start_sim))
        motion_finish = self.snapshot()
        self.pub.publish(Twist()); self.spin_for(0.5); finish = self.snapshot()
        elapsed_sim = command_elapsed_sim; elapsed_wall = time.monotonic() - start_wall

        def delta(a, b): return (a['x'] - b['x'], a['y'] - b['y'], wrap(a['yaw'] - b['yaw'])) if a and b else (None, None, None)
        gt_d, odom_d, local_d = delta(motion_finish[0], start[0]), delta(motion_finish[1], start[1]), delta(motion_finish[2], start[2])
        expected = (command[0] * elapsed_sim, command[1] * elapsed_sim, command[2] * elapsed_sim)
        gt_disp = math.hypot(gt_d[0], gt_d[1]) if gt_d[0] is not None else None; expected_disp = math.hypot(expected[0], expected[1])
        active = trace[max(0, len(trace) * 3 // 4):]
        gt_integral_x = gt_integral_y = 0.0
        for before, after in zip(trace, trace[1:]):
            dt = float(after['sim_time']) - float(before['sim_time'])
            if dt > 0 and before.get('gt_vx_world') is not None and after.get('gt_vx_world') is not None:
                gt_integral_x += 0.5 * (float(before['gt_vx_world']) + float(after['gt_vx_world'])) * dt
                gt_integral_y += 0.5 * (float(before['gt_vy_world']) + float(after['gt_vy_world'])) * dt
        gt_integral = math.hypot(gt_integral_x, gt_integral_y)
        publish_gaps = [b - a for a, b in zip(publish_times, publish_times[1:])]
        def mean(key):
            vals = [float(r[key]) for r in active if r.get(key) is not None and math.isfinite(float(r[key]))]
            return sum(vals) / len(vals) if vals else None
        expected_wheel = math.hypot(command[0], command[1]) / WHEEL_RADIUS if command[2] == 0 else abs(command[2] * MODULE_X) / WHEEL_RADIUS
        actual_wheel = (abs(mean('wheel_vel_front') or 0.0) + abs(mean('wheel_vel_rear') or 0.0)) / 2.0
        gt_cmd_speed = []
        for row in active:
            if row.get('gt_vx_world') is None or row.get('gt_vy_world') is None or row.get('gt_yaw') is None:
                continue
            c, s = math.cos(float(row['gt_yaw'])), math.sin(float(row['gt_yaw']))
            bx = c * float(row['gt_vx_world']) + s * float(row['gt_vy_world'])
            by = -s * float(row['gt_vx_world']) + c * float(row['gt_vy_world'])
            gt_cmd_speed.append(bx if abs(command[0]) >= abs(command[1]) else by)
        steady_gt_cmd_speed = sum(gt_cmd_speed) / len(gt_cmd_speed) if gt_cmd_speed else None
        result = {'case': name, 'cmd_linear': math.hypot(command[0], command[1]), 'cmd_wz': command[2],
            'elapsed_sim_time': elapsed_sim, 'elapsed_wall_time': elapsed_wall, 'rtf': elapsed_sim / elapsed_wall if elapsed_wall else None,
            'clock_first': clock_validation['first_clock'], 'clock_second': clock_validation['second_clock'],
            'clock_delta': clock_validation['clock_delta'], 'clock_wall_delta': clock_validation['wall_delta'],
            'gt_dx': gt_d[0], 'gt_dy': gt_d[1], 'gt_dyaw': gt_d[2], 'odom_dx': odom_d[0], 'odom_dy': odom_d[1], 'odom_dyaw': odom_d[2],
            'local_dx': local_d[0], 'local_dy': local_d[1], 'local_dyaw': local_d[2], 'expected_dx': expected[0], 'expected_dy': expected[1], 'expected_dyaw': expected[2],
            'actual_displacement': gt_disp, 'expected_displacement': expected_disp,
            'gt_velocity_integral': gt_integral, 'integration_error': abs(gt_disp - gt_integral),
            'velocity_efficiency': gt_disp / expected_disp if expected_disp > 1e-9 else abs(gt_d[2] / expected[2]) if abs(expected[2]) > 1e-9 else None,
            'steady_gt_body_speed': math.hypot(mean('gt_vx_world') or 0.0, mean('gt_vy_world') or 0.0),
            'steady_gt_cmd_speed': steady_gt_cmd_speed,
            'steady_odom_speed': math.hypot(mean('odom_vx') or 0.0, mean('odom_vy') or 0.0), 'command_wheel_rad_s': expected_wheel,
            'actual_wheel_rad_s': actual_wheel, 'wheel_surface_m_s': actual_wheel * WHEEL_RADIUS,
            'traction_ratio': steady_gt_cmd_speed / (actual_wheel * WHEEL_RADIUS) if actual_wheel > 1e-9 and steady_gt_cmd_speed is not None else None,
            'steer_error': max(abs((mean('steer_pos_front') or 0.0) - (mean('steer_cmd_front') or 0.0)), abs((mean('steer_pos_rear') or 0.0) - (mean('steer_cmd_rear') or 0.0))),
            'max_cmd_publish_gap_sim': max(publish_gaps) if publish_gaps else None,
            'mean_cmd_publish_gap_sim': sum(publish_gaps) / len(publish_gaps) if publish_gaps else None,
            'clock_verified': True}
        if abs(command[2]) > 0.0 and command[0] == 0.0 and command[1] == 0.0:
            # Do not infer a physics pass merely from the commanded sign.
            result.update(yaw_report_from_trace(trace, command, result['clock_verified']))
            # Preserve the old field as an endpoint diagnostic for comparison.
            result['gt_dyaw'] = result['integrated_gt_yaw']
        else:
            result['direction_correct'] = bool(gt_d[0] * command[0] > 0 if abs(command[0]) > 0 else gt_d[1] * command[1] > 0)
            result['physics_acceptance_pass'] = bool(result['clock_verified'] and result['direction_correct'])
        return result, trace


def postprocess(trace_path, result_path, source_result_path=None):
    """Backfill explicit yaw fields from an existing raw trace without a rerun."""
    with open(trace_path, newline='', encoding='utf-8') as stream:
        trace = list(csv.DictReader(stream))
    if not trace:
        raise RuntimeError(f'empty raw trace: {trace_path}')
    name = trace[0].get('case', 'yaw+')
    command = (float(trace[0].get('cmd_x') or 0.0), float(trace[0].get('cmd_y') or 0.0),
               float(trace[0].get('cmd_yaw') or 0.0))
    existing = {}
    source = source_result_path or result_path
    if os.path.exists(source):
        with open(source, newline='', encoding='utf-8') as stream:
            existing = next(csv.DictReader(stream), {})
    clock = str(existing.get('clock_verified', 'True')).lower() == 'true'
    result = dict(existing)
    result.update(yaw_report_from_trace(trace, command, clock))
    result['case'] = name
    # Use unwrapped trace heading rather than a manually interpreted CSV column.
    result['gt_dyaw'] = result['integrated_gt_yaw']
    with open(result_path, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=result.keys(), lineterminator='\n'); writer.writeheader(); writer.writerow(result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--postprocess', metavar='RAW_MOTION_CSV', help='backfill named yaw metrics from an existing trace')
    parser.add_argument('--result', help='output result CSV (defaults beside trace)')
    parser.add_argument('--source-result', help='existing result CSV to retain non-yaw fields from')
    args = parser.parse_args()
    if args.postprocess:
        result_path = args.result or os.path.join(os.path.dirname(args.postprocess), 'raw_motion_result.csv')
        result = postprocess(args.postprocess, result_path, args.source_result)
        print(result, flush=True)
        return 0 if result['physics_acceptance_pass'] else 1
    name = os.environ.get('ACCEPTANCE_CASE', 'X+')
    if name not in CASES: raise SystemExit(f'unknown raw case {name}')
    outdir = os.environ.get('ACCEPTANCE_CASE_DIR', 'artifacts/raw_motion'); os.makedirs(outdir, exist_ok=True)
    rclpy.init(); node = Raw()
    try: result, trace = node.run(name, CASES[name])
    finally: node.destroy_node(); rclpy.shutdown()
    fields = list(trace[0].keys()) if trace else list(result.keys())
    with open(os.path.join(outdir, 'raw_motion.csv'), 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator='\n'); writer.writeheader(); writer.writerows(trace)
    with open(os.path.join(outdir, 'raw_motion_result.csv'), 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=result.keys(), lineterminator='\n'); writer.writeheader(); writer.writerow(result)
    with open(os.path.join(outdir, 'clock_validation.json'), 'w', encoding='utf-8') as stream:
        json.dump({'use_sim_time': bool(node.get_parameter('use_sim_time').value),
                   'clock_topic_seen': bool(node.clock_samples),
                   'ros_clock_start': result['clock_first'], 'ros_clock_end': result['clock_second'],
                   'sim_elapsed': result['clock_delta'], 'wall_elapsed': result['clock_wall_delta'],
                   'rtf': result['clock_delta'] / result['clock_wall_delta'] if result['clock_wall_delta'] else None,
                   'max_cmd_publish_gap_sim': result['max_cmd_publish_gap_sim'],
                   'clock_verified': bool(node.get_parameter('use_sim_time').value and node.clock_samples)}, stream, indent=2)
    print(result, flush=True); return 0 if result['physics_acceptance_pass'] else 1


if __name__ == '__main__': raise SystemExit(main())
