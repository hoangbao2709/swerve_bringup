#!/usr/bin/env python3
"""Low-rate, sim-time telemetry for the four passive casters during pure yaw.

Passive Gazebo joints are intentionally queried through GetJointProperties:
they are not exported by this robot's ros2_control joint-state interface.
"""
import argparse
import csv
import json
import math
import os
import time

import rclpy
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import GetEntityState, GetJointProperties
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

CASTERS = {
    'FL': ('wheel_front_left', 0.505, 0.195),
    'FR': ('wheel_front_right', 0.505, -0.195),
    'RL': ('wheel_rear_left', -0.505, 0.195),
    'RR': ('wheel_rear_right', -0.505, -0.195),
}
WHEEL_RADIUS = 0.0675
MODULE_X = 0.300042


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw(quaternion):
    return math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


def percentile(values, percentile_value):
    values = sorted(v for v in values if v is not None and math.isfinite(v))
    if not values:
        return None
    index = (len(values) - 1) * percentile_value / 100.0
    lo, hi = int(math.floor(index)), int(math.ceil(index))
    return values[lo] if lo == hi else values[lo] + (values[hi] - values[lo]) * (index - lo)


class CasterYawProbe(Node):
    def __init__(self):
        super().__init__('caster_yaw_probe', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.clock_samples = []
        self.joints = {'position': {}, 'velocity': {}}
        self.imu = None
        self.odom = None
        self.steer_command = self.drive_command = None
        self.contacts = {prefix: (0, 0.0, 0.0) for prefix, _, _ in CASTERS.values()}
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Clock, '/clock', self.clock_cb, qos)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 50)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 50)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 50)
        self.create_subscription(Float64MultiArray, '/steering_controller/commands', self.steer_cb, 20)
        self.create_subscription(Float64MultiArray, '/drive_controller/commands', self.drive_cb, 20)
        for prefix, _, _ in CASTERS.values():
            self.create_subscription(ContactsState, f'/contact_load/{prefix}_link',
                                     lambda message, p=prefix: self.contact_cb(p, message), qos)
        self.command = self.create_publisher(Twist, '/cmd_vel', 10)
        self.entity_client = self.create_client(GetEntityState, '/get_entity_state')
        # Gazebo Classic ROS 2 normally exposes the root endpoint; some
        # deployments retain the ROS 1-style /gazebo namespace.  Probe both
        # so a missing endpoint is never silently recorded as a zero joint.
        self.joint_clients = [
            ('/get_joint_properties', self.create_client(GetJointProperties, '/get_joint_properties')),
            ('/gazebo/get_joint_properties', self.create_client(GetJointProperties, '/gazebo/get_joint_properties')),
        ]
        self.joint_service_name = None

    def clock_cb(self, message):
        self.clock_samples.append(message.clock.sec + message.clock.nanosec * 1e-9)
        self.clock_samples = self.clock_samples[-20:]

    def joint_cb(self, message):
        self.joints = {'position': dict(zip(message.name, message.position)),
                       'velocity': dict(zip(message.name, message.velocity))}

    def imu_cb(self, message):
        self.imu = {'wz': message.angular_velocity.z, 'yaw': yaw(message.orientation)}

    def odom_cb(self, message):
        pose, twist = message.pose.pose, message.twist.twist
        self.odom = {'x': pose.position.x, 'y': pose.position.y, 'yaw': yaw(pose.orientation),
                     'vx': twist.linear.x, 'vy': twist.linear.y, 'wz': twist.angular.z}

    def steer_cb(self, message): self.steer_command = list(message.data)
    def drive_cb(self, message): self.drive_command = list(message.data)

    def contact_cb(self, prefix, message):
        normal = tangential = 0.0
        for state in message.states:
            force = state.total_wrench.force
            normal += abs(force.z)
            tangential += math.hypot(force.x, force.y)
        self.contacts[prefix] = (len(message.states), normal, tangential)

    def now(self): return self.get_clock().now().nanoseconds * 1e-9

    def call(self, client, request, timeout=0.12):
        if not client.service_is_ready():
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.002)
        return future.result() if future.done() else None

    def joint_properties(self, joint_name):
        request = GetJointProperties.Request(joint_name=joint_name)
        ordered = sorted(self.joint_clients, key=lambda item: item[0] != self.joint_service_name)
        for service_name, client in ordered:
            response = self.call(client, request)
            if response is not None:
                self.joint_service_name = service_name
                return response
        return None

    @staticmethod
    def encoder_wz(steer, wheel):
        # Same two-drive-module least-squares estimator used in raw_motion.py.
        rows = ((1., 0., 0.), (0., 1., MODULE_X), (1., 0., 0.), (0., 1., -MODULE_X))
        rhs = (WHEEL_RADIUS * wheel[0] * math.cos(steer[0]), WHEEL_RADIUS * wheel[0] * math.sin(steer[0]),
               WHEEL_RADIUS * wheel[1] * math.cos(steer[1]), WHEEL_RADIUS * wheel[1] * math.sin(steer[1]))
        normal, nrhs = [[0.] * 3 for _ in range(3)], [0.] * 3
        for row, value in zip(rows, rhs):
            for i in range(3):
                nrhs[i] += row[i] * value
                for j in range(3): normal[i][j] += row[i] * row[j]
        for column in range(3):
            pivot = max(range(column, 3), key=lambda row: abs(normal[row][column]))
            if abs(normal[pivot][column]) < 1e-12: return 0.0
            normal[column], normal[pivot] = normal[pivot], normal[column]
            nrhs[column], nrhs[pivot] = nrhs[pivot], nrhs[column]
            divisor = normal[column][column]
            normal[column] = [v / divisor for v in normal[column]]; nrhs[column] /= divisor
            for row in range(3):
                if row == column: continue
                factor = normal[row][column]
                normal[row] = [normal[row][j] - factor * normal[column][j] for j in range(3)]
                nrhs[row] -= factor * nrhs[column]
        return nrhs[2]

    def snapshot(self, case, command):
        response = self.call(self.entity_client, GetEntityState.Request(name='swerve_base', reference_frame='world'))
        if not response or not response.success:
            return None
        pose, twist = response.state.pose, response.state.twist
        body_yaw = yaw(pose.orientation)
        cosine, sine = math.cos(body_yaw), math.sin(body_yaw)
        # Entity twist is world-frame.  Desired caster heading is body-frame.
        body_vx = cosine * twist.linear.x + sine * twist.linear.y
        body_vy = -sine * twist.linear.x + cosine * twist.linear.y
        positions, velocities = self.joints['position'], self.joints['velocity']
        steer = (float(positions.get('steer_front_joint', 0.0)), float(positions.get('steer_rear_joint', 0.0)))
        wheel = (float(velocities.get('wheel_front_drive_joint', 0.0)), float(velocities.get('wheel_rear_drive_joint', 0.0)))
        row = {
            'case': case, 'sim_time': self.now(), 'cmd_vx': command[0], 'cmd_vy': command[1], 'cmd_wz': command[2],
            'gt_x': pose.position.x, 'gt_y': pose.position.y, 'gt_yaw': body_yaw,
            'gt_vx': body_vx, 'gt_vy': body_vy, 'gt_wz': twist.angular.z,
            'odom_x': self.odom['x'] if self.odom else None, 'odom_y': self.odom['y'] if self.odom else None,
            'odom_yaw': self.odom['yaw'] if self.odom else None, 'odom_vx': self.odom['vx'] if self.odom else None,
            'odom_vy': self.odom['vy'] if self.odom else None, 'odom_wz': self.odom['wz'] if self.odom else None,
            'imu_wz': self.imu['wz'] if self.imu else None,
            'steer_front_actual': steer[0], 'steer_rear_actual': steer[1],
            'wheel_front_actual': wheel[0], 'wheel_rear_actual': wheel[1],
            'encoder_wz': self.encoder_wz(steer, wheel),
            'steer_front_command': self.steer_command[0] if self.steer_command else None,
            'steer_rear_command': self.steer_command[1] if self.steer_command and len(self.steer_command) > 1 else None,
            'wheel_front_command': self.drive_command[0] if self.drive_command else None,
            'wheel_rear_command': self.drive_command[1] if self.drive_command and len(self.drive_command) > 1 else None,
        }
        for label, (prefix, x, y) in CASTERS.items():
            swivel = self.joint_properties(f'{prefix}_swivel_joint')
            roll = self.joint_properties(f'{prefix}_roll_joint')
            swivel_position = swivel.position[0] if swivel and swivel.success and swivel.position else None
            swivel_velocity = swivel.rate[0] if swivel and swivel.success and swivel.rate else None
            roll_velocity = roll.rate[0] if roll and roll.success and roll.rate else None
            desired = math.atan2(body_vy + twist.angular.z * x, body_vx - twist.angular.z * y) \
                if math.hypot(body_vx - twist.angular.z * y, body_vy + twist.angular.z * x) > 1e-6 else None
            # A rolling wheel can use heading theta or theta+pi.  wrap(2*d)/2
            # is its signed axis-equivalent error in [-pi/2, pi/2].
            error = wrap(2.0 * (swivel_position - desired)) / 2.0 if desired is not None and swivel_position is not None else None
            count, normal, tangential = self.contacts[prefix]
            row.update({
                f'{label}_swivel_position': swivel_position, f'{label}_swivel_velocity': swivel_velocity,
                f'{label}_roll_velocity': roll_velocity, f'{label}_desired_heading': desired,
                f'{label}_heading_error_signed': error, f'{label}_heading_error': abs(error) if error is not None else None,
                f'{label}_contact_count': count, f'{label}_normal_metric': normal, f'{label}_tangential_metric': tangential,
            })
        return row


def caster_summary(rows, label):
    prefix = label + '_'
    error = [r[prefix + 'heading_error'] for r in rows if r[prefix + 'heading_error'] is not None]
    swivel = [r[prefix + 'swivel_position'] for r in rows if r[prefix + 'swivel_position'] is not None]
    swvel = [abs(r[prefix + 'swivel_velocity']) for r in rows if r[prefix + 'swivel_velocity'] is not None]
    rollvel = [abs(r[prefix + 'roll_velocity']) for r in rows if r[prefix + 'roll_velocity'] is not None]
    tangential = [r[prefix + 'tangential_metric'] for r in rows if r[prefix + 'tangential_metric'] is not None]
    travel = sum(abs(wrap(after - before)) for before, after in zip(swivel, swivel[1:]))
    return {'initial_swivel': swivel[0] if swivel else None,
            'desired_heading': rows[-1].get(prefix + 'desired_heading'), 'actual_heading': swivel[-1] if swivel else None,
            'mean_equivalent_heading_error': sum(error) / len(error) if error else None,
            'p95_heading_error': percentile(error, 95), 'final_heading_error': error[-1] if error else None,
            'swivel_travel': travel, 'mean_abs_swivel_velocity': sum(swvel) / len(swvel) if swvel else None,
            'mean_abs_roll_velocity': sum(rollvel) / len(rollvel) if rollvel else None,
            'mean_tangential_metric': sum(tangential) / len(tangential) if tangential else None,
            'p95_tangential_metric': percentile(tangential, 95)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=('yaw+', 'yaw-'), default=os.environ.get('ACCEPTANCE_CASE'),
                        help='defaults to ACCEPTANCE_CASE for run_acceptance integration')
    parser.add_argument('--output-dir', default=os.environ.get('ACCEPTANCE_CASE_DIR'),
                        help='defaults to ACCEPTANCE_CASE_DIR for run_acceptance integration')
    parser.add_argument('--duration', type=float, default=8.0)
    parser.add_argument('--sample-period', type=float, default=0.15, help='simulation seconds; 5-10 Hz only')
    args = parser.parse_args()
    if args.case not in ('yaw+', 'yaw-'):
        parser.error('--case must be yaw+ or yaw-')
    if not args.output_dir:
        parser.error('--output-dir is required outside run_acceptance')
    command = (0.0, 0.0, 0.2 if args.case == 'yaw+' else -0.2)
    os.makedirs(args.output_dir, exist_ok=True)
    rclpy.init(); node = CasterYawProbe(); rows = []
    try:
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and (len(node.clock_samples) < 3 or node.clock_samples[-1] <= node.clock_samples[0]):
            rclpy.spin_once(node, timeout_sec=0.02)
        if len(node.clock_samples) < 3:
            raise RuntimeError('/clock did not advance')
        # Let state/contact subscriptions populate before the first sample.
        settle_end = time.monotonic() + 1.0
        while time.monotonic() < settle_end: rclpy.spin_once(node, timeout_sec=0.01)
        start_sim, next_sample = node.now(), node.now()
        watchdog = time.monotonic() + max(120.0, args.duration * 20.0)
        while node.now() - start_sim < args.duration and time.monotonic() < watchdog:
            message = Twist(); message.angular.z = command[2]; node.command.publish(message)
            rclpy.spin_once(node, timeout_sec=0.002)
            if node.now() >= next_sample:
                row = node.snapshot(args.case, command)
                if row:
                    row['elapsed_sim_time'] = row['sim_time'] - start_sim
                    rows.append(row)
                next_sample += args.sample_period
        node.command.publish(Twist())
    finally:
        node.destroy_node(); rclpy.shutdown()
    if len(rows) < 10:
        raise RuntimeError(f'insufficient caster samples: {len(rows)}')
    casters = {label: caster_summary(rows, label) for label in CASTERS}
    # Stall means the chassis is below 10% of requested yaw while commanded
    # drive-derived yaw remains materially active, sustained for three samples.
    stall = None
    for index in range(2, len(rows)):
        window = rows[index - 2:index + 1]
        if window[0]['elapsed_sim_time'] < 0.5:
            continue
        if all(abs(r['gt_wz']) <= abs(command[2]) * 0.10 and abs(r['encoder_wz']) >= abs(command[2]) * 0.10 for r in window):
            stall = window[0]
            break
    telemetry_valid = all(casters[label]['initial_swivel'] is not None for label in CASTERS)
    aligned_bad = all((casters[label]['mean_equivalent_heading_error'] or 0.0) >= 0.30 for label in CASTERS)
    swivel_still = all((casters[label]['swivel_travel'] or float('inf')) <= 0.20 for label in CASTERS)
    drive_active = max(abs(r['encoder_wz']) for r in rows) >= abs(command[2]) * 0.50
    if not telemetry_valid:
        blocker = 'insufficient_evidence'
    elif stall and aligned_bad and swivel_still and drive_active:
        blocker = True
    else:
        blocker = False
    summary = {
        'case': args.case, 'samples': len(rows), 'sim_duration': rows[-1]['elapsed_sim_time'],
        'clock_verified': True, 'telemetry_valid': telemetry_valid,
        'joint_properties_service': node.joint_service_name,
        'mean_gt_wz': sum(r['gt_wz'] for r in rows) / len(rows),
        'mean_encoder_wz': sum(r['encoder_wz'] for r in rows) / len(rows),
        'mean_odom_wz': sum(r['odom_wz'] for r in rows if r['odom_wz'] is not None) / max(1, sum(r['odom_wz'] is not None for r in rows)),
        'mean_imu_wz': sum(r['imu_wz'] for r in rows if r['imu_wz'] is not None) / max(1, sum(r['imu_wz'] is not None for r in rows)),
        'casters': casters,
        't_stall': stall['elapsed_sim_time'] if stall else None,
        'at_stall': {label: {'heading_error': stall.get(label + '_heading_error'),
                             'tangential_metric': stall.get(label + '_tangential_metric')} for label in CASTERS} if stall else None,
        'zero_trail_confirmed_as_blocker': blocker,
        'evidence': {'caster_heading_error_large': aligned_bad, 'caster_swivel_nearly_stationary': swivel_still,
                     'drive_wheels_active': drive_active, 'gt_stall_detected': stall is not None},
    }
    with open(os.path.join(args.output_dir, 'caster_alignment.csv'), 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys(), lineterminator='\n'); writer.writeheader(); writer.writerows(rows)
    with open(os.path.join(args.output_dir, 'caster_alignment_summary.json'), 'w', encoding='utf-8') as stream:
        json.dump(summary, stream, indent=2)
    print(json.dumps(summary, indent=2))
    return 0 if telemetry_valid else 2


if __name__ == '__main__':
    raise SystemExit(main())
