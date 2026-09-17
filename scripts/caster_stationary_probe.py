#!/usr/bin/env python3
"""Five-sim-second no-command stability and caster telemetry probe."""
import argparse
import csv
import json
import math
import time

import rclpy
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import GetEntityState, GetJointProperties
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

WHEELS = ('wheel_front_drive_link', 'wheel_rear_drive_link', 'wheel_front_left_link',
          'wheel_front_right_link', 'wheel_rear_left_link', 'wheel_rear_right_link')
PASSIVE = tuple(f'wheel_{corner}_{axis}_joint' for corner in
                ('front_left', 'front_right', 'rear_left', 'rear_right') for axis in ('swivel', 'roll'))

def rpy(q):
    roll = math.atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y))
    pitch = math.asin(max(-1., min(1., 2*(q.w*q.y - q.z*q.x))))
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    return roll, pitch, yaw

class Probe(Node):
    def __init__(self):
        super().__init__('caster_stationary_probe', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.clock = []
        self.joints = {'position': {}, 'velocity': {}}
        self.contact = {name: (0, 0., 0.) for name in WHEELS}
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Clock, '/clock', self.clock_cb, qos)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 50)
        for name in WHEELS:
            self.create_subscription(ContactsState, f'/contact_load/{name}', lambda msg, n=name: self.contact_cb(n, msg), qos)
        self.entity = self.create_client(GetEntityState, '/get_entity_state')
        self.joint_services = [
            self.create_client(GetJointProperties, '/get_joint_properties'),
            self.create_client(GetJointProperties, '/gazebo/get_joint_properties')]

    def clock_cb(self, msg): self.clock.append(msg.clock.sec + msg.clock.nanosec * 1e-9); self.clock = self.clock[-20:]
    def joint_cb(self, msg): self.joints = {'position': dict(zip(msg.name, msg.position)), 'velocity': dict(zip(msg.name, msg.velocity))}
    def contact_cb(self, name, msg):
        normal = tangential = 0.
        for state in msg.states:
            force = state.total_wrench.force; normal += abs(force.z); tangential += math.hypot(force.x, force.y)
        self.contact[name] = (len(msg.states), normal, tangential)
    def now(self): return self.get_clock().now().nanoseconds * 1e-9
    def call(self, client, request, timeout=.15):
        if not client.service_is_ready(): return None
        future = client.call_async(request); end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end: rclpy.spin_once(self, timeout_sec=.002)
        return future.result() if future.done() else None
    def joint_properties(self, joint):
        request = GetJointProperties.Request(joint_name=joint)
        for service in self.joint_services:
            response = self.call(service, request, .08)
            if response is not None: return response
        return None
    def snapshot(self):
        entity = self.call(self.entity, GetEntityState.Request(name='swerve_base', reference_frame='world'))
        if not entity or not entity.success: return None
        pose, twist = entity.state.pose, entity.state.twist; roll, pitch, yaw = rpy(pose.orientation)
        row = {'sim_time': self.now(), 'gt_x': pose.position.x, 'gt_y': pose.position.y, 'gt_z': pose.position.z,
               'gt_roll': roll, 'gt_pitch': pitch, 'gt_yaw': yaw, 'gt_vx': twist.linear.x,
               'gt_vy': twist.linear.y, 'gt_wz': twist.angular.z}
        for wheel, (count, normal, tangential) in self.contact.items():
            row[f'{wheel}_contact_count'] = count; row[f'{wheel}_normal_metric'] = normal; row[f'{wheel}_tangential_metric'] = tangential
        for joint in ('steer_front_joint', 'steer_rear_joint', 'wheel_front_drive_joint', 'wheel_rear_drive_joint'):
            row[f'{joint}_position'] = self.joints['position'].get(joint); row[f'{joint}_velocity'] = self.joints['velocity'].get(joint)
        for joint in PASSIVE:
            response = self.joint_properties(joint)
            row[f'{joint}_position'] = response.position[0] if response and response.success and response.position else None
            row[f'{joint}_velocity'] = response.rate[0] if response and response.success and response.rate else None
        return row

def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--output', default='artifacts/proper_caster_stationary'); parser.add_argument('--duration', type=float, default=5.0)
    args = parser.parse_args(); rclpy.init(); node = Probe(); rows = []
    try:
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline and (len(node.clock) < 3 or node.clock[-1] <= node.clock[0]): rclpy.spin_once(node, timeout_sec=.02)
        if len(node.clock) < 3: raise RuntimeError('/clock did not advance')
        start = node.now(); next_sample = start
        while node.now() - start < args.duration and time.monotonic() - deadline < 120:
            rclpy.spin_once(node, timeout_sec=.002)
            if node.now() >= next_sample:
                row = node.snapshot()
                if row: rows.append(row)
                next_sample += .04
    finally:
        node.destroy_node(); rclpy.shutdown()
    if len(rows) < 20: raise RuntimeError(f'insufficient samples: {len(rows)}')
    first, last = rows[0], rows[-1]
    ratios = {wheel: sum(r[f'{wheel}_contact_count'] > 0 for r in rows) / len(rows) for wheel in WHEELS}
    normals = {wheel: sum(r[f'{wheel}_normal_metric'] for r in rows) / len(rows) for wheel in WHEELS}
    drive_share = (normals[WHEELS[0]] + normals[WHEELS[1]]) / sum(normals.values()) if sum(normals.values()) else 0.
    summary = {'samples': len(rows), 'sim_duration': last['sim_time'] - first['sim_time'],
               'delta_z': last['gt_z'] - first['gt_z'], 'max_abs_roll': max(abs(r['gt_roll']) for r in rows),
               'max_abs_pitch': max(abs(r['gt_pitch']) for r in rows), 'xy_drift': math.hypot(last['gt_x']-first['gt_x'], last['gt_y']-first['gt_y']),
               'yaw_drift': last['gt_yaw'] - first['gt_yaw'], 'contact_ratio': ratios, 'mean_normal_metric': normals, 'drive_share': drive_share,
               'joint_states_contains_passive': {joint: joint in node.joints['position'] for joint in PASSIVE}}
    summary['pass'] = (abs(summary['delta_z']) <= .01 and summary['max_abs_roll'] <= .1 and summary['max_abs_pitch'] <= .1 and
                       summary['xy_drift'] <= .02 and all(value > 0 for value in ratios.values()) and drive_share > .01)
    with open(args.output + '.csv', 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys()); writer.writeheader(); writer.writerows(rows)
    with open(args.output + '.json', 'w', encoding='utf-8') as stream: json.dump(summary, stream, indent=2)
    print(json.dumps(summary, indent=2)); return 0 if summary['pass'] else 2
if __name__ == '__main__': raise SystemExit(main())
