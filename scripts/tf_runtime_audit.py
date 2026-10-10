#!/usr/bin/env python3
"""Capture TF publisher ownership, edge timestamps, and simulation-clock timing.

Run against an already-running isolated ROS domain. This observer never
publishes TF or robot commands. Outputs are written to a caller-provided
directory so generated audit artifacts do not enter the source tree.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor, await_or_execute
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


class PublisherInfoExecutor(SingleThreadedExecutor):
    """Pass DDS publisher identity through Humble's subscription message info."""

    def __init__(self, node, info_subscriptions):
        super().__init__()
        self.info_subscriptions = set(info_subscriptions)
        self.add_node(node)

    def _take_subscription(self, sub):
        with sub.handle:
            return sub.handle.take_message(sub.msg_type, sub.raw)

    async def _execute_subscription(self, sub, message_info):
        if message_info is None:
            return
        message, info = message_info
        if sub in self.info_subscriptions:
            await await_or_execute(sub.callback, message, info)
        else:
            await await_or_execute(sub.callback, message)


def ros_stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class TfAudit(Node):
    def __init__(self, *, duration_s: float, max_age_s: float):
        super().__init__('tf_runtime_audit', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time',
                rclpy.Parameter.Type.BOOL, True)])
        self.duration_s = duration_s
        self.max_age_s = max_age_s
        self.started_monotonic = time.monotonic()
        self.latest_clock_s = None
        self.clock_samples = []
        self.clock_resets = []
        self.message_info_identity_available = False
        self.transform_counts = {'dynamic': 0, 'static': 0}
        self.transforms = defaultdict(lambda: {
            'count': 0, 'publisher_gids': set(), 'first_wall_s': None,
            'last_wall_s': None, 'first_stamp_s': None, 'last_stamp_s': None,
            'age_min_s': None, 'age_max_s': None, 'stale_count': 0,
            'future_count': 0, 'sample_stamps_s': [],
        })

        clock_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        tf_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.VOLATILE)
        static_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Clock, '/clock', self._clock_cb, clock_qos)
        self.dynamic_sub = self.create_subscription(
            TFMessage, '/tf', lambda msg, info: self._tf_cb(msg, info, False), tf_qos)
        self.static_sub = self.create_subscription(
            TFMessage, '/tf_static', lambda msg, info: self._tf_cb(msg, info, True), static_qos)
        self.audit_executor = PublisherInfoExecutor(
            self, (self.dynamic_sub, self.static_sub))

    def _clock_cb(self, msg):
        now = ros_stamp_seconds(msg.clock)
        previous = self.latest_clock_s
        wall = time.monotonic()
        if previous is not None and now < previous - 0.5:
            self.clock_resets.append({
                'previous_sim_s': previous, 'current_sim_s': now,
                'backward_jump_s': previous - now,
                'wall_elapsed_s': wall - self.clock_samples[-1]['wall_monotonic_s'],
            })
        self.latest_clock_s = now
        if not self.clock_samples or wall - self.clock_samples[-1]['wall_monotonic_s'] >= 0.05:
            self.clock_samples.append({'sim_s': now, 'wall_monotonic_s': wall})

    def _tf_cb(self, msg, info, is_static):
        wall = time.monotonic()
        if isinstance(info, dict):
            publisher_gid = info.get('publisher_gid')
            source_timestamp_ns = int(info.get('source_timestamp', 0) or 0)
            received_timestamp_ns = int(info.get('received_timestamp', 0) or 0)
        else:
            publisher_gid = getattr(info, 'publisher_gid', None) if info is not None else None
            source_timestamp_ns = int(getattr(info, 'source_timestamp', 0) or 0)
            received_timestamp_ns = int(getattr(info, 'received_timestamp', 0) or 0)
        gid = bytes(publisher_gid).hex() if publisher_gid is not None else None
        self.message_info_identity_available = self.message_info_identity_available or gid is not None
        channel = 'static' if is_static else 'dynamic'
        for transform in msg.transforms:
            parent = str(transform.header.frame_id).lstrip('/')
            child = str(transform.child_frame_id).lstrip('/')
            stamp = ros_stamp_seconds(transform.header.stamp)
            key = (channel, parent, child, gid)
            item = self.transforms[key]
            item['count'] += 1
            item['publisher_gids'].add(gid)
            item['first_wall_s'] = wall if item['first_wall_s'] is None else item['first_wall_s']
            item['last_wall_s'] = wall
            item['first_stamp_s'] = stamp if item['first_stamp_s'] is None else item['first_stamp_s']
            item['last_stamp_s'] = stamp
            if source_timestamp_ns > 0:
                item.setdefault('source_timestamps_ns', []).append(source_timestamp_ns)
            if source_timestamp_ns > 0 and received_timestamp_ns >= source_timestamp_ns:
                latency_s = (received_timestamp_ns - source_timestamp_ns) * 1e-9
                item.setdefault('delivery_latency_s', []).append(latency_s)
            self.transform_counts[channel] += 1
            if not is_static and self.latest_clock_s is not None:
                age = self.latest_clock_s - stamp
                item['age_min_s'] = age if item['age_min_s'] is None else min(item['age_min_s'], age)
                item['age_max_s'] = age if item['age_max_s'] is None else max(item['age_max_s'], age)
                item['stale_count'] += age > self.max_age_s
                item['future_count'] += age < -0.05
                if len(item['sample_stamps_s']) < 10000:
                    item['sample_stamps_s'].append(stamp)

    def collect(self):
        deadline = self.started_monotonic + self.duration_s
        while time.monotonic() < deadline and rclpy.ok():
            self.audit_executor.spin_once(timeout_sec=min(0.05, deadline - time.monotonic()))

    def publisher_endpoints(self):
        output = {}
        for topic in ('/tf', '/tf_static', '/clock', '/odom', '/odometry/filtered',
                      '/map', '/navigation_map'):
            rows = []
            try:
                endpoints = self.get_publishers_info_by_topic(topic)
            except Exception:
                endpoints = []
            for endpoint in endpoints:
                rows.append({
                    'node_name': endpoint.node_name,
                    'node_namespace': endpoint.node_namespace,
                    'topic_type': endpoint.topic_type,
                    'endpoint_gid': bytes(endpoint.endpoint_gid).hex(),
                    'qos': {
                        'reliability': str(endpoint.qos_profile.reliability),
                        'durability': str(endpoint.qos_profile.durability),
                        'history': str(endpoint.qos_profile.history),
                        'depth': int(endpoint.qos_profile.depth),
                    },
                })
            output[topic] = rows
        return output

    def reports(self, *, mode: str, map_id: str | None, map_revision: str | None):
        elapsed = max(1e-9, time.monotonic() - self.started_monotonic)
        endpoints = self.publisher_endpoints()
        endpoint_names = {}
        for rows in endpoints.values():
            for endpoint in rows:
                endpoint_names[endpoint['endpoint_gid']] = (
                    endpoint['node_namespace'].rstrip('/') + '/' + endpoint['node_name'])

        edge_rows = []
        edge_owners = defaultdict(set)
        edge_gids = defaultdict(set)
        for (channel, parent, child, gid), item in self.transforms.items():
            node_name = endpoint_names.get(gid)
            edge = (channel, parent, child)
            edge_owners[edge].add(node_name or gid)
            edge_gids[edge].add(gid)
            wall_span = max(0.0, (item['last_wall_s'] or 0.0) - (item['first_wall_s'] or 0.0))
            stamp_deltas = [right - left for left, right in zip(
                item['sample_stamps_s'], item['sample_stamps_s'][1:]) if right > left]
            edge_rows.append({
                'channel': channel, 'parent_frame': parent, 'child_frame': child,
                'publisher_node': node_name, 'publisher_gid': gid,
                'transform_count': item['count'],
                'wall_rate_hz': (item['count'] - 1) / wall_span if wall_span > 0 else None,
                'sim_stamp_rate_hz': ((len(stamp_deltas) / (item['sample_stamps_s'][-1]
                                      - item['sample_stamps_s'][0]))
                                      if len(item['sample_stamps_s']) > 1
                                      and item['sample_stamps_s'][-1] > item['sample_stamps_s'][0]
                                      else None),
                'max_transform_interval_sim_s': max(stamp_deltas) if stamp_deltas else None,
                'first_stamp_sim_s': item['first_stamp_s'],
                'latest_stamp_sim_s': item['last_stamp_s'],
                'age_min_sim_s': item['age_min_s'],
                'age_max_sim_s': item['age_max_s'],
                'stale_samples_over_threshold': item['stale_count'],
                'future_samples_over_50ms': item['future_count'],
                'source_timestamp_samples': len(item.get('source_timestamps_ns', [])),
                'max_source_delivery_latency_s': max(item.get('delivery_latency_s', []), default=None),
            })
        duplicate_edges = []
        for edge, owners in edge_owners.items():
            if len(owners) > 1 or len(edge_gids[edge]) > 1:
                duplicate_edges.append({
                    'channel': edge[0], 'parent_frame': edge[1], 'child_frame': edge[2],
                    'publisher_nodes': sorted(str(value) for value in owners),
                    'publisher_endpoint_count': len(edge_gids[edge]),
                })

        source_commit = None
        try:
            source_commit = subprocess.run(
                ['git', 'rev-parse', 'HEAD'], check=True, capture_output=True,
                text=True, timeout=3).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            pass
        now = datetime.now(timezone.utc).isoformat()
        environment = {
            'captured_at_utc': now,
            'source_commit': source_commit,
            'working_tree': 'dirty' if subprocess.run(
                ['git', 'status', '--short'], capture_output=True, text=True,
                timeout=3).stdout.strip() else 'clean',
            'ros_distro': os.environ.get('ROS_DISTRO'),
            'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
            'use_sim_time': bool(self.get_clock().clock_type == rclpy.clock.ClockType.ROS_TIME),
            'runtime_mode': mode,
            'active_map_id': map_id,
            'active_map_revision': map_revision,
            'audit_window_wall_s': elapsed,
            'audit_window_start_sim_s': self.clock_samples[0]['sim_s'] if self.clock_samples else None,
            'audit_window_end_sim_s': self.latest_clock_s,
            'message_info_publisher_gid_available': self.message_info_identity_available,
        }
        expected_owner = 'slam_toolbox' if mode.lower() in ('mapping', 'unified') else 'ekf_v30e'
        architecture = {
            'environment': environment,
            'expected_runtime_contract': {
                'map_to_odom_owner': expected_owner,
                'odom_to_base_footprint_owner': 'ekf_filter_node',
                'swerve_odometry_publish_tf_expected': False,
                'frame_chain': ['map', 'odom', 'base_footprint'],
            },
            'observed_edges': edge_rows,
            'duplicate_dynamic_or_static_edges': duplicate_edges,
            'duplicate_edge_attribution_status': (
                'ATTRIBUTABLE' if self.message_info_identity_available
                else 'UNKNOWN_RCLPY_MESSAGE_INFO_DID_NOT_INCLUDE_PUBLISHER_GID'),
            'clock': {
                'sample_count': len(self.clock_samples),
                'first_sim_s': self.clock_samples[0]['sim_s'] if self.clock_samples else None,
                'last_sim_s': self.latest_clock_s,
                'sim_elapsed_s': (self.latest_clock_s - self.clock_samples[0]['sim_s']
                                  if self.clock_samples and self.latest_clock_s is not None else None),
                'backward_jumps_over_0_5s': self.clock_resets,
            },
        }
        timing = {
            'environment': environment,
            'max_tf_age_threshold_sim_s': self.max_age_s,
            'clock_samples': len(self.clock_samples),
            'backward_jumps_over_0_5s': self.clock_resets,
            'edge_timing': [row for row in edge_rows if row['channel'] == 'dynamic'],
            'transform_counts': self.transform_counts,
            'publisher_edge_identity_available': self.message_info_identity_available,
        }
        return architecture, endpoints, timing


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--duration', type=float, default=8.0)
    parser.add_argument('--max-age', type=float, default=0.5)
    parser.add_argument('--mode', default='unified')
    parser.add_argument('--map-id')
    parser.add_argument('--map-revision')
    args = parser.parse_args()
    if args.duration <= 0 or args.max_age <= 0:
        parser.error('--duration and --max-age must be positive')

    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = TfAudit(duration_s=args.duration, max_age_s=args.max_age)
    try:
        node.collect()
        architecture, publishers, timing = node.reports(
            mode=args.mode, map_id=args.map_id, map_revision=args.map_revision)
        for name, value in (
                ('tf_architecture.json', architecture),
                ('tf_publishers.json', {'environment': architecture['environment'],
                                        'publishers_by_topic': publishers}),
                ('tf_timing_report.json', timing)):
            (output_dir / name).write_text(json.dumps(value, indent=2, sort_keys=True) + '\n',
                                           encoding='utf-8')
        if not architecture['observed_edges']:
            status = 'FAIL_NO_TF_SAMPLES'
            exit_code = 2
        elif not architecture['environment']['message_info_publisher_gid_available']:
            # A successful topic-level discovery is not proof that observed
            # frame edges have unique publishers. Humble's Python MessageInfo
            # on this RMW omits publisher_gid, so report the attribution gate
            # as inconclusive instead of treating an empty duplicate list as
            # evidence of uniqueness.
            status = 'INCONCLUSIVE_PUBLISHER_ID_UNAVAILABLE'
            exit_code = 3
        elif architecture['duplicate_dynamic_or_static_edges']:
            status = 'FAIL_DUPLICATE_TF_EDGE_PUBLISHERS'
            exit_code = 4
        else:
            status = 'PASS'
            exit_code = 0
        print(json.dumps({
            'output_dir': str(output_dir),
            'dynamic_edges': [row for row in architecture['observed_edges']
                              if row['channel'] == 'dynamic'],
            'duplicates': architecture['duplicate_dynamic_or_static_edges'],
            'clock': architecture['clock'],
            'status': status,
        }, indent=2))
        return exit_code
    finally:
        node.audit_executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
