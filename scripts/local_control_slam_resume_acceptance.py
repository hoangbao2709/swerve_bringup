#!/usr/bin/env python3
"""Resume a saved SLAM session through the Web UI and prove it extends /map."""
from __future__ import annotations

import json
import hashlib
import base64
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import traceback
from collections import deque
from threading import RLock, Thread
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'waretwin' / 'backend'))

os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'config.settings')
import django
django.setup()

import rclpy
import websocket
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces
from gazebo_msgs.srv import GetPhysicsProperties
from geometry_msgs.msg import Twist
import end_to_end_acceptance as acceptance
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener
from twin.local_control import slam_map_restoration_evidence


def write_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2), encoding='utf-8')
    os.replace(temporary, path)


def stop_browser_process(process) -> None:
    """Stop only the isolated Playwright process group, with bounded waits."""
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=5)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError('isolated Playwright process group did not exit after SIGKILL') from exc


class ResumeReadinessNode(Node):
    """Post-restart ROS observer with its own sim-time and TF cache epoch."""

    def __init__(self):
        super().__init__('slam_resume_readiness_probe', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.tf = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf, self)


class ResumeReadinessGate:
    """Independent, continuously fresh ROS gate before resumed Web Teleop."""

    WINDOW_S = 5.0
    MAX_WALL_AGE_S = 1.0
    MAX_SIM_AGE_S = 1.0
    MAX_FUTURE_SKEW_S = 0.5
    MIN_RATE_HZ = 1.0
    MIN_RATE_HZ_BY_TOPIC = {
        'clock': 3.0,
        'model_states': 5.0,
        'joint_states': 5.0,
        'odom': 3.0,
        'scan': 1.0,
        'map': 0.2,
    }

    def __init__(self, node, robot_id: str):
        self.node = node
        self.robot_id = robot_id
        self.lock = RLock()
        self.samples = {key: deque(maxlen=1000) for key in (
            'clock', 'model_states', 'joint_states', 'odom', 'scan', 'map')}
        self.scan_tf_candidates = deque(maxlen=100)
        self.command_samples = {
            key: deque(maxlen=2000)
            for key in ('selected', 'owner', 'diagnostics')
        }
        self.model_pose_samples = deque(maxlen=1000)
        self.tf_edges = {}
        self.tf_edge_samples = {}
        self.tf_out_of_order_messages_ignored = 0
        self.latest_clock_s = 0.0
        self.latest_model_pose = None
        self.latest_joint_positions = {}
        self.latest_odom_pose = None
        self.latest_scan = None
        self.latest_map = None
        self.map_history = deque(maxlen=50)
        self.physics_response = None
        self.physics_response_wall = 0.0
        self.controllers_response = None
        self.controllers_response_wall = 0.0
        self.hardware_response = None
        self.hardware_response_wall = 0.0
        self.service_futures = {}
        self.next_service_poll = 0.0
        self.graph_snapshot = None
        self.graph_snapshot_wall = None
        self.graph_snapshot_complete = False
        self.graph_snapshot_duration_s = None
        self.physics_client = node.create_client(
            GetPhysicsProperties, '/gazebo/get_physics_properties')
        self.controllers_client = node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.hardware_client = node.create_client(
            ListHardwareInterfaces, '/controller_manager/list_hardware_interfaces')
        sensor_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions = [
            node.create_subscription(Clock, '/clock', self._clock_cb, sensor_qos),
            node.create_subscription(acceptance.ModelStates, '/model_states',
                                     self._model_cb, sensor_qos),
            node.create_subscription(JointState, '/joint_states', self._joint_cb, sensor_qos),
            node.create_subscription(Odometry, '/odom', self._odom_cb, sensor_qos),
            node.create_subscription(LaserScan, '/scan', self._scan_cb, sensor_qos),
            node.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos),
            node.create_subscription(TFMessage, '/tf', self._tf_cb, sensor_qos),
            node.create_subscription(TFMessage, '/tf_static', self._tf_static_cb,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                                durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            node.create_subscription(
                Twist, '/cmd_vel_selected',
                lambda msg: self._command_cb('selected', (
                    float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))), 50),
            node.create_subscription(
                String, '/command_owner',
                lambda msg: self._command_cb('owner', str(msg.data)), 50),
            node.create_subscription(
                String, '/command_arbiter/diagnostics',
                lambda msg: self._command_diagnostics_cb(msg.data), 10),
        ]

    def _command_cb(self, key, value):
        with self.lock:
            self.command_samples[key].append((time.monotonic(), value))

    def _command_diagnostics_cb(self, payload):
        try:
            value = json.loads(str(payload))
        except (ValueError, TypeError):
            value = {'invalid_json': True}
        self._command_cb('diagnostics', value)

    def _command_topic_status(self, key, now):
        with self.lock:
            age_reference = max(now, time.monotonic())
            rows = [row for row in self.command_samples[key]
                    if 0.0 <= age_reference - row[0] <= self.WINDOW_S]
        age = age_reference - rows[-1][0] if rows else None
        rate = ((len(rows) - 1) / (rows[-1][0] - rows[0][0])
                if len(rows) > 1 and rows[-1][0] > rows[0][0] else 0.0)
        return {
            'samples_5s': len(rows),
            'wall_age_s': age,
            'last_wall_received_monotonic_s': rows[-1][0] if rows else None,
            'rate_hz': rate,
            'latest': rows[-1][1] if rows else None,
        }

    def _command_samples_between(self, key, start, end):
        with self.lock:
            return [(wall, value) for wall, value in self.command_samples[key]
                    if start <= wall <= end]

    @staticmethod
    def _stamp(message):
        stamp = message.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _frame(value):
        return str(value or '').strip().lstrip('/')

    def _sample(self, key, stamp_s):
        received = time.monotonic()
        with self.lock:
            self.samples[key].append((received, float(stamp_s)))
        return received

    def _clock_cb(self, msg):
        stamp = msg.clock.sec + msg.clock.nanosec * 1e-9
        with self.lock:
            self.latest_clock_s = stamp
            self.samples['clock'].append((time.monotonic(), float(stamp)))

    def _model_cb(self, msg):
        try:
            index = msg.name.index(acceptance.EXPECTED_ROBOT_ENTITY)
            pose = msg.pose[index]
            model_pose = [pose.position.x, pose.position.y, pose.position.z]
            if all(math.isfinite(value) for value in model_pose):
                received = time.monotonic()
                sim_stamp = self.node.get_clock().now().nanoseconds * 1e-9
                with self.lock:
                    self.latest_model_pose = model_pose
                    self.model_pose_samples.append((received, sim_stamp, model_pose))
                    self.samples['model_states'].append((received, sim_stamp))
        except (ValueError, IndexError, AttributeError):
            return

    def _joint_cb(self, msg):
        names = ('steer_front_joint', 'steer_rear_joint',
                 'wheel_front_drive_joint', 'wheel_rear_drive_joint')
        positions = {name: float(msg.position[i]) for i, name in enumerate(msg.name)
                     if name in names and i < len(msg.position)
                     and math.isfinite(float(msg.position[i]))}
        if set(positions) == set(names):
            with self.lock:
                self.latest_joint_positions = positions
                self.samples['joint_states'].append((time.monotonic(), self._stamp(msg)))

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        odom_pose = [float(p.x), float(p.y), float(p.z)]
        if all(math.isfinite(value) for value in odom_pose):
            with self.lock:
                self.latest_odom_pose = odom_pose
                self.samples['odom'].append((time.monotonic(), self._stamp(msg)))

    def _scan_cb(self, msg):
        stamp = self._stamp(msg)
        received = time.monotonic()
        valid_ranges = sum(1 for value in msg.ranges
                           if math.isfinite(value) and msg.range_min <= value <= msg.range_max)
        latest_scan = {
            'frame_id': self._frame(msg.header.frame_id), 'stamp_s': stamp,
            'valid_ranges': valid_ranges, 'wall_received_monotonic_s': received,
        }
        if valid_ranges:
            with self.lock:
                self.latest_scan = latest_scan
                self.samples['scan'].append((received, stamp))
                self.scan_tf_candidates.append((received, latest_scan['frame_id'], stamp))

    def _map_cb(self, msg):
        width, height = int(msg.info.width), int(msg.info.height)
        if width <= 0 or height <= 0 or len(msg.data) != width * height:
            return
        occupancy = bytes(int(value) + 1 for value in msg.data)
        stamp = self._stamp(msg)
        known = sum(1 for value in msg.data if int(value) >= 0)
        summary = {
            'width': width, 'height': height,
            'resolution': float(msg.info.resolution),
            'origin': [float(msg.info.origin.position.x),
                       float(msg.info.origin.position.y)],
            'known_cells': known,
            'stamp_s': stamp,
            'signature': hashlib.sha256(occupancy).hexdigest(),
            'wall_received_monotonic_s': time.monotonic(),
        }
        with self.lock:
            self.latest_map = summary
            self.map_history.append(summary)
            self.samples['map'].append((summary['wall_received_monotonic_s'], stamp))

    def _tf_cb(self, msg):
        self._record_tf(msg, is_static=False)

    def _tf_static_cb(self, msg):
        self._record_tf(msg, is_static=True)

    def _record_tf(self, msg, is_static):
        wall = time.monotonic()
        with self.lock:
            for transform in msg.transforms:
                parent = self._frame(transform.header.frame_id)
                child = self._frame(transform.child_frame_id)
                stamp = self._stamp(transform)
                edge = (parent, child)
                previous = self.tf_edges.get(edge)
                if previous is not None:
                    # TF messages can arrive out of timestamp order. Keep the
                    # newest sample for each edge so a delayed packet cannot
                    # make a live transform look stale again.
                    if bool(previous['static']) != bool(is_static):
                        self.tf_out_of_order_messages_ignored += 1
                        continue
                    if not is_static and stamp < float(previous['stamp_s']):
                        self.tf_out_of_order_messages_ignored += 1
                        continue
                self.tf_edges[edge] = {
                    'stamp_s': stamp, 'wall_received_monotonic_s': wall,
                    'static': bool(is_static),
                }
                self.tf_edge_samples.setdefault(edge, deque(maxlen=100)).append(
                    (wall, stamp))

    def _send_service_requests(self):
        now = time.monotonic()
        if now < self.next_service_poll:
            return
        self.next_service_poll = now + 1.0
        clients = {
            'physics': (self.physics_client, GetPhysicsProperties.Request()),
            'controllers': (self.controllers_client, ListControllers.Request()),
            'hardware': (self.hardware_client, ListHardwareInterfaces.Request()),
        }
        for key, (client, request) in clients.items():
            future = self.service_futures.get(key)
            if future is not None and future.done():
                try:
                    response = future.result()
                    if key == 'physics':
                        self.physics_response = response
                        self.physics_response_wall = now
                    elif key == 'controllers':
                        self.controllers_response = response
                        self.controllers_response_wall = now
                    else:
                        self.hardware_response = response
                        self.hardware_response_wall = now
                except Exception:
                    pass
                self.service_futures.pop(key, None)
            if key not in self.service_futures and client.service_is_ready():
                self.service_futures[key] = client.call_async(request)

    def _topic_stats(self, key, now, sim_now, max_wall_age=None,
                     max_sim_age=None, min_rate_hz=None):
        with self.lock:
            rows = list(self.samples[key])
        if not rows:
            return {'rate_hz': None, 'wall_age_s': None, 'stamp_s': None,
                    'sim_age_s': None, 'stamp_increasing': False, 'samples_5s': 0}
        latest_wall, latest_stamp = rows[-1]
        recent = [row for row in rows if now - row[0] <= 5.0]
        if not recent:
            wall_age = max(0.0, now - latest_wall)
            sim_age = sim_now - latest_stamp
            return {
                'rate_hz': None, 'wall_age_s': wall_age, 'stamp_s': latest_stamp,
                'sim_age_s': sim_age, 'stamp_increasing': False,
                'stamp_span_s': 0.0, 'duplicate_stamp_pairs': 0,
                'out_of_order_stamp_pairs': 0, 'samples_5s': 0, 'fresh': False,
            }
        rate = ((len(recent) - 1) / (recent[-1][0] - recent[0][0])
                if len(recent) > 1 and recent[-1][0] > recent[0][0] else None)
        # Multiple publications can legitimately share one simulation stamp
        # when /clock is quantized below a sensor's publication rate. Require
        # a nondecreasing stream with actual progress, not a strict increase
        # on every adjacent message.
        increasing = (len(recent) > 1
                      and all(recent[i][1] >= recent[i - 1][1]
                              for i in range(1, len(recent)))
                      and recent[-1][1] > recent[0][1])
        duplicate_stamps = sum(
            1 for i in range(1, len(recent))
            if recent[i][1] == recent[i - 1][1])
        out_of_order_stamps = sum(
            1 for i in range(1, len(recent))
            if recent[i][1] < recent[i - 1][1])
        wall_age = max(0.0, now - latest_wall)
        sim_age = sim_now - latest_stamp
        max_wall_age = max_wall_age or self.MAX_WALL_AGE_S
        max_sim_age = max_sim_age or self.MAX_SIM_AGE_S
        min_rate_hz = (self.MIN_RATE_HZ_BY_TOPIC.get(key, self.MIN_RATE_HZ)
                       if min_rate_hz is None else min_rate_hz)
        return {
            'rate_hz': rate, 'wall_age_s': wall_age, 'stamp_s': latest_stamp,
            'sim_age_s': sim_age, 'stamp_increasing': increasing,
            'stamp_span_s': recent[-1][1] - recent[0][1],
            'duplicate_stamp_pairs': duplicate_stamps,
            'out_of_order_stamp_pairs': out_of_order_stamps,
            'samples_5s': len(recent),
            'fresh': (wall_age <= max_wall_age
                      and -self.MAX_FUTURE_SKEW_S <= sim_age <= max_sim_age
                      and rate is not None and rate >= min_rate_hz
                      and increasing),
        }

    def _transform_stats(self, edge, sim_now, now):
        with self.lock:
            value = self.tf_edges.get(edge)
            history = list(self.tf_edge_samples.get(edge, ()))
        if not value:
            return {'available': False, 'wall_age_s': None, 'stamp_s': None,
                    'sim_age_s': None, 'rate_hz': None, 'samples_5s': 0,
                    'fresh': False}
        wall_age = max(0.0, now - value['wall_received_monotonic_s'])
        stamp = float(value['stamp_s'])
        sim_age = sim_now - stamp if stamp > 0.0 else None
        static = bool(value['static'])
        recent = [row for row in history if now - row[0] <= self.WINDOW_S]
        rate = ((len(recent) - 1) / (recent[-1][0] - recent[0][0])
                if len(recent) > 1 and recent[-1][0] > recent[0][0] else None)
        fresh = (static or (wall_age <= self.MAX_WALL_AGE_S
                            and sim_age is not None
                            and -self.MAX_FUTURE_SKEW_S <= sim_age <= self.MAX_SIM_AGE_S))
        return {'available': True, 'static': static, 'wall_age_s': wall_age,
                'stamp_s': stamp, 'sim_age_s': sim_age, 'rate_hz': rate,
                'samples_5s': len(recent), 'fresh': fresh}

    def _lookup_stats(self, target, source, sim_now, query_stamp_s=None,
                      timeout_s=0.0):
        try:
            query_time = (Time(nanoseconds=max(1, int(float(query_stamp_s) * 1e9)),
                               clock_type=self.node.get_clock().clock_type)
                          if query_stamp_s is not None else Time())
            value = self.node.tf.lookup_transform(
                target, source, query_time, timeout=Duration(seconds=timeout_s))
            stamp = self._stamp(value)
            age = sim_now - stamp
            return {'available': True, 'stamp_s': stamp, 'sim_age_s': age,
                    'query_stamp_s': query_stamp_s,
                    'lookup_wait_s': timeout_s,
                    'fresh': stamp > 0.0
                    and -self.MAX_FUTURE_SKEW_S <= age <= self.MAX_SIM_AGE_S}
        except (TransformException, RuntimeError, ValueError) as exc:
            return {'available': False, 'stamp_s': None, 'sim_age_s': None,
                    'fresh': False, 'query_stamp_s': query_stamp_s,
                    'reason': f'{type(exc).__name__}: {exc}'}

    @staticmethod
    def _endpoint_rows(rows):
        return [{'node': row.node_name, 'namespace': row.node_namespace,
                 'topic_type': row.topic_type} for row in rows]

    def _graph(self):
        if self.graph_snapshot is not None:
            age = time.monotonic() - self.graph_snapshot_wall
            if age < 1.0 or (self.graph_snapshot_complete and age < 5.0):
                return self.graph_snapshot
        started = time.monotonic()
        nodes = {self._frame(name) for name, _namespace in
                 self.node.get_node_names_and_namespaces()}
        pubs = {}
        subs = {}
        topics = ('/clock', '/model_states', '/joint_states', '/odom', '/tf',
                  '/tf_static', '/scan', '/map', '/drive_controller/commands',
                  '/steering_controller/commands', '/cmd_vel_selected', '/command_owner',
                  '/command_arbiter/diagnostics')
        for topic in topics:
            try:
                pubs[topic] = self._endpoint_rows(self.node.get_publishers_info_by_topic(topic))
                subs[topic] = self._endpoint_rows(self.node.get_subscriptions_info_by_topic(topic))
            except Exception:
                pubs[topic], subs[topic] = [], []
        self.graph_snapshot = {'nodes': nodes, 'publishers': pubs, 'subscriptions': subs}
        self.graph_snapshot_wall = time.monotonic()
        self.graph_snapshot_duration_s = self.graph_snapshot_wall - started
        self.graph_snapshot_complete = (
            'slam_toolbox' in nodes
            and 'command_arbiter' in nodes
            and 'swerve_controller' in nodes
            and any(self._frame(row['node']) == 'slam_toolbox'
                    for row in pubs.get('/map', []))
            and any(self._frame(row['node']) == 'slam_toolbox'
                    for row in pubs.get('/tf', []))
            and any(self._frame(row['node']) == 'slam_toolbox'
                    for row in subs.get('/scan', []))
            and any(self._frame(row['node']) == 'swerve_controller'
                    for row in subs.get('/cmd_vel_selected', []))
            and any(self._frame(row['node']) == 'command_arbiter'
                    for row in pubs.get('/cmd_vel_selected', []))
            and any(self._frame(row['node']) == 'command_arbiter'
                    for row in pubs.get('/command_owner', []))
            and any(self._frame(row['node']) == 'command_arbiter'
                    for row in pubs.get('/command_arbiter/diagnostics', []))
            and any(self._frame(row['node']) == 'swerve_controller'
                    for row in pubs.get('/drive_controller/commands', []))
            and any(self._frame(row['node']) == 'swerve_controller'
                    for row in pubs.get('/steering_controller/commands', [])))
        return self.graph_snapshot

    def snapshot(self):
        self._send_service_requests()
        now = time.monotonic()
        with self.lock:
            clock_value = float(self.latest_clock_s)
            latest_scan = dict(self.latest_scan or {})
            scan_tf_candidates = list(self.scan_tf_candidates)
            latest_map = dict(self.latest_map or {})
            latest_model_pose = list(self.latest_model_pose or [])
            latest_joint_positions = dict(self.latest_joint_positions)
            latest_odom_pose = list(self.latest_odom_pose or [])
            clock_rows = [row for row in self.samples['clock']
                          if now - row[0] <= self.WINDOW_S]
            model_pose_rows = [row for row in self.model_pose_samples
                               if now - row[0] <= self.WINDOW_S]
        ros_clock_value = self.node.get_clock().now().nanoseconds * 1e-9
        sim_now = max(clock_value, ros_clock_value)
        stats = {key: self._topic_stats(
            key, now, sim_now,
            max_wall_age=3.5 if key == 'map' else None,
            max_sim_age=3.5 if key == 'map' else None,
            min_rate_hz=0.2 if key == 'map' else None)
                 for key in self.samples}
        clock_delta = (clock_rows[-1][1] - clock_rows[0][1]
                       if len(clock_rows) > 1 else 0.0)
        clock_wall_delta = (clock_rows[-1][0] - clock_rows[0][0]
                            if len(clock_rows) > 1 else 0.0)
        clock_hz = ((len(clock_rows) - 1) / clock_wall_delta
                    if len(clock_rows) > 1 and clock_wall_delta > 0 else None)
        rtf = clock_delta / clock_wall_delta if clock_wall_delta > 0 else None

        graph = self._graph()
        nodes = graph['nodes']
        pubs = graph['publishers']
        subs = graph['subscriptions']

        map_odom_tf = self._transform_stats(('map', 'odom'), sim_now, now)
        odom_base_tf = self._transform_stats(('odom', 'base_footprint'), sim_now, now)
        map_base_lookup = self._lookup_stats('map', 'base_footprint', sim_now)
        scan_frame = latest_scan.get('frame_id')
        lidar_lookup = {
            'available': False, 'stamp_s': None, 'sim_age_s': None,
            'fresh': False, 'query_stamp_s': latest_scan.get('stamp_s'),
            'reason': 'no exact recent scan-time transform found',
        }
        latest_scan_lookup = None
        if scan_frame:
            # Scan publication and dynamic base TF are asynchronous. Query exact
            # LaserScan stamps in a bounded recent window so the latest scan may
            # wait one TF publication cycle without treating a slightly older,
            # otherwise-valid scan as current sensor input. SCAN_FRESH below
            # independently requires the newest /scan sample itself to be live.
            for scan_received, candidate_frame, candidate_stamp in reversed(scan_tf_candidates):
                scan_wall_age = max(0.0, now - scan_received)
                if scan_wall_age > self.MAX_WALL_AGE_S:
                    break
                if candidate_frame != scan_frame:
                    continue
                candidate_lookup = self._lookup_stats(
                    'map', candidate_frame, sim_now, candidate_stamp)
                if latest_scan_lookup is None:
                    latest_scan_lookup = candidate_lookup
                if candidate_lookup.get('fresh'):
                    lidar_lookup = {
                        **candidate_lookup,
                        'query_frame': f'map -> {candidate_frame}',
                        'scan_frame': candidate_frame,
                        'scan_sample_wall_age_s': scan_wall_age,
                        'latest_scan_stamp_s': latest_scan.get('stamp_s'),
                        'latest_scan_lookup_fresh': bool(latest_scan_lookup.get('fresh')),
                    }
                    break
            else:
                if latest_scan_lookup is not None:
                    lidar_lookup = {
                        **latest_scan_lookup,
                        'query_frame': f'map -> {scan_frame}',
                        'scan_frame': scan_frame,
                        'scan_sample_wall_age_s': max(0.0, now - (
                            scan_tf_candidates[-1][0] if scan_tf_candidates else now)),
                        'latest_scan_stamp_s': latest_scan.get('stamp_s'),
                        'latest_scan_lookup_fresh': False,
                    }

        controllers = {}
        if self.controllers_response is not None:
            for row in self.controllers_response.controller:
                controllers[row.name] = {
                    'state': row.state,
                    'claimed_interfaces': list(row.claimed_interfaces),
                    'required_command_interfaces': list(row.required_command_interfaces),
                    'required_state_interfaces': list(row.required_state_interfaces),
                }
        expected_controllers = ('joint_state_broadcaster', 'steering_controller', 'drive_controller')
        controllers_active = (time.monotonic() - self.controllers_response_wall <= 3.0
                              and all(controllers.get(name, {}).get('state') == 'active'
                                      for name in expected_controllers))
        claimed = {name: bool(controllers.get(name, {}).get('claimed_interfaces'))
                   for name in ('steering_controller', 'drive_controller')}
        controllers_active = controllers_active and all(claimed.values())

        hardware = {}
        if self.hardware_response is not None:
            hardware = {
                'command_interfaces': [
                    {'name': row.name, 'available': row.is_available, 'claimed': row.is_claimed}
                    for row in self.hardware_response.command_interfaces],
                'state_interfaces': [
                    {'name': row.name, 'available': row.is_available, 'claimed': row.is_claimed}
                    for row in self.hardware_response.state_interfaces],
            }
        required_commands = {
            'steer_front_joint/position', 'steer_rear_joint/position',
            'wheel_front_drive_joint/velocity', 'wheel_rear_drive_joint/velocity',
        }
        hardware_command_rows = {
            row['name']: row for row in hardware.get('command_interfaces', [])
        }
        hardware_commands_ready = (
            time.monotonic() - self.hardware_response_wall <= 3.0
            and all(hardware_command_rows.get(name, {}).get('available')
                    and hardware_command_rows.get(name, {}).get('claimed')
                    for name in required_commands))
        controllers_active = controllers_active and hardware_commands_ready

        physics = None
        if self.physics_response is not None and time.monotonic() - self.physics_response_wall <= 3.0:
            physics = {
                'service': '/gazebo/get_physics_properties',
                'success': bool(self.physics_response.success),
                'paused': bool(self.physics_response.pause),
                'max_update_rate': float(self.physics_response.max_update_rate),
                'status': self.physics_response.status_message,
            }
        model_fresh = stats['model_states'].get('wall_age_s') is not None and \
            stats['model_states']['wall_age_s'] <= self.MAX_WALL_AGE_S and \
            (stats['model_states'].get('rate_hz') or 0.0) >= \
            self.MIN_RATE_HZ_BY_TOPIC['model_states']
        model_pose_span = None
        if len(model_pose_rows) >= 2:
            first, last = model_pose_rows[0], model_pose_rows[-1]
            model_pose_span = {
                'first_sim_s': first[1], 'last_sim_s': last[1],
                'sim_delta_s': max(0.0, last[1] - first[1]),
                'first_xyz_m': list(first[2]), 'last_xyz_m': list(last[2]),
                'xy_delta_m': math.hypot(last[2][0] - first[2][0],
                                         last[2][1] - first[2][1]),
                'samples': len(model_pose_rows),
            }
        physics_active = (stats['clock'].get('fresh', False)
                          and clock_delta > 0.20 and (rtf or 0.0) > 0.05 and model_fresh
                          and model_pose_span is not None
                          and model_pose_span['sim_delta_s'] > 0.20
                          and (physics is None or (physics['success'] and not physics['paused'])))

        # This lookup is made at the exact LaserScan stamp and verifies the
        # complete map -> LiDAR chain. The direct dynamic edges are gated
        # independently below so one bad edge report remains diagnosable.
        tf_lidar = lidar_lookup.get('fresh', False)
        scan_fresh = stats['scan'].get('fresh', False) and bool(scan_frame)
        slam_subscribers = {self._frame(row['node']) for row in subs.get('/scan', [])}
        map_publishers = {self._frame(row['node']) for row in pubs.get('/map', [])}
        tf_publishers = {self._frame(row['node']) for row in pubs.get('/tf', [])}
        incompatible = nodes.intersection({
            'map_server', 'lifecycle_manager_mapping_map', 'ekf_v30e',
            'v30e_sim_node', 'amcl', 'tag_route_planner',
        })
        command_topics = {
            key: self._command_topic_status(key, now)
            for key in ('selected', 'owner', 'diagnostics')
        }
        selected_publishers = {self._frame(row['node'])
                               for row in pubs.get('/cmd_vel_selected', [])}
        owner_publishers = {self._frame(row['node'])
                            for row in pubs.get('/command_owner', [])}
        diagnostics_publishers = {self._frame(row['node'])
                                  for row in pubs.get('/command_arbiter/diagnostics', [])}
        selected_fresh = (command_topics['selected']['wall_age_s'] is not None
                          and command_topics['selected']['wall_age_s'] <= 1.0
                          and (command_topics['selected']['rate_hz'] or 0.0)
                          >= self.MIN_RATE_HZ)
        owner_fresh = (command_topics['owner']['wall_age_s'] is not None
                       and command_topics['owner']['wall_age_s'] <= 1.0
                       and (command_topics['owner']['rate_hz'] or 0.0)
                       >= self.MIN_RATE_HZ)
        diagnostics = command_topics['diagnostics']['latest']
        diagnostics_fresh = (
            command_topics['diagnostics']['wall_age_s'] is not None
            and command_topics['diagnostics']['wall_age_s'] <= 1.0
            and (command_topics['diagnostics']['rate_hz'] or 0.0)
            >= self.MIN_RATE_HZ
            and isinstance(diagnostics, dict)
            and not diagnostics.get('invalid_json')
            and diagnostics.get('active_control_mode') in ('MANUAL', 'AUTONOMOUS')
            and not diagnostics.get('estop_active', True)
        )
        selected_values = command_topics['selected']['latest']
        selected_valid = (isinstance(selected_values, (list, tuple))
                          and len(selected_values) == 3
                          and all(math.isfinite(value) for value in selected_values))
        owner_valid = command_topics['owner']['latest'] in {
            'NONE', 'WEB_MANUAL', 'DIRECT_MANUAL', 'NAV2', 'TAG_ROUTE', 'ESTOP'}
        slam_active = ('slam_toolbox' in nodes and not incompatible
                       and map_publishers == {'slam_toolbox'}
                       and 'slam_toolbox' in tf_publishers)
        arbiter_ready = (
            'command_arbiter' in nodes
            and selected_publishers == {'command_arbiter'}
            and owner_publishers == {'command_arbiter'}
            and diagnostics_publishers == {'command_arbiter'}
            and owner_fresh and owner_valid and selected_fresh and selected_valid
            and diagnostics_fresh
        )
        swerve_ready = ('swerve_controller' in nodes
                        and any(self._frame(row['node']) == 'swerve_controller'
                                for row in subs.get('/cmd_vel_selected', []))
                        and any(self._frame(row['node']) == 'swerve_controller'
                                for row in pubs.get('/drive_controller/commands', []))
                        and any(self._frame(row['node']) == 'swerve_controller'
                                for row in pubs.get('/steering_controller/commands', [])))
        map_fresh = (stats['map'].get('fresh', False)
                     and (stats['map'].get('rate_hz') or 0.0) >= 0.2
                     and stats['map'].get('stamp_increasing', False))
        gates = {
            'CLOCK_FRESH': stats['clock'].get('fresh', False) and clock_delta > 0.20,
            'GAZEBO_PHYSICS_ACTIVE': physics_active,
            'CONTROLLERS_ACTIVE': controllers_active,
            'COMMAND_ARBITER_READY': arbiter_ready,
            'SWERVE_CONTROLLER_READY': swerve_ready,
            'JOINT_STATES_FRESH': stats['joint_states'].get('fresh', False),
            'ODOM_FRESH': stats['odom'].get('fresh', False),
            'ODOM_TF_FRESH': odom_base_tf.get('fresh', False),
            'MAP_ODOM_TF_FRESH': map_odom_tf.get('fresh', False),
            'MAP_BASE_TF_FRESH': map_base_lookup.get('fresh', False),
            'TF_LIDAR_FRESH': tf_lidar,
            'SCAN_FRESH': scan_fresh,
            'SLAM_ACTIVE': slam_active,
            'SLAM_INPUT_FRESH': scan_fresh and 'slam_toolbox' in slam_subscribers and tf_lidar,
            'MAP_LIVE': map_fresh and slam_active,
        }
        sources = {}
        topics = {
            'clock': '/clock', 'model_states': '/model_states',
            'joint_states': '/joint_states', 'odom': '/odom',
            'scan': '/scan', 'map': '/map',
        }
        for key, topic in topics.items():
            with self.lock:
                last_received = self.samples[key][-1][0] if self.samples[key] else None
            sources[topic] = {
                **stats[key], 'publishers': pubs.get(topic, []),
                'last_wall_received_monotonic_s': last_received,
            }
        tf_sources = {
            'odom->base_footprint': {
                **odom_base_tf, 'topic': '/tf', 'publishers': pubs.get('/tf', []),
            },
            'map->odom': {
                **map_odom_tf, 'topic': '/tf', 'publishers': pubs.get('/tf', []),
            },
            'map->base_footprint': {
                **map_base_lookup, 'query_frame': 'map -> base_footprint',
            },
            'map->lidar': {
                **lidar_lookup, 'scan_frame': scan_frame,
                'latest_scan_stamp_s': latest_scan.get('stamp_s'),
            },
        }
        return {
            'evaluated_at_wall_s': time.monotonic(),
            'latest_clock_s': sim_now,
            'clock_hz': clock_hz,
            'clock_sim_delta_s': clock_delta,
            'clock_wall_delta_s': clock_wall_delta,
            'gazebo_rtf': rtf,
            'physics': physics or {
                'service': '/gazebo/get_physics_properties',
                'status': 'not advertised; physics inferred from advancing /clock and /model_states',
                'paused': None,
            },
            'model_pose': latest_model_pose or None,
            'model_pose_span': model_pose_span,
            'joint_positions': latest_joint_positions,
            'odom_pose': latest_odom_pose or None,
            'odom_age_ms': (stats['odom'].get('wall_age_s') * 1000
                            if stats['odom'].get('wall_age_s') is not None else None),
            'controllers': controllers,
            'hardware_interfaces': hardware,
            'controllers_query_age_s': (now - self.controllers_response_wall
                                        if self.controllers_response_wall else None),
            'hardware_query_age_s': (now - self.hardware_response_wall
                                     if self.hardware_response_wall else None),
            'hardware_commands_ready': hardware_commands_ready,
            'tf_publishers': pubs.get('/tf', []),
            'map_publishers': pubs.get('/map', []),
            'slam_scan_subscribers': subs.get('/scan', []),
            'command_topics': command_topics,
            'command_topic_publishers': {
                '/cmd_vel_selected': sorted(selected_publishers),
                '/command_owner': sorted(owner_publishers),
                '/command_arbiter/diagnostics': sorted(diagnostics_publishers),
            },
            'graph_snapshot_age_s': (now - self.graph_snapshot_wall
                                     if self.graph_snapshot_wall else None),
            'graph_snapshot_duration_s': self.graph_snapshot_duration_s,
            'sources': sources,
            'transforms': tf_sources,
            'tf_out_of_order_messages_ignored': self.tf_out_of_order_messages_ignored,
            'map': latest_map or None,
            'gates': gates,
            'all_gates_pass': all(gates.values()),
        }


def map_extension_evidence(before: dict, after: dict) -> dict:
    """Count newly-known cells after mapping in world coordinates."""
    evidence = {'passed': False, 'newly_known_cells': 0,
                'known_cells_outside_prior_extent': 0, 'reason': None}
    try:
        def unpack(snapshot):
            if snapshot.get('data_encoding') != 'zlib-base64-offset1':
                raise ValueError('unsupported OccupancyGrid wire encoding')
            width, height = int(snapshot['width']), int(snapshot['height'])
            raw = zlib.decompress(base64.b64decode(snapshot['data_zlib_base64'], validate=True))
            if len(raw) != width * height:
                raise ValueError('OccupancyGrid payload size does not match metadata')
            origin = snapshot['origin']
            resolution = float(snapshot['resolution'])
            if isinstance(origin, dict):
                origin = (float(origin['x']), float(origin['y']), float(origin.get('yaw', 0.0)))
            else:
                origin = tuple(float(value) for value in origin)
            if len(origin) != 3 or resolution <= 0.0:
                raise ValueError('invalid OccupancyGrid geometry')
            return width, height, resolution, origin, raw

        before_w, before_h, before_res, before_origin, before_data = unpack(before)
        after_w, after_h, after_res, after_origin, after_data = unpack(after)
        if not math.isclose(before_res, after_res, rel_tol=0.0, abs_tol=1e-6):
            raise ValueError('map resolution changed during the same resumed session')
        bx, by, byaw = before_origin
        ax, ay, ayaw = after_origin
        bcos, bsin = math.cos(byaw), math.sin(byaw)
        acos, asin = math.cos(ayaw), math.sin(ayaw)
        new_known = outside = 0
        for ay_idx in range(after_h):
            for ax_idx in range(after_w):
                after_value = after_data[ay_idx * after_w + ax_idx] - 1
                if after_value < 0:
                    continue
                world_x = ax + acos * ((ax_idx + 0.5) * after_res) - asin * ((ay_idx + 0.5) * after_res)
                world_y = ay + asin * ((ax_idx + 0.5) * after_res) + acos * ((ay_idx + 0.5) * after_res)
                dx, dy = world_x - bx, world_y - by
                before_x = math.floor((bcos * dx + bsin * dy) / before_res)
                before_y = math.floor((-bsin * dx + bcos * dy) / before_res)
                if not (0 <= before_x < before_w and 0 <= before_y < before_h):
                    outside += 1
                elif before_data[before_y * before_w + before_x] == 0:
                    new_known += 1
        evidence.update({
            'newly_known_cells': new_known,
            'known_cells_outside_prior_extent': outside,
            'before_dimensions': [before_w, before_h],
            'after_dimensions': [after_w, after_h],
            'before_known_cells': sum(1 for value in before_data if value),
            'after_known_cells': sum(1 for value in after_data if value),
            'before_signature': before.get('signature') or hashlib.sha256(before_data).hexdigest(),
            'after_signature': after.get('signature') or hashlib.sha256(after_data).hexdigest(),
        })
        evidence['passed'] = bool(new_known or outside or
                                  before_w != after_w or before_h != after_h)
        evidence['reason'] = None if evidence['passed'] else 'no previously-unknown world cells became known'
    except (KeyError, TypeError, ValueError, zlib.error) as exc:
        evidence['reason'] = f'{type(exc).__name__}: {exc}'
    return evidence


def saved_session_artifact_evidence(record: dict, map_root: Path) -> dict:
    """Validate the distinct post-resume registry row and all saved products."""
    evidence = {'passed': False, 'files': {}, 'reason': None}
    try:
        artifacts = {}
        for key in ('_yaml', '_image', '_slam_posegraph', '_slam_data'):
            relative = str(record[key])
            path = (map_root / relative).resolve(strict=True)
            if not path.is_relative_to(map_root) or not path.is_file() or path.stat().st_size <= 0:
                raise ValueError(f'{key} does not resolve to a nonempty in-registry artifact')
            artifacts[key] = path
            evidence['files'][key] = {'path': str(path), 'size_bytes': path.stat().st_size}

        import yaml
        metadata = yaml.safe_load(artifacts['_yaml'].read_text(encoding='utf-8'))
        if not isinstance(metadata, dict):
            raise ValueError('saved map YAML is not a mapping')
        image_in_yaml = (artifacts['_yaml'].parent / str(metadata.get('image', ''))).resolve(strict=True)
        if image_in_yaml != artifacts['_image']:
            raise ValueError('saved YAML image reference does not match the registry image')
        resolution = float(metadata['resolution'])
        origin = [float(value) for value in metadata['origin']]
        if (not math.isclose(resolution, float(record['resolution']), rel_tol=0.0, abs_tol=1e-6)
                or len(origin) != 3
                or any(not math.isclose(origin[i], float(record['origin'][i]),
                                         rel_tol=0.0, abs_tol=1e-4) for i in range(3))):
            raise ValueError('saved YAML geometry does not match map registry metadata')
        occupied_threshold = float(metadata['occupied_thresh'])
        free_threshold = float(metadata['free_thresh'])
        negate = int(metadata['negate'])
        if (not 0.0 <= free_threshold < occupied_threshold <= 1.0
                or negate not in (0, 1)):
            raise ValueError('saved YAML threshold/negate metadata is invalid')

        data = artifacts['_image'].read_bytes()
        image_sha256 = hashlib.sha256(data).hexdigest()
        if record.get('image_sha256') and image_sha256 != record.get('image_sha256'):
            raise ValueError('saved image hash does not match map registry metadata')
        cursor = 0
        tokens = []
        while len(tokens) < 4:
            while cursor < len(data) and data[cursor] in b' \t\r\n':
                cursor += 1
            if cursor < len(data) and data[cursor] == ord('#'):
                while cursor < len(data) and data[cursor] not in b'\r\n':
                    cursor += 1
                continue
            end = cursor
            while end < len(data) and data[end] not in b' \t\r\n#':
                end += 1
            if end == cursor:
                raise ValueError('saved PGM header is truncated')
            tokens.append(data[cursor:end].decode('ascii'))
            cursor = end
            if cursor < len(data) and data[cursor] == 13 and cursor + 1 < len(data) \
                    and data[cursor + 1] == 10:
                cursor += 2
            elif cursor < len(data) and data[cursor] in b' \t\r\n':
                cursor += 1
        if tokens[0] != 'P5':
            raise ValueError(f'unsupported saved PGM encoding {tokens[0]!r}')
        width, height, max_value = int(tokens[1]), int(tokens[2]), int(tokens[3])
        if max_value > 255 or width <= 0 or height <= 0 \
                or len(data) - cursor != width * height:
            raise ValueError('saved PGM dimensions/payload are invalid')
        if (width, height) != (int(record['width']), int(record['height'])):
            raise ValueError('saved PGM dimensions do not match registry metadata')
        evidence.update({
            'passed': True,
            'name': record.get('name'), 'map_id': record.get('id'),
            'revision': record.get('revision'),
            'dimensions': [width, height], 'resolution': resolution, 'origin': origin,
            'known_cells': int(record.get('known_cells') or 0),
            'image_sha256': image_sha256,
            'image_format': 'P5', 'image_max_value': max_value,
            'yaml_metadata': metadata,
        })
    except Exception as exc:
        evidence['reason'] = f'{type(exc).__name__}: {exc}'
    return evidence


def main() -> int:
    robot_id = os.environ.get('ROBOT_ID', 'R01')
    map_name = os.environ.get('SAVED_MAP_NAME', 'slam_accumulated_20261001_01')
    backend = os.environ.get('BACKEND_URL', '').rstrip('/')
    frontend = os.environ.get('FRONTEND_URL', '')
    artifact_root = Path(os.environ.get('WARETWIN_ARTIFACT_ROOT') or ROOT / 'generated' / 'maps').expanduser().resolve()
    registry_path = artifact_root / 'local_robot_maps' / robot_id / 'registry.json'
    records = json.loads(registry_path.read_text(encoding='utf-8'))
    record = next((row for row in records if row.get('name') == map_name
                   and row.get('robot_id') == robot_id), None)
    if not record:
        raise RuntimeError(f'{map_name} is not registered for {robot_id}')
    map_root = registry_path.parent.resolve()
    yaml_path = (map_root / str(record['_yaml'])).resolve(strict=True)
    image_path = (map_root / str(record['_image'])).resolve(strict=True)
    posegraph_path = (map_root / str(record['_slam_posegraph'])).resolve(strict=True)
    session_data_path = (map_root / str(record['_slam_data'])).resolve(strict=True)
    for artifact in (yaml_path, image_path, posegraph_path, session_data_path):
        if not artifact.is_relative_to(map_root) or not artifact.is_file() or artifact.stat().st_size <= 0:
            raise RuntimeError(f'saved local map/session artifact is missing or out of scope: {artifact.name}')
    if (record.get('slam_session_state') or {}).get('status') != 'AVAILABLE':
        raise RuntimeError('registered saved SLAM session is not AVAILABLE')

    evidence_dir = Path(tempfile.mkdtemp(prefix='slam-resume-', dir=ROOT / '.runtime'))
    browser_log = evidence_dir / 'browser.log'
    output = ROOT / '.runtime' / 'local-map-slam-resume-acceptance.json'
    result = {
        'passed': False, 'source': 'Mapping tab -> Django -> supervisor -> SLAM Toolbox configure -> live /map',
        'robot_id': robot_id, 'saved_map_name': map_name,
        'map_id': record['id'], 'map_revision': record['revision'],
        'source_mapping_session_id': record.get('source_mapping_session_id'),
        'saved_map': {
            'yaml': str(yaml_path), 'image': str(image_path),
            'posegraph_bytes': posegraph_path.stat().st_size,
            'session_data_bytes': session_data_path.stat().st_size,
            'dimensions': [record['width'], record['height']],
            'resolution': record['resolution'], 'origin': record['origin'],
            'known_cells': record.get('known_cells'),
            'image_sha256': record.get('image_sha256'),
        },
        'evidence_directory': str(evidence_dir),
    }
    rclpy.init()
    probe = acceptance.MotionProbe()
    token = acceptance.authenticate(backend)
    ws_url = backend.replace('https://', 'wss://').replace('http://', 'ws://') + '/ws?token=' + token
    ws = websocket.create_connection(ws_url, timeout=5.0, enable_multithread=True)
    ws.settimeout(0.02)
    readiness = None
    readiness_node = None
    readiness_executor = None
    readiness_thread = None
    browser = None
    try:
        ready = probe.wait_until(lambda: probe.odom is not None and probe.gazebo_pose is not None
            and probe.sim_time() > 0 and probe.runtime_status is not None
            and robot_id in (probe.runtime_status.get('connected_robot_ids') or []), 45.0, ws)
        if not ready:
            raise RuntimeError('live Gazebo pose, clock, or authenticated bridge was not ready')
        runtime_state = str(probe.runtime_status.get('runtime_state') or '').upper()
        if runtime_state not in ('NAVIGATION', 'MAPPING'):
            raise RuntimeError(
                f'bounded resume acceptance requires supervised NAVIGATION or MAPPING mode, got {runtime_state or "UNKNOWN"}')
        manual_ok, manual_reason = acceptance.set_mode(probe, ws, robot_id, 'MANUAL', timeout=20.0)
        if not manual_ok:
            raise RuntimeError(f'robot did not confirm MANUAL mode before resume: {manual_reason}')
        result['before_resume'] = {
            'gazebo_pose': probe.gazebo_pose, 'odom_pose': probe.odom,
            'runtime_mode': runtime_state,
        }

        child_env = os.environ.copy()
        child_env.update({
            'BACKEND_URL': backend, 'FRONTEND_URL': frontend,
            'ROBOT_ID': robot_id, 'SAVED_MAP_NAME': map_name,
            'SLAM_RESUME_ACCEPTANCE_DIR': str(evidence_dir),
            'SLAM_RESUME_TELEOP_HOLD_S': os.environ.get(
                'SLAM_RESUME_TELEOP_HOLD_S', '4.5'),
        })
        for key in ('SLAM_RESUME_SKIP_REQUEST', 'SLAM_RESUME_RESTORE_EVIDENCE'):
            if os.environ.get(key):
                child_env[key] = os.environ[key]
        with browser_log.open('w', encoding='utf-8') as log_stream:
            browser = subprocess.Popen(
                ['node', str(ROOT / 'scripts' / 'local_control_slam_resume_browser.cjs')],
                cwd=ROOT, env=child_env, stdout=log_stream, stderr=subprocess.STDOUT,
                start_new_session=True)
            started = time.monotonic()
            restore_started = None
            readiness_since = None
            next_readiness_sample = 0.0
            latest_readiness = None
            teleop_gate_path = evidence_dir / 'teleop-gate.json'
            teleop_start = teleop_end = None
            teleop_start_monotonic = teleop_end_monotonic = None
            start_selected_index = start_owner_index = start_drive_index = None
            start_joint_index = start_odom = None
            while browser.poll() is None:
                if ws is not None:
                    ws_error_index = len(probe.ws_errors)
                    try:
                        probe.pump(ws, 0.02)
                    except (OSError, websocket.WebSocketException) as exc:
                        # The supervisor deliberately closes the bridge/ROS
                        # websocket while replacing the Navigation runtime.
                        # Keep collecting direct ROS samples after that normal
                        # handover; this observer socket is not the product
                        # Web Teleop path under test.
                        result['observer_websocket_disconnect'] = (
                            f'{type(exc).__name__}: {exc}')
                        try:
                            ws.close()
                        except Exception:
                            pass
                        ws = None
                        probe.pump(None, 0.02)
                    if (ws is not None and any(
                            error.startswith('websocket_closed:')
                            for error in probe.ws_errors[ws_error_index:])):
                        result['observer_websocket_disconnect'] = \
                            probe.ws_errors[-1]
                        try:
                            ws.close()
                        except Exception:
                            pass
                        ws = None
                else:
                    probe.pump(None, 0.02)
                restored_marker = evidence_dir / 'session-restored.json'
                if restore_started is None and restored_marker.is_file():
                    restore_started = time.monotonic()
                    result['session_restored_wall_monotonic_s'] = restore_started
                    readiness_node = ResumeReadinessNode()
                    readiness = ResumeReadinessGate(readiness_node, robot_id)
                    readiness_executor = SingleThreadedExecutor()
                    readiness_executor.add_node(readiness_node)
                    readiness_thread = Thread(
                        target=readiness_executor.spin,
                        name='slam-resume-readiness', daemon=True)
                    readiness_thread.start()
                    # The existing UI/motion probe has observed the pre-restart
                    # simulation epoch. Clear its cached transforms and samples
                    # so map-pose evidence cannot reuse timestamps from before
                    # Gazebo reset its simulation clock.
                    probe.tf.clear()
                    probe.odom = None
                    probe.odom_sample_monotonic = None
                    probe.gazebo_pose = None
                    probe.gazebo_pose_sample_monotonic = None
                    probe.map = None
                    probe.joint_state = None
                    probe.command_owner_events.clear()
                    probe.selected_cmd_events.clear()
                    probe.drive_events.clear()
                    probe.steering_events.clear()
                    probe.joint_events.clear()
                    write_json(teleop_gate_path, {
                        'state': 'WAITING', 'updated_at_ms': int(time.time() * 1000),
                        'reason': 'collecting a stable post-resume sensor/controller/TF readiness window',
                        'gates': {},
                    })
                if readiness is not None:
                    readiness._send_service_requests()
                if (readiness is not None and restore_started is not None
                        and time.monotonic() >= next_readiness_sample):
                    next_readiness_sample = time.monotonic() + 0.25
                    latest_readiness = readiness.snapshot()
                    result['post_resume_readiness'] = latest_readiness
                    write_json(ROOT / '.runtime' / 'local-map-slam-resume-readiness.json', {
                        'map_id': record['id'], 'map_revision': record['revision'],
                        'readiness': latest_readiness,
                    })
                    if latest_readiness['all_gates_pass']:
                        readiness_since = readiness_since or time.monotonic()
                    else:
                        readiness_since = None
                    stable_s = (time.monotonic() - readiness_since
                                if readiness_since is not None else 0.0)
                    if stable_s >= readiness.WINDOW_S:
                        write_json(teleop_gate_path, {
                            'state': 'APPROVED', 'updated_at_ms': int(time.time() * 1000),
                            'stable_window_s': stable_s,
                            'gates': latest_readiness['gates'],
                            'snapshot': latest_readiness,
                        })
                    elif time.monotonic() - restore_started >= 120.0:
                        failed = [name for name, passed in latest_readiness['gates'].items()
                                  if not passed]
                        write_json(teleop_gate_path, {
                            'state': 'BLOCKED', 'updated_at_ms': int(time.time() * 1000),
                            'reason': 'post-resume readiness did not remain fresh long enough; no motion was sent',
                            'failed_gates': failed, 'gates': latest_readiness['gates'],
                            'snapshot': latest_readiness,
                        })
                    else:
                        write_json(teleop_gate_path, {
                            'state': 'WAITING', 'updated_at_ms': int(time.time() * 1000),
                            'stable_window_s': stable_s,
                            'gates': latest_readiness['gates'],
                            'failed_gates': [name for name, passed in latest_readiness['gates'].items()
                                             if not passed],
                        })
                if teleop_start is None:
                    marker = evidence_dir / 'teleop-start.json'
                    if marker.is_file():
                        teleop_start = json.loads(marker.read_text(encoding='utf-8'))
                        teleop_start_monotonic = time.monotonic()
                        gate_file = json.loads(teleop_gate_path.read_text(encoding='utf-8'))
                        result['teleop_gate_at_command'] = gate_file
                        if (gate_file.get('state') != 'APPROVED'
                                or int(time.time() * 1000) - int(gate_file.get('updated_at_ms') or 0) > 500
                                or not all(gate_file.get('gates', {}).values())):
                            stop_browser_process(browser)
                            raise RuntimeError('post-resume readiness lease expired before Web Teleop')
                        start_selected_index = len(probe.selected_cmd_events)
                        start_owner_index = len(probe.command_owner_events)
                        start_drive_index = len(probe.drive_events)
                        start_joint_index = len(probe.joint_events)
                        start_odom = list(readiness.latest_odom_pose or []) or None
                        result['teleop_start_pose'] = {
                            'gazebo_pose': probe.gazebo_pose, 'map_pose': probe.map_pose(),
                            'odom_pose': probe.odom, 'sim_time': probe.sim_time(),
                            'map': readiness.latest_map,
                        }
                if teleop_end is None:
                    marker = evidence_dir / 'teleop-end.json'
                    if marker.is_file():
                        teleop_end = json.loads(marker.read_text(encoding='utf-8'))
                        teleop_end_monotonic = time.monotonic()
                        result['teleop_end_pose'] = {
                            'gazebo_pose': probe.gazebo_pose, 'map_pose': probe.map_pose(),
                            'odom_pose': probe.odom, 'sim_time': probe.sim_time(),
                        }
                if time.monotonic() - started > 22 * 60:
                    stop_browser_process(browser)
                    raise TimeoutError('Web saved-session resume acceptance exceeded 22 minutes')

            if latest_readiness is not None:
                result['post_resume_readiness'] = latest_readiness
            if teleop_start is None:
                gate_file = json.loads(teleop_gate_path.read_text(encoding='utf-8')) \
                    if teleop_gate_path.is_file() else None
                result['teleop_gate'] = gate_file
                result['motion_blocked_before_command'] = True
                result['failure_reason'] = (gate_file or {}).get('reason') or \
                    'post-resume readiness gate did not approve a movement command'

        browser_code = browser.returncode
        result['browser_exit_code'] = browser_code
        browser_result_path = evidence_dir / 'browser-result.json'
        if not browser_result_path.is_file():
            raise RuntimeError('browser helper did not persist its result; see browser.log')
        browser_result = json.loads(browser_result_path.read_text(encoding='utf-8'))
        result['browser'] = browser_result
        if browser_code != 0 or browser_result.get('errors'):
            raise RuntimeError(f'Web resume/teleop browser acceptance failed: {browser_log.read_text(encoding="utf-8")[-2500:]}')
        restored = browser_result.get('resume_responses') or []
        resume_result = next((item.get('body') for item in reversed(restored)
                              if (item.get('body') or {}).get('status') == 'RESUMED'), None)
        if resume_result is None:
            resume_result = (browser_result.get('restoration_carryover') or {}).get('body')
        initial_map = browser_result.get('initial_map_snapshot') or {}
        extended_map = browser_result.get('extended_map_snapshot') or {}
        initial_check = slam_map_restoration_evidence(record, yaml_path, image_path, initial_map)
        extended_check = (slam_map_restoration_evidence(record, yaml_path, image_path, extended_map)
                          if extended_map else {'passed': False, 'reason': 'no post-motion SLAM map snapshot'})
        result['session_restore_response'] = resume_result
        result['initial_map_restoration_check'] = initial_check
        result['post_motion_old_map_check'] = extended_check
        save_request = browser_result.get('resumed_map_save') or {}
        refreshed_registry = json.loads(registry_path.read_text(encoding='utf-8'))
        resumed_record = next((row for row in refreshed_registry
                               if row.get('name') == save_request.get('name')
                               and row.get('robot_id') == robot_id), None)
        original_record_after = next((row for row in refreshed_registry
                                      if row.get('id') == record.get('id')), None)
        artifact_check = (saved_session_artifact_evidence(resumed_record, map_root)
                          if resumed_record else {
                              'passed': False, 'reason': 'new resumed map is absent from registry'})
        original_unchanged = bool(original_record_after and all(
            original_record_after.get(key) == record.get(key)
            for key in ('id', 'name', 'revision', 'image_sha256',
                        '_yaml', '_image', '_slam_posegraph', '_slam_data')))
        new_id = (resumed_record or {}).get('id')
        saved_dimensions = [int((resumed_record or {}).get('width') or 0),
                           int((resumed_record or {}).get('height') or 0)]
        extended_dimensions = [int(extended_map.get('width') or 0),
                               int(extended_map.get('height') or 0)]
        save_dimensions_match = bool(extended_dimensions[0] > 0
                                     and saved_dimensions == extended_dimensions)
        save_cells_match = bool(resumed_record
                                and int(resumed_record.get('known_cells') or 0)
                                >= int(initial_map.get('known_cells') or 0))
        resumed_save_passed = bool(
            save_request.get('distinct_name')
            and save_request.get('name') != map_name
            and new_id and new_id != record.get('id')
            and (resumed_record or {}).get('canonical_map_promoted') is False
            and ((resumed_record or {}).get('slam_session_state') or {}).get('status') == 'AVAILABLE'
            and artifact_check.get('passed') and original_unchanged
            and save_dimensions_match and save_cells_match)
        result['resumed_map_save_evidence'] = {
            'passed': resumed_save_passed,
            'requested_name': save_request.get('name'),
            'registry_map_id': new_id,
            'distinct_from_original': bool(new_id and new_id != record.get('id')),
            'canonical_map_promoted': (resumed_record or {}).get('canonical_map_promoted'),
            'original_registry_unchanged': original_unchanged,
            'dimensions': saved_dimensions,
            'extended_map_dimensions': extended_dimensions,
            'saved_known_cells': (resumed_record or {}).get('known_cells'),
            'extended_map_known_cells': extended_map.get('known_cells'),
            'artifacts': artifact_check,
        }

        if teleop_start is None or teleop_end is None:
            result['acceptance'] = {
                'SLAM_SESSION_LOAD': bool(resume_result and resume_result.get('status') == 'RESUMED'),
                'OLD_MAP_RESTORED': bool(initial_check.get('passed')),
                'POST_RESUME_READINESS_GATE': bool(
                    (result.get('teleop_gate') or {}).get('state') == 'APPROVED'),
                'RESUME_WEB_TELEOP': False,
                'RESUMED_MAP_EXTENDS': False,
                'OLD_MAP_PRESERVED_AFTER_EXTENSION': False,
                'RESUMED_MAP_SAVE': resumed_save_passed,
                'RESUME_MAPPING': False,
            }
            browser_diagnostics = browser_result.get('readiness_gate')
            if browser_diagnostics:
                result['teleop_gate'] = browser_diagnostics
            result['browser_exit_code'] = browser_code
            raise RuntimeError(result['failure_reason'])

        try:
            settled = probe.wait_mechanical_settling(ws, timeout=15.0, wall_timeout=60.0)
        except Exception as exc:
            settled = {'passed': False, 'reason': f'{type(exc).__name__}: {exc}'}
        final_gazebo_pose = probe.gazebo_pose
        start_pose = result.get('teleop_start_pose', {}).get('gazebo_pose')
        motion = acceptance.MotionProbe.body_displacement(start_pose, final_gazebo_pose)
        odom_end = readiness.latest_odom_pose
        odom_start_xy = start_odom
        odom_displacement_m = (math.hypot(odom_end[0] - odom_start_xy[0],
                                         odom_end[1] - odom_start_xy[1])
                               if odom_end and odom_start_xy else None)
        gazebo_displacement_m = (math.hypot(final_gazebo_pose[0] - start_pose[0],
                                           final_gazebo_pose[1] - start_pose[1])
                                 if final_gazebo_pose and start_pose else None)
        selected = probe.selected_cmd_events[start_selected_index or 0:]
        owners = probe.command_owner_events[start_owner_index or 0:]
        direct_selected = readiness._command_samples_between(
            'selected', teleop_start_monotonic or 0.0,
            teleop_end_monotonic or time.monotonic())
        direct_owners = readiness._command_samples_between(
            'owner', teleop_start_monotonic or 0.0,
            teleop_end_monotonic or time.monotonic())
        drives = probe.drive_events[start_drive_index or 0:]
        joint_events = probe.joint_events[start_joint_index or 0:]
        first_joint_positions = joint_events[0][1] if joint_events else {}
        last_joint_positions = joint_events[-1][1] if joint_events else {}
        joints_changed = any(abs(last_joint_positions.get(name, value) - value) > 1e-4
                             for name, value in first_joint_positions.items())
        nonzero_selected = [row for row in direct_selected
                            if any(abs(value) > 1e-4 for value in row[1])]
        probe_nonzero_selected = [row for row in selected
                                  if any(abs(value) > 1e-4 for value in row[1:])]
        manual_owner_times = [wall for wall, owner in direct_owners
                              if owner == 'WEB_MANUAL']
        manual_owner_max_gap = (max((right - left for left, right in
                                     zip(manual_owner_times, manual_owner_times[1:])),
                                    default=None))
        manual_source_continuous = (len(manual_owner_times) >= 5
                                    and manual_owner_max_gap is not None
                                    and manual_owner_max_gap <= 0.50)
        result['teleop_motion'] = {
            'action': 'FORWARD', 'start_gazebo_pose': start_pose,
            'final_gazebo_pose': final_gazebo_pose,
            'gazebo_displacement': motion,
            'gazebo_displacement_m': gazebo_displacement_m,
            'odom_start': odom_start_xy, 'odom_end': odom_end,
            'odom_displacement_m': odom_displacement_m,
            'joint_state_samples': len(joint_events),
            'joint_positions_changed': joints_changed,
            'selected_nonzero_samples': len(nonzero_selected),
            'motion_probe_selected_nonzero_samples': len(probe_nonzero_selected),
            'motion_probe_owner_samples': len(owners),
            'web_manual_owner_seen': any(owner == 'WEB_MANUAL' for _, owner in direct_owners),
            'web_manual_owner_samples': len(manual_owner_times),
            'web_manual_owner_max_gap_s': manual_owner_max_gap,
            'web_manual_source_continuous': manual_source_continuous,
            'direct_command_sample_count': len(direct_selected),
            'direct_command_owner_sample_count': len(direct_owners),
            'drive_command_active': any(any(abs(value) > 1e-4 for value in row[1:]) for row in drives),
            'stop_settled': settled,
        }
        nodes = subprocess.run(
            ['ros2', 'node', 'list', '--no-daemon', '--spin-time', '1'],
            cwd=ROOT, capture_output=True, text=True, timeout=15, check=False)
        node_names = [line.strip() for line in nodes.stdout.splitlines() if line.strip()]
        result['mapping_ownership'] = {
            'slam_toolbox_present': any(name.rstrip('/').endswith('/slam_toolbox') for name in node_names),
            'ekf_v30e_present': any('ekf_v30e' in name for name in node_names),
            'amcl_present': any('amcl' in name for name in node_names),
            'nodes': node_names,
        }

        saved_cells = int(record.get('known_cells') or 0)
        initial_cells = int(initial_map.get('known_cells') or 0)
        final_cells = int(extended_map.get('known_cells') or 0)
        dimensions_grew = (extended_map.get('width'), extended_map.get('height')) != (
            initial_map.get('width'), initial_map.get('height'))
        extension = map_extension_evidence(initial_map, extended_map) if extended_map else {
            'passed': False, 'reason': 'no post-motion map snapshot'}
        map_extended = (final_cells > initial_cells or dimensions_grew
                        or extension.get('passed', False))
        gate_at_command = result.get('teleop_gate_at_command') or {}
        gates_at_command = gate_at_command.get('gates') or {}
        resume_web_teleop = bool(
            result['teleop_motion']['web_manual_source_continuous']
            and result['teleop_motion']['selected_nonzero_samples'] > 0
            and result['teleop_motion']['drive_command_active']
            and result['teleop_motion']['joint_positions_changed']
            and gazebo_displacement_m is not None and gazebo_displacement_m > 0.10
            and odom_displacement_m is not None and odom_displacement_m > 0.05
            and settled.get('passed'))
        result['acceptance'] = {
            'SLAM_SESSION_LOAD': bool(resume_result and resume_result.get('status') == 'RESUMED'),
            'OLD_MAP_RESTORED': bool(initial_check.get('passed')),
            'POST_RESUME_READINESS_GATE': bool(gates_at_command and all(gates_at_command.values())),
            'RESUME_WEB_TELEOP': resume_web_teleop,
            'RESUMED_MAP_EXTENDS': bool(map_extended and extension.get('passed')),
            'OLD_MAP_PRESERVED_AFTER_EXTENSION': bool(
                extended_check.get('passed')
                and int(extended_check.get('covered_saved_cells') or 0) >= saved_cells
                and float(extended_check.get('cell_class_agreement_ratio') or 0.0) >= 0.995),
            'RESUMED_MAP_SAVE': resumed_save_passed,
            'MAP_ODOM_OWNER_SLAM_TOOLBOX': bool(
                result['mapping_ownership']['slam_toolbox_present']
                and not result['mapping_ownership']['ekf_v30e_present']
                and not result['mapping_ownership']['amcl_present']),
        }
        result['map_extension_evidence'] = extension
        result['mapping_extension'] = {
            'saved_known_cells': saved_cells, 'restored_before_teleop_known_cells': initial_cells,
            'after_teleop_known_cells': final_cells,
            'known_cell_delta': final_cells - initial_cells,
            'saved_map_dimensions': [record['width'], record['height']],
            'restored_dimensions': [initial_map.get('width'), initial_map.get('height')],
            'after_teleop_dimensions': [extended_map.get('width'), extended_map.get('height')],
            'extent_grew': dimensions_grew,
            'before_signature': initial_map.get('signature'),
            'after_signature': extended_map.get('signature'),
            'newly_known_cells': extension.get('newly_known_cells'),
            'known_cells_outside_prior_extent': extension.get('known_cells_outside_prior_extent'),
        }
        result['acceptance']['RESUME_MAPPING'] = bool(
            all(result['acceptance'].get(key, False) for key in (
                'SLAM_SESSION_LOAD', 'OLD_MAP_RESTORED', 'POST_RESUME_READINESS_GATE',
                'RESUME_WEB_TELEOP', 'RESUMED_MAP_EXTENDS',
                'OLD_MAP_PRESERVED_AFTER_EXTENSION', 'RESUMED_MAP_SAVE',
                'MAP_ODOM_OWNER_SLAM_TOOLBOX')))
        result['passed'] = all(result['acceptance'].values())
        if not result['passed']:
            result['failure_reason'] = 'one or more saved-session restoration/extension gates failed'
    except Exception as exc:
        result['failure_reason'] = f'{type(exc).__name__}: {exc}'
        result['failure_traceback'] = traceback.format_exc()
    finally:
        cleanup_errors = []
        try:
            stop_browser_process(browser)
        except Exception as exc:
            cleanup_errors.append(f'browser: {type(exc).__name__}: {exc}')
        if readiness_executor is not None:
            try:
                if not readiness_executor.shutdown(timeout_sec=2.0):
                    cleanup_errors.append('readiness executor did not shut down within 2 seconds')
            except Exception as exc:
                cleanup_errors.append(f'readiness executor: {type(exc).__name__}: {exc}')
        if readiness_thread is not None:
            readiness_thread.join(timeout=2)
            if readiness_thread.is_alive():
                cleanup_errors.append('readiness thread did not exit within 2 seconds')
        if readiness_node is not None:
            try:
                readiness_node.destroy_node()
            except Exception as exc:
                cleanup_errors.append(f'readiness node: {type(exc).__name__}: {exc}')
        if ws is not None:
            try:
                ws.close()
            except Exception as exc:
                cleanup_errors.append(f'websocket: {type(exc).__name__}: {exc}')
        try:
            probe.destroy_node()
        except Exception as exc:
            cleanup_errors.append(f'motion probe: {type(exc).__name__}: {exc}')
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as exc:
            cleanup_errors.append(f'rclpy shutdown: {type(exc).__name__}: {exc}')
        if cleanup_errors:
            result['cleanup_errors'] = cleanup_errors
        write_json(output, result)

    print(json.dumps({'passed': result.get('passed'),
                      'acceptance': result.get('acceptance'),
                      'failure_reason': result.get('failure_reason'),
                      'report': str(output), 'evidence_directory': str(evidence_dir)}, indent=2))
    return 0 if result.get('passed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
