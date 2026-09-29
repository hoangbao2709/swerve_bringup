from __future__ import annotations

import json
import hashlib
import math
import queue
import threading
import os
import re
import time
from collections import deque
from pathlib import Path
from urllib.parse import quote
from datetime import datetime, timezone

import rclpy
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.clock import Clock as RclpyClock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from nav2_msgs.action import NavigateToPose
from rosgraph_msgs.msg import Clock
from swerve_bringup.action import GoToTag
from sensor_msgs.msg import JointState, LaserScan, PointCloud2
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformException, TransformListener

from .qos import canonical_map_qos_profile, gazebo_clock_qos_profile
from .coordinates import (is_small_future_tf_skew, pose_from_transform,
                          quaternion_yaw, rotate_translate_xy)


def yaw_from_quaternion(q) -> float:
    return quaternion_yaw(q)


class SwerveBridge(Node):
    def __init__(self):
        super().__init__('swerve_bridge')
        # WebSocket leases and manual dead-man expiry are measured with
        # time.monotonic(), so their servicing must not slow down with Gazebo's
        # simulation clock (which can run far below real time in VMware).
        self._wall_clock = RclpyClock(clock_type=ClockType.STEADY_TIME)
        self.declare_parameter('robot_id', 'R01')
        self.declare_parameter('namespace', '')
        self.declare_parameter('runtime_state', 'MAPPING')
        self.declare_parameter('require_nav2_map', False)
        self.declare_parameter('require_tag_map', False)
        self.declare_parameter('django_ws_url', os.environ.get('ROS_WS_URL', 'ws://127.0.0.1:8000/ws/ros'))
        self.declare_parameter('django_token', '')
        self.declare_parameter('telemetry_rate', 10.0)
        self.declare_parameter('heartbeat_rate', 1.0)
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('lidar_topic', '/lidar/points')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_path_topic', '/plan')
        self.declare_parameter('local_path_topic', '/local_plan')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_footprint_frame', 'base_footprint')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('tf_max_age_s', 2.0)
        self.declare_parameter('tf_future_tolerance_s', 0.1)
        self.declare_parameter('lidar_ui_hz', 5.0)
        self.declare_parameter('lidar_max_points', 720)
        self.declare_parameter('clock_topic', '/clock')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('emergency_stop_topic', '/emergency_stop')
        self.declare_parameter('navigate_action', '/go_to_tag')
        self.declare_parameter('navigate_pose_action', '/navigate_to_pose')
        self.declare_parameter('navigate_server_timeout', 2.0)
        self.declare_parameter('manual_linear_velocity', 0.25)
        self.declare_parameter('manual_angular_velocity', 0.60)
        self.declare_parameter('manual_command_timeout', 0.40)
        self.declare_parameter('artifact_root', 'generated/maps')
        self.declare_parameter('gazebo_world_file', '')
        self.declare_parameter('nav2_map_file', '')
        self.declare_parameter('datamatrix_map_file', '')
        self.declare_parameter('tag_graph_file', '')
        self.declare_parameter('map_sync_request_file', '.runtime/map-sync-request.json')
        self.declare_parameter('allow_unpublished_fallback', False)
        self.robot_id = str(self.get_parameter('robot_id').value)
        self.namespace = self._normalize_namespace(self.get_parameter('namespace').value)
        self.runtime_state = str(self.get_parameter('runtime_state').value).upper()
        self.require_nav2_map = bool(self.get_parameter('require_nav2_map').value)
        self.require_tag_map = bool(self.get_parameter('require_tag_map').value)
        if self.runtime_state not in ('IDLE', 'SIMULATION', 'MAPPING', 'NAVIGATION', 'ERROR'):
            self.get_logger().warning(f'Unknown runtime_state={self.runtime_state}; using MAPPING')
            self.runtime_state = 'MAPPING'
        configured_ws_url = str(self.get_parameter('django_ws_url').value).strip()
        self.ws_url = configured_ws_url or os.environ.get(
            'ROS_WS_URL', 'ws://127.0.0.1:8000/ws/ros')
        self.token = str(self.get_parameter('django_token').value)
        self.artifact_root = Path(str(self.get_parameter('artifact_root').value)).expanduser()
        self.gazebo_world_file = Path(str(self.get_parameter('gazebo_world_file').value)).expanduser()
        self.nav2_map_file = Path(str(self.get_parameter('nav2_map_file').value)).expanduser()
        self.datamatrix_map_file = Path(str(self.get_parameter('datamatrix_map_file').value)).expanduser()
        self.tag_graph_file = Path(str(self.get_parameter('tag_graph_file').value)).expanduser()
        self.map_sync_request_file = Path(str(self.get_parameter('map_sync_request_file').value)).expanduser()
        self.allow_unpublished_fallback = bool(self.get_parameter('allow_unpublished_fallback').value)
        self.telemetry_period = 1.0 / max(0.1, float(self.get_parameter('telemetry_rate').value))
        self.latest_odom = None
        self.last_joint_state = None
        self.last_lidar_monotonic = None
        self.last_lidar_frame_id = None
        self.last_lidar_stamp = None
        self.lidar_intervals = deque(maxlen=20)
        self.scan_intervals = deque(maxlen=20)
        self.scan_sim_intervals = deque(maxlen=20)
        self.last_scan_monotonic = None
        self.last_scan_stamp = None
        self.latest_scan = None
        self.last_scan_publish_monotonic = 0.0
        self.latest_map = None
        self.latest_map_signature = None
        self.latest_global_path = None
        self.latest_local_path = None
        self.latest_goal_pose = None
        self.last_global_path_signature = None
        self.last_local_path_signature = None
        self.last_goal_signature = None
        self.last_sent_map_signature = None
        self.last_tf_error = None
        self.last_clock_monotonic = None
        self.simulation_time = None
        self.previous_simulation_time = None
        self.previous_simulation_wall = None
        self.gazebo_rtf = None
        self.scan_topic_name = None
        self.nav_state = 'IDLE'
        self.control_mode = 'AUTONOMOUS'
        self.manual_twist = Twist()
        self.manual_deadline = 0.0
        self.emergency_stop_active = False
        self.active_goal = None
        self.active_pose_goal = None
        self.active_context = None
        self.active_pose_context = None
        self.paused_pose_context = None
        self.goal_request_pending = False
        self.cancel_pending = False
        self.pending_cancel_state = None
        self.pending_replan = None
        self.incoming = queue.Queue()
        self.ws = None
        self.ws_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.active_map_revision = None
        self.ros_map_revision = self._read_running_world_revision()
        self.nav2_revision = None
        self.nav2_configured_revision = None
        self.tag_map_revision = None
        self.tag_map_configured_revision = None
        self.tf_status = False
        self.tf_error = 'waiting for map to base transform'
        self.active_published_version = None
        # This is measured from the world actually selected for this launch,
        # not inferred from a MAP_PUBLISHED command.  A bridge can validate an
        # artifact without Gazebo having loaded that world yet.
        self.gazebo_revision = self._read_running_world_revision()
        self.map_sync_status = 'ROS_OFFLINE'
        self.map_sync_error = None
        self.datamatrix_map_path = None
        self.tag_graph_path = None
        self.gazebo_world_path = None
        self.command_times = deque()
        self.controller_list_future = None
        self.controller_states = []
        self.last_bridge_error = None
        self.connection_backoff_s = 1.0

        odom_topic = self._scoped_topic(self.get_parameter('odom_topic').value)
        joint_states_topic = self._scoped_topic(self.get_parameter('joint_states_topic').value)
        lidar_topic = self._scoped_topic(self.get_parameter('lidar_topic').value)
        clock_topic = self._scoped_topic(self.get_parameter('clock_topic').value)
        cmd_vel_topic = self._scoped_topic(self.get_parameter('cmd_vel_topic').value)
        emergency_stop_topic = self._scoped_topic(self.get_parameter('emergency_stop_topic').value)
        navigate_action = self._scoped_topic(self.get_parameter('navigate_action').value)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 20)
        self.create_subscription(JointState, joint_states_topic, self.joint_cb, 10)
        self.create_subscription(PointCloud2, lidar_topic, self.lidar_cb, 10)
        # Gazebo Classic publishes /clock as best-effort + volatile.  The
        # default rclpy profile is reliable, which is incompatible and leaves
        # simulation_time permanently null.  Match the Gazebo profile without
        # changing the publisher or any robot-control topic.
        self.create_subscription(Clock, clock_topic, self.clock_cb, gazebo_clock_qos_profile())
        self.cmd_pub = self.create_publisher(
            Twist, cmd_vel_topic, 20)
        estop_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.estop_pub = self.create_publisher(
            Bool, emergency_stop_topic, estop_qos)
        controller_service = self._scoped_topic('/controller_manager/list_controllers')
        self.controller_client = self.create_client(
            ListControllers, controller_service)
        self.nav_client = ActionClient(self, GoToTag, navigate_action)
        navigate_pose_action = self._scoped_topic(self.get_parameter('navigate_pose_action').value) if self.has_parameter('navigate_pose_action') else self._scoped_topic('/navigate_to_pose')
        self.nav_pose_client = ActionClient(self, NavigateToPose, navigate_pose_action)
        scan_topic = self._scoped_topic(self.get_parameter('scan_topic').value)
        map_topic = self._scoped_topic(self.get_parameter('map_topic').value)
        global_path_topic = self._scoped_topic(self.get_parameter('global_path_topic').value)
        local_path_topic = self._scoped_topic(self.get_parameter('local_path_topic').value)
        goal_topic = self._scoped_topic(self.get_parameter('goal_topic').value)
        self.scan_topic_name = scan_topic
        self.create_subscription(LaserScan, scan_topic, self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, map_topic, self.map_cb, canonical_map_qos_profile())
        self.create_subscription(RosPath, global_path_topic, self.global_path_cb, 1)
        self.create_subscription(RosPath, local_path_topic, self.local_path_cb, 1)
        self.create_subscription(PoseStamped, goal_topic, self.goal_pose_cb, 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0 / max(0.1, float(self.get_parameter('lidar_ui_hz').value)), self.detail_timer)
        self.create_timer(self.telemetry_period, self.telemetry_timer)
        self.create_timer(
            1.0 / max(0.1, float(self.get_parameter('heartbeat_rate').value)),
            self.heartbeat_timer, clock=self._wall_clock)
        self.create_timer(0.05, self.manual_timer, clock=self._wall_clock)
        self.create_timer(0.05, self.process_commands, clock=self._wall_clock)
        self.thread = threading.Thread(target=self.websocket_loop, daemon=True)
        self.thread.start()

    @staticmethod
    def _normalize_namespace(value) -> str:
        return str(value or '').strip().strip('/')

    def _scoped_topic(self, value) -> str:
        """Resolve a configured absolute/relative topic for one robot.

        The default namespace is empty, preserving the existing single-robot
        graph.  A non-empty namespace lets another bridge instance consume and
        publish the same contract under `/robot_N/...` without changing the
        Django command schema.
        """
        topic = str(value or '').strip()
        if not self.namespace or not topic:
            return topic
        return f'/{self.namespace}/{topic.lstrip("/")}'

    def _read_running_world_revision(self):
        """Read the immutable revision manifest for the launched Gazebo world."""
        if not self.gazebo_world_file.is_file():
            return None
        candidates = (
            self.gazebo_world_file.parent / 'manifest.json',
            self.gazebo_world_file.parent.parent / 'manifest.json',
        )
        for manifest_path in candidates:
            try:
                manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
            except (OSError, json.JSONDecodeError):
                continue
            raw_revision = manifest.get('revision', manifest.get('map_revision'))
            try:
                return int(raw_revision)
            except (TypeError, ValueError):
                continue
        return None

    def odom_cb(self, msg):
        self.latest_odom = msg

    def joint_cb(self, msg):
        self.last_joint_state = msg

    def lidar_cb(self, msg):
        now = time.monotonic()
        if self.last_lidar_monotonic is not None:
            interval = now - self.last_lidar_monotonic
            if interval > 0.0:
                self.lidar_intervals.append(interval)
        self.last_lidar_monotonic = now
        self.last_lidar_frame_id = str(msg.header.frame_id or '')
        stamp = msg.header.stamp
        self.last_lidar_stamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def scan_cb(self, msg):
        now = time.monotonic()
        stamp = msg.header.stamp
        stamp_value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if self.last_scan_monotonic is not None:
            interval = now - self.last_scan_monotonic
            if interval > 0.0:
                self.scan_intervals.append(interval)
        if self.last_scan_stamp is not None:
            interval = stamp_value - self.last_scan_stamp
            if interval > 0.0:
                self.scan_sim_intervals.append(interval)
        self.last_scan_monotonic = now
        self.last_scan_stamp = stamp_value
        self.latest_scan = msg
        self.last_lidar_monotonic = now
        self.last_lidar_frame_id = str(msg.header.frame_id or '')
        self.last_lidar_stamp = stamp_value

    def map_cb(self, msg):
        # Header timestamps can advance even when the occupancy content is
        # unchanged.  Keep the snapshot cache content-addressed so a map
        # publisher running at 10 Hz does not push the same full grid over the
        # bridge on every callback.
        origin = msg.info.origin
        signature = (
            str(msg.header.frame_id or 'map'),
            msg.info.width,
            msg.info.height,
            float(msg.info.resolution),
            round(float(origin.position.x), 6),
            round(float(origin.position.y), 6),
            round(yaw_from_quaternion(origin.orientation), 6),
            hash(bytes((int(value) + 1) % 256 for value in msg.data)),
            self.ros_map_revision,
        )
        self.latest_map = msg
        self.latest_map_signature = signature

    def global_path_cb(self, msg):
        self.latest_global_path = msg

    def local_path_cb(self, msg):
        self.latest_local_path = msg

    def goal_pose_cb(self, msg):
        self.latest_goal_pose = msg

    def clock_cb(self, msg):
        now = time.monotonic()
        stamp = msg.clock
        simulation_time = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if self.simulation_time is not None and self.previous_simulation_wall is not None:
            sim_delta = simulation_time - self.simulation_time
            wall_delta = now - self.previous_simulation_wall
            if sim_delta >= 0.0 and wall_delta > 0.0:
                self.gazebo_rtf = sim_delta / wall_delta
        self.previous_simulation_time = self.simulation_time
        self.previous_simulation_wall = now
        self.simulation_time = simulation_time
        self.last_clock_monotonic = now

    def send(self, payload):
        if not isinstance(payload, dict):
            return False
        with self.ws_lock:
            if self.ws is None:
                return False
            try:
                self.ws.send(json.dumps(payload))
                return True
            except Exception as exc:
                self.get_logger().warning(f'ROS bridge send failed: {exc}')
                return False

    def send_bridge_status(self, state, error=None):
        self.last_bridge_error = error
        payload = {'type': 'BRIDGE_STATUS', 'state': state, 'robot_id': self.robot_id,
                   'runtime_state': self.runtime_state, 'timestamp': self.now()}
        if error:
            payload['error'] = str(error)[:300]
        self.send(payload)

    def websocket_loop(self):
        try:
            import websocket
        except ImportError:
            self.get_logger().error('websocket-client is required for Django bridge')
            return
        backoff = 1.0
        while not self.stop_event.is_set():
            ws = None
            try:
                self.send_bridge_status('CONNECTING')
                separator = '&' if '?' in self.ws_url else '?'
                url = f'{self.ws_url}{separator}token={quote(self.token)}&robot_id={quote(self.robot_id)}'
                # The Django Channels consumer accepts JSON frames but does
                # not negotiate a WebSocket subprotocol. Passing
                # subprotocols=['json'] makes websocket-client reject the
                # otherwise valid 101 response with "Invalid WebSocket
                # Header".
                ws = websocket.create_connection(url, timeout=2, enable_multithread=True)
                with self.ws_lock:
                    self.ws = ws
                backoff = 1.0
                self.send_bridge_status('CONNECTED')
                self.send_map_revision_status()
                while not self.stop_event.is_set():
                    try:
                        raw = ws.recv()
                        if raw:
                            data = json.loads(raw)
                            if isinstance(data, dict):
                                self.incoming.put(data)
                            else:
                                self.get_logger().warning('Ignoring non-object command from Django bridge')
                    except websocket.WebSocketTimeoutException:
                        continue
                if ws is not None:
                    ws.close()
            except Exception as exc:
                self.get_logger().warning(f'Django websocket disconnected: {exc}')
                self.send_bridge_status('RECONNECTING', exc)
                with self.ws_lock:
                    self.ws = None
                self.stop_event.wait(backoff)
                backoff = min(30.0, backoff * 2.0)
            finally:
                with self.ws_lock:
                    if self.ws is ws:
                        self.ws = None

    def now(self):
        return datetime.now(timezone.utc).isoformat()

    def telemetry_timer(self):
        msg = self.latest_odom
        if msg is None:
            return
        t = msg.twist.twist
        try:
            pose, base_frame = self._lookup_robot_pose()
            self.tf_status = True
            self.tf_error = None
            self.send({'type': 'ROBOT_STATE', 'robot_id': self.robot_id,
                       'frame_id': str(self.get_parameter('map_frame').value),
                       'map_revision': self.ros_map_revision,
                       'base_frame_id': base_frame, **pose,
                       'vx': t.linear.x, 'vy': t.linear.y, 'wz': t.angular.z,
                       'navigation_state': self.nav_state,
                       'control_mode': self.control_mode, 'timestamp': self.now()})
        except (TransformException, ValueError, TypeError) as exc:
            self.tf_status = False
            self.tf_error = str(exc)[:300]
        self._refresh_map_sync_status()

    def manual_timer(self):
        """Publish the dead-man command while the web operator is holding it."""
        if self.emergency_stop_active or self.control_mode != 'MANUAL':
            return
        if time.monotonic() > self.manual_deadline:
            self.manual_twist = Twist()
        self.cmd_pub.publish(self.manual_twist)

    def heartbeat_timer(self):
        self.send({'type': 'HEARTBEAT', 'robot_id': self.robot_id,
                   'nav2_state': self.nav_state, 'bridge_state': 'CONNECTED',
                   'runtime_state': self.runtime_state, 'control_mode': self.control_mode,
                   'timestamp': self.now()})
        diagnostics = self.collect_diagnostics()
        self.send({'type': 'ROS_DIAGNOSTICS', 'robot_id': self.robot_id,
                   'diagnostics': diagnostics, 'timestamp': self.now()})
        self.send({'type': 'SYSTEM_DIAGNOSTICS', 'robot_id': self.robot_id,
                   'diagnostics': {**diagnostics, 'ros_bridge': True, 'gazebo_rtf': self.gazebo_rtf,
                                   'errors': self.detail_errors(diagnostics)},
                   'timestamp': self.now()})
        self.send({'type': 'CONTROLLER_STATE', 'controller': {
            'robot_id': self.robot_id, 'controllers': self.controller_states[:100], 'timestamp': self.now(),
        }})
        self.send_map_revision_status()

        with self.ws_lock:
            ws = self.ws
            if ws is not None:
                try:
                    ws.ping()
                except Exception as exc:
                    self.get_logger().warning(f'ROS bridge heartbeat ping failed: {exc}')

    @staticmethod
    def _frequency(intervals):
        if not intervals:
            return None
        average = sum(intervals) / len(intervals)
        return 1.0 / average if average > 0.0 else None

    def _transform_xy(self, x, y, source_frame, stamp):
        target_frame = str(self.get_parameter('map_frame').value or 'map')
        if not source_frame:
            raise TransformException('global pose frame_id is empty')
        if source_frame == target_frame:
            return float(x), float(y), 0.0
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, stamp, timeout=Duration(seconds=0.05))
        except TransformException as exact_error:
            # Nav2 may stamp a fresh path slightly ahead of the newest
            # map->odom sample. Reuse the actual latest transform only for
            # this bounded future skew; stale/past requests remain errors.
            try:
                latest = self.tf_buffer.lookup_transform(
                    target_frame, source_frame, Time())
            except TransformException:
                raise exact_error
            tolerance = max(0.0, float(self.get_parameter(
                'tf_future_tolerance_s').value))
            if not is_small_future_tf_skew(
                    stamp, latest.header.stamp, tolerance):
                raise exact_error
            transform = latest
        t, q = transform.transform.translation, transform.transform.rotation
        translated = rotate_translate_xy(
            float(x), float(y), (float(t.x), float(t.y), float(t.z)),
            (float(q.x), float(q.y), float(q.z), float(q.w)))
        return translated[0], translated[1], yaw_from_quaternion(q)

    def _lookup_robot_pose(self):
        target = str(self.get_parameter('map_frame').value or 'map')
        candidates = (str(self.get_parameter('base_footprint_frame').value),
                     str(self.get_parameter('base_link_frame').value))
        max_age = max(0.0, float(self.get_parameter('tf_max_age_s').value))
        errors = []
        for frame in candidates:
            if not frame:
                continue
            try:
                transform = self.tf_buffer.lookup_transform(
                    target, frame, Time(), timeout=Duration(seconds=0.05))
                stamp = transform.header.stamp
                stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
                now_s = self.get_clock().now().nanoseconds * 1e-9
                age = now_s - stamp_s
                if stamp_s > 0.0 and age > max_age:
                    raise TransformException(f'TF {target}->{frame} stale ({age:.2f}s)')
                return (pose_from_transform(transform), frame)
            except TransformException as exc:
                errors.append(f'{frame}: {exc}')
        raise TransformException(f'no fresh {target}->base transform ({"; ".join(errors)})')

    def _pose_path_payload(self, msg, robot_id):
        if msg is None:
            return None
        points = []
        source_frame = str(msg.header.frame_id or '')
        for pose in msg.poses[:2000]:
            frame = str(pose.header.frame_id or source_frame)
            stamp = (Time.from_msg(pose.header.stamp)
                     if pose.header.stamp.sec or pose.header.stamp.nanosec else Time())
            x, y, _ = self._transform_xy(
                pose.pose.position.x, pose.pose.position.y, frame, stamp)
            points.append([x, y])
        stamp = msg.header.stamp
        return {
            'robot_id': robot_id,
            'frame_id': str(self.get_parameter('map_frame').value or 'map'),
            'map_revision': self.ros_map_revision,
            'timestamp': datetime.now(timezone.utc).isoformat(),
            'points': points,
            'stamp': float(stamp.sec) + float(stamp.nanosec) * 1e-9,
        }

    def _goal_payload(self, msg, robot_id, status=None):
        if msg is None:
            return None
        pose = msg.pose
        frame = str(msg.header.frame_id or '')
        stamp = (Time.from_msg(msg.header.stamp)
                 if msg.header.stamp.sec or msg.header.stamp.nanosec else Time())
        x, y, tf_yaw = self._transform_xy(
            pose.position.x, pose.position.y, frame, stamp)
        yaw = yaw_from_quaternion(pose.orientation) + tf_yaw
        return {
            'robot_id': robot_id,
            'frame_id': str(self.get_parameter('map_frame').value or 'map'),
            'map_revision': self.ros_map_revision,
            'x': x, 'y': y, 'yaw': math.atan2(math.sin(yaw), math.cos(yaw)),
            'status': status,
            'timestamp': datetime.now(timezone.utc).isoformat(),
        }

    def _transform_scan(self, scan):
        source_frame = str(scan.header.frame_id or '')
        target_frame = str(self.get_parameter('map_frame').value or 'map')
        if not source_frame:
            raise TransformException('LaserScan frame_id is empty')
        transform = None
        if source_frame != target_frame:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, Time.from_msg(scan.header.stamp), timeout=Duration(seconds=0.05))
        if transform is None:
            translation = (0.0, 0.0, 0.0)
            quaternion = (0.0, 0.0, 0.0, 1.0)
        else:
            t = transform.transform.translation
            q = transform.transform.rotation
            translation = (float(t.x), float(t.y), float(t.z))
            quaternion = (float(q.x), float(q.y), float(q.z), float(q.w))

        qx, qy, qz, qw = quaternion
        valid = []
        for index, value in enumerate(scan.ranges):
            distance = float(value)
            if not math.isfinite(distance) or distance < float(scan.range_min) or distance > float(scan.range_max):
                continue
            angle = float(scan.angle_min) + index * float(scan.angle_increment)
            lx, ly, lz = distance * math.cos(angle), distance * math.sin(angle), 0.0
            # Quaternion rotation, expanded to avoid a dependency on
            # tf2_geometry_msgs in the bridge process.
            tx = 2.0 * (qy * lz - qz * ly)
            ty = 2.0 * (qz * lx - qx * lz)
            tz = 2.0 * (qx * ly - qy * lx)
            rx = lx + qw * tx + (qy * tz - qz * ty)
            ry = ly + qw * ty + (qz * tx - qx * tz)
            rz = lz + qw * tz + (qx * ty - qy * tx)
            valid.append((distance, [translation[0] + rx, translation[1] + ry]))

        limit = max(1, int(self.get_parameter('lidar_max_points').value))
        stride = max(1, math.ceil(len(valid) / limit))
        sampled = valid[::stride][:limit]
        return {
            'robot_id': self.robot_id,
            'topic': self.scan_topic_name,
            'frame_id': target_frame,
            'source_frame_id': source_frame,
            'timestamp': datetime.now(timezone.utc).isoformat(),
            'stamp': self.last_scan_stamp,
            'angle_min': float(scan.angle_min), 'angle_max': float(scan.angle_max),
            'angle_increment': float(scan.angle_increment),
            'range_min': float(scan.range_min), 'range_max': float(scan.range_max),
            'point_count': len(sampled),
            'minimum_range': min((item[0] for item in valid), default=None),
            'maximum_range': max((item[0] for item in valid), default=None),
            'scan_hz_sim': self._frequency(self.scan_sim_intervals),
            'scan_hz_wall': self._frequency(self.scan_intervals),
            'points': [point for _distance, point in sampled],
        }

    def detail_timer(self):
        """Publish bounded, robot-scoped detail snapshots to Django."""
        if self.latest_scan is not None and time.monotonic() - self.last_scan_publish_monotonic >= 1.0 / max(0.1, float(self.get_parameter('lidar_ui_hz').value)):
            try:
                payload = self._transform_scan(self.latest_scan)
                self.send({'type': 'LIDAR_SCAN', 'scan': payload})
                self.last_scan_publish_monotonic = time.monotonic()
                self.last_tf_error = None
            except (TransformException, ValueError, TypeError) as exc:
                self.last_tf_error = str(exc)[:300]

        if self.latest_map is not None and self.latest_map_signature != self.last_sent_map_signature:
            msg = self.latest_map
            origin = msg.info.origin
            stamp = msg.header.stamp
            sent = self.send({'type': 'MAP_SNAPSHOT', 'map': {
                'robot_id': self.robot_id, 'frame_id': str(msg.header.frame_id or ''),
                'map_revision': self.ros_map_revision,
                'timestamp': datetime.now(timezone.utc).isoformat(),
                'stamp': float(stamp.sec) + float(stamp.nanosec) * 1e-9,
                'width': int(msg.info.width), 'height': int(msg.info.height),
                'resolution': float(msg.info.resolution),
                'origin': {'x': float(origin.position.x), 'y': float(origin.position.y), 'yaw': yaw_from_quaternion(origin.orientation)},
                'data': [int(value) for value in msg.data],
            }})
            if sent:
                self.last_sent_map_signature = self.latest_map_signature

        for attr, kind in (("latest_global_path", "NAV_GLOBAL_PATH"), ("latest_local_path", "NAV_LOCAL_PATH")):
            msg = getattr(self, attr)
            if msg is None:
                continue
            signature = (len(msg.poses), tuple((round(float(p.pose.position.x), 3), round(float(p.pose.position.y), 3)) for p in msg.poses))
            marker = 'last_global_path_signature' if kind == 'NAV_GLOBAL_PATH' else 'last_local_path_signature'
            if signature != getattr(self, marker):
                try:
                    payload = self._pose_path_payload(msg, self.robot_id)
                except (TransformException, ValueError, TypeError) as exc:
                    self.last_tf_error = str(exc)[:300]
                    continue
                self.last_tf_error = None
                if self.send({'type': kind, 'path': payload}):
                    setattr(self, marker, signature)

        if self.latest_goal_pose is not None:
            try:
                payload = self._goal_payload(self.latest_goal_pose, self.robot_id)
            except (TransformException, ValueError, TypeError) as exc:
                self.last_tf_error = str(exc)[:300]
                return
            self.last_tf_error = None
            signature = (payload['x'], payload['y'], payload['yaw'], payload['frame_id'])
            if signature != self.last_goal_signature:
                if self.send({'type': 'NAV_GOAL', 'goal': payload}):
                    self.last_goal_signature = signature

    def detail_errors(self, diagnostics=None):
        errors = []
        now = datetime.now(timezone.utc).isoformat()
        if self.last_scan_monotonic is not None and time.monotonic() - self.last_scan_monotonic > 3.0:
            errors.append({'severity': 'WARNING', 'code': 'LIDAR_TIMEOUT', 'message': 'LiDAR timeout', 'timestamp': now})
        if self.last_tf_error:
            errors.append({'severity': 'ERROR', 'code': 'TF_UNAVAILABLE', 'message': f'TF unavailable: {self.last_tf_error}', 'timestamp': now})
        if self.tf_error:
            errors.append({'severity': 'ERROR', 'code': 'TF_UNAVAILABLE', 'message': f'TF unavailable: {self.tf_error}', 'timestamp': now})
        if self.controller_states and any(str(item.get('state', '')).lower() != 'active' for item in self.controller_states):
            errors.append({'severity': 'ERROR', 'code': 'CONTROLLER_INACTIVE', 'message': 'controller inactive', 'timestamp': now})
        if self.nav_state == 'FAILED':
            errors.append({'severity': 'ERROR', 'code': 'NAV_GOAL_FAILED', 'message': 'Nav2 goal failed', 'timestamp': now})
        return errors

    def collect_diagnostics(self):
        """Collect a measured ROS graph snapshot for Django diagnostics."""
        try:
            node_names = sorted(
                f'{namespace.rstrip("/")}/{name.lstrip("/")}' if namespace != '/'
                else f'/{name.lstrip("/")}'
                for name, namespace in self.get_node_names_and_namespaces())
            topic_names = sorted(name for name, _types in self.get_topic_names_and_types())
            service_names = {name for name, _types in self.get_service_names_and_types()}
        except Exception as exc:
            self.get_logger().warning(f'ROS graph diagnostics failed: {exc}')
            node_names, topic_names, service_names = [], [], set()

        controller_manager = (
            any('controller_manager' in name for name in node_names)
            or self._scoped_topic('/controller_manager/list_controllers') in service_names
        )
        if controller_manager and self.controller_client.service_is_ready():
            if self.controller_list_future is not None and self.controller_list_future.done():
                try:
                    response = self.controller_list_future.result()
                    self.controller_states = [
                        {'name': item.name, 'state': item.state}
                        for item in response.controller
                    ]
                except Exception as exc:
                    self.get_logger().warning(f'controller list response failed: {exc}')
                self.controller_list_future = None
            if self.controller_list_future is None:
                try:
                    self.controller_list_future = self.controller_client.call_async(ListControllers.Request())
                except Exception as exc:
                    self.get_logger().warning(f'controller list request failed: {exc}')
                    self.controller_list_future = None

        lidar_age = None
        lidar_frequency = self._frequency(self.scan_intervals) or self._frequency(self.lidar_intervals)
        if self.last_lidar_monotonic is not None:
            lidar_age = max(0.0, time.monotonic() - self.last_lidar_monotonic)
        gazebo = (
            any('gazebo' in name.lower() for name in node_names)
            or (self.last_clock_monotonic is not None and time.monotonic() - self.last_clock_monotonic <= 3.0)
        )
        slam = any('slam_toolbox' in name.lower() for name in node_names)
        nav2_nodes = ('map_server', 'amcl', 'controller_server', 'planner_server',
                      'bt_navigator', 'lifecycle_manager_navigation')
        nav2 = any(any(marker in name for marker in nav2_nodes) for name in node_names)
        self.nav2_revision = self._loaded_nav2_revision(nav2)
        tag_nodes_ready = all(any(name.rstrip('/').endswith('/' + suffix)
                                  or name.rstrip('/') == suffix
                                  for name in node_names)
                              for suffix in ('v30e_sim_node', 'tag_route_planner'))
        self.tag_map_revision = self.tag_map_configured_revision if (
            self.require_tag_map and tag_nodes_ready) else None
        self._refresh_map_sync_status()
        tf = self.tf_status and self._scoped_topic('/tf') in topic_names
        lidar = lidar_age is not None and lidar_age <= 3.0
        return {
            'ros': bool(node_names),
            'gazebo': gazebo,
            'controller_manager': controller_manager,
            'slam': slam,
            'nav2': nav2,
            'tf': tf,
            'lidar': lidar,
            'nodes': node_names[:200],
            'topics': topic_names[:300],
            'controllers': self.controller_states[:100],
            'simulation_time': self.simulation_time,
            'gazebo_rtf': self.gazebo_rtf,
            'errors': self.detail_errors(),
            'metrics': {
                'lidar_age_s': lidar_age,
                'lidar_frequency_hz': lidar_frequency,
                'scan_frequency_sim_hz': self._frequency(self.scan_sim_intervals),
                'lidar_frame_id': self.last_lidar_frame_id,
                'lidar_stamp': self.last_lidar_stamp,
                'gazebo_rtf': self.gazebo_rtf,
                # DDS does not expose dropped samples through this API. Keep
                # this explicitly unknown instead of presenting fabricated 0.
                'lidar_dropped_messages': None,
            },
        }

    def send_map_revision_status(self):
        self._refresh_map_sync_status()
        self.send({
            'type': 'MAP_REVISION_STATUS',
            'robot_id': self.robot_id,
            'map_revision': self.ros_map_revision,
            'ros_revision': self.ros_map_revision,
            'published_version': self.active_published_version,
            'gazebo_revision': self.gazebo_revision,
            'nav2_revision': self.nav2_revision,
            'tag_map_revision': self.tag_map_revision,
            'nav2_required': self.require_nav2_map,
            'tag_map_required': self.require_tag_map,
            'tf_status': bool(self.tf_status),
            'status': self.map_sync_status,
            'error': self.map_sync_error or self.tf_error,
        })

    def _refresh_map_sync_status(self):
        if self.active_map_revision is None:
            return
        nav2_required = self.require_nav2_map
        tag_map_required = self.require_tag_map
        aligned = (self.ros_map_revision == self.active_map_revision
                   and self.gazebo_revision == self.active_map_revision
                   and (not tag_map_required or self.tag_map_revision == self.active_map_revision)
                   and (not nav2_required or self.nav2_revision == self.active_map_revision))
        if aligned and self.tf_status:
            self.map_sync_status = 'SYNCED'
            self.map_sync_error = None
        elif aligned:
            self.map_sync_status = 'OUT_OF_SYNC'
            self.map_sync_error = self.tf_error or 'map to base TF is not available'
        else:
            self.map_sync_status = 'OUT_OF_SYNC'
            expected = self.active_map_revision
            mismatches = []
            components = [('ROS', self.ros_map_revision), ('Gazebo', self.gazebo_revision)]
            if nav2_required:
                components.append(('Nav2', self.nav2_revision))
            if tag_map_required:
                components.append(('tag map', self.tag_map_revision))
            for name, revision in components:
                if revision != expected:
                    mismatches.append(f'{name}={revision if revision is not None else "N/A"}')
            self.map_sync_error = (
                f'map revision {expected} mismatch: {", ".join(mismatches)}'
                if mismatches else self.tf_error or 'map to base TF is not available'
            )

    @staticmethod
    def _read_pgm(path):
        data = path.read_bytes()
        index, tokens = 0, []
        while len(tokens) < 4:
            while index < len(data):
                if data[index] in b' \t\r\n':
                    index += 1
                elif data[index:index + 1] == b'#':
                    newline = data.find(b'\n', index)
                    if newline < 0:
                        raise ValueError('PGM comment is not terminated')
                    index = newline + 1
                else:
                    break
            start = index
            while index < len(data) and data[index] not in b' \t\r\n':
                index += 1
            if start == index:
                raise ValueError('invalid PGM header')
            tokens.append(data[start:index])
        if tokens[0] != b'P5' or tokens[3] != b'255':
            raise ValueError('Nav2 image must be an 8-bit binary PGM')
        if index >= len(data) or data[index] not in b' \t\r\n':
            raise ValueError('PGM header has no raster delimiter')
        delimiter = data[index]
        index += 1
        if delimiter == 13 and index < len(data) and data[index] == 10:
            index += 1
        width, height = int(tokens[1]), int(tokens[2])
        raster = data[index:]
        if len(raster) != width * height:
            raise ValueError('PGM raster length does not match its dimensions')
        return width, height, raster

    def _loaded_nav2_revision(self, nav2_node_present):
        """Only report a Nav2 revision after /map exactly matches its image."""
        if not nav2_node_present or self.latest_map is None:
            return None
        revision = self.nav2_configured_revision
        if revision is None or not self.nav2_map_file.is_file():
            return None
        try:
            text = self.nav2_map_file.read_text(encoding='utf-8')
            image_match = re.search(r'^image:\s*(.+?)\s*$', text, re.MULTILINE)
            resolution_match = re.search(r'^resolution:\s*([-+0-9.eE]+)', text, re.MULTILINE)
            origin_match = re.search(r'^origin:\s*\[\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\]', text, re.MULTILINE)
            frame_match = re.search(r'^frame_id:\s*(\S+)', text, re.MULTILINE)
            if not all((image_match, resolution_match, origin_match, frame_match)) or frame_match.group(1) != 'map':
                return None
            image_path = (self.nav2_map_file.parent / image_match.group(1).strip().strip('"\'' )).resolve(strict=True)
            if not image_path.is_relative_to(self.nav2_map_file.parent.resolve()):
                return None
            width, height, pixels = self._read_pgm(image_path)
            grid = self.latest_map
            if str(grid.header.frame_id or '') != 'map' or int(grid.info.width) != width or int(grid.info.height) != height:
                return None
            if not math.isclose(float(grid.info.resolution), float(resolution_match.group(1)), abs_tol=1e-6):
                return None
            expected_origin = tuple(float(origin_match.group(index)) for index in (1, 2, 3))
            actual = grid.info.origin
            actual_origin = (float(actual.position.x), float(actual.position.y), yaw_from_quaternion(actual.orientation))
            if any(not math.isclose(a, b, abs_tol=1e-5) for a, b in zip(actual_origin, expected_origin)):
                return None
            occupancy = bytes(int(value) for value in grid.data)
            if len(occupancy) != width * height:
                return None
            # OccupancyGrid row 0 is south/bottom; PGM row 0 is north/top.
            for row in range(height):
                image_row = height - row - 1
                begin = image_row * width
                expected_row = bytes(100 if pixels[begin + col] <= 89 else 0 for col in range(width))
                if occupancy[row * width:(row + 1) * width] != expected_row:
                    return None
            return revision
        except (OSError, ValueError, TypeError, AttributeError):
            return None

    @staticmethod
    def _verify_artifact_bundle(root, manifest, revision):
        manifest_path = root / 'manifest.json'
        try:
            disk_manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        except (OSError, ValueError) as exc:
            raise ValueError(f'cannot read artifact manifest: {exc}') from exc
        if int(disk_manifest.get('revision', -1)) != revision:
            raise ValueError('artifact manifest revision does not match published revision')
        if disk_manifest.get('frame_id') != 'map' or disk_manifest.get('units') != 'm':
            raise ValueError('artifact manifest must declare frame_id=map and units=m')
        if manifest and int(manifest.get('revision', -1)) != revision:
            raise ValueError('WebSocket manifest revision does not match published revision')
        base = root.resolve()
        for relative, expected in (disk_manifest.get('sha256') or {}).items():
            path = (root / str(relative)).resolve()
            if not path.is_relative_to(base):
                raise ValueError(f'artifact path escapes revision directory: {relative}')
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            if digest != expected:
                raise ValueError(f'artifact hash mismatch: {relative}')
        canonical = json.loads((root / 'canonical_map.json').read_text(encoding='utf-8'))
        if canonical.get('frame_id') != 'map' or int(canonical.get('revision', -1)) != revision:
            raise ValueError('canonical map revision/frame does not match manifest')
        return disk_manifest

    @staticmethod
    def _path_revision(path):
        try:
            path = path.resolve(strict=True)
            for parent in path.parents:
                manifest_path = parent / 'manifest.json'
                if not manifest_path.is_file():
                    continue
                manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
                if str(path).startswith(str(parent.resolve()) + os.sep):
                    return int(manifest['revision'])
        except (OSError, ValueError, KeyError, TypeError):
            return None
        return None

    def _request_map_reload(self, root, revision, manifest):
        self.manual_twist = Twist()
        self.manual_deadline = 0.0
        self.cmd_pub.publish(Twist())
        for active in (self.active_pose_goal, self.active_goal):
            if active is not None:
                try:
                    active.cancel_goal_async()
                except Exception as exc:
                    self.get_logger().warning(f'Could not cancel navigation before map reload: {exc}')
        payload = {'revision': int(revision), 'artifact_dir': str(root.resolve()),
                   'warehouse_id': manifest.get('warehouse_id'),
                   'generated_at': datetime.now(timezone.utc).isoformat()}
        request_path = self.map_sync_request_file
        request_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = request_path.with_suffix(request_path.suffix + '.tmp')
        temporary.write_text(json.dumps(payload, sort_keys=True) + '\n', encoding='utf-8')
        os.replace(temporary, request_path)

    def apply_published_map(self, data):
        """Validate a published bundle and request an owning-stack reload."""
        raw_revision = data.get('map_revision', data.get('revision'))
        try:
            revision = int(raw_revision)
        except (TypeError, ValueError):
            self.map_sync_status = 'ERROR'
            self.map_sync_error = 'published map revision is missing'
            self.send_map_revision_status()
            return
        artifact_dir = data.get('artifact_dir')
        candidate = Path(str(artifact_dir)).expanduser() if artifact_dir else None
        fallback = self.artifact_root / str(data.get('warehouse_code') or data.get('warehouse_id')) / str(revision)
        root = candidate if candidate is not None and candidate.is_dir() else fallback
        manifest = data.get('artifact_manifest') or {}
        names = manifest.get('artifacts') if isinstance(manifest, dict) else {}
        required = [str(names.get('canonical_map', 'canonical_map.json')),
                    str(names.get('datamatrix_map', 'datamatrix_map.yaml')),
                    str(names.get('tag_graph', 'tag_graph.yaml')),
                    str(names.get('gazebo_world', 'gazebo/warehouse.world')),
                    str(names.get('gazebo_manifest', 'gazebo/manifest.json')),
                    str(names.get('nav2_map', 'nav2/warehouse.yaml'))]
        missing = [name for name in required if not (root / name).is_file()]
        if missing:
            self.map_sync_status = 'ERROR'
            self.map_sync_error = f'artifacts missing for revision {revision}: {", ".join(missing)}'
            self.send_map_revision_status()
            return
        try:
            manifest = self._verify_artifact_bundle(root, manifest, revision)
        except (OSError, ValueError, TypeError) as exc:
            self.map_sync_status = 'ERROR'
            self.map_sync_error = str(exc)
            self.send_map_revision_status()
            return

        self.active_published_version = data.get('published_version')
        self.gazebo_revision = self._read_running_world_revision()
        self.ros_map_revision = self.gazebo_revision
        self.datamatrix_map_path = str(root / required[1])
        self.tag_graph_path = str(root / required[2])
        self.gazebo_world_path = str(root / required[3])
        self.nav2_configured_revision = self._path_revision(self.nav2_map_file)
        self.nav2_revision = self._loaded_nav2_revision(self.require_nav2_map)
        self.tag_map_configured_revision = revision if (
            self._path_revision(self.datamatrix_map_file) == revision
            and self._path_revision(self.tag_graph_file) == revision) else None
        self.tag_map_revision = None
        aligned = (self.ros_map_revision == revision and self.gazebo_revision == revision
                   and (not self.require_tag_map or self.tag_map_configured_revision == revision)
                   and (not self.require_nav2_map or self.nav2_configured_revision == revision))
        if not aligned:
            try:
                self._request_map_reload(root, revision, manifest)
                self.map_sync_status = 'SYNCING'
                self.map_sync_error = 'runtime consumers are loading a different map revision'
            except OSError as exc:
                self.map_sync_status = 'ERROR'
                self.map_sync_error = f'could not request runtime reload: {exc}'
        else:
            self.active_map_revision = revision
            self._refresh_map_sync_status()
        if self.map_sync_status not in ('SYNCED', 'SYNCING', 'ERROR'):
            self.map_sync_status = 'OUT_OF_SYNC'
        else:
            self.send({'type': 'MAP_REVISION_ACK', 'map_revision': revision,
                       'ros_revision': self.ros_map_revision,
                       'gazebo_revision': self.gazebo_revision,
                       'nav2_revision': self.nav2_revision,
                       'tag_map_revision': self.tag_map_revision,
                       'tf_status': self.tf_status,
                       'status': self.map_sync_status, 'artifact_dir': str(root)})
        self.send_map_revision_status()

    def process_commands(self):
        while True:
            try:
                data = self.incoming.get_nowait()
            except queue.Empty:
                return
            if not isinstance(data, dict):
                continue
            now = time.monotonic()
            while self.command_times and now - self.command_times[0] > 1.0:
                self.command_times.popleft()
            if len(self.command_times) >= 100:
                self.get_logger().warning('ROS bridge command rate limit exceeded; dropping command')
                self.send({'type': 'BRIDGE_ERROR', 'code': 'RATE_LIMITED',
                           'message': 'command rate limit exceeded', 'timestamp': self.now()})
                continue
            self.command_times.append(now)
            kind = str(data.get('type', '')).upper()
            try:
                if kind == 'MAP_PUBLISHED':
                    self.apply_published_map(data)
                elif kind in ('NAV_GOAL', 'TAG_NAV_GOAL'):
                    self.navigate(data)
                elif kind in ('CANCEL_NAVIGATION', 'TAG_NAV_CANCEL', 'TAG_NAV_PAUSE'):
                    self.cancel_navigation(data)
                elif kind == 'TAG_NAV_RESUME':
                    self.resume_navigation(data)
                elif kind == 'NAV_CANCEL':
                    self.cancel_navigation(data)
                elif kind == 'NAV_PAUSE':
                    data = {**data, 'type': 'TAG_NAV_PAUSE'}
                    self.cancel_navigation(data)
                elif kind == 'NAV_RESUME':
                    self.resume_navigation(data)
                elif kind == 'TAG_NAV_REPLAN':
                    self.cancel_navigation(data)
                elif kind == 'CONTROL_MODE':
                    self.set_control_mode(data)
                elif kind == 'MANUAL_CMD':
                    self.manual_command(data)
                elif kind == 'EMERGENCY_STOP':
                    self.emergency_stop(data)
                elif kind == 'CLEAR_EMERGENCY_STOP':
                    self.clear_emergency_stop(data)
            except Exception as exc:
                self.get_logger().error(f'ROS bridge command {kind} failed: {exc}')
                self.send({'type': 'BRIDGE_ERROR', 'code': 'COMMAND_FAILED',
                           'command': kind, 'message': str(exc)[:300], 'timestamp': self.now()})

    def emergency_stop(self, data):
        self.emergency_stop_active = True
        self.nav_state = 'EMERGENCY_STOPPED'
        self.manual_twist = Twist()
        self.manual_deadline = 0.0
        self.pending_cancel_state = 'EMERGENCY_STOPPED'
        self.pending_replan = None
        self.cmd_pub.publish(Twist())
        self.estop_pub.publish(Bool(data=True))
        if (self.active_goal is not None or self.active_pose_goal is not None) and not self.cancel_pending:
            self.cancel_pending = True
            try:
                active_handle = self.active_pose_goal or self.active_goal
                future = active_handle.cancel_goal_async()
                future.add_done_callback(lambda _future: setattr(self, 'cancel_pending', False))
            except Exception as exc:
                self.cancel_pending = False
                self.get_logger().warning(f'failed to cancel navigation during emergency stop: {exc}')
        self.send_nav_status({
            'robot_id': self.robot_id,
            'schedule_id': data.get('schedule_id'),
            'stop_id': data.get('stop_id'),
        }, 'EMERGENCY_STOPPED', 'emergency stop asserted')

    def clear_emergency_stop(self, _data):
        self.emergency_stop_active = False
        self.estop_pub.publish(Bool(data=False))
        self.nav_state = 'IDLE'
        self.manual_twist = Twist()
        self.manual_deadline = 0.0
        self.send_nav_status({'robot_id': self.robot_id}, 'IDLE', 'emergency stop cleared')

    def send_control_status(self, accepted: bool, reason=None):
        payload = {
            'type': 'ROBOT_CONTROL_STATUS', 'robot_id': self.robot_id,
            'mode': self.control_mode, 'accepted': bool(accepted),
            'timestamp': self.now(),
        }
        if reason:
            payload['reason'] = str(reason)[:300]
        self.send(payload)

    def set_control_mode(self, data):
        mode = str(data.get('mode') or '').upper()
        if mode not in ('MANUAL', 'AUTONOMOUS'):
            self.send_control_status(False, 'mode must be MANUAL or AUTONOMOUS')
            return
        if self.emergency_stop_active:
            self.send_control_status(False, 'emergency stop is active')
            return
        if mode == 'MANUAL' and (self.active_goal is not None or self.active_pose_goal is not None) and not self.cancel_pending:
            self.pending_cancel_state = 'CANCELLED'
            self.pending_replan = None
            self.cancel_pending = True
            try:
                active_handle = self.active_pose_goal or self.active_goal
                future = active_handle.cancel_goal_async()
                future.add_done_callback(lambda _future: setattr(self, 'cancel_pending', False))
            except Exception as exc:
                self.cancel_pending = False
                self.get_logger().warning(f'failed to cancel autonomous goal before manual mode: {exc}')
        self.control_mode = mode
        self.manual_twist = Twist()
        self.manual_deadline = 0.0
        self.cmd_pub.publish(Twist())
        self.nav_state = 'MANUAL' if mode == 'MANUAL' else 'IDLE'
        self.send_control_status(True)

    def manual_command(self, data):
        if self.emergency_stop_active:
            self.send_control_status(False, 'emergency stop is active')
            return
        if self.control_mode != 'MANUAL':
            self.send_control_status(False, 'switch to MANUAL mode before sending motion commands')
            return
        action = str(data.get('action') or '').upper()
        linear = float(self.get_parameter('manual_linear_velocity').value)
        angular = float(self.get_parameter('manual_angular_velocity').value)
        command = Twist()
        if action == 'FORWARD':
            command.linear.x = linear
        elif action == 'BACKWARD':
            command.linear.x = -linear
        elif action == 'LEFT':
            command.linear.y = linear
        elif action == 'RIGHT':
            command.linear.y = -linear
        elif action == 'ROTATE_LEFT':
            command.angular.z = angular
        elif action == 'ROTATE_RIGHT':
            command.angular.z = -angular
        elif action != 'STOP':
            self.send_control_status(False, f'unsupported manual action: {action}')
            return
        self.manual_twist = command
        self.manual_deadline = time.monotonic() + max(
            0.10, float(self.get_parameter('manual_command_timeout').value))
        self.nav_state = 'MANUAL' if action != 'STOP' else 'IDLE'
        self.send_control_status(True)

    def navigate(self, data):
        if data.get('x') is not None and data.get('y') is not None:
            self.navigate_pose(data)
            return
        context = {
            'schedule_id': data.get('schedule_id'),
            'stop_id': data.get('stop_id'),
            'robot_id': self.robot_id,
            'target_tag_id': data.get('target_tag_id'),
        }
        if self.emergency_stop_active:
            self.send_nav_status(context, 'FAILED', 'emergency stop is active')
            return
        if self.control_mode != 'AUTONOMOUS':
            self.send_nav_status(context, 'FAILED', 'switch to AUTONOMOUS mode before navigation')
            return
        if self.active_goal is not None or self.active_pose_goal is not None or self.goal_request_pending:
            self.send_nav_status(context, 'FAILED', 'another tag-navigation goal is already active')
            return
        if data.get('target_tag_id') is None:
            self.send_nav_status(context, 'FAILED', 'target_tag_id is required; x/y navigation is not supported')
            return
        try:
            target_tag_id = int(data['target_tag_id'])
        except (TypeError, ValueError):
            self.send_nav_status(context, 'FAILED', 'target_tag_id must be an integer')
            return
        timeout = float(self.get_parameter('navigate_server_timeout').value)
        if not self.nav_client.wait_for_server(timeout_sec=timeout):
            self.nav_state = 'FAILED'
            self.send_nav_status(
                context, 'FAILED',
                f'GoToTag action server unavailable after {timeout:.1f}s',
            )
            return
        goal = GoToTag.Goal()
        goal.target_tag_id = target_tag_id
        self.nav_state = 'PENDING'
        self.goal_request_pending = True
        try:
            future = self.nav_client.send_goal_async(goal)
            future.add_done_callback(lambda f: self.goal_response(f, context))
        except Exception as exc:
            self.goal_request_pending = False
            self.nav_state = 'FAILED'
            self.send_nav_status(context, 'FAILED', f'failed to send Nav2 goal: {exc}')

    def navigate_pose(self, data):
        context = {
            'robot_id': self.robot_id, 'x': float(data.get('x')), 'y': float(data.get('y')),
            'yaw': float(data.get('yaw') or 0.0), 'frame_id': str(data.get('frame_id') or 'map'),
        }
        if self.emergency_stop_active:
            self.send_nav_status(context, 'FAILED', 'emergency stop is active')
            return
        if self.control_mode != 'AUTONOMOUS':
            self.send_nav_status(context, 'FAILED', 'switch to AUTONOMOUS mode before navigation')
            return
        if self.active_goal is not None or self.active_pose_goal is not None or self.goal_request_pending:
            self.send_nav_status(context, 'FAILED', 'another navigation goal is already active')
            return
        timeout = float(self.get_parameter('navigate_server_timeout').value)
        if not self.nav_pose_client.wait_for_server(timeout_sec=timeout):
            self.nav_state = 'FAILED'
            self.send_nav_status(context, 'FAILED', f'NavigateToPose action server unavailable after {timeout:.1f}s')
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = context['frame_id']
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = context['x']
        goal.pose.pose.position.y = context['y']
        goal.pose.pose.orientation.z = math.sin(context['yaw'] / 2.0)
        goal.pose.pose.orientation.w = math.cos(context['yaw'] / 2.0)
        self.nav_state = 'PENDING'
        self.goal_request_pending = True
        try:
            future = self.nav_pose_client.send_goal_async(goal)
            future.add_done_callback(lambda f: self.pose_goal_response(f, context))
        except Exception as exc:
            self.goal_request_pending = False
            self.nav_state = 'FAILED'
            self.send_nav_status(context, 'FAILED', f'failed to send NavigateToPose goal: {exc}')

    def send_nav_status(self, context, status, reason=None):
        payload = {
            'type': 'NAV_STATUS', **context, 'status': status,
            'timestamp': self.now(),
        }
        if reason:
            payload['reason'] = reason
        self.send(payload)

    def goal_response(self, future, context):
        self.goal_request_pending = False
        try:
            handle = future.result()
        except Exception as exc:
            self.nav_state = 'FAILED'
            self.send_nav_status(context, 'FAILED', f'GoToTag goal request failed: {exc}')
            return
        if handle is None or not handle.accepted:
            self.nav_state = 'FAILED'
            self.send_nav_status(context, 'FAILED', 'GoToTag rejected the goal')
            return
        self.active_goal = handle
        self.active_context = context
        self.cancel_pending = False
        self.pending_cancel_state = None
        self.nav_state = 'NAVIGATING'
        self.send_nav_status(context, 'ACTIVE')
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f: self.goal_result(f, handle, context))

    def pose_goal_response(self, future, context):
        self.goal_request_pending = False
        try:
            handle = future.result()
        except Exception as exc:
            self.nav_state = 'FAILED'
            self.send_nav_status(context, 'FAILED', f'NavigateToPose request failed: {exc}')
            return
        if handle is None or not handle.accepted:
            self.nav_state = 'FAILED'
            self.send_nav_status(context, 'FAILED', 'NavigateToPose rejected the goal')
            return
        self.active_pose_goal = handle
        self.active_pose_context = context
        self.paused_pose_context = None
        self.cancel_pending = False
        self.pending_cancel_state = None
        self.nav_state = 'NAVIGATING'
        self.send({'type': 'NAV_GOAL', 'goal': {
            **context, 'status': 'ACTIVE', 'timestamp': self.now(),
        }})
        self.send_nav_status(context, 'ACTIVE')
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f: self.pose_goal_result(f, handle, context))

    def pose_goal_result(self, future, handle, context):
        try:
            response = future.result()
            status_code = response.status
        except Exception as exc:
            if self.active_pose_goal is handle:
                self.active_pose_goal = None
                self.active_pose_context = None
                self.cancel_pending = False
            self.nav_state = 'FAILED'
            self.send_nav_status(context, 'FAILED', f'NavigateToPose result failed: {exc}')
            return
        status = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED: 'FAILED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
        }.get(status_code, 'FAILED')
        pending_cancel_state = self.pending_cancel_state
        self.pending_cancel_state = None
        self.pending_replan = None
        if self.emergency_stop_active:
            status = 'EMERGENCY_STOPPED'
        elif status == 'CANCELED' and pending_cancel_state:
            status = pending_cancel_state
        self.nav_state = 'IDLE' if status == 'SUCCEEDED' else status
        if status == 'PAUSED':
            self.paused_pose_context = context
        if self.active_pose_goal is handle:
            self.active_pose_goal = None
            self.active_pose_context = None
            self.cancel_pending = False
        reason = None if status != 'FAILED' else f'NavigateToPose finished with status code {status_code}'
        self.send_nav_status(context, status, reason)

    def goal_result(self, future, handle, context):
        try:
            response = future.result()
            status_code = response.status
        except Exception as exc:
            if self.active_goal is handle:
                self.active_goal = None
                self.active_context = None
                self.cancel_pending = False
            self.nav_state = 'FAILED'
            self.send_nav_status(context, 'FAILED', f'Nav2 result failed: {exc}')
            return
        status = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED: 'FAILED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
        }.get(status_code, 'FAILED')
        pending_cancel_state = self.pending_cancel_state
        pending_replan = self.pending_replan
        self.pending_cancel_state = None
        self.pending_replan = None
        if self.emergency_stop_active:
            status = 'EMERGENCY_STOPPED'
        elif status == 'CANCELED' and pending_cancel_state:
            status = pending_cancel_state
        self.nav_state = 'IDLE' if status == 'SUCCEEDED' else status
        if self.active_goal is handle:
            self.active_goal = None
            self.active_context = None
            self.cancel_pending = False
        reason = None if status != 'FAILED' else f'GoToTag finished with status code {status_code}'
        self.send_nav_status(context, status, reason)
        if pending_replan and status == 'PLANNING' and not self.emergency_stop_active:
            # A replan is a cancel-then-new-goal operation because the action
            # server accepts only one active route.  The new goal is submitted
            # only after Nav2 has confirmed cancellation of the old segment.
            self.navigate(pending_replan)

    def cancel_navigation(self, data):
        kind = str(data.get('type') or '').upper()
        is_replan = kind == 'TAG_NAV_REPLAN'
        cancel_state = 'PLANNING' if is_replan else 'PAUSED' if kind == 'TAG_NAV_PAUSE' else 'CANCELLED'
        context = self.active_pose_context or self.active_context or self.paused_pose_context or {
            'robot_id': self.robot_id,
            'schedule_id': data.get('schedule_id'),
            'stop_id': data.get('stop_id'),
            'target_tag_id': data.get('target_tag_id'),
        }
        if is_replan and not data.get('target_tag_id'):
            data = {**data, 'target_tag_id': context.get('target_tag_id')}
        if self.active_pose_goal is not None:
            self.cancel_pose_navigation(context, cancel_state)
            return
        if self.active_goal is None:
            self.cmd_pub.publish(Twist())
            if is_replan and data.get('target_tag_id') is not None:
                self.navigate(data)
            else:
                self.nav_state = cancel_state
                self.send_nav_status(context, cancel_state, 'no accepted Nav2 goal; state updated locally')
            return
        if self.cancel_pending:
            return
        self.pending_cancel_state = cancel_state
        self.pending_replan = dict(data) if is_replan else None
        self.cancel_pending = True
        try:
            future = self.active_goal.cancel_goal_async()
            future.add_done_callback(lambda f: self.cancel_response(f, context))
        except Exception as exc:
            self.cancel_pending = False
            self.pending_cancel_state = None
            self.pending_replan = None
            self.send_nav_status(context, 'FAILED', f'failed to request Nav2 cancellation: {exc}')

    def cancel_pose_navigation(self, context, cancel_state):
        if self.active_pose_goal is None:
            self.cmd_pub.publish(Twist())
            self.nav_state = cancel_state
            self.send_nav_status(context, cancel_state, 'no accepted NavigateToPose goal; state updated locally')
            return
        if self.cancel_pending:
            return
        self.pending_cancel_state = cancel_state
        self.pending_replan = None
        self.cancel_pending = True
        try:
            future = self.active_pose_goal.cancel_goal_async()
            future.add_done_callback(lambda f: self.cancel_response(f, context))
        except Exception as exc:
            self.cancel_pending = False
            self.pending_cancel_state = None
            self.send_nav_status(context, 'FAILED', f'failed to request NavigateToPose cancellation: {exc}')

    def cancel_response(self, future, context):
        try:
            response = future.result()
            accepted = bool(response.goals_canceling)
        except Exception as exc:
            self.cancel_pending = False
            self.pending_replan = None
            self.pending_cancel_state = None
            self.send_nav_status(context, 'FAILED', f'Nav2 cancellation request failed: {exc}')
            return
        if not accepted:
            self.cancel_pending = False
            self.pending_replan = None
            self.pending_cancel_state = None
            self.send_nav_status(context, 'FAILED', 'Nav2 rejected the cancellation request')
        # A successful request is not completion. goal_result() reports
        # CANCELED only after Nav2 returns STATUS_CANCELED.

    def resume_navigation(self, data):
        """Resume a paused mission by submitting its approved tag goal again."""
        if self.active_pose_goal is not None:
            self.send_nav_status(self.active_pose_context or {'robot_id': self.robot_id}, 'ACTIVE')
            return
        if self.active_goal is not None or self.goal_request_pending:
            self.send_nav_status(self.active_context or {'robot_id': self.robot_id}, 'ACTIVE')
            return
        if self.paused_pose_context and self.paused_pose_context.get('x') is not None:
            self.navigate(self.paused_pose_context)
            return
        target = data.get('target_tag_id')
        if target is None and self.active_context:
            target = self.active_context.get('target_tag_id')
        if target is None:
            self.send_nav_status({'robot_id': self.robot_id}, 'FAILED', 'target_tag_id is required to resume')
            return
        self.navigate({**data, 'target_tag_id': target})

    def destroy_node(self):
        self.stop_event.set()
        self.cmd_pub.publish(Twist())
        with self.ws_lock:
            if self.ws is not None:
                self.ws.close()
                self.ws = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SwerveBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        # Ctrl-C can already have shut down the default context through the
        # executor. Avoid turning a normal stop into an RCLError traceback.
        if rclpy.ok():
            rclpy.shutdown()
