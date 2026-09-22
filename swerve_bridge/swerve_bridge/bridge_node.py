from __future__ import annotations

import json
import math
import queue
import threading
import os
import time
from collections import deque
from pathlib import Path
from urllib.parse import quote
from datetime import datetime, timezone

import rclpy
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from swerve_bringup.action import GoToTag
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SwerveBridge(Node):
    def __init__(self):
        super().__init__('swerve_bridge')
        self.declare_parameter('robot_id', 'R01')
        self.declare_parameter('namespace', '')
        self.declare_parameter('runtime_state', 'MAPPING')
        self.declare_parameter('django_ws_url', os.environ.get('ROS_WS_URL', 'ws://127.0.0.1:8000/ws/ros'))
        self.declare_parameter('django_token', '')
        self.declare_parameter('telemetry_rate', 10.0)
        self.declare_parameter('heartbeat_rate', 1.0)
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('lidar_topic', '/lidar/points')
        self.declare_parameter('clock_topic', '/clock')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('emergency_stop_topic', '/emergency_stop')
        self.declare_parameter('navigate_action', '/go_to_tag')
        self.declare_parameter('navigate_server_timeout', 2.0)
        self.declare_parameter('manual_linear_velocity', 0.25)
        self.declare_parameter('manual_angular_velocity', 0.60)
        self.declare_parameter('manual_command_timeout', 0.40)
        self.declare_parameter('artifact_root', 'generated/maps')
        self.declare_parameter('gazebo_world_file', '')
        self.declare_parameter('allow_unpublished_fallback', False)
        self.robot_id = str(self.get_parameter('robot_id').value)
        self.namespace = self._normalize_namespace(self.get_parameter('namespace').value)
        self.runtime_state = str(self.get_parameter('runtime_state').value).upper()
        if self.runtime_state not in ('IDLE', 'SIMULATION', 'MAPPING', 'NAVIGATION', 'ERROR'):
            self.get_logger().warning(f'Unknown runtime_state={self.runtime_state}; using MAPPING')
            self.runtime_state = 'MAPPING'
        configured_ws_url = str(self.get_parameter('django_ws_url').value).strip()
        self.ws_url = configured_ws_url or os.environ.get(
            'ROS_WS_URL', 'ws://127.0.0.1:8000/ws/ros')
        self.token = str(self.get_parameter('django_token').value)
        self.artifact_root = Path(str(self.get_parameter('artifact_root').value)).expanduser()
        self.gazebo_world_file = Path(str(self.get_parameter('gazebo_world_file').value)).expanduser()
        self.allow_unpublished_fallback = bool(self.get_parameter('allow_unpublished_fallback').value)
        self.telemetry_period = 1.0 / max(0.1, float(self.get_parameter('telemetry_rate').value))
        self.latest_odom = None
        self.last_joint_state = None
        self.last_lidar_monotonic = None
        self.last_lidar_frame_id = None
        self.last_lidar_stamp = None
        self.lidar_intervals = deque(maxlen=20)
        self.last_clock_monotonic = None
        self.simulation_time = None
        self.nav_state = 'IDLE'
        self.control_mode = 'AUTONOMOUS'
        self.manual_twist = Twist()
        self.manual_deadline = 0.0
        self.emergency_stop_active = False
        self.active_goal = None
        self.active_context = None
        self.goal_request_pending = False
        self.cancel_pending = False
        self.pending_cancel_state = None
        self.pending_replan = None
        self.incoming = queue.Queue()
        self.ws = None
        self.ws_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.active_map_revision = None
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
        clock_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Clock, clock_topic, self.clock_cb, clock_qos)
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
        self.create_timer(self.telemetry_period, self.telemetry_timer)
        self.create_timer(1.0 / max(0.1, float(self.get_parameter('heartbeat_rate').value)), self.heartbeat_timer)
        self.create_timer(0.05, self.manual_timer)
        self.create_timer(0.05, self.process_commands)
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

    def clock_cb(self, msg):
        self.last_clock_monotonic = time.monotonic()
        stamp = msg.clock
        self.simulation_time = float(stamp.sec) + float(stamp.nanosec) * 1e-9

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
        o, t = msg.pose.pose, msg.twist.twist
        self.send({'type': 'ROBOT_STATE', 'robot_id': self.robot_id,
                   'x': o.position.x, 'y': o.position.y, 'z': o.position.z,
                   'yaw': yaw_from_quaternion(o.orientation), 'vx': t.linear.x,
                   'vy': t.linear.y, 'wz': t.angular.z, 'navigation_state': self.nav_state,
                   'control_mode': self.control_mode,
                   'timestamp': self.now()})

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
        self.send({'type': 'ROS_DIAGNOSTICS', 'robot_id': self.robot_id,
                   'diagnostics': self.collect_diagnostics(), 'timestamp': self.now()})
        self.send_map_revision_status()

        with self.ws_lock:
            ws = self.ws
            if ws is not None:
                try:
                    ws.ping()
                except Exception as exc:
                    self.get_logger().warning(f'ROS bridge heartbeat ping failed: {exc}')

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
        lidar_frequency = None
        if self.last_lidar_monotonic is not None:
            lidar_age = max(0.0, time.monotonic() - self.last_lidar_monotonic)
        if self.lidar_intervals:
            average = sum(self.lidar_intervals) / len(self.lidar_intervals)
            if average > 0.0:
                lidar_frequency = 1.0 / average
        gazebo = (
            any('gazebo' in name.lower() for name in node_names)
            or (self.last_clock_monotonic is not None and time.monotonic() - self.last_clock_monotonic <= 3.0)
        )
        slam = any('slam_toolbox' in name.lower() for name in node_names)
        nav2_nodes = ('map_server', 'amcl', 'controller_server', 'planner_server',
                      'bt_navigator', 'lifecycle_manager_navigation')
        nav2 = any(any(marker in name for marker in nav2_nodes) for name in node_names)
        tf = self._scoped_topic('/tf') in topic_names and self._scoped_topic('/tf_static') in topic_names
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
            'metrics': {
                'lidar_age_s': lidar_age,
                'lidar_frequency_hz': lidar_frequency,
                'lidar_frame_id': self.last_lidar_frame_id,
                'lidar_stamp': self.last_lidar_stamp,
                # DDS does not expose dropped samples through this API. Keep
                # this explicitly unknown instead of presenting fabricated 0.
                'lidar_dropped_messages': None,
            },
        }

    def send_map_revision_status(self):
        self.send({
            'type': 'MAP_REVISION_STATUS',
            'map_revision': self.active_map_revision,
            'published_version': self.active_published_version,
            'gazebo_revision': self.gazebo_revision,
            'status': self.map_sync_status,
            'error': self.map_sync_error,
        })

    def apply_published_map(self, data):
        """Load the immutable ROS artifacts selected by the backend."""
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
        required = [str(names.get('datamatrix_map', 'datamatrix_map.yaml')), str(names.get('tag_graph', 'tag_graph.yaml')), str(names.get('gazebo_world', 'gazebo/warehouse.world'))]
        missing = [name for name in required if not (root / name).is_file()]
        if missing:
            if self.allow_unpublished_fallback:
                self.map_sync_status = 'ERROR'
                self.map_sync_error = f'artifacts missing (development fallback enabled): {", ".join(missing)}'
            else:
                self.map_sync_status = 'ERROR'
                self.map_sync_error = f'artifacts missing for revision {revision}: {", ".join(missing)}'
            self.send_map_revision_status()
            return
        # Reading the files here makes the bridge fail fast before acknowledging
        # a revision.  The ROS navigation nodes can consume the same paths from
        # the selected immutable artifact directory.
        try:
            for name in required:
                (root / name).read_bytes()
        except OSError as exc:
            self.map_sync_status = 'ERROR'
            self.map_sync_error = str(exc)
            self.send_map_revision_status()
            return
        self.active_map_revision = revision
        self.active_published_version = data.get('published_version')
        # Keep the world revision measured from the launch.  If this process is
        # attached to a package fallback world, report OUT_OF_SYNC rather than
        # claiming that Gazebo live-reloaded the newly published geometry.
        self.gazebo_revision = self._read_running_world_revision()
        self.datamatrix_map_path = str(root / required[0])
        self.tag_graph_path = str(root / required[1])
        self.gazebo_world_path = str(root / required[2])
        self.map_sync_error = None
        self.map_sync_status = 'SYNCED' if self.gazebo_revision == revision else 'OUT_OF_SYNC'
        self.send({'type': 'MAP_REVISION_ACK', 'map_revision': revision,
                   'published_version': self.active_published_version,
                   'gazebo_revision': self.gazebo_revision,
                   'status': self.map_sync_status, 'artifact_dir': str(root)})

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
        if self.active_goal is not None and not self.cancel_pending:
            self.cancel_pending = True
            try:
                future = self.active_goal.cancel_goal_async()
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
        if mode == 'MANUAL' and self.active_goal is not None and not self.cancel_pending:
            self.pending_cancel_state = 'CANCELLED'
            self.pending_replan = None
            self.cancel_pending = True
            try:
                future = self.active_goal.cancel_goal_async()
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
        if self.active_goal is not None or self.goal_request_pending:
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
        context = self.active_context or {
            'robot_id': self.robot_id,
            'schedule_id': data.get('schedule_id'),
            'stop_id': data.get('stop_id'),
            'target_tag_id': data.get('target_tag_id'),
        }
        if is_replan and not data.get('target_tag_id'):
            data = {**data, 'target_tag_id': context.get('target_tag_id')}
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
        if self.active_goal is not None or self.goal_request_pending:
            self.send_nav_status(self.active_context or {'robot_id': self.robot_id}, 'ACTIVE')
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
