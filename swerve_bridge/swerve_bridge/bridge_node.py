from __future__ import annotations

import json
import hashlib
import math
import queue
from .control_mailbox import ControlMailbox
from .outbound_mailbox import OutboundMailbox
from .visualization_worker import VisualizationWorker
import threading
import os
import re
import time
import uuid
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
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import LoadMap
from rosgraph_msgs.msg import Clock
from swerve_bringup.action import GoToTag
from sensor_msgs.msg import JointState, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from tf2_ros import Buffer, TransformException, TransformListener
from robot_localization.srv import SetPose
from slam_toolbox.srv import (Pause as SlamPause, SaveMap as SlamSaveMap,
                              SerializePoseGraph as SlamSerializePoseGraph)

from .qos import canonical_map_qos_profile, gazebo_clock_qos_profile
from .coordinates import (is_small_future_tf_skew, pose_from_transform,
                          quaternion_yaw, rotate_translate_xy)
from .web_map_renderer import (
    LatestFrameBuffer, bounded_cloud_points, compress_occupancy_grid,
    laser_scan_xy, path_length, successful_path_result,
    transform_points_xyz,
    occupancy_content_signature,
    occupancy_grid_statistics,
)


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
        self.declare_parameter('lidar_filtered_topic', '/lidar/points_filtered')
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
        self.declare_parameter('lidar_web_3d_hz', 3.0)
        self.declare_parameter('lidar_max_3d_points', 4000)
        self.declare_parameter('lidar_3d_voxel_size', 0.04)
        self.declare_parameter('lidar_3d_min_range', 0.15)
        self.declare_parameter('lidar_3d_max_range', 25.0)
        self.declare_parameter('lidar_3d_min_height', -1.0)
        self.declare_parameter('lidar_3d_max_height', 3.0)
        self.declare_parameter('clock_topic', '/clock')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_manual')
        self.declare_parameter('control_mode_topic', '/robot_control_mode')
        self.declare_parameter('emergency_stop_topic', '/emergency_stop')
        self.declare_parameter('navigate_action', '/go_to_tag')
        self.declare_parameter('navigate_pose_action', '/navigate_to_pose')
        self.declare_parameter('navigate_server_timeout', 2.0)
        self.declare_parameter('manual_linear_velocity', 0.25)
        self.declare_parameter('manual_angular_velocity', 0.60)
        self.declare_parameter('manual_command_timeout', 0.40)
        self.declare_parameter('artifact_root', 'generated/maps')
        self.declare_parameter('local_map_root', os.path.join(
            os.environ.get('WARETWIN_ARTIFACT_ROOT') or os.path.join(os.getcwd(), 'generated', 'maps'),
            'local_robot_maps'))
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
        self.local_map_root = (Path(str(self.get_parameter('local_map_root').value)).expanduser()
                               / self.robot_id).resolve()
        self.gazebo_world_file = Path(str(self.get_parameter('gazebo_world_file').value)).expanduser()
        self.nav2_map_file = Path(str(self.get_parameter('nav2_map_file').value)).expanduser()
        self.datamatrix_map_file = Path(str(self.get_parameter('datamatrix_map_file').value)).expanduser()
        self.tag_graph_file = Path(str(self.get_parameter('tag_graph_file').value)).expanduser()
        self.map_sync_request_file = Path(str(self.get_parameter('map_sync_request_file').value)).expanduser()
        self.allow_unpublished_fallback = bool(self.get_parameter('allow_unpublished_fallback').value)
        self.telemetry_period = 1.0 / max(0.1, float(self.get_parameter('telemetry_rate').value))
        self.latest_odom = None
        self.last_odom_monotonic = None
        self.last_odom_frame_id = None
        self.last_odom_child_frame_id = None
        self.odom_intervals = deque(maxlen=30)
        self.last_joint_state = None
        self.last_lidar_monotonic = None
        self.last_lidar_frame_id = None
        self.last_lidar_stamp = None
        self.lidar_intervals = deque(maxlen=20)
        self.scan_intervals = deque(maxlen=20)
        self.scan_sim_intervals = deque(maxlen=20)
        self.map_intervals = deque(maxlen=30)
        self.last_scan_monotonic = None
        self.last_scan_stamp = None
        self.last_scan_frame_id = None
        self.last_map_monotonic = None
        self.last_scan_map_tf_valid = False
        self.last_scan_map_tf_error = 'waiting for map <- scan transform'
        self.latest_scan = None
        self.latest_cloud = None
        self.latest_filtered_cloud = None
        self.latest_cloud_monotonic = None
        self.latest_filtered_cloud_monotonic = None
        self.raw_cloud_intervals = deque(maxlen=30)
        self.filtered_cloud_intervals = deque(maxlen=30)
        self.last_raw_cloud_monotonic = None
        self.last_filtered_cloud_monotonic = None
        self.last_scan_publish_monotonic = 0.0
        self.last_cloud_publish_monotonic = 0.0
        self.cloud_intervals = deque(maxlen=20)
        self.last_cloud_frame_wall = None
        self.web_cloud_revision = 0
        self.last_web_cloud_source_stamp = None
        self.lidar_frame_buffer = LatestFrameBuffer()
        self.detail_view = 'GLOBAL'
        self.detail_view_epoch = 0
        self.view_timing = {}
        self.last_detail_callback_wall = None
        self.detail_callback_intervals = deque(maxlen=30)
        self.visualization_metrics = {'map_compressions': 0}
        self.last_lidar_send_monotonic = None
        self.lidar_output_intervals = deque(maxlen=30)
        self.last_lidar_output_metadata = {}
        self.web_cloud_epoch = f'{self.robot_id}-{time.time_ns()}'
        self.latest_map = None
        self.latest_map_signature = None
        self.latest_map_generation = 0
        self.processed_map_generation = 0
        self.processed_map = None
        self.processed_map_payload = None
        self.processed_map_payload_signature = None
        self.latest_map_received_monotonic = None
        self.mapping_session_id = uuid.uuid4().hex[:12]
        self.mapping_map_revision = None
        self.mapping_map_version = 0
        self.latest_map_statistics = None
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
        self.requested_mode = 'AUTONOMOUS'
        self.applied_mode = 'AUTONOMOUS'
        self.mode_transition_state = 'APPLIED'
        self.mode_request_id = None
        self.mode_request_started = 0.0
        self.manual_generation = 0
        self.manual_timing_enabled = os.environ.get('WARETWIN_MANUAL_TIMING') == '1'
        self.control_timing = {}
        self.manual_twist = Twist()
        self.manual_deadline = 0.0
        self.emergency_stop_active = False
        self.active_goal = None
        self.active_pose_goal = None
        self.active_context = None
        self.active_pose_context = None
        self.path_preview_goals = {}
        self.active_vda_order = None
        self.slam_paused = False
        self.slam_mapping_elapsed_s = 0.0
        self.slam_mapping_started_monotonic = time.monotonic() if self.runtime_state == 'MAPPING' else None
        self.loaded_local_map_id = None
        self.loaded_local_map_revision = None
        self.local_map_load_pending = False
        self.pending_local_map_load = None
        self.pending_initial_pose = None
        self.map_callback_count = 0
        self.path_preview_approvals = {}
        self.paused_pose_context = None
        self.goal_request_pending = False
        self.cancel_pending = False
        self.pending_cancel_state = None
        self.pending_replan = None
        self.incoming = ControlMailbox()
        self.ws = None
        self.ws_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.outbound = OutboundMailbox()
        self.outbound_disconnect = threading.Event()
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
        lidar_filtered_topic = self._scoped_topic(self.get_parameter('lidar_filtered_topic').value)
        clock_topic = self._scoped_topic(self.get_parameter('clock_topic').value)
        cmd_vel_topic = self._scoped_topic(self.get_parameter('cmd_vel_topic').value)
        control_mode_topic = self._scoped_topic(self.get_parameter('control_mode_topic').value)
        emergency_stop_topic = self._scoped_topic(self.get_parameter('emergency_stop_topic').value)
        navigate_action = self._scoped_topic(self.get_parameter('navigate_action').value)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 20)
        self.create_subscription(JointState, joint_states_topic, self.joint_cb, 10)
        self.create_subscription(PointCloud2, lidar_topic, self.lidar_cb, 10)
        self.create_subscription(PointCloud2, lidar_filtered_topic, self.filtered_lidar_cb, 1)
        # Gazebo Classic publishes /clock as best-effort + volatile.  The
        # default rclpy profile is reliable, which is incompatible and leaves
        # simulation_time permanently null.  Match the Gazebo profile without
        # changing the publisher or any robot-control topic.
        self.create_subscription(Clock, clock_topic, self.clock_cb, gazebo_clock_qos_profile())
        self.cmd_pub = self.create_publisher(
            Twist, cmd_vel_topic, 20)
        mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.control_mode_pub = self.create_publisher(String, control_mode_topic, mode_qos)
        mode_message = String()
        mode_message.data = self.control_mode
        self.control_mode_pub.publish(mode_message)
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
        self.path_preview_client = ActionClient(
            self, ComputePathToPose, self._scoped_topic('/compute_path_to_pose'))
        self.map_load_client = self.create_client(
            LoadMap, self._scoped_topic('/map_server/load_map'))
        self.initial_pose_client = self.create_client(
            SetPose, self._scoped_topic('/set_pose'))
        self.slam_pause_client = self.create_client(
            SlamPause, self._scoped_topic('/slam_toolbox/pause_new_measurements'))
        self.slam_save_client = self.create_client(
            SlamSaveMap, self._scoped_topic('/slam_toolbox/save_map'))
        self.slam_serialize_client = self.create_client(
            SlamSerializePoseGraph, self._scoped_topic('/slam_toolbox/serialize_map'))
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
        self.command_diagnostics = {}
        self.create_subscription(
            String, self._scoped_topic('/command_arbiter/diagnostics'),
            self.command_diagnostics_cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Visualization has its own monotonic wall-clock worker. Sensor ROS
        # callbacks only replace the latest references; control keeps its ROS
        # executor and never waits for cloud transforms/map compression.
        self.visualization_worker = VisualizationWorker(self.detail_timer,
            lambda: 1.0 / max(.1, float(self.get_parameter(
                'lidar_web_3d_hz' if self.detail_view == 'LIDAR_3D' else 'lidar_ui_hz').value)),
            lambda exc: self.get_logger().warning(f'visualization preparation failed: {type(exc).__name__}'))
        self.map_snapshot_worker = VisualizationWorker(self.map_snapshot_timer,
            lambda: 3600.0,
            lambda exc: self.get_logger().warning(f'map snapshot preparation failed: {type(exc).__name__}'),
            name='web-map-snapshot')
        self.diagnostics_worker = VisualizationWorker(self.diagnostics_timer,
            lambda: 1.0 / max(.1, float(self.get_parameter('heartbeat_rate').value)),
            lambda exc: self.get_logger().warning(f'diagnostics preparation failed: {type(exc).__name__}'),
            name='web-diagnostics')
        self.telemetry_pending = threading.Event()
        self.create_timer(self.telemetry_period, self.request_telemetry,
                          clock=self._wall_clock)
        self.create_timer(0.1, self.poll_local_control_confirmations,
                          clock=self._wall_clock)
        self.create_timer(
            1.0 / max(0.1, float(self.get_parameter('heartbeat_rate').value)),
            self.profile_callback(self.heartbeat_timer), clock=self._wall_clock)
        self.create_timer(0.05, self.manual_timer, clock=self._wall_clock)
        self.create_timer(0.05, self.process_commands, clock=self._wall_clock)
        self.outbound_thread = threading.Thread(target=self.outbound_sender, daemon=True)
        self.outbound_thread.start()
        self.visualization_worker.start()
        self.map_snapshot_worker.start()
        self.diagnostics_worker.start()
        self.thread = threading.Thread(target=self.websocket_loop, daemon=True)
        self.thread.start()

    @staticmethod
    def _normalize_namespace(value) -> str:
        return str(value or '').strip().strip('/')

    def profile_callback(self, callback):
        if not self.manual_timing_enabled:
            return callback
        def measured():
            started = time.monotonic()
            try:
                return callback()
            finally:
                elapsed = time.monotonic() - started
                key = callback.__name__ + '_max_duration'
                self.control_timing[key] = max(self.control_timing.get(key, 0), elapsed)
                if elapsed > 0.1:
                    self.get_logger().info('CALLBACK_TIMING ' + json.dumps({
                        'callback': callback.__name__, 'duration': elapsed,
                        'started': started, 'finished': time.monotonic()}))
        return measured

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
        now = time.monotonic()
        if self.last_odom_monotonic is not None and now > self.last_odom_monotonic:
            self.odom_intervals.append(now - self.last_odom_monotonic)
        self.last_odom_monotonic = now
        self.last_odom_frame_id = str(msg.header.frame_id or '')
        self.last_odom_child_frame_id = str(msg.child_frame_id or '')
        self.latest_odom = msg

    def joint_cb(self, msg):
        self.last_joint_state = msg

    def lidar_cb(self, msg):
        now = time.monotonic()
        if self.last_raw_cloud_monotonic is not None and now > self.last_raw_cloud_monotonic:
            self.raw_cloud_intervals.append(now - self.last_raw_cloud_monotonic)
        self.last_raw_cloud_monotonic = now
        if self.last_lidar_monotonic is not None:
            interval = now - self.last_lidar_monotonic
            if interval > 0.0:
                self.lidar_intervals.append(interval)
        self.last_lidar_monotonic = now
        self.last_lidar_frame_id = str(msg.header.frame_id or '')
        stamp = msg.header.stamp
        self.last_lidar_stamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        self.latest_cloud = msg
        self.latest_cloud_monotonic = now

    def filtered_lidar_cb(self, msg):
        now = time.monotonic()
        if self.last_filtered_cloud_monotonic is not None and now > self.last_filtered_cloud_monotonic:
            self.filtered_cloud_intervals.append(now - self.last_filtered_cloud_monotonic)
        self.last_filtered_cloud_monotonic = now
        self.latest_filtered_cloud = msg
        self.latest_filtered_cloud_monotonic = now
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
        self.last_scan_frame_id = str(msg.header.frame_id or '')
        self.latest_scan = msg
        self.last_lidar_monotonic = now
        self.last_lidar_frame_id = str(msg.header.frame_id or '')
        self.last_lidar_stamp = stamp_value

    def map_cb(self, msg):
        # Keep the executor callback O(1): hashing, map validation, statistics,
        # compression and WebSocket serialization run on the isolated bounded
        # map worker, never alongside manual/E-STOP servicing.
        now = time.monotonic()
        if self.last_map_monotonic is not None and now > self.last_map_monotonic:
            self.map_intervals.append(now - self.last_map_monotonic)
        self.last_map_monotonic = now
        self.latest_map = msg
        self.latest_map_received_monotonic = now
        self.latest_map_generation += 1
        self.map_callback_count += 1
        self.map_snapshot_worker.wake()

    def _update_latest_map_signature(self):
        generation = self.latest_map_generation
        msg = self.latest_map
        if generation != self.latest_map_generation:
            return
        if msg is None:
            return
        origin = msg.info.origin
        if generation == self.processed_map_generation and self.latest_map_signature:
            # The active map identity can change after /map is received (for
            # example once LoadMap's raster is validated). Reuse the previous
            # content hash here instead of hashing a large OccupancyGrid again.
            content_signature = self.latest_map_signature[7]
        else:
            content_signature = occupancy_content_signature(msg.data)
        identity_revision = (self.mapping_session_id if self.runtime_state == 'MAPPING'
                             else self.ros_map_revision)
        if self.runtime_state == 'MAPPING':
            map_source = 'SLAM_TOOLBOX'
            active_map_id = f'SLAM-{self.mapping_session_id}'
            active_map_revision = content_signature[:12]
        else:
            active_identity = self.active_map_identity()
            map_source = 'LOCAL_MAP' if self.loaded_local_map_id else 'NAV2_MAP'
            active_map_id = active_identity.get('active_map_id')
            active_map_revision = active_identity.get('active_map_revision')
        signature = (
            str(msg.header.frame_id or 'map'),
            int(msg.info.width), int(msg.info.height),
            float(msg.info.resolution),
            round(float(origin.position.x), 6),
            round(float(origin.position.y), 6),
            round(yaw_from_quaternion(origin.orientation), 6),
            content_signature, identity_revision, active_map_id,
            active_map_revision, map_source,
        )
        if generation != self.latest_map_generation:
            return
        if signature != self.latest_map_signature:
            self.mapping_map_version += 1
        self.latest_map_signature = signature
        self.processed_map = msg
        self.processed_map_generation = generation
        if self.runtime_state == 'MAPPING':
            self.mapping_map_revision = content_signature[:12]
        self._confirm_local_map_if_ready()

    def command_diagnostics_cb(self, msg):
        try:
            data = json.loads(str(msg.data or '{}'))
        except (TypeError, ValueError):
            return
        if not isinstance(data, dict):
            return
        self.command_diagnostics = {
            key: data.get(key) for key in (
                'active_command_source', 'active_control_mode', 'last_command_age',
                'manual_source_active', 'nav_source_active', 'tag_source_active',
                'estop_active',
            )
        }
        self.send({'type': 'COMMAND_DIAGNOSTICS', 'robot_id': self.robot_id,
                   **self.command_diagnostics, 'timestamp': self.now()})
        applied = data.get('active_control_mode')
        if applied in ('MANUAL', 'AUTONOMOUS'):
            self.applied_mode = applied
        if (self.mode_transition_state == 'REQUESTED'
                and data.get('control_mode_request_id') == self.mode_request_id
                and applied == self.requested_mode):
            self.mode_transition_state = 'APPLIED'
            self.send_control_status(True)

    def active_map_identity(self):
        if self.runtime_state == 'MAPPING':
            return {
                'active_map_id': f'SLAM-{self.mapping_session_id}',
                'active_map_revision': self.mapping_map_revision,
                'canonical_map_revision': (str(self.ros_map_revision)
                                           if self.ros_map_revision is not None else None),
            }
        if self.loaded_local_map_id:
            return {
                'active_map_id': self.loaded_local_map_id,
                'active_map_revision': str(self.loaded_local_map_revision or ''),
                'canonical_map_revision': (str(self.ros_map_revision)
                                           if self.ros_map_revision is not None else None),
            }
        revision = str(self.ros_map_revision) if self.ros_map_revision is not None else None
        return {
            'active_map_id': 'CANONICAL' if revision is not None else None,
            'active_map_revision': revision,
            'canonical_map_revision': revision,
        }

    def set_detail_view(self, view, request=None):
        view = str(view or '').upper()
        if view not in ('GLOBAL', 'LIDAR_2D', 'LIDAR_3D'):
            return False
        if view != self.detail_view or request is not None:
            self.detail_view = view
            self.detail_view_epoch += 1
            self.lidar_frame_buffer.clear()
            self.outbound.discard_views()
            self.last_web_cloud_source_stamp = None
            self.last_scan_publish_monotonic = 0.0
            self.last_cloud_publish_monotonic = 0.0
        request = request or {}
        self.view_timing = {'request_id': request.get('request_id'),
            'django_received_ms': request.get('django_received_ms'),
            'bridge_received_ms': request.get('_bridge_received_ms'),
            'bridge_applied_ms': time.time() * 1000}
        self.send({'type': 'ROBOT_DETAIL_VIEW_STATUS', 'robot_id': self.robot_id,
            'requested_view': view, 'applied_view': self.detail_view,
            'view_epoch': self.detail_view_epoch, 'state': 'APPLIED',
            'request_id': request.get('request_id'), 'timestamp': self.now(),
            'bridge_epoch': self.web_cloud_epoch,
            'view_timing': dict(self.view_timing)})
        self.visualization_worker.wake()
        return True

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
        if payload.get('type') in ('LIDAR_MAP_2D', 'LIDAR_MAP_3D', 'MAP_SNAPSHOT', 'LIDAR_SCAN') and not payload.get('_latest_frame_signal'):
            payload = {**payload, 'view_timing': {**payload.get('view_timing', self.view_timing),
                'frame_prepared_ms': time.time() * 1000,
                'detail_interval_wall_ms': (self.detail_callback_intervals[-1] * 1000
                    if self.detail_callback_intervals else None),
                **self.visualization_metrics}}
        with self.ws_lock:
            if self.ws is None:
                return False
            ws = self.ws
        try:
            self.outbound.offer(payload)
            return True
        except BufferError:
            # Never silently lose a safety/control acknowledgement. Fail
            # closed and invalidate manual ownership if the bounded peer is
            # unable to keep up; reconnect resets output and command epochs.
            self.incoming.put({'type': 'MANUAL_DISCONNECT'})
            self.outbound.clear()
            self.outbound_disconnect.set()
            with self.ws_lock:
                if self.ws is ws:
                    self.ws = None
            self.get_logger().error('critical bridge output overflow; manual control invalidated')
            return False

    def outbound_sender(self):
        while not self.stop_event.is_set():
            item = self.outbound.take()
            if item is None:
                continue
            epoch, payload = item
            if payload.get('_latest_frame_signal'):
                payload = self.lidar_frame_buffer.take(timeout=0)
                if payload is None:
                    continue
            with self.ws_lock:
                ws = self.ws
            if ws is None or not self.outbound.current(epoch):
                continue
            if payload.get('type') in ('LIDAR_MAP_2D', 'LIDAR_MAP_3D') and (
                    self.detail_view != payload.get('view') or payload.get('view_epoch') != self.detail_view_epoch):
                continue
            started = time.monotonic()
            try:
                if payload.get('type') == '_SOCKET_PING':
                    ws.ping()
                    continue
                if isinstance(payload.get('view_timing'), dict):
                    payload['view_timing']['sender_send_ms'] = time.time() * 1000
                if payload.get('type') == 'LIDAR_MAP_3D':
                    payload['send_timestamp'] = self.now()
                    now = time.monotonic()
                    if self.last_lidar_send_monotonic is not None:
                        self.lidar_output_intervals.append(now - self.last_lidar_send_monotonic)
                    payload['web_output_fps'] = self._frequency(self.lidar_output_intervals)
                    payload['dropped_frames'] = self.lidar_frame_buffer.dropped_frames
                ws.send(json.dumps(payload))
                if payload.get('type') == 'LIDAR_MAP_3D':
                    self.last_lidar_send_monotonic = started
                    self.last_lidar_output_metadata = {
                        key: payload.get(key) for key in (
                            'source_timestamp', 'send_timestamp', 'frame_id', 'point_count',
                            'source_fps', 'web_output_fps', 'dropped_frames', 'view')}
                    self.send({'type': 'LIDAR_STREAM_DIAGNOSTICS', 'robot_id': self.robot_id,
                               **self.last_lidar_output_metadata})
            except Exception as exc:
                self.get_logger().warning(f'ROS bridge send failed: {type(exc).__name__}')
                self.incoming.put({'type': 'MANUAL_DISCONNECT'})
                with self.ws_lock:
                    if self.ws is ws:
                        self.ws = None
                self.outbound.clear()
                self.outbound_disconnect.set()
            finally:
                self.control_timing['max_sync_ws_send_duration'] = max(
                    self.control_timing.get('max_sync_ws_send_duration', 0), time.monotonic() - started)

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
                self.outbound_disconnect.clear()
                with self.ws_lock:
                    self.ws = ws
                self.last_web_cloud_source_stamp = None
                # A reconnect may follow a Django restart which lost its cache.
                # This is a genuinely uncached consumer, not a tab transition.
                self.last_sent_map_signature = None
                self.map_snapshot_worker.wake()
                backoff = 1.0
                self.send_bridge_status('CONNECTED')
                self.send_map_revision_status()
                if self.loaded_local_map_id:
                    self.send({'type': 'LOCAL_MAP_STATUS', 'robot_id': self.robot_id,
                               'loaded': True, 'map_id': self.loaded_local_map_id,
                               'map_revision': self.loaded_local_map_revision,
                               'active_map_revision': self.loaded_local_map_revision,
                               'canonical_map_revision': self.ros_map_revision,
                               'frame_id': 'map', 'timestamp': self.now()})
                while not self.stop_event.is_set() and not self.outbound_disconnect.is_set():
                    try:
                        raw = ws.recv()
                        if raw:
                            data = json.loads(raw)
                            if isinstance(data, dict):
                                if data.get('type') == 'DETAIL_VIEW':
                                    data['_bridge_received_ms'] = time.time() * 1000
                                if '_timing' in data:
                                    data['_timing']['T4'] = time.monotonic()
                                self.incoming.put(data)
                                if self.manual_timing_enabled and data.get('type') == 'MANUAL_CMD' and '_timing' in data:
                                    self.get_logger().info('MANUAL_RECEIVE_TIMING ' + json.dumps({
                                        'sequence_id': data.get('sequence_id'), **data['_timing'],
                                        'T5': time.monotonic(), 'generation': self.incoming.generation}))
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
                self.incoming.put({'type': 'MANUAL_DISCONNECT'})
                self.outbound.clear()
                with self.ws_lock:
                    if self.ws is ws:
                        self.ws = None
                self.lidar_frame_buffer.clear()
                self.last_web_cloud_source_stamp = None

    def now(self):
        return datetime.now(timezone.utc).isoformat()

    def request_telemetry(self):
        # Preserve the existing telemetry cadence, but TF waits and map-status
        # work must not occupy the executor shared with manual/E-STOP callbacks.
        self.telemetry_pending.set()
        self.visualization_worker.wake()

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
                       **self.active_map_identity(),
                       'mapping_session_id': self.mapping_session_id if self.runtime_state == 'MAPPING' else None,
                       'base_frame_id': base_frame, **pose,
                       'vx': t.linear.x, 'vy': t.linear.y, 'wz': t.angular.z,
                       'navigation_state': self.nav_state,
                       'control_mode': self.applied_mode, 'timestamp': self.now()})
        except (TransformException, ValueError, TypeError) as exc:
            self.tf_status = False
            self.tf_error = str(exc)[:300]
        self._refresh_map_sync_status()

    def manual_timer(self):
        self.trace_control_callback('manual_timer')
        with self.incoming.lock:
            # Ingress may already be fresh while the separate command timer
            # has not run. Apply through the same validator before deciding
            # the old lease expired; pending safety barriers take precedence.
            pending = self.incoming.take_pending_manual()
            if pending is not None:
                self._apply_manual_command(pending)
            self._manual_timer()

    def _manual_timer(self):
        """Publish the dead-man command while the web operator is holding it."""
        if self.mode_transition_state == 'REQUESTED' and time.monotonic() - self.mode_request_started > 5.0:
            self.mode_transition_state = 'FAILED'
            self.send_control_status(False, 'arbiter mode application timed out')
        if (self.manual_generation != self.incoming.generation
                or self.mode_transition_state != 'APPLIED'):
            self.manual_twist = Twist()
            self.manual_deadline = 0.0
            # Consume the invalidation after publishing its safety zero. Once
            # the mode is applied, idle output must expire to owner NONE;
            # continually publishing zero would retain a fresh manual source.
            self.manual_generation = self.incoming.generation
            self.cmd_pub.publish(self.manual_twist)
            return
        if self.emergency_stop_active or self.control_mode != 'MANUAL':
            return
        if self.manual_deadline <= 0.0:
            return
        if time.monotonic() > self.manual_deadline:
            self.manual_twist = Twist()
            self.manual_deadline = 0.0
            self.cmd_pub.publish(self.manual_twist)
            return
        self.cmd_pub.publish(self.manual_twist)

    def _robot_is_stopped(self):
        if self.latest_odom is None:
            return False
        twist = self.latest_odom.twist.twist
        values = (twist.linear.x, twist.linear.y, twist.angular.z)
        return all(math.isfinite(float(value)) and abs(float(value)) <= 0.05
                   for value in values)

    def heartbeat_timer(self):
        self.send({'type': 'HEARTBEAT', 'robot_id': self.robot_id,
                   'nav2_state': self.nav_state, 'bridge_state': 'CONNECTED',
                   'runtime_state': self.runtime_state, 'control_mode': self.applied_mode,
                   'mapping_state': ('PAUSED' if self.slam_paused else 'MAPPING') if self.runtime_state == 'MAPPING' else 'INACTIVE',
                   'mapping_session_id': self.mapping_session_id if self.runtime_state == 'MAPPING' else None,
                   'mapping_elapsed_s': self._mapping_elapsed_s(),
                   'timestamp': self.now()})
        # Do not let graph discovery / map validation delay manual/control
        # servicing. Diagnostics have the same cadence on a bounded worker.
        self.send({'type': '_SOCKET_PING'})

    def diagnostics_timer(self):
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

    def _transform_scan(self, scan, requested_frame=None):
        source_frame = str(scan.header.frame_id or '')
        target_frame = str(requested_frame or self.get_parameter('map_frame').value or 'map')
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

        limit = max(1, int(self.get_parameter('lidar_max_points').value))
        valid_ranges = [float(value) for value in scan.ranges
                        if math.isfinite(float(value))
                        and float(scan.range_min) <= float(value) <= float(scan.range_max)]
        local_points = laser_scan_xy(
            scan.ranges, float(scan.angle_min), float(scan.angle_increment),
            float(scan.range_min), float(scan.range_max), max_points=limit,
        )
        transformed = transform_points_xyz(
            ((x, y, 0.0) for x, y in local_points), translation, quaternion)
        sampled = [[x, y] for x, y, _z in transformed]
        sensor_pose = {
            'x': translation[0], 'y': translation[1],
            'yaw': yaw_from_quaternion(quaternion),
        }
        return {
            'robot_id': self.robot_id,
            'topic': self.scan_topic_name,
            'frame_id': target_frame,
            'source_frame_id': source_frame,
            'mapping_session_id': (self.mapping_session_id
                                   if self.runtime_state == 'MAPPING' else None),
            'sensor_pose': sensor_pose,
            'timestamp': datetime.now(timezone.utc).isoformat(),
            'stamp': self.last_scan_stamp,
            'angle_min': float(scan.angle_min), 'angle_max': float(scan.angle_max),
            'angle_increment': float(scan.angle_increment),
            'range_min': float(scan.range_min), 'range_max': float(scan.range_max),
            'point_count': len(sampled),
            'minimum_range': min(valid_ranges, default=None),
            'maximum_range': max(valid_ranges, default=None),
            'scan_hz_sim': self._frequency(self.scan_sim_intervals),
            'scan_hz_wall': self._frequency(self.scan_intervals),
            'points': sampled,
        }

    def _map_to_base(self, x, y, stamp=None):
        target = str(self.get_parameter('base_footprint_frame').value or 'base_footprint')
        source = str(self.get_parameter('map_frame').value or 'map')
        if target == source:
            return float(x), float(y), 0.0
        when = stamp if stamp is not None else Time()
        transform = self.tf_buffer.lookup_transform(
            target, source, when, timeout=Duration(seconds=0.05))
        t, q = transform.transform.translation, transform.transform.rotation
        result = rotate_translate_xy(
            float(x), float(y), (float(t.x), float(t.y), float(t.z)),
            (float(q.x), float(q.y), float(q.z), float(q.w)))
        return result[0], result[1], yaw_from_quaternion(q)

    def _local_path(self):
        msg = self.latest_global_path
        if msg is None:
            return []
        target = str(self.get_parameter('base_footprint_frame').value or 'base_footprint')
        source = str(msg.header.frame_id or self.get_parameter('map_frame').value or 'map')
        transform = self.tf_buffer.lookup_transform(target, source, Time(), timeout=Duration(seconds=.05))
        t, q = transform.transform.translation, transform.transform.rotation
        values = (t.x, t.y, t.z, q.x, q.y, q.z, q.w)
        key = (id(msg), tuple(round(float(value), 3) for value in values))
        cache = getattr(self, '_local_path_cache', None)
        if cache is not None and cache[0] == key:
            return cache[2]
        points = transform_points_xyz(((item.pose.position.x, item.pose.position.y, 0)
            for item in msg.poses), values[:3], values[3:])
        result = [[x, y] for x, y, _z in points]
        self._local_path_cache = (key, msg, result)
        return result

    def _local_goal(self):
        msg = self.latest_goal_pose
        if msg is None:
            return None
        frame = str(msg.header.frame_id or '')
        x, y = float(msg.pose.position.x), float(msg.pose.position.y)
        yaw = yaw_from_quaternion(msg.pose.orientation)
        if frame == str(self.get_parameter('map_frame').value or 'map'):
            x, y, yaw_tf = self._map_to_base(x, y, Time())
            yaw += yaw_tf
        elif frame != str(self.get_parameter('base_footprint_frame').value or 'base_footprint'):
            raise TransformException(f'goal frame {frame!r} cannot be projected to the LiDAR view')
        return {'x': x, 'y': y, 'yaw': math.atan2(math.sin(yaw), math.cos(yaw))}

    def _render_lidar_2d(self):
        view_epoch, view_timing = self.detail_view_epoch, dict(self.view_timing)
        if self.latest_scan is None:
            return
        target = str(self.get_parameter('base_footprint_frame').value or 'base_footprint')
        scan = self._transform_scan(self.latest_scan, target)
        try:
            route = self._local_path()
            goal = self._local_goal()
        except TransformException:
            route, goal = [], None
        if view_epoch != self.detail_view_epoch or self.detail_view != 'LIDAR_2D':
            return
        self.send({
            'type': 'LIDAR_MAP_2D', 'robot_id': self.robot_id,
            'timestamp': scan['timestamp'], 'source_stamp': scan['stamp'],
            'source_timestamp': scan['stamp'], 'send_timestamp': self.now(),
            'frame_id': target, 'source_frame_id': scan['source_frame_id'],
            'point_count': scan['point_count'], 'points': scan['points'],
            'path': route, 'goal': goal,
            'source_fps': self._frequency(self.scan_intervals),
            'web_output_fps': self._frequency(self.scan_intervals),
            'dropped_frames': 0,
            'render_fps': self._frequency(self.scan_intervals),
            'view': 'LIDAR_2D', 'view_epoch': view_epoch,
            'bridge_epoch': self.web_cloud_epoch, 'request_id': view_timing.get('request_id'),
            'view_timing': view_timing,
        })
        self.send({'type': 'LIDAR_STREAM_DIAGNOSTICS', 'robot_id': self.robot_id,
                   'source_timestamp': scan['stamp'], 'send_timestamp': self.now(),
                   'frame_id': target, 'point_count': scan['point_count'],
                   'source_fps': self._frequency(self.scan_intervals),
                   'web_output_fps': self._frequency(self.scan_intervals),
                   'dropped_frames': 0, 'view': 'LIDAR_2D'})

    def _render_lidar_3d(self):
        render_started = time.monotonic()
        view_epoch, view_timing = self.detail_view_epoch, dict(self.view_timing)
        filtered_is_fresh = (self.latest_filtered_cloud is not None
                             and self.latest_filtered_cloud_monotonic is not None
                             and time.monotonic() - self.latest_filtered_cloud_monotonic <= 1.0)
        source = self.latest_filtered_cloud if filtered_is_fresh else self.latest_cloud
        if source is None:
            return
        stamp = source.header.stamp
        source_stamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if source_stamp == self.last_web_cloud_source_stamp:
            return
        source_frame = str(source.header.frame_id or '')
        target = str(self.get_parameter('base_footprint_frame').value or 'base_footprint')
        if not source_frame:
            raise TransformException('PointCloud2 frame_id is empty')
        stamp = Time.from_msg(source.header.stamp)
        if source_frame == target:
            transform = None
        else:
            transform = self.tf_buffer.lookup_transform(
                target, source_frame, stamp, timeout=Duration(seconds=0.05))
        translation, quaternion = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
        if transform is not None:
            t, q = transform.transform.translation, transform.transform.rotation
            translation = (float(t.x), float(t.y), float(t.z))
            quaternion = (float(q.x), float(q.y), float(q.z), float(q.w))

        raw = point_cloud2.read_points(source, field_names=('x', 'y', 'z'), skip_nans=True)
        values = bounded_cloud_points(
            raw, translation, quaternion,
            max_points=int(self.get_parameter('lidar_max_3d_points').value),
            min_range=float(self.get_parameter('lidar_3d_min_range').value),
            max_range=float(self.get_parameter('lidar_3d_max_range').value),
            min_height=float(self.get_parameter('lidar_3d_min_height').value),
            max_height=float(self.get_parameter('lidar_3d_max_height').value),
            voxel_size=float(self.get_parameter('lidar_3d_voxel_size').value),
        )
        try:
            route = self._local_path()
            goal = self._local_goal()
        except TransformException:
            route, goal = [], None
        xyz = [[round(x, 3), round(y, 3), round(z, 3)] for x, y, z in values]
        bounds = None
        if values:
            bounds = {
                'min': [min(row[index] for row in xyz) for index in range(3)],
                'max': [max(row[index] for row in xyz) for index in range(3)],
            }
        next_revision = self.web_cloud_revision + 1
        payload = {
            'type': 'LIDAR_MAP_3D', 'robot_id': self.robot_id,
            'timestamp': datetime.now(timezone.utc).isoformat(),
            'source_stamp': source_stamp,
            'source_timestamp': source_stamp,
            'render_timestamp': datetime.now(timezone.utc).isoformat(),
            'frame_id': target, 'source_frame_id': source_frame,
            'point_count': len(xyz), 'points': xyz, 'bounds': bounds,
            'source_fps': self._frequency(
                self.filtered_cloud_intervals if filtered_is_fresh else self.raw_cloud_intervals),
            'web_output_fps': self._frequency(self.lidar_output_intervals),
            'dropped_frames': self.lidar_frame_buffer.dropped_frames,
            'render_fps': self._frequency(self.lidar_output_intervals),
            'epoch': self.web_cloud_epoch, 'revision': next_revision,
            'path': route, 'goal': goal, 'view': 'LIDAR_3D',
            'view_epoch': view_epoch, 'bridge_epoch': self.web_cloud_epoch,
            'request_id': view_timing.get('request_id'),
        }
        self.visualization_metrics['cloud_render_ms'] = (time.monotonic() - render_started) * 1000
        payload['view_timing'] = {**view_timing, **self.visualization_metrics,
            'frame_prepared_ms': time.time() * 1000,
            'detail_interval_wall_ms': (self.detail_callback_intervals[-1] * 1000
                if self.detail_callback_intervals else None)}
        if view_epoch != self.detail_view_epoch or self.detail_view != 'LIDAR_3D':
            self.visualization_metrics['stale_preparations_dropped'] = self.visualization_metrics.get('stale_preparations_dropped', 0) + 1
            return
        self.lidar_frame_buffer.offer(payload)
        if not self.send({'type': 'LIDAR_MAP_3D', '_latest_frame_signal': True}):
            self.lidar_frame_buffer.clear()
        self.last_web_cloud_source_stamp = source_stamp
        self.web_cloud_revision = next_revision

    def map_snapshot_timer(self):
        """Hash/compress only changed /map content on its isolated worker."""
        self._update_latest_map_signature()
        signature = self.latest_map_signature
        if (self.processed_map is None or signature is None
                or self.processed_map_generation != self.latest_map_generation):
            return
        if self.processed_map_payload_signature != signature:
            msg = self.processed_map
            generation = self.processed_map_generation
            origin = msg.info.origin
            stamp = msg.header.stamp
            started = time.monotonic()
            statistics = occupancy_grid_statistics(msg.data)
            total_cells = int(msg.info.width) * int(msg.info.height)
            compressed = compress_occupancy_grid(msg.data)
            if generation != self.latest_map_generation:
                return
            self.visualization_metrics['map_compression_ms'] = (time.monotonic() - started) * 1000
            self.visualization_metrics['map_compressions'] += 1
            self.latest_map_statistics = statistics
            self.processed_map_payload = {'type': 'MAP_SNAPSHOT', 'map': {
                'robot_id': self.robot_id, 'frame_id': str(msg.header.frame_id or ''),
                'map_source': ('SLAM_TOOLBOX' if self.runtime_state == 'MAPPING'
                               else 'LOCAL_MAP' if self.loaded_local_map_id else 'NAV2_MAP'),
                'mapping_session_id': self.mapping_session_id if self.runtime_state == 'MAPPING' else None,
                'map_revision': self.ros_map_revision,
                **self.active_map_identity(),
                'map_version': self.mapping_map_version,
                'timestamp': datetime.now(timezone.utc).isoformat(),
                'stamp': float(stamp.sec) + float(stamp.nanosec) * 1e-9,
                'width': int(msg.info.width), 'height': int(msg.info.height),
                'resolution': float(msg.info.resolution),
                'unknown_cells': total_cells - statistics['known_cells'],
                'explored_area_m2': statistics['known_cells'] * float(msg.info.resolution) ** 2,
                **statistics,
                'origin': {'x': float(origin.position.x), 'y': float(origin.position.y),
                           'yaw': yaw_from_quaternion(origin.orientation)},
                'data_encoding': 'zlib-base64-offset1',
                'data_zlib_base64': compressed,
            }}
            self.processed_map_payload_signature = signature
        if self.last_sent_map_signature != signature and self.processed_map_payload is not None:
            if self.send(self.processed_map_payload):
                self.last_sent_map_signature = signature

    def detail_timer(self):
        """Publish bounded, robot-scoped detail snapshots to Django."""
        wall = time.monotonic()
        view_epoch = self.detail_view_epoch
        if self.telemetry_pending.is_set():
            self.telemetry_pending.clear()
            self.profile_callback(self.telemetry_timer)()
        if self.last_detail_callback_wall is not None:
            self.detail_callback_intervals.append(wall - self.last_detail_callback_wall)
        self.last_detail_callback_wall = wall
        mapping_view = self.runtime_state == 'MAPPING'
        scan_view = self.detail_view in ('GLOBAL', 'LIDAR_2D')
        scan_due = (self.latest_scan is not None
                    and time.monotonic() - self.last_scan_publish_monotonic
                    >= 1.0 / max(0.1, float(self.get_parameter('lidar_ui_hz').value)))
        if mapping_view and scan_due:
            try:
                payload = self._transform_scan(
                    self.latest_scan,
                    str(self.get_parameter('map_frame').value or 'map'))
                self.send({'type': 'LIDAR_SCAN', 'scan': payload})
                self.last_scan_map_tf_valid = True
                self.last_scan_map_tf_error = None
                self.last_scan_publish_monotonic = wall
                self.last_tf_error = None
            except (TransformException, ValueError, TypeError) as exc:
                self.last_scan_map_tf_valid = False
                self.last_scan_map_tf_error = str(exc)[:300]
                self.last_tf_error = str(exc)[:300]
        elif scan_view and scan_due and self.latest_scan is not None:
            try:
                if self.detail_view == 'GLOBAL':
                    payload = self._transform_scan(self.latest_scan)
                    self.send({'type': 'LIDAR_SCAN', 'scan': payload})
                else:
                    self._render_lidar_2d()
                if view_epoch == self.detail_view_epoch:
                    self.last_scan_publish_monotonic = wall
                self.last_tf_error = None
            except (TransformException, ValueError, TypeError) as exc:
                self.last_tf_error = str(exc)[:300]

        # Keep the detailed 2D sensor view alive as well as the independent
        # map-frame scan overlay used by the Mapping canvas.
        if mapping_view and self.detail_view == 'LIDAR_2D' and self.latest_scan is not None:
            try:
                self._render_lidar_2d()
            except (TransformException, ValueError, TypeError) as exc:
                self.last_tf_error = str(exc)[:300]

        cloud_hz = max(0.1, float(self.get_parameter('lidar_web_3d_hz').value))
        cloud_available = self.latest_cloud is not None or self.latest_filtered_cloud is not None
        if (self.detail_view == 'LIDAR_3D' and cloud_available
                and time.monotonic() - self.last_cloud_publish_monotonic >= 1.0 / cloud_hz):
            try:
                self._render_lidar_3d()
                if view_epoch == self.detail_view_epoch:
                    self.last_cloud_publish_monotonic = wall
                self.last_tf_error = None
            except (TransformException, ValueError, TypeError, KeyError, IndexError) as exc:
                self.last_tf_error = str(exc)[:300]

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

        now_monotonic = time.monotonic()
        scan_age = (None if self.last_scan_monotonic is None else
                    max(0.0, now_monotonic - self.last_scan_monotonic))
        odom_age = (None if self.last_odom_monotonic is None else
                    max(0.0, now_monotonic - self.last_odom_monotonic))
        map_age = (None if self.latest_map_received_monotonic is None else
                   max(0.0, now_monotonic - self.latest_map_received_monotonic))
        lidar_age = None
        lidar_frequency = self._frequency(self.scan_intervals) or self._frequency(self.lidar_intervals)
        if self.last_lidar_monotonic is not None:
            lidar_age = max(0.0, now_monotonic - self.last_lidar_monotonic)
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
        slam_state = ('PAUSED' if self.slam_paused else
                      'ACTIVE' if self.runtime_state == 'MAPPING' and slam else
                      'INACTIVE')
        map_grid = self.processed_map
        map_statistics = self.latest_map_statistics or {}
        mapping_tf_valid = bool(tf and self.last_scan_map_tf_valid)
        mapping_status = {
            'slam_state': slam_state,
            'mapping_session_id': (self.mapping_session_id
                                   if self.runtime_state == 'MAPPING' else None),
            'scan_live': scan_age is not None and scan_age <= 3.0,
            'scan_hz': self._frequency(self.scan_intervals),
            'scan_frame': self.last_scan_frame_id,
            'scan_age_s': scan_age,
            'odom_live': odom_age is not None and odom_age <= 3.0,
            'odom_hz': self._frequency(self.odom_intervals),
            'odom_frame': self.last_odom_frame_id,
            'base_frame': self.last_odom_child_frame_id,
            'tf_valid': mapping_tf_valid,
            'tf_lidar_to_map_valid': self.last_scan_map_tf_valid,
            'tf_error': (None if mapping_tf_valid else
                         self.last_scan_map_tf_error if not self.last_scan_map_tf_valid
                         else self.tf_error),
            'map_live': (map_grid is not None and slam and map_age is not None
                         and map_age <= 5.0),
            'map_hz': self._frequency(self.map_intervals),
            'map_width_cells': int(map_grid.info.width) if map_grid is not None else None,
            'map_height_cells': int(map_grid.info.height) if map_grid is not None else None,
            'resolution_m_per_cell': float(map_grid.info.resolution) if map_grid is not None else None,
            'origin_x': float(map_grid.info.origin.position.x) if map_grid is not None else None,
            'origin_y': float(map_grid.info.origin.position.y) if map_grid is not None else None,
            'map_version': self.mapping_map_version,
            **map_statistics,
            'unknown_cells': (int(map_grid.info.width) * int(map_grid.info.height)
                              - int(map_statistics.get('known_cells', 0))
                              if map_grid is not None else None),
            'explored_area_m2': (int(map_statistics.get('known_cells', 0))
                                 * float(map_grid.info.resolution) ** 2
                                 if map_grid is not None else None),
            'map_age_s': map_age,
            'map_odom_owner': ('SLAM_TOOLBOX' if self.runtime_state == 'MAPPING' and slam
                               and not any(name.rstrip('/').endswith('/ekf_v30e')
                                           or name.rstrip('/') == 'ekf_v30e'
                                           or name.rstrip('/').endswith('/map_server')
                                           or name.rstrip('/') == 'map_server'
                                           for name in node_names)
                               else None),
        }
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
            'map_state': {
                **self.active_map_identity(),
                'local_active_map_id': self.loaded_local_map_id,
                'local_active_map_revision': self.loaded_local_map_revision,
                'canonical_map_revision': self.ros_map_revision,
                'map_sync_status': ('LOCAL_ONLY' if self.runtime_state == 'MAPPING'
                                    or self.loaded_local_map_id else self.map_sync_status),
            },
            'mapping': mapping_status,
            'command_ownership': dict(self.command_diagnostics),
            'lidar_stream': dict(self.last_lidar_output_metadata),
            'simulation_time': self.simulation_time,
            'gazebo_rtf': self.gazebo_rtf,
            'errors': self.detail_errors(),
            'metrics': {
                'lidar_age_s': lidar_age,
                'lidar_frequency_hz': lidar_frequency,
                'scan_age_s': scan_age,
                'scan_frequency_wall_hz': self._frequency(self.scan_intervals),
                'odom_age_s': odom_age,
                'odom_frequency_wall_hz': self._frequency(self.odom_intervals),
                'map_age_s': map_age,
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
        if self.runtime_state == 'MAPPING':
            self.map_sync_status = 'LOCAL_ONLY'
            self.map_sync_error = None
            return
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
        if tokens[0] not in (b'P2', b'P5') or tokens[3] != b'255':
            raise ValueError('Nav2 image must be an 8-bit P2 or P5 PGM')
        width, height = int(tokens[1]), int(tokens[2])
        if width <= 0 or height <= 0:
            raise ValueError('PGM dimensions must be positive')
        if tokens[0] == b'P5':
            if index >= len(data) or data[index] not in b' \t\r\n':
                raise ValueError('PGM header has no raster delimiter')
            delimiter = data[index]
            index += 1
            if delimiter == 13 and index < len(data) and data[index] == 10:
                index += 1
            raster = data[index:]
            if len(raster) != width * height:
                raise ValueError('PGM raster length does not match its dimensions')
        else:
            values = []
            while index < len(data):
                while index < len(data) and data[index] in b' \t\r\n':
                    index += 1
                if index >= len(data):
                    break
                if data[index:index + 1] == b'#':
                    newline = data.find(b'\n', index)
                    if newline < 0:
                        break
                    index = newline + 1
                    continue
                start = index
                while index < len(data) and data[index] not in b' \t\r\n#':
                    index += 1
                try:
                    pixel = int(data[start:index])
                except ValueError as exc:
                    raise ValueError('PGM raster contains an invalid pixel') from exc
                if not 0 <= pixel <= 255:
                    raise ValueError('PGM pixel is outside 0-255')
                values.append(pixel)
            if len(values) != width * height:
                raise ValueError('PGM raster length does not match its dimensions')
            raster = bytes(values)
        return width, height, raster

    def _local_map_matches_yaml(self, grid, yaml_path):
        try:
            import yaml
            document = yaml.safe_load(yaml_path.read_text(encoding='utf-8')) or {}
            image_path = Path(str(document['image']))
            if not image_path.is_absolute():
                image_path = yaml_path.parent / image_path
            image_path = image_path.resolve(strict=True)
            if not image_path.is_relative_to(self.local_map_root):
                return False
            width, height, pixels = self._read_pgm(image_path)
            resolution = float(document['resolution'])
            origin = tuple(float(value) for value in document['origin'])
            actual_origin = grid.info.origin
            actual = (float(actual_origin.position.x), float(actual_origin.position.y),
                      yaw_from_quaternion(actual_origin.orientation))
            if (str(grid.header.frame_id or '') != 'map'
                    or int(grid.info.width) != width or int(grid.info.height) != height
                    or not math.isclose(float(grid.info.resolution), resolution, abs_tol=1e-6)
                    or len(origin) != 3
                    or any(not math.isclose(a, b, abs_tol=1e-5)
                           for a, b in zip(actual, origin))):
                return False
            if len(grid.data) != width * height:
                return False
            occupied_threshold = float(document.get('occupied_thresh', 0.65))
            free_threshold = float(document.get('free_thresh', 0.196))
            negate = bool(int(document.get('negate', 0)))
            for row in range(height):
                image_row = height - row - 1
                for column in range(width):
                    gray = pixels[image_row * width + column] / 255.0
                    probability = gray if negate else 1.0 - gray
                    expected = (100 if probability > occupied_threshold else
                                0 if probability < free_threshold else -1)
                    if int(grid.data[row * width + column]) != expected:
                        return False
            return True
        except (OSError, ValueError, TypeError, KeyError, AttributeError, ImportError):
            return False

    def _confirm_local_map_if_ready(self):
        pending = self.pending_local_map_load
        if not pending or not pending.get('service_confirmed'):
            return
        if self.map_callback_count <= pending['baseline_map_count']:
            return
        if not self._local_map_matches_yaml(self.latest_map, pending['yaml_path']):
            return
        self.pending_local_map_load = None
        self.local_map_load_pending = False
        self.loaded_local_map_id = pending['map_id']
        self.loaded_local_map_revision = pending['map_revision']
        self.last_sent_map_signature = None
        self.map_snapshot_worker.wake()
        identity = self.active_map_identity()
        self.send({'type': 'LOCAL_MAP_STATUS', 'robot_id': self.robot_id,
                   'loaded': True, 'map_id': self.loaded_local_map_id,
                   'map_revision': self.loaded_local_map_revision,
                   **identity, 'frame_id': 'map', 'timestamp': self.now()})
        self._send_local_control_result(pending['data'], True, {
            'map_id': self.loaded_local_map_id,
            'active_map_revision': self.loaded_local_map_revision,
            'canonical_map_revision': self.ros_map_revision,
            'map_sync_status': 'LOCAL_ONLY',
            'runtime_map_confirmed': True,
        })

    def poll_local_control_confirmations(self):
        pending = self.pending_local_map_load
        if pending and time.monotonic() > pending['deadline_monotonic']:
            self.pending_local_map_load = None
            self.local_map_load_pending = False
            self._send_local_control_result(
                pending['data'], False,
                error='Nav2 accepted map load but /map did not confirm the selected artifact')
        self._check_initial_pose_confirmation()

    def _check_initial_pose_confirmation(self):
        pending = self.pending_initial_pose
        if not pending:
            return
        active = self.active_map_identity()
        if (active.get('active_map_id') != pending['active_map_id']
                or active.get('active_map_revision') != pending['active_map_revision']):
            self.pending_initial_pose = None
            self._send_local_control_result(pending['data'], False,
                                            error='active map changed while initializing pose')
            return
        try:
            actual, _frame = self._lookup_robot_pose()
            distance = math.hypot(float(actual['x']) - pending['pose']['x'],
                                  float(actual['y']) - pending['pose']['y'])
            yaw_error = math.atan2(math.sin(float(actual['yaw']) - pending['pose']['yaw']),
                                   math.cos(float(actual['yaw']) - pending['pose']['yaw']))
            if distance <= 0.25 and abs(yaw_error) <= 0.35:
                self.pending_initial_pose = None
                self._send_local_control_result(pending['data'], True, {
                    'message': 'authoritative ekf_v30e /set_pose accepted and map-frame TF confirmed',
                    'frame_id': 'map', 'pose': pending['pose'],
                    'active_map_id': pending['active_map_id'],
                    'active_map_revision': pending['active_map_revision'],
                    'runtime_pose_confirmed': True,
                })
                return
        except (TransformException, ValueError, TypeError, KeyError):
            pass
        if time.monotonic() > pending['deadline_monotonic']:
            self.pending_initial_pose = None
            self._send_local_control_result(pending['data'], False,
                                            error='ekf_v30e /set_pose accepted the request but map-frame TF did not confirm the initial pose')

    def _loaded_nav2_revision(self, nav2_node_present):
        # Revalidating the complete raster on every heartbeat can monopolize
        # the single ROS executor. Cache only successful, content-bound checks;
        # a new grid, configured revision or modified artifact invalidates it.
        if not nav2_node_present or self.latest_map is None:
            return None
        signature = getattr(self, 'latest_map_signature', None)
        try:
            stat = self.nav2_map_file.stat()
            key = (self.nav2_configured_revision, signature,
                   str(self.nav2_map_file.resolve()), stat.st_ino,
                   stat.st_mtime_ns, stat.st_ctime_ns, stat.st_size)
            cached = getattr(self, '_nav2_validation_cache', None)
            if signature is not None and cached and cached['key'] == key:
                image_stat = cached['image'].stat()
                if (image_stat.st_ino, image_stat.st_mtime_ns, image_stat.st_ctime_ns,
                        image_stat.st_size) == cached['image_stat']:
                    return cached['revision']
        except OSError:
            return None
        result = self._verify_loaded_nav2_revision(nav2_node_present)
        if result is not None and signature is not None:
            try:
                text = self.nav2_map_file.read_text(encoding='utf-8')
                image_match = re.search(r'^image:\s*(.+?)\s*$', text, re.MULTILINE)
                image = (self.nav2_map_file.parent / image_match.group(1).strip().strip('"\'')).resolve(strict=True)
                image_stat = image.stat()
                self._nav2_validation_cache = {
                    'key': key, 'image': image,
                    'image_stat': (image_stat.st_ino, image_stat.st_mtime_ns,
                                   image_stat.st_ctime_ns, image_stat.st_size),
                    'revision': result,
                }
            except (OSError, AttributeError):
                pass
        return result

    def _verify_loaded_nav2_revision(self, nav2_node_present):
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
        self.trace_control_callback('process_commands')
        while True:
            try:
                data = self.incoming.get_nowait()
            except queue.Empty:
                return
            if not isinstance(data, dict):
                continue
            now = time.monotonic()
            kind = str(data.get('type', '')).upper()
            while self.command_times and now - self.command_times[0] > 1.0:
                self.command_times.popleft()
            if len(self.command_times) >= 100 and kind not in ('CONTROL_MODE', 'MANUAL_CMD', 'MANUAL_DISCONNECT', 'EMERGENCY_STOP', 'CLEAR_EMERGENCY_STOP'):
                self.get_logger().warning('ROS bridge command rate limit exceeded; dropping command')
                self.send({'type': 'BRIDGE_ERROR', 'code': 'RATE_LIMITED',
                           'message': 'command rate limit exceeded', 'timestamp': self.now()})
                continue
            self.command_times.append(now)
            try:
                if kind == 'MAP_PUBLISHED':
                    self.apply_published_map(data)
                elif kind == 'PATH_PREVIEW':
                    self.preview_path(data)
                elif kind == 'LOCAL_CONTROL':
                    self.local_control(data)
                elif kind == 'DETAIL_VIEW':
                    view = data.get('view')
                    if not self.set_detail_view(view, data):
                        self.send({'type': 'BRIDGE_ERROR', 'robot_id': self.robot_id,
                                   'code': 'INVALID_DETAIL_VIEW', 'message': 'unsupported robot detail view'})
                elif kind == 'VDA5050_ORDER':
                    self.accept_vda_order(data)
                elif kind == 'VDA5050_INSTANT_ACTIONS':
                    self.send({'type': 'VDA5050_RUNTIME_STATUS', 'robot_id': self.robot_id,
                               'status': 'UNSUPPORTED',
                               'message': 'instant action execution is not configured in this ROS bridge'})
                elif kind in ('NAV_GOAL', 'TAG_NAV_GOAL'):
                    if kind == 'NAV_GOAL' and not self.consume_path_preview(data):
                        self.send_nav_status(
                            {'robot_id': self.robot_id, 'x': data.get('x'), 'y': data.get('y')},
                            'FAILED', 'PATH_PREVIEW_INVALID: goal requires a current approved Nav2 preview')
                        continue
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
                elif kind == 'MANUAL_DISCONNECT':
                    self.manual_twist = Twist()
                    self.manual_deadline = 0.0
                    self.cmd_pub.publish(Twist())
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

    def preview_path(self, data):
        request_id = str(data.get('request_id') or '')
        goal = {
            'x': float(data.get('x', float('nan'))),
            'y': float(data.get('y', float('nan'))),
            'yaw': float(data.get('yaw', 0.0)),
        }
        active_map = self.active_map_identity()

        def result(status, reason=None, points=None):
            points = points or []
            current_map = self.active_map_identity()
            if (current_map.get('active_map_id') != active_map.get('active_map_id')
                    or current_map.get('active_map_revision') != active_map.get('active_map_revision')):
                status = 'INVALID'
                reason = 'active map changed while Nav2 was computing the preview'
                points = []
            local_points = []
            local_goal = None
            if points:
                try:
                    local_points = [list(self._map_to_base(x, y, Time())[:2]) for x, y in points]
                except TransformException:
                    local_points = []
            try:
                if all(math.isfinite(value) for value in goal.values()):
                    local_x, local_y, map_to_base_yaw = self._map_to_base(goal['x'], goal['y'], Time())
                    local_yaw = goal['yaw'] + map_to_base_yaw
                    local_goal = {
                        'x': local_x, 'y': local_y,
                        'yaw': math.atan2(math.sin(local_yaw), math.cos(local_yaw)),
                    }
            except TransformException:
                pass
            self.send({
                'type': 'PATH_PREVIEW_RESULT', 'robot_id': self.robot_id,
                'request_id': request_id, 'status': status, 'reason': reason,
                **active_map,
                'frame_id': 'map', 'path': points, 'local_path': local_points,
                'local_goal': local_goal,
                'path_length_m': path_length(points),
                'goal': {**goal, 'frame_id': 'map'}, 'timestamp': self.now(),
            })
            if status == 'VALID' and points:
                now = time.monotonic()
                self.path_preview_approvals = {
                    key: value for key, value in self.path_preview_approvals.items()
                    if now - value.get('created_monotonic', now) <= 120.0
                }
                self.path_preview_approvals[request_id] = {
                    'goal': dict(goal), 'active_map_id': active_map['active_map_id'],
                    'active_map_revision': active_map['active_map_revision'],
                    'created_monotonic': now,
                }
                if len(self.path_preview_approvals) > 128:
                    oldest = sorted(self.path_preview_approvals,
                                    key=lambda key: self.path_preview_approvals[key]['created_monotonic'])[:-128]
                    for key in oldest:
                        self.path_preview_approvals.pop(key, None)

        if not request_id or data.get('frame_id', 'map') != 'map':
            result('INVALID', 'path preview requires a request id and map-frame target')
            return
        if self.control_mode != 'AUTONOMOUS' or self.emergency_stop_active:
            result('INVALID', 'path preview requires AUTONOMOUS mode with no emergency stop')
            return
        if (str(data.get('active_map_id') or '') != str(active_map.get('active_map_id') or '')
                or str(data.get('active_map_revision') or '') != str(active_map.get('active_map_revision') or '')
                or not active_map.get('active_map_revision')):
            result('INVALID', 'PATH_PREVIEW_MAP_MISMATCH: request does not match the active robot map')
            return
        if not all(math.isfinite(value) for value in goal.values()):
            result('INVALID', 'goal coordinates must be finite')
            return
        if not self.path_preview_client.server_is_ready():
            result('NO_PATH', 'Nav2 ComputePathToPose action server is unavailable')
            return
        action_goal = ComputePathToPose.Goal()
        action_goal.goal.header.frame_id = 'map'
        action_goal.goal.header.stamp = self.get_clock().now().to_msg()
        action_goal.goal.pose.position.x = goal['x']
        action_goal.goal.pose.position.y = goal['y']
        action_goal.goal.pose.orientation.z = math.sin(goal['yaw'] / 2.0)
        action_goal.goal.pose.orientation.w = math.cos(goal['yaw'] / 2.0)
        action_goal.use_start = False
        self.path_preview_goals[request_id] = goal
        future = self.path_preview_client.send_goal_async(action_goal)
        future.add_done_callback(lambda completed: self.path_preview_goal_response(
            completed, request_id, goal, result))

    def consume_path_preview(self, data):
        request_id = str(data.get('preview_request_id') or '')
        if not request_id:
            return False
        preview = self.path_preview_approvals.get(request_id)
        if not preview:
            return False
        active = self.active_map_identity()
        try:
            goal_matches = all(
                math.isfinite(float(data.get(axis)))
                and abs(float(preview['goal'][axis]) - float(data[axis])) <= 1e-4
                for axis in ('x', 'y', 'yaw'))
        except (KeyError, TypeError, ValueError):
            goal_matches = False
        valid = (
            time.monotonic() - preview['created_monotonic'] <= 120.0
            and preview['active_map_id'] == active.get('active_map_id')
            and preview['active_map_revision'] == active.get('active_map_revision')
            and str(data.get('active_map_id') or '') == active.get('active_map_id')
            and str(data.get('active_map_revision') or '') == active.get('active_map_revision')
            and goal_matches
        )
        if valid:
            self.path_preview_approvals.pop(request_id, None)
        return valid

    def path_preview_goal_response(self, future, request_id, goal, send_result):
        try:
            handle = future.result()
        except Exception as exc:
            self.path_preview_goals.pop(request_id, None)
            send_result('NO_PATH', f'Nav2 planner request failed: {type(exc).__name__}')
            return
        if handle is None or not handle.accepted:
            self.path_preview_goals.pop(request_id, None)
            send_result('NO_PATH', 'Nav2 planner rejected the path request')
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda completed: self.path_preview_result(
            completed, request_id, send_result))

    def path_preview_result(self, future, request_id, send_result):
        self.path_preview_goals.pop(request_id, None)
        try:
            response = future.result()
            poses = response.result.path.poses
            if not successful_path_result(
                    response.status, GoalStatus.STATUS_SUCCEEDED, len(poses)):
                reason = ('Nav2 path request did not succeed'
                          if response.status != GoalStatus.STATUS_SUCCEEDED
                          else 'Nav2 returned an empty path')
                send_result('NO_PATH', reason)
                return
            path_payload = self._pose_path_payload(response.result.path, self.robot_id)
        except Exception as exc:
            send_result('NO_PATH', f'Nav2 path result failed: {type(exc).__name__}')
            return
        send_result('VALID', None, path_payload['points'])

    def _send_local_control_result(self, data, ok, result=None, error=None):
        self.send({
            'type': 'LOCAL_CONTROL_RESULT', 'robot_id': self.robot_id,
            'request_id': str(data.get('request_id') or ''),
            'operation': str(data.get('operation') or ''),
            'ok': bool(ok), 'result': result or {}, 'error': error,
            'timestamp': self.now(),
        })

    def local_control(self, data):
        operation = str(data.get('operation') or '').upper()
        if operation in ('MAPPING_START', 'MAPPING_STOP'):
            if self.runtime_state != 'MAPPING':
                self._send_local_control_result(data, False, error='SLAM Toolbox is not active in this runtime')
                return
            if not self.slam_pause_client.service_is_ready():
                self._send_local_control_result(data, False, error='SLAM Toolbox pause service is unavailable')
                return
            wanted_paused = operation == 'MAPPING_STOP'
            if self.slam_paused == wanted_paused:
                self._send_local_control_result(data, True, {'mapping_state': 'PAUSED' if wanted_paused else 'MAPPING'})
                return
            future = self.slam_pause_client.call_async(SlamPause.Request())
            future.add_done_callback(lambda completed: self.mapping_toggle_result(
                completed, data, wanted_paused))
            return
        if operation == 'MAP_SAVE':
            if self.runtime_state != 'MAPPING':
                self._send_local_control_result(data, False, error='map saving requires SLAM Toolbox mapping mode')
                return
            if not self.slam_save_client.service_is_ready():
                self._send_local_control_result(data, False, error='SLAM Toolbox save_map service is unavailable')
                return
            if not self.slam_serialize_client.service_is_ready():
                self._send_local_control_result(data, False,
                    error='SLAM Toolbox serialize_map service is unavailable; resumable session state was not saved')
                return
            prefix = Path(str(data.get('output_prefix') or '')).expanduser().resolve()
            session_prefix = Path(str(data.get('session_output_prefix') or '')).expanduser().resolve()
            if (not prefix.parent.is_dir() or prefix.parent != self.local_map_root
                    or prefix.name in ('', '.', '..')
                    or session_prefix.parent != self.local_map_root
                    or session_prefix.name in ('', '.', '..')):
                self._send_local_control_result(data, False, error='map save location is invalid')
                return
            if any(path.exists() for path in (
                    prefix.with_suffix('.yaml'), prefix.with_suffix('.pgm'),
                    session_prefix.with_suffix('.posegraph'), session_prefix.with_suffix('.data'))):
                self._send_local_control_result(data, False,
                    error='map save artifact target already exists; choose a new map name')
                return
            request = SlamSaveMap.Request()
            request.name.data = str(prefix)
            future = self.slam_save_client.call_async(request)
            future.add_done_callback(lambda completed: self.map_save_result(
                completed, data, prefix, session_prefix))
            return
        if operation == 'MAP_LOAD':
            if self.runtime_state != 'NAVIGATION':
                self._send_local_control_result(data, False, error='map loading requires the active Nav2 navigation runtime')
                return
            if (self.emergency_stop_active or self.control_mode != 'MANUAL' or not self._robot_is_stopped()
                    or self.active_goal is not None or self.active_pose_goal is not None
                    or self.goal_request_pending or self.local_map_load_pending):
                self._send_local_control_result(data, False,
                                                error='map loading requires a stopped robot in MANUAL mode with no active goal')
                return
            yaml_path = Path(str(data.get('map_yaml') or '')).expanduser().resolve()
            local_root = self.local_map_root
            if (not yaml_path.is_file() or yaml_path.suffix.lower() not in ('.yaml', '.yml')
                    or not yaml_path.is_relative_to(local_root)):
                self._send_local_control_result(data, False, error='selected map YAML does not exist')
                return
            map_id = str(data.get('map_id') or '')
            map_revision = str(data.get('map_revision') or '')
            if not map_id or not map_revision:
                self._send_local_control_result(data, False,
                                                error='selected local map identity or revision is missing')
                return
            if not self.map_load_client.service_is_ready():
                self._send_local_control_result(data, False, error='Nav2 map_server/load_map service is unavailable')
                return
            request = LoadMap.Request()
            request.map_url = str(yaml_path)
            self.local_map_load_pending = True
            self.pending_local_map_load = {
                'data': data, 'map_id': map_id, 'map_revision': map_revision,
                'yaml_path': yaml_path, 'baseline_map_count': self.map_callback_count,
                'deadline_monotonic': time.monotonic() + 20.0,
                'service_confirmed': False,
            }
            try:
                future = self.map_load_client.call_async(request)
            except Exception as exc:
                self.local_map_load_pending = False
                self.pending_local_map_load = None
                self._send_local_control_result(data, False,
                                                error=f'Nav2 map load request failed: {type(exc).__name__}')
                return
            future.add_done_callback(lambda completed: self.map_load_result(completed, data))
            return
        if operation == 'INITIAL_POSE':
            if (self.emergency_stop_active or self.control_mode != 'MANUAL' or not self._robot_is_stopped()
                    or self.active_goal is not None or self.active_pose_goal is not None
                    or self.goal_request_pending or self.local_map_load_pending):
                self._send_local_control_result(data, False,
                                                error='initial pose requires a stopped robot in MANUAL mode with no active goal')
                return
            active = self.active_map_identity()
            if (not active.get('active_map_id') or not active.get('active_map_revision')
                    or str(data.get('active_map_id') or '') != active['active_map_id']
                    or str(data.get('active_map_revision') or '') != active['active_map_revision']):
                self._send_local_control_result(data, False,
                                                error='initial pose request does not match the confirmed active map')
                return
            if not self.initial_pose_client.service_is_ready():
                self._send_local_control_result(data, False, error='authoritative /set_pose service is unavailable')
                return
            request = SetPose.Request()
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.pose.position.x = float(data['x'])
            pose.pose.pose.position.y = float(data['y'])
            pose.pose.pose.orientation.z = math.sin(float(data['yaw']) / 2.0)
            pose.pose.pose.orientation.w = math.cos(float(data['yaw']) / 2.0)
            pose.pose.covariance[0] = 0.04
            pose.pose.covariance[7] = 0.04
            pose.pose.covariance[35] = 0.03
            request.pose = pose
            self.pending_initial_pose = {
                'data': data, 'pose': {key: float(data[key]) for key in ('x', 'y', 'yaw')},
                'active_map_id': active['active_map_id'],
                'active_map_revision': active['active_map_revision'],
                'deadline_monotonic': time.monotonic() + 8.0,
            }
            try:
                future = self.initial_pose_client.call_async(request)
            except Exception as exc:
                self.pending_initial_pose = None
                self._send_local_control_result(data, False,
                                                error=f'initial pose service request failed: {type(exc).__name__}')
                return
            future.add_done_callback(lambda completed: self.initial_pose_service_result(completed, data))
            return
        self._send_local_control_result(data, False, error=f'unsupported local control operation {operation}')

    def mapping_toggle_result(self, future, data, wanted_paused):
        try:
            response = future.result()
            success = bool(response.status)
        except Exception as exc:
            self._send_local_control_result(data, False, error=f'SLAM Toolbox pause request failed: {type(exc).__name__}')
            return
        if not success:
            self._send_local_control_result(data, False, error='SLAM Toolbox did not confirm the mapping state transition')
            return
        if wanted_paused:
            if self.slam_mapping_started_monotonic is not None:
                self.slam_mapping_elapsed_s += max(0.0, time.monotonic() - self.slam_mapping_started_monotonic)
            self.slam_mapping_started_monotonic = None
        else:
            self.slam_mapping_started_monotonic = time.monotonic()
        self.slam_paused = wanted_paused
        self._send_local_control_result(data, True, {
            'mapping_state': 'PAUSED' if wanted_paused else 'MAPPING',
        })

    def _mapping_elapsed_s(self):
        elapsed = self.slam_mapping_elapsed_s
        if self.slam_mapping_started_monotonic is not None and not self.slam_paused:
            elapsed += max(0.0, time.monotonic() - self.slam_mapping_started_monotonic)
        return round(elapsed, 1)

    def map_save_result(self, future, data, prefix, session_prefix):
        try:
            response = future.result()
            success = int(response.result) == int(SlamSaveMap.Response.RESULT_SUCCESS)
        except Exception as exc:
            self._send_local_control_result(data, False, error=f'SLAM Toolbox save_map failed: {type(exc).__name__}')
            return
        yaml_path = prefix.with_suffix('.yaml')
        if not success or not yaml_path.is_file():
            self._cleanup_failed_map_save(yaml_path, prefix.with_suffix('.pgm'), session_prefix)
            self._send_local_control_result(data, False, error='SLAM Toolbox did not persist a map YAML file')
            return
        try:
            import yaml
            document = yaml.safe_load(yaml_path.read_text(encoding='utf-8')) or {}
            image = Path(str(document['image']))
            if not image.is_absolute():
                image = yaml_path.parent / image
            image = image.resolve(strict=True)
        except Exception as exc:
            self._cleanup_failed_map_save(yaml_path, prefix.with_suffix('.pgm'), session_prefix)
            self._send_local_control_result(data, False, error=f'map saver output is incomplete: {type(exc).__name__}')
            return
        request = SlamSerializePoseGraph.Request()
        request.filename = str(session_prefix)
        try:
            session_future = self.slam_serialize_client.call_async(request)
        except Exception as exc:
            self._cleanup_failed_map_save(yaml_path, image, session_prefix)
            self._send_local_control_result(data, False,
                error=f'SLAM Toolbox pose-graph serialization request failed: {type(exc).__name__}')
            return
        session_future.add_done_callback(lambda completed: self.map_serialize_result(
            completed, data, yaml_path, image, session_prefix))

    def map_serialize_result(self, future, data, yaml_path, image, session_prefix):
        try:
            response = future.result()
            success = int(response.result) == int(SlamSerializePoseGraph.Response.RESULT_SUCCESS)
        except Exception as exc:
            self._cleanup_failed_map_save(yaml_path, image, session_prefix)
            self._send_local_control_result(data, False,
                error=f'SLAM Toolbox pose-graph serialization failed: {type(exc).__name__}')
            return
        posegraph_path = session_prefix.with_suffix('.posegraph')
        serialized_data_path = session_prefix.with_suffix('.data')
        if (not success or not posegraph_path.is_file() or not serialized_data_path.is_file()
                or posegraph_path.stat().st_size <= 0 or serialized_data_path.stat().st_size <= 0):
            self._cleanup_failed_map_save(yaml_path, image, session_prefix)
            self._send_local_control_result(data, False,
                error='SLAM Toolbox did not persist both non-empty posegraph and data session artifacts')
            return
        self._send_local_control_result(data, True, {
            'name': str(data.get('name') or ''), 'yaml_path': str(yaml_path),
            'image_path': str(image),
            'slam_session': {
                'engine': 'SLAM_TOOLBOX', 'status': 'AVAILABLE',
                'posegraph_path': str(posegraph_path),
                'data_path': str(serialized_data_path),
            },
        })

    def _cleanup_failed_map_save(self, yaml_path, image_path, session_prefix):
        """Remove only this failed request's outputs below the robot map root."""
        allowed = (Path(yaml_path), Path(image_path),
                   Path(session_prefix).with_suffix('.posegraph'),
                   Path(session_prefix).with_suffix('.data'))
        for path in allowed:
            try:
                resolved = path.resolve()
                if resolved.is_relative_to(self.local_map_root) and resolved.is_file():
                    resolved.unlink()
            except OSError as exc:
                self.get_logger().warning(f'failed map-save cleanup for {path.name}: {type(exc).__name__}')

    def map_load_result(self, future, data):
        try:
            response = future.result()
            success = int(response.result) == int(LoadMap.Response.RESULT_SUCCESS)
        except Exception as exc:
            self.local_map_load_pending = False
            self.pending_local_map_load = None
            self._send_local_control_result(data, False, error=f'Nav2 map load failed: {type(exc).__name__}')
            return
        if not success:
            self.local_map_load_pending = False
            self.pending_local_map_load = None
            self._send_local_control_result(data, False, error=f'Nav2 map_server rejected the selected map (result={response.result})')
            return
        if not self.pending_local_map_load:
            self.local_map_load_pending = False
            self._send_local_control_result(data, False,
                                            error='map load request state was lost before ROS confirmation')
            return
        self.pending_local_map_load['service_confirmed'] = True
        self._confirm_local_map_if_ready()

    def initial_pose_service_result(self, future, data):
        try:
            future.result()
        except Exception as exc:
            self.pending_initial_pose = None
            self._send_local_control_result(data, False,
                                            error=f'ekf_v30e /set_pose service failed: {type(exc).__name__}')
            return
        self._check_initial_pose_confirmation()

    def simple_service_result(self, future, data, message):
        try:
            future.result()
        except Exception as exc:
            self._send_local_control_result(data, False, error=f'ROS service request failed: {type(exc).__name__}')
            return
        self._send_local_control_result(data, True, {'message': message, 'frame_id': 'map'})

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

    def send_control_status(self, accepted: bool, reason=None, manual_refresh=False):
        payload = {
            'type': 'ROBOT_CONTROL_STATUS', 'robot_id': self.robot_id,
            'mode': self.applied_mode, 'accepted': bool(accepted),
            'requested_mode': self.requested_mode, 'applied_mode': self.applied_mode,
            'mode_transition_state': self.mode_transition_state,
            'request_id': self.mode_request_id,
            'timestamp': self.now(),
        }
        if reason:
            payload['reason'] = str(reason)[:300]
        if manual_refresh:
            # A hold republishes real ROS commands at the unchanged cadence;
            # its mode ACK has no per-command fields and is identical. Avoid
            # flooding every Web client with duplicate mode acknowledgements.
            signature = tuple(payload[key] for key in ('accepted', 'requested_mode',
                'applied_mode', 'mode_transition_state', 'request_id'))
            previous = getattr(self, '_manual_status_sent', None)
            wall = time.monotonic()
            if previous and previous[0] == signature and wall - previous[1] < 1:
                return
            self._manual_status_sent = (signature, wall)
        self.send(payload)

    def set_control_mode(self, data):
        mode = str(data.get('mode') or '').upper()
        if mode not in ('MANUAL', 'AUTONOMOUS'):
            self.send_control_status(False, 'mode must be MANUAL or AUTONOMOUS')
            return
        if self.emergency_stop_active:
            self.send_control_status(False, 'emergency stop is active')
            return
        if self.local_map_load_pending and mode != 'MANUAL':
            self.send_control_status(False, 'map loading is in progress')
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
        previous_mode = self.control_mode
        self.control_mode = mode
        self.requested_mode = mode
        self.mode_transition_state = 'REQUESTED'
        self.mode_request_id = str(data.get('request_id') or uuid.uuid4().hex)
        self.mode_request_started = time.monotonic()
        self.manual_twist = Twist()
        self.manual_deadline = 0.0
        if previous_mode == 'MANUAL':
            self.cmd_pub.publish(Twist())
        mode_message = String()
        mode_message.data = json.dumps({'mode': mode, 'request_id': self.mode_request_id})
        self.control_mode_pub.publish(mode_message)
        self.nav_state = 'MANUAL' if mode == 'MANUAL' else 'IDLE'
        self.send_control_status(True)

    def manual_command(self, data):
        with self.incoming.lock:
            self._apply_manual_command(data)

    def _apply_manual_command(self, data):
        applied_started = time.monotonic()
        action = str(data.get('action') or '').upper()
        if not self.incoming.current(data):
            return
        received = data.get('_received_monotonic', time.monotonic())
        lease = max(0.10, float(self.get_parameter('manual_command_timeout').value))
        if action != 'STOP' and (time.monotonic() - received > lease
                                 or self.mode_transition_state != 'APPLIED'):
            return
        if self.local_map_load_pending and action != 'STOP':
            self.send_control_status(False, 'map loading is in progress')
            return
        if self.emergency_stop_active:
            self.send_control_status(False, 'emergency stop is active')
            return
        if self.control_mode != 'MANUAL':
            self.send_control_status(False, 'switch to MANUAL mode before sending motion commands')
            return
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
        self.manual_generation = data['_generation']
        self.manual_deadline = received + lease if action != 'STOP' else 0.0
        # Publish each authorized command immediately. The steady-clock timer
        # below refreshes it while the lease is live, but it must not be the
        # first hop between a Web command and the command arbiter: a loaded
        # single-threaded ROS executor can delay that timer callback.
        self.cmd_pub.publish(command)
        if getattr(self, 'manual_timing_enabled', False) and '_timing' in data:
            timing = dict(data['_timing'], T5=data.get('_received_monotonic'),
                          T6=applied_started, T7=time.monotonic())
            self.get_logger().info('MANUAL_TIMING ' + json.dumps({
                'sequence_id': data.get('sequence_id'), 'generation': data.get('_generation'),
                'action': action, **timing, **self.control_timing}))
        self.nav_state = 'MANUAL' if action != 'STOP' else 'IDLE'
        self.send_control_status(True, manual_refresh=action != 'STOP')

    def trace_control_callback(self, name):
        if not getattr(self, 'manual_timing_enabled', False):
            return
        now = time.monotonic()
        previous = self.control_timing.get(name + '_last')
        self.control_timing[name + '_last'] = now
        if previous is not None:
            key = name + '_max_delay'
            self.control_timing[key] = max(self.control_timing.get(key, 0), now - previous)

    def navigate(self, data):
        if self.local_map_load_pending:
            self.send_nav_status({'robot_id': self.robot_id}, 'FAILED', 'map loading is in progress')
            return
        if data.get('x') is not None and data.get('y') is not None:
            self.navigate_pose(data)
            return
        context = {
            'schedule_id': data.get('schedule_id'),
            'stop_id': data.get('stop_id'),
            'robot_id': self.robot_id,
            'target_tag_id': data.get('target_tag_id'),
        }
        if self.loaded_local_map_id:
            self.send_nav_status(context, 'FAILED', 'navigation is blocked while a non-canonical local map is loaded')
            return
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
        # Do not wait for graph discovery inside the shared ROS executor:
        # safety/control callbacks must remain runnable when Nav is offline.
        if not self.nav_client.server_is_ready():
            self.nav_state = 'FAILED'
            self.send_nav_status(
                context, 'FAILED',
                'GoToTag action server unavailable',
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
            'preview_request_id': data.get('preview_request_id'),
            'active_map_id': data.get('active_map_id'),
            'active_map_revision': data.get('active_map_revision'),
        }
        vda_context = data.get('vda_order_context')
        if isinstance(vda_context, dict):
            context.update({key: value for key, value in vda_context.items()
                            if key in ('vda_order_id', 'vda_order_update_id', 'vda_node_id', 'vda_node_index')})
        if self.emergency_stop_active:
            self.send_nav_status(context, 'FAILED', 'emergency stop is active')
            return
        web_local_goal = bool(data.get('preview_request_id'))
        if self.loaded_local_map_id and not web_local_goal:
            self.send_nav_status(context, 'FAILED', 'fleet navigation is blocked while a robot-local map is active')
            return
        if web_local_goal:
            active = self.active_map_identity()
            if (str(data.get('active_map_id') or '') != str(active.get('active_map_id') or '')
                    or str(data.get('active_map_revision') or '') != str(active.get('active_map_revision') or '')):
                self.send_nav_status(context, 'FAILED', 'PATH_PREVIEW_MAP_MISMATCH: the active robot map changed')
                return
        if self.loaded_local_map_id and not self.loaded_local_map_revision:
            self.send_nav_status(context, 'FAILED', 'active local map revision is not confirmed')
            return
        if self.local_map_load_pending:
            self.send_nav_status(context, 'FAILED', 'map loading is in progress')
            return
        if self.control_mode != 'AUTONOMOUS':
            self.send_nav_status(context, 'FAILED', 'switch to AUTONOMOUS mode before navigation')
            return
        if self.active_goal is not None or self.active_pose_goal is not None or self.goal_request_pending:
            self.send_nav_status(context, 'FAILED', 'another navigation goal is already active')
            return
        if not self.nav_pose_client.server_is_ready():
            self.nav_state = 'FAILED'
            self.send_nav_status(context, 'FAILED', 'NavigateToPose action server unavailable')
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
        if context.get('vda_order_id'):
            if status == 'SUCCEEDED':
                self.advance_vda_order(context)
            else:
                self.active_vda_order = None
                self.send({'type': 'VDA5050_RUNTIME_STATUS', 'robot_id': self.robot_id,
                           'status': status, 'order_id': context.get('vda_order_id'),
                           'node_id': context.get('vda_node_id'), 'reason': reason})

    def accept_vda_order(self, data):
        order = data.get('order')
        if not isinstance(order, dict):
            self.send({'type': 'VDA5050_RUNTIME_STATUS', 'robot_id': self.robot_id,
                       'status': 'REJECTED', 'reason': 'order payload must be an object'})
            return
        order_id = str(order.get('orderId') or '')
        try:
            update_id = int(order.get('orderUpdateId', 0))
            raw_nodes = order.get('nodes')
        except (TypeError, ValueError):
            raw_nodes = None
            update_id = -1
        if (not order_id or len(order_id) > 128 or update_id < 0
                or not isinstance(raw_nodes, list) or not raw_nodes or len(raw_nodes) > 200):
            self.send({'type': 'VDA5050_RUNTIME_STATUS', 'robot_id': self.robot_id,
                       'status': 'REJECTED', 'reason': 'orderId, orderUpdateId, or nodes are invalid'})
            return
        if self.control_mode != 'AUTONOMOUS' or self.emergency_stop_active:
            self.send({'type': 'VDA5050_RUNTIME_STATUS', 'robot_id': self.robot_id,
                       'status': 'REJECTED', 'order_id': order_id,
                       'reason': 'autonomous mode is unavailable or emergency stop is active'})
            return
        if self.loaded_local_map_id:
            self.send({'type': 'VDA5050_RUNTIME_STATUS', 'robot_id': self.robot_id,
                       'status': 'REJECTED', 'order_id': order_id,
                       'reason': 'fleet orders are blocked while a non-canonical local map is loaded'})
            return
        if self.local_map_load_pending:
            self.send({'type': 'VDA5050_RUNTIME_STATUS', 'robot_id': self.robot_id,
                       'status': 'REJECTED', 'order_id': order_id,
                       'reason': 'fleet orders are blocked while a local map is loading'})
            return
        if self.map_sync_status != 'SYNCED':
            self.send({'type': 'VDA5050_RUNTIME_STATUS', 'robot_id': self.robot_id,
                       'status': 'REJECTED', 'order_id': order_id,
                       'reason': f'fleet orders are blocked while map synchronization is {self.map_sync_status}'})
            return
        if self.active_vda_order or self.active_pose_goal or self.active_goal or self.goal_request_pending:
            self.send({'type': 'VDA5050_RUNTIME_STATUS', 'robot_id': self.robot_id,
                       'status': 'REJECTED', 'order_id': order_id,
                       'reason': 'another navigation goal is active'})
            return
        targets = []
        for node in raw_nodes:
            if not isinstance(node, dict) or node.get('released') is not True:
                continue
            position = node.get('nodePosition')
            if not isinstance(position, dict) or str(position.get('mapId') or 'map') not in ('map', ''):
                continue
            try:
                x, y = float(position['x']), float(position['y'])
                yaw = float(position.get('theta', 0.0))
                sequence_id = int(node.get('sequenceId', 0))
                allowed_deviation = float(position.get('allowedDeviationXY', 0.25) or 0.25)
            except (KeyError, TypeError, ValueError):
                continue
            if not all(math.isfinite(value) for value in (x, y, yaw, allowed_deviation)):
                continue
            targets.append({
                'x': x, 'y': y, 'yaw': yaw,
                'node_id': str(node.get('nodeId') or sequence_id),
                'sequence_id': sequence_id,
                'allowed_deviation_xy': max(0.05, min(1.0, allowed_deviation)),
            })
        if not targets:
            self.send({'type': 'VDA5050_RUNTIME_STATUS', 'robot_id': self.robot_id,
                       'status': 'REJECTED', 'order_id': order_id,
                       'reason': 'order has no released map-frame navigation node'})
            return
        try:
            pose, _frame = self._lookup_robot_pose()
            first = targets[0]
            if math.hypot(pose[0] - first['x'], pose[1] - first['y']) <= first['allowed_deviation_xy']:
                targets.pop(0)
        except TransformException:
            pass
        if not targets:
            self.send({'type': 'VDA5050_RUNTIME_STATUS', 'robot_id': self.robot_id,
                       'status': 'FINISHED', 'order_id': order_id, 'order_update_id': update_id})
            return
        self.active_vda_order = {
            'order_id': order_id, 'order_update_id': update_id,
            'targets': targets, 'index': 0,
        }
        self.send({'type': 'VDA5050_RUNTIME_STATUS', 'robot_id': self.robot_id,
                   'status': 'RUNNING', 'order_id': order_id, 'order_update_id': update_id})
        self._navigate_vda_target()

    def _navigate_vda_target(self):
        order = self.active_vda_order
        if not order or order['index'] >= len(order['targets']):
            return
        target = order['targets'][order['index']]
        self.navigate_pose({
            **target, 'frame_id': 'map',
            'vda_order_context': {
                'vda_order_id': order['order_id'],
                'vda_order_update_id': order['order_update_id'],
                'vda_node_id': target['node_id'],
                'vda_node_index': order['index'],
            },
        })

    def advance_vda_order(self, context):
        order = self.active_vda_order
        if not order or context.get('vda_order_id') != order.get('order_id'):
            return
        order['index'] += 1
        if order['index'] >= len(order['targets']):
            self.send({'type': 'VDA5050_RUNTIME_STATUS', 'robot_id': self.robot_id,
                       'status': 'FINISHED', 'order_id': order['order_id'],
                       'order_update_id': order['order_update_id']})
            self.active_vda_order = None
            return
        self._navigate_vda_target()

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
        if self.local_map_load_pending:
            self.send_nav_status({'robot_id': self.robot_id}, 'FAILED',
                                 'navigation resume is blocked while a local map transition is active')
            return
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
        self.visualization_worker.close()
        self.map_snapshot_worker.close()
        self.diagnostics_worker.close()
        self.outbound.clear()
        self.lidar_frame_buffer.clear()
        if self.context.ok():
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
