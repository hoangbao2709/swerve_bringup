#!/usr/bin/env python3
"""Authoritative readiness gate for a fresh swerve navigation session."""
import argparse
import fcntl
import json
import math
import os
import re
import sys
import time
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET
import uuid
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnEntity
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.srv import ManageLifecycleNodes
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


# In UNIFIED, SLAM Toolbox alone owns live /map and map->odom. A separate
# lifecycle map_server publishes the selected published bundle on
# /canonical_map; the bridge registers that raster into the active SLAM frame
# on /navigation_map for Nav2. The two map products never share a topic.
NAV2_LIVE_SLAM_LIFECYCLE_NODES = (
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
)
NAV2_STATIC_MAP_LIFECYCLE_NODES = ('map_server', *NAV2_LIVE_SLAM_LIFECYCLE_NODES)
NAV2_REGISTERED_CANONICAL_LIFECYCLE_NODES = (
    'canonical_map_server', *NAV2_LIVE_SLAM_LIFECYCLE_NODES,
)
NAV2_SETTLED_STATE_LABELS = {'unconfigured', 'inactive', 'active', 'finalized'}
NAV2_TRANSITION_STATE_LABELS = {
    'configuring', 'cleaningup', 'shuttingdown', 'activating',
    'deactivating', 'errorprocessing',
}
NAV2_STARTUP_STATES = {
    'NOT_REQUESTED', 'REQUESTED', 'IN_PROGRESS', 'ACTIVE', 'FAILED',
}
CONTROLLER_SERVICE_RESPONSE_TIMEOUT_S = 10.0
# Controller state queries are deliberately much slower than the control loop:
# the spawners own activation, and readiness only verifies the settled result.
CONTROLLER_QUERY_INTERVAL_S = 2.0
# Give Gazebo time to finish loading the published warehouse before requiring
# the first /clock messages.  Once messages arrive, the three increasing
# samples must still fit inside the tighter sample window.
CLOCK_READY_WAIT_TIMEOUT_S = 120.0
CLOCK_SAMPLE_SPAN_TIMEOUT_S = 5.0
STARTUP_TIMELINE_ORDER = (
    'T0_START_STACK',
    'T1_GAZEBO_PROCESS_START',
    'T2_CLOCK_READY',
    'T3_SPAWN_SERVICE_READY',
    'T4_SPAWN_REQUEST_BEGIN',
    'T5_SPAWN_REQUEST_RETURN',
    'T6_ROBOT_ENTITY_CONFIRMED',
    'T7_CONTROLLER_MANAGER_READY',
    'T8_JOINT_STATE_BROADCASTER_ACTIVE',
    'T9_STEERING_CONTROLLER_ACTIVE',
    'T10_DRIVE_CONTROLLER_ACTIVE',
    'JOINT_STATES_READY',
    'T11_ODOM_READY',
    'FILTERED_ODOM_READY',
    'T12_LIDAR_RAW_READY',
    'T13_LIDAR_FILTERED_READY',
    'T14_SCAN_READY',
    'T15_NAV2_LIFECYCLE_STARTUP_BEGIN',
    'T16_NAV2_ACTION_SERVER_READY',
)
TIMELINE_STAGE_NAMES = {
    'GAZEBO_PROCESS_READY': 'T1_GAZEBO_PROCESS_START',
    'CLOCK_READY': 'T2_CLOCK_READY',
    'GAZEBO_FACTORY_READY': 'T3_SPAWN_SERVICE_READY',
    'ROBOT_SPAWNED': 'T6_ROBOT_ENTITY_CONFIRMED',
    'CONTROLLER_MANAGER_READY': 'T7_CONTROLLER_MANAGER_READY',
    'JOINT_STATE_BROADCASTER_ACTIVE': 'T8_JOINT_STATE_BROADCASTER_ACTIVE',
    'STEERING_CONTROLLER_ACTIVE': 'T9_STEERING_CONTROLLER_ACTIVE',
    'DRIVE_CONTROLLER_ACTIVE': 'T10_DRIVE_CONTROLLER_ACTIVE',
    'JOINT_STATES_READY': 'JOINT_STATES_READY',
    'ODOM_READY': 'T11_ODOM_READY',
    'FILTERED_ODOM_READY': 'FILTERED_ODOM_READY',
    'LIDAR_RAW_READY': 'T12_LIDAR_RAW_READY',
    'LIDAR_FILTERED_READY': 'T13_LIDAR_FILTERED_READY',
    'SCAN_READY': 'T14_SCAN_READY',
    'ACTION_SERVER_READY': 'T16_NAV2_ACTION_SERVER_READY',
}
SPAWN_REQUEST_RE = re.compile(
    r'\[(\d+\.\d+)\]\s+\[spawn_swerve\]: Calling service /spawn_entity')
SPAWN_RESPONSE_RE = re.compile(
    r'\[(\d+\.\d+)\]\s+\[spawn_swerve\]: Spawn status: (.*)')
CONTROLLER_ACTIVE_RE = re.compile(
    r'\[(\d+\.\d+)\].*\[spawn_(joint_state_broadcaster|steering_controller|drive_controller)\]:'
    r'.*Configured and activated (joint_state_broadcaster|steering_controller|drive_controller)')
CONTROLLER_TIMELINE_NAMES = {
    'joint_state_broadcaster': 'T8_JOINT_STATE_BROADCASTER_ACTIVE',
    'steering_controller': 'T9_STEERING_CONTROLLER_ACTIVE',
    'drive_controller': 'T10_DRIVE_CONTROLLER_ACTIVE',
}
SWERVE_REQUIRED_LINKS = {
    'base_footprint', 'base_link', 'steer_front_link', 'steer_rear_link',
    'wheel_front_drive_link', 'wheel_rear_drive_link',
}
SWERVE_REQUIRED_JOINTS = {
    'steer_front_joint', 'steer_rear_joint',
    'wheel_front_drive_joint', 'wheel_rear_drive_joint',
}

# Map server, bridge navigation map, and SLAM publish latched grids. A volatile
# sensor-data subscriber can miss the only transient-local map sample when
# readiness joins late.
MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def read_nav2_startup_state(state_file):
    """Read the cross-probe one-shot state for this ROS launch generation."""
    if not state_file:
        return {'state': 'NOT_REQUESTED'}
    try:
        payload = json.loads(Path(state_file).read_text(encoding='utf-8'))
    except FileNotFoundError:
        return {'state': 'NOT_REQUESTED'}
    except (OSError, ValueError, TypeError):
        return {'state': 'INVALID'}
    if not isinstance(payload, dict) or payload.get('state') not in NAV2_STARTUP_STATES:
        return {'state': 'INVALID'}
    return payload


def update_nav2_startup_state(state_file, state, **details):
    """Atomically persist startup state while serializing readiness processes."""
    state = str(state).upper()
    if state not in NAV2_STARTUP_STATES:
        raise ValueError(f'invalid Nav2 startup state: {state}')
    if not state_file:
        return {'state': state, **details}
    path = Path(state_file)
    path.parent.mkdir(parents=True, exist_ok=True)
    lock_path = path.with_suffix(path.suffix + '.lock')
    with lock_path.open('a', encoding='utf-8') as lock_stream:
        fcntl.flock(lock_stream.fileno(), fcntl.LOCK_EX)
        previous = read_nav2_startup_state(path)
        payload = {
            **previous,
            **details,
            'state': state,
            'updated_at': datetime.now(timezone.utc).isoformat(),
        }
        temporary = path.with_suffix(path.suffix + '.tmp')
        temporary.write_text(json.dumps(payload, sort_keys=True), encoding='utf-8')
        os.replace(temporary, path)
        fcntl.flock(lock_stream.fileno(), fcntl.LOCK_UN)
    return payload


def claim_nav2_startup(state_file, launch_id=None, lifecycle_before=None):
    """Atomically consume the sole STARTUP request allowed for one launch."""
    if not state_file:
        return True, update_nav2_startup_state(
            None, 'REQUESTED', launch_id=launch_id or str(uuid.uuid4()),
            lifecycle_before=lifecycle_before,
        )
    path = Path(state_file)
    path.parent.mkdir(parents=True, exist_ok=True)
    lock_path = path.with_suffix(path.suffix + '.lock')
    with lock_path.open('a', encoding='utf-8') as lock_stream:
        fcntl.flock(lock_stream.fileno(), fcntl.LOCK_EX)
        previous = read_nav2_startup_state(path)
        if previous['state'] != 'NOT_REQUESTED':
            fcntl.flock(lock_stream.fileno(), fcntl.LOCK_UN)
            return False, previous
        payload = {
            'state': 'REQUESTED',
            'launch_id': launch_id or previous.get('launch_id') or str(uuid.uuid4()),
            'request_id': str(uuid.uuid4()),
            'requested_at': datetime.now(timezone.utc).isoformat(),
            'lifecycle_before': lifecycle_before,
        }
        temporary = path.with_suffix(path.suffix + '.tmp')
        temporary.write_text(json.dumps(payload, sort_keys=True), encoding='utf-8')
        os.replace(temporary, path)
        fcntl.flock(lock_stream.fileno(), fcntl.LOCK_UN)
    return True, payload


class Readiness(Node):
    """Checks live data and state, never just graph membership."""
    def __init__(self, model, mode='navigation', map_file=None,
                 robot_id='R01', backend_url=None, log_path=None,
                 lifecycle_state_file=None):
        super().__init__('navigation_readiness_probe', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        if not self.get_parameter('use_sim_time').value:
            raise RuntimeError('FAIL: readiness probe use_sim_time is false')
        self.model = model
        self.mode = str(mode).lower()
        if self.mode not in ('mapping', 'navigation', 'unified'):
            raise ValueError(f'unsupported readiness mode: {mode}')
        self.mapping_required = self.mode in ('mapping', 'unified')
        self.navigation_required = self.mode in ('navigation', 'unified')
        self.nav2_lifecycle_nodes = (
            NAV2_STATIC_MAP_LIFECYCLE_NODES if self.mode == 'navigation'
            else NAV2_REGISTERED_CANONICAL_LIFECYCLE_NODES if self.mode == 'unified'
            else ()
        )
        self.nav2_required_nodes = (
            self.nav2_lifecycle_nodes + ('lifecycle_manager_navigation',)
            if self.navigation_required else ()
        )
        self.expected_map_file = os.path.realpath(map_file) if map_file else None
        self.expected_canonical_revision = self._bundle_revision_for_map(self.expected_map_file)
        self.robot_id = str(robot_id)
        self.backend_url = backend_url
        self.log_path = log_path
        self.lifecycle_state_file = (
            Path(lifecycle_state_file).expanduser()
            if lifecycle_state_file else None
        )
        self.timeline_events = {}
        self.timeline_reported = False
        self.stack_start_monotonic = self._monotonic_env(
            'WARETWIN_STACK_START_MONOTONIC_S')
        self.wall_to_monotonic_offset = time.monotonic() - time.time()
        gazebo_start = self._monotonic_env('WARETWIN_GAZEBO_STARTED_MONOTONIC_S')
        if self.stack_start_monotonic is not None:
            self._record_timeline(
                'T0_START_STACK', self.stack_start_monotonic,
                detail='start_stack began',
            )
        if gazebo_start is not None:
            self._record_timeline(
                'T1_GAZEBO_PROCESS_START', gazebo_start,
                detail=f"gzserver pid={os.environ.get('WARETWIN_GAZEBO_PID', 'unknown')}",
            )
        self.last_service_error = None
        self.clock_samples = []
        self.readiness_subscriptions = {}
        self.model_states_seen = False
        self.gazebo_model_names = set()
        self.joints_seen = False
        self.odom_seen = False
        self.filtered_odom_seen = False
        self.scan_seen = False
        self.map_seen = False
        self.canonical_map_grid = None
        self.navigation_map_grid = None
        self.global_costmap_grid = None
        self.navigation_map_metadata = None
        self.sensor_receive_times = {
            'scan': deque(maxlen=30),
            'filtered_odom': deque(maxlen=30),
        }
        self.scan_frame_id = None
        self.scan_stamp = None
        self.scan_stamp_message = None
        self.filtered_odom_frame_id = None
        self.filtered_odom_child_frame_id = None
        self.filtered_odom_stamp = None
        self.mapping_tf_error = None
        self.points_seen = False
        self.filtered_points_seen = False
        self.command_owner_seen = False
        clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        self.readiness_subscriptions['clock'] = self.create_subscription(
            Clock, '/clock', self._clock_cb, clock_qos)
        # A best-effort/volatile subscriber is compatible with both the
        # Gazebo sensor publishers and reliable application publishers.  The
        # probe only needs one real sample, not a rate measurement.
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        self.readiness_subscriptions['model_states'] = self.create_subscription(
            ModelStates, '/model_states', self._model_states_cb, sensor_qos)
        self.readiness_subscriptions['joint_states'] = self.create_subscription(
            JointState, '/joint_states', self._joint_cb, sensor_qos)
        self.readiness_subscriptions['odom'] = self.create_subscription(
            Odometry, '/odom', lambda msg: self._odom_cb('odom_seen', msg), sensor_qos)
        self.readiness_subscriptions['filtered_odom'] = self.create_subscription(
            Odometry, '/odometry/filtered',
            lambda msg: self._odom_cb('filtered_odom_seen', msg), sensor_qos)
        self.readiness_subscriptions['scan'] = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, sensor_qos)
        if self.mapping_required:
            self.readiness_subscriptions['map'] = self.create_subscription(
                OccupancyGrid, '/map', self._map_cb, MAP_QOS)
        if self.mode == 'unified':
            self.readiness_subscriptions['canonical_map'] = self.create_subscription(
                OccupancyGrid, '/canonical_map', self._canonical_map_cb, MAP_QOS)
            self.readiness_subscriptions['navigation_map'] = self.create_subscription(
                OccupancyGrid, '/navigation_map', self._navigation_map_cb, MAP_QOS)
            self.readiness_subscriptions['navigation_map_metadata'] = self.create_subscription(
                String, '/navigation_map_metadata', self._navigation_map_metadata_cb, MAP_QOS)
            self.readiness_subscriptions['global_costmap'] = self.create_subscription(
                OccupancyGrid, '/global_costmap/costmap', self._global_costmap_cb, MAP_QOS)
        self.readiness_subscriptions['points'] = self.create_subscription(
            PointCloud2, '/lidar/points',
            lambda msg: self._cloud_cb('points_seen', msg), sensor_qos)
        self.readiness_subscriptions['filtered_points'] = self.create_subscription(
            PointCloud2, '/lidar/points_filtered',
            lambda msg: self._cloud_cb('filtered_points_seen', msg), sensor_qos)
        self.readiness_subscriptions['command_owner'] = self.create_subscription(
            String, '/command_owner', self._command_owner_cb, 10)
        self.factory = self.create_client(SpawnEntity, '/spawn_entity')
        self.robot_description = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')
        self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.hardware = self.create_client(
            ListHardwareInterfaces, '/controller_manager/list_hardware_interfaces')
        names = self.nav2_lifecycle_nodes
        self.lifecycle = {name: self.create_client(GetState, f'/{name}/get_state') for name in names}
        self.nav_lifecycle_manager = (
            self.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
            if self.navigation_required else None)
        self.nav_lifecycle_manager_parameters = (
            self.create_client(GetParameters, '/lifecycle_manager_navigation/get_parameters')
            if self.navigation_required else None)
        self.map_parameters = (self.create_client(GetParameters, '/map_server/get_parameters')
                               if self.mode == 'navigation' else None)
        self.canonical_map_parameters = (
            self.create_client(GetParameters, '/canonical_map_server/get_parameters')
            if self.mode == 'unified' else None)
        self.slam_parameters = (self.create_client(GetParameters, '/slam_toolbox/get_parameters')
                                if self.mapping_required else None)
        self.bridge_parameters = self.create_client(GetParameters, '/swerve_bridge/get_parameters')
        self.path_action = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

    @staticmethod
    def _monotonic_env(name):
        try:
            value = float(os.environ[name])
        except (KeyError, TypeError, ValueError):
            return None
        return value if math.isfinite(value) else None

    @staticmethod
    def _bundle_revision_for_map(map_file):
        """Resolve revision only from the manifest that owns the selected map artifact."""
        if not map_file:
            return None
        selected = Path(map_file).resolve()
        for parent in selected.parents:
            manifest_path = parent / 'manifest.json'
            if not manifest_path.is_file():
                continue
            try:
                manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
                revision = int(manifest['revision'])
                artifacts = manifest.get('artifacts') or {}
                roles = [artifacts.get('nav2_map'), *(manifest.get('nav2_maps') or {}).values()]
                owned_paths = {(parent / str(value)).resolve() for value in roles if value}
            except (OSError, ValueError, TypeError, KeyError):
                return None
            return revision if selected in owned_paths else None
        return None

    def _record_timeline(self, name, at=None, status='PASS', detail=None):
        at = time.monotonic() if at is None else float(at)
        if not math.isfinite(at):
            return
        event = {'stage': name, 'status': status, 'at_monotonic': at}
        if detail:
            event['detail'] = str(detail)
        previous = self.timeline_events.get(name)
        # A log timestamp is more precise than a later readiness poll. Keep a
        # previously recorded event unless a successful observation upgrades a
        # failed/missing observation.
        if previous is None or (previous['status'] != 'PASS' and status == 'PASS'):
            self.timeline_events[name] = event

    def _log_timestamp_to_monotonic(self, timestamp):
        value = float(timestamp) + self.wall_to_monotonic_offset
        if self.stack_start_monotonic is not None and value < self.stack_start_monotonic - 5.0:
            return None
        if value > time.monotonic() + 5.0:
            return None
        return value

    def _capture_log_timeline(self):
        if not self.log_path or self.stack_start_monotonic is None:
            return
        try:
            with open(self.log_path, encoding='utf-8', errors='replace') as stream:
                lines = stream.readlines()
        except OSError:
            return
        for line in lines:
            request = SPAWN_REQUEST_RE.search(line)
            if request:
                at = self._log_timestamp_to_monotonic(request.group(1))
                if at is not None:
                    self._record_timeline(
                        'T4_SPAWN_REQUEST_BEGIN', at,
                        detail='Calling service /spawn_entity; source=ros.log',
                    )
            response = SPAWN_RESPONSE_RE.search(line)
            if response:
                at = self._log_timestamp_to_monotonic(response.group(1))
                if at is not None:
                    detail = response.group(2).strip()
                    status = 'PASS' if 'Successfully spawned entity' in detail else 'FAIL'
                    self._record_timeline(
                        'T5_SPAWN_REQUEST_RETURN', at, status=status,
                        detail=f'{detail}; source=ros.log',
                    )
            controller = CONTROLLER_ACTIVE_RE.search(line)
            if controller:
                at = self._log_timestamp_to_monotonic(controller.group(1))
                controller_name = controller.group(3)
                if at is not None and controller_name in CONTROLLER_TIMELINE_NAMES:
                    self._record_timeline(
                        CONTROLLER_TIMELINE_NAMES[controller_name], at,
                        detail=f'{controller_name}=active; source=ros.log',
                    )

    def _print_startup_timeline(self, result):
        if self.timeline_reported:
            return
        self._capture_log_timeline()
        rows = []
        previous_at = None
        for name in STARTUP_TIMELINE_ORDER:
            event = self.timeline_events.get(name)
            if event is None:
                row = {'stage': name, 'status': 'UNVERIFIED', 'reason': 'not_observed'}
                print(f'{name}=UNVERIFIED reason=not_observed', flush=True)
                rows.append(row)
                continue
            at = event['at_monotonic']
            row = {'stage': name, 'status': event['status']}
            if self.stack_start_monotonic is not None:
                elapsed = at - self.stack_start_monotonic
                row['elapsed_s'] = round(elapsed, 3)
                line = f'{name}={event["status"]} elapsed={elapsed:.3f}s'
            else:
                line = f'{name}={event["status"]} elapsed=UNVERIFIED'
            if previous_at is not None and at >= previous_at:
                delta = at - previous_at
                row['since_previous_s'] = round(delta, 3)
                line += f' since_previous={delta:.3f}s'
            elif previous_at is not None:
                row['since_previous_s'] = None
                line += ' since_previous=UNVERIFIED reason=timestamp_order'
            if event.get('detail'):
                row['detail'] = event['detail']
                line += f' detail={event["detail"]}'
            print(line, flush=True)
            rows.append(row)
            if previous_at is None or at >= previous_at:
                previous_at = at

        request = self.timeline_events.get('T4_SPAWN_REQUEST_BEGIN')
        response = self.timeline_events.get('T5_SPAWN_REQUEST_RETURN')
        if request and response and response['at_monotonic'] >= request['at_monotonic']:
            duration = response['at_monotonic'] - request['at_monotonic']
            result['spawn_entity_duration_s'] = round(duration, 3)
            print(f'SPAWN_ENTITY_DURATION={duration:.3f}s source=ros.log', flush=True)
        result['startup_timeline'] = rows
        self.timeline_reported = True

    def _stop_monitoring(self, key):
        subscription = self.readiness_subscriptions.pop(key, None)
        if subscription is not None:
            self.destroy_subscription(subscription)

    def _clock_cb(self, msg):
        stamp_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        received_at = time.monotonic()
        self.clock_samples.append((stamp_ns, received_at))
        self.clock_samples = self.clock_samples[-3:]
        if self._clock_ready():
            self._record_timeline('T2_CLOCK_READY', received_at,
                                  detail='three advancing /clock samples')

    def _model_states_cb(self, msg):
        self.model_states_seen = True
        self.gazebo_model_names = set(msg.name)

    def _joint_cb(self, msg):
        required = {
            'steer_front_joint', 'steer_rear_joint',
            'wheel_front_drive_joint', 'wheel_rear_drive_joint',
        }
        self.joints_seen = (
            required.issubset(msg.name)
            and len(msg.position) == len(msg.name)
            and (not msg.velocity or len(msg.velocity) == len(msg.name))
            and all(map(math.isfinite, msg.position))
            and all(map(math.isfinite, msg.velocity))
        )
        if self.joints_seen:
            self._record_timeline('JOINT_STATES_READY', detail='valid required joints')
            self._stop_monitoring('joint_states')

    def _odom_cb(self, name, msg):
        pose = msg.pose.pose.position
        valid = all(map(math.isfinite, (pose.x, pose.y, pose.z)))
        if valid:
            setattr(self, name, True)
            timeline_name = 'T11_ODOM_READY' if name == 'odom_seen' else 'FILTERED_ODOM_READY'
            self._record_timeline(timeline_name, detail='valid odometry sample')
            if name == 'filtered_odom_seen':
                self.sensor_receive_times['filtered_odom'].append(time.monotonic())
                self.filtered_odom_frame_id = str(msg.header.frame_id or '')
                self.filtered_odom_child_frame_id = str(msg.child_frame_id or '')
                stamp = msg.header.stamp
                self.filtered_odom_stamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _cloud_cb(self, name, msg):
        if msg.width > 0 and msg.height > 0 and msg.data:
            setattr(self, name, True)
            timeline_name = ('T12_LIDAR_RAW_READY' if name == 'points_seen'
                             else 'T13_LIDAR_FILTERED_READY')
            self._record_timeline(timeline_name, detail='valid PointCloud2 sample')
            self._stop_monitoring('points' if name == 'points_seen' else 'filtered_points')

    def _scan_cb(self, msg):
        self.scan_seen = bool(msg.ranges) and any(
            math.isfinite(value) and msg.range_min <= value <= msg.range_max
            for value in msg.ranges)
        if self.scan_seen:
            self.sensor_receive_times['scan'].append(time.monotonic())
            self.scan_frame_id = str(msg.header.frame_id or '')
            stamp = msg.header.stamp
            self.scan_stamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            self.scan_stamp_message = stamp
            self._record_timeline('T14_SCAN_READY', detail='valid LaserScan sample')

    @staticmethod
    def _sample_hz(receive_times):
        if len(receive_times) < 2:
            return None
        elapsed = receive_times[-1] - receive_times[0]
        return (len(receive_times) - 1) / elapsed if elapsed > 0.0 else None

    def _print_mapping_sensor_inputs(self):
        scan_hz = self._sample_hz(self.sensor_receive_times['scan'])
        odom_hz = self._sample_hz(self.sensor_receive_times['filtered_odom'])
        print(f'SCAN_HZ_WALL={scan_hz:.3f}' if scan_hz is not None else
              'SCAN_HZ_WALL=UNVERIFIED reason=need_two_scan_samples', flush=True)
        print(f'SCAN_FRAME={self.scan_frame_id or "UNKNOWN"} SCAN_STAMP={self.scan_stamp}', flush=True)
        print(f'ODOM_HZ_WALL={odom_hz:.3f}' if odom_hz is not None else
              'ODOM_HZ_WALL=UNVERIFIED reason=need_two_filtered_odom_samples', flush=True)
        print(f'ODOM_FRAME={self.filtered_odom_frame_id or "UNKNOWN"} '
              f'BASE_FRAME={self.filtered_odom_child_frame_id or "UNKNOWN"} '
              f'ODOM_STAMP={self.filtered_odom_stamp} '
              f'SCAN_ODOM_STAMP_SKEW={abs(self.scan_stamp - self.filtered_odom_stamp) if self.scan_stamp is not None and self.filtered_odom_stamp is not None else "UNVERIFIED"}',
              flush=True)

    def _map_cb(self, msg):
        self.map_seen = (
            msg.info.width > 0 and msg.info.height > 0
            and len(msg.data) == msg.info.width * msg.info.height
        )
        if self.map_seen:
            self._stop_monitoring('map')

    def _canonical_map_cb(self, msg):
        self.canonical_map_grid = msg

    def _navigation_map_cb(self, msg):
        self.navigation_map_grid = msg

    def _global_costmap_cb(self, msg):
        self.global_costmap_grid = msg

    def _navigation_map_metadata_cb(self, msg):
        try:
            value = json.loads(str(msg.data))
        except (TypeError, ValueError):
            self.navigation_map_metadata = {'ready': False, 'reason': 'metadata_json_invalid'}
            return
        self.navigation_map_metadata = value if isinstance(value, dict) else {
            'ready': False, 'reason': 'metadata_not_object'}

    def _command_owner_cb(self, msg):
        self.command_owner_seen = str(msg.data) in (
            'NONE', 'ESTOP', 'WEB_MANUAL', 'DIRECT_MANUAL', 'NAV2', 'TAG_ROUTE')

    def _topic_present(self, topic):
        try:
            return any(name == topic for name, _ in self.get_topic_names_and_types())
        except Exception:
            return False

    def _command_arbiter_graph_ready(self):
        return self._command_arbiter_node_ready() and self._swerve_controller_node_ready()

    def _command_arbiter_node_ready(self):
        publishers = self.get_publishers_info_by_topic('/cmd_vel_selected')
        arbiter_publishes_selected = any(
            row.node_name == 'command_arbiter' for row in publishers)
        return bool(self._node_present('command_arbiter')
                    and self.command_owner_seen
                    and arbiter_publishes_selected)

    def _swerve_controller_node_ready(self):
        subscribers = self.get_subscriptions_info_by_topic('/cmd_vel_selected')
        controller_subscribes_selected = any(
            row.node_name == 'swerve_controller' for row in subscribers)
        return bool(self._node_present('swerve_controller')
                    and controller_subscribes_selected)

    def _command_arbiter_stage(self, result, deadline):
        ready = self._spin_until(self._command_arbiter_node_ready, deadline)
        return self._report_stage(
            result,
            'COMMAND_ARBITER_READY',
            ready,
            'command_arbiter_or_selected_output_unavailable',
            success_detail=' node=/command_arbiter publisher=/cmd_vel_selected command_owner_observed',
        )

    def _swerve_controller_stage(self, result, deadline):
        ready = self._spin_until(self._swerve_controller_node_ready, deadline)
        return self._report_stage(
            result,
            'SWERVE_CONTROLLER_READY',
            ready,
            'swerve_controller_or_selected_command_subscription_unavailable',
            success_detail=' node=/swerve_controller subscriber=/cmd_vel_selected',
        )

    @staticmethod
    def _gazebo_process_present():
        """Confirm the production Gazebo Classic server process exists."""
        try:
            for entry in os.scandir('/proc'):
                if not entry.name.isdigit():
                    continue
                try:
                    with open(os.path.join(entry.path, 'comm'), encoding='utf-8') as stream:
                        if stream.read().strip() == 'gzserver':
                            return True
                except (FileNotFoundError, PermissionError, ProcessLookupError):
                    # A process may exit while /proc is being enumerated.
                    continue
        except OSError:
            return False
        return False

    def _node_present(self, node):
        wanted = node.lstrip('/')
        try:
            return any(name.lstrip('/') == wanted or name.endswith('/' + wanted)
                       for name, _ in self.get_node_names_and_namespaces())
        except Exception:
            return False

    def _mapping_runtime_authority(self):
        try:
            names = {name.lstrip('/') for name, _namespace in self.get_node_names_and_namespaces()}
            map_publishers = self.get_publishers_info_by_topic('/map')
            tf_publishers = self.get_publishers_info_by_topic('/tf')
        except Exception as exc:
            return False, f'ros_graph_query_failed:{type(exc).__name__}'
        conflicts = sorted(names.intersection({
            'map_server', 'lifecycle_manager_mapping_map', 'ekf_v30e',
            'v30e_sim_node', 'tag_route_planner',
        }))
        map_nodes = {str(row.node_name).lstrip('/') for row in map_publishers}
        tf_nodes = {str(row.node_name).lstrip('/') for row in tf_publishers}
        if 'slam_toolbox' not in names:
            return False, 'slam_toolbox_node_missing'
        if conflicts:
            return False, f'incompatible_mapping_nodes_present:{",".join(conflicts)}'
        if map_nodes != {'slam_toolbox'}:
            return False, f'/map_publishers_must_be_slam_toolbox_only:{sorted(map_nodes)}'
        if 'slam_toolbox' not in tf_nodes:
            return False, f'slam_toolbox_not_publishing_tf:{sorted(tf_nodes)}'
        if 'map_server' in names:
            return False, 'legacy_map_server_node_must_not_compete_with_live_SLAM'
        return True, (
            'live_map=/map publisher=slam_toolbox; map_to_odom_owner=slam_toolbox; '
            'canonical and registered navigation rasters use distinct topics'
        )

    def _mapping_slam_parameters(self, deadline):
        names = [
            'mode', 'map_frame', 'odom_frame', 'base_frame', 'scan_topic', 'map_name',
            'resolution', 'map_update_interval', 'minimum_travel_distance',
            'minimum_travel_heading', 'transform_publish_period',
            'use_scan_matching', 'do_loop_closing',
        ]
        response = self._service_call(
            self.slam_parameters, min(deadline, time.monotonic() + 3.0),
            lambda request: setattr(request, 'names', names),
        )
        if response is None or len(response.values) != len(names):
            return False, self.last_service_error or 'slam_toolbox parameters unavailable'
        values = dict(zip(names, response.values))

        def string(name):
            value = values[name]
            return value.string_value if value.type == ParameterType.PARAMETER_STRING else None

        def number(name):
            value = values[name]
            if value.type == ParameterType.PARAMETER_DOUBLE:
                return float(value.double_value)
            if value.type == ParameterType.PARAMETER_INTEGER:
                return float(value.integer_value)
            return None

        def boolean(name):
            value = values[name]
            return bool(value.bool_value) if value.type == ParameterType.PARAMETER_BOOL else None

        actual = {
            'mode': string('mode'), 'map_frame': string('map_frame'),
            'odom_frame': string('odom_frame'), 'base_frame': string('base_frame'),
            'scan_topic': string('scan_topic'), 'map_name': string('map_name'),
            'resolution': number('resolution'),
            'map_update_interval': number('map_update_interval'),
            'minimum_travel_distance': number('minimum_travel_distance'),
            'minimum_travel_heading': number('minimum_travel_heading'),
            'transform_publish_period': number('transform_publish_period'),
            'use_scan_matching': boolean('use_scan_matching'),
            'do_loop_closing': boolean('do_loop_closing'),
        }
        expected = {
            'mode': 'mapping', 'map_frame': 'map', 'odom_frame': 'odom',
            'base_frame': 'base_footprint', 'scan_topic': '/scan', 'map_name': '/map',
            'resolution': 0.05, 'map_update_interval': 2.0,
            'minimum_travel_distance': 0.15, 'minimum_travel_heading': 0.15,
            'transform_publish_period': 0.02,
            'use_scan_matching': True, 'do_loop_closing': True,
        }
        mismatches = []
        for name, wanted in expected.items():
            got = actual[name]
            if isinstance(wanted, float):
                valid = got is not None and math.isclose(got, wanted, rel_tol=0.0, abs_tol=1e-6)
            else:
                valid = got == wanted
            if not valid:
                mismatches.append(f'{name}={got!r}(expected {wanted!r})')
        detail = json.dumps(actual, sort_keys=True)
        return not mismatches, ('; '.join(mismatches) if mismatches else detail)

    def _mapping_input_frames_valid(self):
        if not self.scan_frame_id:
            self.mapping_tf_error = 'scan_frame_id_empty'
            return False
        if self.filtered_odom_frame_id != 'odom':
            self.mapping_tf_error = f'filtered_odom_frame_mismatch:{self.filtered_odom_frame_id!r}:expected_odom'
            return False
        if self.filtered_odom_child_frame_id != 'base_footprint':
            self.mapping_tf_error = f'filtered_odom_child_frame_mismatch:{self.filtered_odom_child_frame_id!r}:expected_base_footprint'
            return False
        if self.scan_stamp_message is None:
            self.mapping_tf_error = 'scan_stamp_unavailable'
            return False
        try:
            scan_time = rclpy.time.Time.from_msg(self.scan_stamp_message)
            self.tf.lookup_transform('base_footprint', self.scan_frame_id, scan_time)
            self.tf.lookup_transform('map', 'odom', scan_time)
            self.mapping_tf_error = None
            return True
        except (TransformException, RuntimeError, TypeError, ValueError) as exc:
            self.mapping_tf_error = f'{type(exc).__name__}:{str(exc)[:240]}'
            return False

    def _report_stage(self, result, name, ok, failure_reason=None,
                      success_detail=None):
        if ok is None:
            result['stages'][name] = None
            print(f'{name}=UNVERIFIED reason={failure_reason or "not_checked"}', flush=True)
            return None
        result['stages'][name] = bool(ok)
        timeline_name = TIMELINE_STAGE_NAMES.get(name)
        if timeline_name:
            self._record_timeline(
                timeline_name,
                status='PASS' if ok else 'FAIL',
                detail=success_detail.strip() if ok and success_detail else failure_reason,
            )
        if ok:
            print(f'{name}=PASS{success_detail or ""}', flush=True)
        else:
            print(f'{name}=FAIL reason={failure_reason or "readiness_timeout"}', flush=True)
            if self.log_path:
                print(f'RELEVANT_LOG={self.log_path}', flush=True)
        return ok

    def _stage(self, result, name, predicate, success_label=None,
               failure_reason='readiness_timeout'):
        ok = self._spin_until(predicate, self.deadline)
        detail = f' detail={success_label}' if success_label else None
        return self._report_stage(result, name, ok, failure_reason, detail)

    def _topic_stage(self, result, name, topic, seen_name):
        def ready():
            return self._topic_present(topic) and bool(getattr(self, seen_name))

        ok = self._spin_until(ready, self.deadline)
        if ok:
            return self._report_stage(result, name, True, success_detail=f' topic={topic}')
        reason = f'topic_missing:{topic}' if not self._topic_present(topic) else f'no_valid_message:{topic}'
        return self._report_stage(result, name, False, reason)

    def _lidar_pipeline(self, result, deadline):
        """Collect all three sensor stages under one bounded readiness window."""
        stages = (
            ('LIDAR_RAW_READY', '/lidar/points', 'points_seen'),
            ('LIDAR_FILTERED_READY', '/lidar/points_filtered', 'filtered_points_seen'),
            ('SCAN_READY', '/scan', 'scan_seen'),
        )

        def all_ready():
            return all(self._topic_present(topic) and bool(getattr(self, seen))
                       for _stage, topic, seen in stages)

        self._spin_until(all_ready, deadline)
        successful = True
        for name, topic, seen_name in stages:
            topic_exists = self._topic_present(topic)
            valid_message = bool(getattr(self, seen_name))
            if topic_exists and valid_message:
                self._report_stage(result, name, True, success_detail=f' topic={topic}')
                continue
            reason = f'topic_missing:{topic}' if not topic_exists else f'no_valid_message:{topic}'
            self._report_stage(result, name, False, reason)
            successful = False
        return successful

    def _spin_until(self, predicate, deadline):
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def _service_call(self, client, deadline, prepare=None):
        """Call one service while spinning; never call this from a predicate."""
        self.last_service_error = None
        service_name = getattr(client, 'service_name', 'ROS service')
        if not self._spin_until(client.service_is_ready, deadline):
            self.last_service_error = f'{service_name} service unavailable'
            return None
        request = client.srv_type.Request()
        if prepare:
            prepare(request)
        future = client.call_async(request)
        while rclpy.ok() and time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            self.last_service_error = f'{service_name} response timed out'
            return None
        try:
            return future.result()
        except Exception as exc:
            self.last_service_error = f'{type(exc).__name__}: {exc}'
            self.get_logger().warning(f'service call failed: {type(exc).__name__}: {exc}')
            return None

    def _clock_ready(self):
        if len(self.clock_samples) < 3:
            return False
        first, second, third = self.clock_samples[-3:]
        if not (second[0] > first[0] and third[0] > second[0]):
            return False
        return 0.0 < third[1] - first[1] <= CLOCK_SAMPLE_SPAN_TIMEOUT_S

    def _factory_ready(self):
        types = dict(self.get_service_names_and_types()).get('/spawn_entity', [])
        return (self.factory.service_is_ready()
                and 'gazebo_msgs/srv/SpawnEntity' in types)

    def _read_robot_description(self, deadline):
        response = self._service_call(
            self.robot_description, min(deadline, time.monotonic() + 2.0),
            lambda request: setattr(request, 'names', ['robot_description']),
        )
        if response is None or not response.values:
            return None
        description = response.values[0].string_value
        if not description:
            return None
        try:
            root = ET.fromstring(description)
        except ET.ParseError as exc:
            self.last_service_error = f'invalid robot_description XML: {exc}'
            return None
        valid = self._is_swerve_description(root)
        if not valid:
            self.last_service_error = (
                f'robot_description lacks required swerve links/joints for Gazebo entity {self.model}')
            return None
        return description

    @staticmethod
    def _is_swerve_description(root):
        if root.tag != 'robot' or not root.attrib.get('name'):
            return False
        links = {item.attrib.get('name') for item in root.findall('link')}
        joints = {item.attrib.get('name') for item in root.findall('joint')}
        return SWERVE_REQUIRED_LINKS.issubset(links) and SWERVE_REQUIRED_JOINTS.issubset(joints)

    def _wait_entity(self, deadline):
        self.last_spawn_error = f'Gazebo /model_states has not confirmed entity {self.model}'
        while time.monotonic() < deadline:
            if not self.model_states_seen:
                self.last_spawn_error = 'no live Gazebo sample on /model_states'
            elif self.model not in self.gazebo_model_names:
                self.last_spawn_error = (
                    f'entity {self.model} absent from /model_states; '
                    f'entities={sorted(self.gazebo_model_names)}'
                )
            else:
                self.last_spawn_error = None
                self._record_timeline(
                    'T6_ROBOT_ENTITY_CONFIRMED',
                    detail=f'entity={self.model}; source=/model_states',
                )
                return True
            rclpy.spin_once(self, timeout_sec=0.2)
        return False

    def _spawn_log_errors(self):
        if not self.log_path:
            return []
        try:
            with open(self.log_path, 'rb') as stream:
                stream.seek(0, os.SEEK_END)
                size = stream.tell()
                stream.seek(max(0, size - 131072))
                tail = stream.read().decode('utf-8', errors='replace')
        except OSError:
            return []
        matches = []
        for line in tail.splitlines():
            lowered = line.lower()
            if ('spawn' in lowered or 'entity' in lowered) and re.search(
                    r'failed|error|exception|unable|timed out|timeout', lowered):
                matches.append(line.strip()[:500])
        return matches[-3:]

    def _wait_controllers(self, result, deadline):
        names = ('joint_state_broadcaster', 'steering_controller', 'drive_controller')
        required_interfaces = {
            'steer_front_joint/position', 'steer_rear_joint/position',
            'wheel_front_drive_joint/velocity', 'wheel_rear_drive_joint/velocity',
        }
        required_topics = {
            '/steering_controller/commands', '/drive_controller/commands',
        }
        self.controller_states = {}
        self.hardware_interfaces = set()
        self.controller_topic_names = set()
        self.controller_failure = 'controller activation sequence did not complete'
        self.controller_manager_callable = False
        active_timeline_names = {
            'joint_state_broadcaster': 'T8_JOINT_STATE_BROADCASTER_ACTIVE',
            'steering_controller': 'T9_STEERING_CONTROLLER_ACTIVE',
            'drive_controller': 'T10_DRIVE_CONTROLLER_ACTIVE',
        }
        # The launch event chain starts these nodes only after the drive
        # controller spawner has exited successfully. Waiting for both graph
        # endpoints avoids sending list_controllers requests while a spawner is
        # still configuring/activating a controller.
        sequence_ready = self._spin_until(
            lambda: self._node_present('command_arbiter')
            and self._node_present('swerve_controller'), deadline)
        last_query_error = None
        if not sequence_ready:
            self.controller_failure = 'controller_activation_sequence_not_complete'

        next_query_at = time.monotonic()
        while sequence_ready and time.monotonic() < deadline:
            if time.monotonic() < next_query_at:
                rclpy.spin_once(self, timeout_sec=min(
                    0.1, max(0.0, next_query_at - time.monotonic())))
                continue
            response = self._service_call(
                self.controllers,
                min(deadline, time.monotonic() + CONTROLLER_SERVICE_RESPONSE_TIMEOUT_S),
            )
            if response:
                self.controller_manager_callable = True
                last_query_error = None
                self.controller_states = {
                    c.name: c.state for c in response.controller
                }
                self._capture_log_timeline()
                for controller_name, timeline_name in active_timeline_names.items():
                    if self.controller_states.get(controller_name) == 'active':
                        self._record_timeline(
                            timeline_name,
                            detail=f'{controller_name}=active; source=list_controllers',
                        )
            else:
                last_query_error = self.last_service_error or 'list_controllers_response_unavailable'
            inactive = {name: self.controller_states.get(name, 'missing')
                        for name in names if self.controller_states.get(name) != 'active'}
            if inactive:
                if response:
                    self.controller_failure = 'controllers_not_active:' + ','.join(
                        f'{name}={state}' for name, state in inactive.items())
                elif last_query_error:
                    self.controller_failure = f'list_controllers_unavailable:{last_query_error}'
                next_query_at = time.monotonic() + CONTROLLER_QUERY_INTERVAL_S
                continue
            hardware = self._service_call(
                self.hardware, min(deadline, time.monotonic() + CONTROLLER_SERVICE_RESPONSE_TIMEOUT_S))
            if hardware is None:
                self.controller_failure = self.last_service_error or 'hardware interface query failed'
                next_query_at = time.monotonic() + CONTROLLER_QUERY_INTERVAL_S
                continue
            interface_rows = list(hardware.command_interfaces)
            self.hardware_interfaces = {row.name.lstrip('/') for row in interface_rows}
            missing_interfaces = required_interfaces - self.hardware_interfaces
            unclaimed = {row.name.lstrip('/') for row in interface_rows
                         if row.name.lstrip('/') in required_interfaces and not row.is_claimed}
            if missing_interfaces or unclaimed:
                details = []
                if missing_interfaces:
                    details.append('missing_interfaces=' + ','.join(sorted(missing_interfaces)))
                if unclaimed:
                    details.append('unclaimed_interfaces=' + ','.join(sorted(unclaimed)))
                self.controller_failure = ';'.join(details)
                next_query_at = time.monotonic() + CONTROLLER_QUERY_INTERVAL_S
                continue
            try:
                self.controller_topic_names = {
                    name for name, _types in self.get_topic_names_and_types()
                }
            except Exception:
                self.controller_topic_names = set()
            missing_topics = required_topics - self.controller_topic_names
            if missing_topics:
                self.controller_failure = 'missing_command_topics:' + ','.join(sorted(missing_topics))
                next_query_at = time.monotonic() + CONTROLLER_QUERY_INTERVAL_S
                continue
            self.controller_failure = None
            break

        manager_failure = None
        if not self.controller_manager_callable:
            manager_failure = (
                'not_queried_until_activation_chain_complete' if not sequence_ready
                else last_query_error or 'list_controllers_never_responded'
            )
        self._report_stage(
            result, 'CONTROLLER_MANAGER_READY',
            self.controller_manager_callable if sequence_ready else None,
            manager_failure,
            ' service=/controller_manager/list_controllers responded' if self.controller_manager_callable else None,
        )
        controller_stages = {
            'joint_state_broadcaster': 'JOINT_STATE_BROADCASTER_ACTIVE',
            'steering_controller': 'STEERING_CONTROLLER_ACTIVE',
            'drive_controller': 'DRIVE_CONTROLLER_ACTIVE',
        }
        for name, stage in controller_stages.items():
            state = self.controller_states.get(name)
            controller_status = (
                state == 'active' if state is not None or self.controller_manager_callable
                else None
            )
            self._report_stage(
                result, stage, controller_status,
                f'{name}_state={state or "not_observed"}',
                f' {name}=active' if state == 'active' else None,
            )
        return self.controller_failure is None

    def _print_required_status_summary(self, result):
        """Emit one concise, independently reported startup result per layer."""
        mode = getattr(self, 'mode', 'navigation')
        required_nodes = getattr(
            self, 'nav2_required_nodes',
            NAV2_STATIC_MAP_LIFECYCLE_NODES + ('lifecycle_manager_navigation',),
        )
        groups = (
            ('GAZEBO_READY', ('GAZEBO_PROCESS_READY', 'CLOCK_READY',
                              'GAZEBO_FACTORY_READY', 'GAZEBO_WORLD_READY')),
            ('ROBOT_SPAWNED', ('ROBOT_SPAWNED',)),
            ('CONTROLLER_MANAGER_READY', ('CONTROLLER_MANAGER_READY',)),
            ('JOINT_STATE_BROADCASTER_ACTIVE', ('JOINT_STATE_BROADCASTER_ACTIVE',)),
            ('STEERING_CONTROLLER_ACTIVE', ('STEERING_CONTROLLER_ACTIVE',)),
            ('DRIVE_CONTROLLER_ACTIVE', ('DRIVE_CONTROLLER_ACTIVE',)),
            ('COMMAND_ARBITER_READY', ('COMMAND_ARBITER_READY',)),
            ('SWERVE_CONTROLLER_READY', ('SWERVE_CONTROLLER_READY',)),
            ('ODOM_READY', ('ODOM_READY',)),
            ('LIDAR_READY', ('LIDAR_RAW_READY', 'LIDAR_FILTERED_READY', 'SCAN_READY')),
            ('TF_READY', ('TF_READY',)),
            ('NAV2_READY', tuple(f'NODE_{name.upper()}_READY' for name in required_nodes)
             + ('NAV2_LIFECYCLE_READY',
                'MAP_READY' if mode == 'unified' else 'MAP_FILE_READY',
                *(('REGISTERED_NAVIGATION_MAP_READY',) if mode == 'unified' else ()),
                'COMPUTE_PATH_ACTION_SERVER_READY', 'ACTION_SERVER_READY')),
            ('BRIDGE_READY', ('ROS_BRIDGE_R01_READY',)),
        )
        for summary_name, stages in groups:
            observed = [result['stages'].get(name) for name in stages]
            if all(value is True for value in observed):
                status = 'PASS'
                detail = ''
            elif any(value is False for value in observed):
                status = 'FAIL'
                failed = [name for name, value in zip(stages, observed) if value is False]
                detail = ' failed=' + ','.join(failed)
            else:
                status = 'UNVERIFIED'
                pending = [name for name, value in zip(stages, observed) if value is None]
                detail = ' not_checked=' + ','.join(pending)
            print(f'{summary_name}={status}{detail}', flush=True)

    @staticmethod
    def _lifecycle_state_value(state):
        if state is None:
            return {'id': None, 'label': 'unavailable'}
        return {'id': int(state.id), 'label': str(state.label).strip().lower()}

    @staticmethod
    def _lifecycle_states_text(states):
        return ','.join(
            f'/{name}={state.get("label", "unknown")}({state.get("id")})'
            for name, state in states.items()
        )

    @staticmethod
    def _lifecycle_states_are(states, wanted_id):
        return bool(states) and all(
            state.get('id') == wanted_id for state in states.values()
        )

    @staticmethod
    def _lifecycle_states_settled(states):
        return bool(states) and all(
            state.get('id') is not None
            and state.get('label') in NAV2_SETTLED_STATE_LABELS
            for state in states.values()
        )

    def _wait_lifecycle_settled(self, deadline):
        states = {}
        transition_reported = False
        while time.monotonic() < deadline:
            states = self._lifecycle_snapshot(deadline)
            if self._lifecycle_states_settled(states):
                return states
            if not transition_reported and any(
                state.get('label') in NAV2_TRANSITION_STATE_LABELS
                for state in states.values()
            ):
                transition_reported = True
                print('NAV2_LIFECYCLE_TRANSITION_IN_PROGRESS=PASS '
                      f'states={self._lifecycle_states_text(states)}', flush=True)
            rclpy.spin_once(self, timeout_sec=0.2)
        return states

    def _lifecycle_snapshot(self, deadline):
        states = {}
        for name, client in self.lifecycle.items():
            response = self._service_call(client, min(deadline, time.monotonic() + 2.0))
            state = response.current_state if response else None
            states[name] = self._lifecycle_state_value(state)
        return states

    def _nav2_graph_counts(self):
        counts = {'map_server': 0, 'canonical_map_server': 0,
                  'lifecycle_manager_navigation': 0,
                  'lifecycle_manager_mapping_map': 0}
        for name, _namespace in self.get_node_names_and_namespaces():
            if name in counts:
                counts[name] += 1
        return counts

    def _read_nav2_manager_configuration(self, deadline):
        response = self._service_call(
            self.nav_lifecycle_manager_parameters,
            min(deadline, time.monotonic() + 3.0),
            lambda request: setattr(request, 'names', ['autostart', 'node_names']),
        )
        if response is None or len(response.values) != 2:
            return None, self.last_service_error or 'lifecycle manager parameters unavailable'
        autostart, node_names = response.values
        if autostart.type != ParameterType.PARAMETER_BOOL:
            return None, f'autostart_parameter_wrong_type:{autostart.type}'
        if node_names.type != ParameterType.PARAMETER_STRING_ARRAY:
            return None, f'node_names_parameter_wrong_type:{node_names.type}'
        return {
            'autostart': bool(autostart.bool_value),
            'node_names': list(node_names.string_array_value),
        }, None

    def _read_map_yaml_parameter(self, deadline):
        client = (self.canonical_map_parameters if self.mode == 'unified'
                  else self.map_parameters)
        response = self._service_call(
            client, min(deadline, time.monotonic() + 3.0),
            lambda request: setattr(request, 'names', ['yaml_filename']),
        )
        if response is None or not response.values:
            return None, self.last_service_error or 'selected navigation map_server yaml_filename unavailable'
        value = response.values[0]
        if value.type != ParameterType.PARAMETER_STRING:
            return None, f'map_server_yaml_filename_wrong_type:{value.type}'
        actual = os.path.realpath(value.string_value)
        if self.expected_map_file and actual != self.expected_map_file:
            return actual, f'map_server_yaml_filename_mismatch:expected={self.expected_map_file}:actual={actual}'
        return actual, None

    @staticmethod
    def _grid_geometry(msg):
        if msg is None:
            return None
        origin = msg.info.origin
        q = origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        resolution = float(msg.info.resolution)
        width, height = int(msg.info.width), int(msg.info.height)
        return {
            'frame_id': str(msg.header.frame_id or ''),
            'resolution': resolution, 'width': width, 'height': height,
            'origin_x': float(origin.position.x), 'origin_y': float(origin.position.y),
            'origin_yaw': yaw,
            'min_x': float(origin.position.x),
            'max_x': float(origin.position.x) + width * resolution,
            'min_y': float(origin.position.y),
            'max_y': float(origin.position.y) + height * resolution,
            'valid': (width > 0 and height > 0 and resolution > 0.0
                      and len(msg.data) == width * height
                      and all(math.isfinite(value) for value in (
                          resolution, float(origin.position.x),
                          float(origin.position.y), yaw))),
        }

    def _unified_navigation_map_error(self):
        """Validate the published bundle, registration, transformed grid, and Nav2 consumer."""
        if self.expected_canonical_revision is None:
            return 'published_bundle_manifest_or_nav2_artifact_missing_or_mismatched'
        canonical = self._grid_geometry(self.canonical_map_grid)
        navigation = self._grid_geometry(self.navigation_map_grid)
        costmap = self._grid_geometry(self.global_costmap_grid)
        status = self.navigation_map_metadata
        if not canonical or not canonical['valid'] or canonical['frame_id'] != 'map':
            return 'canonical_map_not_ready_or_invalid'
        if not navigation or not navigation['valid'] or navigation['frame_id'] != 'map':
            return 'registered_navigation_map_not_ready_or_invalid'
        if not isinstance(status, dict) or status.get('ready') is not True:
            return 'registered_navigation_map_status_not_ready'
        try:
            canonical_revision = int(status.get('canonical_map_revision'))
            registration_revision = int(status.get('registration_revision'))
        except (TypeError, ValueError):
            return 'navigation_map_revision_metadata_invalid'
        if canonical_revision != self.expected_canonical_revision:
            return (f'canonical_revision_mismatch:expected={self.expected_canonical_revision}:'
                    f'actual={canonical_revision}')
        if (status.get('navigation_map_source') != 'PUBLISHED_CANONICAL_REGISTERED'
                or not str(status.get('active_map_id') or '').startswith('SLAM-')
                or not str(status.get('active_map_revision') or '').startswith('session-')
                or registration_revision <= 0
                or not status.get('registration_source')):
            return 'registered_navigation_map_identity_or_registration_invalid'
        if status.get('frame_id') != 'map':
            return f'navigation_map_frame_mismatch:{status.get("frame_id")}'
        try:
            for key in ('width', 'height'):
                if int(status.get(key) or 0) != navigation[key]:
                    return f'navigation_map_{key}_metadata_mismatch'
            for key in ('resolution', 'origin_x', 'origin_y'):
                if not math.isclose(float(status.get(key)), navigation[key], abs_tol=1e-5):
                    return f'navigation_map_{key}_metadata_mismatch'
            if (int(status.get('source_width') or 0) != canonical['width']
                    or int(status.get('source_height') or 0) != canonical['height']
                    or not math.isclose(float(status.get('source_resolution')), canonical['resolution'], abs_tol=1e-6)
                    or not math.isclose(float(status.get('source_origin_x')), canonical['origin_x'], abs_tol=1e-5)
                    or not math.isclose(float(status.get('source_origin_y')), canonical['origin_y'], abs_tol=1e-5)
                    or not math.isclose(float(status.get('source_origin_yaw')), canonical['origin_yaw'], abs_tol=1e-5)):
                return 'navigation_map_source_geometry_does_not_match_published_canonical_grid'
        except (TypeError, ValueError):
            return 'navigation_map_geometry_metadata_invalid'
        registration = status.get('registration')
        if not isinstance(registration, dict):
            return 'navigation_map_registration_transform_missing'
        try:
            tx, ty, angle = (float(registration[key]) for key in ('tx', 'ty', 'yaw'))
        except (KeyError, TypeError, ValueError):
            return 'navigation_map_registration_transform_invalid'
        if not all(math.isfinite(value) for value in (tx, ty, angle)):
            return 'navigation_map_registration_transform_non_finite'
        corners = []
        cosine, sine = math.cos(angle), math.sin(angle)
        source_cosine, source_sine = math.cos(canonical['origin_yaw']), math.sin(canonical['origin_yaw'])
        for local_x, local_y in ((0.0, 0.0),
                                 (canonical['width'] * canonical['resolution'], 0.0),
                                 (0.0, canonical['height'] * canonical['resolution']),
                                 (canonical['width'] * canonical['resolution'],
                                  canonical['height'] * canonical['resolution'])):
            x = canonical['origin_x'] + source_cosine * local_x - source_sine * local_y
            y = canonical['origin_y'] + source_sine * local_x + source_cosine * local_y
            corners.append((tx + cosine * x - sine * y,
                             ty + sine * x + cosine * y))
        expected = {
            'min_x': min(point[0] for point in corners),
            'max_x': max(point[0] for point in corners),
            'min_y': min(point[1] for point in corners),
            'max_y': max(point[1] for point in corners),
        }
        for key, value in expected.items():
            try:
                actual = float(status[key])
            except (KeyError, TypeError, ValueError):
                return f'navigation_map_registered_extent_missing:{key}'
            if not math.isclose(actual, value, abs_tol=canonical['resolution'] + 1e-4):
                return f'navigation_map_registered_extent_mismatch:{key}'
        if not costmap or not costmap['valid'] or costmap['frame_id'] != 'map':
            return 'global_costmap_not_ready_or_invalid'
        tolerance = max(navigation['resolution'], costmap['resolution']) + 1e-4
        if (costmap['min_x'] > navigation['min_x'] + tolerance
                or costmap['min_y'] > navigation['min_y'] + tolerance
                or costmap['max_x'] < navigation['max_x'] - tolerance
                or costmap['max_y'] < navigation['max_y'] - tolerance):
            return 'global_costmap_extent_does_not_cover_registered_navigation_map'
        try:
            map_nodes = {str(row.node_name).lstrip('/')
                         for row in self.get_publishers_info_by_topic('/map')}
            canonical_nodes = {str(row.node_name).lstrip('/')
                               for row in self.get_publishers_info_by_topic('/canonical_map')}
            navigation_nodes = {str(row.node_name).lstrip('/')
                                for row in self.get_publishers_info_by_topic('/navigation_map')}
            costmap_subscribers = {str(row.node_name).lstrip('/')
                                   for row in self.get_subscriptions_info_by_topic('/navigation_map')}
        except Exception as exc:
            return f'navigation_map_graph_query_failed:{type(exc).__name__}'
        if map_nodes != {'slam_toolbox'}:
            return f'/map_publishers_must_be_slam_toolbox_only:{sorted(map_nodes)}'
        if canonical_nodes != {'canonical_map_server'}:
            return f'/canonical_map_publishers_must_be_canonical_map_server_only:{sorted(canonical_nodes)}'
        if navigation_nodes != {'swerve_bridge'}:
            return f'/navigation_map_publishers_must_be_swerve_bridge_only:{sorted(navigation_nodes)}'
        if not costmap_subscribers.intersection({'global_costmap', 'planner_server', 'controller_server'}):
            return f'nav2_global_costmap_not_subscribed_to_navigation_map:{sorted(costmap_subscribers)}'
        print(
            'NAVIGATION_MAP_CONTRACT=PASS '
            f'topic=/navigation_map source=PUBLISHED_CANONICAL_REGISTERED '
            f'id={status.get("navigation_map_id")} revision={status.get("navigation_map_revision")} '
            f'canonical_revision={self.expected_canonical_revision} '
            f'size={navigation["width"]}x{navigation["height"]} '
            f'extent=[{navigation["min_x"]:.3f},{navigation["max_x"]:.3f}]x'
            f'[{navigation["min_y"]:.3f},{navigation["max_y"]:.3f}] '
            f'global_costmap={costmap["width"]}x{costmap["height"]} '
            f'costmap_extent=[{costmap["min_x"]:.3f},{costmap["max_x"]:.3f}]x'
            f'[{costmap["min_y"]:.3f},{costmap["max_y"]:.3f}]', flush=True)
        return None

    def _wait_unified_navigation_map(self, deadline):
        last_error = 'registered_navigation_map_not_ready'
        while time.monotonic() < deadline:
            last_error = self._unified_navigation_map_error()
            if last_error is None:
                return True, None
            rclpy.spin_once(self, timeout_sec=0.2)
        return False, last_error

    def _startup_state(self):
        return read_nav2_startup_state(self.lifecycle_state_file)

    def _update_startup_state(self, state, **details):
        payload = update_nav2_startup_state(self.lifecycle_state_file, state, **details)
        return payload

    def _lifecycle_log_offset(self):
        if not self.log_path:
            return None
        try:
            return os.path.getsize(self.log_path)
        except OSError:
            return None

    def _lifecycle_manager_log_lines(self, offset):
        if not self.log_path or offset is None:
            return []
        try:
            with open(self.log_path, 'rb') as stream:
                stream.seek(offset)
                content = stream.read().decode('utf-8', errors='replace')
        except OSError:
            return []
        lines = []
        for line in content.splitlines():
            if ('[lifecycle_manager_navigation]' in line or '[map_server]' in line
                    or 'transition invoked while in transition' in line):
                lines.append(re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', line.strip()))
        return lines[-80:]

    def _print_lifecycle_manager_log(self, offset):
        lines = self._lifecycle_manager_log_lines(offset)
        first_failure = None
        for line in lines:
            match = re.search(r'Failed to change state for node:\s*([^.:]+)', line)
            if match:
                first_failure = match.group(1).strip()
                break
        if first_failure:
            print(f'NAV2_FIRST_LIFECYCLE_FAILURE={first_failure}', flush=True)
        for line in lines:
            print(f'NAV2_LIFECYCLE_LOG={line}', flush=True)
        if not lines:
            print('NAV2_LIFECYCLE_LOG=UNAVAILABLE', flush=True)

    def _deferred_nav2_contract(self, deadline):
        graph_counts = self._nav2_graph_counts()
        print('NAV2_LIFECYCLE_GRAPH=' + ','.join(
            f'{name}={count}' for name, count in graph_counts.items()), flush=True)
        expected_map_server_count = 1 if self.mode == 'navigation' else 0
        expected_canonical_server_count = 1 if self.mode == 'unified' else 0
        if (graph_counts['map_server'] != expected_map_server_count
                or graph_counts['canonical_map_server'] != expected_canonical_server_count
                or graph_counts['lifecycle_manager_navigation'] != 1
                or graph_counts['lifecycle_manager_mapping_map'] != 0):
            return False, f'invalid_nav2_lifecycle_graph:{graph_counts}'
        manager, error = self._read_nav2_manager_configuration(deadline)
        if error:
            return False, error
        print('NAV2_LIFECYCLE_MANAGER_CONFIG=' + json.dumps(manager, sort_keys=True), flush=True)
        expected_names = list(self.nav2_lifecycle_nodes)
        if manager['autostart']:
            return False, 'deferred_nav2_contract_violated:autostart=true'
        if manager['node_names'] != expected_names:
            return False, f'nav2_lifecycle_manager_node_names_mismatch:{manager["node_names"]}'
        if self.mode in ('navigation', 'unified'):
            if self.mode == 'unified' and self.expected_canonical_revision is None:
                return False, 'published_map_bundle_revision_unavailable_or_selected_map_not_in_bundle'
            map_yaml, error = self._read_map_yaml_parameter(deadline)
            if error:
                return False, error
            server = 'canonical_map_server' if self.mode == 'unified' else 'map_server'
            revision = (f' canonical_revision={self.expected_canonical_revision}'
                        if self.mode == 'unified' else '')
            print(f'MAP_SERVER_YAML=PASS server=/{server} path={map_yaml}{revision}', flush=True)
            if self.mode == 'unified':
                print('NAV2_MAP_SOURCE=REGISTERED_CANONICAL '
                      'canonical_topic=/canonical_map navigation_topic=/navigation_map '
                      'live_slam_topic=/map', flush=True)
        else:
            print('NAV2_MAP_SOURCE=LIVE_SLAM topic=/map map_server=absent', flush=True)
        return True, None

    def _ensure_nav2_lifecycle_ready(self, result, deadline):
        states = self._wait_lifecycle_settled(deadline)
        state_text = self._lifecycle_states_text(states)
        print(f'NAV2_LIFECYCLE_BEFORE={state_text}', flush=True)
        startup = self._startup_state()
        startup_state = startup['state']
        print(f'NAV2_STARTUP_STATE_BEFORE={startup_state}', flush=True)

        def fail(reason, after=None, log_offset=None):
            if after is not None:
                print('NAV2_LIFECYCLE_AFTER=' + self._lifecycle_states_text(after), flush=True)
            self._report_stage(result, 'NAV2_LIFECYCLE_READY', False, reason)
            result['stages']['NAV2_STARTUP_STATE'] = 'FAILED'
            print('NAV2_STARTUP_STATE=FAILED', flush=True)
            if startup_state not in ('FAILED', 'INVALID'):
                self._update_startup_state(
                    'FAILED', failure=reason,
                    lifecycle_before=states,
                    lifecycle_after=after,
                )
            if log_offset is not None:
                self._print_lifecycle_manager_log(log_offset)
            return False, states, reason

        if startup_state == 'INVALID':
            return fail('nav2_startup_state_file_invalid')
        if startup_state == 'FAILED':
            return fail('previous_nav2_startup_request_failed_without_retry')
        if not states:
            return fail('nav2_lifecycle_state_snapshot_unavailable')

        if self._lifecycle_states_are(states, 3):
            if startup_state not in ('REQUESTED', 'IN_PROGRESS', 'ACTIVE'):
                return fail(f'unexpected_nav2_active_before_startup_request:{startup_state}')
            self._update_startup_state('ACTIVE', lifecycle_after=states)
            result['stages']['NAV2_STARTUP_STATE'] = 'ACTIVE'
            self._report_stage(
                result, 'NAV2_LIFECYCLE_READY', True,
                success_detail=' active=' + ','.join('/' + name for name in self.nav2_lifecycle_nodes),
            )
            print('NAV2_STARTUP_STATE=ACTIVE', flush=True)
            return True, states, None

        if startup_state in ('REQUESTED', 'IN_PROGRESS', 'ACTIVE'):
            # A prior readiness process owns the only request for this launch.
            # Never issue another STARTUP while it is in flight or after a
            # partial transition; inspect the settled state and report it.
            reason = ('nav2_startup_request_already_consumed_but_nodes_not_active:'
                      f'{startup_state}:{state_text}')
            return fail(reason, after=states)

        if not self._lifecycle_states_are(states, 1):
            reason = f'nav2_lifecycle_settled_partial_state_no_startup:{state_text}'
            return fail(reason, after=states)

        contract_ok, contract_error = self._deferred_nav2_contract(deadline)
        if not contract_ok:
            return fail(contract_error or 'deferred_nav2_contract_invalid', after=states)

        confirmed = self._wait_lifecycle_settled(deadline)
        print('NAV2_LIFECYCLE_PRE_REQUEST_CONFIRM='
              + self._lifecycle_states_text(confirmed), flush=True)
        if not self._lifecycle_states_are(confirmed, 1):
            reason = ('nav2_lifecycle_state_changed_before_startup_request:'
                      + self._lifecycle_states_text(confirmed))
            return fail(reason, after=confirmed)
        states = confirmed

        claimed, request_state = claim_nav2_startup(
            self.lifecycle_state_file,
            lifecycle_before=states,
        )
        if not claimed:
            observed_state = request_state['state']
            print(f'NAV2_STARTUP_REQUEST_SKIPPED=PASS state={observed_state}', flush=True)
            states = self._wait_lifecycle_settled(deadline)
            if self._lifecycle_states_are(states, 3):
                self._update_startup_state('ACTIVE', lifecycle_after=states)
                result['stages']['NAV2_STARTUP_STATE'] = 'ACTIVE'
                return True, states, None
            reason = (
                f'nav2_startup_already_claimed_by_another_probe:{observed_state}:'
                f'{self._lifecycle_states_text(states)}'
            )
            print('NAV2_LIFECYCLE_AFTER=' + self._lifecycle_states_text(states), flush=True)
            self._report_stage(result, 'NAV2_LIFECYCLE_READY', False, reason)
            result['stages']['NAV2_STARTUP_STATE'] = observed_state
            return False, states, reason

        request_id = request_state.get('request_id')
        self._update_startup_state(
            'IN_PROGRESS', request_id=request_id,
            request_sent_at=datetime.now(timezone.utc).isoformat(),
            request_sent_monotonic_ns=time.monotonic_ns(),
        )
        result['stages']['NAV2_STARTUP_STATE'] = 'IN_PROGRESS'
        self._record_timeline(
            'T15_NAV2_LIFECYCLE_STARTUP_BEGIN',
            detail=f'one-shot STARTUP request_id={request_id}',
        )
        log_offset = self._lifecycle_log_offset()
        print('NAV2_STARTUP_REQUEST_SENT=PASS '
              f'request_id={request_id} wall_time={datetime.now(timezone.utc).isoformat()} '
              f'monotonic_ns={time.monotonic_ns()}', flush=True)
        response = self._service_call(
            self.nav_lifecycle_manager,
            deadline,
            lambda request: setattr(request, 'command', ManageLifecycleNodes.Request.STARTUP),
        )
        response_success = bool(response and response.success)
        print(f'NAV2_STARTUP_RESPONSE={str(response_success).lower()}', flush=True)
        after = self._lifecycle_snapshot(deadline)
        print('NAV2_LIFECYCLE_AFTER=' + self._lifecycle_states_text(after), flush=True)
        self._print_lifecycle_manager_log(log_offset)
        if not response_success:
            error = self.last_service_error or 'response.success=false'
            reason = f'lifecycle_manager_navigation_startup_failed:{error}'
            return fail(reason, after=after)

        states = self._wait_lifecycle_settled(deadline)
        if self._lifecycle_states_are(states, 3):
            self._update_startup_state('ACTIVE', lifecycle_after=states)
            result['stages']['NAV2_STARTUP_STATE'] = 'ACTIVE'
            self._report_stage(
                result, 'NAV2_LIFECYCLE_READY', True,
                success_detail=' active=' + ','.join('/' + name for name in self.nav2_lifecycle_nodes),
            )
            print('NAV2_STARTUP_STATE=ACTIVE', flush=True)
            return True, states, None

        reason = 'nav2_lifecycle_not_active_after_startup:' + self._lifecycle_states_text(states)
        return fail(reason, after=states)

    def _wait_map_file(self, deadline):
        if self.expected_map_file is None:
            return True
        while time.monotonic() < deadline:
            response = self._service_call(
                self.map_parameters,
                min(deadline, time.monotonic() + 2.0),
                lambda request: setattr(request, 'names', ['yaml_filename']),
            )
            if response is not None and response.values:
                actual = str(response.values[0].string_value)
                if os.path.realpath(actual) == self.expected_map_file:
                    return True
                self.get_logger().warning(
                    f'map_server yaml_filename={actual!r}, expected={self.expected_map_file!r}')
            rclpy.spin_once(self, timeout_sec=0.2)
        return False

    def _wait_ros_bridge(self, deadline):
        last_reason = 'bridge_identity_not_ready'
        while time.monotonic() < deadline:
            if not self._node_present('swerve_bridge'):
                last_reason = 'bridge_node_missing:/swerve_bridge'
                rclpy.spin_once(self, timeout_sec=0.2)
                continue
            response = self._service_call(
                self.bridge_parameters, min(deadline, time.monotonic() + 2.0),
                lambda request: setattr(request, 'names', ['robot_id', 'runtime_state']),
            )
            if response is None or len(response.values) < 2:
                last_reason = self.last_service_error or 'bridge parameter response missing'
                rclpy.spin_once(self, timeout_sec=0.2)
                continue
            robot_id = response.values[0].string_value
            runtime_state = response.values[1].string_value.upper()
            if robot_id != self.robot_id:
                last_reason = f'bridge_robot_id_mismatch:expected={self.robot_id}:actual={robot_id}'
                rclpy.spin_once(self, timeout_sec=0.2)
                continue
            if runtime_state != self.mode.upper():
                last_reason = f'bridge_runtime_state_mismatch:expected={self.mode.upper()}:actual={runtime_state}'
                rclpy.spin_once(self, timeout_sec=0.2)
                continue
            if not self.backend_url:
                last_reason = 'backend_health_url_not_configured'
                break
            try:
                with urllib.request.urlopen(
                    self.backend_url.rstrip('/') + '/api/health/', timeout=1.0
                ) as response_stream:
                    health = json.loads(response_stream.read().decode('utf-8'))
                online_robot_ids = health.get('online_robot_ids')
                if (health.get('ros_bridge') and health.get('ros')
                        and isinstance(online_robot_ids, list)
                        and self.robot_id in online_robot_ids):
                    print(f'BACKEND_R01_HEARTBEAT=PASS robot_id={self.robot_id} '
                          'source=/api/health online_robot_ids', flush=True)
                    return True, None
                last_reason = (
                    f'backend_has_no_live_ros_bridge:ros_bridge={health.get("ros_bridge")},'
                    f'ros={health.get("ros")},online_robot_ids={online_robot_ids}'
                )
            except (OSError, ValueError, urllib.error.URLError) as exc:
                last_reason = f'backend_health_error:{type(exc).__name__}:{exc}'
            rclpy.spin_once(self, timeout_sec=0.2)
        return False, last_reason

    def check(self, timeout):
        started, deadline = time.monotonic(), time.monotonic() + timeout
        self.deadline = deadline
        result = {'stages': {}, 'startup_wall_time': None, 'startup_timeline': [],
                  'spawn_entity_duration_s': None, 'nav_ready': False, 'reason': None}

        if not self._stage(
            result, 'GAZEBO_PROCESS_READY', self._gazebo_process_present,
            'process=gzserver', 'process_not_running:gzserver',
        ):
            return self._finish(result, 'Gazebo server process is not running')

        clock_deadline = min(deadline, time.monotonic() + CLOCK_READY_WAIT_TIMEOUT_S)
        clock_ok = self._spin_until(self._clock_ready, clock_deadline)
        if clock_ok:
            first, _ = self.clock_samples[-3]
            last, _ = self.clock_samples[-1]
            delta = (last - first) / 1_000_000_000.0
            self._report_stage(result, 'CLOCK_READY', True,
                               success_detail=f' samples=3 delta={delta:.6f}')
            self._stop_monitoring('clock')
        else:
            self._report_stage(result, 'CLOCK_READY', False, 'clock_not_advancing')
            return self._finish(result, 'clock_not_advancing')

        if not self._stage(
            result, 'GAZEBO_FACTORY_READY', self._factory_ready,
            'service_type=gazebo_msgs/srv/SpawnEntity',
            'service_missing_or_wrong_type:/spawn_entity:expected=gazebo_msgs/srv/SpawnEntity',
        ):
            actual_types = dict(self.get_service_names_and_types()).get('/spawn_entity', [])
            return self._finish(result, f'/spawn_entity unavailable or wrong type: {actual_types}')

        # Humble's standard Gazebo launch does not load the legacy world
        # properties service plugin. Confirm a live Gazebo world-state sample
        # instead; this same stream verifies the robot after SpawnEntity.
        if not self._stage(
            result, 'GAZEBO_WORLD_READY', lambda: self.model_states_seen,
            f'live_topic=/model_states models={len(self.gazebo_model_names)}',
            'no_live_sample:/model_states',
        ):
            return self._finish(result, 'Gazebo /model_states is not publishing')
        print(f'GAZEBO_WORLD={sorted(self.gazebo_model_names)}', flush=True)

        if not self._stage(
            result, 'ROBOT_DESCRIPTION_SERVICE_READY',
            self.robot_description.service_is_ready,
            '/robot_state_publisher/get_parameters available',
            'service_unavailable:/robot_state_publisher/get_parameters',
        ):
            return self._finish(result, 'robot_state_publisher parameter service unavailable')
        description = self._read_robot_description(deadline)
        try:
            description_root = ET.fromstring(description) if description else None
            description_valid = bool(
                description_root is not None and self._is_swerve_description(description_root))
        except ET.ParseError as exc:
            description_valid = False
            self.last_service_error = f'invalid robot_description XML: {exc}'
        if not description_valid:
            error = self.last_service_error or (
                f'robot_description missing or lacks required swerve links/joints for {self.model}')
            self._report_stage(result, 'ROBOT_DESCRIPTION_READY', False, error)
            return self._finish(result, f'robot_description validation failed: {error}')
        self._report_stage(
            result, 'ROBOT_DESCRIPTION_READY', True,
            success_detail=f' model={self.model} links={len(description_root.findall("link"))} '
                           f'joints={len(description_root.findall("joint"))}',
        )

        entity_confirmed = self._wait_entity(deadline)
        result['stages']['ROBOT_SPAWNED'] = entity_confirmed
        if result['stages']['ROBOT_SPAWNED']:
            self._report_stage(result, 'ROBOT_SPAWNED', True,
                               success_detail=f' entity={self.model}')
        else:
            reason = self.last_spawn_error or f'entity_not_created:{self.model}'
            spawn_log_errors = self._spawn_log_errors()
            if spawn_log_errors:
                reason += '; gazebo_spawn_runtime=' + ' | '.join(spawn_log_errors)
            self._report_stage(result, 'ROBOT_SPAWNED', False, reason)
            if self.log_path:
                print(f'SPAWN_RUNTIME_LOG={self.log_path}', flush=True)
        if not result['stages']['ROBOT_SPAWNED']:
            return self._finish(result, f'robot spawn failed: {self.last_spawn_error}')

        controllers_ok = self._wait_controllers(result, deadline)
        states_text = ','.join(f'{name}={state}' for name, state in self.controller_states.items())
        if controllers_ok:
            self._report_stage(result, 'CONTROLLERS_READY', True,
                               success_detail=f' states={states_text}')
            self._report_stage(
                result, 'CONTROLLER_HARDWARE_INTERFACES_READY', True,
                success_detail=' interfaces=' + ','.join(sorted(self.hardware_interfaces)),
            )
            self._report_stage(
                result, 'CONTROLLER_COMMAND_TOPICS_READY', True,
                success_detail=' topics=/steering_controller/commands,/drive_controller/commands',
            )
        else:
            reason = self.controller_failure or 'controller_manager_readiness_failed'
            self._report_stage(result, 'CONTROLLERS_READY', False, reason)
            self._report_stage(result, 'CONTROLLER_HARDWARE_INTERFACES_READY', False, reason)
            self._report_stage(result, 'CONTROLLER_COMMAND_TOPICS_READY', False, reason)
            print('CONTROLLER_DEBUG=check gazebo_ros2_control plugin, /controller_manager, '
                  'config/controllers.yaml, robot_description, and sequential spawner logs', flush=True)
            if self.log_path:
                print(f'CONTROLLER_RUNTIME_LOG={self.log_path}', flush=True)
        if not controllers_ok:
            return self._finish(result, f'required ros2_control controllers not ready: {self.controller_failure}')

        if not self._command_arbiter_stage(result, deadline):
            self._report_stage(result, 'SWERVE_CONTROLLER_READY', None,
                               'not_checked_after_command_arbiter_failure')
            return self._finish(result, 'command arbiter or selected velocity path is unavailable')
        if not self._swerve_controller_stage(result, deadline):
            return self._finish(result, 'swerve controller selected-command path is unavailable')

        if not self._topic_stage(result, 'JOINT_STATES_READY', '/joint_states', 'joints_seen'):
            return self._finish(result, 'no valid /joint_states message with required steering/drive joints')
        if not self._topic_stage(result, 'ODOM_READY', '/odom', 'odom_seen'):
            return self._finish(result, 'no valid message on /odom')
        if not self._topic_stage(result, 'FILTERED_ODOM_READY', '/odometry/filtered', 'filtered_odom_seen'):
            return self._finish(result, 'no valid message on /odometry/filtered')
        if not self._lidar_pipeline(result, deadline):
            missing = [topic for topic, seen_name in (
                ('/lidar/points', 'points_seen'),
                ('/lidar/points_filtered', 'filtered_points_seen'),
                ('/scan', 'scan_seen'),
            ) if not self._topic_present(topic) or not getattr(self, seen_name)]
            return self._finish(result, 'lidar_pipeline_not_ready:' + ','.join(missing))

        def tf_exists(target, source):
            try:
                self.tf.lookup_transform(target, source, rclpy.time.Time())
                return True
            except (TransformException, RuntimeError):
                return False

        if not self._stage(
            result, 'LOCAL_TF_READY',
            lambda: tf_exists('odom', 'base_footprint') and tf_exists('base_footprint', 'base_link'),
            'TF odom -> base_footprint -> base_link',
            'tf_unavailable:odom->base_footprint->base_link',
        ):
            return self._finish(result, 'required local TF odom->base_footprint->base_link unavailable')
        if self.mapping_required:
            if not self._topic_stage(result, 'MAP_READY', '/map', 'map_seen'):
                return self._finish(result, 'no valid message on /map')
            if not self._stage(
                result, 'GLOBAL_TF_READY',
                lambda: tf_exists('map', 'odom') and tf_exists('map', 'base_link'),
                'TF map -> odom -> base_link', 'tf_unavailable:map->odom->base_link',
            ):
                return self._finish(result, 'global TF chain map->odom->base_link unavailable')
            self._report_stage(result, 'TF_READY', True,
                               success_detail=' frames=odom->base_footprint,map->base_link')
            slam_ok = self._stage(result, 'SLAM_READY', lambda: self._node_present('slam_toolbox'),
                                  'SLAM Toolbox', 'node_missing:/slam_toolbox')
            if not slam_ok:
                return self._finish(result, 'SLAM Toolbox is not running')
            owner_ok, owner_detail = self._mapping_runtime_authority()
            if not self._report_stage(
                    result, 'SLAM_MAP_ODOM_AUTHORITY', owner_ok,
                    owner_detail, success_detail=f' detail={owner_detail}'):
                return self._finish(result, f'SLAM mapping authority invalid: {owner_detail}')
            parameters_ok, parameters_detail = self._mapping_slam_parameters(deadline)
            if not self._report_stage(
                    result, 'SLAM_PARAMETERS_READY', parameters_ok,
                    parameters_detail, success_detail=f' parameters={parameters_detail}'):
                return self._finish(result, f'SLAM Toolbox mapping parameters invalid: {parameters_detail}')
            input_tf_ok = self._spin_until(self._mapping_input_frames_valid, deadline)
            if not self._report_stage(
                    result, 'MAPPING_TF_READY', input_tf_ok,
                    self.mapping_tf_error or 'scan/odom timestamps or frames are not transform-compatible',
                    success_detail=(f' scan_frame={self.scan_frame_id} odom_frame={self.filtered_odom_frame_id} '
                                    f'base_frame={self.filtered_odom_child_frame_id} '
                                    'TF=map->odom->base_footprint->scan_frame at scan stamp')):
                self._print_mapping_sensor_inputs()
                return self._finish(result, f'mapping TF invalid: {self.mapping_tf_error}')
            self._print_mapping_sensor_inputs()
        if self.navigation_required:
            # In UNIFIED, mapping readiness above has already verified that
            # SLAM Toolbox owns the live /map and map->odom chain. Legacy
            # Navigation retains its separate localization contract.
            if not self._stage(
                result, 'GLOBAL_TF_READY',
                lambda: tf_exists('map', 'odom') and tf_exists('map', 'base_link'),
                'TF map -> odom -> base_link', 'tf_unavailable:map->odom->base_link',
            ):
                return self._finish(result, 'global TF chain map->odom->base_link unavailable')
            self._report_stage(result, 'TF_READY', True,
                               success_detail=' frames=odom->base_footprint,map->base_link')
            nav_nodes_ok = True
            for name in self.nav2_required_nodes:
                node_ok = self._stage(
                    result, f'NODE_{name.upper()}_READY', lambda name=name: self._node_present(name),
                    f'Nav2 {name}', f'node_missing:/{name}',
                )
                nav_nodes_ok = nav_nodes_ok and node_ok
            if not nav_nodes_ok:
                return self._finish(result, 'one or more required Nav2 nodes are not running')
            lifecycle_ok, lifecycle_states, lifecycle_error = self._ensure_nav2_lifecycle_ready(
                result, deadline)
            if not lifecycle_ok:
                return self._finish(result, f'Nav2 lifecycle not READY: {lifecycle_error}')
            if self.mode == 'unified':
                map_file_ok = self.map_seen and self._node_present('slam_toolbox')
                self._report_stage(
                    result, 'LIVE_SLAM_MAP_READY', map_file_ok,
                    None if map_file_ok else 'live_slam_map_unavailable',
                    success_detail=' publisher=/slam_toolbox topic=/map map_server=absent' if map_file_ok else None,
                )
            else:
                map_file_ok = self._wait_map_file(deadline)
                if map_file_ok:
                    self._report_stage(
                        result, 'MAP_FILE_READY', True,
                        success_detail=f' yaml_filename={self.expected_map_file}' if self.expected_map_file else None,
                    )
                else:
                    self._report_stage(
                        result, 'MAP_FILE_READY', False,
                        f'map_server_yaml_filename_mismatch:expected={self.expected_map_file}',
                    )
            if not map_file_ok:
                return self._finish(result, 'Nav2 map source is not ready')
            if not self._stage(result, 'COMPUTE_PATH_ACTION_SERVER_READY',
                               lambda: self.path_action.wait_for_server(timeout_sec=0.0),
                               '/compute_path_to_pose action server',
                               'action_server_unavailable:/compute_path_to_pose'):
                return self._finish(result, '/compute_path_to_pose action server not ready')
            if not self._stage(result, 'ACTION_SERVER_READY',
                               lambda: self.action.wait_for_server(timeout_sec=0.0),
                               '/navigate_to_pose action server',
                               'action_server_unavailable:/navigate_to_pose'):
                return self._finish(result, '/navigate_to_pose action server not ready')
            result['stages']['NAVIGATE_TO_POSE_ACTION_SERVER_READY'] = result['stages'].get(
                'ACTION_SERVER_READY', False)

        bridge_ok, bridge_reason = self._wait_ros_bridge(deadline)
        self._report_stage(
            result, 'ROS_BRIDGE_R01_READY', bridge_ok,
            bridge_reason,
            f' robot_id={self.robot_id} backend_health=connected' if bridge_ok else None,
        )
        if not bridge_ok:
            return self._finish(result, f'R01 ROS bridge is not ready: {bridge_reason}')
        if self.mode == 'unified':
            navigation_map_ok, navigation_map_error = self._wait_unified_navigation_map(deadline)
            self._report_stage(
                result, 'REGISTERED_NAVIGATION_MAP_READY', navigation_map_ok,
                navigation_map_error,
            )
            if not navigation_map_ok:
                return self._finish(
                    result, f'full registered Nav2 navigation map is not ready: {navigation_map_error}')
        result['startup_wall_time'] = time.monotonic() - started
        result['startup_sim_time'] = self.get_clock().now().nanoseconds * 1e-9
        result['nav_ready'] = True
        ready_label = ('Unified SLAM + registered full-map Nav2 runtime READY' if self.mode == 'unified'
                       else 'Mapping stack READY' if self.mode == 'mapping'
                       else 'Navigation stack READY')
        print(ready_label, flush=True)
        print('NAV_READY PASS', flush=True)
        self._print_required_status_summary(result)
        self._print_startup_timeline(result)
        return result

    def _finish(self, result, reason):
        result['reason'] = reason
        self._print_required_status_summary(result)
        self._print_startup_timeline(result)
        print(f'NAV_READY FAIL: {reason}', flush=True)
        if self.log_path:
            print(f'RELEVANT_LOG={self.log_path}', flush=True)
        return result


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--model', default='swerve_base')
    parser.add_argument('--robot-id', default='R01')
    parser.add_argument('--mode', choices=('unified', 'mapping', 'navigation'), default='unified')
    parser.add_argument('--map-file')
    parser.add_argument('--backend-url')
    parser.add_argument('--log-path')
    parser.add_argument(
        '--lifecycle-state-file',
        default=os.environ.get('WARETWIN_NAV2_LIFECYCLE_STATE_FILE'),
        help='managed runtime file that prevents repeated Nav2 STARTUP requests',
    )
    parser.add_argument('--json')
    args, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = Readiness(args.model, args.mode, args.map_file,
                     args.robot_id, args.backend_url, args.log_path,
                     args.lifecycle_state_file)
    try:
        try:
            result = node.check(args.timeout)
        except ExternalShutdownException:
            result = {'stages': {}, 'startup_wall_time': None, 'nav_ready': False,
                      'reason': 'ROS context shut down during readiness probe'}
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as stream:
            json.dump(result, stream, indent=2)
    return 0 if result['nav_ready'] else 1


if __name__ == '__main__':
    sys.exit(main())
