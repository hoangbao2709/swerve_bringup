#!/usr/bin/env python3
"""Authoritative readiness gate for a fresh swerve navigation session."""
import argparse
import json
import math
import os
import re
import sys
import time
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET

import rclpy
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnEntity
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.srv import ManageLifecycleNodes
from rcl_interfaces.srv import GetParameters
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


# navigation.launch.py owns exactly these Nav2 lifecycle nodes.  The project
# deliberately uses the V30E/tag localization filter for map->odom, so AMCL
# is not part of this launch contract and must not be invented as a readiness
# dependency.  If that launch is changed to include AMCL, add it here with its
# lifecycle entry at the same time.
NAV2_LIFECYCLE_NODES = (
    'map_server',
    'planner_server',
    'controller_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
)
NAV2_REQUIRED_NODES = NAV2_LIFECYCLE_NODES + ('lifecycle_manager_navigation',)
CONTROLLER_SERVICE_RESPONSE_TIMEOUT_S = 10.0
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

# Both map_server and SLAM Toolbox publish the canonical map with the ROS map
# QoS (reliable + transient-local). A volatile sensor-data subscriber can miss
# the only latched map sample when the readiness probe joins late.
MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Readiness(Node):
    """Checks live data and state, never just graph membership."""
    def __init__(self, model, mode='navigation', map_file=None,
                 robot_id='R01', backend_url=None, log_path=None):
        super().__init__('navigation_readiness_probe', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        if not self.get_parameter('use_sim_time').value:
            raise RuntimeError('FAIL: readiness probe use_sim_time is false')
        self.model = model
        self.mode = str(mode).lower()
        if self.mode not in ('mapping', 'navigation'):
            raise ValueError(f'unsupported readiness mode: {mode}')
        self.expected_map_file = os.path.realpath(map_file) if map_file else None
        self.robot_id = str(robot_id)
        self.backend_url = backend_url
        self.log_path = log_path
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
        self.points_seen = False
        self.filtered_points_seen = False
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
        if self.mode == 'mapping':
            self.readiness_subscriptions['map'] = self.create_subscription(
                OccupancyGrid, '/map', self._map_cb, MAP_QOS)
        self.readiness_subscriptions['points'] = self.create_subscription(
            PointCloud2, '/lidar/points',
            lambda msg: self._cloud_cb('points_seen', msg), sensor_qos)
        self.readiness_subscriptions['filtered_points'] = self.create_subscription(
            PointCloud2, '/lidar/points_filtered',
            lambda msg: self._cloud_cb('filtered_points_seen', msg), sensor_qos)
        self.factory = self.create_client(SpawnEntity, '/spawn_entity')
        self.robot_description = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')
        self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.hardware = self.create_client(
            ListHardwareInterfaces, '/controller_manager/list_hardware_interfaces')
        names = NAV2_LIFECYCLE_NODES if self.mode == 'navigation' else ()
        self.lifecycle = {name: self.create_client(GetState, f'/{name}/get_state') for name in names}
        self.nav_lifecycle_manager = (
            self.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
            if self.mode == 'navigation' else None)
        self.map_parameters = self.create_client(GetParameters, '/map_server/get_parameters')
        self.bridge_parameters = self.create_client(GetParameters, '/swerve_bridge/get_parameters')
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
            self._stop_monitoring('odom' if name == 'odom_seen' else 'filtered_odom')

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
            self._record_timeline('T14_SCAN_READY', detail='valid LaserScan sample')
            self._stop_monitoring('scan')

    def _map_cb(self, msg):
        self.map_seen = (
            msg.info.width > 0 and msg.info.height > 0
            and len(msg.data) == msg.info.width * msg.info.height
        )
        if self.map_seen:
            self._stop_monitoring('map')

    def _topic_present(self, topic):
        try:
            return any(name == topic for name, _ in self.get_topic_names_and_types())
        except Exception:
            return False

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

    def _report_stage(self, result, name, ok, failure_reason=None,
                      success_detail=None):
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

    def _wait_controllers(self, deadline):
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
        self.controller_failure = 'controller manager did not respond'
        active_timeline_names = {
            'joint_state_broadcaster': 'T8_JOINT_STATE_BROADCASTER_ACTIVE',
            'steering_controller': 'T9_STEERING_CONTROLLER_ACTIVE',
            'drive_controller': 'T10_DRIVE_CONTROLLER_ACTIVE',
        }
        while time.monotonic() < deadline:
            if self.controllers.service_is_ready():
                self._record_timeline(
                    'T7_CONTROLLER_MANAGER_READY',
                    detail='service=/controller_manager/list_controllers',
                )
            response = self._service_call(
                self.controllers,
                min(deadline, time.monotonic() + CONTROLLER_SERVICE_RESPONSE_TIMEOUT_S),
            )
            self.controller_states = {c.name: c.state for c in response.controller} if response else {}
            if response:
                self._capture_log_timeline()
                for controller_name, timeline_name in active_timeline_names.items():
                    if self.controller_states.get(controller_name) == 'active':
                        self._record_timeline(
                            timeline_name,
                            detail=f'{controller_name}=active; source=list_controllers',
                        )
            inactive = {name: self.controller_states.get(name, 'missing')
                        for name in names if self.controller_states.get(name) != 'active'}
            if inactive:
                self.controller_failure = 'controllers_not_active:' + ','.join(
                    f'{name}={state}' for name, state in inactive.items())
                rclpy.spin_once(self, timeout_sec=0.2)
                continue
            hardware = self._service_call(
                self.hardware, min(deadline, time.monotonic() + CONTROLLER_SERVICE_RESPONSE_TIMEOUT_S))
            if hardware is None:
                self.controller_failure = self.last_service_error or 'hardware interface query failed'
                rclpy.spin_once(self, timeout_sec=0.2)
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
                rclpy.spin_once(self, timeout_sec=0.2)
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
                rclpy.spin_once(self, timeout_sec=0.2)
                continue
            self.controller_failure = None
            return True
            rclpy.spin_once(self, timeout_sec=0.2)
        return False

    def _wait_lifecycle(self, deadline):
        states = {}
        while time.monotonic() < deadline:
            for name, client in self.lifecycle.items():
                response = self._service_call(client, min(deadline, time.monotonic() + 2.0))
                states[name] = response.current_state.id if response else None
            if all(state == 3 for state in states.values()):
                return states
            rclpy.spin_once(self, timeout_sec=0.2)
        return states

    def _lifecycle_snapshot(self, deadline):
        states = {}
        for name, client in self.lifecycle.items():
            response = self._service_call(client, min(deadline, time.monotonic() + 2.0))
            states[name] = response.current_state.id if response else None
        return states

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
                if health.get('ros_bridge') and health.get('ros'):
                    return True, None
                last_reason = (
                    f'backend_has_no_live_ros_bridge:ros_bridge={health.get("ros_bridge")},'
                    f'ros={health.get("ros")}'
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

        controllers_ok = self._wait_controllers(deadline)
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
        if self.mode == 'mapping':
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
        else:
            # The V30E simulation owns map->odom in navigation mode. Wait for
            # that live transform and the local odom chain before activating
            # Nav2, so its costmaps never enter their lifecycle transition
            # without the robot's required TF being available.
            if not self._stage(
                result, 'GLOBAL_TF_READY',
                lambda: tf_exists('map', 'odom') and tf_exists('map', 'base_link'),
                'TF map -> odom -> base_link', 'tf_unavailable:map->odom->base_link',
            ):
                return self._finish(result, 'global TF chain map->odom->base_link unavailable')
            self._report_stage(result, 'TF_READY', True,
                               success_detail=' frames=odom->base_footprint,map->base_link')
            nav_nodes_ok = True
            for name in NAV2_REQUIRED_NODES:
                node_ok = self._stage(
                    result, f'NODE_{name.upper()}_READY', lambda name=name: self._node_present(name),
                    f'Nav2 {name}', f'node_missing:/{name}',
                )
                nav_nodes_ok = nav_nodes_ok and node_ok
            if not nav_nodes_ok:
                return self._finish(result, 'one or more required Nav2 nodes are not running')
            lifecycle_states = self._lifecycle_snapshot(deadline)
            if not all(state == 3 for state in lifecycle_states.values()):
                # start_stack sets autostart=false while Gazebo is cold, then
                # this single readiness participant requests normal Nav2
                # lifecycle startup once all lower-layer prerequisites pass.
                # The production launcher leaves lifecycle nodes unconfigured
                # until this gate has verified Gazebo, controls, TF, and sensors.
                if all(state == 1 for state in lifecycle_states.values()):
                    self._record_timeline(
                        'T15_NAV2_LIFECYCLE_STARTUP_BEGIN',
                        detail='calling lifecycle_manager_navigation STARTUP',
                    )
                    response = self._service_call(
                        self.nav_lifecycle_manager,
                        deadline,
                        lambda request: setattr(request, 'command', ManageLifecycleNodes.Request.STARTUP),
                    )
                    if response is None or not response.success:
                        error = self.last_service_error or 'response.success=false'
                        self._report_stage(
                            result, 'NAV2_LIFECYCLE_READY', False,
                            f'lifecycle_manager_navigation_startup_failed:{error}',
                        )
                        return self._finish(result, f'Nav2 lifecycle manager startup failed: {error}')
                    result['stages']['NAV2_STARTUP_REQUESTED'] = True
                    print('NAV2_STARTUP_REQUESTED=PASS', flush=True)
                lifecycle_states = self._wait_lifecycle(deadline)
            lifecycle_ok = bool(lifecycle_states) and all(
                lifecycle_states.get(name) == 3 for name in NAV2_LIFECYCLE_NODES
            )
            result['stages']['NAV2_LIFECYCLE_READY'] = lifecycle_ok
            if lifecycle_ok:
                self._report_stage(
                    result, 'NAV2_LIFECYCLE_READY', True,
                    success_detail=' active=' + ','.join('/' + name for name in NAV2_LIFECYCLE_NODES),
                )
            else:
                inactive = ', '.join(
                    f'/{name}={lifecycle_states.get(name)}'
                    for name in NAV2_LIFECYCLE_NODES if lifecycle_states.get(name) != 3
                )
                self._report_stage(result, 'NAV2_LIFECYCLE_READY', False,
                                   f'inactive_lifecycle_nodes:{inactive}')
                return self._finish(result, f'Nav2 lifecycle not ACTIVE: {inactive}')
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
                return self._finish(result, 'map_server yaml_filename does not match requested map')
            if not self._stage(result, 'ACTION_SERVER_READY',
                               lambda: self.action.wait_for_server(timeout_sec=0.0),
                               '/navigate_to_pose action server',
                               'action_server_unavailable:/navigate_to_pose'):
                return self._finish(result, '/navigate_to_pose action server not ready')

        bridge_ok, bridge_reason = self._wait_ros_bridge(deadline)
        self._report_stage(
            result, 'ROS_BRIDGE_R01_READY', bridge_ok,
            bridge_reason,
            f' robot_id={self.robot_id} backend_health=connected' if bridge_ok else None,
        )
        if not bridge_ok:
            return self._finish(result, f'R01 ROS bridge is not ready: {bridge_reason}')
        result['startup_wall_time'] = time.monotonic() - started
        result['startup_sim_time'] = self.get_clock().now().nanoseconds * 1e-9
        result['nav_ready'] = True
        ready_label = 'Mapping stack READY' if self.mode == 'mapping' else 'Navigation stack READY'
        print(ready_label, flush=True)
        print('NAV_READY PASS', flush=True)
        self._print_startup_timeline(result)
        return result

    def _finish(self, result, reason):
        result['reason'] = reason
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
    parser.add_argument('--mode', choices=('mapping', 'navigation'), default='navigation')
    parser.add_argument('--map-file')
    parser.add_argument('--backend-url')
    parser.add_argument('--log-path')
    parser.add_argument('--json')
    args, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = Readiness(args.model, args.mode, args.map_file,
                     args.robot_id, args.backend_url, args.log_path)
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
