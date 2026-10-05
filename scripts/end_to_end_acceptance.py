#!/usr/bin/env python3
"""Exercise real ROS motion and the frontend's Django WebSocket commands."""
from __future__ import annotations

import argparse
import json
import math
import os
import time
import urllib.error
import urllib.request
import uuid

import rclpy
import websocket
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor, await_or_execute
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener
from mechanical_settling import MechanicalSettling


TRANSLATION_TOLERANCE_M = 0.05
ROTATION_TOLERANCE_RAD = 0.05
NAV_GOAL_XY_TOLERANCE_M = 0.05
NAV_GOAL_YAW_TOLERANCE_RAD = 0.05
MOTION_DURATION_SIM_S = 2.0
WEB_COMMAND_REFRESH_WALL_S = 0.10
EXPECTED_ROBOT_ENTITY = 'swerve_base'


class PublisherInfoExecutor(SingleThreadedExecutor):
    """Keep Humble's DDS MessageInfo for /cmd_vel publisher attribution."""

    def __init__(self, node, publisher_subscription):
        super().__init__()
        self.publisher_subscription = publisher_subscription
        self.add_node(node)

    def _take_subscription(self, sub):
        with sub.handle:
            return sub.handle.take_message(sub.msg_type, sub.raw)

    async def _execute_subscription(self, sub, message_info):
        if message_info is None:
            return
        message, info = message_info
        if sub is self.publisher_subscription:
            await await_or_execute(sub.callback, message, info)
        else:
            await await_or_execute(sub.callback, message)


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def pose_from_odom(msg: Odometry) -> tuple[float, float, float]:
    p = msg.pose.pose.position
    return p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation)


def pose_from_pose(pose) -> tuple[float, float, float]:
    p = pose.position
    return p.x, p.y, yaw_from_quaternion(pose.orientation)


def dist(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


class MotionProbe(Node):
    def __init__(self):
        super().__init__('simulation_e2e_acceptance', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.odom = None
        self.odom_velocity = None
        self.odom_sample_monotonic = None
        self.gazebo_velocity = None
        self.filtered_odom = None
        self.odom_count = 0
        self.gazebo_pose = None
        self.gazebo_pose_count = 0
        self.gazebo_pose_sample_monotonic = None
        self.joint_state = None
        self.wheel_radius = None
        self.joint_events: list[tuple[float, dict, dict]] = []
        self.map = None
        self.cmd_events: list[tuple[float, float, float, float]] = []
        self.manual_cmd_events: list[tuple[float, float, float, float]] = []
        self.nav_cmd_events: list[tuple[float, float, float, float]] = []
        self.selected_cmd_events: list[tuple[float, float, float, float]] = []
        self.command_owner_events: list[tuple[float, str]] = []
        self.cmd_publisher_gids: list[str | None] = []
        self.steering_events: list[tuple[float, ...]] = []
        self.drive_events: list[tuple[float, ...]] = []
        self.action_status_events: list[tuple[float, dict[bytes, int]]] = []
        self.ws_messages: list[dict] = []
        self.ws_errors: list[str] = []
        self.nav_statuses: list[dict] = []
        self.control_statuses: list[dict] = []
        self.runtime_status = None
        self.command = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_command = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos)
        self.create_subscription(Odometry, '/odometry/filtered', self._filtered_odom_cb, qos)
        self.create_subscription(ModelStates, '/model_states', self._model_states_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 20)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.cmd_subscription = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_cb, 20)
        self.create_subscription(
            Twist, '/cmd_vel_manual',
            lambda msg: self._record_command(self.manual_cmd_events, msg), 20)
        self.create_subscription(
            Twist, '/cmd_vel_nav',
            lambda msg: self._record_command(self.nav_cmd_events, msg), 20)
        self.create_subscription(
            Twist, '/cmd_vel_selected',
            lambda msg: self._record_command(self.selected_cmd_events, msg), 20)
        self.create_subscription(
            String, '/command_owner',
            lambda msg: self.command_owner_events.append((time.monotonic(), str(msg.data))), 20)
        self.create_subscription(Float64MultiArray, '/steering_controller/commands',
                                 lambda msg: self.steering_events.append(
                                     (time.monotonic(), *tuple(msg.data))), 20)
        self.create_subscription(Float64MultiArray, '/drive_controller/commands',
                                 lambda msg: self.drive_events.append(
                                     (time.monotonic(), *tuple(msg.data))), 20)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self._action_status_cb, 10)
        self.navigation = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.motion_executor = PublisherInfoExecutor(self, self.cmd_subscription)

    def _odom_cb(self, msg):
        p = pose_from_odom(msg)
        if all(math.isfinite(value) for value in p):
            self.odom = p
            self.odom_velocity = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)
            self.odom_sample_monotonic = time.monotonic()
            self.odom_count += 1

    def _filtered_odom_cb(self, msg):
        p = pose_from_odom(msg)
        if all(math.isfinite(value) for value in p):
            self.filtered_odom = p

    def _model_states_cb(self, msg):
        try:
            index = msg.name.index(EXPECTED_ROBOT_ENTITY)
        except ValueError:
            return
        if index >= len(msg.pose):
            return
        pose = pose_from_pose(msg.pose[index])
        if all(math.isfinite(value) for value in pose):
            self.gazebo_pose = pose
            body_twist = msg.twist[index]
            self.gazebo_velocity = (body_twist.linear.x, body_twist.linear.y, body_twist.angular.z)
            self.gazebo_pose_count += 1
            self.gazebo_pose_sample_monotonic = time.monotonic()

    def _joint_state_cb(self, msg):
        positions = {name: float(msg.position[index]) for index, name in enumerate(msg.name)
                     if index < len(msg.position) and math.isfinite(msg.position[index])}
        velocities = {name: float(msg.velocity[index]) for index, name in enumerate(msg.name)
                      if index < len(msg.velocity) and math.isfinite(msg.velocity[index])}
        if not positions and not velocities:
            return
        sample_time = time.monotonic()
        self.joint_state = {'monotonic_s': sample_time,
                            'positions': positions, 'velocities': velocities}
        self.joint_events.append((sample_time, positions, velocities))
        if len(self.joint_events) > 5000:
            del self.joint_events[:-2500]

    def _map_cb(self, msg):
        if (msg.info.width > 0 and msg.info.height > 0
                and len(msg.data) == msg.info.width * msg.info.height):
            self.map = msg

    def _cmd_cb(self, msg, info=None):
        self._record_command(self.cmd_events, msg)
        gid = getattr(info, 'publisher_gid', None)
        self.cmd_publisher_gids.append(bytes(gid).hex() if gid is not None else None)

    @staticmethod
    def _record_command(events, msg):
        events.append((time.monotonic(), msg.linear.x, msg.linear.y, msg.angular.z))

    def _action_status_cb(self, msg):
        self.action_status_events.append((
            time.monotonic(), {bytes(item.goal_info.goal_id.uuid): int(item.status)
                               for item in msg.status_list},
        ))

    def pump(self, ws=None, timeout=0.02):
        self.motion_executor.spin_once(timeout_sec=timeout)
        if ws is not None:
            try:
                raw = ws.recv()
            except websocket.WebSocketTimeoutException:
                return
            except websocket.WebSocketConnectionClosedException as exc:
                self.ws_errors.append(f'websocket_closed:{exc}')
                return
            if not raw:
                return
            try:
                message = json.loads(raw)
            except (TypeError, json.JSONDecodeError):
                self.ws_errors.append('invalid_websocket_json')
                return
            if not isinstance(message, dict):
                return
            self.ws_messages.append(message)
            if message.get('type') == 'RUNTIME_STATUS':
                self.runtime_status = message
            elif message.get('type') == 'NAV_STATUS':
                self.nav_statuses.append(message)
            elif message.get('type') == 'ROBOT_CONTROL_STATUS':
                self.control_statuses.append(message)
            elif message.get('type') == 'ERROR':
                self.ws_errors.append(
                    f'{message.get("code", "ERROR")}:{message.get("message", "")}'
                )

    def wait_until(self, predicate, timeout, ws=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump(ws, min(0.05, max(0.001, deadline - time.monotonic())))
            if predicate():
                return True
        return bool(predicate())

    def sim_time(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_mechanical_settling(self, ws=None, timeout=45.0,
                                body_tolerance=0.02, wheel_position_rate=0.005,
                                steering_tolerance=0.03, stable_window=0.8,
                                wall_timeout=180.0, clock_stall_timeout=5.0,
                                body_position_rate=0.001):
        """Timeout/window are simulation seconds; wall time is only a watchdog.

        Wheel position range, not a possibly inconsistent instantaneous velocity,
        proves actual rotation. Body pose drift is checked in addition to twists.
        """
        if hasattr(self, 'wheel_radius') and self.wheel_radius is None:
            client = self.create_client(GetParameters, '/swerve_controller/get_parameters')
            try:
                if not client.wait_for_service(timeout_sec=3.0):
                    return {'passed': False, 'reason': 'SETTLING_WHEEL_GEOMETRY_UNAVAILABLE'}
                request = GetParameters.Request()
                request.names = ['wheel_radius']
                future = client.call_async(request)
                if not self.wait_until(future.done, 5.0, ws):
                    return {'passed': False, 'reason': 'SETTLING_WHEEL_GEOMETRY_TIMEOUT'}
                radius = future.result().values[0].double_value
                if not math.isfinite(radius) or radius <= 0:
                    return {'passed': False, 'reason': 'SETTLING_WHEEL_GEOMETRY_INVALID'}
                self.wheel_radius = radius
            finally:
                self.destroy_client(client)
        monitor = MechanicalSettling(self.sim_time(), time.monotonic(),
            timeout_sim=timeout, timeout_wall=wall_timeout,
            clock_stall_wall=clock_stall_timeout, window_sim=stable_window,
            body_tolerance=body_tolerance, wheel_position_rate=wheel_position_rate,
            body_position_rate=body_position_rate, steering_span=steering_tolerance,
            wheel_radius=getattr(self, 'wheel_radius', None))
        while True:
            self.pump(ws, 0.005)
            sample = None
            if (self.joint_state and self.selected_cmd_events and self.drive_events
                    and self.steering_events and self.odom_sample_monotonic is not None
                    and self.gazebo_pose_sample_monotonic is not None):
                joints = self.joint_state
                sample = {
                    'selected': list(self.selected_cmd_events[-1][1:]),
                    'drive': list(self.drive_events[-1][1:]),
                    'steering_targets': list(self.steering_events[-1][1:]),
                    'wheel_positions': [joints['positions'].get(name, math.nan)
                        for name in ('wheel_front_drive_joint', 'wheel_rear_drive_joint')],
                    'wheel_velocities': [joints['velocities'].get(name, math.nan)
                        for name in ('wheel_front_drive_joint', 'wheel_rear_drive_joint')],
                    'steering_positions': [joints['positions'].get(name, math.nan)
                        for name in ('steer_front_joint', 'steer_rear_joint')],
                    'body_velocity': self.gazebo_velocity, 'odom_velocity': self.odom_velocity,
                    'body_pose': self.gazebo_pose,
                    'source_wall_times': [joints['monotonic_s'], self.selected_cmd_events[-1][0],
                        self.drive_events[-1][0], self.steering_events[-1][0],
                        self.odom_sample_monotonic, self.gazebo_pose_sample_monotonic],
                }
            result = monitor.update(self.sim_time(), time.monotonic(), sample)
            if result is not None:
                return {**result, 'duration_s': result['settling_wall_seconds']}

    def map_pose(self):
        try:
            transform = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
            return (transform.translation.x, transform.translation.y,
                    yaw_from_quaternion(transform.rotation))
        except (TransformException, RuntimeError):
            return None

    @staticmethod
    def joint_snapshot(sample):
        if sample is None:
            return None
        return {'positions': dict(sample['positions']),
                'velocities': dict(sample['velocities'])}

    @staticmethod
    def body_displacement(pose0, pose1):
        if pose0 is None or pose1 is None:
            return None
        dx, dy = pose1[0] - pose0[0], pose1[1] - pose0[1]
        return {
            'dx': dx, 'dy': dy,
            'forward_m': dx * math.cos(pose0[2]) + dy * math.sin(pose0[2]),
            'lateral_m': -dx * math.sin(pose0[2]) + dy * math.cos(pose0[2]),
            'dyaw_rad': wrap_angle(pose1[2] - pose0[2]),
        }

    def observe_idle_cmd_vel(self, duration_wall_s=1.0):
        start_time = time.monotonic()
        start_index = len(self.cmd_events)
        selected_index = len(self.selected_cmd_events)
        owner_index = len(self.command_owner_events)
        deadline = start_time + duration_wall_s
        while time.monotonic() < deadline:
            self.pump(timeout=min(0.02, deadline - time.monotonic()))
        events = self.cmd_events[start_index:]
        nonzero = [row for row in events if any(abs(value) > 1e-4 for value in row[1:])]
        selected_events = self.selected_cmd_events[selected_index:]
        selected_nonzero = [row for row in selected_events
                            if any(abs(value) > 1e-4 for value in row[1:])]
        observed_owners = self.command_owner_events[owner_index:]
        selected_owner = (observed_owners[-1][1] if observed_owners else
                          self.command_owner_events[-1][1] if self.command_owner_events else None)
        publisher_names = {}
        for endpoint in self.get_publishers_info_by_topic('/cmd_vel'):
            gid = bytes(endpoint.endpoint_gid).hex()
            namespace = endpoint.node_namespace.rstrip('/')
            publisher_names[gid] = (f'{namespace}/{endpoint.node_name}'
                                    if namespace else f'/{endpoint.node_name}')
        publisher_gids = self.cmd_publisher_gids[start_index:start_index + len(events)]
        latest_action_statuses = (self.action_status_events[-1][1]
                                  if self.action_status_events else {})
        return {
            'duration_wall_s': time.monotonic() - start_time,
            'samples': len(events), 'nonzero_samples': len(nonzero),
            'selected_samples': len(selected_events),
            'selected_nonzero_samples': len(selected_nonzero),
            'selected_velocity_zero': bool(selected_events) and not selected_nonzero,
            'selected_owner': selected_owner,
            'selected_owner_none_seen': any(owner == 'NONE' for _, owner in observed_owners),
            'events': [{'elapsed_wall_s': row[0] - start_time,
                        'linear_x': row[1], 'linear_y': row[2], 'angular_z': row[3],
                        'publisher_gid': publisher_gids[index],
                        'publisher_node': publisher_names.get(publisher_gids[index])}
                       for index, row in enumerate(events)],
            'publisher_endpoints': publisher_names,
            'navigate_to_pose_statuses': [
                {'goal_id': goal_id.hex(), 'status': status}
                for goal_id, status in latest_action_statuses.items()],
            'unexpected_idle_traffic': bool(nonzero) or len(events) > 2,
        }

    def stop_direct(self, timeout=1.5, *, nav_source=False):
        start_time = time.monotonic()
        start_index = len(self.drive_events)
        stop_until = start_time + timeout
        last_publish = 0.0
        zero_seen = False
        while time.monotonic() < stop_until:
            now = time.monotonic()
            if now - last_publish >= 0.10:
                (self.nav_command if nav_source else self.command).publish(Twist())
                last_publish = now
            self.pump(timeout=0.02)
            zero_seen = any(all(abs(value) <= 1e-4 for value in row[1:])
                            for row in self.drive_events[start_index:])
            if zero_seen and now - start_time >= 0.20:
                break
        return {'zero_drive_command_seen': zero_seen,
                'wall_time_s': time.monotonic() - start_time}

    def direct_motion(self, label, velocity, duration=MOTION_DURATION_SIM_S,
                      timeout=45.0):
        sensors_ready = self.wait_until(
            lambda: self.odom is not None and self.gazebo_pose is not None
            and self.joint_state is not None, min(timeout, 15.0))
        if not sensors_ready:
            return {'passed': False,
                    'reason': 'missing_/odom_/model_states_or_/joint_states_before_motion'}
        pose0, gazebo0 = self.odom, self.gazebo_pose
        joint0 = self.joint_snapshot(self.joint_state)
        start_sim = self.sim_time()
        start_wall = time.monotonic()
        command_index = len(self.cmd_events)
        selected_index = len(self.selected_cmd_events)
        owner_index = len(self.command_owner_events)
        steering_index = len(self.steering_events)
        drive_index = len(self.drive_events)
        joint_index = len(self.joint_events)
        last_publish = 0.0
        first_command_sent = None
        command_publish_count = 0
        command_send_times = []
        last_command_publish_sim = None
        wall_deadline = start_wall + timeout
        while self.sim_time() - start_sim < duration and time.monotonic() < wall_deadline:
            now = time.monotonic()
            if now - last_publish >= 0.10:
                msg = Twist()
                msg.linear.x, msg.linear.y, msg.angular.z = velocity
                self.command.publish(msg)
                last_publish = now
                command_publish_count += 1
                command_send_times.append(now)
                last_command_publish_sim = self.sim_time()
                if first_command_sent is None:
                    first_command_sent = now
            self.pump(timeout=0.02)
        elapsed_sim = self.sim_time() - start_sim
        watchdog_result = {'tested': False}
        if label == 'FORWARD' and last_command_publish_sim is not None:
            watchdog_start_wall = time.monotonic()
            watchdog_selected_index = len(self.selected_cmd_events)
            watchdog_cmd_index = len(self.cmd_events)
            watchdog_zero_seen = False
            # The arbiter lease is measured in wall time, while the controller
            # update loop uses Gazebo time.  Observe the selected command after
            # the lease expires; waiting for a zero drive-controller sample
            # here makes this gate depend on the VM's simulation RTF.
            watchdog_lease_s = 0.5
            watchdog_expired_at = command_send_times[-1] + watchdog_lease_s + 0.10
            while time.monotonic() - watchdog_start_wall < 2.5:
                self.pump(timeout=0.02)
            selected_after_lease = [
                row for row in self.selected_cmd_events[watchdog_selected_index:]
                if row[0] >= watchdog_expired_at
            ]
            watchdog_zero_seen = bool(selected_after_lease) and all(
                all(abs(value) <= 1e-4 for value in row[1:])
                for row in selected_after_lease)
            watchdog_result = {
                'tested': True,
                'command_timeout_config_s': 0.5,
                'zero_selected_command_seen': watchdog_zero_seen,
                'unexpected_cmd_vel_samples_after_publisher_stopped':
                    len(self.cmd_events[watchdog_cmd_index:]),
                'delay_after_last_command_publish_wall_s':
                    max(0.0, time.monotonic() - command_send_times[-1])
                    if watchdog_zero_seen else None,
                'lease_timeout_wall_s': watchdog_lease_s,
                'waited_wall_s': time.monotonic() - watchdog_start_wall,
                'wait_wall_s': time.monotonic() - watchdog_start_wall,
            }
        stop_result = self.stop_direct()
        pose1, gazebo1 = self.odom, self.gazebo_pose
        joint1 = self.joint_snapshot(self.joint_state)
        recent_cmd = self.cmd_events[command_index:]
        active_cmd = [row for row in recent_cmd
                      if any(abs(value) > 1e-4 for value in row[1:])]
        selected_cmd = self.selected_cmd_events[selected_index:]
        active_selected_cmd = [row for row in selected_cmd
                               if any(abs(value) > 1e-4 for value in row[1:])]
        owners = self.command_owner_events[owner_index:]
        direct_owner_seen = any(owner == 'DIRECT_MANUAL' for _, owner in owners)
        drive = self.drive_events[drive_index:]
        steering = self.steering_events[steering_index:]
        joint_samples = self.joint_events[joint_index:]
        drive_active = any(any(abs(value) > 1e-4 for value in row[1:]) for row in drive)
        steering_active = any(any(abs(value) > 1e-4 for value in row[1:]) for row in steering)
        gazebo_delta = self.body_displacement(gazebo0, gazebo1)
        odom_delta = self.body_displacement(pose0, pose1)
        is_rotation = abs(velocity[2]) > 1e-9
        threshold = ROTATION_TOLERANCE_RAD if is_rotation else TRANSLATION_TOLERANCE_M
        if gazebo_delta is None or odom_delta is None:
            physical_metric = odom_metric = None
        elif is_rotation:
            physical_metric = gazebo_delta['dyaw_rad']
            odom_metric = odom_delta['dyaw_rad']
        else:
            direction = 'forward_m' if label == 'FORWARD' else 'lateral_m'
            physical_metric = gazebo_delta[direction]
            odom_metric = odom_delta[direction]
        wheel_velocity_max = max((abs(value) for _, _, velocities in joint_samples
                                  for name, value in velocities.items()
                                  if name.endswith('_drive_joint')), default=0.0)
        wheel_position_change = 0.0
        if joint0 is not None and joint1 is not None:
            for name in ('wheel_front_drive_joint', 'wheel_rear_drive_joint'):
                if name in joint0['positions'] and name in joint1['positions']:
                    wheel_position_change = max(
                        wheel_position_change,
                        abs(joint1['positions'][name] - joint0['positions'][name]))
        steering_position_change = 0.0
        if joint0 is not None and joint1 is not None:
            for name in ('steer_front_joint', 'steer_rear_joint'):
                if name in joint0['positions'] and name in joint1['positions']:
                    steering_position_change = max(
                        steering_position_change,
                        abs(wrap_angle(joint1['positions'][name] - joint0['positions'][name])))
        first_cmd_observed = next((row[0] for row in active_cmd), None)
        first_drive_observed = next((row[0] for row in drive
                                     if any(abs(value) > 1e-4 for value in row[1:])), None)
        first_steering_observed = next((row[0] for row in steering), None)
        direction_passed = (physical_metric is not None and physical_metric > threshold
                            and odom_metric is not None and odom_metric > threshold * 0.5)
        steering_required = label in ('STRAFE', 'ROTATE')
        steering_response = (steering_active and steering_position_change > 0.10
                             if steering_required else True)
        passed = bool(
            elapsed_sim > 0.0 and direction_passed and active_cmd and active_selected_cmd
            and direct_owner_seen and drive_active
            and (wheel_velocity_max > 0.05 or wheel_position_change > 0.05)
            and steering_response and stop_result['zero_drive_command_seen']
            and (label != 'FORWARD' or watchdog_result.get('zero_selected_command_seen')))
        max_command_refresh_gap = max(
            (later - earlier for earlier, later in zip(command_send_times, command_send_times[1:])),
            default=None)
        return {
            'passed': passed,
            'reason': None if passed else
                'missing_cmd_vel_controller_drive_joint_response_or_physical_pose_delta',
            'command': {'linear_x': velocity[0], 'linear_y': velocity[1],
                        'angular_z': velocity[2], 'publish_count': command_publish_count,
                        'first_sent_monotonic_s': first_command_sent,
                        'max_refresh_gap_s': max_command_refresh_gap},
            'first_cmd_vel_observed_after_s': (first_cmd_observed - start_wall
                                               if first_cmd_observed else None),
            'first_steering_command_after_s': (first_steering_observed - start_wall
                                               if first_steering_observed else None),
            'first_drive_command_after_s': (first_drive_observed - start_wall
                                            if first_drive_observed else None),
            'p0': pose0, 'p1': pose1,
            'gazebo_p0': gazebo0, 'gazebo_p1': gazebo1,
            'gazebo_body_delta': gazebo_delta, 'odom_body_delta': odom_delta,
            'directional_displacement_m_or_yaw_rad': physical_metric,
            'tolerance': threshold, 'sim_duration_s': elapsed_sim,
            'cmd_vel_nonzero_samples': len(active_cmd),
            'selected_cmd_vel_nonzero_samples': len(active_selected_cmd),
            'command_owner': 'DIRECT_MANUAL' if direct_owner_seen else None,
            'steering_command_samples': len(steering),
            'steering_nonzero': steering_active,
            'drive_command_samples': len(drive), 'drive_nonzero': drive_active,
            'wheel_velocity_max_rad_s': wheel_velocity_max,
            'wheel_position_change_rad': wheel_position_change,
            'steering_position_change_rad': steering_position_change,
            'joint_state_samples': len(joint_samples),
            'joint_states_before': joint0, 'joint_states_after': joint1,
            'stop_result': stop_result,
            'watchdog_result': watchdog_result,
        }

    def map_goal_candidate(self):
        if self.map is None:
            return None, 'map_message_unavailable'
        pose = self.map_pose()
        if pose is None:
            return None, 'TF_map_to_base_footprint_unavailable'
        grid = self.map
        origin = grid.info.origin
        origin_yaw = yaw_from_quaternion(origin.orientation)
        cos_yaw, sin_yaw = math.cos(origin_yaw), math.sin(origin_yaw)
        resolution = float(grid.info.resolution)

        def cell(x, y):
            dx, dy = x - origin.position.x, y - origin.position.y
            local_x = cos_yaw * dx + sin_yaw * dy
            local_y = -sin_yaw * dx + cos_yaw * dy
            gx, gy = math.floor(local_x / resolution), math.floor(local_y / resolution)
            if not (0 <= gx < grid.info.width and 0 <= gy < grid.info.height):
                return None
            return int(grid.data[gy * grid.info.width + gx])

        def is_clear(x, y):
            occupancy = cell(x, y)
            return occupancy is not None and 0 <= occupancy <= 20

        offsets = (0.0, math.pi / 2.0, -math.pi / 2.0, math.pi,
                   math.pi / 4.0, -math.pi / 4.0,
                   3.0 * math.pi / 4.0, -3.0 * math.pi / 4.0)
        for radius in (0.55, 0.75, 0.95, 1.2, 1.45):
            for offset in offsets:
                angle = pose[2] + offset
                target = (pose[0] + radius * math.cos(angle),
                          pose[1] + radius * math.sin(angle))
                steps = max(2, math.ceil(radius / 0.05))
                path_clear = True
                for step in range(1, steps + 1):
                    fraction = step / steps
                    x = pose[0] + (target[0] - pose[0]) * fraction
                    y = pose[1] + (target[1] - pose[1]) * fraction
                    if not is_clear(x, y):
                        path_clear = False
                        break
                if not path_clear:
                    continue
                clearance = True
                for test_radius in (0.15, 0.25, 0.32):
                    for index in range(8):
                        test_angle = index * math.pi / 4.0
                        if not is_clear(target[0] + test_radius * math.cos(test_angle),
                                        target[1] + test_radius * math.sin(test_angle)):
                            clearance = False
                            break
                    if not clearance:
                        break
                if clearance:
                    return (target[0], target[1], pose[2]), None
        return None, 'no_free_straight_path_with_clearance_within_1.45m'

    def direct_navigation(self, timeout):
        if not self.navigation.wait_for_server(timeout_sec=min(timeout, 5.0)):
            return {'passed': False, 'server_ready': False,
                    'reason': 'action_server_unavailable:/navigate_to_pose'}
        if not self.wait_until(lambda: self.map is not None and self.map_pose() is not None,
                               min(timeout, 15.0)):
            return {'passed': False, 'server_ready': True,
                    'reason': 'map_or_map_to_base_footprint_unavailable'}
        goal_pose, goal_error = self.map_goal_candidate()
        if goal_pose is None:
            return {'passed': False, 'server_ready': True, 'reason': goal_error}

        x, y, yaw = goal_pose
        pose0, gazebo0 = self.map_pose(), self.gazebo_pose
        if pose0 is None or gazebo0 is None:
            return {'passed': False, 'server_ready': True,
                    'reason': 'map_or_gazebo_pose_missing_before_goal', 'goal': goal_pose}
        command_index = len(self.nav_cmd_events)
        selected_index = len(self.selected_cmd_events)
        owner_index = len(self.command_owner_events)
        drive_index = len(self.drive_events)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        deadline = time.monotonic() + timeout
        goal_future = self.navigation.send_goal_async(goal)
        while time.monotonic() < deadline and not goal_future.done():
            self.pump(timeout=0.03)
        if not goal_future.done():
            self.stop_direct(nav_source=True)
            return {'passed': False, 'server_ready': True,
                    'reason': 'goal_acceptance_timeout:/navigate_to_pose',
                    'goal': goal_pose}
        goal_handle = goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.stop_direct(nav_source=True)
            return {'passed': False, 'server_ready': True,
                    'reason': 'goal_rejected:/navigate_to_pose', 'goal': goal_pose,
                    'accepted': False}

        result_future = goal_handle.get_result_async()
        while time.monotonic() < deadline and not result_future.done():
            self.pump(timeout=0.03)
        if not result_future.done():
            goal_handle.cancel_goal_async()
            self.wait_until(lambda: result_future.done(), 3.0)
            terminal_status = 'TIMEOUT'
            exact_reason = f'no terminal action result within {timeout:.1f}s'
        else:
            action_result = result_future.result()
            status_names = {
                GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                GoalStatus.STATUS_ABORTED: 'ABORTED',
                GoalStatus.STATUS_CANCELED: 'CANCELED',
            }
            terminal_status = status_names.get(int(action_result.status),
                                               f'STATUS_{int(action_result.status)}')
            exact_reason = None if terminal_status == 'SUCCEEDED' else (
                f'/navigate_to_pose terminal action status={terminal_status} '
                '(NavigateToPose result has no error detail field)'
            )

        stop_result = self.stop_direct(nav_source=True)
        pose1, gazebo1 = self.map_pose(), self.gazebo_pose
        commands = self.nav_cmd_events[command_index:]
        active_commands = [row for row in commands
                           if any(abs(value) > 1e-4 for value in row[1:])]
        selected = self.selected_cmd_events[selected_index:]
        active_selected = [row for row in selected
                           if any(abs(value) > 1e-4 for value in row[1:])]
        owners = self.command_owner_events[owner_index:]
        nav_owner_seen = any(owner == 'NAV2' for _, owner in owners)
        drives = self.drive_events[drive_index:]
        drive_active = any(any(abs(value) > 1e-4 for value in row[1:]) for row in drives)
        pose_change = dist(pose0, pose1) if pose1 is not None else None
        physical_change = dist(gazebo0, gazebo1) if gazebo1 is not None else None
        final_goal_error = dist(gazebo1, goal_pose) if gazebo1 is not None else None
        final_yaw_error = (abs(wrap_angle(gazebo1[2] - goal_pose[2]))
                           if gazebo1 is not None else None)
        passed = bool(
            terminal_status == 'SUCCEEDED' and active_commands and active_selected
            and nav_owner_seen and drive_active
            and pose_change is not None and pose_change > TRANSLATION_TOLERANCE_M
            and physical_change is not None and physical_change > TRANSLATION_TOLERANCE_M
            and final_goal_error is not None
            and final_goal_error <= NAV_GOAL_XY_TOLERANCE_M
            and final_yaw_error is not None
            and final_yaw_error <= NAV_GOAL_YAW_TOLERANCE_RAD
            and stop_result['zero_drive_command_seen'])
        return {
            'passed': passed, 'server_ready': True, 'accepted': True, 'status': terminal_status,
            'reason': None if passed else (exact_reason or
                'goal_succeeded_without_observed_cmd_vel_controller_command_or_pose_change'),
            'goal': goal_pose, 'p0': pose0, 'p1': pose1,
            'gazebo_p0': gazebo0, 'gazebo_p1': gazebo1,
            'displacement_m': pose_change, 'gazebo_displacement_m': physical_change,
            'final_goal_error_m': final_goal_error,
            'final_goal_yaw_error_rad': final_yaw_error,
            'goal_xy_tolerance_m': NAV_GOAL_XY_TOLERANCE_M,
            'goal_yaw_tolerance_rad': NAV_GOAL_YAW_TOLERANCE_RAD,
            'stop_result': stop_result,
            'cmd_vel_nonzero_samples': len(active_commands),
            'selected_cmd_vel_nonzero_samples': len(active_selected),
            'command_owner': 'NAV2' if nav_owner_seen else None,
            'drive_command_samples': len(drives), 'drive_nonzero': drive_active,
        }


def http_json(url: str, body=None, timeout=8.0):
    data = json.dumps(body).encode('utf-8') if body is not None else None
    headers = {'Content-Type': 'application/json'} if data is not None else {}
    request = urllib.request.Request(url, data=data, headers=headers,
                                     method='POST' if data is not None else 'GET')
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.loads(response.read().decode('utf-8'))
    except (OSError, ValueError, urllib.error.URLError) as exc:
        raise RuntimeError(f'{url} failed: {type(exc).__name__}: {exc}') from exc


def find_goal_status(probe, x, y, robot_id):
    relevant = [item for item in probe.nav_statuses
                if item.get('robot_id') == robot_id
                and abs(float(item.get('x', x)) - x) <= 1e-4
                and abs(float(item.get('y', y)) - y) <= 1e-4]
    return relevant[-1] if relevant else None


def set_mode(probe, ws, robot_id, mode, timeout=8.0):
    initial = len(probe.control_statuses)
    ws.send(json.dumps({'type': 'ROBOT_MODE', 'robot_id': robot_id, 'mode': mode}))
    passed = probe.wait_until(
        lambda: any(item.get('robot_id') == robot_id and item.get('mode') == mode
                    and item.get('accepted') is True
                    and item.get('mode_transition_state') == 'APPLIED'
                    and item.get('applied_mode') == mode
                    for item in probe.control_statuses[initial:]),
        timeout, ws,
    )
    reason = None
    if not passed:
        failures = [item for item in probe.control_statuses[initial:]
                    if item.get('robot_id') == robot_id and item.get('accepted') is False]
        reason = str(failures[-1].get('reason') if failures else 'no_accepted_ROBOT_CONTROL_STATUS')
    return passed, reason


def web_manual(probe, ws, robot_id, action, duration_sim_s=0.8, timeout=45.0):
    from manual_refresh import ManualRefreshWorker
    settling = probe.wait_mechanical_settling(ws, timeout=timeout)
    if not settling['passed']:
        return {'passed': False, 'reason': settling['reason'], 'settling': settling}
    sensors_ready = probe.wait_until(
        lambda: probe.odom is not None and probe.gazebo_pose is not None
        and probe.joint_state is not None, 10.0, ws)
    if not sensors_ready:
        return {'passed': False,
                'reason': 'missing_/odom_/model_states_or_/joint_states_before_manual_command'}
    pose0, gazebo0 = probe.odom, probe.gazebo_pose
    joint0 = probe.joint_snapshot(probe.joint_state)
    sim_start = probe.sim_time()
    manual_start_wall = time.monotonic()
    wall_deadline = time.monotonic() + timeout
    send_times = []
    command_index = len(probe.manual_cmd_events)
    selected_index = len(probe.selected_cmd_events)
    owner_index = len(probe.command_owner_events)
    drive_index = len(probe.drive_events)
    steering_index = len(probe.steering_events)
    joint_index = len(probe.joint_events)
    control_index = len(probe.control_statuses)
    sender = ManualRefreshWorker(lambda payload: ws.send(json.dumps(payload)), robot_id,
                                 WEB_COMMAND_REFRESH_WALL_S).start()
    try:
        sender.hold(action)
        while (probe.sim_time() - sim_start < duration_sim_s
               and time.monotonic() < wall_deadline):
            if sender.error:
                raise RuntimeError('manual refresh transport failed') from sender.error
            probe.pump(ws, 0.02)
        stop_time = time.monotonic()
    finally:
        sender.close()
    send_times = [row['T0'] for row in sender.events if row['action'] == action]
    zero_seen = probe.wait_until(
        lambda: any(row[0] >= stop_time and all(abs(value) <= 1e-4 for value in row[1:])
                    for row in probe.manual_cmd_events[command_index:]),
        min(5.0, max(1.0, timeout / 4.0)), ws,
    )
    stop_settling = probe.wait_mechanical_settling(ws, timeout=timeout)
    pose1 = probe.odom
    gazebo1 = probe.gazebo_pose
    joint1 = probe.joint_snapshot(probe.joint_state)
    commands = probe.manual_cmd_events[command_index:]
    active_commands = [row for row in commands if row[0] < stop_time
                       and any(abs(value) > 1e-4 for value in row[1:])]
    selected_commands = probe.selected_cmd_events[selected_index:]
    active_selected_commands = [row for row in selected_commands
                                if any(abs(value) > 1e-4 for value in row[1:])]
    owners = probe.command_owner_events[owner_index:]
    manual_owner_seen = any(owner == 'WEB_MANUAL' for _, owner in owners)
    drives = probe.drive_events[drive_index:]
    steering = probe.steering_events[steering_index:]
    joint_samples = probe.joint_events[joint_index:]
    drive_active = any(row[0] < stop_time and any(abs(value) > 1e-4 for value in row[1:])
                       for row in drives)
    gazebo_delta = probe.body_displacement(gazebo0, gazebo1)
    odom_delta = probe.body_displacement(pose0, pose1)
    pose_change = dist(pose0, pose1) if pose1 is not None else None
    if action in ('FORWARD', 'BACKWARD'):
        physical_metric = gazebo_delta['forward_m'] if gazebo_delta else None
        odom_metric = odom_delta['forward_m'] if odom_delta else None
        if action == 'BACKWARD':
            physical_metric = -physical_metric if physical_metric is not None else None
            odom_metric = -odom_metric if odom_metric is not None else None
        expected_component = 'linear_x'
        expected_sign = 1 if action == 'FORWARD' else -1
    elif action in ('LEFT', 'RIGHT'):
        physical_metric = gazebo_delta['lateral_m'] if gazebo_delta else None
        odom_metric = odom_delta['lateral_m'] if odom_delta else None
        if action == 'RIGHT':
            physical_metric = -physical_metric if physical_metric is not None else None
            odom_metric = -odom_metric if odom_metric is not None else None
        expected_component = 'linear_y'
        expected_sign = 1 if action == 'LEFT' else -1
    else:
        physical_metric = gazebo_delta['dyaw_rad'] if gazebo_delta else None
        odom_metric = odom_delta['dyaw_rad'] if odom_delta else None
        if action == 'ROTATE_RIGHT':
            physical_metric = -physical_metric if physical_metric is not None else None
            odom_metric = -odom_metric if odom_metric is not None else None
        expected_component = 'angular_z'
        expected_sign = 1 if action == 'ROTATE_LEFT' else -1
    wheel_velocity_max = max((abs(value) for _, _, velocities in joint_samples
                              for name, value in velocities.items()
                              if name.endswith('_drive_joint')), default=0.0)
    wheel_position_change = 0.0
    if joint0 is not None and joint1 is not None:
        for name in ('wheel_front_drive_joint', 'wheel_rear_drive_joint'):
            if name in joint0['positions'] and name in joint1['positions']:
                wheel_position_change = max(
                    wheel_position_change,
                    abs(joint1['positions'][name] - joint0['positions'][name]))
    action_statuses = probe.control_statuses[control_index:]
    command_ack = any(item.get('robot_id') == robot_id and item.get('accepted') is True
                      for item in action_statuses)
    component_index = {'linear_x': 1, 'linear_y': 2, 'angular_z': 3}[expected_component]
    command_direction_seen = any(
        row[component_index] * expected_sign > 0.01
        and all(abs(row[index]) <= 1e-3 for index in range(1, 4) if index != component_index)
        for row in active_selected_commands)
    direction_tolerance = (ROTATION_TOLERANCE_RAD if expected_component == 'angular_z'
                           else TRANSLATION_TOLERANCE_M)
    passed = bool(command_ack and command_direction_seen and active_selected_commands
                  and manual_owner_seen and drive_active
                  and (wheel_velocity_max > 0.05 or wheel_position_change > 0.05)
                  and physical_metric is not None
                  and physical_metric > direction_tolerance
                  and odom_metric is not None and odom_metric > direction_tolerance * 0.5
                  and zero_seen and stop_settling['passed'])
    max_refresh_gap = max((later - earlier for earlier, later in zip(send_times, send_times[1:])),
                          default=None)
    connected_ids = ((probe.runtime_status or {}).get('connected_robot_ids') or [])
    runtime_status = probe.runtime_status or {}
    robot_map_sync = (runtime_status.get('robot_map_sync') or {}).get(robot_id) or {}
    local_active_map = (runtime_status.get('local_active_maps') or {}).get(robot_id) or {}

    def command_observations(events):
        selected = events[:4] + events[-4:] if len(events) > 8 else events
        return [{'elapsed_wall_s': round(row[0] - manual_start_wall, 3),
                 'linear_x': row[1], 'linear_y': row[2], 'angular_z': row[3]}
                for row in selected]

    return {
        'passed': passed,
        'settling': settling, 'stop_settling': stop_settling,
        'reason': None if passed else
            'missing_r01_bridge_ack_directional_cmd_controller_joint_physical_motion_or_stop_zero',
        'robot_id': robot_id, 'bridge_connected_robot_ids': connected_ids,
        'bridge_r01_connected': robot_id in connected_ids,
        'web_command': {'type': 'ROBOT_MANUAL', 'robot_id': robot_id, 'action': action},
        'p0': pose0, 'p1': pose1, 'displacement_m': pose_change,
        'gazebo_p0': gazebo0, 'gazebo_p1': gazebo1,
        'gazebo_body_delta': gazebo_delta, 'odom_body_delta': odom_delta,
        'cmd_vel_nonzero_samples': len(active_commands),
        'manual_source_topic_samples': len(commands),
        'selected_cmd_vel_nonzero_samples': len(active_selected_commands),
        'command_owner': 'WEB_MANUAL' if manual_owner_seen else None,
        'observed_cmd_vel': ([{'linear_x': row[1], 'linear_y': row[2],
                               'angular_z': row[3]} for row in active_commands[:5]]),
        'expected_direction_component': expected_component,
        'expected_direction_sign': expected_sign,
        'command_direction_seen': command_direction_seen,
        'physical_directional_delta': physical_metric,
        'odom_directional_delta': odom_metric,
        'direction_tolerance': direction_tolerance,
        'controller_command_ack': command_ack,
        'control_statuses': [{'accepted': item.get('accepted'),
                              'reason': item.get('reason'),
                              'mode': item.get('mode')}
                             for item in action_statuses[-8:]],
        'manual_source_observations': command_observations(commands),
        'selected_command_observations': command_observations(selected_commands),
        'runtime_map_status': {
            'status': runtime_status.get('map_sync_status'),
            'error': runtime_status.get('map_sync_error'),
            'robot': robot_map_sync,
            'local_active': local_active_map,
            'tf_status': runtime_status.get('tf_status'),
        },
        'drive_command_samples': len(drives), 'drive_nonzero': drive_active,
        'steering_command_samples': len(steering), 'stop_zero_seen': zero_seen,
        'sim_duration_s': probe.sim_time() - sim_start,
        'web_command_count': len(send_times),
        'web_command_max_refresh_gap_s': max_refresh_gap,
        'watchdog_expected_timeout_s': 0.5,
        'wheel_velocity_max_rad_s': wheel_velocity_max,
        'wheel_position_change_rad': wheel_position_change,
        'joint_states_before': joint0, 'joint_states_after': joint1,
        'joint_state_samples': len(joint_samples),
    }


def web_navigation(probe, ws, robot_id, timeout):
    if not probe.wait_until(lambda: probe.map is not None and probe.map_pose() is not None,
                            10.0, ws):
        return {'passed': False, 'reason': 'map_or_map_to_base_footprint_unavailable'}
    if probe.gazebo_pose is None:
        return {'passed': False, 'reason': 'gazebo_model_pose_unavailable_before_web_goal'}
    goal, goal_error = probe.map_goal_candidate()
    if goal is None:
        return {'passed': False, 'reason': goal_error}
    mode_ok, mode_error = set_mode(probe, ws, robot_id, 'AUTONOMOUS')
    if not mode_ok:
        return {'passed': False, 'reason': f'ROBOT_MODE_AUTONOMOUS_rejected:{mode_error}', 'goal': goal}

    x, y, yaw = goal
    request_id = str(uuid.uuid4())
    active_map = ((probe.runtime_status or {}).get('local_active_maps') or {}).get(robot_id) or {}
    if not active_map.get('active_map_id') or not active_map.get('active_map_revision'):
        return {'passed': False, 'reason': 'active_robot_map_identity_unavailable', 'goal': goal}
    preview_payload = {
        'type': 'PATH_PREVIEW_REQUEST', 'robot_id': robot_id,
        'request_id': request_id, 'x': x, 'y': y, 'yaw': yaw,
        'frame_id': 'map', 'active_map_id': active_map['active_map_id'],
        'active_map_revision': active_map['active_map_revision'],
    }
    ws.send(json.dumps(preview_payload))
    preview = None
    preview_deadline = time.monotonic() + min(30.0, max(10.0, timeout * 0.25))
    while time.monotonic() < preview_deadline:
        probe.pump(ws, 0.03)
        preview = next((message for message in reversed(probe.ws_messages)
                        if message.get('type') == 'PATH_PREVIEW_RESULT'
                        and message.get('robot_id') == robot_id
                        and message.get('request_id') == request_id), None)
        if preview is not None:
            break
    path = preview.get('path') if isinstance(preview, dict) else None
    preview_goal = preview.get('goal') if isinstance(preview, dict) else None
    try:
        preview_target_matches = isinstance(preview_goal, dict) and all(
            abs(float(preview_goal.get(axis, float('nan'))) - expected) <= 1e-4
            for axis, expected in (('x', x), ('y', y), ('yaw', yaw)))
    except (TypeError, ValueError):
        preview_target_matches = False
    preview_valid = bool(
        preview and str(preview.get('status') or '').upper() == 'VALID'
        and isinstance(path, list) and path
        and preview.get('active_map_id') == active_map['active_map_id']
        and str(preview.get('active_map_revision')) == str(active_map['active_map_revision'])
        and preview_target_matches
    )
    if not preview_valid:
        return {
            'passed': False,
            'reason': (preview or {}).get('reason', 'valid_nav2_path_preview_not_received'),
            'goal': goal, 'preview': preview, 'path_preview_request': preview_payload,
            'active_map': active_map,
            'runtime_map_status': {
                'status': (probe.runtime_status or {}).get('map_sync_status'),
                'error': (probe.runtime_status or {}).get('map_sync_error'),
                'robot': ((probe.runtime_status or {}).get('robot_map_sync') or {}).get(robot_id),
                'local_active': ((probe.runtime_status or {}).get('local_active_maps') or {}).get(robot_id),
                'tf_status': (probe.runtime_status or {}).get('tf_status'),
            },
        }

    pose0, gazebo0 = probe.map_pose(), probe.gazebo_pose
    command_index = len(probe.nav_cmd_events)
    selected_index = len(probe.selected_cmd_events)
    owner_index = len(probe.command_owner_events)
    drive_index = len(probe.drive_events)
    action_status_index = len(probe.action_status_events)
    goal_payload = {
        'type': 'NAV_GOAL', 'robot_id': robot_id, 'x': x, 'y': y,
        'yaw': yaw, 'frame_id': 'map', 'preview_request_id': request_id,
        'active_map_id': active_map['active_map_id'],
        'active_map_revision': active_map['active_map_revision'],
    }
    ws.send(json.dumps(goal_payload))
    start = time.monotonic()
    accepted = False
    terminal = None
    while time.monotonic() - start < timeout:
        probe.pump(ws, 0.03)
        status = find_goal_status(probe, x, y, robot_id)
        if status:
            state = str(status.get('status') or '').upper()
            if state in ('ACTIVE', 'NAVIGATING'):
                accepted = True
            if state in ('SUCCEEDED', 'FAILED', 'CANCELED', 'EMERGENCY_STOPPED'):
                terminal = status
                break
        if any(message.get('type') == 'ERROR'
               for message in probe.ws_messages[-4:]):
            recent = [f'{item.get("code")}:{item.get("message")}'
                      for item in probe.ws_messages[-4:] if item.get('type') == 'ERROR']
            if recent:
                terminal = {'status': 'FAILED', 'reason': recent[-1]}
                break
    if terminal is None:
        ws.send(json.dumps({'type': 'NAV_CANCEL', 'robot_id': robot_id}))
        terminal = {'status': 'TIMEOUT', 'reason': f'no terminal NAV_STATUS within {timeout:.1f}s'}

    # Nav2 can report its terminal result just before wheel/chassis motion has
    # settled. Also, `goal` is in the active map frame while Gazebo ModelStates
    # is in the world frame; only compare the target against map -> base TF.
    settling = probe.wait_mechanical_settling(ws, timeout=20.0, wall_timeout=90.0)

    pose1, gazebo1 = probe.map_pose(), probe.gazebo_pose
    commands = probe.nav_cmd_events[command_index:]
    active_commands = [row for row in commands if any(abs(value) > 1e-4 for value in row[1:])]
    selected_commands = probe.selected_cmd_events[selected_index:]
    active_selected_commands = [row for row in selected_commands
                                if any(abs(value) > 1e-4 for value in row[1:])]
    owners = probe.command_owner_events[owner_index:]
    nav_owner_seen = any(owner == 'NAV2' for _, owner in owners)
    drives = probe.drive_events[drive_index:]
    drive_active = any(any(abs(value) > 1e-4 for value in row[1:]) for row in drives)
    pose_change = dist(pose0, pose1) if pose1 is not None else None
    physical_change = dist(gazebo0, gazebo1) if gazebo1 is not None else None
    goal_distance = dist(pose1, goal) if pose1 is not None else None
    goal_yaw_error = (abs(wrap_angle(pose1[2] - goal[2]))
                      if pose1 is not None else None)
    status_name = str(terminal.get('status') or 'UNKNOWN').upper()
    accepted = accepted or status_name == 'SUCCEEDED'
    passed = bool(accepted and status_name == 'SUCCEEDED' and active_commands
                  and active_selected_commands and nav_owner_seen
                  and drive_active and pose_change is not None
                  and pose_change > TRANSLATION_TOLERANCE_M
                  and physical_change is not None
                  and physical_change > TRANSLATION_TOLERANCE_M
                  and settling.get('passed') is True
                  and goal_distance is not None
                  and goal_distance <= NAV_GOAL_XY_TOLERANCE_M
                  and goal_yaw_error is not None
                  and goal_yaw_error <= NAV_GOAL_YAW_TOLERANCE_RAD)
    return {
        'passed': passed,
        'reason': None if passed else str(terminal.get('reason') or f'action_status={status_name}'),
        'accepted': accepted, 'status': status_name, 'goal': goal,
        'path_preview_request': preview_payload,
        'path_preview': {
            'status': preview.get('status'), 'request_id': request_id,
            'active_map_id': preview.get('active_map_id'),
            'active_map_revision': preview.get('active_map_revision'),
            'path_length_m': preview.get('path_length_m'),
            'path_points': len(path),
        },
        'runtime_map_status': {
            'status': (probe.runtime_status or {}).get('map_sync_status'),
            'error': (probe.runtime_status or {}).get('map_sync_error'),
            'robot': ((probe.runtime_status or {}).get('robot_map_sync') or {}).get(robot_id),
            'local_active': ((probe.runtime_status or {}).get('local_active_maps') or {}).get(robot_id),
            'tf_status': (probe.runtime_status or {}).get('tf_status'),
        },
        'web_payload': goal_payload,
        'ros_goal': {'x': goal[0], 'y': goal[1], 'yaw': goal[2], 'frame_id': 'map'},
        'goal_reason': terminal.get('reason'), 'p0': pose0, 'p1': pose1,
        'gazebo_p0': gazebo0, 'gazebo_p1': gazebo1,
        'settling': settling, 'goal_pose_frame': 'map',
        'displacement_m': pose_change, 'gazebo_displacement_m': physical_change,
        'goal_distance_m': goal_distance, 'goal_yaw_error_rad': goal_yaw_error,
        'goal_xy_tolerance_m': NAV_GOAL_XY_TOLERANCE_M,
        'goal_yaw_tolerance_rad': NAV_GOAL_YAW_TOLERANCE_RAD,
        'cmd_vel_nonzero_samples': len(active_commands),
        'selected_cmd_vel_nonzero_samples': len(active_selected_commands),
        'command_owner': 'NAV2' if nav_owner_seen else None,
        'drive_command_samples': len(drives), 'drive_nonzero': drive_active,
        'action_status_topic_samples': len(probe.action_status_events[action_status_index:]),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend-url', required=True)
    parser.add_argument('--robot-id', default='R01')
    parser.add_argument('--motion-timeout', type=float, default=45.0)
    parser.add_argument('--navigation-timeout', type=float, default=180.0)
    parser.add_argument('--manual-duration-sim', type=float, default=1.5)
    parser.add_argument('--json')
    args = parser.parse_args()

    result = {'stages': {}, 'passed': False}
    try:
        ws_url = args.backend_url.replace('https://', 'wss://').replace('http://', 'ws://') + '/ws'
        ws = websocket.create_connection(ws_url, timeout=5.0, enable_multithread=True)
        ws.settimeout(0.02)
    except Exception as exc:
        reason = f'{type(exc).__name__}:{exc}'
        print(f'DJANGO_WEBSOCKET_READY=FAIL reason={reason}', flush=True)
        result['stages']['DJANGO_WEBSOCKET_READY'] = False
        result['reason'] = reason
        if args.json:
            with open(args.json, 'w', encoding='utf-8') as stream:
                json.dump(result, stream, indent=2)
        return 1

    print('DJANGO_WEBSOCKET_READY=PASS protocol=/ws', flush=True)
    result['stages']['DJANGO_WEBSOCKET_READY'] = True
    rclpy.init()
    probe = MotionProbe()
    try:
        sim_ready = probe.wait_until(
            lambda: probe.odom is not None and probe.sim_time() > 0.0, 20.0, ws)
        result['stages']['ROS_POSE_READY'] = sim_ready
        print(f'ROS_POSE_READY={"PASS" if sim_ready else "FAIL reason=odom_or_clock_missing"}', flush=True)
        if not sim_ready:
            result['reason'] = 'no valid /odom or advancing /clock before movement checks'
            return 1

        idle = probe.observe_idle_cmd_vel()
        result['idle_cmd_vel'] = idle
        idle_arbiter_passed = bool(
            idle['selected_velocity_zero']
            and idle['selected_owner'] == 'NONE'
            and idle['selected_owner_none_seen']
        )
        result['stages']['COMMAND_ARBITER_IDLE'] = idle_arbiter_passed
        print(f'COMMAND_ARBITER_IDLE={"PASS" if idle_arbiter_passed else "FAIL"} '
              f'selected_samples={idle["selected_samples"]} '
              f'selected_nonzero={idle["selected_nonzero_samples"]} '
              f'owner={idle["selected_owner"]}', flush=True)
        print(f'IDLE_CMD_VEL samples={idle["samples"]} nonzero={idle["nonzero_samples"]} '
              f'unexpected={idle["unexpected_idle_traffic"]}', flush=True)

        bridge_ready_for_direct = probe.wait_until(
            lambda: probe.runtime_status is not None
            and args.robot_id in probe.runtime_status.get('connected_robot_ids', []),
            15.0, ws,
        )
        result['stages']['R01_BRIDGE_READY_FOR_DIRECT_CONTROL'] = bridge_ready_for_direct
        manual_mode_ready = False
        if bridge_ready_for_direct:
            manual_mode_ready, mode_reason = set_mode(probe, ws, args.robot_id, 'MANUAL')
            result['stages']['DIRECT_MANUAL_MODE_R01'] = manual_mode_ready
            if not manual_mode_ready:
                result['direct_manual_mode_reason'] = mode_reason
        else:
            result['stages']['DIRECT_MANUAL_MODE_R01'] = False

        sequence = (
            ('FORWARD', (0.25, 0.0, 0.0)),
            ('STRAFE', (0.0, 0.25, 0.0)),
            ('ROTATE', (0.0, 0.0, 0.4)),
        )
        direct_results = {}
        direct_blocker = None
        if idle['unexpected_idle_traffic'] or not bridge_ready_for_direct or not manual_mode_ready:
            direct_blocker = 'DIRECT_FORWARD'
            result['stages']['DIRECT_FORWARD'] = None
            reason = ('unexpected_idle_cmd_vel_traffic' if idle['unexpected_idle_traffic']
                      else 'R01_bridge_unavailable' if not bridge_ready_for_direct
                      else 'MANUAL_mode_not_accepted')
            print(f'DIRECT_FORWARD=UNVERIFIED reason={reason}; '
                  'motion not commanded', flush=True)
        else:
            for label, velocity in sequence:
                motion = probe.direct_motion(label, velocity, timeout=args.motion_timeout)
                direct_results[label] = motion
                passed = bool(motion['passed'])
                stage_name = f'DIRECT_{label}'
                result['stages'][stage_name] = passed
                result['stages'][f'DIRECT_ROS_{label}'] = passed
                result['stages'][f'DIRECT_{label}_COMMAND_OWNER'] = (
                    motion.get('command_owner') == 'DIRECT_MANUAL')
                status = 'PASS' if passed else 'FAIL'
                print(f'{stage_name}={status} reason={motion.get("reason")} '
                      f'command={motion.get("command")} '
                      f'gazebo_p0={motion.get("gazebo_p0")} gazebo_p1={motion.get("gazebo_p1")} '
                      f'gazebo_delta={motion.get("gazebo_body_delta")} '
                      f'odom_delta={motion.get("odom_body_delta")} '
                      f'wheel_velocity_max={motion.get("wheel_velocity_max_rad_s")} '
                      f'joint_samples={motion.get("joint_state_samples")}', flush=True)
                if not passed:
                    direct_blocker = stage_name
                    break
        result['direct_ros'] = direct_results

        all_direct = (not direct_blocker and len(direct_results) == len(sequence)
                      and all(item['passed'] for item in direct_results.values()))
        direct_nav_mode_ready = False
        if all_direct:
            direct_nav_mode_ready, direct_nav_mode_reason = set_mode(
                probe, ws, args.robot_id, 'AUTONOMOUS')
            result['stages']['DIRECT_AUTONOMOUS_MODE_R01'] = direct_nav_mode_ready
        else:
            direct_nav_mode_reason = f'blocked_by_{direct_blocker}'
        direct_navigation = ({'passed': False,
                              'reason': (f'AUTONOMOUS_mode_not_accepted:{direct_nav_mode_reason}'
                                         if all_direct else f'blocked_by_{direct_blocker}'),
                              'server_ready': None, 'accepted': None, 'status': 'UNVERIFIED'}
                             if not (all_direct and direct_nav_mode_ready) else
                             probe.direct_navigation(args.navigation_timeout))
        result['direct_navigation'] = direct_navigation
        if all_direct and direct_nav_mode_ready:
            direct_nav_stages = (
                ('DIRECT_NAV_SERVER_READY', bool(direct_navigation.get('server_ready'))),
                ('DIRECT_NAV_GOAL_ACCEPTED', bool(direct_navigation.get('accepted'))),
                ('DIRECT_NAV_CMD_VEL', direct_navigation.get('cmd_vel_nonzero_samples', 0) > 0),
                ('DIRECT_NAV_COMMAND_OWNER', direct_navigation.get('command_owner') == 'NAV2'),
                ('DIRECT_NAV_CONTROLLER_COMMAND', bool(direct_navigation.get('drive_nonzero'))),
                ('DIRECT_NAV_MOTION', direct_navigation.get('gazebo_displacement_m') is not None
                 and direct_navigation['gazebo_displacement_m'] > TRANSLATION_TOLERANCE_M),
                ('DIRECT_NAV_GOAL_TOLERANCE', direct_navigation.get('final_goal_error_m') is not None
                 and direct_navigation['final_goal_error_m'] <= NAV_GOAL_XY_TOLERANCE_M
                 and direct_navigation.get('final_goal_yaw_error_rad') is not None
                 and direct_navigation['final_goal_yaw_error_rad'] <= NAV_GOAL_YAW_TOLERANCE_RAD),
                ('DIRECT_NAV_SUCCEEDED', direct_navigation.get('status') == 'SUCCEEDED'),
            )
            for stage_name, passed in direct_nav_stages:
                result['stages'][stage_name] = bool(passed)
                print(f'{stage_name}={"PASS" if passed else "FAIL"} '
                      f'reason={direct_navigation.get("reason")} '
                      f'status={direct_navigation.get("status")} '
                      f'start={direct_navigation.get("gazebo_p0")} '
                      f'goal={direct_navigation.get("goal")} '
                      f'final={direct_navigation.get("gazebo_p1")} '
                      f'goal_error_m={direct_navigation.get("final_goal_error_m")} '
                      f'goal_yaw_error_rad={direct_navigation.get("final_goal_yaw_error_rad")}',
                      flush=True)
        else:
            for stage_name in ('DIRECT_NAV_SERVER_READY', 'DIRECT_NAV_GOAL_ACCEPTED',
                               'DIRECT_NAV_CMD_VEL', 'DIRECT_NAV_COMMAND_OWNER',
                               'DIRECT_NAV_CONTROLLER_COMMAND',
                               'DIRECT_NAV_MOTION', 'DIRECT_NAV_GOAL_TOLERANCE',
                               'DIRECT_NAV_SUCCEEDED'):
                result['stages'][stage_name] = None
            nav_reason = (f'AUTONOMOUS_mode_not_accepted:{direct_nav_mode_reason}'
                          if all_direct else f'blocked_by_{direct_blocker}')
            print(f'DIRECT_NAV_GOAL=UNVERIFIED reason={nav_reason}', flush=True)

        manual = {'passed': False, 'reason': f'blocked_by_{direct_blocker}',
                  'bridge_r01_connected': None}
        bridge_ready = probe.wait_until(
            lambda: probe.runtime_status is not None
            and args.robot_id in probe.runtime_status.get('connected_robot_ids', []),
            15.0, ws,
        )

        def active_robot_map_ready():
            status = probe.runtime_status or {}
            active = (status.get('local_active_maps') or {}).get(args.robot_id) or {}
            return (status.get('map_sync_status') in ('SYNCED', 'CANONICAL')
                    or active.get('map_sync_status') in ('SYNCED', 'CANONICAL', 'LOCAL_ONLY'))

        map_synced = probe.wait_until(active_robot_map_ready, 15.0, ws)
        result['stages']['ROS_BRIDGE_R01_WEBSOCKET_READY'] = bridge_ready
        print(f'ROS_BRIDGE_R01_WEBSOCKET_READY={"PASS" if bridge_ready else "FAIL reason=R01_not_connected_to_Django"} '
              f'connected_robot_ids={(probe.runtime_status or {}).get("connected_robot_ids")}', flush=True)
        result['stages']['WEB_MAP_SYNC_READY'] = map_synced
        print(f'WEB_MAP_SYNC_READY={"PASS" if map_synced else "FAIL reason=" + str((probe.runtime_status or {}).get("map_sync_error") or "active_robot_map_not_ready")}', flush=True)
        if bridge_ready and map_synced:
            mode_ok, mode_reason = set_mode(probe, ws, args.robot_id, 'MANUAL')
            result['stages']['WEB_MANUAL_MODE_R01'] = mode_ok
            print(f'WEB_MANUAL_MODE_R01={"PASS" if mode_ok else "FAIL reason=" + str(mode_reason)}', flush=True)
            manual_actions = ('FORWARD', 'BACKWARD', 'LEFT', 'RIGHT',
                              'ROTATE_LEFT', 'ROTATE_RIGHT')
            manual_results = {}
            if mode_ok:
                for action in manual_actions:
                    action_result = web_manual(
                        probe, ws, args.robot_id, action,
                        duration_sim_s=max(0.2, args.manual_duration_sim),
                        timeout=args.motion_timeout)
                    manual_results[action] = action_result
                    result['stages'][f'WEB_MANUAL_{action}'] = bool(action_result['passed'])
                    print(f'WEB_MANUAL_{action}={"PASS" if action_result["passed"] else "FAIL"} '
                              f'reason={action_result.get("reason")} '
                              f'cmd_vel={action_result.get("observed_cmd_vel")} '
                              f'manual_source_samples={action_result.get("manual_source_topic_samples")} '
                              f'selected_samples={action_result.get("selected_cmd_vel_nonzero_samples")} '
                              f'gazebo_delta={action_result.get("gazebo_body_delta")} '
                          f'expected_direction_delta={action_result.get("physical_directional_delta")} '
                          f'stop_zero={action_result.get("stop_zero_seen")} '
                          f'refresh_gap_s={action_result.get("web_command_max_refresh_gap_s")}',
                          flush=True)
                    if not action_result['passed']:
                        break
                for action in manual_actions:
                    if action not in manual_results:
                        result['stages'][f'WEB_MANUAL_{action}'] = None
            else:
                manual_results['FORWARD'] = {
                    'passed': False,
                    'reason': f'manual_mode_not_accepted:{mode_reason}',
                }
                for action in manual_actions:
                    result['stages'][f'WEB_MANUAL_{action}'] = False
            forward_result = manual_results.get('FORWARD', {})
            manual = dict(forward_result)
            manual['actions'] = manual_results
            manual['passed'] = (len(manual_results) == len(manual_actions)
                                and all(item.get('passed') for item in manual_results.values()))
        else:
            manual = {'passed': False, 'reason': 'R01_bridge_or_active_map_not_ready'}
        result['web_manual'] = manual
        for stage_name, passed, detail in (
            ('WEB_MANUAL_CMD_VEL', manual.get('selected_cmd_vel_nonzero_samples', 0) > 0,
             f'selected_samples={manual.get("selected_cmd_vel_nonzero_samples", 0)} '
             f'manual_source_topic_samples={manual.get("manual_source_topic_samples", 0)}'),
            ('WEB_MANUAL_CONTROLLER_COMMAND', bool(manual.get('drive_nonzero')),
             f'drive_samples={manual.get("drive_command_samples", 0)}'),
            ('WEB_MANUAL_COMMAND_OWNER', manual.get('command_owner') == 'WEB_MANUAL',
             f'owner={manual.get("command_owner")}'),
            ('WEB_MANUAL_COMMAND_ACCEPTED', bool(manual.get('controller_command_ack')),
             'R01_bridge_acknowledged_manual_command'),
            ('WEB_MANUAL_MOTION', bool(manual.get('passed')),
             f'gazebo_delta={manual.get("gazebo_body_delta")}'),
            ('WEB_MANUAL_STOP_ZERO', bool(manual.get('stop_zero_seen')),
             'zero_cmd_vel_observed_after_STOP'),
        ):
            result['stages'][stage_name] = bool(passed)
            print(f'{stage_name}={"PASS" if passed else "FAIL"} '
                  f'reason={manual.get("reason")} {detail}', flush=True)

        manual_ok = bool(manual.get('passed'))
        navigation = {'passed': False, 'reason': 'blocked_by_failed_precondition',
                      'status': 'UNVERIFIED', 'accepted': None}
        if (bridge_ready and map_synced
                and result['stages'].get('WEB_MANUAL_STOP_ZERO') is not False):
            mode_ok, mode_reason = set_mode(probe, ws, args.robot_id, 'AUTONOMOUS')
            if not mode_ok:
                navigation['reason'] = f'AUTONOMOUS_mode_not_accepted:{mode_reason}'
                print(f'WEB_NAV_GOAL_R01=UNVERIFIED reason={navigation["reason"]}', flush=True)
                result['web_navigation'] = navigation
                mode_ok = False
            if mode_ok:
                navigation = web_navigation(probe, ws, args.robot_id, args.navigation_timeout)
        else:
            reason = ('web_manual_r01_failed' if not manual_ok
                      else 'R01_bridge_or_active_map_not_ready')
            navigation['reason'] = reason
            print(f'WEB_NAV_GOAL_R01=UNVERIFIED reason={reason}', flush=True)
        result['web_navigation'] = navigation
        nav_status = str(navigation.get('status') or 'UNKNOWN')
        if navigation.get('status') != 'UNVERIFIED':
            for stage_name, passed, detail in (
                ('WEB_NAV_GOAL_ACCEPTED', bool(navigation.get('accepted')),
                 f'accepted={navigation.get("accepted")}'),
                ('WEB_NAV_CMD_VEL', navigation.get('cmd_vel_nonzero_samples', 0) > 0,
                 f'samples={navigation.get("cmd_vel_nonzero_samples", 0)}'),
                ('WEB_NAV_COMMAND_OWNER', navigation.get('command_owner') == 'NAV2',
                 f'owner={navigation.get("command_owner")}'),
                ('WEB_NAV_CONTROLLER_COMMAND', bool(navigation.get('drive_nonzero')),
                 f'drive_samples={navigation.get("drive_command_samples", 0)}'),
                ('WEB_NAV_MOTION', navigation.get('gazebo_displacement_m') is not None
                 and navigation['gazebo_displacement_m'] > TRANSLATION_TOLERANCE_M,
                 f'gazebo_displacement={navigation.get("gazebo_displacement_m")}'),
                ('WEB_NAV_GOAL_TOLERANCE', navigation.get('goal_distance_m') is not None
                 and navigation['goal_distance_m'] <= NAV_GOAL_XY_TOLERANCE_M
                 and navigation.get('goal_yaw_error_rad') is not None
                 and navigation['goal_yaw_error_rad'] <= NAV_GOAL_YAW_TOLERANCE_RAD,
                 f'goal_error={navigation.get("goal_distance_m")} yaw_error={navigation.get("goal_yaw_error_rad")}'),
                ('WEB_NAV_SUCCEEDED', nav_status == 'SUCCEEDED', f'status={nav_status}'),
            ):
                result['stages'][stage_name] = bool(passed)
                print(f'{stage_name}={"PASS" if passed else "FAIL"} '
                      f'reason={navigation.get("reason")} {detail}', flush=True)
        else:
            for stage_name in ('WEB_NAV_GOAL_ACCEPTED', 'WEB_NAV_CMD_VEL',
                               'WEB_NAV_COMMAND_OWNER', 'WEB_NAV_CONTROLLER_COMMAND', 'WEB_NAV_MOTION',
                               'WEB_NAV_GOAL_TOLERANCE', 'WEB_NAV_SUCCEEDED'):
                result['stages'][stage_name] = None

        all_web_manual = all(result['stages'].get(key) is True for key in (
            'WEB_MANUAL_CMD_VEL', 'WEB_MANUAL_CONTROLLER_COMMAND', 'WEB_MANUAL_COMMAND_OWNER',
            'WEB_MANUAL_COMMAND_ACCEPTED', 'WEB_MANUAL_MOTION', 'WEB_MANUAL_STOP_ZERO',
            'WEB_MANUAL_FORWARD', 'WEB_MANUAL_BACKWARD', 'WEB_MANUAL_LEFT',
            'WEB_MANUAL_RIGHT', 'WEB_MANUAL_ROTATE_LEFT', 'WEB_MANUAL_ROTATE_RIGHT'))
        all_web_nav = all(result['stages'].get(key) is True for key in (
            'WEB_NAV_GOAL_ACCEPTED', 'WEB_NAV_CMD_VEL', 'WEB_NAV_COMMAND_OWNER',
            'WEB_NAV_CONTROLLER_COMMAND',
            'WEB_NAV_MOTION', 'WEB_NAV_GOAL_TOLERANCE', 'WEB_NAV_SUCCEEDED'))
        all_direct_navigation = bool(direct_navigation.get('passed'))
        result['passed'] = bool(all_direct and all_direct_navigation
                                and all_web_manual and all_web_nav)
        result['reason'] = None if result['passed'] else 'one_or_more_runtime_acceptance_stages_failed'
        print(f'END_TO_END_ACCEPTANCE={"PASS" if result["passed"] else "FAIL reason=" + result["reason"]}', flush=True)
        return 0 if result['passed'] else 1
    finally:
        try:
            ws.close()
        except Exception:
            pass
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if args.json:
            with open(args.json, 'w', encoding='utf-8') as stream:
                json.dump(result, stream, indent=2)


if __name__ == '__main__':
    raise SystemExit(main())
