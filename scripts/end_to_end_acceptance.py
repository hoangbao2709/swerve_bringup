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
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformException, TransformListener


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
        self.filtered_odom = None
        self.odom_count = 0
        self.gazebo_pose = None
        self.gazebo_pose_count = 0
        self.gazebo_pose_sample_monotonic = None
        self.joint_state = None
        self.joint_events: list[tuple[float, dict, dict]] = []
        self.map = None
        self.cmd_events: list[tuple[float, float, float, float]] = []
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
        self.cmd_events.append((time.monotonic(), msg.linear.x,
                                msg.linear.y, msg.angular.z))
        gid = getattr(info, 'publisher_gid', None)
        self.cmd_publisher_gids.append(bytes(gid).hex() if gid is not None else None)

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
        deadline = start_time + duration_wall_s
        while time.monotonic() < deadline:
            self.pump(timeout=min(0.02, deadline - time.monotonic()))
        events = self.cmd_events[start_index:]
        nonzero = [row for row in events if any(abs(value) > 1e-4 for value in row[1:])]
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

    def stop_direct(self, timeout=1.5):
        start_time = time.monotonic()
        start_index = len(self.drive_events)
        stop_until = start_time + timeout
        last_publish = 0.0
        zero_seen = False
        while time.monotonic() < stop_until:
            now = time.monotonic()
            if now - last_publish >= 0.10:
                self.command.publish(Twist())
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
            watchdog_start_sim = self.sim_time()
            watchdog_start_wall = time.monotonic()
            watchdog_drive_index = len(self.drive_events)
            watchdog_cmd_index = len(self.cmd_events)
            watchdog_zero_seen = False
            while (self.sim_time() - watchdog_start_sim < 1.5
                   and time.monotonic() - watchdog_start_wall < 12.0):
                self.pump(timeout=0.02)
                controller_rows = self.drive_events[watchdog_drive_index:]
                incoming_rows = self.cmd_events[watchdog_cmd_index:]
                if incoming_rows:
                    break
                if any(all(abs(value) <= 1e-4 for value in row[1:])
                       for row in controller_rows):
                    watchdog_zero_seen = True
                    break
            watchdog_zero_sim = self.sim_time()
            watchdog_result = {
                'tested': True,
                'command_timeout_config_s': 0.5,
                'zero_drive_command_seen': watchdog_zero_seen,
                'unexpected_cmd_vel_samples_after_publisher_stopped':
                    len(self.cmd_events[watchdog_cmd_index:]),
                'delay_after_last_command_publish_sim_s':
                    max(0.0, watchdog_zero_sim - last_command_publish_sim)
                    if watchdog_zero_seen else None,
                'wait_wall_s': time.monotonic() - watchdog_start_wall,
            }
        stop_result = self.stop_direct()
        pose1, gazebo1 = self.odom, self.gazebo_pose
        joint1 = self.joint_snapshot(self.joint_state)
        recent_cmd = self.cmd_events[command_index:]
        active_cmd = [row for row in recent_cmd
                      if any(abs(value) > 1e-4 for value in row[1:])]
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
            elapsed_sim > 0.0 and direction_passed and active_cmd and drive_active
            and (wheel_velocity_max > 0.05 or wheel_position_change > 0.05)
            and steering_response and stop_result['zero_drive_command_seen']
            and (label != 'FORWARD' or watchdog_result.get('zero_drive_command_seen')))
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
        command_index = len(self.cmd_events)
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
            self.stop_direct()
            return {'passed': False, 'server_ready': True,
                    'reason': 'goal_acceptance_timeout:/navigate_to_pose',
                    'goal': goal_pose}
        goal_handle = goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.stop_direct()
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

        stop_result = self.stop_direct()
        pose1, gazebo1 = self.map_pose(), self.gazebo_pose
        commands = self.cmd_events[command_index:]
        active_commands = [row for row in commands
                           if any(abs(value) > 1e-4 for value in row[1:])]
        drives = self.drive_events[drive_index:]
        drive_active = any(any(abs(value) > 1e-4 for value in row[1:]) for row in drives)
        pose_change = dist(pose0, pose1) if pose1 is not None else None
        physical_change = dist(gazebo0, gazebo1) if gazebo1 is not None else None
        final_goal_error = dist(gazebo1, goal_pose) if gazebo1 is not None else None
        final_yaw_error = (abs(wrap_angle(gazebo1[2] - goal_pose[2]))
                           if gazebo1 is not None else None)
        passed = bool(
            terminal_status == 'SUCCEEDED' and active_commands and drive_active
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
            'drive_command_samples': len(drives), 'drive_nonzero': drive_active,
        }


def http_json(url: str, body=None, token=None, timeout=8.0):
    data = json.dumps(body).encode('utf-8') if body is not None else None
    headers = {'Content-Type': 'application/json'} if data is not None else {}
    if token:
        headers['Authorization'] = f'Bearer {token}'
    request = urllib.request.Request(url, data=data, headers=headers,
                                     method='POST' if data is not None else 'GET')
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.loads(response.read().decode('utf-8'))
    except (OSError, ValueError, urllib.error.URLError) as exc:
        raise RuntimeError(f'{url} failed: {type(exc).__name__}: {exc}') from exc


def authenticate(backend_url):
    username = os.environ.get('TWIN_ADMIN_USERNAME', '').strip()
    password = os.environ.get('TWIN_ADMIN_PASSWORD', '')
    if not username or not password:
        raise RuntimeError('TWIN_ADMIN_USERNAME or TWIN_ADMIN_PASSWORD is missing from backend/.env')
    response = http_json(backend_url.rstrip('/') + '/api/auth/login',
                         {'username': username, 'password': password})
    token = str(response.get('access_token') or '')
    if not token:
        raise RuntimeError('Django login returned no access_token')
    return token


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
    sensors_ready = probe.wait_until(
        lambda: probe.odom is not None and probe.gazebo_pose is not None
        and probe.joint_state is not None, 10.0, ws)
    if not sensors_ready:
        return {'passed': False,
                'reason': 'missing_/odom_/model_states_or_/joint_states_before_manual_command'}
    pose0, gazebo0 = probe.odom, probe.gazebo_pose
    joint0 = probe.joint_snapshot(probe.joint_state)
    sim_start = probe.sim_time()
    wall_deadline = time.monotonic() + timeout
    next_send = 0.0
    send_times = []
    command_index = len(probe.cmd_events)
    drive_index = len(probe.drive_events)
    steering_index = len(probe.steering_events)
    joint_index = len(probe.joint_events)
    control_index = len(probe.control_statuses)
    payload = json.dumps({'type': 'ROBOT_MANUAL', 'robot_id': robot_id, 'action': action})
    while (probe.sim_time() - sim_start < duration_sim_s
           and time.monotonic() < wall_deadline):
        now = time.monotonic()
        if now >= next_send:
            ws.send(payload)
            send_times.append(now)
            next_send = now + WEB_COMMAND_REFRESH_WALL_S
        probe.pump(ws, 0.02)
    stop_time = time.monotonic()
    ws.send(json.dumps({'type': 'ROBOT_MANUAL', 'robot_id': robot_id, 'action': 'STOP'}))
    zero_seen = probe.wait_until(
        lambda: any(row[0] >= stop_time and all(abs(value) <= 1e-4 for value in row[1:])
                    for row in probe.cmd_events[command_index:]),
        min(5.0, max(1.0, timeout / 4.0)), ws,
    )
    pose1 = probe.odom
    gazebo1 = probe.gazebo_pose
    joint1 = probe.joint_snapshot(probe.joint_state)
    commands = probe.cmd_events[command_index:]
    active_commands = [row for row in commands if row[0] < stop_time
                       and any(abs(value) > 1e-4 for value in row[1:])]
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
        for row in active_commands)
    direction_tolerance = (ROTATION_TOLERANCE_RAD if expected_component == 'angular_z'
                           else TRANSLATION_TOLERANCE_M)
    passed = bool(command_ack and command_direction_seen and drive_active
                  and (wheel_velocity_max > 0.05 or wheel_position_change > 0.05)
                  and physical_metric is not None
                  and physical_metric > direction_tolerance
                  and odom_metric is not None and odom_metric > direction_tolerance * 0.5
                  and zero_seen)
    max_refresh_gap = max((later - earlier for earlier, later in zip(send_times, send_times[1:])),
                          default=None)
    connected_ids = ((probe.runtime_status or {}).get('connected_robot_ids') or [])
    return {
        'passed': passed,
        'reason': None if passed else
            'missing_r01_bridge_ack_directional_cmd_controller_joint_physical_motion_or_stop_zero',
        'robot_id': robot_id, 'bridge_connected_robot_ids': connected_ids,
        'bridge_r01_connected': robot_id in connected_ids,
        'web_command': {'type': 'ROBOT_MANUAL', 'robot_id': robot_id, 'action': action},
        'p0': pose0, 'p1': pose1, 'displacement_m': pose_change,
        'gazebo_p0': gazebo0, 'gazebo_p1': gazebo1,
        'gazebo_body_delta': gazebo_delta, 'odom_body_delta': odom_delta,
        'cmd_vel_nonzero_samples': len(active_commands),
        'observed_cmd_vel': ([{'linear_x': row[1], 'linear_y': row[2],
                               'angular_z': row[3]} for row in active_commands[:5]]),
        'expected_direction_component': expected_component,
        'expected_direction_sign': expected_sign,
        'command_direction_seen': command_direction_seen,
        'physical_directional_delta': physical_metric,
        'odom_directional_delta': odom_metric,
        'direction_tolerance': direction_tolerance,
        'controller_command_ack': command_ack,
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
    pose0, gazebo0 = probe.map_pose(), probe.gazebo_pose
    command_index = len(probe.cmd_events)
    drive_index = len(probe.drive_events)
    action_status_index = len(probe.action_status_events)
    goal_payload = {
        'type': 'NAV_GOAL', 'robot_id': robot_id, 'x': x, 'y': y,
        'yaw': yaw, 'frame_id': 'map',
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

    pose1, gazebo1 = probe.map_pose(), probe.gazebo_pose
    commands = probe.cmd_events[command_index:]
    active_commands = [row for row in commands if any(abs(value) > 1e-4 for value in row[1:])]
    drives = probe.drive_events[drive_index:]
    drive_active = any(any(abs(value) > 1e-4 for value in row[1:]) for row in drives)
    pose_change = dist(pose0, pose1) if pose1 is not None else None
    physical_change = dist(gazebo0, gazebo1) if gazebo1 is not None else None
    goal_distance = dist(gazebo1, goal) if gazebo1 is not None else None
    goal_yaw_error = (abs(wrap_angle(gazebo1[2] - goal[2]))
                      if gazebo1 is not None else None)
    status_name = str(terminal.get('status') or 'UNKNOWN').upper()
    accepted = accepted or status_name == 'SUCCEEDED'
    passed = bool(accepted and status_name == 'SUCCEEDED' and active_commands
                  and drive_active and pose_change is not None
                  and pose_change > TRANSLATION_TOLERANCE_M
                  and physical_change is not None
                  and physical_change > TRANSLATION_TOLERANCE_M
                  and goal_distance is not None
                  and goal_distance <= NAV_GOAL_XY_TOLERANCE_M
                  and goal_yaw_error is not None
                  and goal_yaw_error <= NAV_GOAL_YAW_TOLERANCE_RAD)
    return {
        'passed': passed,
        'reason': None if passed else str(terminal.get('reason') or f'action_status={status_name}'),
        'accepted': accepted, 'status': status_name, 'goal': goal,
        'web_payload': {'type': 'NAV_GOAL', 'robot_id': robot_id,
                        'x': goal[0], 'y': goal[1], 'yaw': goal[2], 'frame_id': 'map'},
        'ros_goal': {'x': goal[0], 'y': goal[1], 'yaw': goal[2], 'frame_id': 'map'},
        'goal_reason': terminal.get('reason'), 'p0': pose0, 'p1': pose1,
        'gazebo_p0': gazebo0, 'gazebo_p1': gazebo1,
        'displacement_m': pose_change, 'gazebo_displacement_m': physical_change,
        'goal_distance_m': goal_distance, 'goal_yaw_error_rad': goal_yaw_error,
        'goal_xy_tolerance_m': NAV_GOAL_XY_TOLERANCE_M,
        'goal_yaw_tolerance_rad': NAV_GOAL_YAW_TOLERANCE_RAD,
        'cmd_vel_nonzero_samples': len(active_commands),
        'drive_command_samples': len(drives), 'drive_nonzero': drive_active,
        'action_status_topic_samples': len(probe.action_status_events[action_status_index:]),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend-url', required=True)
    parser.add_argument('--robot-id', default='R01')
    parser.add_argument('--motion-timeout', type=float, default=45.0)
    parser.add_argument('--navigation-timeout', type=float, default=180.0)
    parser.add_argument('--json')
    args = parser.parse_args()

    result = {'stages': {}, 'passed': False}
    token = None
    try:
        token = authenticate(args.backend_url)
        ws_url = args.backend_url.replace('https://', 'wss://').replace('http://', 'ws://') + '/ws'
        ws = websocket.create_connection(ws_url + '?token=' + token, timeout=5.0,
                                         enable_multithread=True)
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
        print(f'IDLE_CMD_VEL samples={idle["samples"]} nonzero={idle["nonzero_samples"]} '
              f'unexpected={idle["unexpected_idle_traffic"]}', flush=True)

        sequence = (
            ('FORWARD', (0.25, 0.0, 0.0)),
            ('STRAFE', (0.0, 0.25, 0.0)),
            ('ROTATE', (0.0, 0.0, 0.4)),
        )
        direct_results = {}
        direct_blocker = None
        if idle['unexpected_idle_traffic']:
            direct_blocker = 'DIRECT_FORWARD'
            result['stages']['DIRECT_FORWARD'] = None
            print('DIRECT_FORWARD=UNVERIFIED reason=unexpected_idle_cmd_vel_traffic; '
                  'motion not commanded', flush=True)
        else:
            for label, velocity in sequence:
                motion = probe.direct_motion(label, velocity, timeout=args.motion_timeout)
                direct_results[label] = motion
                passed = bool(motion['passed'])
                stage_name = f'DIRECT_{label}'
                result['stages'][stage_name] = passed
                result['stages'][f'DIRECT_ROS_{label}'] = passed
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
        direct_navigation = ({'passed': False, 'reason': f'blocked_by_{direct_blocker}',
                              'server_ready': None, 'accepted': None, 'status': 'UNVERIFIED'}
                             if not all_direct else
                             probe.direct_navigation(args.navigation_timeout))
        result['direct_navigation'] = direct_navigation
        if all_direct:
            direct_nav_stages = (
                ('DIRECT_NAV_SERVER_READY', bool(direct_navigation.get('server_ready'))),
                ('DIRECT_NAV_GOAL_ACCEPTED', bool(direct_navigation.get('accepted'))),
                ('DIRECT_NAV_CMD_VEL', direct_navigation.get('cmd_vel_nonzero_samples', 0) > 0),
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
                               'DIRECT_NAV_CMD_VEL', 'DIRECT_NAV_CONTROLLER_COMMAND',
                               'DIRECT_NAV_MOTION', 'DIRECT_NAV_GOAL_TOLERANCE',
                               'DIRECT_NAV_SUCCEEDED'):
                result['stages'][stage_name] = None
            print(f'DIRECT_NAV_GOAL=UNVERIFIED reason=blocked_by_{direct_blocker}', flush=True)

        manual = {'passed': False, 'reason': f'blocked_by_{direct_blocker}',
                  'bridge_r01_connected': None}
        bridge_ready = False
        map_synced = False
        if all_direct:
            bridge_ready = probe.wait_until(
                lambda: probe.runtime_status is not None
                and args.robot_id in probe.runtime_status.get('connected_robot_ids', []),
                15.0, ws,
            )
            map_synced = probe.wait_until(
                lambda: probe.runtime_status is not None
                and probe.runtime_status.get('map_sync_status') == 'SYNCED',
                15.0, ws,
            )
            result['stages']['ROS_BRIDGE_R01_WEBSOCKET_READY'] = bridge_ready
            print(f'ROS_BRIDGE_R01_WEBSOCKET_READY={"PASS" if bridge_ready else "FAIL reason=R01_not_connected_to_Django"} '
                  f'connected_robot_ids={(probe.runtime_status or {}).get("connected_robot_ids")}', flush=True)
            result['stages']['WEB_MAP_SYNC_READY'] = map_synced
            print(f'WEB_MAP_SYNC_READY={"PASS" if map_synced else "FAIL reason=" + str((probe.runtime_status or {}).get("map_sync_error") or "status_not_SYNCED")}', flush=True)
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
                            probe, ws, args.robot_id, action, duration_sim_s=0.8,
                            timeout=args.motion_timeout)
                        manual_results[action] = action_result
                        result['stages'][f'WEB_MANUAL_{action}'] = bool(action_result['passed'])
                        print(f'WEB_MANUAL_{action}={"PASS" if action_result["passed"] else "FAIL"} '
                              f'reason={action_result.get("reason")} '
                              f'cmd_vel={action_result.get("observed_cmd_vel")} '
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
                manual = {'passed': False, 'reason': 'R01_bridge_or_published_map_not_ready'}
        else:
            print(f'WEB_MANUAL_R01=UNVERIFIED reason=blocked_by_{direct_blocker}', flush=True)
        result['web_manual'] = manual
        if all_direct:
            for stage_name, passed, detail in (
                ('WEB_MANUAL_CMD_VEL', manual.get('cmd_vel_nonzero_samples', 0) > 0,
                 f'samples={manual.get("cmd_vel_nonzero_samples", 0)}'),
                ('WEB_MANUAL_CONTROLLER_COMMAND', bool(manual.get('drive_nonzero')),
                 f'drive_samples={manual.get("drive_command_samples", 0)}'),
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
        else:
            for stage_name in ('WEB_MANUAL_CMD_VEL', 'WEB_MANUAL_CONTROLLER_COMMAND',
                               'WEB_MANUAL_COMMAND_ACCEPTED', 'WEB_MANUAL_MOTION',
                               'WEB_MANUAL_STOP_ZERO'):
                result['stages'][stage_name] = None

        manual_ok = bool(manual.get('passed'))
        navigation = {'passed': False, 'reason': 'blocked_by_failed_precondition',
                      'status': 'UNVERIFIED', 'accepted': None}
        if all_direct and direct_navigation.get('passed') and manual_ok and bridge_ready and map_synced:
            navigation = web_navigation(probe, ws, args.robot_id, args.navigation_timeout)
        else:
            reason = ('direct_nav_goal_failed' if all_direct and not direct_navigation.get('passed')
                      else 'web_manual_r01_failed' if all_direct and not manual_ok
                      else f'blocked_by_{direct_blocker}')
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
                               'WEB_NAV_CONTROLLER_COMMAND', 'WEB_NAV_MOTION',
                               'WEB_NAV_GOAL_TOLERANCE', 'WEB_NAV_SUCCEEDED'):
                result['stages'][stage_name] = None

        all_web_manual = all(result['stages'].get(key) is True for key in (
            'WEB_MANUAL_CMD_VEL', 'WEB_MANUAL_CONTROLLER_COMMAND',
            'WEB_MANUAL_COMMAND_ACCEPTED', 'WEB_MANUAL_MOTION', 'WEB_MANUAL_STOP_ZERO',
            'WEB_MANUAL_FORWARD', 'WEB_MANUAL_BACKWARD', 'WEB_MANUAL_LEFT',
            'WEB_MANUAL_RIGHT', 'WEB_MANUAL_ROTATE_LEFT', 'WEB_MANUAL_ROTATE_RIGHT'))
        all_web_nav = all(result['stages'].get(key) is True for key in (
            'WEB_NAV_GOAL_ACCEPTED', 'WEB_NAV_CMD_VEL', 'WEB_NAV_CONTROLLER_COMMAND',
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
        if token:
            try:
                http_json(args.backend_url.rstrip('/') + '/api/auth/logout', {}, token=token)
            except Exception:
                pass
        if args.json:
            with open(args.json, 'w', encoding='utf-8') as stream:
                json.dump(result, stream, indent=2)


if __name__ == '__main__':
    raise SystemExit(main())
