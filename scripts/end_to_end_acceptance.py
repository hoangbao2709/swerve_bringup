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
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformException, TransformListener


TRANSLATION_TOLERANCE_M = 0.035
ROTATION_TOLERANCE_RAD = 0.10
MOTION_DURATION_SIM_S = 1.0
WEB_COMMAND_REFRESH_WALL_S = 0.10


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def pose_from_odom(msg: Odometry) -> tuple[float, float, float]:
    p = msg.pose.pose.position
    return p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation)


def dist(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


class MotionProbe(Node):
    def __init__(self):
        super().__init__('simulation_e2e_acceptance', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.odom = None
        self.odom_count = 0
        self.map = None
        self.cmd_events: list[tuple[float, float, float, float]] = []
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
        self.create_subscription(OccupancyGrid, '/map', self._map_cb,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 20)
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

    def _odom_cb(self, msg):
        p = pose_from_odom(msg)
        if all(math.isfinite(value) for value in p):
            self.odom = p
            self.odom_count += 1

    def _map_cb(self, msg):
        if (msg.info.width > 0 and msg.info.height > 0
                and len(msg.data) == msg.info.width * msg.info.height):
            self.map = msg

    def _cmd_cb(self, msg):
        self.cmd_events.append((time.monotonic(), msg.linear.x,
                                msg.linear.y, msg.angular.z))

    def _action_status_cb(self, msg):
        self.action_status_events.append((
            time.monotonic(), {bytes(item.goal_info.goal_id.uuid): int(item.status)
                               for item in msg.status_list},
        ))

    def pump(self, ws=None, timeout=0.02):
        rclpy.spin_once(self, timeout_sec=timeout)
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

    def stop_direct(self):
        stop_until = time.monotonic() + 0.4
        while time.monotonic() < stop_until:
            self.command.publish(Twist())
            self.pump(timeout=0.02)

    def direct_motion(self, label, velocity, duration=MOTION_DURATION_SIM_S,
                      timeout=45.0):
        if not self.wait_until(lambda: self.odom is not None, min(timeout, 15.0)):
            return {'passed': False, 'reason': 'no_valid_/odom_pose'}
        pose0 = self.odom
        start_sim = self.sim_time()
        start_wall = time.monotonic()
        command_index = len(self.cmd_events)
        steering_index = len(self.steering_events)
        drive_index = len(self.drive_events)
        last_publish = 0.0
        wall_deadline = start_wall + timeout
        while self.sim_time() - start_sim < duration and time.monotonic() < wall_deadline:
            now = time.monotonic()
            if now - last_publish >= 0.10:
                msg = Twist()
                msg.linear.x, msg.linear.y, msg.angular.z = velocity
                self.command.publish(msg)
                last_publish = now
            self.pump(timeout=0.02)
        elapsed_sim = self.sim_time() - start_sim
        self.stop_direct()
        pose1 = self.odom
        recent_cmd = self.cmd_events[command_index:]
        active_cmd = [row for row in recent_cmd
                      if any(abs(value) > 1e-4 for value in row[1:])]
        drive = self.drive_events[drive_index:]
        steering = self.steering_events[steering_index:]
        drive_active = any(any(abs(value) > 1e-4 for value in row[1:]) for row in drive)
        steering_active = any(any(abs(value) > 1e-4 for value in row[1:]) for row in steering)
        if pose1 is None:
            measured = None
        elif abs(velocity[2]) > 1e-9:
            measured = abs(wrap_angle(pose1[2] - pose0[2]))
        else:
            measured = dist(pose0, pose1)
        threshold = ROTATION_TOLERANCE_RAD if abs(velocity[2]) > 1e-9 else TRANSLATION_TOLERANCE_M
        passed = (elapsed_sim > 0.0 and measured is not None and measured > threshold
                  and bool(active_cmd) and drive_active)
        return {
            'passed': passed,
            'reason': None if passed else 'missing_cmd_vel_controller_command_or_pose_displacement',
            'p0': pose0, 'p1': pose1, 'displacement_m_or_yaw_rad': measured,
            'tolerance': threshold, 'sim_duration_s': elapsed_sim,
            'cmd_vel_nonzero_samples': len(active_cmd),
            'steering_command_samples': len(steering),
            'steering_nonzero': steering_active,
            'drive_command_samples': len(drive), 'drive_nonzero': drive_active,
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
        pose0 = self.map_pose()
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

        self.stop_direct()
        pose1 = self.map_pose()
        commands = self.cmd_events[command_index:]
        active_commands = [row for row in commands
                           if any(abs(value) > 1e-4 for value in row[1:])]
        drives = self.drive_events[drive_index:]
        drive_active = any(any(abs(value) > 1e-4 for value in row[1:]) for row in drives)
        pose_change = dist(pose0, pose1) if pose1 is not None else None
        passed = bool(terminal_status == 'SUCCEEDED' and active_commands and drive_active
                      and pose_change is not None
                      and pose_change > TRANSLATION_TOLERANCE_M)
        return {
            'passed': passed, 'server_ready': True, 'accepted': True, 'status': terminal_status,
            'reason': None if passed else (exact_reason or
                'goal_succeeded_without_observed_cmd_vel_controller_command_or_pose_change'),
            'goal': goal_pose, 'p0': pose0, 'p1': pose1,
            'displacement_m': pose_change,
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
    if probe.odom is None:
        if not probe.wait_until(lambda: probe.odom is not None, 10.0, ws):
            return {'passed': False, 'reason': 'no_valid_/odom_before_manual_command'}
    pose0 = probe.odom
    sim_start = probe.sim_time()
    wall_deadline = time.monotonic() + timeout
    next_send = 0.0
    command_index = len(probe.cmd_events)
    drive_index = len(probe.drive_events)
    steering_index = len(probe.steering_events)
    control_index = len(probe.control_statuses)
    payload = json.dumps({'type': 'ROBOT_MANUAL', 'robot_id': robot_id, 'action': action})
    while (probe.sim_time() - sim_start < duration_sim_s
           and time.monotonic() < wall_deadline):
        now = time.monotonic()
        if now >= next_send:
            ws.send(payload)
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
    commands = probe.cmd_events[command_index:]
    active_commands = [row for row in commands if row[0] < stop_time
                       and any(abs(value) > 1e-4 for value in row[1:])]
    drives = probe.drive_events[drive_index:]
    steering = probe.steering_events[steering_index:]
    drive_active = any(row[0] < stop_time and any(abs(value) > 1e-4 for value in row[1:])
                       for row in drives)
    pose_change = dist(pose0, pose1) if pose1 is not None else None
    action_statuses = probe.control_statuses[control_index:]
    command_ack = any(item.get('robot_id') == robot_id and item.get('accepted') is True
                      for item in action_statuses)
    passed = bool(command_ack and active_commands and drive_active
                  and pose_change is not None
                  and pose_change > TRANSLATION_TOLERANCE_M and zero_seen)
    return {
        'passed': passed,
        'reason': None if passed else 'missing_bridge_ack_nonzero_cmd_vel_controller_command_motion_or_stop_zero',
        'p0': pose0, 'p1': pose1, 'displacement_m': pose_change,
        'cmd_vel_nonzero_samples': len(active_commands),
        'controller_command_ack': command_ack,
        'drive_command_samples': len(drives), 'drive_nonzero': drive_active,
        'steering_command_samples': len(steering), 'stop_zero_seen': zero_seen,
        'sim_duration_s': probe.sim_time() - sim_start,
    }


def web_navigation(probe, ws, robot_id, timeout):
    if not probe.wait_until(lambda: probe.map is not None and probe.map_pose() is not None,
                            10.0, ws):
        return {'passed': False, 'reason': 'map_or_map_to_base_footprint_unavailable'}
    goal, goal_error = probe.map_goal_candidate()
    if goal is None:
        return {'passed': False, 'reason': goal_error}
    mode_ok, mode_error = set_mode(probe, ws, robot_id, 'AUTONOMOUS')
    if not mode_ok:
        return {'passed': False, 'reason': f'ROBOT_MODE_AUTONOMOUS_rejected:{mode_error}', 'goal': goal}

    x, y, yaw = goal
    pose0 = probe.map_pose()
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

    pose1 = probe.map_pose()
    commands = probe.cmd_events[command_index:]
    active_commands = [row for row in commands if any(abs(value) > 1e-4 for value in row[1:])]
    drives = probe.drive_events[drive_index:]
    drive_active = any(any(abs(value) > 1e-4 for value in row[1:]) for row in drives)
    pose_change = dist(pose0, pose1) if pose1 is not None else None
    goal_distance = dist(pose1, goal) if pose1 is not None else None
    status_name = str(terminal.get('status') or 'UNKNOWN').upper()
    accepted = accepted or status_name == 'SUCCEEDED'
    passed = bool(accepted and status_name == 'SUCCEEDED' and active_commands
                  and drive_active and pose_change is not None
                  and pose_change > TRANSLATION_TOLERANCE_M)
    return {
        'passed': passed,
        'reason': None if passed else str(terminal.get('reason') or f'action_status={status_name}'),
        'accepted': accepted, 'status': status_name, 'goal': goal,
        'goal_reason': terminal.get('reason'), 'p0': pose0, 'p1': pose1,
        'displacement_m': pose_change, 'goal_distance_m': goal_distance,
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
        ws = websocket.create_connection(ws_url + '?token=' + token, timeout=0.02,
                                         enable_multithread=True)
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

        direct_results = {}
        for label, velocity in (
            ('FORWARD', (0.16, 0.0, 0.0)),
            ('STRAFE', (0.0, 0.16, 0.0)),
            ('ROTATE', (0.0, 0.0, 0.25)),
        ):
            motion = probe.direct_motion(label, velocity, timeout=args.motion_timeout)
            direct_results[label] = motion
            result['stages'][f'DIRECT_ROS_{label}'] = bool(motion['passed'])
            print(f'DIRECT_ROS_{label}={"PASS" if motion["passed"] else "FAIL reason=" + str(motion["reason"])} '
                  f'delta={motion.get("displacement_m_or_yaw_rad")}', flush=True)
        result['direct_ros'] = direct_results

        direct_navigation = probe.direct_navigation(args.navigation_timeout)
        result['direct_navigation'] = direct_navigation
        for stage_name, passed, detail in (
            ('DIRECT_NAV_SERVER_READY', bool(direct_navigation.get('server_ready')),
             'server=/navigate_to_pose'),
            ('DIRECT_NAV_GOAL_ACCEPTED', bool(direct_navigation.get('accepted')),
             f"status={direct_navigation.get('status', 'UNKNOWN')}"),
            ('DIRECT_NAV_CMD_VEL', direct_navigation.get('cmd_vel_nonzero_samples', 0) > 0,
             f"samples={direct_navigation.get('cmd_vel_nonzero_samples', 0)}"),
            ('DIRECT_NAV_CONTROLLER_COMMAND', bool(direct_navigation.get('drive_nonzero')),
             f"drive_samples={direct_navigation.get('drive_command_samples', 0)}"),
            ('DIRECT_NAV_MOTION', direct_navigation.get('displacement_m') is not None
             and direct_navigation['displacement_m'] > TRANSLATION_TOLERANCE_M,
             f"displacement={direct_navigation.get('displacement_m')}"),
            ('DIRECT_NAV_SUCCEEDED', direct_navigation.get('status') == 'SUCCEEDED',
             f"reason={direct_navigation.get('reason') or 'goal_succeeded'}"),
        ):
            result['stages'][stage_name] = bool(passed)
            print(f'{stage_name}={"PASS" if passed else "FAIL reason=" + str(direct_navigation.get("reason"))} {detail}', flush=True)

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
        print(f'ROS_BRIDGE_R01_WEBSOCKET_READY={"PASS" if bridge_ready else "FAIL reason=R01_not_connected_to_Django"}', flush=True)
        result['stages']['WEB_MAP_SYNC_READY'] = map_synced
        print(f'WEB_MAP_SYNC_READY={"PASS" if map_synced else "FAIL reason=" + str((probe.runtime_status or {}).get("map_sync_error") or "status_not_SYNCED")}', flush=True)

        mode_ok, mode_reason = set_mode(probe, ws, args.robot_id, 'MANUAL')
        result['stages']['WEB_MANUAL_MODE_R01'] = mode_ok
        print(f'WEB_MANUAL_MODE_R01={"PASS" if mode_ok else "FAIL reason=" + str(mode_reason)}', flush=True)
        manual = (web_manual(probe, ws, args.robot_id, 'FORWARD', timeout=args.motion_timeout)
                  if mode_ok else {'passed': False, 'reason': 'manual_mode_not_accepted'})
        result['web_manual'] = manual
        for stage_name, passed, detail in (
            ('WEB_MANUAL_CMD_VEL', manual['cmd_vel_nonzero_samples'] > 0,
             f'samples={manual["cmd_vel_nonzero_samples"]}'),
            ('WEB_MANUAL_CONTROLLER_COMMAND', manual.get('drive_nonzero', False),
             f'drive_samples={manual.get("drive_command_samples", 0)}'),
            ('WEB_MANUAL_COMMAND_ACCEPTED', manual.get('controller_command_ack', False),
             'R01_bridge_acknowledged_manual_command'),
            ('WEB_MANUAL_MOTION', manual['passed'],
             f'displacement={manual.get("displacement_m")}'),
            ('WEB_MANUAL_STOP_ZERO', manual.get('stop_zero_seen', False),
             'zero_cmd_vel_observed'),
        ):
            result['stages'][stage_name] = bool(passed)
            print(f'{stage_name}={"PASS" if passed else "FAIL reason=" + str(manual.get("reason"))} {detail}', flush=True)

        navigation = (web_navigation(probe, ws, args.robot_id, args.navigation_timeout)
                      if bridge_ready and map_synced else {
                          'passed': False,
                          'reason': 'R01_bridge_or_published_map_not_ready',
                      })
        result['web_navigation'] = navigation
        nav_status = str(navigation.get('status') or 'UNKNOWN')
        result['stages']['WEB_NAV_GOAL_ACCEPTED'] = bool(navigation.get('accepted'))
        print(f'WEB_NAV_GOAL_ACCEPTED={"PASS" if navigation.get("accepted") else "FAIL reason=" + str(navigation.get("reason"))}', flush=True)
        result['stages']['WEB_NAV_CMD_VEL'] = navigation.get('cmd_vel_nonzero_samples', 0) > 0
        print(f'WEB_NAV_CMD_VEL={"PASS" if result["stages"]["WEB_NAV_CMD_VEL"] else "FAIL reason=" + str(navigation.get("reason"))}', flush=True)
        result['stages']['WEB_NAV_CONTROLLER_COMMAND'] = bool(navigation.get('drive_nonzero'))
        print(f'WEB_NAV_CONTROLLER_COMMAND={"PASS" if result["stages"]["WEB_NAV_CONTROLLER_COMMAND"] else "FAIL reason=" + str(navigation.get("reason"))}', flush=True)
        result['stages']['WEB_NAV_MOTION'] = (
            navigation.get('displacement_m') is not None
            and navigation['displacement_m'] > TRANSLATION_TOLERANCE_M
        )
        print(f'WEB_NAV_MOTION={"PASS" if result["stages"]["WEB_NAV_MOTION"] else "FAIL reason=" + str(navigation.get("reason"))}', flush=True)
        result['stages']['WEB_NAV_SUCCEEDED'] = nav_status.upper() == 'SUCCEEDED'
        print(f'WEB_NAV_SUCCEEDED={"PASS" if result["stages"]["WEB_NAV_SUCCEEDED"] else "FAIL reason=" + str(navigation.get("goal_reason") or navigation.get("reason") or f"status={nav_status}")}', flush=True)

        all_direct = all(item['passed'] for item in direct_results.values())
        all_web_manual = all(result['stages'][key] for key in (
            'WEB_MANUAL_CMD_VEL', 'WEB_MANUAL_CONTROLLER_COMMAND',
            'WEB_MANUAL_COMMAND_ACCEPTED', 'WEB_MANUAL_MOTION', 'WEB_MANUAL_STOP_ZERO'))
        all_web_nav = all(result['stages'][key] for key in (
            'WEB_NAV_GOAL_ACCEPTED', 'WEB_NAV_CMD_VEL',
            'WEB_NAV_CONTROLLER_COMMAND', 'WEB_NAV_MOTION', 'WEB_NAV_SUCCEEDED'))
        all_direct_navigation = all(result['stages'][key] for key in (
            'DIRECT_NAV_SERVER_READY', 'DIRECT_NAV_GOAL_ACCEPTED', 'DIRECT_NAV_CMD_VEL',
            'DIRECT_NAV_CONTROLLER_COMMAND', 'DIRECT_NAV_MOTION', 'DIRECT_NAV_SUCCEEDED'))
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
