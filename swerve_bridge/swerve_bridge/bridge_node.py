from __future__ import annotations

import json
import math
import queue
import threading
from urllib.parse import quote
from datetime import datetime, timezone

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import JointState


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SwerveBridge(Node):
    def __init__(self):
        super().__init__('swerve_bridge')
        self.declare_parameter('robot_id', 'R01')
        self.declare_parameter('namespace', '')
        self.declare_parameter('django_ws_url', 'ws://127.0.0.1:8000/ws/ros')
        self.declare_parameter('django_token', '')
        self.declare_parameter('telemetry_rate', 10.0)
        self.declare_parameter('heartbeat_rate', 1.0)
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('navigate_server_timeout', 2.0)
        self.robot_id = str(self.get_parameter('robot_id').value)
        self.ws_url = str(self.get_parameter('django_ws_url').value)
        self.token = str(self.get_parameter('django_token').value)
        self.telemetry_period = 1.0 / max(0.1, float(self.get_parameter('telemetry_rate').value))
        self.latest_odom = None
        self.last_joint_state = None
        self.nav_state = 'IDLE'
        self.active_goal = None
        self.active_context = None
        self.goal_request_pending = False
        self.cancel_pending = False
        self.incoming = queue.Queue()
        self.ws = None
        self.ws_lock = threading.Lock()
        self.stop_event = threading.Event()

        self.create_subscription(Odometry, str(self.get_parameter('odom_topic').value), self.odom_cb, 20)
        self.create_subscription(JointState, str(self.get_parameter('joint_states_topic').value), self.joint_cb, 10)
        self.nav_client = ActionClient(self, NavigateToPose, str(self.get_parameter('navigate_action').value))
        self.create_timer(self.telemetry_period, self.telemetry_timer)
        self.create_timer(1.0 / max(0.1, float(self.get_parameter('heartbeat_rate').value)), self.heartbeat_timer)
        self.create_timer(0.05, self.process_commands)
        self.thread = threading.Thread(target=self.websocket_loop, daemon=True)
        self.thread.start()

    def odom_cb(self, msg):
        self.latest_odom = msg

    def joint_cb(self, msg):
        self.last_joint_state = msg

    def send(self, payload):
        with self.ws_lock:
            if self.ws is None:
                return False
            try:
                self.ws.send(json.dumps(payload))
                return True
            except Exception as exc:
                self.get_logger().warning(f'ROS bridge send failed: {exc}')
                return False

    def websocket_loop(self):
        try:
            import websocket
        except ImportError:
            self.get_logger().error('websocket-client is required for Django bridge')
            return
        while not self.stop_event.is_set():
            try:
                separator = '&' if '?' in self.ws_url else '?'
                url = f'{self.ws_url}{separator}token={quote(self.token)}'
                ws = websocket.create_connection(url, timeout=2, subprotocols=['json'])
                with self.ws_lock:
                    self.ws = ws
                while not self.stop_event.is_set():
                    try:
                        raw = ws.recv()
                        if raw:
                            self.incoming.put(json.loads(raw))
                    except websocket.WebSocketTimeoutException:
                        continue
                ws.close()
            except Exception as exc:
                self.get_logger().warning(f'Django websocket disconnected: {exc}')
                with self.ws_lock:
                    self.ws = None
                self.stop_event.wait(2.0)

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
                   'timestamp': self.now()})

    def heartbeat_timer(self):
        self.send({'type': 'HEARTBEAT', 'robot_id': self.robot_id,
                   'nav2_state': self.nav_state, 'timestamp': self.now()})

    def process_commands(self):
        while True:
            try:
                data = self.incoming.get_nowait()
            except queue.Empty:
                return
            kind = str(data.get('type', '')).upper()
            if kind == 'NAV_GOAL':
                self.navigate(data)
            elif kind == 'CANCEL_NAVIGATION':
                self.cancel_navigation(data)

    def navigate(self, data):
        context = {
            'schedule_id': data.get('schedule_id'),
            'stop_id': data.get('stop_id'),
            'robot_id': self.robot_id,
        }
        if self.active_goal is not None or self.goal_request_pending:
            self.send_nav_status(context, 'FAILED', 'another Nav2 goal is already active')
            return
        timeout = float(self.get_parameter('navigate_server_timeout').value)
        if not self.nav_client.wait_for_server(timeout_sec=timeout):
            self.nav_state = 'FAILED'
            self.send_nav_status(
                context, 'FAILED',
                f'Nav2 action server unavailable after {timeout:.1f}s',
            )
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = str(data.get('frame_id') or 'map')
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(data.get('x', 0.0))
        goal.pose.pose.position.y = float(data.get('y', 0.0))
        yaw = float(data.get('yaw', 0.0))
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
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
            self.send_nav_status(context, 'FAILED', f'Nav2 goal request failed: {exc}')
            return
        if handle is None or not handle.accepted:
            self.nav_state = 'FAILED'
            self.send_nav_status(context, 'FAILED', 'Nav2 rejected the goal')
            return
        self.active_goal = handle
        self.active_context = context
        self.cancel_pending = False
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
        self.nav_state = 'IDLE' if status == 'SUCCEEDED' else status
        if self.active_goal is handle:
            self.active_goal = None
            self.active_context = None
            self.cancel_pending = False
        reason = None if status != 'FAILED' else f'Nav2 finished with status code {status_code}'
        self.send_nav_status(context, status, reason)

    def cancel_navigation(self, data):
        context = self.active_context or {
            'robot_id': self.robot_id,
            'schedule_id': data.get('schedule_id'),
            'stop_id': data.get('stop_id'),
        }
        if self.active_goal is None:
            self.send_nav_status(context, 'FAILED', 'no accepted Nav2 goal to cancel')
            return
        if self.cancel_pending:
            return
        self.cancel_pending = True
        try:
            future = self.active_goal.cancel_goal_async()
            future.add_done_callback(lambda f: self.cancel_response(f, context))
        except Exception as exc:
            self.cancel_pending = False
            self.send_nav_status(context, 'FAILED', f'failed to request Nav2 cancellation: {exc}')

    def cancel_response(self, future, context):
        try:
            response = future.result()
            accepted = bool(response.goals_canceling)
        except Exception as exc:
            self.cancel_pending = False
            self.send_nav_status(context, 'FAILED', f'Nav2 cancellation request failed: {exc}')
            return
        if not accepted:
            self.cancel_pending = False
            self.send_nav_status(context, 'FAILED', 'Nav2 rejected the cancellation request')
        # A successful request is not completion. goal_result() reports
        # CANCELED only after Nav2 returns STATUS_CANCELED.

    def destroy_node(self):
        self.stop_event.set()
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
        rclpy.shutdown()
