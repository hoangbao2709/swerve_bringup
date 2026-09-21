#!/usr/bin/env python3
"""Execute GoToTag strictly one approved DataMatrix corridor at a time."""
from __future__ import annotations

import math
import os
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Vector3Stamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Int32, String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml
from ament_index_python.packages import get_package_share_directory

from swerve_bringup.action import GoToTag
from tag_navigation_core import RouteExecutor, RouteState, TagGraph


def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class TagRoutePlanner(Node):
    def __init__(self):
        super().__init__('tag_route_planner')
        self.group = ReentrantCallbackGroup()
        defaults = {
            'tag_graph_file': 'tag_graph.yaml', 'map_frame': 'map', 'base_frame': 'base_footprint',
            'navigate_action': '/navigate_to_pose', 'tag_id_topic': '/v30e/tag_id',
            'offset_topic': '/v30e/offset', 'cmd_vel_topic': '/cmd_vel', 'approach_radius': .20,
            'approach_speed': .05, 'tag_acquire_timeout': 5.0, 'max_approach_distance': .30,
            'tag_capture_x_tol': .10, 'tag_capture_y_tol': .10, 'detection_max_age': .5,
            'offset_max_age': .5, 'replan_on_wrong_tag': True,
        }
        for name, value in defaults.items(): self.declare_parameter(name, value)
        graph_path = self.get_parameter('tag_graph_file').value
        if not os.path.isabs(graph_path):
            graph_path = os.path.join(get_package_share_directory('swerve_bringup'), 'config', graph_path)
        with open(graph_path, encoding='utf-8') as stream:
            self.graph = TagGraph(yaml.safe_load(stream)['tags'])
        self.executor = RouteExecutor(self.graph)
        self.lock = threading.RLock(); self.active = None
        self.last_tag = None; self.last_tag_monotonic = 0.0
        self.last_offset = None; self.last_offset_monotonic = 0.0
        self.last_nav_distance = None; self.segment_start = None
        self.tf = Buffer(); self.tf_listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, self.get_parameter('navigate_action').value,
                                callback_group=self.group)
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        # Path is rendered by RViz as the approved tag-to-tag corridor polyline.
        from nav_msgs.msg import Path
        self.route_pub = self.create_publisher(Path, '/tag_navigation/route', 1)
        self.current_pub = self.create_publisher(Int32, '/tag_navigation/current_tag', 10)
        self.next_pub = self.create_publisher(Int32, '/tag_navigation/next_tag', 10)
        self.target_pub = self.create_publisher(Int32, '/tag_navigation/target_tag', 10)
        self.state_pub = self.create_publisher(String, '/tag_navigation/state', 10)
        self.measurement_pub = self.create_publisher(PoseWithCovarianceStamped, '/tag_navigation/global_measurement', 10)
        self.create_subscription(Int32, self.get_parameter('tag_id_topic').value, self.tag_cb, 20,
                                 callback_group=self.group)
        self.create_subscription(Vector3Stamped, self.get_parameter('offset_topic').value, self.offset_cb, 20,
                                 callback_group=self.group)
        self.server = ActionServer(self, GoToTag, '/go_to_tag', self.execute, goal_callback=self.goal_cb,
                                   cancel_callback=self.cancel_cb, callback_group=self.group)
        self.create_timer(.05, self.tick, callback_group=self.group)
        self.publish_status()

    def now(self): return time.monotonic()
    def p(self, name): return self.get_parameter(name).value

    def offset_ok(self):
        return self.last_offset is not None and self.now() - self.last_offset_monotonic <= float(self.p('offset_max_age')) and \
            abs(self.last_offset.x) <= float(self.p('tag_capture_x_tol')) and abs(self.last_offset.y) <= float(self.p('tag_capture_y_tol'))

    def offset_cb(self, msg):
        with self.lock:
            self.last_offset, self.last_offset_monotonic = msg.vector, self.now()
            self.anchor_if_valid()
            self.publish_global_measurement_if_possible()

    def tag_cb(self, msg):
        with self.lock:
            self.last_tag, self.last_tag_monotonic = int(msg.data), self.now()
            self.anchor_if_valid()
            self.publish_global_measurement_if_possible()

    def anchor_if_valid(self):
        """Boot remains UNANCHORED until one tag and its in-tolerance offset agree."""
        if self.executor.current_tag is None and self.last_tag in self.graph.tags and self.offset_ok():
            self.executor.anchor(self.last_tag)

    def publish_global_measurement_if_possible(self):
        if self.last_tag not in self.graph.tags or self.last_offset is None: return
        marker = self.graph.tags[self.last_tag]; c, s = math.cos(marker['yaw']), math.sin(marker['yaw'])
        msg = PoseWithCovarianceStamped(); msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = self.p('map_frame')
        msg.pose.pose.position.x = marker['x'] + c * self.last_offset.x - s * self.last_offset.y
        msg.pose.pose.position.y = marker['y'] + s * self.last_offset.x + c * self.last_offset.y
        yaw = marker['yaw'] + self.last_offset.z
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = .0025; msg.pose.covariance[35] = math.radians(1) ** 2
        self.measurement_pub.publish(msg)

    def goal_cb(self, goal):
        if goal.target_tag_id not in self.graph.tags: return GoalResponse.REJECT
        with self.lock:
            return GoalResponse.ACCEPT if self.active is None else GoalResponse.REJECT

    def cancel_cb(self, _): return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        with self.lock:
            if self.executor.current_tag is None:
                goal_handle.abort(); result = GoToTag.Result(); result.success = False; result.reason = 'UNANCHORED: wait for a valid V30E tag'
                return result
            try: self.executor.start(goal_handle.request.target_tag_id)
            except (ValueError, RuntimeError) as error:
                goal_handle.abort(); result = GoToTag.Result(); result.success = False; result.reason = str(error); return result
            self.active = {'handle': goal_handle, 'done': threading.Event(), 'success': False, 'reason': ''}
            self.publish_route(); self.publish_status()
            if self.executor.state == RouteState.ARRIVED:
                self.finish(True, 'target tag is already anchored and confirmed')
            else:
                self.start_segment()
        while rclpy.ok() and not self.active['done'].wait(.05):
            if goal_handle.is_cancel_requested: self.finish(False, 'cancelled', canceled=True)
        result = GoToTag.Result(); result.success = self.active['success']; result.reason = self.active['reason']
        if result.success: goal_handle.succeed()
        elif goal_handle.is_cancel_requested: goal_handle.canceled()
        else: goal_handle.abort()
        with self.lock: self.active = None
        return result

    def tick(self):
        with self.lock:
            if self.active is None: self.publish_status(); return
            expected = self.executor.next_tag
            fresh_detection = self.last_tag_monotonic >= (self.segment_start or float('inf')) and self.now() - self.last_tag_monotonic <= float(self.p('detection_max_age'))
            if expected is not None and self.last_tag == expected and fresh_detection:
                outcome = self.executor.expected_seen(expected, self.offset_ok())
                if outcome == 'advanced':
                    self.cancel_segment(); self.publish_route(); self.publish_status()
                    if self.executor.state == RouteState.ARRIVED: self.finish(True, 'target tag detected and offset is within tolerance')
                    else: self.start_segment()
                    return
            elif expected is not None and self.last_tag in self.graph.tags and self.last_tag != expected and fresh_detection:
                detected = self.last_tag; self.get_logger().warning(f'EXPECTED_TAG={expected} DETECTED_TAG={detected}')
                self.executor.state = RouteState.WRONG_TAG; self.publish_status()
                if bool(self.p('replan_on_wrong_tag')):
                    try:
                        self.cancel_segment(); self.executor.replan_from(detected); self.get_logger().warning('Explicit replan from wrong detected tag')
                        self.publish_route(); self.start_segment()
                    except ValueError as error: self.finish(False, f'WRONG_TAG and no replan: {error}')
                else: self.finish(False, f'EXPECTED_TAG={expected} DETECTED_TAG={detected}')
                return
            if self.executor.state == RouteState.APPROACH_TAG:
                elapsed = self.now() - self.segment_start
                if elapsed > float(self.p('tag_acquire_timeout')) or self.approach_distance() > float(self.p('max_approach_distance')):
                    self.stop(); self.executor.state = RouteState.TAG_ACQUIRE_FAILED; self.publish_status(); self.finish(False, 'TAG_ACQUIRE_FAILED: expected tag was not detected')
                else: self.approach_along_segment()
            elif self.distance_to_expected() is not None and self.distance_to_expected() <= float(self.p('approach_radius')):
                self.executor.state = RouteState.APPROACH_TAG; self.segment_start = self.now(); self.cancel_segment(); self.publish_status()

    def start_segment(self):
        expected = self.executor.next_tag
        if expected is None: return
        self.executor.state = RouteState.NAVIGATING_SEGMENT; self.segment_start = self.now(); self.last_nav_distance = None
        if not self.nav.wait_for_server(timeout_sec=2.0): self.finish(False, 'NavigateToPose server unavailable'); return
        tag = self.graph.tags[expected]; goal = NavigateToPose.Goal(); goal.pose.header.frame_id = self.p('map_frame'); goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = tag['x'], tag['y']
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(tag['yaw']/2), math.cos(tag['yaw']/2)
        future = self.nav.send_goal_async(goal, feedback_callback=lambda f: setattr(self, 'last_nav_distance', float(f.feedback.distance_remaining)))
        future.add_done_callback(self.nav_response)

    def nav_response(self, future):
        try: handle = future.result()
        except Exception as error: self.finish(False, f'Nav2 send failed: {error}'); return
        if not handle.accepted: self.finish(False, 'Nav2 rejected segment'); return
        self.active['nav_handle'] = handle
        handle.get_result_async().add_done_callback(self.nav_result)

    def nav_result(self, future):
        try: status = future.result().status
        except Exception as error: self.finish(False, f'Nav2 segment error: {error}'); return
        if self.active is None or self.executor.state == RouteState.APPROACH_TAG: return
        # Nav2 success only means acquisition zone reached; detector remains the gate.
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.executor.state = RouteState.APPROACH_TAG; self.segment_start = self.now(); self.publish_status()
        elif status != GoalStatus.STATUS_CANCELED: self.finish(False, f'Nav2 segment status {status}')

    def cancel_segment(self):
        handle = self.active.get('nav_handle') if self.active else None
        if handle is not None: handle.cancel_goal_async()

    def pose(self):
        try:
            transform = self.tf.lookup_transform(self.p('map_frame'), self.p('base_frame'), Time())
            t = transform.transform.translation; return t.x, t.y, yaw_from_quaternion(transform.transform.rotation)
        except TransformException: return None

    def distance_to_expected(self):
        pose = self.pose(); expected = self.executor.next_tag
        if pose is None or expected is None: return self.last_nav_distance
        tag = self.graph.tags[expected]; return math.hypot(tag['x'] - pose[0], tag['y'] - pose[1])

    def approach_distance(self):
        return abs(self.p('approach_radius') - (self.distance_to_expected() or 0.0))

    def approach_along_segment(self):
        pose = self.pose(); expected = self.executor.next_tag
        if pose is None or expected is None: return
        tag = self.graph.tags[expected]; dx, dy = tag['x'] - pose[0], tag['y'] - pose[1]; length = math.hypot(dx, dy)
        if length < .001: return
        # /cmd_vel is base-frame; rotate the map corridor vector once, no search spin.
        c, s = math.cos(pose[2]), math.sin(pose[2]); out = Twist(); speed = float(self.p('approach_speed'))
        out.linear.x = speed * (c * dx + s * dy) / length; out.linear.y = speed * (-s * dx + c * dy) / length
        self.cmd_pub.publish(out)

    def stop(self): self.cmd_pub.publish(Twist())
    def finish(self, success, reason, canceled=False):
        if self.active is None or self.active['done'].is_set(): return
        self.stop(); self.cancel_segment(); self.active['success'], self.active['reason'] = success, reason; self.active['done'].set()

    def publish_route(self):
        from nav_msgs.msg import Path
        path = Path(); path.header.frame_id = self.p('map_frame'); path.header.stamp = self.get_clock().now().to_msg()
        for tag_id in self.executor.route:
            tag = self.graph.tags[tag_id]; pose = PoseStamped(); pose.header = path.header; pose.pose.position.x, pose.pose.position.y = tag['x'], tag['y']; pose.pose.orientation.w = 1.0; path.poses.append(pose)
        self.route_pub.publish(path)

    def publish_status(self):
        def emit(pub, value): msg = Int32(); msg.data = int(value or 0); pub.publish(msg)
        emit(self.current_pub, self.executor.current_tag); emit(self.next_pub, self.executor.next_tag); emit(self.target_pub, self.executor.target_tag)
        state = String(); state.data = self.executor.state; self.state_pub.publish(state)
        if self.active:
            feedback = GoToTag.Feedback(); feedback.current_tag_id = self.executor.current_tag or 0; feedback.next_tag_id = self.executor.next_tag or 0; feedback.target_tag_id = self.executor.target_tag or 0; feedback.state = self.executor.state
            feedback.progress = (self.executor.index / max(1, len(self.executor.route) - 1)); self.active['handle'].publish_feedback(feedback)


def main(args=None):
    rclpy.init(args=args); node = TagRoutePlanner(); executor = MultiThreadedExecutor(num_threads=4); executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        executor.shutdown(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
