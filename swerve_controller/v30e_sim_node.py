#!/usr/bin/env python3
"""Functional V30E/DataMatrix simulator.

The simulated reader observes the robot pose and a fixed DataMatrix landmark,
computes the same relative measurement a downward-facing V30E reports, then
reconstructs an absolute map pose from (TagID, marker map pose, offset).
It never publishes the robot pose as a Gazebo ground-truth pose directly.
"""
import math
import os
import signal
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped
from gazebo_msgs.srv import GetEntityState
from rclpy.node import Node
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray
import yaml


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def relative_measurement(robot, marker):
    """Return V30E AgvX, AgvY, AgvAngle in marker axes."""
    dx = robot[0] - marker['x']
    dy = robot[1] - marker['y']
    c, s = math.cos(marker['yaw']), math.sin(marker['yaw'])
    return (c * dx + s * dy, -s * dx + c * dy,
            wrap(robot[2] - marker['yaw']))


def absolute_pose(marker, offset):
    """Reconstruct map pose from the marker and V30E measurement."""
    c, s = math.cos(marker['yaw']), math.sin(marker['yaw'])
    x = marker['x'] + c * offset[0] - s * offset[1]
    y = marker['y'] + s * offset[0] + c * offset[1]
    return x, y, wrap(marker['yaw'] + offset[2])


class V30ESim(Node):
    def __init__(self):
        super().__init__('v30e_sim_node')
        default_map = os.path.join(os.path.dirname(__file__), '..', 'config', 'datamatrix_map.yaml')
        self.declare_parameter('marker_map', default_map)
        self.declare_parameter('get_entity_state_service', '/get_entity_state')
        self.declare_parameter('model_name', 'swerve_base')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', 'v30e_link')
        self.declare_parameter('marker_size', 0.20)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('working_height', 0.10)
        self.declare_parameter('working_height_tolerance', 0.02)
        self.declare_parameter('ground_z', 0.0)
        self.declare_parameter('camera_height_from_base', 0.10)
        self.declare_parameter('camera_x', 0.0)
        self.declare_parameter('camera_y', 0.0)
        self.declare_parameter('camera_yaw', 0.0)
        self.declare_parameter('horizontal_fov', 1.04719755)
        self.declare_parameter('vertical_fov', 1.04719755)
        self.declare_parameter('offset_angle_in_degrees', False)
        path = self.get_parameter('marker_map').value
        if not os.path.isabs(path):
            path = os.path.join(os.getcwd(), path)
        with open(path, encoding='utf-8') as f:
            self.markers = yaml.safe_load(f)['markers']
        self.last_ground_truth = None
        self.next_state_request_monotonic = 0.0
        self.state_client = self.create_client(
            GetEntityState, self.get_parameter('get_entity_state_service').value)
        self.state_request_pending = False
        self.tag_pub = self.create_publisher(Int32, '/v30e/tag_id', 10)
        self.offset_pub = self.create_publisher(Vector3Stamped, '/v30e/offset', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/v30e/pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/v30e/markers', 1)
        self.timer = self.create_timer(1.0 / float(self.get_parameter('publish_rate').value), self.tick)
        self.publish_markers()
        self.get_logger().info('Using Gazebo /get_entity_state internally; odometry is not a V30E measurement input')

    def publish_markers(self):
        arr = MarkerArray()
        for i, marker in enumerate(self.markers):
            m = Marker(); m.header.frame_id = self.get_parameter('map_frame').value
            m.ns = 'v30e_datamatrix'; m.id = i; m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y = float(marker['x']), float(marker['y'])
            m.pose.position.z = 0.012; m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = float(self.get_parameter('marker_size').value); m.scale.z = 0.008
            m.color.r, m.color.g, m.color.b, m.color.a = 0.02, 0.02, 0.02, 0.95
            m.lifetime.sec = 0
            arr.markers.append(m)
        self.marker_pub.publish(arr)

    def entity_state_done(self, future):
        self.state_request_pending = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn('V30E state request failed: %s' % exc, throttle_duration_sec=2.0)
            return
        if not response.success:
            self.get_logger().warn('get_entity_state failed: %s' %
                                   getattr(response, 'status_message', 'entity not available'),
                                   throttle_duration_sec=2.0)
            return
        pose = response.state.pose
        p = pose.position
        self.last_ground_truth = (p.x, p.y, p.z, yaw_from_quaternion(pose.orientation))
        self.publish_detection()

    def camera_pose_and_height(self):
        x, y, root_z, yaw = self.last_ground_truth
        cx = float(self.get_parameter('camera_x').value)
        cy = float(self.get_parameter('camera_y').value)
        c, s = math.cos(yaw), math.sin(yaw)
        camera = (x + c * cx - s * cy, y + s * cx + c * cy,
                  wrap(yaw + float(self.get_parameter('camera_yaw').value)))
        height = root_z + float(self.get_parameter('camera_height_from_base').value)
        return camera, height

    def marker_in_fov(self, camera, height, marker):
        expected = float(self.get_parameter('working_height').value)
        tolerance = float(self.get_parameter('working_height_tolerance').value)
        ground_z = float(self.get_parameter('ground_z').value)
        if abs(height - ground_z - expected) > tolerance:
            return False
        half_x = max(0.0, height - ground_z) * math.tan(float(self.get_parameter('horizontal_fov').value) / 2.0)
        half_y = max(0.0, height - ground_z) * math.tan(float(self.get_parameter('vertical_fov').value) / 2.0)
        half_marker = float(self.get_parameter('marker_size').value) / 2.0
        dx, dy = marker['x'] - camera[0], marker['y'] - camera[1]
        c, s = math.cos(camera[2]), math.sin(camera[2])
        camera_x, camera_y = c * dx + s * dy, -s * dx + c * dy
        return abs(camera_x) <= half_x + half_marker and abs(camera_y) <= half_y + half_marker

    def tick(self):
        now = time.monotonic()
        if (self.state_request_pending or not self.state_client.service_is_ready()
                or now < self.next_state_request_monotonic):
            return
        # The world plugin reports a failed request while the robot is still
        # being inserted. Avoid hammering Gazebo's world lock during startup;
        # once spawned, poll at the configured sensor rate.
        request_period = (1.0 / float(self.get_parameter('publish_rate').value)
                         if self.last_ground_truth is not None else 1.0)
        self.next_state_request_monotonic = now + request_period
        request = GetEntityState.Request()
        request.name = self.get_parameter('model_name').value
        request.reference_frame = 'world'
        self.state_request_pending = True
        future = self.state_client.call_async(request)
        future.add_done_callback(self.entity_state_done)

    def publish_detection(self):
        camera, height = self.camera_pose_and_height()
        candidates = []
        for m in self.markers:
            if self.marker_in_fov(camera, height, m):
                dx, dy = camera[0] - m['x'], camera[1] - m['y']
                candidates.append((math.hypot(dx, dy), m))
        if not candidates:
            return
        _, marker = min(candidates, key=lambda item: item[0])
        # Ground truth is used only to synthesize this sensor observation.
        # It is never published as the V30E pose.
        offset = relative_measurement(camera, marker)
        if self.get_parameter('offset_angle_in_degrees').value:
            offset = (offset[0], offset[1], math.degrees(offset[2]))
        tag = Int32(); tag.data = int(marker['tag_id']); self.tag_pub.publish(tag)
        out = Vector3Stamped(); out.header.stamp = self.get_clock().now().to_msg(); out.header.frame_id = self.get_parameter('camera_frame').value
        out.vector.x, out.vector.y, out.vector.z = offset; self.offset_pub.publish(out)
        angle = math.radians(offset[2]) if self.get_parameter('offset_angle_in_degrees').value else offset[2]
        camera_x, camera_y, camera_yaw = absolute_pose(marker, (offset[0], offset[1], angle))
        robot_yaw = wrap(camera_yaw - float(self.get_parameter('camera_yaw').value))
        c, s = math.cos(robot_yaw), math.sin(robot_yaw)
        cx = float(self.get_parameter('camera_x').value)
        cy = float(self.get_parameter('camera_y').value)
        x, y, yaw = camera_x - c * cx + s * cy, camera_y - s * cx - c * cy, robot_yaw
        pose = PoseWithCovarianceStamped(); pose.header.stamp = out.header.stamp; pose.header.frame_id = self.get_parameter('map_frame').value
        pose.pose.pose.position.x, pose.pose.pose.position.y = x, y
        pose.pose.pose.orientation.z, pose.pose.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        pose.pose.covariance[0] = 0.0025; pose.pose.covariance[7] = 0.0025; pose.pose.covariance[35] = math.radians(1.0) ** 2
        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args); node = V30ESim()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
