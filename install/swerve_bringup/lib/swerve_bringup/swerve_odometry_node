#!/usr/bin/env python3
"""Encoder-based odometry for the two-module swerve base."""

import math
from typing import Dict, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def solve_3x3(a, b):
    """Small Gaussian-elimination solver; avoids a numpy dependency."""
    m = [list(row) + [value] for row, value in zip(a, b)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(m[row][col]))
        if abs(m[pivot][col]) < 1e-12:
            return (0.0, 0.0, 0.0)
        m[col], m[pivot] = m[pivot], m[col]
        divisor = m[col][col]
        m[col] = [value / divisor for value in m[col]]
        for row in range(3):
            if row == col:
                continue
            factor = m[row][col]
            m[row] = [m[row][i] - factor * m[col][i] for i in range(4)]
    return tuple(m[i][3] for i in range(3))


class SwerveOdometry(Node):
    def __init__(self) -> None:
        super().__init__('swerve_odometry')
        self.declare_parameter('wheel_radius', 0.0675)
        self.declare_parameter('wheel_velocity_sign', 1.0)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('joint_state_timeout', 0.3)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('modules.front.x', 0.300042)
        self.declare_parameter('modules.front.y', 0.049988)
        self.declare_parameter('modules.rear.x', -0.300042)
        self.declare_parameter('modules.rear.y', 0.050013)

        self.radius = float(self.get_parameter('wheel_radius').value)
        self.velocity_sign = float(self.get_parameter('wheel_velocity_sign').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.joint_state_timeout = float(
            self.get_parameter('joint_state_timeout').value)
        self.modules = ('front', 'rear')
        self.xy = {
            'front': (float(self.get_parameter('modules.front.x').value),
                      float(self.get_parameter('modules.front.y').value)),
            'rear': (float(self.get_parameter('modules.rear.x').value),
                     float(self.get_parameter('modules.rear.y').value)),
        }
        self.steer_names = {'front': 'steer_front_joint', 'rear': 'steer_rear_joint'}
        self.wheel_names = {
            'front': 'wheel_front_drive_joint',
            'rear': 'wheel_rear_drive_joint',
        }
        self.steer = {'front': 0.0, 'rear': 0.0}
        self.wheel_velocity = {'front': 0.0, 'rear': 0.0}
        self.previous_wheel_position: Dict[str, Optional[float]] = {
            'front': None, 'rear': None}
        self.previous_state_time: Optional[float] = None
        self.last_joint_state_time = None
        self.body_velocity = (0.0, 0.0, 0.0)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_update = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 20)
        rate = float(self.get_parameter('publish_rate').value)
        self.timer = self.create_timer(1.0 / rate, self.update)
        self.get_logger().info(
            f'Encoder odometry ready: radius={self.radius:.4f} m, '
            f'velocity_sign={self.velocity_sign:+.1f}')

    def joint_state_callback(self, msg: JointState) -> None:
        self.last_joint_state_time = self.get_clock().now()
        values = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        state_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        state_now = state_stamp if state_stamp > 0.0 else (
            self.get_clock().now().nanoseconds * 1e-9)
        for module in self.modules:
            steer_name = self.steer_names[module]
            wheel_name = self.wheel_names[module]
            if steer_name in values and math.isfinite(values[steer_name]):
                self.steer[module] = values[steer_name]

            if wheel_name in velocities and math.isfinite(velocities[wheel_name]):
                self.wheel_velocity[module] = velocities[wheel_name]
            elif wheel_name in values and math.isfinite(values[wheel_name]):
                old = self.previous_wheel_position[module]
                if old is not None and self.previous_state_time is not None:
                    dt = state_now - self.previous_state_time
                    if dt > 1e-6:
                        self.wheel_velocity[module] = (values[wheel_name] - old) / dt
                self.previous_wheel_position[module] = values[wheel_name]
        if any(name in values for name in self.wheel_names.values()):
            self.previous_state_time = state_now

        self.body_velocity = self.compute_body_velocity()

    def compute_body_velocity(self):
        # Each module contributes:
        #   wheel_v*cos(angle) = vx - wz*y
        #   wheel_v*sin(angle) = vy + wz*x
        rows = []
        rhs = []
        for module in self.modules:
            x, y = self.xy[module]
            speed = self.velocity_sign * self.radius * self.wheel_velocity[module]
            angle = self.steer[module]
            rows.extend(((1.0, 0.0, -y), (0.0, 1.0, x)))
            rhs.extend((speed * math.cos(angle), speed * math.sin(angle)))

        normal = [[0.0] * 3 for _ in range(3)]
        normal_rhs = [0.0] * 3
        for row, value in zip(rows, rhs):
            for i in range(3):
                normal_rhs[i] += row[i] * value
                for j in range(3):
                    normal[i][j] += row[i] * row[j]
        return solve_3x3(normal, normal_rhs)

    def update(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds * 1e-9
        self.last_update = now
        if dt <= 0.0 or dt > 0.25:
            dt = 0.0

        stale = (
            self.last_joint_state_time is None or
            (now - self.last_joint_state_time).nanoseconds * 1e-9 > self.joint_state_timeout
        )
        if stale:
            # Never integrate the final encoder velocity forever when the
            # controller/Gazebo joint-state stream disappears.
            self.body_velocity = (0.0, 0.0, 0.0)

        vx, vy, wz = self.body_velocity
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (c * vx - s * vy) * dt
        self.y += (s * vx + c * vy) * dt
        self.yaw += wz * dt
        self.publish(now)

    def publish(self, stamp) -> None:
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)
        vx, vy, wz = self.body_velocity

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        # x, y, yaw are the measured planar quantities; remaining dimensions
        # are intentionally high-uncertainty because this node is 2D odometry.
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[7] = 0.05
        odom.twist.covariance[35] = 0.1
        odom.twist.covariance[14] = 1e6
        odom.twist.covariance[21] = 1e6
        odom.twist.covariance[28] = 1e6
        self.odom_pub.publish(odom)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SwerveOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
