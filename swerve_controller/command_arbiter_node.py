#!/usr/bin/env python3
"""Select one safe velocity source before the swerve controller."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from command_ownership import choose_command


class CommandArbiter(Node):
    def __init__(self) -> None:
        super().__init__('command_arbiter')
        self.declare_parameter('direct_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('manual_cmd_vel_topic', '/cmd_vel_manual')
        self.declare_parameter('nav_cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('tag_cmd_vel_topic', '/cmd_vel_tag')
        self.declare_parameter('control_mode_topic', '/robot_control_mode')
        self.declare_parameter('tag_route_state_topic', '/tag_navigation/state')
        self.declare_parameter('emergency_stop_topic', '/emergency_stop')
        self.declare_parameter('selected_cmd_vel_topic', '/cmd_vel_selected')
        self.declare_parameter('command_owner_topic', '/command_owner')
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('control_rate', 50.0)

        self.command_timeout = max(0.05, float(self.get_parameter('command_timeout').value))
        control_rate = max(1.0, float(self.get_parameter('control_rate').value))
        self.control_mode = 'AUTONOMOUS'
        self.tag_route_state = 'IDLE'
        self.emergency_stop = False
        self.sources = {
            owner: None for owner in ('WEB_MANUAL', 'DIRECT_MANUAL', 'NAV2', 'TAG_ROUTE')
        }

        self.selected_pub = self.create_publisher(
            Twist, str(self.get_parameter('selected_cmd_vel_topic').value), 10)
        self.owner_pub = self.create_publisher(
            String, str(self.get_parameter('command_owner_topic').value), 10)
        source_topics = (
            ('DIRECT_MANUAL', 'direct_cmd_vel_topic'),
            ('WEB_MANUAL', 'manual_cmd_vel_topic'),
            ('NAV2', 'nav_cmd_vel_topic'),
            ('TAG_ROUTE', 'tag_cmd_vel_topic'),
        )
        for owner, parameter in source_topics:
            self.create_subscription(
                Twist, str(self.get_parameter(parameter).value),
                lambda msg, selected_owner=owner: self.command_callback(selected_owner, msg),
                10,
            )

        retained_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, str(self.get_parameter('control_mode_topic').value),
            self.control_mode_callback, retained_qos)
        self.create_subscription(
            Bool, str(self.get_parameter('emergency_stop_topic').value),
            self.emergency_stop_callback, retained_qos)
        self.create_subscription(
            String, str(self.get_parameter('tag_route_state_topic').value),
            self.tag_route_state_callback, 10)
        self.timer = self.create_timer(1.0 / control_rate, self.publish_selection)

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def clear_sources(self) -> None:
        for owner in self.sources:
            self.sources[owner] = None

    def command_callback(self, owner: str, msg: Twist) -> None:
        values = (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning(f'dropping non-finite {owner} velocity command')
            return
        self.sources[owner] = (values, self.now_seconds())

    def control_mode_callback(self, msg: String) -> None:
        mode = str(msg.data or '').upper()
        if mode not in ('MANUAL', 'AUTONOMOUS'):
            self.get_logger().warning(f'ignoring unsupported control mode {mode!r}')
            return
        if mode != self.control_mode:
            self.clear_sources()
            self.control_mode = mode
            self.get_logger().info(f'velocity ownership changed to {mode}')

    def tag_route_state_callback(self, msg: String) -> None:
        state = str(msg.data or '').upper()
        if state != self.tag_route_state:
            if state == 'APPROACH_TAG' or self.tag_route_state == 'APPROACH_TAG':
                self.sources['NAV2'] = None
                self.sources['TAG_ROUTE'] = None
            self.tag_route_state = state

    def emergency_stop_callback(self, msg: Bool) -> None:
        self.emergency_stop = bool(msg.data)
        if self.emergency_stop:
            self.clear_sources()
            self.get_logger().error('E-STOP active; all velocity sources are blocked')

    def publish_selection(self) -> None:
        owner, values = choose_command(
            now=self.now_seconds(),
            timeout=self.command_timeout,
            mode=self.control_mode,
            sources=self.sources,
            tag_route_state=self.tag_route_state,
            emergency_stop=self.emergency_stop,
        )
        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = values
        self.selected_pub.publish(command)
        owner_message = String()
        owner_message.data = owner
        self.owner_pub.publish(owner_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.selected_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
