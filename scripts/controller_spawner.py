#!/usr/bin/env python3
"""Serial Humble controller startup with bounded, persistent service clients.

Humble controller-manager 2.40's spawner can wait forever when Fast DDS loses
the first list_controllers response during endpoint discovery. Retry only
read-only queries. Never repeat a mutation whose acknowledgement was lost:
reconcile its resulting state instead, or fail the launch dependency chain.
"""
from __future__ import annotations

import argparse
import time

import rclpy
from controller_manager_msgs.srv import (
    ConfigureController, ListControllers, LoadController, SwitchController,
)
from rclpy.node import Node


ORDER = ('joint_state_broadcaster', 'steering_controller', 'drive_controller')
CLAIMS = {
    'joint_state_broadcaster': set(),
    'steering_controller': {'steer_front_joint/position', 'steer_rear_joint/position'},
    'drive_controller': {'wheel_front_drive_joint/velocity', 'wheel_rear_drive_joint/velocity'},
}


def validate_predecessors(controller, rows):
    states = {row.name: row.state for row in rows}
    for name in ORDER[:ORDER.index(controller)]:
        if states.get(name) != 'active':
            raise RuntimeError(f'predecessor {name} must be active, got {states.get(name, "missing")}')


def validate_active(controller, rows):
    validate_predecessors(controller, rows)
    current = next((row for row in rows if row.name == controller), None)
    if current is None or current.state != 'active':
        raise RuntimeError(f'{controller} is not active')
    actual = set(current.claimed_interfaces)
    if actual != CLAIMS[controller]:
        raise RuntimeError(f'{controller} claimed {sorted(actual)}, expected {sorted(CLAIMS[controller])}')
    for row in rows:
        if row.name != controller and actual.intersection(row.claimed_interfaces):
            raise RuntimeError(f'duplicate command interface claim by {row.name}')


class ControllerSpawner(Node):
    def __init__(self, controller, manager_timeout=180.0, response_timeout=10.0):
        super().__init__(f'spawn_{controller}')
        self.controller = controller
        self.manager_deadline = time.monotonic() + manager_timeout
        self.response_timeout = response_timeout
        # Advertise all reply endpoints before making the first request. Retain
        # them through load/configure/switch/verification and query retries.
        self.service_clients = {
            'list_controllers': self.create_client(ListControllers, '/controller_manager/list_controllers'),
            'load_controller': self.create_client(LoadController, '/controller_manager/load_controller'),
            'configure_controller': self.create_client(ConfigureController, '/controller_manager/configure_controller'),
            'switch_controller': self.create_client(SwitchController, '/controller_manager/switch_controller'),
        }

    def call(self, name, request, *, read_only=False):
        client = self.service_clients[name]
        while not client.service_is_ready():
            remaining = self.manager_deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError(f'controller manager service unavailable: {name}')
            client.wait_for_service(timeout_sec=min(1.0, remaining))
        attempts = 3 if read_only else 1
        for attempt in range(1, attempts + 1):
            started = time.monotonic()
            self.get_logger().info(f'CONTROLLER_REQUEST service={name} attempt={attempt} monotonic={started:.6f}')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.response_timeout)
            if future.done() and not future.cancelled():
                if future.exception() is not None:
                    raise RuntimeError(f'{name} failed: {future.exception()}')
                response = future.result()
                if response is not None:
                    self.get_logger().info(f'CONTROLLER_RESPONSE service={name} elapsed_s={time.monotonic() - started:.3f}')
                    return response
            client.remove_pending_request(future)
            future.cancel()
            self.get_logger().warning(f'CONTROLLER_RESPONSE_LOST service={name}; read_only={read_only}; attempt={attempt}')
        if read_only:
            raise RuntimeError(f'{name}: no response after {attempts} bounded read-only attempts')
        return None

    def rows(self):
        return self.call('list_controllers', ListControllers.Request(), read_only=True).controller

    def state(self, rows):
        return next((row.state for row in rows if row.name == self.controller), 'missing')

    def mutate(self, service, request, expected):
        response = self.call(service, request)
        if response is not None and not response.ok:
            raise RuntimeError(f'{service} rejected {self.controller}')
        # Also verify a successful acknowledgement; never mistake configured
        # for active. A lost mutation response is not permission to replay it.
        deadline = time.monotonic() + self.response_timeout
        while True:
            rows = self.rows()
            state = self.state(rows)
            if state == expected:
                self.get_logger().info(f'CONTROLLER_STATE controller={self.controller} state={state}')
                return rows
            if time.monotonic() >= deadline:
                raise RuntimeError(f'{service} did not settle: {self.controller}={state}, expected {expected}; mutation not retried')
            rclpy.spin_once(self, timeout_sec=0.2)

    def activate(self):
        rows = self.rows()
        validate_predecessors(self.controller, rows)
        state = self.state(rows)
        self.get_logger().info(f'CONTROLLER_INITIAL_STATE controller={self.controller} state={state}')
        if state == 'missing':
            request = LoadController.Request()
            request.name = self.controller
            rows = self.mutate('load_controller', request, 'unconfigured')
            state = self.state(rows)
        if state == 'unconfigured':
            request = ConfigureController.Request()
            request.name = self.controller
            rows = self.mutate('configure_controller', request, 'inactive')
            state = self.state(rows)
        if state == 'inactive':
            request = SwitchController.Request()
            request.activate_controllers = [self.controller]
            request.strictness = SwitchController.Request.STRICT
            request.activate_asap = True
            request.timeout.sec = 5
            rows = self.mutate('switch_controller', request, 'active')
        validate_active(self.controller, rows)
        # Keep the existing readiness timeline's activation log contract.
        self.get_logger().info(f'Configured and activated {self.controller}')
        self.get_logger().info(f'CONTROLLER_ACTIVE_CONFIRMED controller={self.controller} claims={sorted(CLAIMS[self.controller])}')


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('controller', choices=ORDER)
    parser.add_argument('--controller-manager-timeout', type=float, default=180.0)
    parser.add_argument('--response-timeout', type=float, default=10.0)
    rclpy.init(args=args)
    options = parser.parse_args(rclpy.utilities.remove_ros_args(args=args)[1:])
    node = ControllerSpawner(options.controller, options.controller_manager_timeout, options.response_timeout)
    try:
        node.activate()
        return 0
    except (RuntimeError, KeyboardInterrupt) as exc:
        node.get_logger().error(f'CONTROLLER_STARTUP_FAILED controller={options.controller}: {exc}')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
