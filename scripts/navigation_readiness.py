#!/usr/bin/env python3
"""Authoritative readiness gate for a fresh swerve navigation session."""
import argparse
import json
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from gazebo_msgs.srv import GetEntityState
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


class Readiness(Node):
    """Checks live data and state, never just graph membership."""
    def __init__(self, model):
        super().__init__('navigation_readiness_probe', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        if not self.get_parameter('use_sim_time').value:
            raise RuntimeError('FAIL: readiness probe use_sim_time is false')
        self.model = model
        self.clock_seen = self.joints_seen = self.odom_seen = self.v30e_seen = False
        clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Clock, '/clock', self._clock_cb, clock_qos)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.create_subscription(Odometry, '/odom', lambda _: self._set('odom_seen'), 10)
        self.create_subscription(PoseWithCovarianceStamped, '/v30e/pose', lambda _: self._set('v30e_seen'), 10)
        self.entity = self.create_client(GetEntityState, '/get_entity_state')
        self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        names = ('map_server', 'controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'waypoint_follower')
        self.lifecycle = {name: self.create_client(GetState, f'/{name}/get_state') for name in names}
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

    def _set(self, name):
        setattr(self, name, True)

    def _clock_cb(self, msg):
        self.clock_seen = msg.clock.sec != 0 or msg.clock.nanosec != 0

    def _joint_cb(self, msg):
        required = {'steer_front_joint', 'steer_rear_joint', 'wheel_front_drive_joint', 'wheel_rear_drive_joint'}
        self.joints_seen = required.issubset(msg.name)

    def _spin_until(self, predicate, deadline):
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def _service_call(self, client, deadline, prepare=None):
        """Call one service while spinning; never call this from a predicate."""
        if not self._spin_until(client.service_is_ready, deadline):
            return None
        request = client.srv_type.Request()
        if prepare:
            prepare(request)
        future = client.call_async(request)
        while rclpy.ok() and time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return None
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().warning(f'service call failed: {type(exc).__name__}: {exc}')
            return None

    def _wait_entity(self, deadline):
        while time.monotonic() < deadline:
            response = self._service_call(self.entity, min(deadline, time.monotonic() + 2.0),
                                          lambda request: (setattr(request, 'name', self.model),
                                                           setattr(request, 'reference_frame', 'world')))
            if response is not None and response.success:
                return response
            rclpy.spin_once(self, timeout_sec=0.2)
        return None

    def _wait_controllers(self, deadline):
        names = ('joint_state_broadcaster', 'steering_controller', 'drive_controller')
        while time.monotonic() < deadline:
            response = self._service_call(self.controllers, min(deadline, time.monotonic() + 2.0))
            states = {c.name: c.state for c in response.controller} if response else {}
            if all(states.get(name) == 'active' for name in names):
                return True
            rclpy.spin_once(self, timeout_sec=0.2)
        return False

    def _wait_lifecycle(self, deadline):
        while time.monotonic() < deadline:
            states = {}
            for name, client in self.lifecycle.items():
                response = self._service_call(client, min(deadline, time.monotonic() + 2.0))
                states[name] = response.current_state.id if response else None
            if all(state == 3 for state in states.values()):
                return True
            rclpy.spin_once(self, timeout_sec=0.2)
        return False

    def check(self, timeout):
        started, deadline = time.monotonic(), time.monotonic() + timeout
        result = {'stages': {}, 'startup_wall_time': None, 'nav_ready': False, 'reason': None}

        def stage(name, predicate):
            ok = self._spin_until(predicate, deadline)
            result['stages'][name] = bool(ok)
            print(f'{name}={"PASS" if ok else "FAIL"}', flush=True)
            return ok

        if not stage('CLOCK_READY', lambda: self.clock_seen):
            return self._finish(result, 'Gazebo /clock has no non-zero sample')
        if not stage('GAZEBO_READY', lambda: self.entity.service_is_ready()):
            return self._finish(result, '/get_entity_state service unavailable')
        entity_response = self._wait_entity(deadline)
        result['stages']['ROBOT_SPAWNED'] = bool(entity_response is not None and entity_response.success)
        print(f'ROBOT_SPAWNED={"PASS" if result["stages"]["ROBOT_SPAWNED"] else "FAIL"}', flush=True)
        if not result['stages']['ROBOT_SPAWNED']:
            return self._finish(result, f'entity {self.model} not available through /get_entity_state')
        controllers_ok = self._wait_controllers(deadline)
        result['stages']['CONTROLLERS_READY'] = controllers_ok
        print(f'CONTROLLERS_READY={"PASS" if controllers_ok else "FAIL"}', flush=True)
        if not controllers_ok:
            return self._finish(result, 'required ros2_control controllers are not active')
        if not stage('JOINT_STATES_READY', lambda: self.joints_seen):
            return self._finish(result, 'required joint state sample unavailable')
        if not stage('ODOM_READY', lambda: self.odom_seen):
            return self._finish(result, '/odom sample unavailable')
        def tf_exists(target, source):
            try:
                self.tf.lookup_transform(target, source, rclpy.time.Time())
                return True
            except TransformException:
                return False
        if not stage('LOCAL_TF_READY', lambda: tf_exists('odom', 'base_footprint') and tf_exists('base_footprint', 'base_link')):
            return self._finish(result, 'local TF chain odom->base_footprint->base_link unavailable')
        if not stage('V30E_READY', lambda: self.v30e_seen):
            return self._finish(result, 'no V30E absolute measurement observed')
        if not stage('GLOBAL_TF_READY', lambda: tf_exists('map', 'odom') and tf_exists('map', 'base_footprint')):
            return self._finish(result, 'global TF chain map->odom->base_footprint unavailable')
        lifecycle_ok = self._wait_lifecycle(deadline)
        result['stages']['NAV2_LIFECYCLE_READY'] = lifecycle_ok
        print(f'NAV2_LIFECYCLE_READY={"PASS" if lifecycle_ok else "FAIL"}', flush=True)
        if not lifecycle_ok:
            return self._finish(result, 'one or more required Nav2 lifecycle nodes are not ACTIVE')
        if not stage('ACTION_SERVER_READY', lambda: self.action.wait_for_server(timeout_sec=0.0)):
            return self._finish(result, '/navigate_to_pose action server not ready')
        result['startup_wall_time'] = time.monotonic() - started
        result['startup_sim_time'] = self.get_clock().now().nanoseconds * 1e-9
        result['nav_ready'] = True
        print('NAV_READY PASS', flush=True)
        return result

    @staticmethod
    def _finish(result, reason):
        result['reason'] = reason
        print(f'NAV_READY FAIL: {reason}', flush=True)
        return result


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--model', default='swerve_base')
    parser.add_argument('--json')
    args, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = Readiness(args.model)
    try:
        result = node.check(args.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as stream:
            json.dump(result, stream, indent=2)
    return 0 if result['nav_ready'] else 1


if __name__ == '__main__':
    sys.exit(main())
