#!/usr/bin/env python3
"""Authoritative readiness gate for a fresh swerve navigation session."""
import argparse
import json
import os
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.srv import ManageLifecycleNodes
from rcl_interfaces.srv import GetParameters
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


# navigation.launch.py owns exactly these Nav2 lifecycle nodes.  The project
# deliberately uses the V30E/tag localization filter for map->odom, so AMCL
# is not part of this launch contract and must not be invented as a readiness
# dependency.  If that launch is changed to include AMCL, add it here with its
# lifecycle entry at the same time.
NAV2_LIFECYCLE_NODES = (
    'map_server',
    'planner_server',
    'controller_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
)
CONTROLLER_SERVICE_RESPONSE_TIMEOUT_S = 10.0

# Both map_server and SLAM Toolbox publish the canonical map with the ROS map
# QoS (reliable + transient-local). A volatile sensor-data subscriber can miss
# the only latched map sample when the readiness probe joins late.
MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Readiness(Node):
    """Checks live data and state, never just graph membership."""
    def __init__(self, model, mode='navigation', map_file=None):
        super().__init__('navigation_readiness_probe', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        if not self.get_parameter('use_sim_time').value:
            raise RuntimeError('FAIL: readiness probe use_sim_time is false')
        self.model = model
        self.mode = str(mode).lower()
        if self.mode not in ('mapping', 'navigation'):
            raise ValueError(f'unsupported readiness mode: {mode}')
        self.expected_map_file = os.path.realpath(map_file) if map_file else None
        self.clock_seen = False
        self.joints_seen = False
        self.odom_seen = False
        self.filtered_odom_seen = False
        self.scan_seen = False
        self.map_seen = False
        self.points_seen = False
        self.filtered_points_seen = False
        self.v30e_seen = False
        clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Clock, '/clock', self._clock_cb, clock_qos)
        # A best-effort/volatile subscriber is compatible with both the
        # Gazebo sensor publishers and reliable application publishers.  The
        # probe only needs one real sample, not a rate measurement.
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, sensor_qos)
        self.create_subscription(Odometry, '/odom', lambda _: self._set('odom_seen'), sensor_qos)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 lambda _: self._set('filtered_odom_seen'), sensor_qos)
        self.create_subscription(LaserScan, '/scan', lambda _: self._set('scan_seen'), sensor_qos)
        self.create_subscription(OccupancyGrid, '/map', lambda _: self._set('map_seen'), MAP_QOS)
        self.create_subscription(PointCloud2, '/lidar/points', lambda _: self._set('points_seen'), sensor_qos)
        self.create_subscription(PointCloud2, '/lidar/points_filtered',
                                 lambda _: self._set('filtered_points_seen'), sensor_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/v30e/pose',
                                 lambda _: self._set('v30e_seen'), sensor_qos)
        self.entity = self.create_client(GetEntityState, '/get_entity_state')
        self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        names = NAV2_LIFECYCLE_NODES if self.mode == 'navigation' else ()
        self.lifecycle = {name: self.create_client(GetState, f'/{name}/get_state') for name in names}
        self.nav_lifecycle_manager = (
            self.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
            if self.mode == 'navigation' else None)
        self.map_parameters = self.create_client(GetParameters, '/map_server/get_parameters')
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

    def _set(self, name):
        setattr(self, name, True)

    def _clock_cb(self, msg):
        # Zero is a valid first simulation-time sample.  Runtime acceptance
        # separately verifies that the value advances while Gazebo runs.
        self.clock_seen = True

    def _joint_cb(self, msg):
        # The gate requires a received message. Controller activation already
        # verifies the expected joint interfaces; do not reject a valid sample
        # merely because a publisher temporarily emitted an empty name array.
        self.joints_seen = True

    def _topic_present(self, topic):
        try:
            return any(name == topic for name, _ in self.get_topic_names_and_types())
        except Exception:
            return False

    def _node_present(self, node):
        wanted = node.lstrip('/')
        try:
            return any(name.lstrip('/') == wanted or name.endswith('/' + wanted)
                       for name, _ in self.get_node_names_and_namespaces())
        except Exception:
            return False

    def _stage(self, result, name, predicate, success_label, failure_label):
        ok = self._spin_until(predicate, self.deadline)
        result['stages'][name] = bool(ok)
        print(f'{name}={"PASS" if ok else "FAIL"}', flush=True)
        print(f'[OK] {success_label}' if ok else f'[FAIL] {failure_label}', flush=True)
        return ok

    def _topic_stage(self, result, name, topic, seen_name):
        def ready():
            return self._topic_present(topic) and bool(getattr(self, seen_name))

        if self._stage(result, name, ready, f'{topic} publishing',
                       f'{topic} exists but no message received'):
            return True
        if not self._topic_present(topic):
            print(f'[FAIL] {topic} is not advertised', flush=True)
        return False

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
            response = self._service_call(
                self.controllers,
                min(deadline, time.monotonic() + CONTROLLER_SERVICE_RESPONSE_TIMEOUT_S),
            )
            states = {c.name: c.state for c in response.controller} if response else {}
            if all(states.get(name) == 'active' for name in names):
                return True
            rclpy.spin_once(self, timeout_sec=0.2)
        return False

    def _wait_lifecycle(self, deadline):
        states = {}
        while time.monotonic() < deadline:
            for name, client in self.lifecycle.items():
                response = self._service_call(client, min(deadline, time.monotonic() + 2.0))
                states[name] = response.current_state.id if response else None
            if all(state == 3 for state in states.values()):
                return states
            rclpy.spin_once(self, timeout_sec=0.2)
        return states

    def _lifecycle_snapshot(self, deadline):
        states = {}
        for name, client in self.lifecycle.items():
            response = self._service_call(client, min(deadline, time.monotonic() + 2.0))
            states[name] = response.current_state.id if response else None
        return states

    def _wait_map_file(self, deadline):
        if self.expected_map_file is None:
            return True
        while time.monotonic() < deadline:
            response = self._service_call(
                self.map_parameters,
                min(deadline, time.monotonic() + 2.0),
                lambda request: setattr(request, 'names', ['yaml_filename']),
            )
            if response is not None and response.values:
                actual = str(response.values[0].string_value)
                if os.path.realpath(actual) == self.expected_map_file:
                    return True
                self.get_logger().warning(
                    f'map_server yaml_filename={actual!r}, expected={self.expected_map_file!r}')
            rclpy.spin_once(self, timeout_sec=0.2)
        return False

    def check(self, timeout):
        started, deadline = time.monotonic(), time.monotonic() + timeout
        self.deadline = deadline
        result = {'stages': {}, 'startup_wall_time': None, 'nav_ready': False, 'reason': None}

        if not self._stage(result, 'CLOCK_READY', lambda: self.clock_seen,
                           '/clock publishing', '/clock exists but no message received'):
            return self._finish(result, 'Gazebo /clock has no message')
        if not self._stage(result, 'GAZEBO_READY', lambda: self.entity.service_is_ready(),
                           'Gazebo', '/get_entity_state service unavailable'):
            return self._finish(result, '/get_entity_state service unavailable')
        entity_response = self._wait_entity(deadline)
        result['stages']['ROBOT_SPAWNED'] = bool(entity_response is not None and entity_response.success)
        print(f'ROBOT_SPAWNED={"PASS" if result["stages"]["ROBOT_SPAWNED"] else "FAIL"}', flush=True)
        print('[OK] Robot spawned' if result['stages']['ROBOT_SPAWNED'] else '[FAIL] Robot not spawned', flush=True)
        if not result['stages']['ROBOT_SPAWNED']:
            return self._finish(result, f'entity {self.model} not available through /get_entity_state')
        controllers_ok = self._wait_controllers(deadline)
        result['stages']['CONTROLLERS_READY'] = controllers_ok
        print(f'CONTROLLERS_READY={"PASS" if controllers_ok else "FAIL"}', flush=True)
        print('[OK] controller_manager and required controllers active' if controllers_ok
              else '[FAIL] controller_manager/controllers are not active', flush=True)
        if not controllers_ok:
            return self._finish(result, 'required ros2_control controllers are not active')
        if not self._topic_stage(result, 'JOINT_STATES_READY', '/joint_states', 'joints_seen'):
            return self._finish(result, 'required joint state sample unavailable')
        if not self._topic_stage(result, 'ODOM_READY', '/odom', 'odom_seen'):
            return self._finish(result, '/odom sample unavailable')

        if not self._topic_stage(result, 'FILTERED_ODOM_READY', '/odometry/filtered', 'filtered_odom_seen'):
            return self._finish(result, '/odometry/filtered sample unavailable')
        if not self._topic_stage(result, 'SCAN_READY', '/scan', 'scan_seen'):
            return self._finish(result, '/scan sample unavailable')

        def tf_exists(target, source):
            try:
                self.tf.lookup_transform(target, source, rclpy.time.Time())
                return True
            except (TransformException, RuntimeError):
                return False

        if not self._stage(result, 'LOCAL_TF_READY',
                           lambda: tf_exists('odom', 'base_link'),
                           'TF odom -> base_link', 'TF odom -> base_link did not resolve before timeout'):
            return self._finish(result, 'TF odom -> base_link unavailable')
        if self.mode == 'mapping':
            if not self._topic_stage(result, 'MAP_READY', '/map', 'map_seen'):
                return self._finish(result, '/map exists but no message received')
            if not self._stage(result, 'GLOBAL_TF_READY',
                               lambda: tf_exists('map', 'odom') and tf_exists('map', 'base_link'),
                               'TF map -> odom -> base_link',
                               'TF map -> odom -> base_link did not resolve before timeout'):
                return self._finish(result, 'global TF chain map->odom->base_link unavailable')
            slam_ok = self._stage(result, 'SLAM_READY', lambda: self._node_present('slam_toolbox'),
                                  'SLAM Toolbox', 'SLAM Toolbox node is not running')
            if not slam_ok:
                return self._finish(result, 'SLAM Toolbox is not running')
        else:
            # The V30E simulation owns map->odom in navigation mode. Wait for
            # that live transform and the local odom chain before activating
            # Nav2, so its costmaps never enter their lifecycle transition
            # without the robot's required TF being available.
            if not self._stage(result, 'GLOBAL_TF_READY',
                               lambda: tf_exists('map', 'odom') and tf_exists('map', 'base_link'),
                               'TF map -> odom -> base_link',
                               'TF map -> odom -> base_link did not resolve before timeout'):
                return self._finish(result, 'global TF chain map->odom->base_link unavailable')
            nav_nodes_ok = True
            for name in NAV2_LIFECYCLE_NODES:
                node_ok = self._stage(
                    result, f'NODE_{name.upper()}_READY', lambda name=name: self._node_present(name),
                    f'Nav2 {name}', f'Nav2 node /{name} is not running',
                )
                nav_nodes_ok = nav_nodes_ok and node_ok
            if not nav_nodes_ok:
                return self._finish(result, 'one or more required Nav2 nodes are not running')
            lifecycle_states = self._lifecycle_snapshot(deadline)
            if not all(state == 3 for state in lifecycle_states.values()):
                # start_stack sets autostart=false while Gazebo is cold, then
                # this single readiness participant requests normal Nav2
                # lifecycle startup once all lower-layer prerequisites pass.
                # Direct launches keep the navigation.launch.py default.
                if all(state == 1 for state in lifecycle_states.values()):
                    response = self._service_call(
                        self.nav_lifecycle_manager,
                        deadline,
                        lambda request: setattr(request, 'command', ManageLifecycleNodes.Request.STARTUP),
                    )
                    if response is None or not response.success:
                        return self._finish(result, 'Nav2 lifecycle manager startup request failed')
                    result['stages']['NAV2_STARTUP_REQUESTED'] = True
                    print('NAV2_STARTUP_REQUESTED=PASS', flush=True)
                lifecycle_states = self._wait_lifecycle(deadline)
            lifecycle_ok = bool(lifecycle_states) and all(
                lifecycle_states.get(name) == 3 for name in NAV2_LIFECYCLE_NODES
            )
            result['stages']['NAV2_LIFECYCLE_READY'] = lifecycle_ok
            print(f'NAV2_LIFECYCLE_READY={"PASS" if lifecycle_ok else "FAIL"}', flush=True)
            if lifecycle_ok:
                print('[OK] required Nav2 lifecycle nodes active', flush=True)
            else:
                inactive = ', '.join(
                    f'/{name}={lifecycle_states.get(name)}'
                    for name in NAV2_LIFECYCLE_NODES if lifecycle_states.get(name) != 3
                )
                print(f'[FAIL] Nav2 lifecycle not active: {inactive}', flush=True)
                return self._finish(result, 'one or more required Nav2 lifecycle nodes are not ACTIVE')
            map_file_ok = self._stage(
                result, 'MAP_FILE_READY', lambda: self._wait_map_file(deadline),
                f'map_server yaml_filename={self.expected_map_file}' if self.expected_map_file else 'map_server map parameter',
                'map_server yaml_filename does not match requested absolute path',
            )
            if not map_file_ok:
                return self._finish(result, 'map_server yaml_filename does not match requested map')
            if not self._stage(result, 'ACTION_SERVER_READY',
                               lambda: self.action.wait_for_server(timeout_sec=0.0),
                               '/navigate_to_pose action server',
                               '/navigate_to_pose action server not ready'):
                return self._finish(result, '/navigate_to_pose action server not ready')
        result['startup_wall_time'] = time.monotonic() - started
        result['startup_sim_time'] = self.get_clock().now().nanoseconds * 1e-9
        result['nav_ready'] = True
        ready_label = 'Mapping stack READY' if self.mode == 'mapping' else 'Navigation stack READY'
        print(f'[OK] {ready_label}', flush=True)
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
    parser.add_argument('--mode', choices=('mapping', 'navigation'), default='navigation')
    parser.add_argument('--map-file')
    parser.add_argument('--json')
    args, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = Readiness(args.model, args.mode, args.map_file)
    try:
        try:
            result = node.check(args.timeout)
        except ExternalShutdownException:
            result = {'stages': {}, 'startup_wall_time': None, 'nav_ready': False,
                      'reason': 'ROS context shut down during readiness probe'}
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as stream:
            json.dump(result, stream, indent=2)
    return 0 if result['nav_ready'] else 1


if __name__ == '__main__':
    sys.exit(main())
