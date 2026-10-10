#!/usr/bin/env python3
"""Exercise real ROS motion and the frontend's Django WebSocket commands."""
from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import time
import urllib.error
import urllib.request
import uuid
from collections import deque
from threading import RLock, Thread

import rclpy
import websocket
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.clock import ClockType
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor, await_or_execute
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener
from mechanical_settling import MechanicalSettling


TRANSLATION_TOLERANCE_M = 0.05
ROTATION_TOLERANCE_RAD = 0.05
NAV_GOAL_XY_TOLERANCE_M = 0.05
NAV_GOAL_YAW_TOLERANCE_RAD = 0.05
MAX_TF_COMPONENT_AGE_SIM_S = 0.5
MAX_TF_COMPONENT_SKEW_SIM_S = 0.5
TF_LOOKUP_TIMEOUT_S = 0.05
MAX_SIM_CLOCK_WALL_AGE_S = 1.0
MOTION_DURATION_SIM_S = 2.0
WEB_COMMAND_REFRESH_WALL_S = 0.10
EXPECTED_ROBOT_ENTITY = 'swerve_base'


class PublisherInfoExecutor(SingleThreadedExecutor):
    """Keep Humble's DDS MessageInfo for /cmd_vel publisher attribution."""

    def __init__(self, node, info_subscriptions):
        super().__init__()
        self.info_subscriptions = set(info_subscriptions)
        self.add_node(node)

    def _take_subscription(self, sub):
        with sub.handle:
            return sub.handle.take_message(sub.msg_type, sub.raw)

    async def _execute_subscription(self, sub, message_info):
        if message_info is None:
            return
        message, info = message_info
        if sub in self.info_subscriptions:
            await await_or_execute(sub.callback, message, info)
        else:
            await await_or_execute(sub.callback, message)


class EpochTfObserver(Node):
    """Receive TF independently and replace subscriptions after a clock rewind.

    The acceptance probe performs blocking HTTP/WebSocket work between its ROS
    callbacks. Isolating TF reception prevents those operations from starving
    the transform buffer. Recreating the listener discards queued volatile
    /tf samples and reacquires transient-local /tf_static data.
    """

    def __init__(self):
        super().__init__('simulation_e2e_tf_observer', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.simulation_time_s = None
        self.clock_received_monotonic = None
        self.clock_samples = []
        self.clock_source_samples = deque(maxlen=20000)
        self.clock_resets = []
        self.epoch_generation = 0
        self.clock_update_callbacks = []
        self._snapshot_lock = RLock()
        self.tf = Buffer()
        clock_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        self.clock_subscription = self.create_subscription(
            Clock, '/clock', self._clock_cb, clock_qos)
        self.tf_listener = TransformListener(self.tf, self)
        # ``Node.executor`` is a managed property that registers this node with
        # an executor; it is not a place to store the executor instance. Keep
        # the observer's executor separately so replacing it cannot detach the
        # node or leave the worker without an executor.
        self.tf_executor = PublisherInfoExecutor(self, (self.clock_subscription,))
        self.thread = Thread(target=self.tf_executor.spin,
                             name='simulation-e2e-tf-observer', daemon=True)
        self.thread.start()

    def _clock_cb(self, msg, info=None):
        stamp = msg.clock
        current = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        received = time.monotonic()
        source_ns = int(info.get('source_timestamp', 0) or 0) if isinstance(info, dict) else 0
        received_ns = int(info.get('received_timestamp', 0) or 0) if isinstance(info, dict) else 0
        with self._snapshot_lock:
            previous = self.simulation_time_s
            if previous is not None and current < previous - 0.5:
                self.tf_listener.unregister()
                self.tf = Buffer()
                self.tf_listener = TransformListener(self.tf, self)
                self.epoch_generation += 1
                self.clock_resets.append({
                    'previous_sim_s': previous,
                    'current_sim_s': current,
                    'backward_jump_s': previous - current,
                    'epoch_generation': self.epoch_generation,
                    'wall_monotonic_s': received,
                })
                self.clock_source_samples.clear()
            self.simulation_time_s = current
            self.clock_received_monotonic = received
            if source_ns > 0:
                sample = {
                    'source_timestamp_ns': source_ns,
                    'received_timestamp_ns': received_ns,
                    'simulation_time_s': current,
                }
                source_stamps = [row['source_timestamp_ns']
                                 for row in self.clock_source_samples]
                index = bisect.bisect_left(source_stamps, source_ns)
                if index < len(source_stamps) and source_stamps[index] == source_ns:
                    self.clock_source_samples[index] = sample
                else:
                    self.clock_source_samples.insert(index, sample)
                    if len(self.clock_source_samples) > self.clock_source_samples.maxlen:
                        self.clock_source_samples.popleft()
            if not self.clock_samples or received - self.clock_samples[-1]['wall_monotonic_s'] >= 0.25:
                self.clock_samples.append({'simulation_time_s': current,
                                           'wall_monotonic_s': received})
            epoch = self.epoch_generation
            callbacks = tuple(self.clock_update_callbacks)
        for callback in callbacks:
            callback(epoch)

    def add_clock_update_callback(self, callback):
        self.clock_update_callbacks.append(callback)

    def clock_source_timestamp_bounds(self):
        with self._snapshot_lock:
            if not self.clock_source_samples:
                return None, None
            return (self.clock_source_samples[0]['source_timestamp_ns'],
                    self.clock_source_samples[-1]['source_timestamp_ns'])

    def tf_snapshot(self):
        with self._snapshot_lock:
            return (self.tf, self.epoch_generation, self.simulation_time_s,
                    self.clock_received_monotonic)

    def simulation_time_at_source_timestamp(self, source_timestamp_ns):
        """Map a Gazebo message source timestamp through the live /clock samples."""
        try:
            target = int(source_timestamp_ns)
        except (TypeError, ValueError, OverflowError):
            return None
        with self._snapshot_lock:
            samples = list(self.clock_source_samples)
        if target <= 0 or len(samples) < 2:
            return None
        source_times = [sample['source_timestamp_ns'] for sample in samples]
        right_index = bisect.bisect_left(source_times, target)
        if right_index < len(samples) and source_times[right_index] == target:
            return {
                'simulation_time_s': samples[right_index]['simulation_time_s'],
                'source_timestamp_ns': target,
                'clock_source_timestamps_ns': [target],
                'interpolation_span_wall_s': 0.0,
                'method': 'exact_rmw_source_timestamp',
            }
        if right_index == 0 or right_index >= len(samples):
            return None
        before, after = samples[right_index - 1], samples[right_index]
        span_ns = after['source_timestamp_ns'] - before['source_timestamp_ns']
        if span_ns <= 0:
            return None
        fraction = (target - before['source_timestamp_ns']) / span_ns
        simulation_time_s = (before['simulation_time_s']
            + (after['simulation_time_s'] - before['simulation_time_s']) * fraction)
        return {
            'simulation_time_s': simulation_time_s,
            'source_timestamp_ns': target,
            'clock_source_timestamps_ns': [before['source_timestamp_ns'],
                                           after['source_timestamp_ns']],
            'interpolation_span_wall_s': span_ns * 1e-9,
            'method': 'interpolated_between_rmw_clock_source_timestamps',
        }

    def close(self):
        self.tf_executor.shutdown(timeout_sec=2.0)
        self.thread.join(timeout=2.0)
        self.tf_listener.unregister()
        self.tf_executor.remove_node(self)
        self.destroy_node()


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


def failed_acceptance_conditions(checks: dict[str, bool]) -> list[str]:
    """Return every failed or unavailable named release criterion."""
    return [name for name, passed in checks.items() if passed is not True]


def compose_planar_transforms(parent_to_mid: tuple[float, float, float],
                              mid_to_child: tuple[float, float, float]) -> tuple[float, float, float]:
    """Compose planar map->odom and odom->base transforms."""
    px, py, pyaw = parent_to_mid
    mx, my, myaw = mid_to_child
    cosine, sine = math.cos(pyaw), math.sin(pyaw)
    return (px + cosine * mx - sine * my,
            py + sine * mx + cosine * my,
            wrap_angle(pyaw + myaw))


def map_pose_from_world(map_pose_reference: tuple[float, float, float],
                        world_pose_reference: tuple[float, float, float],
                        current_world_pose: tuple[float, float, float]) -> tuple[float, float, float]:
    """Project Gazebo ground truth through a measured map/world alignment."""
    delta_yaw = wrap_angle(map_pose_reference[2] - world_pose_reference[2])
    cosine, sine = math.cos(delta_yaw), math.sin(delta_yaw)
    tx = map_pose_reference[0] - (cosine * world_pose_reference[0]
                                  - sine * world_pose_reference[1])
    ty = map_pose_reference[1] - (sine * world_pose_reference[0]
                                  + cosine * world_pose_reference[1])
    return (cosine * current_world_pose[0] - sine * current_world_pose[1] + tx,
            sine * current_world_pose[0] + cosine * current_world_pose[1] + ty,
            wrap_angle(current_world_pose[2] + delta_yaw))


def transform_is_fresh(age_sim_s: float, *, maximum_age: float = MAX_TF_COMPONENT_AGE_SIM_S) -> bool:
    """Reject stale or materially future-dated TF samples in simulation time."""
    return math.isfinite(age_sim_s) and -0.05 <= age_sim_s <= maximum_age


def transform_is_confirmable_when_settled(sample: dict | None) -> bool:
    """Bound post-stop TF age; start-of-motion admission still requires fresh TF.

    SLAM may stop refreshing map->odom while the chassis is stationary. This
    bounded post-settle observation is accepted only alongside mechanical
    settling and an independent Gazebo-to-map accuracy check.
    """
    if not isinstance(sample, dict) or sample.get('frame_ids_valid') is not True:
        return False
    try:
        skew = float(sample['component_time_skew_sim_s'])
        ages = [float(sample[key]['age_sim_s'])
                for key in ('map_to_odom', 'odom_to_base')]
        pose = tuple(float(value) for value in sample['pose'])
    except (KeyError, TypeError, ValueError, OverflowError):
        return False
    return (all(math.isfinite(value) for value in (*ages, *pose, skew))
            and all(-0.05 <= age <= 1.0 for age in ages)
            and skew <= MAX_TF_COMPONENT_SKEW_SIM_S)


def pose_pair_skew_sim_s(map_sample: dict | None,
                         gazebo_sim_s: float | None) -> float | None:
    if not isinstance(map_sample, dict) or gazebo_sim_s is None:
        return None
    try:
        odom_stamp = float(map_sample['stamp_sim_s'])
        gazebo_stamp = float(gazebo_sim_s)
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
    if not math.isfinite(odom_stamp) or not math.isfinite(gazebo_stamp):
        return None
    return abs(gazebo_stamp - odom_stamp)


def common_tf_sample_time(map_to_odom_stamp_s: float,
                          odom_to_base_stamp_s: float,
                          requested_stamp_s: float | None = None) -> float | None:
    """Select a non-extrapolating timestamp shared by the two dynamic TF edges.

    Asking TF2 for ``map -> base`` at Time(0) can return an old compound-chain
    timestamp even while the individual dynamic edges have newer samples. The
    latest common timestamp for this known ``map -> odom -> base`` chain is the
    earlier latest edge timestamp. TF2 then interpolates each edge at exactly
    that time; requests later than either edge are rejected, never extrapolated.
    """
    try:
        map_stamp = float(map_to_odom_stamp_s)
        odom_stamp = float(odom_to_base_stamp_s)
        requested = (min(map_stamp, odom_stamp) if requested_stamp_s is None
                     else float(requested_stamp_s))
    except (TypeError, ValueError, OverflowError):
        return None
    if not all(math.isfinite(value) for value in (map_stamp, odom_stamp, requested)):
        return None
    latest_common = min(map_stamp, odom_stamp)
    return requested if requested <= latest_common + 1e-9 else None


def common_pose_pair_stamp(tf_stamp_s: float, gazebo_stamp_s: float,
                           minimum_stamp_s: float | None = None) -> float | None:
    """Choose an exact shared pose time, optionally after a state transition."""
    try:
        tf_stamp = float(tf_stamp_s)
        gazebo_stamp = float(gazebo_stamp_s)
        minimum = None if minimum_stamp_s is None else float(minimum_stamp_s)
    except (TypeError, ValueError, OverflowError):
        return None
    values = (tf_stamp, gazebo_stamp) if minimum is None else (
        tf_stamp, gazebo_stamp, minimum)
    if not all(math.isfinite(value) for value in values):
        return None
    shared = min(tf_stamp, gazebo_stamp)
    if minimum is not None and shared < minimum - 1e-6:
        return None
    return shared


def ready_navigation_map_signature(runtime_status, robot_id,
                                   verified_local_map_identity=None):
    """Return the readiness identity for the active runtime's map contract.

    UNIFIED mode requires the bridge's registered canonical map tuple to match
    the active SLAM map.  A supervised NAVIGATION handoff instead loads a
    robot-local saved map directly in Nav2; that mode intentionally has no
    canonical registration tuple, so callers must supply the exact local map
    identity whose YAML/PGM and live OccupancyGrid were independently checked.
    """
    if not isinstance(runtime_status, dict):
        return None
    active = (runtime_status.get('local_active_maps') or {}).get(robot_id) or {}
    nav_map = (runtime_status.get('navigation_maps') or {}).get(robot_id) or {}
    active_id = str(active.get('active_map_id') or '')
    active_revision = str(active.get('active_map_revision') or '')
    if verified_local_map_identity is not None:
        expected_id = str(verified_local_map_identity.get('active_map_id') or '')
        expected_revision = str(verified_local_map_identity.get('active_map_revision') or '')
        capabilities = (runtime_status.get('robot_capabilities') or {}).get(robot_id) or {}
        local_id = str(active.get('local_active_map_id') or '')
        local_revision = str(active.get('local_active_map_revision') or '')
        if (str(runtime_status.get('runtime_state') or '').upper() != 'NAVIGATION'
                or active.get('map_source') != 'LOCAL_MAP'
                or active.get('map_sync_status') != 'LOCAL_ONLY'
                or not expected_id or not expected_revision
                or active_id != expected_id or active_revision != expected_revision
                or local_id != expected_id or local_revision != expected_revision
                or capabilities.get('nav2_ready') is not True):
            return None
        # The caller supplies this identity only after independently validating
        # that Nav2's fresh /navigation_map raster exactly matches the selected
        # registry artifact.  Keep it separate from canonical registrations.
        return ('LOCAL_MAP', active_id, active_revision,
                str(active.get('map_content_revision') or ''))
    nav_id = str(nav_map.get('active_map_id') or '')
    nav_revision = str(nav_map.get('active_map_revision') or '')
    navigation_revision = str(nav_map.get('navigation_map_revision') or '')
    if (nav_map.get('ready') is not True or not active_id or not active_revision
            or nav_id != active_id or nav_revision != active_revision
            or not navigation_revision):
        return None
    return (active_id, active_revision, navigation_revision,
            str(nav_map.get('canonical_map_revision') or ''),
            str(nav_map.get('registration_revision') or ''),
            str(nav_map.get('registration_source') or ''))


def interpolate_gazebo_pose(samples, simulation_time_s: float):
    """Return ModelStates pose at a shared simulation stamp; never extrapolate."""
    try:
        target = float(simulation_time_s)
    except (TypeError, ValueError, OverflowError):
        return None
    if not math.isfinite(target) or not samples:
        return None
    ordered = list(samples)
    for sample in ordered:
        stamp = float(sample['simulation_time_s'])
        if abs(stamp - target) <= 1e-9:
            return {
                'pose': tuple(sample['pose']), 'simulation_time_s': target,
                'source_stamps_sim_s': [stamp], 'interpolation_span_sim_s': 0.0,
                'method': 'exact_model_states_callback_stamp',
            }
    for before, after in zip(ordered, ordered[1:]):
        t0, t1 = float(before['simulation_time_s']), float(after['simulation_time_s'])
        if t0 <= target <= t1 and t1 > t0:
            fraction = (target - t0) / (t1 - t0)
            yaw_delta = wrap_angle(float(after['pose'][2]) - float(before['pose'][2]))
            return {
                'pose': (
                    float(before['pose'][0])
                    + (float(after['pose'][0]) - float(before['pose'][0])) * fraction,
                    float(before['pose'][1])
                    + (float(after['pose'][1]) - float(before['pose'][1])) * fraction,
                    wrap_angle(float(before['pose'][2]) + yaw_delta * fraction),
                ),
                'simulation_time_s': target,
                'source_stamps_sim_s': [t0, t1],
                'interpolation_span_sim_s': t1 - t0,
                'method': 'linear_model_states_interpolation',
            }
    return None


def insert_gazebo_pose_sample(samples, sample, max_samples=10000):
    """Keep independently timestamped Gazebo poses in simulation-time order.

    ROS callbacks can be dispatched late after another callback performs
    blocking work. Callback arrival order is therefore not a safe proxy for
    source-time order. Keep history sorted and replace duplicate stamps so
    interpolation and latest-sample selection remain deterministic.
    """
    stamp = float(sample['simulation_time_s'])
    if not math.isfinite(stamp):
        return deque(samples, maxlen=max_samples)
    ordered = list(samples)
    stamps = [float(row['simulation_time_s']) for row in ordered]
    index = bisect.bisect_left(stamps, stamp)
    if index < len(ordered) and abs(stamps[index] - stamp) <= 1e-9:
        replacement = dict(sample)
        replacement['simulation_time_s'] = stamps[index]
        ordered[index] = replacement
    elif index > 0 and abs(stamps[index - 1] - stamp) <= 1e-9:
        replacement = dict(sample)
        replacement['simulation_time_s'] = stamps[index - 1]
        ordered[index - 1] = replacement
    else:
        ordered.insert(index, sample)
    if len(ordered) > max_samples:
        ordered = ordered[-max_samples:]
    return deque(ordered, maxlen=max_samples)


class MotionProbe(Node):
    def __init__(self):
        super().__init__('simulation_e2e_acceptance', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.tf_observer = EpochTfObserver()
        self.odom = None
        self.odom_velocity = None
        self.odom_sample_monotonic = None
        self.gazebo_velocity = None
        self.filtered_odom = None
        self.odom_count = 0
        self.gazebo_pose = None
        self.gazebo_pose_count = 0
        self.gazebo_pose_sample_monotonic = None
        self.gazebo_pose_sim_s = None
        self.gazebo_pose_history = deque(maxlen=10000)
        self.gazebo_history_lock = RLock()
        self.gazebo_history_epoch = 0
        self.pending_gazebo_samples = deque(maxlen=100)
        self.gazebo_unaligned_sample_count = 0
        self.gazebo_discarded_unaligned_sample_count = 0
        self.gazebo_unaligned_last = None
        self.gazebo_valid_sample_count = 0
        self.tf_observer.add_clock_update_callback(self._flush_pending_gazebo_samples)
        self.joint_state = None
        self.wheel_radius = None
        self.joint_events: list[tuple[float, dict, dict]] = []
        self.map = None
        self.map_sample_count = 0
        self.map_sample_monotonic = None
        self.navigation_map = None
        self.navigation_map_sample_count = 0
        self.navigation_map_sample_monotonic = None
        self.cmd_events: list[tuple[float, float, float, float]] = []
        self.manual_cmd_events: list[tuple[float, float, float, float]] = []
        self.nav_cmd_events: list[tuple[float, float, float, float]] = []
        self.selected_cmd_events: list[tuple[float, float, float, float]] = []
        self.command_owner_events: list[tuple[float, str]] = []
        self.cmd_publisher_gids: list[str | None] = []
        self.steering_events: list[tuple[float, ...]] = []
        self.drive_events: list[tuple[float, ...]] = []
        self.action_status_events: list[tuple[float, dict[bytes, int]]] = []
        self.ws_messages: list[dict] = []
        self.ws_errors: list[str] = []
        self.command_diagnostics: list[tuple[float, dict]] = []
        self.operator_ws_closed = False
        self.nav_statuses: list[dict] = []
        self.control_statuses: list[dict] = []
        self.runtime_status = None
        self.navigation_trace_enabled = False
        self.navigation_trace_started = None
        self.navigation_trace_last_sample = 0.0
        self.navigation_trace: list[dict] = []
        self.command = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_command = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos)
        self.create_subscription(Odometry, '/odometry/filtered', self._filtered_odom_cb, qos)
        # Gazebo ModelStates is independent ground truth. Keep its callback on
        # the dedicated observer executor with /clock and TF so synchronous
        # HTTP/WebSocket calls on MotionProbe cannot queue stale samples and
        # later mislabel an old callback as the newest pose.
        self.model_states_subscription = self.tf_observer.create_subscription(
            ModelStates, '/model_states', self._model_states_cb, 10)
        self.tf_observer.tf_executor.info_subscriptions.add(
            self.model_states_subscription)
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 20)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # The active Nav2 map is published on /navigation_map in the
        # REGISTERED_CANONICAL and STATIC_MAP launch modes. Keep a separate
        # observation from SLAM's /map so acceptance checks do not mistake a
        # stopped/stale SLAM grid for the map Nav2 actually loaded.
        self.create_subscription(OccupancyGrid, '/navigation_map', self._navigation_map_cb,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.cmd_subscription = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_cb, 20)
        self.create_subscription(
            Twist, '/cmd_vel_manual',
            lambda msg: self._record_command(self.manual_cmd_events, msg), 20)
        self.create_subscription(
            Twist, '/cmd_vel_nav',
            lambda msg: self._record_command(self.nav_cmd_events, msg), 20)
        self.create_subscription(
            Twist, '/cmd_vel_selected',
            lambda msg: self._record_command(self.selected_cmd_events, msg), 20)
        self.create_subscription(
            String, '/command_owner',
            lambda msg: self.command_owner_events.append((time.monotonic(), str(msg.data))), 20)
        self.create_subscription(Float64MultiArray, '/steering_controller/commands',
                                 lambda msg: self.steering_events.append(
                                     (time.monotonic(), *tuple(msg.data))), 20)
        self.create_subscription(Float64MultiArray, '/drive_controller/commands',
                                 lambda msg: self.drive_events.append(
                                     (time.monotonic(), *tuple(msg.data))), 20)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self._action_status_cb, 10)
        self.navigation = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.motion_executor = PublisherInfoExecutor(
            self, (self.cmd_subscription,))

    def _odom_cb(self, msg):
        p = pose_from_odom(msg)
        if all(math.isfinite(value) for value in p):
            self.odom = p
            self.odom_velocity = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)
            self.odom_sample_monotonic = time.monotonic()
            self.odom_count += 1

    def _filtered_odom_cb(self, msg):
        p = pose_from_odom(msg)
        if all(math.isfinite(value) for value in p):
            self.filtered_odom = p

    def _model_states_cb(self, msg, info=None):
        try:
            index = msg.name.index(EXPECTED_ROBOT_ENTITY)
        except ValueError:
            return
        if index >= len(msg.pose):
            return
        pose = pose_from_pose(msg.pose[index])
        if all(math.isfinite(value) for value in pose):
            _, epoch, simulation_time_s, _clock_wall = self.tf_observer.tf_snapshot()
            info_source_timestamp = (int(info.get('source_timestamp', 0) or 0)
                                     if isinstance(info, dict) else 0)
            info_received_timestamp = (int(info.get('received_timestamp', 0) or 0)
                                       if isinstance(info, dict) else 0)
            source_time = self.tf_observer.simulation_time_at_source_timestamp(
                info_source_timestamp)
            body_twist = msg.twist[index]
            velocity = (body_twist.linear.x, body_twist.linear.y,
                        body_twist.angular.z)
            self.gazebo_pose_count += 1
            if source_time is None:
                # ModelStates has no Header. Never turn callback dispatch time
                # into pose freshness. A source stamp just ahead of /clock is
                # held briefly and retried when the next clock bracket arrives.
                # Stamps outside the retained clock range are discarded.
                self.gazebo_unaligned_sample_count += 1
                self.gazebo_unaligned_last = {
                    'source_timestamp_ns': info_source_timestamp,
                    'received_timestamp_ns': info_received_timestamp,
                    'callback_epoch': epoch,
                    'callback_wall_monotonic_s': time.monotonic(),
                }
                if info_source_timestamp > 0 and info_received_timestamp > 0:
                    self.pending_gazebo_samples.append({
                        'pose': pose, 'velocity': velocity,
                        'source_timestamp_ns': info_source_timestamp,
                        'received_timestamp_ns': info_received_timestamp,
                        'epoch': epoch,
                    })
                    self._flush_pending_gazebo_samples(epoch)
                else:
                    self.gazebo_discarded_unaligned_sample_count += 1
                return
            self._record_aligned_gazebo_sample(
                pose, velocity, info_source_timestamp, info_received_timestamp,
                source_time, epoch)

    def _record_aligned_gazebo_sample(self, pose, velocity, source_timestamp_ns,
                                      received_timestamp_ns, source_time, epoch):
            simulation_time_s = float(source_time['simulation_time_s'])
            received_age = (time.time_ns() - received_timestamp_ns) * 1e-9
            # Convert the RMW wall-clock receive time to a monotonic reference
            # while retaining the original age, so freshness checks include
            # executor queue delay and do not bless late callbacks.
            sample_monotonic = time.monotonic() - max(0.0, received_age)
            with self.gazebo_history_lock:
                if epoch != self.gazebo_history_epoch:
                    self.gazebo_pose_history.clear()
                    self.gazebo_history_epoch = epoch
            self.gazebo_pose = pose
            self.gazebo_velocity = velocity
            self.gazebo_pose_sample_monotonic = sample_monotonic
            self.gazebo_pose_sim_s = simulation_time_s
            sample = {
                'simulation_time_s': simulation_time_s,
                'pose': pose,
                'velocity': velocity,
                'wall_monotonic_s': sample_monotonic,
                'received_timestamp_ns': received_timestamp_ns,
                'source_timestamp_ns': source_timestamp_ns,
                'timestamp_evidence': source_time,
            }
            with self.gazebo_history_lock:
                self.gazebo_pose_history = insert_gazebo_pose_sample(
                    self.gazebo_pose_history, sample,
                    max_samples=self.gazebo_pose_history.maxlen or 10000)
            self.gazebo_valid_sample_count += 1

    def _flush_pending_gazebo_samples(self, current_epoch=None):
        if not self.pending_gazebo_samples:
            return
        if current_epoch is None:
            _, current_epoch, _, _ = self.tf_observer.tf_snapshot()
        lower_ns, upper_ns = self.tf_observer.clock_source_timestamp_bounds()
        pending = list(self.pending_gazebo_samples)
        self.pending_gazebo_samples.clear()
        for sample in pending:
            source_ns = sample['source_timestamp_ns']
            if sample['epoch'] != current_epoch:
                self.gazebo_discarded_unaligned_sample_count += 1
                continue
            source_time = self.tf_observer.simulation_time_at_source_timestamp(source_ns)
            if source_time is None:
                if lower_ns is not None and source_ns < lower_ns:
                    self.gazebo_discarded_unaligned_sample_count += 1
                else:
                    self.pending_gazebo_samples.append(sample)
                continue
            self._record_aligned_gazebo_sample(
                sample['pose'], sample['velocity'], source_ns,
                sample['received_timestamp_ns'], source_time, current_epoch)

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
            self.map_sample_count += 1
            self.map_sample_monotonic = time.monotonic()

    def _navigation_map_cb(self, msg):
        if (msg.info.width > 0 and msg.info.height > 0
                and len(msg.data) == msg.info.width * msg.info.height):
            self.navigation_map = msg
            self.navigation_map_sample_count += 1
            self.navigation_map_sample_monotonic = time.monotonic()

    def _cmd_cb(self, msg, info=None):
        self._record_command(self.cmd_events, msg)
        gid = getattr(info, 'publisher_gid', None)
        self.cmd_publisher_gids.append(bytes(gid).hex() if gid is not None else None)

    @staticmethod
    def _record_command(events, msg):
        events.append((time.monotonic(), msg.linear.x, msg.linear.y, msg.angular.z))

    def _action_status_cb(self, msg):
        self.action_status_events.append((
            time.monotonic(), {bytes(item.goal_info.goal_id.uuid): int(item.status)
                               for item in msg.status_list},
        ))

    def pump(self, ws=None, timeout=0.02):
        self.motion_executor.spin_once(timeout_sec=timeout)
        try:
            if ws is not None:
                try:
                    raw = ws.recv()
                except websocket.WebSocketTimeoutException:
                    raw = None
                except websocket.WebSocketConnectionClosedException as exc:
                    self.ws_errors.append(f'websocket_closed:{exc}')
                    self.operator_ws_closed = True
                    raw = None
                except (ConnectionResetError, BrokenPipeError, OSError) as exc:
                    self.ws_errors.append(f'websocket_transport_error:{type(exc).__name__}:{exc}')
                    self.operator_ws_closed = True
                    raw = None
                if raw:
                    try:
                        message = json.loads(raw)
                    except (TypeError, json.JSONDecodeError):
                        self.ws_errors.append('invalid_websocket_json')
                        message = None
                    if isinstance(message, dict):
                        self.ws_messages.append(message)
                        if message.get('type') == 'RUNTIME_STATUS':
                            self.runtime_status = message
                        elif message.get('type') == 'NAV_STATUS':
                            self.nav_statuses.append(message)
                        elif message.get('type') == 'ROBOT_CONTROL_STATUS':
                            self.control_statuses.append(message)
                        elif message.get('type') == 'COMMAND_DIAGNOSTICS':
                            self.command_diagnostics.append((time.monotonic(), message))
                        elif message.get('type') == 'ERROR':
                            self.ws_errors.append(
                                f'{message.get("code", "ERROR")}:{message.get("message", "")}'
                            )
        finally:
            self.record_navigation_sample()

    def record_navigation_sample(self, *, force=False):
        if not self.navigation_trace_enabled:
            return
        monotonic_now = time.monotonic()
        if not force and monotonic_now - self.navigation_trace_last_sample < 0.10:
            return
        if self.navigation_trace_started is None:
            self.navigation_trace_started = monotonic_now
        map_sample, gazebo_sample = self.map_gazebo_pose_pair()
        gazebo_pose = ((gazebo_sample or {}).get('pose')
                       if map_sample is not None else None)
        gazebo_age = (gazebo_sample or {}).get('sample_age_wall_s')
        def latest(events):
            return list(events[-1][1:]) if events else None
        self.navigation_trace.append({
            'elapsed_wall_s': round(monotonic_now - self.navigation_trace_started, 3),
            'observer_simulation_time_s': self.sim_time(),
            'simulation_time_s': ((map_sample or {}).get('pose_pair_timestamp_sim_s')
                                  if map_sample is not None else None),
            'map_tf': map_sample,
            'gazebo_pose': gazebo_pose,
            'gazebo_velocity': self.gazebo_velocity,
            'gazebo_sample_age_wall_s': gazebo_age,
            'odom_velocity': self.odom_velocity,
            'nav_command': latest(self.nav_cmd_events),
            'selected_command': latest(self.selected_cmd_events),
            'command_owner': latest(self.command_owner_events),
        })
        self.navigation_trace_last_sample = monotonic_now
    def wait_until(self, predicate, timeout, ws=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump(ws, min(0.05, max(0.001, deadline - time.monotonic())))
            if predicate():
                return True
        return bool(predicate())

    def sim_time(self) -> float:
        _, _epoch, observer_time, _clock_wall = self.tf_observer.tf_snapshot()
        if observer_time is not None:
            return float(observer_time)
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_mechanical_settling(self, ws=None, timeout=45.0,
                                body_tolerance=0.02, wheel_position_rate=0.005,
                                steering_tolerance=0.03, stable_window=0.8,
                                wall_timeout=180.0, clock_stall_timeout=5.0,
                                body_position_rate=0.001):
        """Timeout/window are simulation seconds; wall time is only a watchdog.

        Wheel position range, not a possibly inconsistent instantaneous velocity,
        proves actual rotation. Body pose drift is checked in addition to twists.
        """
        if hasattr(self, 'wheel_radius') and self.wheel_radius is None:
            client = self.create_client(GetParameters, '/swerve_controller/get_parameters')
            try:
                if not client.wait_for_service(timeout_sec=3.0):
                    return {'passed': False, 'reason': 'SETTLING_WHEEL_GEOMETRY_UNAVAILABLE'}
                request = GetParameters.Request()
                request.names = ['wheel_radius']
                future = client.call_async(request)
                if not self.wait_until(future.done, 5.0, ws):
                    return {'passed': False, 'reason': 'SETTLING_WHEEL_GEOMETRY_TIMEOUT'}
                radius = future.result().values[0].double_value
                if not math.isfinite(radius) or radius <= 0:
                    return {'passed': False, 'reason': 'SETTLING_WHEEL_GEOMETRY_INVALID'}
                self.wheel_radius = radius
            finally:
                self.destroy_client(client)
        monitor = MechanicalSettling(self.sim_time(), time.monotonic(),
            timeout_sim=timeout, timeout_wall=wall_timeout,
            clock_stall_wall=clock_stall_timeout, window_sim=stable_window,
            body_tolerance=body_tolerance, wheel_position_rate=wheel_position_rate,
            body_position_rate=body_position_rate, steering_span=steering_tolerance,
            wheel_radius=getattr(self, 'wheel_radius', None))
        while True:
            self.pump(ws, 0.005)
            sample = None
            if (self.joint_state and self.selected_cmd_events and self.drive_events
                    and self.steering_events and self.odom_sample_monotonic is not None
                    and self.gazebo_pose_sample_monotonic is not None):
                joints = self.joint_state
                sample = {
                    'selected': list(self.selected_cmd_events[-1][1:]),
                    'drive': list(self.drive_events[-1][1:]),
                    'steering_targets': list(self.steering_events[-1][1:]),
                    'wheel_positions': [joints['positions'].get(name, math.nan)
                        for name in ('wheel_front_drive_joint', 'wheel_rear_drive_joint')],
                    'wheel_velocities': [joints['velocities'].get(name, math.nan)
                        for name in ('wheel_front_drive_joint', 'wheel_rear_drive_joint')],
                    'steering_positions': [joints['positions'].get(name, math.nan)
                        for name in ('steer_front_joint', 'steer_rear_joint')],
                    'body_velocity': self.gazebo_velocity, 'odom_velocity': self.odom_velocity,
                    'body_pose': self.gazebo_pose,
                    'source_wall_times': [joints['monotonic_s'], self.selected_cmd_events[-1][0],
                        self.drive_events[-1][0], self.steering_events[-1][0],
                        self.odom_sample_monotonic, self.gazebo_pose_sample_monotonic],
                }
            result = monitor.update(self.sim_time(), time.monotonic(), sample)
            if result is not None:
                return {**result, 'duration_s': result['settling_wall_seconds']}

    def map_pose(self):
        sample = self.map_pose_sample()
        return sample['pose'] if sample and sample.get('fresh') else None

    def map_pose_sample(self, at_sim_time_s=None):
        tf, generation, clock_time_s, clock_received = self.tf_observer.tf_snapshot()
        if clock_time_s is None or clock_received is None:
            return None
        try:
            map_to_odom_latest = tf.lookup_transform('map', 'odom', rclpy.time.Time())
            odom_to_base_latest = tf.lookup_transform(
                'odom', 'base_footprint', rclpy.time.Time())

            def stamp_seconds(stamped):
                stamp = stamped.header.stamp
                return int(stamp.sec) + int(stamp.nanosec) * 1e-9

            map_latest_s = stamp_seconds(map_to_odom_latest)
            odom_latest_s = stamp_seconds(odom_to_base_latest)
            target_time_s = common_tf_sample_time(
                map_latest_s, odom_latest_s, at_sim_time_s)
            if target_time_s is None:
                return None
            target_time = rclpy.time.Time(
                nanoseconds=round(target_time_s * 1_000_000_000),
                clock_type=ClockType.ROS_TIME)
            timeout = Duration(seconds=TF_LOOKUP_TIMEOUT_S)
            map_to_odom = tf.lookup_transform(
                'map', 'odom', target_time, timeout=timeout)
            odom_to_base = tf.lookup_transform(
                'odom', 'base_footprint', target_time, timeout=timeout)
            map_to_base = tf.lookup_transform(
                'map', 'base_footprint', target_time, timeout=timeout)
            now_ns = round(float(clock_time_s) * 1_000_000_000)

            def transform_sample(stamped, *, source_stamp_s=None):
                transform = stamped.transform
                pose = (float(transform.translation.x), float(transform.translation.y),
                        yaw_from_quaternion(transform.rotation))
                stamp_ns = round(stamp_seconds(stamped) * 1_000_000_000)
                return {
                    'pose': pose,
                    'stamp_sim_s': stamp_ns / 1_000_000_000,
                    'age_sim_s': (now_ns - stamp_ns) / 1_000_000_000,
                    'parent_frame': stamped.header.frame_id,
                    'child_frame': stamped.child_frame_id,
                    **({'source_latest_stamp_sim_s': source_stamp_s,
                        'source_latest_age_sim_s': clock_time_s - source_stamp_s}
                       if source_stamp_s is not None else {}),
                }

            map_odom_sample = transform_sample(map_to_odom, source_stamp_s=map_latest_s)
            odom_base_sample = transform_sample(odom_to_base, source_stamp_s=odom_latest_s)
            common_sample = transform_sample(map_to_base)
            latest_skew = abs(map_latest_s - odom_latest_s)
            sampled_component_skew = abs(
                map_odom_sample['stamp_sim_s'] - odom_base_sample['stamp_sim_s'])
            pose = common_sample['pose']
            composed_pose = compose_planar_transforms(
                map_odom_sample['pose'], odom_base_sample['pose'])
            component_frames_valid = (
                map_odom_sample['parent_frame'].lstrip('/') == 'map'
                and map_odom_sample['child_frame'].lstrip('/') == 'odom'
                and odom_base_sample['parent_frame'].lstrip('/') == 'odom'
                and odom_base_sample['child_frame'].lstrip('/') == 'base_footprint'
            )
            latest_source_ages = (clock_time_s - map_latest_s,
                                  clock_time_s - odom_latest_s)
            sources_fresh = all(transform_is_fresh(age) for age in latest_source_ages)
            common_age = clock_time_s - target_time_s
            clock_wall_age = time.monotonic() - clock_received
            observer_unchanged = self.tf_observer.tf_snapshot()[1] == generation
            sampled_at_requested_time = (
                abs(map_odom_sample['stamp_sim_s'] - target_time_s) <= 1e-6
                and abs(odom_base_sample['stamp_sim_s'] - target_time_s) <= 1e-6
                and abs(common_sample['stamp_sim_s'] - target_time_s) <= 1e-6
            )
            fresh = bool(component_frames_valid and sampled_at_requested_time
                         and transform_is_fresh(common_age)
                         and clock_wall_age <= MAX_SIM_CLOCK_WALL_AGE_S
                         and observer_unchanged)
            return {
                'pose': pose,
                'method': 'tf_lookup_at_latest_common_sim_time',
                'fresh': fresh,
                'frame_ids_valid': component_frames_valid,
                'sampled_at_requested_time': sampled_at_requested_time,
                'component_time_skew_sim_s': sampled_component_skew,
                'latest_component_stamp_skew_sim_s': latest_skew,
                'latest_component_sources_fresh': sources_fresh,
                'latest_component_source_ages_sim_s': list(latest_source_ages),
                'stamp_sim_s': target_time_s,
                'age_sim_s': common_age,
                'simulation_clock_age_wall_s': clock_wall_age,
                'tf_epoch_generation': generation,
                'map_to_odom': map_odom_sample,
                'odom_to_base': odom_base_sample,
                'map_to_base_at_common_time': common_sample,
                'component_composition_error_m': dist(pose, composed_pose),
                'component_composition_yaw_error_rad': abs(wrap_angle(
                    pose[2] - composed_pose[2])),
                'latest_common_time': common_sample,
                'latest_source_stamps_sim_s': {
                    'map_to_odom': map_latest_s,
                    'odom_to_base': odom_latest_s,
                },
            }
        except (TransformException, RuntimeError):
            return None

    def gazebo_pose_at(self, simulation_time_s):
        """Interpolate independent Gazebo ModelStates at a TF simulation stamp."""
        if not math.isfinite(float(simulation_time_s)):
            return None
        target = float(simulation_time_s)
        with self.gazebo_history_lock:
            history = list(self.gazebo_pose_history)
        if not history:
            return None
        result = interpolate_gazebo_pose(history, target)
        if result is None:
            return None
        contributing = [sample for sample in history
                        if sample['simulation_time_s'] in result['source_stamps_sim_s']]
        received_ns = [int(sample['received_timestamp_ns']) for sample in contributing
                       if sample.get('received_timestamp_ns')]
        result['sample_age_wall_s'] = (
            max(0.0, (time.time_ns() - max(received_ns)) * 1e-9) if received_ns else None)
        result['source_timestamps_ns'] = [sample.get('source_timestamp_ns')
                                          for sample in contributing]
        result['clock_interpolation_evidence'] = [sample.get('timestamp_evidence')
                                                  for sample in contributing]
        return result

    def gazebo_observer_diagnostics(self):
        with self.gazebo_history_lock:
            latest = self.gazebo_pose_history[-1] if self.gazebo_pose_history else None
        receive_age = None
        if latest and latest.get('received_timestamp_ns'):
            receive_age = (time.time_ns() - int(latest['received_timestamp_ns'])) * 1e-9
        source_bounds = self.tf_observer.clock_source_timestamp_bounds()
        _, epoch, clock_sim_s, _ = self.tf_observer.tf_snapshot()
        return {
            'clock_epoch': epoch,
            'clock_sim_s': clock_sim_s,
            'raw_model_states_callbacks': self.gazebo_pose_count,
            'aligned_model_states_samples': self.gazebo_valid_sample_count,
            'unaligned_model_states_callbacks': self.gazebo_unaligned_sample_count,
            'discarded_unaligned_model_states_samples': self.gazebo_discarded_unaligned_sample_count,
            'pending_model_states_samples': len(self.pending_gazebo_samples),
            'last_unaligned_sample': self.gazebo_unaligned_last,
            'clock_source_timestamp_bounds_ns': source_bounds,
            'latest_aligned_sim_time_s': (latest.get('simulation_time_s') if latest else None),
            'latest_aligned_received_age_wall_s': receive_age,
            'latest_aligned_source_timestamp_ns': (latest.get('source_timestamp_ns')
                                                   if latest else None),
        }

    def map_gazebo_pose_pair(self, minimum_stamp_s=None):
        latest = self.map_pose_sample()
        with self.gazebo_history_lock:
            latest_gazebo = (self.gazebo_pose_history[-1]
                             if self.gazebo_pose_history else None)
        if latest is None or latest_gazebo is None:
            return None, None
        target = common_pose_pair_stamp(
            latest['stamp_sim_s'], latest_gazebo['simulation_time_s'], minimum_stamp_s)
        if target is None:
            return None, None
        tf_sample = self.map_pose_sample(at_sim_time_s=target)
        gazebo_sample = self.gazebo_pose_at(target)
        if tf_sample is None or gazebo_sample is None:
            return None, None
        tf_sample['pose_pair_timestamp_sim_s'] = target
        tf_sample['pose_pair_time_skew_sim_s'] = abs(
            tf_sample['stamp_sim_s'] - gazebo_sample['simulation_time_s'])
        tf_sample['gazebo_sample'] = gazebo_sample
        return tf_sample, gazebo_sample

    def destroy_node(self):
        executor = getattr(self, 'motion_executor', None)
        if executor is not None:
            executor.remove_node(self)
        self.tf_observer.close()
        super().destroy_node()

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
        selected_index = len(self.selected_cmd_events)
        owner_index = len(self.command_owner_events)
        deadline = start_time + duration_wall_s
        while time.monotonic() < deadline:
            self.pump(timeout=min(0.02, deadline - time.monotonic()))
        events = self.cmd_events[start_index:]
        nonzero = [row for row in events if any(abs(value) > 1e-4 for value in row[1:])]
        selected_events = self.selected_cmd_events[selected_index:]
        selected_nonzero = [row for row in selected_events
                            if any(abs(value) > 1e-4 for value in row[1:])]
        observed_owners = self.command_owner_events[owner_index:]
        selected_owner = (observed_owners[-1][1] if observed_owners else
                          self.command_owner_events[-1][1] if self.command_owner_events else None)
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
            'selected_samples': len(selected_events),
            'selected_nonzero_samples': len(selected_nonzero),
            'selected_velocity_zero': bool(selected_events) and not selected_nonzero,
            'selected_owner': selected_owner,
            'selected_owner_none_seen': any(owner == 'NONE' for _, owner in observed_owners),
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

    def stop_direct(self, timeout=1.5, *, nav_source=False):
        start_time = time.monotonic()
        start_index = len(self.drive_events)
        stop_until = start_time + timeout
        last_publish = 0.0
        zero_seen = False
        while time.monotonic() < stop_until:
            now = time.monotonic()
            if now - last_publish >= 0.10:
                (self.nav_command if nav_source else self.command).publish(Twist())
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
        selected_index = len(self.selected_cmd_events)
        owner_index = len(self.command_owner_events)
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
            watchdog_start_wall = time.monotonic()
            watchdog_selected_index = len(self.selected_cmd_events)
            watchdog_cmd_index = len(self.cmd_events)
            watchdog_zero_seen = False
            # The arbiter lease is measured in wall time, while the controller
            # update loop uses Gazebo time.  Observe the selected command after
            # the lease expires; waiting for a zero drive-controller sample
            # here makes this gate depend on the VM's simulation RTF.
            watchdog_lease_s = 0.5
            watchdog_expired_at = command_send_times[-1] + watchdog_lease_s + 0.10
            while time.monotonic() - watchdog_start_wall < 2.5:
                self.pump(timeout=0.02)
            selected_after_lease = [
                row for row in self.selected_cmd_events[watchdog_selected_index:]
                if row[0] >= watchdog_expired_at
            ]
            watchdog_zero_seen = bool(selected_after_lease) and all(
                all(abs(value) <= 1e-4 for value in row[1:])
                for row in selected_after_lease)
            watchdog_result = {
                'tested': True,
                'command_timeout_config_s': 0.5,
                'zero_selected_command_seen': watchdog_zero_seen,
                'unexpected_cmd_vel_samples_after_publisher_stopped':
                    len(self.cmd_events[watchdog_cmd_index:]),
                'delay_after_last_command_publish_wall_s':
                    max(0.0, time.monotonic() - command_send_times[-1])
                    if watchdog_zero_seen else None,
                'lease_timeout_wall_s': watchdog_lease_s,
                'waited_wall_s': time.monotonic() - watchdog_start_wall,
                'wait_wall_s': time.monotonic() - watchdog_start_wall,
            }
        stop_result = self.stop_direct()
        pose1, gazebo1 = self.odom, self.gazebo_pose
        joint1 = self.joint_snapshot(self.joint_state)
        recent_cmd = self.cmd_events[command_index:]
        active_cmd = [row for row in recent_cmd
                      if any(abs(value) > 1e-4 for value in row[1:])]
        selected_cmd = self.selected_cmd_events[selected_index:]
        active_selected_cmd = [row for row in selected_cmd
                               if any(abs(value) > 1e-4 for value in row[1:])]
        owners = self.command_owner_events[owner_index:]
        direct_owner_seen = any(owner == 'DIRECT_MANUAL' for _, owner in owners)
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
        if label == 'STRAFE':
            steering_response = steering_active and steering_position_change > 0.10
        elif label == 'ROTATE':
            # Pivot rotation may start while the modules are already aligned
            # tangentially (for example, after the preceding strafe). Requiring
            # a steering-angle *change* would reject a valid measured turn.
            # Instead verify the observed wheel alignment and physical rotation.
            steering_positions = (joint1 or {}).get('positions', {})
            pivot_angles = [steering_positions.get(name) for name in
                            ('steer_front_joint', 'steer_rear_joint')]
            steering_response = (steering_active and len(pivot_angles) == 2
                and all(value is not None and
                        abs(abs(wrap_angle(float(value))) - math.pi / 2.0) <= 0.10
                        for value in pivot_angles))
        else:
            steering_response = True
        passed = bool(
            elapsed_sim > 0.0 and direction_passed and active_cmd and active_selected_cmd
            and direct_owner_seen and drive_active
            and (wheel_velocity_max > 0.05 or wheel_position_change > 0.05)
            and steering_response and stop_result['zero_drive_command_seen']
            and (label != 'FORWARD' or watchdog_result.get('zero_selected_command_seen')))
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
            'selected_cmd_vel_nonzero_samples': len(active_selected_cmd),
            'command_owner': 'DIRECT_MANUAL' if direct_owner_seen else None,
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

    def map_goal_candidate(self, pose=None):
        if self.map is None:
            return None, 'map_message_unavailable'
        pose = pose if pose is not None else self.map_pose()
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
        map_reference, gazebo_reference = self.map_gazebo_pose_pair()
        pose0 = (map_reference or {}).get('pose')
        gazebo0 = (gazebo_reference or {}).get('pose')
        gazebo_reference_age = (gazebo_reference or {}).get('sample_age_wall_s')
        pair_skew = (map_reference or {}).get('pose_pair_time_skew_sim_s')
        if (pose0 is None or not map_reference.get('fresh') or gazebo0 is None
                or gazebo_reference_age is None or gazebo_reference_age > 1.0
                or pair_skew is None or pair_skew > MAX_TF_COMPONENT_SKEW_SIM_S):
            return {'passed': False, 'server_ready': True,
                    'reason': 'fresh_synchronized_map_tf_and_gazebo_pose_pair_unavailable_before_goal',
                    'map_tf_start': map_reference, 'gazebo_pose_start': gazebo0,
                    'gazebo_pose_age_start_wall_s': gazebo_reference_age,
                    'map_tf_gazebo_skew_sim_s': pair_skew}
        goal_pose, goal_error = self.map_goal_candidate(pose0)
        if goal_pose is None:
            return {'passed': False, 'server_ready': True, 'reason': goal_error}

        x, y, yaw = goal_pose
        current_world = self.gazebo_pose
        if (current_world is None or dist(gazebo0, current_world) > 0.01
                or abs(wrap_angle(gazebo0[2] - current_world[2])) > 0.01):
            return {'passed': False, 'server_ready': True,
                    'reason': 'robot_moved_while_navigation_start_pose_was_confirmed',
                    'goal': goal_pose, 'map_tf_start': map_reference,
                    'gazebo_pose_start': gazebo0, 'gazebo_pose_before_goal': current_world}
        command_index = len(self.nav_cmd_events)
        selected_index = len(self.selected_cmd_events)
        owner_index = len(self.command_owner_events)
        drive_index = len(self.drive_events)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Capture the complete direct Nav2 run on the independent TF observer.
        # The terminal pair alone cannot show whether map->odom drifted
        # gradually, jumped during scan matching, or became misaligned only
        # after the action completed.
        self.navigation_trace = []
        self.navigation_trace_started = None
        self.navigation_trace_last_sample = 0.0
        self.navigation_trace_enabled = True
        deadline = time.monotonic() + timeout
        goal_future = self.navigation.send_goal_async(goal)
        while time.monotonic() < deadline and not goal_future.done():
            self.pump(timeout=0.03)
        if not goal_future.done():
            self.stop_direct(nav_source=True)
            self.navigation_trace_enabled = False
            return {'passed': False, 'server_ready': True,
                    'reason': 'goal_acceptance_timeout:/navigate_to_pose',
                    'goal': goal_pose}
        goal_handle = goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.stop_direct(nav_source=True)
            self.navigation_trace_enabled = False
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

        self.record_navigation_sample(force=True)
        stop_result = self.stop_direct(nav_source=True)
        settling = self.wait_mechanical_settling(timeout=20.0, wall_timeout=90.0)
        tf_usable_after_settling = self.wait_until(
            lambda: transform_is_confirmable_when_settled(self.map_pose_sample()), 3.0)
        fresh_gazebo_after_settling = self.wait_until(
            lambda: (self.gazebo_pose_sample_monotonic is not None
                     and time.monotonic() - self.gazebo_pose_sample_monotonic <= 1.0), 3.0)
        self.record_navigation_sample(force=True)
        self.navigation_trace_enabled = False
        map_terminal_sample, gazebo_terminal_sample = self.map_gazebo_pose_pair()
        pose1 = (map_terminal_sample['pose']
                 if transform_is_confirmable_when_settled(map_terminal_sample) else None)
        gazebo1 = (gazebo_terminal_sample or {}).get('pose')
        ground_truth_map_pose = (map_pose_from_world(pose0, gazebo0, gazebo1)
                                 if gazebo1 is not None else None)
        ground_truth_goal_error = (dist(ground_truth_map_pose, goal_pose)
                                   if ground_truth_map_pose is not None else None)
        ground_truth_goal_yaw_error = (
            abs(wrap_angle(ground_truth_map_pose[2] - goal_pose[2]))
            if ground_truth_map_pose is not None else None)
        map_tf_vs_ground_truth_error_m = (
            dist(pose1, ground_truth_map_pose)
            if pose1 is not None and ground_truth_map_pose is not None else None)
        map_tf_vs_ground_truth_yaw_error_rad = (
            abs(wrap_angle(pose1[2] - ground_truth_map_pose[2]))
            if pose1 is not None and ground_truth_map_pose is not None else None)
        commands = self.nav_cmd_events[command_index:]
        active_commands = [row for row in commands
                           if any(abs(value) > 1e-4 for value in row[1:])]
        selected = self.selected_cmd_events[selected_index:]
        active_selected = [row for row in selected
                           if any(abs(value) > 1e-4 for value in row[1:])]
        owners = self.command_owner_events[owner_index:]
        nav_owner_seen = any(owner == 'NAV2' for _, owner in owners)
        drives = self.drive_events[drive_index:]
        drive_active = any(any(abs(value) > 1e-4 for value in row[1:]) for row in drives)
        pose_change = dist(pose0, pose1) if pose1 is not None else None
        physical_change = dist(gazebo0, gazebo1) if gazebo1 is not None else None
        final_goal_error = dist(pose1, goal_pose) if pose1 is not None else None
        final_yaw_error = (abs(wrap_angle(pose1[2] - goal_pose[2]))
                           if pose1 is not None else None)
        acceptance_checks = {
            'action_succeeded': terminal_status == 'SUCCEEDED',
            'navigation_cmd_vel_observed': bool(active_commands),
            'selected_navigation_cmd_vel_observed': bool(active_selected),
            'nav2_command_owner_observed': nav_owner_seen,
            'nonzero_drive_command_observed': drive_active,
            'map_tf_motion_observed': (pose_change is not None
                                       and pose_change > TRANSLATION_TOLERANCE_M),
            'gazebo_motion_observed': (physical_change is not None
                                       and physical_change > TRANSLATION_TOLERANCE_M),
            'map_tf_goal_translation_within_0_05m': (final_goal_error is not None
                and final_goal_error <= NAV_GOAL_XY_TOLERANCE_M),
            'map_tf_goal_yaw_within_0_05rad': (final_yaw_error is not None
                and final_yaw_error <= NAV_GOAL_YAW_TOLERANCE_RAD),
            'settling_confirmed': settling.get('passed') is True,
            'terminal_tf_fresh': tf_usable_after_settling,
            'terminal_gazebo_sample_fresh': fresh_gazebo_after_settling,
            'projected_gazebo_goal_translation_within_0_05m': (
                ground_truth_goal_error is not None
                and ground_truth_goal_error <= NAV_GOAL_XY_TOLERANCE_M),
            'projected_gazebo_goal_yaw_within_0_05rad': (
                ground_truth_goal_yaw_error is not None
                and ground_truth_goal_yaw_error <= NAV_GOAL_YAW_TOLERANCE_RAD),
            'map_tf_gazebo_translation_agreement_within_0_05m': (
                map_tf_vs_ground_truth_error_m is not None
                and map_tf_vs_ground_truth_error_m <= TRANSLATION_TOLERANCE_M),
            'map_tf_gazebo_yaw_agreement_within_0_05rad': (
                map_tf_vs_ground_truth_yaw_error_rad is not None
                and map_tf_vs_ground_truth_yaw_error_rad <= ROTATION_TOLERANCE_RAD),
            'zero_drive_command_after_stop': bool(stop_result.get('zero_drive_command_seen')),
        }
        failed_checks = failed_acceptance_conditions(acceptance_checks)
        passed = not failed_checks
        return {
            'passed': passed, 'server_ready': True, 'accepted': True, 'status': terminal_status,
            'reason': None if passed else (exact_reason or
                'acceptance_checks_failed:' + ','.join(failed_checks)),
            'acceptance_checks': acceptance_checks,
            'failed_acceptance_checks': failed_checks,
            'goal': goal_pose, 'p0': pose0, 'p1': pose1,
            'map_tf_start': map_reference,
            'map_tf_after_settling': map_terminal_sample,
            'gazebo_pose_pair_at_start': gazebo_reference,
            'gazebo_pose_pair_after_settling': gazebo_terminal_sample,
            'gazebo_pose_age_start_wall_s': gazebo_reference_age,
            'map_tf_gazebo_skew_start_sim_s': pair_skew,
            'gazebo_p0': gazebo0, 'gazebo_p1': gazebo1,
            'ground_truth_map_pose_after_settling': ground_truth_map_pose,
            'ground_truth_goal_error_m': ground_truth_goal_error,
            'ground_truth_goal_yaw_error_rad': ground_truth_goal_yaw_error,
            'map_tf_vs_ground_truth_error_m': map_tf_vs_ground_truth_error_m,
            'map_tf_vs_ground_truth_yaw_error_rad': map_tf_vs_ground_truth_yaw_error_rad,
            'settling': settling,
            'fresh_tf_after_settling': bool(map_terminal_sample and map_terminal_sample.get('fresh')),
            'tf_usable_after_settling': tf_usable_after_settling,
            'fresh_gazebo_after_settling': fresh_gazebo_after_settling,
            'displacement_m': pose_change, 'gazebo_displacement_m': physical_change,
            'final_goal_error_m': final_goal_error,
            'final_goal_yaw_error_rad': final_yaw_error,
            'goal_xy_tolerance_m': NAV_GOAL_XY_TOLERANCE_M,
            'goal_yaw_tolerance_rad': NAV_GOAL_YAW_TOLERANCE_RAD,
            'stop_result': stop_result,
            'navigation_trace_samples': self.navigation_trace,
            'cmd_vel_nonzero_samples': len(active_commands),
            'selected_cmd_vel_nonzero_samples': len(active_selected),
            'command_owner': 'NAV2' if nav_owner_seen else None,
            'drive_command_samples': len(drives), 'drive_nonzero': drive_active,
        }


def http_json(url: str, body=None, timeout=8.0):
    data = json.dumps(body).encode('utf-8') if body is not None else None
    headers = {'Content-Type': 'application/json'} if data is not None else {}
    request = urllib.request.Request(url, data=data, headers=headers,
                                     method='POST' if data is not None else 'GET')
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.loads(response.read().decode('utf-8'))
    except (OSError, ValueError, urllib.error.URLError) as exc:
        raise RuntimeError(f'{url} failed: {type(exc).__name__}: {exc}') from exc


def find_goal_status(probe, x, y, robot_id):
    relevant = [item for item in probe.nav_statuses
                if item.get('robot_id') == robot_id
                and abs(float(item.get('x', x)) - x) <= 1e-4
                and abs(float(item.get('y', y)) - y) <= 1e-4]
    return relevant[-1] if relevant else None


def set_mode(probe, ws, robot_id, mode, timeout=8.0):
    initial = len(probe.control_statuses)
    previous_request_ids = {
        str(item.get('request_id')) for item in probe.control_statuses[:initial]
        if item.get('robot_id') == robot_id and item.get('request_id')
    }
    request = {'request_ids': set(), 'applied_ids': set(), 'failure': None}
    ws.send(json.dumps({'type': 'ROBOT_MODE', 'robot_id': robot_id, 'mode': mode}))

    def matching_mode_request_applied():
        for item in probe.control_statuses[initial:]:
            if item.get('robot_id') != robot_id:
                continue
            if item.get('requested_mode', item.get('mode')) != mode:
                continue
            request_id = str(item.get('request_id') or '')
            if not request_id or request_id in previous_request_ids:
                continue
            if (item.get('accepted') is False
                    or item.get('mode_transition_state') == 'FAILED'):
                request['failure'] = str(item.get('reason') or 'mode_transition_rejected')
                return False
            if (item.get('accepted') is True
                    and item.get('mode_transition_state') == 'REQUESTED'):
                request['request_ids'].add(request_id)
            if (item.get('accepted') is True
                    and item.get('mode_transition_state') == 'APPLIED'
                    and item.get('applied_mode') == mode):
                request['applied_ids'].add(request_id)
        return bool(request['request_ids'] & request['applied_ids'])

    passed = probe.wait_until(matching_mode_request_applied, timeout, ws)
    reason = None
    if not passed:
        reason = (request['failure'] or
                  ('no_correlated_ROBOT_CONTROL_STATUS_APPLIED'
                   if request['request_ids'] or request['applied_ids']
                   else 'no_new_ROBOT_CONTROL_STATUS_request_id'))
    return passed, reason


def web_manual(probe, ws, robot_id, action, duration_sim_s=0.8, timeout=45.0):
    from manual_refresh import ManualRefreshWorker
    settling = probe.wait_mechanical_settling(ws, timeout=timeout)
    if not settling['passed']:
        return {'passed': False, 'reason': settling['reason'], 'settling': settling}
    sensors_ready = probe.wait_until(
        lambda: probe.odom is not None and probe.gazebo_pose is not None
        and probe.joint_state is not None, 10.0, ws)
    if not sensors_ready:
        return {'passed': False,
                'reason': 'missing_/odom_/model_states_or_/joint_states_before_manual_command'}
    pose0, gazebo0 = probe.odom, probe.gazebo_pose
    joint0 = probe.joint_snapshot(probe.joint_state)
    sim_start = probe.sim_time()
    manual_start_wall = time.monotonic()
    wall_deadline = time.monotonic() + timeout
    send_times = []
    command_index = len(probe.manual_cmd_events)
    selected_index = len(probe.selected_cmd_events)
    owner_index = len(probe.command_owner_events)
    drive_index = len(probe.drive_events)
    steering_index = len(probe.steering_events)
    joint_index = len(probe.joint_events)
    control_index = len(probe.control_statuses)
    sender = ManualRefreshWorker(lambda payload: ws.send(json.dumps(payload)), robot_id,
                                 WEB_COMMAND_REFRESH_WALL_S).start()
    try:
        sender.hold(action)
        while (probe.sim_time() - sim_start < duration_sim_s
               and time.monotonic() < wall_deadline):
            if sender.error:
                raise RuntimeError('manual refresh transport failed') from sender.error
            probe.pump(ws, 0.02)
        stop_time = time.monotonic()
    finally:
        sender.close()
    send_times = [row['T0'] for row in sender.events if row['action'] == action]
    zero_seen = probe.wait_until(
        lambda: any(row[0] >= stop_time and all(abs(value) <= 1e-4 for value in row[1:])
                    for row in probe.manual_cmd_events[command_index:]),
        min(5.0, max(1.0, timeout / 4.0)), ws,
    )
    stop_settling = probe.wait_mechanical_settling(ws, timeout=timeout)
    pose1 = probe.odom
    gazebo1 = probe.gazebo_pose
    joint1 = probe.joint_snapshot(probe.joint_state)
    commands = probe.manual_cmd_events[command_index:]
    active_commands = [row for row in commands if row[0] < stop_time
                       and any(abs(value) > 1e-4 for value in row[1:])]
    selected_commands = probe.selected_cmd_events[selected_index:]
    active_selected_commands = [row for row in selected_commands
                                if any(abs(value) > 1e-4 for value in row[1:])]
    owners = probe.command_owner_events[owner_index:]
    manual_owner_seen = any(owner == 'WEB_MANUAL' for _, owner in owners)
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
    wheel_response = wheel_velocity_max > 0.05 or wheel_position_change > 0.05
    # A busy Gazebo/ROS executor can deliver the controller-command topic
    # sample after the release timestamp even though the actuator state and
    # odometry prove it responded during the held lease. Preserve both raw
    # signals, but use measured wheel response as equivalent controller
    # evidence instead of producing a false negative from callback scheduling.
    actuator_response = drive_active or wheel_response
    action_statuses = probe.control_statuses[control_index:]
    command_ack = any(item.get('robot_id') == robot_id and item.get('accepted') is True
                      for item in action_statuses)
    component_index = {'linear_x': 1, 'linear_y': 2, 'angular_z': 3}[expected_component]
    command_direction_seen = any(
        row[component_index] * expected_sign > 0.01
        and all(abs(row[index]) <= 1e-3 for index in range(1, 4) if index != component_index)
        for row in active_selected_commands)
    direction_tolerance = (ROTATION_TOLERANCE_RAD if expected_component == 'angular_z'
                           else TRANSLATION_TOLERANCE_M)
    passed = bool(command_ack and command_direction_seen and active_selected_commands
                  and manual_owner_seen and actuator_response and wheel_response
                  and physical_metric is not None
                  and physical_metric > direction_tolerance
                  and odom_metric is not None and odom_metric > direction_tolerance * 0.5
                  and zero_seen and stop_settling['passed'])
    max_refresh_gap = max((later - earlier for earlier, later in zip(send_times, send_times[1:])),
                          default=None)
    connected_ids = ((probe.runtime_status or {}).get('connected_robot_ids') or [])
    runtime_status = probe.runtime_status or {}
    robot_map_sync = (runtime_status.get('robot_map_sync') or {}).get(robot_id) or {}
    local_active_map = (runtime_status.get('local_active_maps') or {}).get(robot_id) or {}

    def command_observations(events):
        selected = events[:4] + events[-4:] if len(events) > 8 else events
        return [{'elapsed_wall_s': round(row[0] - manual_start_wall, 3),
                 'linear_x': row[1], 'linear_y': row[2], 'angular_z': row[3]}
                for row in selected]

    return {
        'passed': passed,
        'settling': settling, 'stop_settling': stop_settling,
        'reason': None if passed else
            'missing_r01_bridge_ack_directional_cmd_controller_joint_physical_motion_or_stop_zero',
        'robot_id': robot_id, 'bridge_connected_robot_ids': connected_ids,
        'bridge_r01_connected': robot_id in connected_ids,
        'web_command': {'type': 'ROBOT_MANUAL', 'robot_id': robot_id, 'action': action},
        'p0': pose0, 'p1': pose1, 'displacement_m': pose_change,
        'gazebo_p0': gazebo0, 'gazebo_p1': gazebo1,
        'gazebo_body_delta': gazebo_delta, 'odom_body_delta': odom_delta,
        'cmd_vel_nonzero_samples': len(active_commands),
        'manual_source_topic_samples': len(commands),
        'selected_cmd_vel_nonzero_samples': len(active_selected_commands),
        'command_owner': 'WEB_MANUAL' if manual_owner_seen else None,
        'observed_cmd_vel': ([{'linear_x': row[1], 'linear_y': row[2],
                               'angular_z': row[3]} for row in active_commands[:5]]),
        'expected_direction_component': expected_component,
        'expected_direction_sign': expected_sign,
        'command_direction_seen': command_direction_seen,
        'physical_directional_delta': physical_metric,
        'odom_directional_delta': odom_metric,
        'direction_tolerance': direction_tolerance,
        'controller_command_ack': command_ack,
        'control_statuses': [{'accepted': item.get('accepted'),
                              'reason': item.get('reason'),
                              'mode': item.get('mode')}
                             for item in action_statuses[-8:]],
        'manual_source_observations': command_observations(commands),
        'selected_command_observations': command_observations(selected_commands),
        'runtime_map_status': {
            'status': runtime_status.get('map_sync_status'),
            'error': runtime_status.get('map_sync_error'),
            'robot': robot_map_sync,
            'local_active': local_active_map,
            'tf_status': runtime_status.get('tf_status'),
        },
        'drive_command_samples': len(drives), 'drive_nonzero': drive_active,
        'actuator_response': actuator_response,
        'actuator_response_source': 'DRIVE_COMMAND_TOPIC' if drive_active else
            'MEASURED_WHEEL_STATE' if wheel_response else None,
        'wheel_response': wheel_response,
        'wheel_velocity_max_rad_s': wheel_velocity_max,
        'wheel_position_change_rad': wheel_position_change,
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


def web_navigation(probe, ws, robot_id, timeout, minimum_pose_stamp_s=None,
                   verified_local_map_identity=None):
    if not probe.wait_until(lambda: probe.map is not None and probe.map_pose() is not None,
                            10.0, ws):
        return {'passed': False, 'reason': 'map_or_map_to_base_footprint_unavailable'}
    paired_start = {'tf': None, 'gazebo': None}

    def capture_fresh_start_pair():
        tf_sample, gazebo_sample = probe.map_gazebo_pose_pair(minimum_pose_stamp_s)
        if tf_sample is None or gazebo_sample is None:
            return False
        age = gazebo_sample.get('sample_age_wall_s')
        pair_skew = tf_sample.get('pose_pair_time_skew_sim_s')
        if (not tf_sample.get('fresh') or age is None or age > 1.0
                or pair_skew is None or pair_skew > MAX_TF_COMPONENT_SKEW_SIM_S):
            return False
        paired_start['tf'] = tf_sample
        paired_start['gazebo'] = gazebo_sample
        return True

    pair_ready = probe.wait_until(capture_fresh_start_pair, 5.0, ws)
    map_reference_sample = paired_start['tf']
    gazebo_reference_sample = paired_start['gazebo']
    pose0 = (map_reference_sample or {}).get('pose')
    gazebo0 = (gazebo_reference_sample or {}).get('pose')
    gazebo_reference_age_wall_s = (gazebo_reference_sample or {}).get('sample_age_wall_s')
    pair_skew = (map_reference_sample or {}).get('pose_pair_time_skew_sim_s')
    if (not pair_ready or pose0 is None or not map_reference_sample.get('fresh') or gazebo0 is None
            or gazebo_reference_age_wall_s is None or gazebo_reference_age_wall_s > 1.0
            or pair_skew is None or pair_skew > MAX_TF_COMPONENT_SKEW_SIM_S):
        return {'passed': False,
                'reason': ('fresh_synchronized_post_transition_map_tf_and_gazebo_pose_pair_unavailable'
                           if minimum_pose_stamp_s is not None else
                           'fresh_synchronized_map_tf_and_gazebo_pose_pair_unavailable_before_web_goal'),
                'map_tf_start': map_reference_sample, 'gazebo_pose_start': gazebo0,
                'gazebo_pose_age_start_wall_s': gazebo_reference_age_wall_s,
                'map_tf_gazebo_skew_start_sim_s': pair_skew,
                'minimum_pose_pair_stamp_sim_s': minimum_pose_stamp_s,
                'pose_observer': probe.gazebo_observer_diagnostics(),
                'latest_tf_sample': probe.map_pose_sample()}
    goal, goal_error = probe.map_goal_candidate(pose0)
    if goal is None:
        return {'passed': False, 'reason': goal_error}
    stable_map = {'signature': None, 'since_monotonic_s': None}

    def navigation_map_stable():
        signature = ready_navigation_map_signature(
            probe.runtime_status, robot_id,
            verified_local_map_identity=verified_local_map_identity)
        if signature is None:
            stable_map.update(signature=None, since_monotonic_s=None)
            return False
        if signature != stable_map['signature']:
            stable_map.update(signature=signature,
                              since_monotonic_s=time.monotonic())
            return False
        return time.monotonic() - stable_map['since_monotonic_s'] >= 2.0

    if not probe.wait_until(navigation_map_stable, min(30.0, max(10.0, timeout * 0.2)), ws):
        return {'passed': False,
                'reason': 'NAVIGATION_MAP_NOT_READY_OR_REGISTRATION_NOT_STABLE',
                'goal': goal,
                'navigation_map_status': (probe.runtime_status or {}).get('navigation_maps', {}).get(robot_id),
                'active_map_status': (probe.runtime_status or {}).get('local_active_maps', {}).get(robot_id),
                'last_stable_map_signature': stable_map['signature'],
                'map_signature_stable_for_wall_s': (
                    time.monotonic() - stable_map['since_monotonic_s']
                    if stable_map['since_monotonic_s'] is not None else 0.0)}
    # Readiness can take several wall seconds while a new registration is
    # published.  Re-sample after the map barrier so the start pose is fresh at
    # the point the operator could actually submit the goal.
    paired_start.update(tf=None, gazebo=None)
    pair_ready = probe.wait_until(capture_fresh_start_pair, 5.0, ws)
    map_reference_sample = paired_start['tf']
    gazebo_reference_sample = paired_start['gazebo']
    pose0 = (map_reference_sample or {}).get('pose')
    gazebo0 = (gazebo_reference_sample or {}).get('pose')
    gazebo_reference_age_wall_s = (gazebo_reference_sample or {}).get('sample_age_wall_s')
    pair_skew = (map_reference_sample or {}).get('pose_pair_time_skew_sim_s')
    if (not pair_ready or pose0 is None or not map_reference_sample.get('fresh')
            or gazebo0 is None or gazebo_reference_age_wall_s is None
            or gazebo_reference_age_wall_s > 1.0 or pair_skew is None
            or pair_skew > MAX_TF_COMPONENT_SKEW_SIM_S):
        return {'passed': False,
                'reason': 'fresh_synchronized_map_tf_and_gazebo_pose_pair_unavailable_after_map_readiness',
                'map_tf_start': map_reference_sample, 'gazebo_pose_start': gazebo0,
                'gazebo_pose_age_start_wall_s': gazebo_reference_age_wall_s,
                'map_tf_gazebo_skew_start_sim_s': pair_skew,
                'minimum_pose_pair_stamp_sim_s': minimum_pose_stamp_s,
                'pose_observer': probe.gazebo_observer_diagnostics(),
                'navigation_map_status': (probe.runtime_status or {}).get(
                    'navigation_maps', {}).get(robot_id),
                'active_map_status': (probe.runtime_status or {}).get(
                    'local_active_maps', {}).get(robot_id)}
    mode_ok, mode_error = set_mode(probe, ws, robot_id, 'AUTONOMOUS')
    if not mode_ok:
        return {'passed': False, 'reason': f'ROBOT_MODE_AUTONOMOUS_rejected:{mode_error}', 'goal': goal}

    x, y, yaw = goal
    request_id = str(uuid.uuid4())
    active_map = ((probe.runtime_status or {}).get('local_active_maps') or {}).get(robot_id) or {}
    if not active_map.get('active_map_id') or not active_map.get('active_map_revision'):
        return {'passed': False, 'reason': 'active_robot_map_identity_unavailable', 'goal': goal}
    preview_payload = {
        'type': 'PATH_PREVIEW_REQUEST', 'robot_id': robot_id,
        'request_id': request_id, 'x': x, 'y': y, 'yaw': yaw,
        'frame_id': 'map', 'active_map_id': active_map['active_map_id'],
        'active_map_revision': active_map['active_map_revision'],
    }
    ws.send(json.dumps(preview_payload))
    preview = None
    preview_deadline = time.monotonic() + min(30.0, max(10.0, timeout * 0.25))
    while time.monotonic() < preview_deadline:
        probe.pump(ws, 0.03)
        preview = next((message for message in reversed(probe.ws_messages)
                        if message.get('type') == 'PATH_PREVIEW_RESULT'
                        and message.get('robot_id') == robot_id
                        and message.get('request_id') == request_id), None)
        if preview is not None:
            break
    path = preview.get('path') if isinstance(preview, dict) else None
    preview_goal = preview.get('goal') if isinstance(preview, dict) else None
    try:
        preview_target_matches = isinstance(preview_goal, dict) and all(
            abs(float(preview_goal.get(axis, float('nan'))) - expected) <= 1e-4
            for axis, expected in (('x', x), ('y', y), ('yaw', yaw)))
    except (TypeError, ValueError):
        preview_target_matches = False
    preview_valid = bool(
        preview and str(preview.get('status') or '').upper() == 'VALID'
        and isinstance(path, list) and path
        and preview.get('active_map_id') == active_map['active_map_id']
        and str(preview.get('active_map_revision')) == str(active_map['active_map_revision'])
        and preview_target_matches
    )
    if not preview_valid:
        return {
            'passed': False,
            'reason': (preview or {}).get('reason', 'valid_nav2_path_preview_not_received'),
            'goal': goal, 'preview': preview, 'path_preview_request': preview_payload,
            'active_map': active_map,
            'runtime_map_status': {
                'status': (probe.runtime_status or {}).get('map_sync_status'),
                'error': (probe.runtime_status or {}).get('map_sync_error'),
                'robot': ((probe.runtime_status or {}).get('robot_map_sync') or {}).get(robot_id),
                'local_active': ((probe.runtime_status or {}).get('local_active_maps') or {}).get(robot_id),
                'tf_status': (probe.runtime_status or {}).get('tf_status'),
            },
        }

    _, gazebo_before_goal_sample = probe.map_gazebo_pose_pair()
    gazebo_before_goal = (gazebo_before_goal_sample or {}).get('pose')
    gazebo_before_goal_age_wall_s = (gazebo_before_goal_sample or {}).get('sample_age_wall_s')
    if (gazebo_before_goal is None or gazebo_before_goal_age_wall_s is None
            or gazebo_before_goal_age_wall_s > 1.0 or dist(gazebo0, gazebo_before_goal) > 0.01
            or abs(wrap_angle(gazebo0[2] - gazebo_before_goal[2])) > 0.01):
        probe.navigation_trace_enabled = False
        return {'passed': False, 'reason': 'robot_moved_while_navigation_preview_was_confirmed',
                'map_tf_start': map_reference_sample,
                'gazebo_pose_start': gazebo0,
                'gazebo_pose_before_goal': gazebo_before_goal,
                'gazebo_pose_age_before_goal_wall_s': gazebo_before_goal_age_wall_s}
    command_index = len(probe.nav_cmd_events)
    selected_index = len(probe.selected_cmd_events)
    owner_index = len(probe.command_owner_events)
    drive_index = len(probe.drive_events)
    action_status_index = len(probe.action_status_events)
    probe.navigation_trace = []
    probe.navigation_trace_started = None
    probe.navigation_trace_last_sample = 0.0
    probe.navigation_trace_enabled = True
    goal_payload = {
        'type': 'NAV_GOAL', 'robot_id': robot_id, 'x': x, 'y': y,
        'yaw': yaw, 'frame_id': 'map', 'preview_request_id': request_id,
        'active_map_id': active_map['active_map_id'],
        'active_map_revision': active_map['active_map_revision'],
        'map_id': preview.get('active_map_id'),
        'map_revision': preview.get('active_map_revision'),
        'source_type': preview.get('source_type') or 'MAP_POINT',
        'source_id': preview.get('source_id'),
        'source_map_id': preview.get('source_map_id'),
        'source_map_revision': preview.get('source_map_revision'),
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
    probe.record_navigation_sample(force=True)
    terminal_trace_index = len(probe.navigation_trace) - 1
    terminal_sample, terminal_gazebo_sample = probe.map_gazebo_pose_pair()
    terminal_pose = (terminal_sample['pose']
                     if terminal_sample and terminal_sample.get('fresh') else None)
    terminal_gazebo = (terminal_gazebo_sample or {}).get('pose')
    terminal_gazebo_age_wall_s = (terminal_gazebo_sample or {}).get('sample_age_wall_s')
    terminal_goal_error = dist(terminal_pose, goal) if terminal_pose is not None else None
    terminal_goal_yaw_error = (abs(wrap_angle(terminal_pose[2] - goal[2]))
                               if terminal_pose is not None else None)

    # Nav2 can report its terminal result just before wheel/chassis motion has
    # settled. Pair the authoritative map-frame TF with Gazebo ground truth,
    # projected through the alignment measured at this goal's start.
    settling = probe.wait_mechanical_settling(ws, timeout=20.0, wall_timeout=90.0)
    tf_usable_after_settling = probe.wait_until(
        lambda: transform_is_confirmable_when_settled(probe.map_pose_sample()), 3.0, ws)
    fresh_gazebo_after_settling = probe.wait_until(
        lambda: (probe.gazebo_pose_sample_monotonic is not None
                 and time.monotonic() - probe.gazebo_pose_sample_monotonic <= 1.0), 3.0, ws)
    probe.record_navigation_sample(force=True)
    probe.navigation_trace_enabled = False

    settled_sample, settled_gazebo_sample = probe.map_gazebo_pose_pair()
    pose1 = (settled_sample['pose']
             if transform_is_confirmable_when_settled(settled_sample) else None)
    gazebo1 = (settled_gazebo_sample or {}).get('pose')
    gazebo_settled_age_wall_s = (settled_gazebo_sample or {}).get('sample_age_wall_s')
    ground_truth_map_pose = (map_pose_from_world(pose0, gazebo0, gazebo1)
                             if gazebo1 is not None else None)
    ground_truth_goal_error = (dist(ground_truth_map_pose, goal)
                               if ground_truth_map_pose is not None else None)
    ground_truth_goal_yaw_error = (
        abs(wrap_angle(ground_truth_map_pose[2] - goal[2]))
        if ground_truth_map_pose is not None else None)
    map_tf_vs_ground_truth_error_m = (
        dist(pose1, ground_truth_map_pose)
        if pose1 is not None and ground_truth_map_pose is not None else None)
    map_tf_vs_ground_truth_yaw_error_rad = (
        abs(wrap_angle(pose1[2] - ground_truth_map_pose[2]))
        if pose1 is not None and ground_truth_map_pose is not None else None)
    commands = probe.nav_cmd_events[command_index:]
    active_commands = [row for row in commands if any(abs(value) > 1e-4 for value in row[1:])]
    selected_commands = probe.selected_cmd_events[selected_index:]
    active_selected_commands = [row for row in selected_commands
                                if any(abs(value) > 1e-4 for value in row[1:])]
    owners = probe.command_owner_events[owner_index:]
    nav_owner_seen = any(owner == 'NAV2' for _, owner in owners)
    drives = probe.drive_events[drive_index:]
    drive_active = any(any(abs(value) > 1e-4 for value in row[1:]) for row in drives)
    pose_change = dist(pose0, pose1) if pose1 is not None else None
    physical_change = dist(gazebo0, gazebo1) if gazebo1 is not None else None
    goal_distance = dist(pose1, goal) if pose1 is not None else None
    goal_yaw_error = (abs(wrap_angle(pose1[2] - goal[2]))
                      if pose1 is not None else None)
    status_name = str(terminal.get('status') or 'UNKNOWN').upper()
    accepted = accepted or status_name == 'SUCCEEDED'
    acceptance_checks = {
        'goal_accepted': accepted,
        'action_succeeded': status_name == 'SUCCEEDED',
        'navigation_cmd_vel_observed': bool(active_commands),
        'selected_navigation_cmd_vel_observed': bool(active_selected_commands),
        'nav2_command_owner_observed': nav_owner_seen,
        'nonzero_drive_command_observed': drive_active,
        'map_tf_motion_observed': (pose_change is not None
                                   and pose_change > TRANSLATION_TOLERANCE_M),
        'gazebo_motion_observed': (physical_change is not None
                                   and physical_change > TRANSLATION_TOLERANCE_M),
        'settling_confirmed': settling.get('passed') is True,
        'terminal_tf_fresh': tf_usable_after_settling,
        'terminal_gazebo_sample_fresh': fresh_gazebo_after_settling,
        'map_tf_goal_translation_within_0_05m': (goal_distance is not None
            and goal_distance <= NAV_GOAL_XY_TOLERANCE_M),
        'map_tf_goal_yaw_within_0_05rad': (goal_yaw_error is not None
            and goal_yaw_error <= NAV_GOAL_YAW_TOLERANCE_RAD),
        'projected_gazebo_goal_translation_within_0_05m': (
            ground_truth_goal_error is not None
            and ground_truth_goal_error <= NAV_GOAL_XY_TOLERANCE_M),
        'projected_gazebo_goal_yaw_within_0_05rad': (
            ground_truth_goal_yaw_error is not None
            and ground_truth_goal_yaw_error <= NAV_GOAL_YAW_TOLERANCE_RAD),
        'map_tf_gazebo_translation_agreement_within_0_05m': (
            map_tf_vs_ground_truth_error_m is not None
            and map_tf_vs_ground_truth_error_m <= TRANSLATION_TOLERANCE_M),
        'map_tf_gazebo_yaw_agreement_within_0_05rad': (
            map_tf_vs_ground_truth_yaw_error_rad is not None
            and map_tf_vs_ground_truth_yaw_error_rad <= ROTATION_TOLERANCE_RAD),
    }
    failed_checks = failed_acceptance_conditions(acceptance_checks)
    passed = not failed_checks
    return {
        'passed': passed,
        'reason': None if passed else (str(terminal.get('reason')) if status_name != 'SUCCEEDED'
            else 'acceptance_checks_failed:' + ','.join(failed_checks)),
        'acceptance_checks': acceptance_checks,
        'failed_acceptance_checks': failed_checks,
        'accepted': accepted, 'status': status_name, 'goal': goal,
        'path_preview_request': preview_payload,
        'path_preview': {
            'status': preview.get('status'), 'request_id': request_id,
            'active_map_id': preview.get('active_map_id'),
            'active_map_revision': preview.get('active_map_revision'),
            'path_length_m': preview.get('path_length_m'),
            'path_points': len(path),
        },
        'runtime_map_status': {
            'status': (probe.runtime_status or {}).get('map_sync_status'),
            'error': (probe.runtime_status or {}).get('map_sync_error'),
            'robot': ((probe.runtime_status or {}).get('robot_map_sync') or {}).get(robot_id),
            'local_active': ((probe.runtime_status or {}).get('local_active_maps') or {}).get(robot_id),
            'tf_status': (probe.runtime_status or {}).get('tf_status'),
        },
        'web_payload': goal_payload,
        'ros_goal': {'x': goal[0], 'y': goal[1], 'yaw': goal[2], 'frame_id': 'map'},
        'goal_reason': terminal.get('reason'), 'p0': pose0, 'p1': pose1,
        'pose_at_terminal': terminal_pose, 'gazebo_pose_at_terminal': terminal_gazebo,
        'map_tf_sample_at_terminal': terminal_sample,
        'gazebo_pose_pair_at_terminal': terminal_gazebo_sample,
        'gazebo_pose_age_at_terminal_wall_s': terminal_gazebo_age_wall_s,
        'map_tf_start': map_reference_sample,
        'minimum_pose_pair_stamp_sim_s': minimum_pose_stamp_s,
        'map_tf_gazebo_skew_start_sim_s': pair_skew,
        'gazebo_pose_pair_at_start': gazebo_reference_sample,
        'gazebo_pose_before_goal': gazebo_before_goal,
        'gazebo_pose_sample_before_goal': gazebo_before_goal_sample,
        'gazebo_pose_age_before_goal_wall_s': gazebo_before_goal_age_wall_s,
        'gazebo_pose_start': gazebo0,
        'gazebo_pose_age_start_wall_s': gazebo_reference_age_wall_s,
        'map_tf_fresh_after_settling': bool(settled_sample and settled_sample.get('fresh')),
        'map_tf_usable_after_settling': tf_usable_after_settling,
        'gazebo_pose_fresh_after_settling': fresh_gazebo_after_settling,
        'gazebo_pose_age_after_settling_wall_s': gazebo_settled_age_wall_s,
        'map_tf_sample_after_settling': settled_sample,
        'gazebo_pose_pair_after_settling': settled_gazebo_sample,
        'ground_truth_map_pose_after_settling': ground_truth_map_pose,
        'ground_truth_goal_error_m': ground_truth_goal_error,
        'ground_truth_goal_yaw_error_rad': ground_truth_goal_yaw_error,
        'map_tf_vs_ground_truth_error_m': map_tf_vs_ground_truth_error_m,
        'map_tf_vs_ground_truth_yaw_error_rad': map_tf_vs_ground_truth_yaw_error_rad,
        'terminal_trace_sample': (probe.navigation_trace[terminal_trace_index]
                                  if 0 <= terminal_trace_index < len(probe.navigation_trace) else None),
        'navigation_trace_samples': probe.navigation_trace,
        'goal_error_at_terminal_m': terminal_goal_error,
        'goal_yaw_error_at_terminal_rad': terminal_goal_yaw_error,
        'gazebo_p0': gazebo0, 'gazebo_p1': gazebo1,
        'settling': settling, 'goal_pose_frame': 'map',
        'displacement_m': pose_change, 'gazebo_displacement_m': physical_change,
        'goal_distance_m': goal_distance, 'goal_yaw_error_rad': goal_yaw_error,
        'goal_xy_tolerance_m': NAV_GOAL_XY_TOLERANCE_M,
        'goal_yaw_tolerance_rad': NAV_GOAL_YAW_TOLERANCE_RAD,
        'cmd_vel_nonzero_samples': len(active_commands),
        'selected_cmd_vel_nonzero_samples': len(active_selected_commands),
        'command_owner': 'NAV2' if nav_owner_seen else None,
        'drive_command_samples': len(drives), 'drive_nonzero': drive_active,
        'action_status_topic_samples': len(probe.action_status_events[action_status_index:]),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend-url', required=True)
    parser.add_argument('--frontend-origin', default=os.environ.get('WARETWIN_FRONTEND_ORIGIN'))
    parser.add_argument('--robot-id', default='R01')
    parser.add_argument('--motion-timeout', type=float, default=45.0)
    parser.add_argument('--navigation-timeout', type=float, default=180.0)
    parser.add_argument('--manual-duration-sim', type=float, default=1.5)
    parser.add_argument('--json')
    args = parser.parse_args()

    result = {'stages': {}, 'passed': False}
    try:
        ws_url = args.backend_url.replace('https://', 'wss://').replace('http://', 'ws://') + '/ws'
        if not args.frontend_origin:
            raise RuntimeError('set --frontend-origin to the configured same-origin browser HMI origin')
        ws = websocket.create_connection(ws_url, origin=args.frontend_origin,
                                         timeout=5.0, enable_multithread=True)
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
        initial_pose_ready = probe.wait_until(
            lambda: probe.map_pose_sample() is not None and probe.gazebo_pose is not None
            and probe.map is not None and probe.map_sample_count > 0, 15.0, ws)
        initial_map_sample = probe.map_pose_sample() if initial_pose_ready else None
        result['initial_pose_map'] = initial_map_sample
        result['initial_pose_gazebo'] = probe.gazebo_pose if initial_pose_ready else None
        result['initial_pose_ready'] = initial_pose_ready
        idle_arbiter_passed = bool(
            idle['selected_velocity_zero']
            and idle['selected_owner'] == 'NONE'
            and idle['selected_owner_none_seen']
        )
        result['stages']['COMMAND_ARBITER_IDLE'] = idle_arbiter_passed
        print(f'COMMAND_ARBITER_IDLE={"PASS" if idle_arbiter_passed else "FAIL"} '
              f'selected_samples={idle["selected_samples"]} '
              f'selected_nonzero={idle["selected_nonzero_samples"]} '
              f'owner={idle["selected_owner"]}', flush=True)
        print(f'IDLE_CMD_VEL samples={idle["samples"]} nonzero={idle["nonzero_samples"]} '
              f'unexpected={idle["unexpected_idle_traffic"]}', flush=True)

        bridge_ready_for_direct = probe.wait_until(
            lambda: probe.runtime_status is not None
            and args.robot_id in probe.runtime_status.get('connected_robot_ids', []),
            15.0, ws,
        )
        result['stages']['R01_BRIDGE_READY_FOR_DIRECT_CONTROL'] = bridge_ready_for_direct
        manual_mode_ready = False
        if bridge_ready_for_direct:
            manual_mode_ready, mode_reason = set_mode(probe, ws, args.robot_id, 'MANUAL')
            result['stages']['DIRECT_MANUAL_MODE_R01'] = manual_mode_ready
            if not manual_mode_ready:
                result['direct_manual_mode_reason'] = mode_reason
        else:
            result['stages']['DIRECT_MANUAL_MODE_R01'] = False

        sequence = (
            ('FORWARD', (0.25, 0.0, 0.0)),
            ('STRAFE', (0.0, 0.25, 0.0)),
            ('ROTATE', (0.0, 0.0, 0.4)),
        )
        direct_results = {}
        direct_blocker = None
        if idle['unexpected_idle_traffic'] or not bridge_ready_for_direct or not manual_mode_ready:
            direct_blocker = 'DIRECT_FORWARD'
            result['stages']['DIRECT_FORWARD'] = None
            reason = ('unexpected_idle_cmd_vel_traffic' if idle['unexpected_idle_traffic']
                      else 'R01_bridge_unavailable' if not bridge_ready_for_direct
                      else 'MANUAL_mode_not_accepted')
            print(f'DIRECT_FORWARD=UNVERIFIED reason={reason}; '
                  'motion not commanded', flush=True)
        else:
            for label, velocity in sequence:
                motion = probe.direct_motion(label, velocity, timeout=args.motion_timeout)
                direct_results[label] = motion
                passed = bool(motion['passed'])
                stage_name = f'DIRECT_{label}'
                result['stages'][stage_name] = passed
                result['stages'][f'DIRECT_ROS_{label}'] = passed
                result['stages'][f'DIRECT_{label}_COMMAND_OWNER'] = (
                    motion.get('command_owner') == 'DIRECT_MANUAL')
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
        direct_nav_mode_ready = False
        if all_direct:
            direct_nav_mode_ready, direct_nav_mode_reason = set_mode(
                probe, ws, args.robot_id, 'AUTONOMOUS')
            result['stages']['DIRECT_AUTONOMOUS_MODE_R01'] = direct_nav_mode_ready
        else:
            direct_nav_mode_reason = f'blocked_by_{direct_blocker}'
        direct_navigation = ({'passed': False,
                              'reason': (f'AUTONOMOUS_mode_not_accepted:{direct_nav_mode_reason}'
                                         if all_direct else f'blocked_by_{direct_blocker}'),
                              'server_ready': None, 'accepted': None, 'status': 'UNVERIFIED'}
                             if not (all_direct and direct_nav_mode_ready) else
                             probe.direct_navigation(args.navigation_timeout))
        result['direct_navigation'] = direct_navigation
        if all_direct and direct_nav_mode_ready:
            direct_nav_stages = (
                ('DIRECT_NAV_SERVER_READY', bool(direct_navigation.get('server_ready'))),
                ('DIRECT_NAV_GOAL_ACCEPTED', bool(direct_navigation.get('accepted'))),
                ('DIRECT_NAV_CMD_VEL', direct_navigation.get('cmd_vel_nonzero_samples', 0) > 0),
                ('DIRECT_NAV_COMMAND_OWNER', direct_navigation.get('command_owner') == 'NAV2'),
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
                               'DIRECT_NAV_CMD_VEL', 'DIRECT_NAV_COMMAND_OWNER',
                               'DIRECT_NAV_CONTROLLER_COMMAND',
                               'DIRECT_NAV_MOTION', 'DIRECT_NAV_GOAL_TOLERANCE',
                               'DIRECT_NAV_SUCCEEDED'):
                result['stages'][stage_name] = None
            nav_reason = (f'AUTONOMOUS_mode_not_accepted:{direct_nav_mode_reason}'
                          if all_direct else f'blocked_by_{direct_blocker}')
            print(f'DIRECT_NAV_GOAL=UNVERIFIED reason={nav_reason}', flush=True)

        manual = {'passed': False, 'reason': f'blocked_by_{direct_blocker}',
                  'bridge_r01_connected': None}
        bridge_ready = probe.wait_until(
            lambda: probe.runtime_status is not None
            and args.robot_id in probe.runtime_status.get('connected_robot_ids', []),
            15.0, ws,
        )

        def active_robot_map_ready():
            status = probe.runtime_status or {}
            active = (status.get('local_active_maps') or {}).get(args.robot_id) or {}
            return (status.get('map_sync_status') in ('SYNCED', 'CANONICAL')
                    or active.get('map_sync_status') in ('SYNCED', 'CANONICAL', 'LOCAL_ONLY'))

        map_synced = probe.wait_until(active_robot_map_ready, 15.0, ws)
        result['stages']['ROS_BRIDGE_R01_WEBSOCKET_READY'] = bridge_ready
        print(f'ROS_BRIDGE_R01_WEBSOCKET_READY={"PASS" if bridge_ready else "FAIL reason=R01_not_connected_to_Django"} '
              f'connected_robot_ids={(probe.runtime_status or {}).get("connected_robot_ids")}', flush=True)
        result['stages']['WEB_MAP_SYNC_READY'] = map_synced
        print(f'WEB_MAP_SYNC_READY={"PASS" if map_synced else "FAIL reason=" + str((probe.runtime_status or {}).get("map_sync_error") or "active_robot_map_not_ready")}', flush=True)
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
                        probe, ws, args.robot_id, action,
                        duration_sim_s=max(0.2, args.manual_duration_sim),
                        timeout=args.motion_timeout)
                    manual_results[action] = action_result
                    result['stages'][f'WEB_MANUAL_{action}'] = bool(action_result['passed'])
                    print(f'WEB_MANUAL_{action}={"PASS" if action_result["passed"] else "FAIL"} '
                              f'reason={action_result.get("reason")} '
                              f'cmd_vel={action_result.get("observed_cmd_vel")} '
                              f'manual_source_samples={action_result.get("manual_source_topic_samples")} '
                              f'selected_samples={action_result.get("selected_cmd_vel_nonzero_samples")} '
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
            manual = {'passed': False, 'reason': 'R01_bridge_or_active_map_not_ready'}
        result['web_manual'] = manual
        for stage_name, passed, detail in (
            ('WEB_MANUAL_CMD_VEL', manual.get('selected_cmd_vel_nonzero_samples', 0) > 0,
             f'selected_samples={manual.get("selected_cmd_vel_nonzero_samples", 0)} '
             f'manual_source_topic_samples={manual.get("manual_source_topic_samples", 0)}'),
            ('WEB_MANUAL_CONTROLLER_COMMAND', bool(manual.get('actuator_response')),
             f'drive_samples={manual.get("drive_command_samples", 0)} '
             f'nonzero_topic={manual.get("drive_nonzero")} '
             f'actuator_source={manual.get("actuator_response_source")}'),
            ('WEB_MANUAL_COMMAND_OWNER', manual.get('command_owner') == 'WEB_MANUAL',
             f'owner={manual.get("command_owner")}'),
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

        manual_ok = bool(manual.get('passed'))
        navigation = {'passed': False, 'reason': 'blocked_by_failed_precondition',
                      'status': 'UNVERIFIED', 'accepted': None}
        if (bridge_ready and map_synced
                and result['stages'].get('WEB_MANUAL_STOP_ZERO') is not False):
            mode_ok, mode_reason = set_mode(probe, ws, args.robot_id, 'AUTONOMOUS')
            if not mode_ok:
                navigation['reason'] = f'AUTONOMOUS_mode_not_accepted:{mode_reason}'
                print(f'WEB_NAV_GOAL_R01=UNVERIFIED reason={navigation["reason"]}', flush=True)
                result['web_navigation'] = navigation
                mode_ok = False
            if mode_ok:
                navigation = web_navigation(probe, ws, args.robot_id, args.navigation_timeout)
        else:
            reason = ('web_manual_r01_failed' if not manual_ok
                      else 'R01_bridge_or_active_map_not_ready')
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
                ('WEB_NAV_COMMAND_OWNER', navigation.get('command_owner') == 'NAV2',
                 f'owner={navigation.get("command_owner")}'),
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
                               'WEB_NAV_COMMAND_OWNER', 'WEB_NAV_CONTROLLER_COMMAND', 'WEB_NAV_MOTION',
                               'WEB_NAV_GOAL_TOLERANCE', 'WEB_NAV_SUCCEEDED'):
                result['stages'][stage_name] = None

        all_web_manual = all(result['stages'].get(key) is True for key in (
            'WEB_MANUAL_CMD_VEL', 'WEB_MANUAL_CONTROLLER_COMMAND', 'WEB_MANUAL_COMMAND_OWNER',
            'WEB_MANUAL_COMMAND_ACCEPTED', 'WEB_MANUAL_MOTION', 'WEB_MANUAL_STOP_ZERO',
            'WEB_MANUAL_FORWARD', 'WEB_MANUAL_BACKWARD', 'WEB_MANUAL_LEFT',
            'WEB_MANUAL_RIGHT', 'WEB_MANUAL_ROTATE_LEFT', 'WEB_MANUAL_ROTATE_RIGHT'))
        all_web_nav = all(result['stages'].get(key) is True for key in (
            'WEB_NAV_GOAL_ACCEPTED', 'WEB_NAV_CMD_VEL', 'WEB_NAV_COMMAND_OWNER',
            'WEB_NAV_CONTROLLER_COMMAND',
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
        if args.json:
            with open(args.json, 'w', encoding='utf-8') as stream:
                json.dump(result, stream, indent=2)


if __name__ == '__main__':
    raise SystemExit(main())
