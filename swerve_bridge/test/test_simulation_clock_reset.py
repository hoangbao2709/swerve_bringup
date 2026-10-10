import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

from geometry_msgs.msg import Twist
from tf2_ros import TransformException

import swerve_bridge.bridge_node as bridge_module
from swerve_bridge.bridge_node import SwerveBridge


def make_bridge(simulation_time):
    bridge = object.__new__(SwerveBridge)
    bridge.simulation_time = simulation_time
    bridge.previous_simulation_time = simulation_time
    bridge.previous_simulation_wall = time.monotonic() - 0.1
    bridge.last_clock_monotonic = None
    bridge.gazebo_rtf = 0.5
    bridge.last_tf_error = None
    bridge.tf_buffer = Mock()
    bridge.tf_listener = Mock()
    bridge.tf_epoch = 0
    bridge.tf_epoch_reset_count = 0
    bridge.cmd_pub = Mock()
    bridge.manual_twist = Twist()
    bridge.manual_twist.linear.x = 0.25
    bridge.manual_deadline = time.monotonic() + 1.0
    bridge.pending_initial_pose = {'data': {'request_id': 'pose'}}
    bridge.pending_local_map_load = {'data': {'request_id': 'load'}}
    bridge.local_map_load_pending = True
    bridge._map_save_lock = threading.Lock()
    bridge.pending_map_save = {'data': {'request_id': 'save'}}
    bridge._send_local_control_result = Mock()
    return bridge


def clock_message(seconds, nanoseconds=0):
    return SimpleNamespace(clock=SimpleNamespace(sec=seconds, nanosec=nanoseconds))


def test_simulation_clock_rewind_invalidates_old_tf_and_pending_operations():
    bridge = make_bridge(60.0)
    new_buffer = Mock()
    new_listener = Mock()
    old_listener = bridge.tf_listener

    with patch.object(bridge_module, 'Buffer', return_value=new_buffer), \
            patch.object(bridge_module, 'TransformListener', return_value=new_listener):
        SwerveBridge.clock_cb(bridge, clock_message(10.0))

    old_listener.unregister.assert_called_once_with()
    assert bridge.tf_buffer is new_buffer
    assert bridge.tf_listener is new_listener
    assert bridge.tf_epoch == 1
    assert bridge.tf_epoch_reset_count == 1
    bridge.cmd_pub.publish.assert_called_once()
    assert bridge.manual_twist.linear.x == 0.0
    assert bridge.manual_deadline == 0.0
    assert bridge.gazebo_rtf is None
    assert bridge.last_tf_error == 'simulation clock rewound; waiting for current-epoch TF'
    assert bridge.simulation_time == 10.0
    assert bridge.pending_initial_pose is None
    assert bridge.pending_local_map_load is None
    assert bridge.local_map_load_pending is False
    assert bridge.pending_map_save is None
    errors = [call.kwargs['error'] for call in bridge._send_local_control_result.call_args_list]
    assert len(errors) == 3
    assert any('initial-pose confirmation' in error for error in errors)
    assert any('map load' in error for error in errors)
    assert any('map save' in error for error in errors)


def test_forward_simulation_clock_does_not_interrupt_valid_operations():
    bridge = make_bridge(60.0)
    bridge.pending_initial_pose = None
    bridge.pending_local_map_load = None
    bridge.local_map_load_pending = False
    bridge.pending_map_save = None

    SwerveBridge.clock_cb(bridge, clock_message(60, 100_000_000))

    bridge.tf_buffer.clear.assert_not_called()
    bridge.tf_listener.unregister.assert_not_called()
    bridge.cmd_pub.publish.assert_not_called()
    assert bridge.manual_twist.linear.x == 0.25
    assert bridge.gazebo_rtf is not None


def test_tf_epoch_reset_fails_closed_if_listener_recreation_fails():
    bridge = object.__new__(SwerveBridge)
    bridge.tf_buffer = Mock()
    bridge.tf_listener = Mock()
    bridge.tf_status = True
    bridge.tf_error = None
    bridge.last_tf_error = None
    bridge.get_logger = Mock(return_value=Mock())

    with patch.object(bridge_module, 'Buffer', return_value=Mock()), \
            patch.object(bridge_module, 'TransformListener', side_effect=RuntimeError('DDS unavailable')):
        assert SwerveBridge._reset_tf_epoch(bridge) is False

    assert bridge.tf_listener is None
    assert bridge.tf_status is False
    assert 'DDS unavailable' in bridge.tf_error


def test_robot_pose_lookup_rejects_a_transform_from_the_previous_simulation_epoch():
    bridge = object.__new__(SwerveBridge)
    bridge.get_parameter = lambda name: SimpleNamespace(value={
        'map_frame': 'map',
        'base_footprint_frame': 'base_footprint',
        'base_link_frame': 'base_link',
        'tf_max_age_s': 0.5,
        'tf_future_tolerance_s': 0.1,
    }[name])
    bridge.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=20_000_000_000))
    transform = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=60, nanosec=0)),
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.0, y=2.0, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )
    bridge.tf_buffer = SimpleNamespace(
        lookup_transform=lambda *_args, **_kwargs: transform)

    try:
        SwerveBridge._lookup_robot_pose(bridge)
    except TransformException as exc:
        assert 'future-dated' in str(exc)
    else:
        raise AssertionError('a previous-epoch transform was accepted as localization')


def test_initial_pose_confirmation_requires_a_tf_sample_after_the_request():
    bridge = object.__new__(SwerveBridge)
    bridge.tf_epoch = 2
    bridge.pending_initial_pose = {
        'data': {'request_id': 'pose'},
        'pose': {'x': 1.0, 'y': 2.0, 'yaw': 0.1},
        'active_map_id': 'map-1',
        'active_map_revision': 'rev-1',
        'request_stamp_s': 10.0,
        'deadline_monotonic': time.monotonic() + 10.0,
        'tf_epoch': 2,
        'service_acknowledged': False,
        'confirmation_start_stamp_s': None,
        'last_confirmation_stamp_s': None,
        'confirmation_sample_count': 0,
    }
    bridge.active_map_identity = lambda: {
        'active_map_id': 'map-1', 'active_map_revision': 'rev-1'}
    bridge.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=10_200_000_000))
    bridge._send_local_control_result = Mock()
    bridge._lookup_robot_pose = Mock(return_value=(
        {'x': 1.0, 'y': 2.0, 'yaw': 0.1, 'source_timestamp_s': 9.9},
        'base_footprint'))

    SwerveBridge._check_initial_pose_confirmation(bridge)

    assert bridge.pending_initial_pose is not None
    bridge._send_local_control_result.assert_not_called()

    # A fresh transform that is merely close under the former 20 cm / 0.35 rad
    # acceptance window must not confirm the pose.
    bridge.pending_initial_pose['service_acknowledged'] = True
    bridge._lookup_robot_pose.return_value = (
        {'x': 1.15, 'y': 2.0, 'yaw': 0.1, 'source_timestamp_s': 10.1},
        'base_footprint')
    SwerveBridge._check_initial_pose_confirmation(bridge)
    assert bridge.pending_initial_pose is not None
    bridge._send_local_control_result.assert_not_called()

    # A valid but transient sample is insufficient; confirmation requires a
    # continuing in-tolerance TF stream after the service acknowledgment.
    bridge._lookup_robot_pose.return_value = (
        {'x': 1.01, 'y': 2.0, 'yaw': 0.1, 'source_timestamp_s': 10.2},
        'base_footprint')
    SwerveBridge._check_initial_pose_confirmation(bridge)
    assert bridge.pending_initial_pose is not None
    bridge._lookup_robot_pose.return_value = (
        {'x': 1.0, 'y': 2.0, 'yaw': 0.1, 'source_timestamp_s': 10.3},
        'base_footprint')
    SwerveBridge._check_initial_pose_confirmation(bridge)
    assert bridge.pending_initial_pose is not None
    bridge._lookup_robot_pose.return_value = (
        {'x': 1.0, 'y': 2.0, 'yaw': 0.1, 'source_timestamp_s': 10.5},
        'base_footprint')
    SwerveBridge._check_initial_pose_confirmation(bridge)

    assert bridge.pending_initial_pose is None
    bridge._send_local_control_result.assert_called_once()
    assert bridge._send_local_control_result.call_args.args[1] is True
    result = bridge._send_local_control_result.call_args.args[2]
    assert result['request_stamp_s'] == 10.0
    assert result['tf_stamp_s'] == 10.5
    assert result['tf_confirmation_samples'] == 3
    assert abs(result['tf_confirmation_duration_sim_s'] - 0.3) < 1e-9


def test_initial_pose_confirmation_rejects_a_transform_epoch_change():
    bridge = object.__new__(SwerveBridge)
    bridge.tf_epoch = 3
    bridge.pending_initial_pose = {
        'data': {'request_id': 'pose'},
        'pose': {'x': 1.0, 'y': 2.0, 'yaw': 0.1},
        'active_map_id': 'map-1',
        'active_map_revision': 'rev-1',
        'request_stamp_s': 10.0,
        'deadline_monotonic': time.monotonic() + 10.0,
        'tf_epoch': 2,
        'service_acknowledged': True,
    }
    bridge.active_map_identity = lambda: {
        'active_map_id': 'map-1', 'active_map_revision': 'rev-1'}
    bridge._send_local_control_result = Mock()

    SwerveBridge._check_initial_pose_confirmation(bridge)

    assert bridge.pending_initial_pose is None
    bridge._send_local_control_result.assert_called_once()
    assert bridge._send_local_control_result.call_args.args[1] is False
    assert 'TF epoch changed' in bridge._send_local_control_result.call_args.kwargs['error']
