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
        'interface_type': 'robot_localization/srv/SetPose',
        'localization_owner': 'test_localizer',
        'interface': '/test_localizer/set_pose',
        'initial_pose_services': [],
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
        'interface_type': 'robot_localization/srv/SetPose',
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


def test_amcl_initial_pose_requires_post_request_localization_pose_and_tf():
    bridge = object.__new__(SwerveBridge)
    bridge.tf_epoch = 1
    bridge.pending_initial_pose = {
        'data': {'request_id': 'pose'},
        'pose': {'x': 1.0, 'y': 2.0, 'yaw': 0.1},
        'active_map_id': 'saved-1', 'active_map_revision': 'rev-7',
        'localization_owner': 'amcl', 'interface': '/amcl/set_initial_pose',
        'interface_type': 'nav2_msgs/srv/SetInitialPose', 'service_acknowledged': True,
        'request_stamp_s': 10.0,
        'deadline_monotonic': time.monotonic() + 10.0,
        'tf_epoch': 1, 'confirmation_start_stamp_s': None,
        'last_confirmation_stamp_s': None, 'confirmation_sample_count': 0,
    }
    bridge.latest_amcl_pose = {
        'stamp_s': 9.9, 'frame_id': 'map', 'x': 1.0, 'y': 2.0, 'yaw': 0.1,
    }
    bridge.active_map_identity = lambda: {
        'active_map_id': 'saved-1', 'active_map_revision': 'rev-7'}
    bridge.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=10_600_000_000))
    bridge._send_local_control_result = Mock()
    bridge._lookup_robot_pose = Mock(return_value=(
        {'x': 1.0, 'y': 2.0, 'yaw': 0.1, 'source_timestamp_s': 10.1},
        'base_footprint'))

    SwerveBridge._check_initial_pose_confirmation(bridge)
    assert bridge.pending_initial_pose is not None
    bridge._send_local_control_result.assert_not_called()

    for stamp in (10.2, 10.3, 10.5):
        bridge.latest_amcl_pose = {
            'stamp_s': stamp, 'frame_id': 'map', 'x': 1.0, 'y': 2.0, 'yaw': 0.1,
        }
        bridge._lookup_robot_pose.return_value = (
            {'x': 1.0, 'y': 2.0, 'yaw': 0.1, 'source_timestamp_s': stamp},
            'base_footprint')
        SwerveBridge._check_initial_pose_confirmation(bridge)

    assert bridge.pending_initial_pose is None
    bridge._send_local_control_result.assert_called_once()
    result = bridge._send_local_control_result.call_args.args[2]
    assert result['localization_owner'] == 'amcl'
    assert result['initial_pose_interface'] == '/amcl/set_initial_pose'
    assert result['localization_update_stamp_s'] == 10.5
    assert result['tf_amcl_position_delta_m'] == 0.0


def test_amcl_initial_pose_topic_requires_post_request_localization_pose_and_tf():
    bridge = object.__new__(SwerveBridge)
    bridge.tf_epoch = 1
    bridge.pending_initial_pose = {
        'data': {'request_id': 'pose-topic'},
        'pose': {'x': 1.0, 'y': 2.0, 'yaw': 0.1},
        'active_map_id': 'saved-1', 'active_map_revision': 'rev-7',
        'localization_owner': 'amcl', 'interface': '/initialpose',
        'interface_type': 'geometry_msgs/msg/PoseWithCovarianceStamped_TOPIC',
        'request_published': True, 'request_stamp_s': 10.0,
        'deadline_monotonic': time.monotonic() + 10.0,
        'tf_epoch': 1, 'confirmation_start_stamp_s': None,
        'last_confirmation_stamp_s': None, 'confirmation_sample_count': 0,
    }
    bridge.latest_amcl_pose = {
        'stamp_s': 9.9, 'frame_id': 'map', 'x': 1.0, 'y': 2.0, 'yaw': 0.1,
    }
    bridge.active_map_identity = lambda: {
        'active_map_id': 'saved-1', 'active_map_revision': 'rev-7'}
    bridge.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=10_600_000_000))
    bridge._send_local_control_result = Mock()
    bridge._lookup_robot_pose = Mock(return_value=(
        {'x': 1.0, 'y': 2.0, 'yaw': 0.1, 'source_timestamp_s': 10.1},
        'base_footprint'))

    SwerveBridge._check_initial_pose_confirmation(bridge)
    assert bridge.pending_initial_pose is not None
    bridge._send_local_control_result.assert_not_called()

    for stamp in (10.2, 10.3, 10.5):
        bridge.latest_amcl_pose = {
            'stamp_s': stamp, 'frame_id': 'map', 'x': 1.0, 'y': 2.0, 'yaw': 0.1,
        }
        bridge._lookup_robot_pose.return_value = (
            {'x': 1.0, 'y': 2.0, 'yaw': 0.1, 'source_timestamp_s': stamp},
            'base_footprint')
        SwerveBridge._check_initial_pose_confirmation(bridge)

    assert bridge.pending_initial_pose is None
    bridge._send_local_control_result.assert_called_once()
    result = bridge._send_local_control_result.call_args.args[2]
    assert result['localization_owner'] == 'amcl'
    assert result['initial_pose_interface'] == '/initialpose'
    assert result['localization_update_stamp_s'] == 10.5
    assert result['tf_amcl_position_delta_m'] == 0.0


def test_localization_interface_discovers_amcl_and_set_pose_servers_without_selecting_odom_ekf():
    bridge = object.__new__(SwerveBridge)
    bridge.runtime_state = 'NAVIGATION'
    bridge.initial_pose_backend = 'AMCL'
    bridge.namespace = ''
    bridge.get_node_names_and_namespaces = lambda: [
        ('map_server', '/'), ('amcl', '/'), ('ekf_filter_node', '/'),
    ]
    bridge.get_publishers_info_by_topic = lambda topic: (
        [SimpleNamespace(node_name='amcl', node_namespace='/',
                         topic_type='geometry_msgs/msg/PoseWithCovarianceStamped')]
        if topic == '/amcl_pose' else [
            SimpleNamespace(node_name='amcl', node_namespace='/'),
            SimpleNamespace(node_name='ekf_filter_node', node_namespace='/'),
        ])
    bridge.get_service_names_and_types_by_node = lambda node, _namespace: (
        [('/set_pose', ['robot_localization/srv/SetPose'])]
        if node == 'ekf_filter_node' else [])
    bridge.get_subscriptions_info_by_topic = lambda _topic: [
        SimpleNamespace(node_name='amcl', node_namespace='/',
                        topic_type='geometry_msgs/msg/PoseWithCovarianceStamped')]
    bridge.nav2_lifecycle_status = lambda: {'states': {'amcl': 'active'}}

    result = SwerveBridge.discover_localization_interface(bridge)

    assert result['ready']
    assert result['owner'] == 'amcl'
    assert result['interface'] == '/initialpose'
    assert result['interface_type'] == 'geometry_msgs/msg/PoseWithCovarianceStamped_TOPIC'
    assert result['initial_pose_api'] == 'topic'
    assert result['initial_pose_services'] == [
        {'name': '/set_pose', 'owner': 'ekf_filter_node',
         'types': ['robot_localization/srv/SetPose']},
    ]


def test_amcl_set_initial_pose_service_is_used_only_with_local_python_type_support():
    bridge = object.__new__(SwerveBridge)
    bridge.runtime_state = 'NAVIGATION'
    bridge.initial_pose_backend = 'AMCL'
    bridge.namespace = ''
    bridge.get_node_names_and_namespaces = lambda: [
        ('map_server', '/'), ('amcl', '/'), ('ekf_filter_node', '/'),
    ]
    bridge.get_publishers_info_by_topic = lambda topic: (
        [SimpleNamespace(node_name='amcl', node_namespace='/',
                         topic_type='geometry_msgs/msg/PoseWithCovarianceStamped')]
        if topic == '/amcl_pose' else [
            SimpleNamespace(node_name='amcl', node_namespace='/'),
            SimpleNamespace(node_name='ekf_filter_node', node_namespace='/'),
        ])
    bridge.get_service_names_and_types_by_node = lambda node, _namespace: (
        [('/set_initial_pose', ['nav2_msgs/srv/SetInitialPose'])]
        if node == 'amcl' else
        [('/set_pose', ['robot_localization/srv/SetPose'])]
        if node == 'ekf_filter_node' else [])
    bridge.get_subscriptions_info_by_topic = lambda _topic: [
        SimpleNamespace(node_name='amcl', node_namespace='/',
                        topic_type='geometry_msgs/msg/PoseWithCovarianceStamped')]
    bridge.nav2_lifecycle_status = lambda: {'states': {'amcl': 'active'}}

    with patch.object(bridge_module, 'get_service',
                      side_effect=AttributeError('generated Python class absent')):
        result = SwerveBridge.discover_localization_interface(bridge)

    assert result['ready']
    assert result['owner'] == 'amcl'
    assert result['interface'] == '/initialpose'
    assert result['initial_pose_api'] == 'topic'
    assert result['discovered_set_initial_pose_services'] == [{
        'name': '/set_initial_pose', 'owner': 'amcl',
        'types': ['nav2_msgs/srv/SetInitialPose'],
    }]
    assert result['service_type_resolution_error'].startswith('AttributeError:')


def test_slam_mapping_modes_do_not_mistake_ekf_set_pose_for_map_initial_pose():
    bridge = object.__new__(SwerveBridge)
    bridge.runtime_state = 'UNIFIED'
    bridge.initial_pose_backend = 'AMCL'
    bridge.initial_pose_service_node = ''
    bridge.namespace = ''
    bridge.get_node_names_and_namespaces = lambda: [
        ('slam_toolbox', '/'), ('ekf_filter_node', '/'),
    ]
    bridge.get_publishers_info_by_topic = lambda _topic: [
        SimpleNamespace(node_name='slam_toolbox', node_namespace='/'),
        SimpleNamespace(node_name='ekf_filter_node', node_namespace='/'),
    ]
    bridge.get_service_names_and_types_by_node = lambda node, _namespace: (
        [('/set_pose', ['robot_localization/srv/SetPose'])]
        if node == 'ekf_filter_node' else [])
    bridge.get_subscriptions_info_by_topic = lambda _topic: []

    result = SwerveBridge.discover_localization_interface(bridge)

    assert not result['ready']
    assert result['owner'] == 'slam_toolbox'
    assert result['interface'] is None
    assert 'does not expose a static-map Initial Pose service' in result['reason']
    assert result['initial_pose_services'] == [{
        'name': '/set_pose', 'owner': 'ekf_filter_node',
        'types': ['robot_localization/srv/SetPose'],
    }]


def test_legacy_set_pose_backend_uses_discovered_endpoint_and_tf_owner():
    bridge = object.__new__(SwerveBridge)
    bridge.runtime_state = 'NAVIGATION'
    bridge.initial_pose_backend = 'SET_POSE'
    bridge.initial_pose_service_node = 'map_filter'
    bridge.namespace = ''
    bridge.get_node_names_and_namespaces = lambda: [
        ('map_filter', '/'), ('ekf_filter_node', '/'),
    ]
    bridge.get_publishers_info_by_topic = lambda _topic: [
        SimpleNamespace(node_name='map_filter', node_namespace='/'),
        SimpleNamespace(node_name='ekf_filter_node', node_namespace='/'),
    ]
    bridge.get_service_names_and_types_by_node = lambda node, _namespace: (
        [('/map_filter/custom_pose_service', ['robot_localization/srv/SetPose'])]
        if node == 'map_filter' else
        [('/set_pose', ['robot_localization/srv/SetPose'])]
        if node == 'ekf_filter_node' else [])
    bridge.get_subscriptions_info_by_topic = lambda _topic: []

    result = SwerveBridge.discover_localization_interface(bridge)

    assert result['ready']
    assert result['owner'] == 'map_filter'
    assert result['interface'] == '/map_filter/custom_pose_service'
    assert result['interface_type'] == 'robot_localization/srv/SetPose'
