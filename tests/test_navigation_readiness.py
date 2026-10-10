import ast
from pathlib import Path
import sys
import time

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def readiness_module():
    scripts = str(ROOT / 'scripts')
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    try:
        import navigation_readiness
    except ImportError as exc:
        pytest.skip(f'ROS readiness dependencies are unavailable: {exc}')
    return navigation_readiness


def test_clock_readiness_requires_three_strictly_advancing_samples(readiness_module):
    probe = readiness_module.Readiness.__new__(readiness_module.Readiness)

    probe.clock_samples = [(10, 1.0), (11, 1.1)]
    assert not probe._clock_ready()

    probe.clock_samples = [(10, 1.0), (11, 1.1), (12, 1.2)]
    assert probe._clock_ready()

    probe.clock_samples = [(10, 1.0), (10, 1.1), (12, 1.2)]
    assert not probe._clock_ready()

    probe.clock_samples = [(10, 1.0), (11, 4.0), (12, 7.0)]
    assert not probe._clock_ready()


def test_model_states_callback_confirms_live_robot_name(readiness_module):
    from types import SimpleNamespace

    probe = readiness_module.Readiness.__new__(readiness_module.Readiness)
    probe.model = 'swerve_base'
    probe.model_states_seen = False
    probe.gazebo_model_names = set()
    probe.timeline_events = {}

    probe._model_states_cb(SimpleNamespace(name=['ground_plane', 'swerve_base']))

    assert probe.model_states_seen
    assert probe.gazebo_model_names == {'ground_plane', 'swerve_base'}
    assert probe._wait_entity(time.monotonic() + 1.0)
    assert probe.timeline_events['T6_ROBOT_ENTITY_CONFIRMED']['status'] == 'PASS'


def test_command_arbiter_readiness_requires_selected_topic_endpoints_and_owner_sample(
    readiness_module,
):
    from types import SimpleNamespace

    probe = readiness_module.Readiness.__new__(readiness_module.Readiness)
    probe.command_owner_seen = True
    probe._node_present = lambda name: name in {'command_arbiter', 'swerve_controller'}
    probe.get_publishers_info_by_topic = lambda topic: [
        SimpleNamespace(node_name='command_arbiter')
    ]
    probe.get_subscriptions_info_by_topic = lambda topic: [
        SimpleNamespace(node_name='swerve_controller')
    ]

    assert probe._command_arbiter_graph_ready()

    probe.command_owner_seen = False
    assert not probe._command_arbiter_graph_ready()
    probe.command_owner_seen = True
    probe.get_subscriptions_info_by_topic = lambda topic: []
    assert not probe._command_arbiter_graph_ready()


def test_mapping_authority_requires_slam_as_the_only_map_publisher(readiness_module):
    from types import SimpleNamespace

    probe = readiness_module.Readiness.__new__(readiness_module.Readiness)
    probe.get_node_names_and_namespaces = lambda: [
        ('slam_toolbox', '/'), ('ekf_filter_node', '/'), ('swerve_bridge', '/'),
    ]
    probe.get_publishers_info_by_topic = lambda topic: (
        [SimpleNamespace(node_name='slam_toolbox')] if topic == '/map' else
        [SimpleNamespace(node_name='slam_toolbox'), SimpleNamespace(node_name='ekf_filter_node')]
    )
    ok, detail = probe._mapping_runtime_authority()
    assert ok
    assert 'live_map=/map publisher=slam_toolbox' in detail

    # Graph node names are not unique in ROS 2. Two endpoints with the same
    # name still mean two independent /map and map->odom publishers.
    probe.get_publishers_info_by_topic = lambda topic: (
        [SimpleNamespace(node_name='slam_toolbox'), SimpleNamespace(node_name='slam_toolbox')]
        if topic == '/map' else
        [SimpleNamespace(node_name='slam_toolbox'), SimpleNamespace(node_name='slam_toolbox'),
         SimpleNamespace(node_name='ekf_filter_node')]
    )
    ok, detail = probe._mapping_runtime_authority()
    assert not ok
    assert '/map_publishers_must_be_slam_toolbox_only:count=2' in detail

    probe.get_node_names_and_namespaces = lambda: [
        ('slam_toolbox', '/'), ('ekf_v30e', '/'),
    ]
    ok, detail = probe._mapping_runtime_authority()
    assert not ok
    assert 'ekf_v30e' in detail


def test_unified_nav2_lifecycle_contract_uses_registered_bundle_map(readiness_module):
    probe = readiness_module.Readiness.__new__(readiness_module.Readiness)
    probe.mode = 'unified'
    probe.nav2_lifecycle_nodes = readiness_module.NAV2_REGISTERED_CANONICAL_LIFECYCLE_NODES
    probe.expected_canonical_revision = 22
    probe._nav2_graph_counts = lambda: {
        'map_server': 0,
        'canonical_map_server': 1,
        'lifecycle_manager_navigation': 1,
        'lifecycle_manager_mapping_map': 0,
    }
    probe._read_nav2_manager_configuration = lambda _deadline: ({
        'autostart': False,
        'node_names': list(readiness_module.NAV2_REGISTERED_CANONICAL_LIFECYCLE_NODES),
    }, None)
    probe._read_map_yaml_parameter = lambda _deadline: ('/bundle/22/nav2/warehouse.yaml', None)

    ready, error = probe._deferred_nav2_contract(time.monotonic() + 1.0)

    assert ready, error


def _grid(width, height, resolution, origin_x, origin_y):
    from types import SimpleNamespace

    return SimpleNamespace(
        header=SimpleNamespace(frame_id='map'),
        info=SimpleNamespace(
            width=width, height=height, resolution=resolution,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=origin_x, y=origin_y),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=[0] * (width * height),
    )


def _full_registered_map_probe(readiness_module):
    from types import SimpleNamespace

    probe = readiness_module.Readiness.__new__(readiness_module.Readiness)
    probe.expected_canonical_revision = 22
    probe.canonical_map_grid = _grid(600, 1200, 0.05, 0.0, 0.0)
    probe.navigation_map_grid = _grid(1200, 600, 0.05, -5.5, -15.0)
    probe.global_costmap_grid = _grid(1200, 600, 0.05, -5.5, -15.0)
    probe.navigation_map_metadata = {
        'ready': True,
        'navigation_map_source': 'PUBLISHED_CANONICAL_REGISTERED',
        'navigation_map_id': 'NAV-22-SLAM-session-1',
        'navigation_map_revision': 'session-session-1:canonical-22:registration-3',
        'canonical_map_revision': 22,
        'active_map_id': 'SLAM-session-1',
        'active_map_revision': 'session-session-1',
        'registration_revision': 3,
        'registration_source': 'GAZEBO_CANONICAL_ALIGNMENT',
        'frame_id': 'map',
        'resolution': 0.05, 'width': 1200, 'height': 600,
        'origin_x': -5.5, 'origin_y': -15.0,
        'min_x': -5.5, 'max_x': 54.5, 'min_y': -15.0, 'max_y': 15.0,
        'source_width': 600, 'source_height': 1200, 'source_resolution': 0.05,
        'source_origin_x': 0.0, 'source_origin_y': 0.0, 'source_origin_yaw': 0.0,
        'registration': {'tx': -5.5, 'ty': 15.0, 'yaw': -1.5707963267948966},
    }
    probe.get_publishers_info_by_topic = lambda topic: [
        SimpleNamespace(node_name={
            '/map': 'slam_toolbox',
            '/canonical_map': 'canonical_map_server',
            '/navigation_map': 'swerve_bridge',
        }[topic])
    ]
    probe.get_subscriptions_info_by_topic = lambda topic: (
        [SimpleNamespace(node_name='global_costmap')] if topic == '/navigation_map' else [])
    return probe


def test_registered_navigation_map_and_costmap_cover_full_bundle_not_live_slam_extent(
    readiness_module,
):
    probe = _full_registered_map_probe(readiness_module)
    assert probe._unified_navigation_map_error() is None

    # The confirmed old live SLAM map was 420x597 at (-5.429, -14.897),
    # so its max X=15.571m cannot cover the registered canonical map.
    probe.global_costmap_grid = _grid(420, 597, 0.05, -5.429391, -14.896761)
    assert probe._unified_navigation_map_error() == (
        'global_costmap_extent_does_not_cover_registered_navigation_map')


def test_registered_navigation_map_readiness_fails_closed_on_revision_and_duplicate_map_owner(
    readiness_module,
):
    probe = _full_registered_map_probe(readiness_module)
    probe.navigation_map_metadata['canonical_map_revision'] = 21
    assert probe._unified_navigation_map_error() == 'canonical_revision_mismatch:expected=22:actual=21'

    probe = _full_registered_map_probe(readiness_module)
    original = probe.get_publishers_info_by_topic
    probe.get_publishers_info_by_topic = lambda topic: (
        original(topic) + [type('Publisher', (), {'node_name': 'map_server'})()]
        if topic == '/map' else original(topic))
    assert '/map_publishers_must_be_slam_toolbox_only' in probe._unified_navigation_map_error()

    probe = _full_registered_map_probe(readiness_module)
    original = probe.get_publishers_info_by_topic
    probe.get_publishers_info_by_topic = lambda topic: (
        original(topic) + [type('Publisher', (), {'node_name': 'slam_toolbox'})()]
        if topic == '/map' else original(topic))
    assert '/map_publishers_must_be_slam_toolbox_only:count=2' in probe._unified_navigation_map_error()


def test_readiness_binds_selected_nav2_yaml_to_published_bundle_revision(readiness_module):
    selected = ROOT / 'generated/maps/WH-TEST-01/22/nav2/warehouse_1.yaml'
    assert readiness_module.Readiness._bundle_revision_for_map(selected) == 22
    assert readiness_module.Readiness._bundle_revision_for_map(
        ROOT / 'swerve_navigation/maps/warehouse.yaml') is None


def test_mapping_readiness_sensor_rate_requires_real_receive_samples(readiness_module):
    probe = readiness_module.Readiness.__new__(readiness_module.Readiness)
    assert probe._sample_hz([1.0]) is None
    assert probe._sample_hz([1.0, 1.1, 1.2]) == pytest.approx(10.0)


def test_simulated_mapping_launch_uses_slam_map_and_disables_v30e_map_owner():
    launch = ROOT / 'launch' / 'system.launch.py'
    tree = ast.parse(launch.read_text(encoding='utf-8'))
    source = launch.read_text(encoding='utf-8')
    names = {node.id for node in ast.walk(tree) if isinstance(node, ast.Name)}

    assert 'simulated_navigation_mode' in names
    assert 'simulated_mapping_mode' not in names
    assert "'map_topic': '/map'" in source
    assert "'transform_publish_period': '0.02'" in source
    assert 'condition=simulated_navigation_mode' in source
    assert 'mapping_map_server' not in names
    assert 'mapping_map_lifecycle' not in names


def test_controller_readiness_queries_only_after_launch_chain_and_at_low_rate(
    readiness_module, monkeypatch, capsys,
):
    from types import SimpleNamespace

    probe = readiness_module.Readiness.__new__(readiness_module.Readiness)
    probe._node_present = lambda _name: True
    probe._spin_until = lambda predicate, _deadline: predicate()
    probe.controllers = SimpleNamespace(
        service_name='/controller_manager/list_controllers',
        service_is_ready=lambda: True,
    )
    probe.hardware = SimpleNamespace(
        service_name='/controller_manager/list_hardware_interfaces',
        service_is_ready=lambda: True,
    )
    probe.log_path = None
    probe.stack_start_monotonic = None
    probe.timeline_events = {}
    probe.hardware_interfaces = set()
    probe.controller_topic_names = set()
    probe.last_service_error = None
    probe.get_topic_names_and_types = lambda: [
        ('/steering_controller/commands', []),
        ('/drive_controller/commands', []),
    ]

    clock = {'now': 100.0}
    query_times = []
    monkeypatch.setattr(readiness_module.time, 'monotonic', lambda: clock['now'])
    monkeypatch.setattr(
        readiness_module.rclpy,
        'spin_once',
        lambda _node, timeout_sec=0.0: clock.__setitem__('now', clock['now'] + timeout_sec),
    )

    all_active = {
        name: 'active' for name in (
            'joint_state_broadcaster', 'steering_controller', 'drive_controller')
    }
    first_states = dict(all_active, steering_controller='inactive')

    def service_call(client, _deadline, prepare=None):
        if client is probe.controllers:
            query_times.append(clock['now'])
            states = first_states if len(query_times) == 1 else all_active
            return SimpleNamespace(controller=[
                SimpleNamespace(name=name, state=state) for name, state in states.items()
            ])
        return SimpleNamespace(command_interfaces=[
            SimpleNamespace(name=name, is_claimed=True) for name in (
                'steer_front_joint/position', 'steer_rear_joint/position',
                'wheel_front_drive_joint/velocity', 'wheel_rear_drive_joint/velocity')
        ])

    probe._service_call = service_call
    result = {'stages': {}}

    assert probe._wait_controllers(result, deadline=clock['now'] + 5.0)
    assert len(query_times) == 2
    assert query_times[1] - query_times[0] >= readiness_module.CONTROLLER_QUERY_INTERVAL_S
    assert result['stages']['CONTROLLER_MANAGER_READY'] is True
    assert result['stages']['JOINT_STATE_BROADCASTER_ACTIVE'] is True
    assert result['stages']['STEERING_CONTROLLER_ACTIVE'] is True
    assert result['stages']['DRIVE_CONTROLLER_ACTIVE'] is True
    capsys.readouterr()


def test_required_readiness_summary_reports_independent_unverified_stages(
    readiness_module, capsys,
):
    probe = readiness_module.Readiness.__new__(readiness_module.Readiness)
    probe._print_required_status_summary({'stages': {
        'GAZEBO_PROCESS_READY': True,
        'CLOCK_READY': True,
        'GAZEBO_FACTORY_READY': True,
        'GAZEBO_WORLD_READY': True,
        'ROBOT_SPAWNED': True,
        'CONTROLLER_MANAGER_READY': True,
        'JOINT_STATE_BROADCASTER_ACTIVE': True,
        'STEERING_CONTROLLER_ACTIVE': False,
        'DRIVE_CONTROLLER_ACTIVE': None,
    }})

    output = capsys.readouterr().out
    assert 'GAZEBO_READY=PASS' in output
    assert 'ROBOT_SPAWNED=PASS' in output
    assert 'CONTROLLER_MANAGER_READY=PASS' in output
    assert 'JOINT_STATE_BROADCASTER_ACTIVE=PASS' in output
    assert 'STEERING_CONTROLLER_ACTIVE=FAIL' in output
    assert 'DRIVE_CONTROLLER_ACTIVE=UNVERIFIED' in output
    assert 'BRIDGE_READY=UNVERIFIED' in output


def test_spawn_duration_uses_request_and_response_log_timestamps(
    readiness_module, tmp_path, capsys,
):
    probe = readiness_module.Readiness.__new__(readiness_module.Readiness)
    wall_now = time.time()
    monotonic_now = time.monotonic()
    request_at = wall_now - 10.0
    response_at = wall_now - 6.5
    log_path = tmp_path / 'ros.log'
    log_path.write_text(
        f'[{request_at:.3f}] [spawn_swerve]: Calling service /spawn_entity\n'
        f'[{response_at:.3f}] [spawn_swerve]: Spawn status: '
        'SpawnEntity: Successfully spawned entity [swerve_base]\n',
        encoding='utf-8',
    )
    probe.log_path = str(log_path)
    probe.stack_start_monotonic = monotonic_now - 30.0
    probe.wall_to_monotonic_offset = monotonic_now - wall_now
    probe.timeline_events = {}
    probe.timeline_reported = False
    result = {}

    probe._print_startup_timeline(result)

    assert result['spawn_entity_duration_s'] == 3.5
    assert probe.timeline_events['T4_SPAWN_REQUEST_BEGIN']['status'] == 'PASS'
    assert probe.timeline_events['T5_SPAWN_REQUEST_RETURN']['status'] == 'PASS'
    assert 'SPAWN_ENTITY_DURATION=3.500s' in capsys.readouterr().out


def test_startup_timeline_keeps_required_stages_in_order(readiness_module):
    stages = readiness_module.STARTUP_TIMELINE_ORDER
    required = (
        'T0_START_STACK',
        'T1_GAZEBO_PROCESS_START',
        'T2_CLOCK_READY',
        'T3_SPAWN_SERVICE_READY',
        'T4_SPAWN_REQUEST_BEGIN',
        'T5_SPAWN_REQUEST_RETURN',
        'T6_ROBOT_ENTITY_CONFIRMED',
        'T7_CONTROLLER_MANAGER_READY',
        'T8_JOINT_STATE_BROADCASTER_ACTIVE',
        'T9_STEERING_CONTROLLER_ACTIVE',
        'T10_DRIVE_CONTROLLER_ACTIVE',
        'JOINT_STATES_READY',
        'T11_ODOM_READY',
        'FILTERED_ODOM_READY',
        'T12_LIDAR_RAW_READY',
        'T13_LIDAR_FILTERED_READY',
        'T14_SCAN_READY',
        'T15_NAV2_LIFECYCLE_STARTUP_BEGIN',
        'T16_NAV2_ACTION_SERVER_READY',
    )

    positions = [stages.index(stage) for stage in required]
    assert positions == sorted(positions)


def test_nav2_startup_claim_is_persisted_and_single_shot(readiness_module, tmp_path):
    state_file = tmp_path / 'runtime' / 'nav2-startup.json'

    assert readiness_module.read_nav2_startup_state(state_file) == {
        'state': 'NOT_REQUESTED',
    }
    claimed, first = readiness_module.claim_nav2_startup(
        state_file, launch_id='launch-test', lifecycle_before={'map_server': {'id': 1}},
    )
    assert claimed
    assert first['state'] == 'REQUESTED'
    assert first['launch_id'] == 'launch-test'

    claimed_again, previous = readiness_module.claim_nav2_startup(state_file)
    assert not claimed_again
    assert previous['state'] == 'REQUESTED'
    assert readiness_module.update_nav2_startup_state(
        state_file, 'IN_PROGRESS', request_id=first['request_id'],
    )['state'] == 'IN_PROGRESS'
    assert readiness_module.read_nav2_startup_state(state_file)['request_id'] == first['request_id']


def test_nav2_lifecycle_transition_states_are_not_considered_settled(readiness_module):
    settled = {
        name: {'id': 1, 'label': 'unconfigured'}
        for name in readiness_module.NAV2_STATIC_MAP_LIFECYCLE_NODES
    }
    assert readiness_module.Readiness._lifecycle_states_settled(settled)

    transitioning = dict(settled)
    transitioning['map_server'] = {'id': 10, 'label': 'activating'}
    assert not readiness_module.Readiness._lifecycle_states_settled(transitioning)
    assert not readiness_module.Readiness._lifecycle_states_are(transitioning, 1)


def test_in_progress_nav2_startup_is_observed_without_issuing_another_startup(readiness_module):
    probe = readiness_module.Readiness.__new__(readiness_module.Readiness)
    names = readiness_module.NAV2_STATIC_MAP_LIFECYCLE_NODES
    states = {name: {'id': 1, 'label': 'unconfigured'} for name in names}
    probe.nav2_lifecycle_nodes = names
    probe._wait_lifecycle_settled = lambda _deadline: states
    probe._startup_state = lambda: {'state': 'IN_PROGRESS', 'request_id': 'same-request'}
    updates = []
    probe._update_startup_state = lambda state, **details: updates.append((state, details)) or {
        'state': state, **details}
    probe._report_stage = lambda result, name, passed, reason=None, success_detail=None: result.setdefault(
        'reported', []).append((name, passed, reason, success_detail))
    result = {'stages': {}}

    ready, observed, reason = probe._ensure_nav2_lifecycle_ready(result, time.monotonic() + 1.0)

    assert not ready
    assert observed == states
    assert 'already_consumed_but_nodes_not_active' in reason
    assert updates and updates[-1][0] == 'IN_PROGRESS'
    assert all(state != 'FAILED' for state, _ in updates)
    assert result['stages']['NAV2_STARTUP_STATE'] == 'IN_PROGRESS'


@pytest.mark.parametrize(('service_error', 'expected_state'), [
    ('/lifecycle_manager_navigation/manage_nodes response timed out', 'IN_PROGRESS'),
    ('/lifecycle_manager_navigation/manage_nodes service unavailable', 'NOT_REQUESTED'),
])
def test_nav2_startup_probe_timeout_does_not_mark_an_ambiguous_activation_failed(
    readiness_module, monkeypatch, service_error, expected_state,
):
    probe = readiness_module.Readiness.__new__(readiness_module.Readiness)
    names = readiness_module.NAV2_REGISTERED_CANONICAL_LIFECYCLE_NODES
    states = {name: {'id': 1, 'label': 'unconfigured'} for name in names}
    manager = {'autostart': False, 'node_names': list(names)}
    probe.mode = 'unified'
    probe.nav2_lifecycle_nodes = names
    probe.nav_lifecycle_manager = object()
    probe.lifecycle_state_file = 'unused-test-state.json'
    probe.expected_canonical_revision = 23
    probe._wait_lifecycle_settled = lambda _deadline: states
    probe._lifecycle_snapshot = lambda _deadline: states
    probe._startup_state = lambda: {'state': 'NOT_REQUESTED', 'launch_id': 'launch-test'}
    probe._nav2_graph_counts = lambda: {
        'map_server': 0, 'canonical_map_server': 1,
        'lifecycle_manager_navigation': 1, 'lifecycle_manager_mapping_map': 0,
    }
    probe._read_nav2_manager_configuration = lambda _deadline: (manager, None)
    probe._read_map_yaml_parameter = lambda _deadline: ('/published/map.yaml', None)
    probe._lifecycle_log_offset = lambda: 0
    probe._print_lifecycle_manager_log = lambda _offset: None
    probe._record_timeline = lambda *_args, **_kwargs: None
    updates = []
    probe._update_startup_state = lambda state, **details: updates.append((state, details)) or {
        'state': state, **details}
    probe._report_stage = lambda result, name, passed, reason=None, success_detail=None: result.setdefault(
        'reported', []).append((name, passed, reason, success_detail))

    def service_call(_client, _deadline, _prepare=None):
        probe.last_service_error = service_error
        return None

    probe._service_call = service_call
    monkeypatch.setattr(readiness_module, 'claim_nav2_startup', lambda *_args, **_kwargs: (
        True, {'state': 'REQUESTED', 'request_id': 'one-shot-request'}))
    result = {'stages': {}}

    ready, observed, reason = probe._ensure_nav2_lifecycle_ready(result, time.monotonic() + 1.0)

    assert not ready
    assert observed == states
    assert service_error in reason
    assert updates[-1][0] == expected_state
    assert all(state != 'FAILED' for state, _ in updates)
    assert result['stages']['NAV2_STARTUP_STATE'] == expected_state


def test_supervisor_resets_nav2_startup_guard_for_each_new_navigation_child(
    readiness_module, tmp_path,
):
    import ros_stack_supervisor

    state_file = tmp_path / 'runtime' / 'nav2-startup.json'
    readiness_module.update_nav2_startup_state(state_file, 'FAILED', failure='test')
    ros_stack_supervisor._reset_nav2_lifecycle_state(state_file, 'navigation')
    state = readiness_module.read_nav2_startup_state(state_file)
    assert state['state'] == 'NOT_REQUESTED'
    assert state['launch_id']

    ros_stack_supervisor._reset_nav2_lifecycle_state(state_file, 'mapping')
    assert not state_file.exists()


def test_gazebo_world_readiness_is_reported_once():
    source = (ROOT / 'scripts/navigation_readiness.py').read_text(encoding='utf-8')
    module = ast.parse(source)
    readiness_class = next(
        node for node in module.body
        if isinstance(node, ast.ClassDef) and node.name == 'Readiness'
    )
    check_method = next(
        node for node in readiness_class.body
        if isinstance(node, ast.FunctionDef) and node.name == '_simulation_prerequisites'
    )
    world_stage_calls = []
    for node in ast.walk(check_method):
        if not isinstance(node, ast.Call) or not node.args:
            continue
        function = node.func
        function_name = (
            function.id if isinstance(function, ast.Name)
            else function.attr if isinstance(function, ast.Attribute)
            else None
        )
        if (function_name in {'_stage', '_report_stage'}
                and isinstance(node.args[1], ast.Constant)
                and node.args[1].value == 'GAZEBO_WORLD_READY'):
            world_stage_calls.append(function_name)

    assert world_stage_calls == ['_stage']
