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
    probe._node_present = lambda name: name == 'command_arbiter'
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


def test_gazebo_world_readiness_is_reported_once():
    source = (ROOT / 'scripts/navigation_readiness.py').read_text(encoding='utf-8')
    module = ast.parse(source)
    readiness_class = next(
        node for node in module.body
        if isinstance(node, ast.ClassDef) and node.name == 'Readiness'
    )
    check_method = next(
        node for node in readiness_class.body
        if isinstance(node, ast.FunctionDef) and node.name == 'check'
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
