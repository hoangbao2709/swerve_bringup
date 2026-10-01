import hashlib
import json
import signal
from pathlib import Path

import pytest

import ros_stack_supervisor
from ros_stack_supervisor import (command_for_mode, command_for_revision,
                                  validate_slam_session_prefix, verify_bundle)


def _bundle(root: Path, revision: int = 12):
    files = {
        'canonical_map.json': json.dumps({
            'frame_id': 'map', 'revision': revision, 'origin': {'x': 0, 'y': 0},
            'width': 1, 'height': 1,
            'floors': [{'id': 'F1', 'boundary': [[0, 0], [1, 0], [1, 1], [0, 1]], 'holes': []}],
        }),
        'gazebo/warehouse.world': '<sdf><world name="warehouse"><plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"/></world></sdf>',
        'gazebo/manifest.json': json.dumps({
            'floors': [{'id': 'F1', 'boundary': [[0, 0], [1, 0], [1, 1], [0, 1]], 'holes': []}],
        }),
        'datamatrix_map.yaml': 'frame_id: map\n',
        'tag_graph.yaml': 'frame_id: map\n',
        'nav2/warehouse_F1.yaml': 'image: warehouse_F1.pgm\nresolution: 1\norigin: [0, 0, 0]\nframe_id: map\n',
        'nav2/warehouse_F1.pgm': 'P5\n1 1\n255\n\xfe',
    }
    for relative, value in files.items():
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(value if isinstance(value, bytes) else value.encode())
    manifest = {
        'revision': revision, 'frame_id': 'map', 'units': 'm',
        'artifacts': {
            'canonical_map': 'canonical_map.json', 'gazebo_world': 'gazebo/warehouse.world',
            'gazebo_manifest': 'gazebo/manifest.json',
            'datamatrix_map': 'datamatrix_map.yaml', 'tag_graph': 'tag_graph.yaml',
            'nav2_map': 'nav2/warehouse_F1.yaml',
        },
        'gazebo_bounds': {'min_x': 0, 'min_y': 0, 'max_x': 1, 'max_y': 1},
        'nav2_maps': {'F1': 'nav2/warehouse_F1.yaml'},
        'nav2_bounds': {'F1': {'width': 1, 'height': 1, 'resolution': 1, 'origin': [0, 0, 0]}},
        'robots': [{'id': 'R02', 'floor_id': 'F1', 'pose': [0.5, 0.5, 0.1, 0.5]}],
        'sha256': {name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in files},
    }
    (root / 'manifest.json').write_text(json.dumps(manifest))


def test_verified_bundle_selects_world_nav2_tags_and_robot_spawn(tmp_path):
    _bundle(tmp_path)
    manifest, selected = verify_bundle(tmp_path, 12, 'R02')
    assert manifest['frame_id'] == 'map'
    assert selected['map'].endswith('nav2/warehouse_F1.yaml')
    assert selected['spawn'] == [0.5, 0.5, 0.1, 0.5]
    command = command_for_revision(
        ['ros2', 'launch', 'system.launch.py', 'world:=old', 'map_file:=old-map', 'mode:=navigation'],
        tmp_path, 12, 'R02',
    )
    assert 'world:=old' not in command
    assert 'map_file:=old-map' not in command
    assert any(arg.endswith('gazebo/warehouse.world') for arg in command)
    assert any(arg.endswith('nav2/warehouse_F1.yaml') for arg in command)


def test_bundle_revision_and_hash_mismatch_fail_closed(tmp_path):
    _bundle(tmp_path)
    with pytest.raises(ValueError, match='revision'):
        verify_bundle(tmp_path, 13, 'R02')
    (tmp_path / 'gazebo/warehouse.world').write_text('<sdf>mutated</sdf>')
    with pytest.raises(ValueError, match='hash mismatch'):
        verify_bundle(tmp_path, 12, 'R02')


def test_bundle_rejects_gazebo_bounds_that_disagree_with_canonical_map(tmp_path):
    _bundle(tmp_path)
    manifest = json.loads((tmp_path / 'manifest.json').read_text())
    manifest['gazebo_bounds']['max_x'] = 2
    (tmp_path / 'manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='Gazebo bounds'):
        verify_bundle(tmp_path, 12, 'R02')


def test_mode_transition_command_preserves_stack_args_and_sets_nav2_readiness_mode():
    base = ['ros2', 'launch', 'swerve_bringup', 'system.launch.py',
            'world:=warehouse.world', 'map_file:=warehouse.yaml',
            'mode:=mapping', 'defer_nav2_start:=false']
    navigation = command_for_mode(base, 'navigation')
    assert 'world:=warehouse.world' in navigation
    assert 'map_file:=warehouse.yaml' in navigation
    assert 'mode:=navigation' in navigation
    assert 'defer_nav2_start:=true' in navigation
    assert 'mode:=mapping' not in navigation
    mapping = command_for_mode(navigation, 'mapping')
    assert 'mode:=mapping' in mapping
    assert not any(arg.startswith('defer_nav2_start:=') for arg in mapping)
    resumed = command_for_mode(mapping, 'mapping',
                               slam_session_file='/maps/local/R02/session')
    assert 'slam_session_file:=/maps/local/R02/session' in resumed
    assert 'slam_start_at_dock:=true' in resumed
    fresh = command_for_mode(resumed, 'mapping')
    assert not any(arg.startswith('slam_session_file:=') for arg in fresh)
    assert not any(arg.startswith('slam_start_at_dock:=') for arg in fresh)
    with pytest.raises(ValueError, match='only start in mapping'):
        command_for_mode(mapping, 'navigation', slam_session_file='/maps/session')
    with pytest.raises(ValueError, match='mapping or navigation'):
        command_for_mode(base, 'local_sim')


def test_saved_slam_session_prefix_must_be_a_complete_robot_local_pair(tmp_path):
    root = tmp_path / 'generated' / 'maps'
    folder = root / 'local_robot_maps' / 'R02'
    folder.mkdir(parents=True)
    prefix = folder / 'saved_slam_session'
    prefix.with_suffix('.posegraph').write_bytes(b'graph')
    prefix.with_suffix('.data').write_bytes(b'sensor data')
    base = ['ros2', 'launch', 'system.launch.py', f'artifact_root:={root}']
    assert validate_slam_session_prefix(str(prefix), 'R02', base, tmp_path) == str(prefix)
    with pytest.raises(ValueError, match='robot local-map store'):
        validate_slam_session_prefix(str(tmp_path / 'outside'), 'R02', base, tmp_path)
    prefix.with_suffix('.data').unlink()
    with pytest.raises(FileNotFoundError):
        validate_slam_session_prefix(str(prefix), 'R02', base, tmp_path)


@pytest.mark.parametrize(('target_ready', 'expected_status', 'expected_mode'), [
    (True, 'READY', 'mapping'),
    (False, 'ROLLED_BACK', 'navigation'),
])
def test_supervisor_mode_change_gates_ready_or_restores_previous_mode(
    tmp_path, monkeypatch, target_ready, expected_status, expected_mode,
):
    class Child:
        def __init__(self):
            self.returncode = None
        def poll(self):
            return self.returncode
        def send_signal(self, _signum):
            self.returncode = 0
        def terminate(self):
            self.returncode = -15
        def kill(self):
            self.returncode = -9
        def wait(self, timeout=None):
            return self.returncode

    request_file = tmp_path / 'map-request.json'
    mode_request = tmp_path / 'mode-request.json'
    status_file = tmp_path / 'mode-status.json'
    stack_env = tmp_path / 'stack.env'
    request_file.write_text('{}')
    mode_request.write_text(json.dumps({
        'request_id': 'mode-test', 'robot_id': 'R01', 'mode': 'mapping',
    }))
    status_file.write_text(json.dumps({'robot_id': 'R01', 'mode': 'navigation', 'status': 'READY'}))
    stack_env.write_text('MODE=navigation\nROS_DOMAIN_ID=0\n')

    children = []
    def create_child(_command, **_kwargs):
        child = Child()
        children.append(child)
        return child
    monkeypatch.setattr(ros_stack_supervisor.subprocess, 'Popen', create_child)
    handlers = {}
    monkeypatch.setattr(ros_stack_supervisor.signal, 'signal', lambda number, handler: handlers.__setitem__(number, handler))
    checks = []
    def ready(_root, mode, _robot_id, _backend, _map_file, _log_path, _deadline, child, **_kwargs):
        checks.append(mode)
        if mode == 'mapping' and not target_ready:
            return False, 'mapping prerequisites failed'
        if mode == 'mapping' and target_ready:
            handlers[signal.SIGTERM](None, None)
        elif mode == 'navigation':
            handlers[signal.SIGTERM](None, None)
        return True, 'ready'
    monkeypatch.setattr(ros_stack_supervisor, '_run_readiness', ready)

    result = ros_stack_supervisor.run_supervisor(
        request_file, 12, 'R01',
        ['ros2', 'launch', 'system.launch.py', 'mode:=navigation', 'defer_nav2_start:=true'],
        mode_request_file=mode_request, mode_status_file=status_file,
        initial_mode='navigation', stack_env_file=stack_env,
        readiness_root=tmp_path, mode_timeout=1.0,
    )
    status = json.loads(status_file.read_text())
    assert result == 0
    assert status['status'] == expected_status
    assert stack_env.read_text().startswith(f'MODE={expected_mode}\n')
    assert checks == (['mapping'] if target_ready else ['mapping', 'navigation'])


def test_supervisor_restarts_same_mapping_mode_with_validated_saved_session(tmp_path, monkeypatch):
    class Child:
        def __init__(self, command):
            self.command = command
            self.returncode = None
        def poll(self):
            return self.returncode
        def send_signal(self, _signum):
            self.returncode = 0
        def terminate(self):
            self.returncode = -15
        def kill(self):
            self.returncode = -9
        def wait(self, timeout=None):
            return self.returncode

    folder = tmp_path / 'generated' / 'maps' / 'local_robot_maps' / 'R01'
    folder.mkdir(parents=True)
    prefix = folder / 'resume_slam_session'
    prefix.with_suffix('.posegraph').write_bytes(b'graph')
    prefix.with_suffix('.data').write_bytes(b'sensor data')
    request_file = tmp_path / 'map-request.json'
    mode_request = tmp_path / 'mode-request.json'
    status_file = tmp_path / 'mode-status.json'
    request_file.write_text('{}')
    mode_request.write_text(json.dumps({
        'request_id': 'resume-test', 'robot_id': 'R01', 'mode': 'mapping',
        'slam_session_file': str(prefix), 'force_restart': True,
    }))
    status_file.write_text(json.dumps({'robot_id': 'R01', 'mode': 'mapping', 'status': 'READY'}))
    children = []

    def create_child(command, **_kwargs):
        child = Child(command)
        children.append(child)
        return child

    monkeypatch.setattr(ros_stack_supervisor.subprocess, 'Popen', create_child)
    handlers = {}
    monkeypatch.setattr(ros_stack_supervisor.signal, 'signal',
                        lambda number, handler: handlers.__setitem__(number, handler))
    checks = []

    def ready(_root, mode, _robot_id, _backend, _map_file, _log_path, _deadline, child, **_kwargs):
        checks.append((mode, child.command))
        handlers[signal.SIGTERM](None, None)
        return True, 'mapping ready with restored SLAM session'

    monkeypatch.setattr(ros_stack_supervisor, '_run_readiness', ready)
    result = ros_stack_supervisor.run_supervisor(
        request_file, 12, 'R01',
        ['ros2', 'launch', 'system.launch.py', 'mode:=mapping'],
        mode_request_file=mode_request, mode_status_file=status_file,
        initial_mode='mapping', readiness_root=tmp_path, mode_timeout=1.0,
    )
    status = json.loads(status_file.read_text())
    assert result == 0
    assert status['status'] == 'READY'
    assert status['request_id'] == 'resume-test'
    assert len(children) == 2
    assert checks[0][0] == 'mapping'
    assert f'slam_session_file:={prefix}' in checks[0][1]
    assert 'slam_start_at_dock:=true' in checks[0][1]
