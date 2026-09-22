from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]


def _run_bash(command: str) -> str:
    result = subprocess.run(
        ['bash', '-c', command],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def test_cors_helper_always_includes_selected_loopback_origin_and_valid_ipv4s():
    output = _run_bash(
        'source scripts/stack_common.sh; stack_cors_origins 5174 "http://example.invalid:9"'
    )
    origins = output.split(',')
    assert 'http://localhost:5174' in origins
    assert 'http://127.0.0.1:5174' in origins
    assert 'http://example.invalid:9' in origins
    assert all(origin.startswith('http://') and 'http://:' not in origin for origin in origins)
    for origin in origins:
        host_port = origin.removeprefix('http://').rsplit(':', 1)
        assert len(host_port) == 2
        host, port = host_port
        if all(part.isdigit() for part in host.split('.')) and len(host.split('.')) == 4:
            assert all(0 <= int(part) <= 255 for part in host.split('.'))
        assert port.isdigit()


def test_setup_requests_sudo_only_after_missing_package_detection():
    source = (ROOT / 'scripts/setup_full_stack.sh').read_text(encoding='utf-8')
    missing_index = source.index('missing=()')
    runtime_check_index = source.index('runtime_repairs=()')
    missing_branch_index = source.index('if ((${#missing[@]} || ${#runtime_repairs[@]})); then')
    apt_index = source.index('"${SUDO[@]}" apt-get update')
    assert missing_index < runtime_check_index < missing_branch_index < apt_index
    assert source.count('sudo -v') == 1
    assert source.index('SUDO_READY=0') < missing_index
    assert 'rosdep db' in source
    assert source.index('rosdep db') < source.index('rosdep init')


def test_setup_and_preflight_cover_controller_plugin_runtime_dependencies():
    setup = (ROOT / 'scripts/setup_full_stack.sh').read_text(encoding='utf-8')
    preflight = (ROOT / 'scripts/preflight_check.sh').read_text(encoding='utf-8')
    for package in ('ros-humble-joint-state-broadcaster',
                    'ros-humble-position-controllers',
                    'ros-humble-velocity-controllers'):
        assert package in setup
    for package in ('joint_state_broadcaster', 'position_controllers', 'velocity_controllers'):
        assert package in preflight


def test_controller_spawner_uses_humble_compatible_cli():
    launch = (ROOT / 'launch/gazebo.launch.py').read_text(encoding='utf-8')
    assert '--controller-manager-timeout' in launch
    assert '--service-call-timeout' not in launch
    assert '--switch-timeout' not in launch
