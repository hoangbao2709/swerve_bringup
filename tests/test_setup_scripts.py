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


def test_production_frontend_build_is_explicit_and_precedes_runtime_start():
    source = (ROOT / 'scripts/start_stack.sh').read_text()
    assert 'WARETWIN_FRONTEND_MODE:-production' in source
    assert 'development) FRONTEND_RUN_SCRIPT=dev' in source
    assert 'FRONTEND_RUN_SCRIPT=preview' in source
    assert source.index('npm run build') < source.index('echo "Starting backend')
    assert "VITE_BACKEND_PORT=\"$BACKEND_PORT_SELECTED\"" in source
    assert '--strictPort' in source


def test_backend_uses_worktree_python_not_copied_activate_path():
    source = (ROOT / 'waretwin/backend/run.sh').read_text()
    assert 'BACKEND_PYTHON_PATH="$PWD/.venv/bin/python"' in source
    assert 'source .venv/bin/activate' not in source
    assert 'pydantic, wsaccel' in source
    assert '"$BACKEND_PYTHON_PATH" manage.py runserver' in source


def test_bridge_hot_reload_registration_checks_worktree_and_real_executable():
    output = _run_bash('''
source scripts/stack_common.sh
stack_pid() { echo "$$"; }
stack_cmdline() { echo "python3 $STACK_ROOT/install/swerve_bridge/lib/swerve_bridge/swerve_bridge_node"; }
stack_owned_pid ros_bridge && echo owned
stack_cmdline() { echo "python3 /other/install/swerve_bridge/lib/swerve_bridge/swerve_bridge_node"; }
if stack_owned_pid ros_bridge; then exit 1; fi
echo unrelated-preserved
''')
    assert output.splitlines() == ['owned', 'unrelated-preserved']
    stop = (ROOT / 'scripts/stop_stack.sh').read_text()
    assert 'for component in ros_bridge ros frontend backend;' in stop
    start = (ROOT / 'scripts/start_stack.sh').read_text()
    assert 'stack_owned_pid ros_bridge' in start
    assert 'stack_owned_group ros_bridge' in start
