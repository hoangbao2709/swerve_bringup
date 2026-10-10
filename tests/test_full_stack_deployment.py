"""Opt-in installed Gazebo/Nav2 readiness and end-to-end motion acceptance."""
import json
import os
from pathlib import Path
import signal
import select
import shutil
import socket
import subprocess
import time
from urllib.request import urlopen

import pytest
from ament_index_python.packages import get_package_share_directory

pytestmark = pytest.mark.skipif(os.environ.get('WARETWIN_SIMULATION_TEST') != '1',
    reason='Gazebo/Nav2 acceptance is opt-in; set WARETWIN_SIMULATION_TEST=1 after build')

ROOT = Path(__file__).resolve().parents[1]


def free_port():
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        return listener.getsockname()[1]


def test_installed_full_stack_reaches_readiness_and_executes_simulated_mission(tmp_path):
    runtime = tmp_path / 'runtime'
    runtime.mkdir()
    runtime.chmod(0o700)
    maps = runtime / 'maps'
    maps.mkdir(mode=0o700)
    maps.chmod(0o700)
    config = runtime / 'config.env'
    config.write_text(f'WARETWIN_DATABASE_PATH={runtime / "db.sqlite3"}\n'
                      f'WARETWIN_ARTIFACT_ROOT={maps}\n'
                      'WARETWIN_RUNTIME_MODE=GAZEBO_ROS\n')
    backend_port, frontend_port = str(free_port()), str(free_port())
    state_home = Path(os.environ.get('XDG_STATE_HOME', Path.home() / '.local/state')).expanduser()
    backend_python = os.environ.get('WARETWIN_TEST_PYTHON', str(state_home / 'waretwin/venv/bin/python'))
    if not Path(backend_python).is_file():
        pytest.fail(f'Provision the isolated WareTwin Python environment first: {backend_python}')
    share = Path(get_package_share_directory('waretwin_web'))
    backend = share / 'backend'
    seed_env = {
        **os.environ,
        'PYTHONPATH': str(backend),
        'WARETWIN_RUNTIME_DIR': str(runtime),
        'WARETWIN_DATABASE_PATH': str(runtime / 'db.sqlite3'),
        'WARETWIN_LOG_DIR': str(runtime / 'logs'),
        'WARETWIN_ARTIFACT_ROOT': str(maps),
        'WARETWIN_OPERATOR_AUTH_MODE': 'LOCAL_LOOPBACK',
    }
    migrate = subprocess.run([backend_python, str(backend / 'manage.py'), 'migrate', '--noinput'],
        cwd=backend, env=seed_env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, timeout=120, check=False)
    assert migrate.returncode == 0, migrate.stdout
    fixture = ROOT / 'generated/maps/WH-TEST-01/23/canonical_map.json'
    seed_code = (
        'import json; from pathlib import Path; '
        'from twin.warehouse_services import ensure_active_map, publish_layout_to_map; '
        f'layout=json.loads(Path({str(fixture)!r}).read_text()); '
        "layout['id']='WH-TEST-01'; layout['revision']=22; layout['published_version']=11; "
        'active=ensure_active_map(layout); active.revision=22; active.published_version=11; '
        'active.layout=layout; active.draft=layout; active.save(); '
        'published=publish_layout_to_map(layout); '
        'assert published.revision == 23 and published.published_version == 12'
    )
    seed = subprocess.run([backend_python, str(backend / 'manage.py'), 'shell', '--verbosity', '0', '-c', seed_code],
        cwd=backend, env=seed_env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, timeout=180, check=False)
    assert seed.returncode == 0, seed.stdout
    domain = '213'
    env = os.environ.copy()
    env.pop('WARETWIN_ARTIFACT_ROOT', None)
    env.pop('WARETWIN_DATABASE_PATH', None)
    env['ROS_LOG_DIR'] = str(runtime / 'ros-logs')
    (runtime / 'ros-logs').mkdir()
    launch = subprocess.Popen(['ros2', 'launch', 'swerve_bringup', 'full_stack.launch.py',
        f'runtime_dir:={runtime}', f'config_file:={config}', f'backend_port:={backend_port}',
        f'frontend_port:={frontend_port}', f'ros_domain_id:={domain}', f'backend_python:={backend_python}', 'mode:=unified',
        'use_sim:=true', 'gui:=false', 'start_rviz:=false', 'ros_timeout:=600'],
        cwd='/tmp', env={**env, 'ROS_DOMAIN_ID': domain}, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, start_new_session=True)
    output = []
    deadline = time.monotonic() + 600
    try:
        while time.monotonic() < deadline:
            if select.select([launch.stdout], [], [], min(1, max(0.0, deadline - time.monotonic())))[0]:
                line = launch.stdout.readline()
                if line:
                    output.append(line)
            if launch.poll() is not None:
                raise AssertionError('Installed full-stack launch exited before readiness:\n' + ''.join(output))
            try:
                status = json.loads((runtime / 'web-status.json').read_text())
                if status.get('state') == 'READY':
                    break
                if status.get('state') in ('ERROR', 'DEGRADED'):
                    raise AssertionError(f'Runtime state {status["state"]}: {status}\n' + ''.join(output))
            except (OSError, ValueError):
                pass
        else:
            raise AssertionError('Existing navigation readiness did not pass within 600 s:\n' + ''.join(output))
        with urlopen(f'http://127.0.0.1:{backend_port}/api/health/', timeout=5) as response:
            health = json.load(response)
        assert health['database'] is True
        assert health['ros_bridge'] is True
        assert health['gazebo'] is True
        mode = json.loads((runtime / 'mode-switch-status.json').read_text())
        assert mode['status'] == 'READY', mode
        assert mode['mode'] == 'unified'
        assert (maps / 'WH-TEST-01/23/manifest.json').is_file()
        mission_json = runtime / 'end-to-end.json'
        mission_log = runtime / 'end-to-end.log'
        python = shutil.which('python3')
        assert python, 'ROS Python 3 executable is unavailable'
        mission = subprocess.run([python, str(ROOT / 'scripts/end_to_end_acceptance.py'),
            '--backend-url', f'http://127.0.0.1:{backend_port}',
            '--frontend-origin', f'http://127.0.0.1:{frontend_port}',
            '--navigation-timeout', '240', '--json', str(mission_json)],
            cwd=ROOT, env={**env, 'ROS_DOMAIN_ID': domain},
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            timeout=900, check=False)
        mission_log.write_text(mission.stdout)
        report = json.loads(mission_json.read_text())
        save_load_json = runtime / 'save-load-navigation.json'
        save_load_log = runtime / 'save-load-navigation.log'
        navigation_runs = int(os.environ.get('WARETWIN_SIMULATION_NAVIGATION_RUNS', '5'))
        assert 1 <= navigation_runs <= 10, (
            'WARETWIN_SIMULATION_NAVIGATION_RUNS must be between 1 and 10')
        save_load = subprocess.run([python, str(ROOT / 'scripts/save_load_map_acceptance.py'),
            '--backend-url', f'http://127.0.0.1:{backend_port}',
            '--frontend-origin', f'http://127.0.0.1:{frontend_port}',
            '--artifact-root', str(maps), '--runtime-dir', str(runtime),
            '--robot-id', 'R01', '--navigation-runs', str(navigation_runs),
            '--navigation-timeout', '240', '--json', str(save_load_json)],
            cwd='/tmp', env={**env, 'ROS_DOMAIN_ID': domain},
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            timeout=3600, check=False)
        save_load_log.write_text(save_load.stdout)
        assert save_load.returncode == 0, (
            f'Save/Load acceptance exited {save_load.returncode}; full output: {save_load_log}\n'
            f'last output lines:\n{save_load.stdout[-12000:]}'
        )
        save_load_report = json.loads(save_load_json.read_text())
        assert save_load_report['passed'] is True, (
            f'Save/Load report failed: phases={save_load_report.get("phases")}, '
            f'reason={save_load_report.get("reason")}; report={save_load_json}'
        )
        assert len(save_load_report['navigation_runs']) == navigation_runs, (
            f'Expected {navigation_runs} saved-map Nav2 runs, '
            f'got {len(save_load_report["navigation_runs"])}; '
            f'report={save_load_json}'
        )
        # Keep the real Save/Load integration evidence even when the preceding
        # wider regression mission exposes a navigation failure. Both gates
        # remain required for this test to pass; this only avoids masking the
        # independent map-persistence acceptance report.
        assert mission.returncode == 0, (
            f'End-to-end acceptance exited {mission.returncode}; full output: {mission_log}\n'
            f'last output lines:\n{mission.stdout[-12000:]}'
        )
        assert report['passed'] is True, (
            f'End-to-end report failed: stages={report.get("stages")}, '
            f'direct_navigation={report.get("direct_navigation")}, '
            f'web_navigation={report.get("web_navigation")}; report={mission_json}'
        )
    finally:
        interrupted = launch.poll() is None
        if interrupted:
            launch.send_signal(signal.SIGINT)
            try:
                launch.wait(timeout=60)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait(timeout=5)
        if launch.stdout:
            output.extend(launch.stdout.readlines())
        if interrupted:
            assert launch.returncode == 0, 'Full-stack shutdown failed:\n' + ''.join(output)
