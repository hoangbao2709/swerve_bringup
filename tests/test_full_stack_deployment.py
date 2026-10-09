"""Opt-in Gazebo/Nav2 acceptance against the installed full-stack launch."""
import json
import os
from pathlib import Path
import signal
import select
import shutil
import socket
import sqlite3
import subprocess
import time
from urllib.request import urlopen

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('WARETWIN_SIMULATION_TEST') != '1',
    reason='Gazebo/Nav2 acceptance is opt-in; set WARETWIN_SIMULATION_TEST=1 after build')

ROOT = Path(__file__).resolve().parents[1]


def free_port():
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        return listener.getsockname()[1]


def test_installed_full_stack_reaches_existing_navigation_readiness(tmp_path):
    runtime = tmp_path / 'runtime'
    runtime.mkdir()
    maps = runtime / 'maps'
    shutil.copytree(ROOT / 'generated/maps/WH-TEST-01', maps / 'WH-TEST-01')
    source = sqlite3.connect(f'file:{ROOT / "waretwin/backend/db.sqlite3"}?mode=ro', uri=True)
    target = sqlite3.connect(runtime / 'db.sqlite3')
    try:
        source.backup(target)
    finally:
        target.close()
        source.close()
    config = runtime / 'config.env'
    config.write_text(f'WARETWIN_DATABASE_PATH={runtime / "db.sqlite3"}\n'
                      f'WARETWIN_ARTIFACT_ROOT={maps}\n'
                      'WARETWIN_RUNTIME_MODE=GAZEBO_ROS\n')
    backend_port, frontend_port = str(free_port()), str(free_port())
    state_home = Path(os.environ.get('XDG_STATE_HOME', Path.home() / '.local/state')).expanduser()
    backend_python = os.environ.get('WARETWIN_TEST_PYTHON', str(state_home / 'waretwin/venv/bin/python'))
    if not Path(backend_python).is_file():
        pytest.fail(f'Provision the isolated WareTwin Python environment first: {backend_python}')
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
                if status.get('state') in ('ERROR', 'UNHEALTHY'):
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
