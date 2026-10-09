"""Real installed launch/HTTP/WebSocket/lifecycle tests (opt-in after build).

Run with WARETWIN_DEPLOYMENT_TEST=1 after sourcing install/setup.bash.
"""
import json
import os
from pathlib import Path
import signal
import shutil
import socket
import sqlite3
import subprocess
import time
from urllib.request import urlopen
from urllib.error import HTTPError

import pytest
import websocket

pytestmark = pytest.mark.skipif(os.environ.get('WARETWIN_DEPLOYMENT_TEST') != '1',
                               reason='installed deployment suite: set WARETWIN_DEPLOYMENT_TEST=1 after build')


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def wait_for(predicate, timeout=90):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            value = predicate()
            if value:
                return value
        except (OSError, ValueError):
            pass
        time.sleep(.2)
    raise AssertionError('condition did not become true before deadline')


def alive(pid):
    path = Path(f'/proc/{pid}/stat')
    return path.exists() and path.read_text().split(')')[1].split()[0] != 'Z'


class Deployment:
    def __init__(self, root, **kwargs):
        self.root = root
        self.root.mkdir(parents=True, exist_ok=True)
        self.backend = str(kwargs.pop('backend_port', free_port()))
        self.frontend = str(free_port())
        self.args = dict(runtime_dir=str(root), backend_port=self.backend, frontend_port=self.frontend,
                         backend_python=os.environ.get('WARETWIN_TEST_PYTHON', str(Path.home() / '.local/state/waretwin/venv/bin/python')),
                         startup_timeout='30')
        self.args.update(kwargs)
        self.log = (root / 'launch.log').open('a')
        self.children = []

    def start(self, full=False):
        env = os.environ.copy()
        for key in ('WARETWIN_DATABASE_PATH', 'WARETWIN_RUNTIME_MODE', 'WARETWIN_ARTIFACT_ROOT', 'ROS_WS_URL'):
            env.pop(key, None)
        self.process = subprocess.Popen(['ros2', 'launch', 'swerve_bringup' if full else 'waretwin_web',
            'full_stack.launch.py' if full else 'web.launch.py',
            *(f'{key}:={value}' for key, value in self.args.items())], cwd='/tmp', env=env,
            stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

    def ready(self):
        def check():
            if self.process.poll() is not None:
                raise AssertionError((self.root / 'launch.log').read_text())
            status = json.loads((self.root / 'web-status.json').read_text())
            return status if status['state'] == 'WEB_READY' else None
        status = wait_for(check)
        self.runtime_pid = status['pid']
        self.frontend_url = status['frontend_url']
        self.backend_url = status['backend_url']
        self.children = [int(pid) for pid in Path(f'/proc/{self.runtime_pid}/task/{self.runtime_pid}/children').read_text().split()]
        return status

    def get(self, path, backend=False):
        return urlopen(f'http://127.0.0.1:{self.backend if backend else self.frontend}{path}', timeout=5)

    def stop(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
        code = self.process.wait(timeout=55)
        self.log.flush()
        for pid in self.children:
            assert not alive(pid), f'orphan child PID {pid}'
        return code


@pytest.fixture
def web(tmp_path):
    deployment = Deployment(tmp_path / 'runtime')
    deployment.start()
    try:
        deployment.ready()
        yield deployment
    finally:
        deployment.stop()
        deployment.log.close()


def test_installed_http_websocket_assets_and_spa(web):
    with web.get('/api/health/', backend=True) as response:
        assert json.load(response)['database'] is True
    with web.get('/api/layout', backend=True) as response:
        assert isinstance(json.load(response), dict)
    share = Path(subprocess.check_output(['ros2', 'pkg', 'prefix', 'waretwin_web'], text=True).strip()) / 'share/waretwin_web'
    python = web.args['backend_python']
    module_env = {**os.environ, 'PYTHONPATH': str(share / 'backend'),
        'WARETWIN_RUNTIME_DIR': str(web.root), 'WARETWIN_LOG_DIR': str(web.root / 'logs'),
        'WARETWIN_DATABASE_PATH': str(web.root / 'db.sqlite3'),
        'WARETWIN_ARTIFACT_ROOT': str(web.root / 'maps')}
    imported = subprocess.run([python, str(share / 'backend/manage.py'), 'shell', '--verbosity', '0', '-c',
        'import inspect; from twin.map_artifacts import export_gazebo_world; print(inspect.getsourcefile(export_gazebo_world))'],
        cwd='/tmp', env=module_env, capture_output=True, text=True, timeout=20)
    assert imported.returncode == 0, imported.stdout + imported.stderr
    assert str(share / 'backend/tools/export_gazebo_world.py') in imported.stdout
    with web.get('/runtime-config.js') as response:
        config = response.read().decode()
        assert web.backend in config
        assert response.headers['Cache-Control'] == 'no-store'
    with web.get('/') as response:
        html = response.read().decode()
    import re
    asset = re.search(r'src="([^"]+\.js)"', html)
    assert asset
    with web.get(asset.group(1)) as response:
        assert 'javascript' in response.headers['Content-Type']
    with web.get('/robots/R01/control') as response:
        assert response.read().decode() == html
    with pytest.raises(HTTPError) as error:
        web.get('/assets/missing.js')
    assert error.value.code == 404
    ws = websocket.create_connection(f'ws://127.0.0.1:{web.backend}/ws', timeout=15)
    try:
        assert json.loads(ws.recv())['type'] == 'FULL'
    finally:
        ws.close()
    # ROS endpoint exists and rejects unauthenticated bridges.
    with pytest.raises(websocket.WebSocketBadStatusException):
        websocket.create_connection(f'ws://127.0.0.1:{web.backend}/ws/ros', timeout=5)


def test_browser_renders_installed_frontend_without_script_errors(web):
    node = shutil.which('node')
    if not node:
        pytest.skip('Node.js/Playwright browser checks require the build-time Node environment')
    script = r"""
import { chromium } from 'playwright';
const browser = await chromium.launch({headless:true,args:['--no-sandbox']});
try {
  const page = await browser.newPage();
  const errors=[];
  page.on('pageerror', error => errors.push(error.message));
  await page.goto(process.env.WARETWIN_TEST_URL, {waitUntil:'domcontentloaded'});
  await page.waitForSelector('#root > *', {timeout:20000});
  await page.waitForTimeout(2000);
  if (errors.length) throw new Error(errors.join('\n'));
  const tabs=await page.locator('[role="tab"]').evaluateAll(nodes => nodes.map(node => node.getAttribute('aria-label')));
  if (JSON.stringify(tabs)!==JSON.stringify(['CONTROL','MAPPING','LOCALIZATION','VDA5050','DIAGNOSIS'])) throw new Error(`unexpected sidebar: ${JSON.stringify(tabs)}`);
  await page.getByRole('tab',{name:'MAPPING'}).click();
  await page.getByText('ACCUMULATED SLAM MAP',{exact:false}).waitFor();
  const mapping=await page.locator('.hmi-mapping-workflow-layout').evaluate(node => ({
    panels:Array.from(node.querySelectorAll(':scope > .local-section-panel > header')).map(header => header.textContent),
    lists:node.querySelectorAll('.hmi-mapping-library-list').length,
    loadButtons:node.querySelectorAll('.hmi-map-load-button').length,
    overflowY:getComputedStyle(node).overflowY,
    scrollHeight:node.scrollHeight,
    clientHeight:node.clientHeight,
  }));
  if (mapping.lists!==1 || mapping.loadButtons!==1 || mapping.panels.length!==8
      || mapping.overflowY!=='auto' || mapping.scrollHeight<=mapping.clientHeight) throw new Error(`incomplete or non-scrollable mapping workflow: ${JSON.stringify(mapping)}`);
  const result=await page.evaluate(() => ({config:window.WARETWIN_CONFIG, text:document.querySelector('#root').innerText}));
  if (result.config.VITE_BACKEND_PORT !== process.env.WARETWIN_TEST_BACKEND) throw new Error('runtime backend config mismatch');
  console.log(JSON.stringify({rendered:result.text.length>0,backendPort:result.config.VITE_BACKEND_PORT,sidebar:tabs,mappingPanels:mapping.panels.length}));
} finally { await browser.close(); }
"""
    completed = subprocess.run([node, '--input-type=module', '-e', script],
        cwd=Path(__file__).resolve().parents[1] / 'waretwin/frontend',
        env={**os.environ, 'WARETWIN_TEST_URL': web.frontend_url,
             'WARETWIN_TEST_BACKEND': web.backend}, capture_output=True, text=True, timeout=45)
    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert '"rendered":true' in completed.stdout


def test_persistence_and_repeated_launch(web):
    db = web.root / 'db.sqlite3'
    with sqlite3.connect(db) as connection:
        connection.execute('CREATE TABLE deployment_sentinel (value TEXT)')
        connection.execute("INSERT INTO deployment_sentinel VALUES ('preserved')")
    secrets = (web.root / 'secrets.json').read_bytes()
    assert (web.root / 'secrets.json').stat().st_mode & 0o777 == 0o600
    assert web.stop() == 0
    web.start()
    web.ready()
    with sqlite3.connect(db) as connection:
        assert connection.execute('SELECT value FROM deployment_sentinel').fetchone()[0] == 'preserved'
    assert (web.root / 'secrets.json').read_bytes() == secrets


@pytest.mark.parametrize('component', ['backend', 'frontend'])
def test_essential_failure_stops_owned_children(web, component):
    pid = next(pid for pid in web.children if
               ('daphne' if component == 'backend' else 'static_server.py') in Path(f'/proc/{pid}/cmdline').read_text())
    os.kill(pid, signal.SIGTERM)
    assert web.process.wait(timeout=55) != 0
    for child in web.children:
        assert not alive(child)


@pytest.mark.parametrize('arguments', [dict(ros_domain_id='999'), dict(backend_python='/missing/python'),
                                      dict(config_file='/missing/config.env')])
def test_invalid_inputs_fail_launch(tmp_path, arguments):
    deployment = Deployment(tmp_path / 'runtime', **arguments)
    deployment.start()
    assert deployment.process.wait(timeout=40) != 0
    deployment.log.close()


def test_occupied_port_is_preserved(tmp_path):
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        listener.listen()
        deployment = Deployment(tmp_path / 'runtime', backend_port=listener.getsockname()[1])
        deployment.start()
        assert deployment.process.wait(timeout=40) != 0
        assert listener.fileno() >= 0
        deployment.log.close()


def test_duplicate_and_legacy_start_are_rejected(web, tmp_path):
    other = Deployment(tmp_path / 'second')
    other.start()
    assert other.process.wait(timeout=30) != 0
    assert 'already owns' in (other.root / 'launch.log').read_text()
    legacy = subprocess.run([str(Path(__file__).resolve().parents[1] / 'scripts/start_stack.sh')],
                            capture_output=True, text=True, timeout=30)
    assert legacy.returncode != 0
    assert 'already owns' in legacy.stderr
    assert web.process.poll() is None
    other.log.close()


def test_full_stack_fails_closed_without_canonical_bundle(tmp_path):
    deployment = Deployment(tmp_path / 'full', ros_timeout='10')
    deployment.start(full=True)
    assert deployment.process.wait(timeout=90) != 0
    assert 'No valid published canonical bundle' in (deployment.root / 'launch.log').read_text()
    status = json.loads((deployment.root / 'web-status.json').read_text())
    assert status['state'] == 'STOPPED'
    assert not (deployment.root / 'logs/ros.log').exists()
    deployment.log.close()
