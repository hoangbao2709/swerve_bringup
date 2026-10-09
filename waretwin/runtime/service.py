#!/usr/bin/env python3
"""Installed web runtime. Owns child groups; never installs dependencies at launch."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import secrets
import shlex
import signal
import socket
import subprocess
import sys
import time
from urllib.request import urlopen


def default_runtime():
    return Path(os.environ.get('WARETWIN_RUNTIME_DIR') or state_home() / 'waretwin').expanduser().resolve()


def state_home():
    configured = Path(os.environ.get('XDG_STATE_HOME', Path.home() / '.local/state')).expanduser()
    return configured.resolve() if configured.is_absolute() else (Path.home() / '.local/state').resolve()


def load_config(path):
    values = {}
    for number, line in enumerate(Path(path).read_text().splitlines(), 1):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        key, separator, raw = line.removeprefix('export ').partition('=')
        if not separator or not key.replace('_', '').isalnum():
            raise ValueError(f'Invalid config line {number}: expected KEY=VALUE')
        parts = shlex.split(raw, comments=True)
        if len(parts) > 1:
            raise ValueError(f'Invalid config line {number}: quote values containing spaces')
        values[key] = parts[0] if parts else ''
    return values


def stop_group(process):
    # Only groups created with start_new_session=True by this runtime qualify.
    for sig, delay in ((signal.SIGINT, 12), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + delay
        while time.monotonic() < deadline:
            process.poll()
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                break
            time.sleep(.1)
        else:
            continue
        break
    process.wait(timeout=3)


class Runtime:
    def __init__(self, args):
        self.args = args
        self.share = Path(args.share).resolve(strict=True)
        self.root = Path(args.runtime_dir).expanduser().resolve() if args.runtime_dir else default_runtime()
        self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
        config = Path(args.config_file).expanduser() if args.config_file else self.root / 'config.env'
        self.env = {**(load_config(config) if config.exists() else {}), **os.environ}
        if args.config_file and not config.is_file():
            raise ValueError(f'Configuration file does not exist: {config}')
        for key in ('backend_host', 'backend_port', 'frontend_host', 'frontend_port', 'ros_domain_id'):
            if getattr(args, key):
                self.env[key.upper()] = getattr(args, key)
        self.children = []
        self.logs = []
        self.stopping = False
        self.python = str(Path(args.backend_python or self.env.get('WARETWIN_BACKEND_PYTHON') or self.root / 'venv/bin/python').expanduser().absolute())

    def prepare(self):
        # Shared across launch methods, runtime directories, and checkouts.
        lock_root = state_home() / 'waretwin'
        lock_root.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.lock = (lock_root / 'stack.lock').open('a')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('A WareTwin launch or legacy stack already owns the runtime') from exc
        self.owns_lock = True
        self.registry = lock_root / 'active-runtime.json'
        temporary = self.registry.with_suffix('.tmp')
        temporary.write_text(json.dumps({'runtime_dir': str(self.root), 'pid': os.getpid()}))
        temporary.replace(self.registry)
        env = self.env
        domain = int(env.get('ROS_DOMAIN_ID', '0'))
        if not 0 <= domain <= 232:
            raise ValueError('ROS_DOMAIN_ID must be between 0 and 232')
        env['ROS_DOMAIN_ID'] = str(domain)
        compact = env.get('WARETWIN_DEMO_VISUAL', 'true').strip().lower()
        if compact not in ('true', 'false', '1', '0', 'yes', 'no', 'on', 'off'):
            raise ValueError('WARETWIN_DEMO_VISUAL must be a boolean')
        env['WARETWIN_DEMO_VISUAL'] = 'true' if compact in ('true', '1', 'yes', 'on') else 'false'
        mode = 'GAZEBO_ROS' if self.args.full_stack and self.args.use_sim == 'true' else 'REAL_ROBOT' if self.args.full_stack else env.get('WARETWIN_RUNTIME_MODE', 'LOCAL_SIM')
        env['WARETWIN_RUNTIME_MODE'] = mode
        secret_path = self.root / 'secrets.json'
        if secret_path.exists():
            saved = json.loads(secret_path.read_text())
        else:
            saved = {key: secrets.token_urlsafe(48) for key in ('DJANGO_SECRET_KEY', 'WARETWIN_ROS_BRIDGE_TOKEN')}
            fd = os.open(secret_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(fd, 'w') as stream:
                json.dump(saved, stream)
        for key, value in saved.items():
            if not env.get(key) or env[key] in ('change-me', 'change-me-in-production', 'development-only-change-me'):
                env[key] = value
        env.update(WARETWIN_RUNTIME_DIR=str(self.root), WARETWIN_LOG_DIR=str(self.root / 'logs'),
                   WARETWIN_STACK_RUNTIME_DIR=str(self.root),
                   WARETWIN_NAV2_LIFECYCLE_STATE_FILE=str(self.root / 'nav2-lifecycle-startup.json'),
                   WARETWIN_STACK_START_MONOTONIC_S=str(time.monotonic()))
        env['WARETWIN_ARTIFACT_ROOT'] = str(Path(env.get('WARETWIN_ARTIFACT_ROOT') or self.root / 'maps').expanduser().resolve())
        (self.root / 'logs').mkdir(exist_ok=True)
        Path(env['WARETWIN_ARTIFACT_ROOT']).mkdir(parents=True, exist_ok=True)
        env.setdefault('BACKEND_HOST', '127.0.0.1')
        env.setdefault('BACKEND_PORT', '8000')
        env.setdefault('FRONTEND_HOST', '127.0.0.1')
        env.setdefault('FRONTEND_PORT', '5173')
        if env['BACKEND_PORT'] == env['FRONTEND_PORT']:
            raise ValueError('Backend and frontend ports must differ')
        for component in ('BACKEND', 'FRONTEND'):
            port = int(env[f'{component}_PORT'])
            if not 0 < port < 65536:
                raise ValueError(f'{component}_PORT must be between 1 and 65535')
            with socket.socket() as probe:
                probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                probe.bind((env[f'{component}_HOST'], port))
        self.backend_url = f"http://{self.probe_host(env['BACKEND_HOST'])}:{env['BACKEND_PORT']}"
        env['ROS_WS_URL'] = self.backend_url.replace('http:', 'ws:') + '/ws/ros'
        self.frontend_url = f"http://{self.probe_host(env['FRONTEND_HOST'])}:{env['FRONTEND_PORT']}"
        hosts = {'localhost', '127.0.0.1', socket.gethostname()}
        for candidate in (socket.gethostname(), env['BACKEND_HOST'], env['FRONTEND_HOST']):
            if candidate not in ('0.0.0.0', '::'):
                hosts.add(candidate)
                try:
                    hosts.update(socket.gethostbyname_ex(candidate)[2])
                except OSError:
                    pass
        env['DJANGO_ALLOWED_HOSTS'] = ','.join(dict.fromkeys(
            [*(x.strip() for x in env.get('DJANGO_ALLOWED_HOSTS', '').split(',') if x.strip()), *sorted(hosts)]))
        origins = {f'http://{host}:{env["FRONTEND_PORT"]}' for host in hosts}
        env['CORS_ALLOWED_ORIGINS'] = ','.join(dict.fromkeys(
            [*(x.strip() for x in env.get('CORS_ALLOWED_ORIGINS', '').split(',') if x.strip()), *sorted(origins)]))
        self.backend_env = env.copy()
        for key in ('PYTHONPATH', 'PYTHONHOME', 'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH'):
            self.backend_env.pop(key, None)
        self.backend_env['PYTHONDONTWRITEBYTECODE'] = '1'
        # Isolated web interpreter, explicit application roots only.
        self.backend_env['PYTHONPATH'] = str(self.share / 'backend')
        subprocess.run([self.python, '-c', 'import django, channels, daphne, dotenv, pydantic, websocket, wsaccel, openpyxl, yaml; import sys; assert sys.version_info[:2] == (3,10); assert sys.prefix != sys.base_prefix'], env=self.backend_env, check=True)
        subprocess.run([self.python, '-m', 'pip', 'check'], env=self.backend_env, check=True)
        if not (self.share / 'frontend/index.html').is_file():
            raise RuntimeError('Installed frontend index.html is missing; rebuild waretwin_web')

    @staticmethod
    def probe_host(host):
        return '127.0.0.1' if host == '0.0.0.0' else host

    def spawn(self, name, command, env=None, cwd=None):
        stream = (self.root / 'logs' / f'{name}.log').open('a')
        self.logs.append(stream)
        process = subprocess.Popen(command, env=env or self.env, cwd=cwd or self.root,
                                   start_new_session=True, stdout=stream, stderr=subprocess.STDOUT)
        self.children.append((name, process))
        return process

    def check_children(self):
        for name, process in self.children:
            if process.poll() is not None:
                raise RuntimeError(f'{name} exited with status {process.returncode}; see {self.root}/logs/{name}.log')

    def wait_http(self, url):
        deadline = time.monotonic() + self.args.startup_timeout
        while not self.stopping and time.monotonic() < deadline:
            self.check_children()
            try:
                with urlopen(url, timeout=2) as response:
                    if response.status == 200:
                        return
            except OSError:
                pass
            time.sleep(.2)
        raise RuntimeError(f'Startup interrupted or HTTP readiness timed out: {url}')

    def run(self):
        def stop(_sig, _frame):
            self.stopping = True
        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGINT, stop)
        try:
            self.prepare()
            for command in ('check', 'migrate', 'sync_master_data'):
                args = [self.python, str(self.share / 'backend/manage.py'), command]
                if command == 'migrate':
                    args.append('--noinput')
                process = self.spawn('prepare-' + command, args, self.backend_env)
                while process.poll() is None and not self.stopping:
                    time.sleep(.1)
                if self.stopping or process.returncode:
                    raise RuntimeError(f'{command} failed or interrupted; inspect runtime logs')
                self.children.remove(('prepare-' + command, process))
            self.spawn('backend', [self.python, '-m', 'daphne', '-b', self.env['BACKEND_HOST'],
                                  '-p', self.env['BACKEND_PORT'], 'config.asgi:application'], self.backend_env)
            self.wait_http(self.backend_url + '/api/health/')
            self.spawn('frontend', [sys.executable, str(self.share / 'runtime/static_server.py'),
                '--directory', str(self.share / 'frontend'), '--host', self.env['FRONTEND_HOST'],
                '--port', self.env['FRONTEND_PORT'], '--backend-port', self.env['BACKEND_PORT'],
                '--api-url', self.env.get('VITE_API_BASE_URL', ''), '--ws-url', self.env.get('VITE_WS_BASE_URL', ''),
                '--runtime-mode', self.env['WARETWIN_RUNTIME_MODE'],
                '--demo-compact-view', self.env['WARETWIN_DEMO_VISUAL']])
            self.wait_http(self.frontend_url)
            if self.args.full_stack:
                self.start_ros()
            self.write_status('STARTING' if self.args.full_stack else 'WEB_READY')
            print(f"WareTwin web ready: {self.frontend_url}; backend: {self.backend_url}", flush=True)
            failures = 0
            while not self.stopping:
                self.check_children()
                try:
                    with urlopen(self.backend_url + '/api/health/', timeout=2) as response:
                        health = json.load(response)
                    with urlopen(self.frontend_url + '/runtime-config.js', timeout=2) as response:
                        response.read()
                    if health.get('database') is not True:
                        raise RuntimeError('Backend database health failed')
                    failures = 0
                    state = 'WEB_READY'
                    if self.args.full_stack:
                        mode_status = json.loads((self.root / 'mode-switch-status.json').read_text())
                        state = mode_status.get('status', 'STARTING')
                        if state == 'READY' and not health.get('ros_bridge'):
                            state = 'BRIDGE_DISCONNECTED'
                    self.write_status(state, health)
                except OSError:
                    failures += 1
                    self.write_status('UNHEALTHY')
                    if failures >= 3:
                        raise RuntimeError('Essential HTTP service failed three consecutive probes')
                time.sleep(1)
            return 0
        finally:
            for _name, process in reversed(self.children):
                stop_group(process)
            if getattr(self, 'owns_lock', False):
                self.write_status('STOPPED')
                try:
                    if json.loads(self.registry.read_text()).get('pid') == os.getpid():
                        self.registry.unlink()
                except (OSError, ValueError):
                    pass
            for stream in self.logs:
                stream.close()

    def write_status(self, state, health=None):
        payload = dict(state=state, pid=os.getpid(), backend_url=getattr(self, 'backend_url', None),
                       frontend_url=getattr(self, 'frontend_url', None), health=health,
                       runtime_dir=str(self.root), process_start=Path(f'/proc/{os.getpid()}/stat').read_text().split(')')[1].split()[19])
        temporary = self.root / 'web-status.json.tmp'
        temporary.write_text(json.dumps(payload))
        temporary.replace(self.root / 'web-status.json')

    def start_ros(self):
        from ament_index_python.packages import get_package_share_directory
        bringup = Path(get_package_share_directory('swerve_bringup'))
        sys.path.insert(0, str(bringup / 'scripts'))
        from ros_stack_supervisor import verify_bundle
        # Read canonical selection with the SAME interpreter/settings/database
        # as Daphne. Health alone does not validate a publish bundle.
        result = subprocess.run([self.python, str(self.share / 'backend/manage.py'), 'shell', '--verbosity', '0', '-c',
            'import json; from twin.map_sync import published_map_payload; print("BUNDLE="+json.dumps(published_map_payload()))'],
            env=self.backend_env, cwd=self.root, text=True, capture_output=True, check=True)
        row = next(line[7:] for line in result.stdout.splitlines() if line.startswith('BUNDLE='))
        published = json.loads(row)
        revision = int(published.get('map_revision') or 0)
        selected = {}
        try:
            _manifest, selected = verify_bundle(Path(published.get('artifact_dir') or ''), revision, self.args.robot_id)
        except (OSError, ValueError, KeyError, TypeError):
            if self.args.allow_dev_world != 'true':
                raise RuntimeError('No valid published canonical bundle. Publish a map before full-stack launch; development fallback requires allow_dev_world:=true')
            revision = 0
            selected['map'] = str(bringup / 'swerve_navigation/maps/warehouse.yaml')
        request = self.root / 'map-sync-request.json'
        mode_request = self.root / 'mode-switch-request.json'
        mode_status = self.root / 'mode-switch-status.json'
        for path in (request, mode_request):
            path.unlink(missing_ok=True)
        mode_status.write_text(json.dumps(dict(robot_id=self.args.robot_id, mode=self.args.mode, status='STARTING')))
        command = ['ros2', 'launch', 'swerve_bringup', 'system.launch.py', f'mode:={self.args.mode}',
                   f'use_sim:={self.args.use_sim}', f'use_sim_time:={self.args.use_sim}',
                   'defer_nav2_start:=true', f'robot_id:={self.args.robot_id}',
                   f'gui:={self.args.gui}', f'start_rviz:={self.args.start_rviz}',
                   f"artifact_root:={self.env['WARETWIN_ARTIFACT_ROOT']}", f'map_sync_request_file:={request}',
                   f"bridge_ws_url:={self.env['ROS_WS_URL']}", f'allow_dev_world:={self.args.allow_dev_world}']
        if self.args.namespace:
            command.append(f'namespace:={self.args.namespace}')
        if self.args.real_sensor_launch:
            command.append(f'real_sensor_launch:={self.args.real_sensor_launch}')
        for key, arg in (('world', 'world'), ('map', 'map_file'), ('datamatrix', 'datamatrix_map_file'), ('graph', 'tag_graph_file')):
            if selected.get(key):
                command.append(f'{arg}:={selected[key]}')
        self.env['WARETWIN_USE_SIM'] = self.args.use_sim
        self.env['WARETWIN_ROS_LOG_PATH'] = str(self.root / 'logs/ros.log')
        stack_env = self.root / 'stack.env'
        stack_env.write_text(f'MODE={self.args.mode}\nROS_DOMAIN_ID={self.env["ROS_DOMAIN_ID"]}\n')
        self.spawn('ros', [sys.executable, str(bringup / 'scripts/ros_stack_supervisor.py'),
            '--request-file', str(request), '--initial-revision', str(revision), '--robot-id', self.args.robot_id,
            '--mode-request-file', str(mode_request), '--mode-status-file', str(mode_status),
            '--initial-mode', self.args.mode, '--stack-env-file', str(stack_env),
            '--readiness-root', str(bringup), '--backend-url', self.backend_url,
            '--map-file', selected['map'], '--initial-readiness', '--mode-timeout', str(self.args.ros_timeout), '--', *command])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--share', required=True)
    for key in ('backend-host', 'backend-port', 'frontend-host', 'frontend-port', 'ros-domain-id', 'config-file', 'runtime-dir', 'backend-python'):
        parser.add_argument('--' + key, default='')
    parser.add_argument('--setup', action='store_true')
    parser.add_argument('--full-stack', action='store_true')
    parser.add_argument('--mode', choices=('unified', 'mapping', 'navigation'), default='unified')
    for key, default in (('use-sim', 'true'), ('allow-dev-world', 'false'), ('gui', 'false'), ('start-rviz', 'false')):
        parser.add_argument('--' + key, choices=('true', 'false'), default=default)
    parser.add_argument('--robot-id', default='R01')
    parser.add_argument('--namespace', default='')
    parser.add_argument('--real-sensor-launch', default='')
    parser.add_argument('--startup-timeout', type=float, default=120)
    parser.add_argument('--ros-timeout', type=float, default=600)
    args = parser.parse_args()
    try:
        runtime = Runtime(args)
        if args.startup_timeout <= 0 or args.ros_timeout <= 0:
            raise ValueError('Timeouts must be positive')
        if args.setup:
            if sys.version_info[:2] != (3, 10):
                raise ValueError('Use /usr/bin/python3 (Python 3.10 on Ubuntu 22.04) for setup')
            venv = runtime.root / 'venv'
            if not (venv / 'bin/python').exists():
                subprocess.run([sys.executable, '-m', 'venv', str(venv)], check=True)
            clean_env = os.environ.copy()
            clean_env.pop('PYTHONPATH', None)
            subprocess.run([str(venv / 'bin/python'), '-m', 'pip', 'install', '-r', str(runtime.share / 'runtime/requirements.lock')], env=clean_env, check=True)
            subprocess.run([str(venv / 'bin/python'), '-m', 'pip', 'check'], env=clean_env, check=True)
            return 0
        return runtime.run()
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError, StopIteration) as exc:
        print(f'WareTwin startup failed: {exc}', file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
