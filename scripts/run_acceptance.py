#!/usr/bin/env python3
"""Run acceptance against the production ``start_stack.sh`` architecture.

This runner deliberately does not launch ``system.launch.py`` itself.  The
production stack owns map selection, the ROS environment, deferred Nav2
lifecycle activation, backend/WebSocket startup, and shutdown.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read_simple_env(path: str, key: str) -> str | None:
    """Read a single KEY=value from the non-secret runtime snapshot."""
    try:
        with open(path, encoding='utf-8') as stream:
            for raw in stream:
                line = raw.strip()
                if not line or line.startswith('#') or '=' not in line:
                    continue
                name, value = line.split('=', 1)
                if name.strip() == key:
                    return value.strip().strip('"').strip("'")
    except OSError:
        pass
    return None


def stack_is_active() -> bool:
    """Refuse to take ownership of a stack that was already running."""
    runtime_dir = os.path.join(ROOT, '.runtime')
    for component in ('backend', 'frontend', 'ros'):
        try:
            with open(os.path.join(runtime_dir, f'{component}.pid'), encoding='utf-8') as stream:
                pid = int(stream.readline().strip())
            os.kill(pid, 0)
            cmd = open(f'/proc/{pid}/cmdline', 'rb').read().replace(b'\0', b' ').decode(errors='replace')
            cwd = os.path.realpath(os.readlink(f'/proc/{pid}/cwd'))
        except (OSError, ValueError):
            continue
        if component == 'backend' and cwd == os.path.join(ROOT, 'waretwin/backend'):
            if 'manage.py runserver' in cmd or 'backend/run.sh' in cmd:
                return True
        if component == 'frontend' and cwd == os.path.join(ROOT, 'waretwin/frontend'):
            if 'vite' in cmd or 'npm' in cmd:
                return True
        if component == 'ros' and (cwd == ROOT or f'{ROOT}/install/' in cmd):
            if any(marker in cmd for marker in (
                'ros_stack_supervisor.py', 'system.launch.py', 'gzserver',
            )):
                return True
    return False


def load_ros_environment() -> dict[str, str]:
    """Load the canonical source file without duplicating ROS env rules here."""
    shell = (
        'set -a; '
        'source "$1/waretwin/backend/.env"; '
        'source "$1/scripts/ros_env.sh" >/dev/null; '
        'env -0'
    )
    loaded = subprocess.run(
        ['bash', '-c', shell, '_', ROOT],
        cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
    )
    if loaded.returncode != 0:
        reason = loaded.stderr.decode(errors='replace').strip()
        raise RuntimeError(f'scripts/ros_env.sh failed: {reason or loaded.returncode}')
    values = {}
    for item in loaded.stdout.decode(errors='surrogateescape').split('\0'):
        if '=' in item:
            key, value = item.split('=', 1)
            values[key] = value
    return values


def process_exists(executable: str) -> bool:
    result = subprocess.run(['pgrep', '-x', executable],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return result.returncode == 0


def print_stage(name: str, passed: bool, reason: str | None = None) -> None:
    if passed:
        print(f'{name}=PASS', flush=True)
    else:
        print(f'{name}=FAIL reason={reason or "not_observed"}', flush=True)


def run_case(args, case_name: str, case_dir: str) -> dict:
    os.makedirs(case_dir, exist_ok=False)
    result = {
        'run': case_name,
        'mode': args.mode,
        'startup_status': 'UNVERIFIED',
        'ros_graph_cli_status': 'UNVERIFIED',
        'readiness_status': 'UNVERIFIED',
        'headless_status': 'UNVERIFIED',
        'direct_ros_status': 'UNVERIFIED',
        'direct_nav_status': 'UNVERIFIED',
        'web_manual_status': 'UNVERIFIED',
        'web_navigation_status': 'UNVERIFIED',
        'shutdown_status': 'UNVERIFIED',
    }
    if stack_is_active():
        reason = 'a stack process is already owned by this worktree; stop it before acceptance'
        print_stage('ACCEPTANCE_STARTUP', False, reason)
        result.update(status='FAIL', reason=reason)
        return result

    env = os.environ.copy()
    env['WARETWIN_ROS_READY_TIMEOUT_S'] = str(int(args.readiness_timeout))
    env['WARETWIN_READINESS_PROBE_TIMEOUT_S'] = str(
        max(10, min(120, int(args.readiness_timeout)))
    )
    env['WARETWIN_ROBOT_ID'] = 'R01'
    start_log_path = os.path.join(case_dir, 'start_stack.log')
    start_ok = False
    stack_attempted = False
    ros_env = None
    backend_url = None
    try:
        stack_attempted = True
        command = [os.path.join(ROOT, 'scripts', 'start_stack.sh'), args.mode,
                   '--headless', '--robot-id', 'R01']
        with open(start_log_path, 'w', encoding='utf-8') as log:
            try:
                startup = subprocess.run(
                    command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                    text=True, timeout=args.readiness_timeout + 180,
                )
                start_ok = startup.returncode == 0
            except subprocess.TimeoutExpired:
                start_ok = False
                with open(start_log_path, 'a', encoding='utf-8') as log:
                    log.write('\nACCEPTANCE_STARTUP=FAIL reason=start_stack_timeout\n')
        print_stage('ACCEPTANCE_STARTUP', start_ok,
                    None if start_ok else f'see:{start_log_path}')
        result['startup_status'] = 'PASS' if start_ok else 'FAIL'
        result['startup_log'] = start_log_path
        if not start_ok:
            result['reason'] = 'production start_stack.sh did not report ready'
            result['status'] = 'FAIL'
            return result

        ros_env = load_ros_environment()
        runtime_path = os.path.join(ROOT, '.runtime', 'stack.env')
        backend_url = (read_simple_env(runtime_path, 'BACKEND_URL')
                       or f"http://127.0.0.1:{read_simple_env(runtime_path, 'BACKEND_PORT') or '8000'}")
        result['backend_url'] = backend_url
        result['ros_environment'] = {
            key: (ros_env.get(key) or '<unset>') for key in (
                'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION',
                'ROS_LOCALHOST_ONLY', 'FASTDDS_BUILTIN_TRANSPORTS',
            )
        }

        graph_log_path = os.path.join(case_dir, 'ros_graph.log')
        try:
            node_list = subprocess.run(
                ['ros2', 'node', 'list', '--no-daemon', '--spin-time', '2'],
                cwd=ROOT, env=ros_env, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, timeout=30, check=False,
            )
            topic_list = subprocess.run(
                ['ros2', 'topic', 'list', '--no-daemon', '--spin-time', '2'],
                cwd=ROOT, env=ros_env, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, timeout=30, check=False,
            )
            graph_text = (
                '=== ros2 node list ===\n' + node_list.stdout
                + '\n=== ros2 topic list ===\n' + topic_list.stdout
            )
            graph_ok = (
                node_list.returncode == 0 and topic_list.returncode == 0
                and 'swerve_bridge' in node_list.stdout
                and '/clock' in topic_list.stdout and '/lidar/points' in topic_list.stdout
                and '/lidar/points_filtered' in topic_list.stdout and '/scan' in topic_list.stdout
            )
            graph_reason = None if graph_ok else 'node_or_topic_graph_incomplete'
        except (OSError, subprocess.SubprocessError) as exc:
            graph_text = f'{type(exc).__name__}: {exc}\n'
            graph_ok = False
            graph_reason = f'{type(exc).__name__}:{exc}'
        with open(graph_log_path, 'w', encoding='utf-8') as stream:
            stream.write(graph_text)
        result['ros_graph_cli_status'] = 'PASS' if graph_ok else 'FAIL'
        result['ros_graph_cli_log'] = graph_log_path
        print_stage('ROS2_NODE_AND_TOPIC_LIST', graph_ok,
                    None if graph_ok else f'{graph_reason}:see:{graph_log_path}')

        readiness_path = os.path.join(case_dir, 'readiness.json')
        readiness_log_path = os.path.join(case_dir, 'readiness.log')
        readiness_cmd = [
            'python3', os.path.join(ROOT, 'scripts', 'navigation_readiness.py'),
            '--mode', args.mode, '--model', 'swerve_base', '--robot-id', 'R01',
            '--backend-url', backend_url, '--log-path', os.path.join(ROOT, 'logs', 'ros.log'),
            '--timeout', str(args.readiness_timeout), '--json', readiness_path,
        ]
        map_file = read_simple_env(runtime_path, 'MAP_FILE')
        if args.mode in ('navigation', 'unified') and map_file:
            readiness_cmd.extend(['--map-file', map_file])
        readiness = subprocess.run(
            readiness_cmd, cwd=ROOT, env=ros_env, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True,
            timeout=args.readiness_timeout + 15, check=False,
        )
        with open(readiness_log_path, 'w', encoding='utf-8') as stream:
            stream.write(readiness.stdout)
        ready_ok = readiness.returncode == 0
        print_stage('POST_START_READINESS', ready_ok,
                    None if ready_ok else f'see:{readiness_log_path}')
        result['readiness_status'] = 'PASS' if ready_ok else 'FAIL'
        result['readiness_log'] = readiness_log_path
        if os.path.exists(readiness_path):
            with open(readiness_path, encoding='utf-8') as stream:
                result['readiness'] = json.load(stream)

        gzserver = process_exists('gzserver')
        no_gzclient = not process_exists('gzclient')
        no_rviz = not process_exists('rviz2')
        headless_ok = gzserver and no_gzclient and no_rviz
        result['headless'] = {
            'gzserver': gzserver, 'gzclient_absent': no_gzclient,
            'rviz2_absent': no_rviz,
        }
        result['headless_status'] = 'PASS' if headless_ok else 'FAIL'
        print_stage('HEADLESS_GZSERVER', gzserver, 'gzserver_not_running')
        print_stage('HEADLESS_GZCLIENT', no_gzclient, 'gzclient_running')
        print_stage('HEADLESS_RVIZ', no_rviz, 'rviz2_running')

        e2e_ok = False
        if ready_ok and args.mode == 'navigation':
            e2e_json = os.path.join(case_dir, 'end_to_end.json')
            e2e_log_path = os.path.join(case_dir, 'end_to_end.log')
            e2e_cmd = [
                'python3', os.path.join(ROOT, 'scripts', 'end_to_end_acceptance.py'),
                '--backend-url', backend_url, '--robot-id', 'R01',
                '--motion-timeout', str(args.motion_timeout),
                '--navigation-timeout', str(args.navigation_timeout),
                '--json', e2e_json,
            ]
            e2e = subprocess.run(
                e2e_cmd, cwd=ROOT, env=ros_env, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True,
                timeout=args.navigation_timeout * 2 + args.motion_timeout * 5 + 90,
                check=False,
            )
            with open(e2e_log_path, 'w', encoding='utf-8') as stream:
                stream.write(e2e.stdout)
            e2e_ok = e2e.returncode == 0
            if os.path.exists(e2e_json):
                with open(e2e_json, encoding='utf-8') as stream:
                    result['end_to_end'] = json.load(stream)
            if result.get('end_to_end'):
                stages = result['end_to_end'].get('stages', {})
                result['direct_ros_status'] = (
                    'PASS' if all(stages.get(f'DIRECT_ROS_{item}') is True
                                  for item in ('FORWARD', 'STRAFE', 'ROTATE')) else 'FAIL'
                )
                result['direct_nav_status'] = (
                    'PASS' if all(stages.get(f'DIRECT_NAV_{item}') is True for item in (
                        'SERVER_READY', 'GOAL_ACCEPTED', 'CMD_VEL',
                        'CONTROLLER_COMMAND', 'MOTION', 'SUCCEEDED',
                    )) else 'FAIL'
                )
                result['web_manual_status'] = (
                    'PASS' if all(stages.get(item) is True for item in (
                        'WEB_MANUAL_CMD_VEL', 'WEB_MANUAL_CONTROLLER_COMMAND',
                        'WEB_MANUAL_COMMAND_ACCEPTED', 'WEB_MANUAL_MOTION',
                        'WEB_MANUAL_STOP_ZERO',
                    )) else 'FAIL'
                )
                result['web_navigation_status'] = (
                    'PASS' if all(stages.get(item) is True for item in (
                        'WEB_NAV_GOAL_ACCEPTED', 'WEB_NAV_CMD_VEL',
                        'WEB_NAV_CONTROLLER_COMMAND', 'WEB_NAV_MOTION',
                        'WEB_NAV_SUCCEEDED',
                    )) else 'FAIL'
                )
            print_stage('END_TO_END_ACCEPTANCE', e2e_ok,
                        None if e2e_ok else f'see:{e2e_log_path}')
            result['end_to_end_log'] = e2e_log_path
        elif args.mode == 'navigation':
            print('END_TO_END_ACCEPTANCE=UNVERIFIED reason=startup_or_readiness_failed', flush=True)

        evaluator_ok = True
        if args.evaluator and ready_ok:
            evaluator_env = dict(ros_env)
            evaluator_env['ACCEPTANCE_CASE'] = case_name
            evaluator_env['ACCEPTANCE_CASE_INDEX'] = case_name
            evaluator_env['ACCEPTANCE_CASE_DIR'] = case_dir
            evaluator_command = [value.replace('{case_dir}', case_dir) for value in args.evaluator]
            try:
                evaluator = subprocess.run(
                    evaluator_command, cwd=ROOT, env=evaluator_env,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, timeout=args.evaluator_timeout, check=False,
                )
                evaluator_ok = evaluator.returncode == 0
                with open(os.path.join(case_dir, 'evaluator.log'), 'w', encoding='utf-8') as stream:
                    stream.write(evaluator.stdout)
            except subprocess.TimeoutExpired:
                evaluator_ok = False
            result['evaluator_status'] = 'PASS' if evaluator_ok else 'FAIL'
            print_stage('EXTERNAL_EVALUATOR', evaluator_ok,
                        None if evaluator_ok else f'evaluator_timeout_or_failure:{args.evaluator_timeout}')

        result['status'] = (
            'PASS' if start_ok and graph_ok and ready_ok and headless_ok
            and (args.mode != 'navigation' or e2e_ok) and evaluator_ok else 'FAIL'
        )
        return result
    except (OSError, RuntimeError, subprocess.SubprocessError, json.JSONDecodeError) as exc:
        result['status'] = 'FAIL'
        result['reason'] = f'{type(exc).__name__}:{exc}'
        print_stage('ACCEPTANCE_RUNNER', False, result['reason'])
        return result
    finally:
        if stack_attempted:
            stop_log_path = os.path.join(case_dir, 'stop_stack.log')
            try:
                stop = subprocess.run(
                    [os.path.join(ROOT, 'scripts', 'stop_stack.sh')], cwd=ROOT,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                    timeout=45, check=False,
                )
                stop_output = stop.stdout
                stopped = stop.returncode == 0
            except (OSError, subprocess.SubprocessError) as exc:
                stop_output = f'{type(exc).__name__}: {exc}\n'
                stopped = False
            result['shutdown_status'] = 'PASS' if stopped else 'FAIL'
            with open(stop_log_path, 'w', encoding='utf-8') as stream:
                stream.write(stop_output)
            print_stage('ACCEPTANCE_STACK_SHUTDOWN', stopped,
                        None if stopped else f'see:{stop_log_path}')
            if result.get('status') == 'PASS' and not stopped:
                result['status'] = 'FAIL'
                result['reason'] = 'production stack did not shut down cleanly'


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--runs', type=int, default=1)
    parser.add_argument('--mode', choices=('mapping', 'navigation'), default='navigation')
    parser.add_argument('--readiness-timeout', type=float, default=1800.0)
    parser.add_argument('--motion-timeout', type=float, default=45.0)
    parser.add_argument('--navigation-timeout', type=float, default=180.0)
    parser.add_argument('--evaluator-timeout', type=float, default=900.0)
    parser.add_argument('--evaluator', nargs='+', help='optional independent evaluator; receives {case_dir}')
    parser.add_argument('--cases', nargs='+', help='acceptance case names')
    parser.add_argument('--outdir', help='artifact output root')
    parser.add_argument('--continue-on-failure', action='store_true')
    args = parser.parse_args()
    if args.runs < 1:
        parser.error('--runs must be positive')
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    root = args.outdir or os.path.join(ROOT, 'artifacts', f'acceptance_{stamp}')
    os.makedirs(os.path.join(root, 'cases'), exist_ok=True)
    names = args.cases or [f'production_{index + 1:02d}' for index in range(args.runs)]
    rows = []
    for index, name in enumerate(names[:args.runs]):
        case_dir = os.path.join(root, 'cases', name)
        row = run_case(args, name, case_dir)
        rows.append(row)
        print(json.dumps(row, sort_keys=True), flush=True)
        if row.get('status') != 'PASS' and not args.continue_on_failure:
            print('STOP: acceptance case failed; production stack was stopped', flush=True)
            break
    passed = len(rows) == args.runs and all(row.get('status') == 'PASS' for row in rows)
    summary = {'runs': rows, 'passed': passed}
    with open(os.path.join(root, 'summary.json'), 'w', encoding='utf-8') as stream:
        json.dump(summary, stream, indent=2)
    with open(os.path.join(root, 'summary.csv'), 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=sorted({key for row in rows for key in row}))
        writer.writeheader()
        for row in rows:
            writer.writerow({key: json.dumps(value, sort_keys=True)
                             if isinstance(value, (dict, list)) else value
                             for key, value in row.items()})
    print(f'ACCEPTANCE_SUMMARY={"PASS" if passed else "FAIL"} path={root}', flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
