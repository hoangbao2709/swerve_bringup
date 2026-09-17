#!/usr/bin/env python3
"""Deterministic fresh-session runner for bringup and acceptance evaluators.

Every launch gets its own process group and ROS domain.  Only that process
group is signalled during teardown; unrelated ROS/Gazebo processes are never
selected by name and killed.
"""
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


def descendants_dead(pgid):
    try:
        output = subprocess.check_output(['ps', '-eo', 'pgid='], text=True)
    except (OSError, subprocess.CalledProcessError):
        return False
    return str(pgid) not in {line.strip() for line in output.splitlines()}


def stop_group(proc, grace):
    if proc.poll() is not None:
        return descendants_dead(proc.pid)
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        return True
    deadline = time.monotonic() + grace
    while time.monotonic() < deadline and proc.poll() is None:
        time.sleep(0.2)
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGTERM)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and proc.poll() is None:
            time.sleep(0.2)
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait(timeout=5)
    return descendants_dead(proc.pid)


def graph_quiet(env, timeout=8.0):
    """Wait for this isolated DDS domain to stop advertising project nodes."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            nodes = subprocess.check_output(
                ['ros2', 'node', 'list'], env=env, text=True,
                stderr=subprocess.DEVNULL, timeout=3).splitlines()
        except (OSError, subprocess.SubprocessError):
            nodes = []
        owned = ('swerve', 'gazebo', 'map_server', 'controller_server',
                 'planner_server', 'behavior_server', 'bt_navigator',
                 'waypoint_follower', 'ekf_', 'v30e', 'lifecycle_manager')
        if not any(any(token in node for token in owned) for node in nodes):
            return True
        time.sleep(0.4)
    return False


def run_case(args, case_name, domain, outdir):
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(domain)
    env['ACCEPTANCE_CASE'] = case_name
    env['ACCEPTANCE_CASE_INDEX'] = case_name
    env['ACCEPTANCE_CASE_DIR'] = outdir
    # Make the runner self-contained when called from an unsourced shell.
    prefixes = [os.path.join(ROOT, 'install', 'swerve_bringup'),
                os.path.join(ROOT, 'install', 'swerve_bridge')]
    existing = env.get('AMENT_PREFIX_PATH', '').split(':')
    env['AMENT_PREFIX_PATH'] = ':'.join(prefixes + [p for p in existing if p])
    env['CMAKE_PREFIX_PATH'] = ':'.join(prefixes + [p for p in env.get('CMAKE_PREFIX_PATH', '').split(':') if p])
    launch_log = open(os.path.join(outdir, 'launch.log'), 'w', encoding='utf-8')
    proc = subprocess.Popen(
        ['ros2', 'launch', 'swerve_bringup', 'system.launch.py',
         'use_sim:=true', 'mode:=navigation', 'gui:=false'],
        cwd=ROOT, env=env, stdout=launch_log, stderr=subprocess.STDOUT,
        start_new_session=True)
    readiness_json = os.path.join(outdir, 'readiness.json')
    readiness = subprocess.run(
        [sys.executable, os.path.join(ROOT, 'scripts', 'navigation_readiness.py'),
         '--timeout', str(args.readiness_timeout), '--json', readiness_json],
        cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, timeout=args.readiness_timeout + 15)
    with open(os.path.join(outdir, 'readiness.log'), 'w', encoding='utf-8') as stream:
        stream.write(readiness.stdout)
    nav_ready = readiness.returncode == 0
    evaluator_rc = None
    evaluator_log = ''
    evaluator_passed = True
    if nav_ready and args.evaluator:
        evaluator_command = [value.replace('{case_dir}', outdir) for value in args.evaluator]
        evaluator = subprocess.run(evaluator_command, cwd=ROOT, env=env,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   text=True, timeout=args.evaluator_timeout)
        evaluator_rc, evaluator_log = evaluator.returncode, evaluator.stdout
        with open(os.path.join(outdir, 'evaluator.log'), 'w', encoding='utf-8') as stream:
            stream.write(evaluator_log)
        summary_path = os.path.join(outdir, 'accuracy_summary.json')
        if os.path.exists(summary_path):
            with open(summary_path, encoding='utf-8') as stream:
                evaluator_passed = bool(json.load(stream).get('passed', False))
        result_path = os.path.join(outdir, 'raw_motion_result.csv')
        if os.path.exists(result_path):
            with open(result_path, encoding='utf-8') as stream:
                evaluator_passed = evaluator_passed and next(csv.DictReader(stream), {}).get('direction_correct') == 'True'
    shutdown_clean = stop_group(proc, args.shutdown_grace)
    quiet = graph_quiet(env)
    launch_log.close()
    return {
        'run': case_name, 'startup_wall_time': json.load(open(readiness_json)).get('startup_wall_time') if os.path.exists(readiness_json) else None,
        'startup_sim_time': json.load(open(readiness_json)).get('startup_sim_time') if os.path.exists(readiness_json) else None,
        'controller_ready': bool(json.load(open(readiness_json)).get('stages', {}).get('CONTROLLERS_READY')) if os.path.exists(readiness_json) else False,
        'local_tf_ready': bool(json.load(open(readiness_json)).get('stages', {}).get('LOCAL_TF_READY')) if os.path.exists(readiness_json) else False,
        'global_tf_ready': bool(json.load(open(readiness_json)).get('stages', {}).get('GLOBAL_TF_READY')) if os.path.exists(readiness_json) else False,
        'nav2_ready': bool(json.load(open(readiness_json)).get('stages', {}).get('NAV2_LIFECYCLE_READY')) if os.path.exists(readiness_json) else False,
        'action_ready': bool(json.load(open(readiness_json)).get('stages', {}).get('ACTION_SERVER_READY')) if os.path.exists(readiness_json) else False,
        'shutdown_clean': bool(shutdown_clean and quiet),
        'no_old_gzserver': bool(shutdown_clean),
        'no_old_test_launch': bool(shutdown_clean),
        'no_old_evaluator': evaluator_rc is not None or not args.evaluator,
        'evaluator_rc': evaluator_rc,
        'status': 'PASS' if nav_ready and shutdown_clean and quiet and evaluator_rc in (None, 0) and evaluator_passed else 'FAIL',
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--runs', type=int, default=1)
    parser.add_argument('--domain', type=int, default=80)
    parser.add_argument('--readiness-timeout', type=float, default=120.0)
    parser.add_argument('--shutdown-grace', type=float, default=15.0)
    parser.add_argument('--evaluator-timeout', type=float, default=900.0)
    parser.add_argument('--evaluator', nargs='+', help='command run only after NAV_READY')
    parser.add_argument('--cases', nargs='+', help='case names exported as ACCEPTANCE_CASE')
    parser.add_argument('--outdir', help='timestamped output root')
    args = parser.parse_args()
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    root = args.outdir or os.path.join(ROOT, 'artifacts', f'acceptance_{stamp}')
    os.makedirs(os.path.join(root, 'cases'), exist_ok=True)
    rows = []
    case_names = args.cases or [f'bringup_{index + 1:02d}' for index in range(args.runs)]
    for index, case in enumerate(case_names[:args.runs]):
        case_dir = os.path.join(root, 'cases', case)
        os.makedirs(case_dir, exist_ok=False)
        row = run_case(args, case, args.domain + index, case_dir)
        rows.append(row)
        print(json.dumps(row, sort_keys=True), flush=True)
        if row['status'] != 'PASS':
            print('STOP: case failed; no next case is launched', flush=True)
            break
    with open(os.path.join(root, 'summary.csv'), 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys() if rows else ['status'])
        writer.writeheader(); writer.writerows(rows)
    with open(os.path.join(root, 'summary.json'), 'w', encoding='utf-8') as stream:
        json.dump({'runs': rows, 'passed': len(rows) == args.runs and all(r['status'] == 'PASS' for r in rows)}, stream, indent=2)
    if not args.evaluator and args.runs >= 1:
        with open(os.path.join(ROOT, 'artifacts', 'bringup_stability.json'), 'w', encoding='utf-8') as stream:
            json.dump({'runs': rows, 'passed': len(rows) == args.runs and all(r['status'] == 'PASS' for r in rows)}, stream, indent=2)
    return 0 if len(rows) == args.runs and all(r['status'] == 'PASS' for r in rows) else 1


if __name__ == '__main__':
    sys.exit(main())
