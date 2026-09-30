#!/usr/bin/env python3
"""Restart the owned ROS launch when a verified map revision is published."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from typing import Sequence


def verify_bundle(root: Path, revision: int, robot_id: str) -> tuple[dict, dict]:
    root = root.resolve(strict=True)
    manifest = json.loads((root / 'manifest.json').read_text(encoding='utf-8'))
    if int(manifest.get('revision', -1)) != revision:
        raise ValueError('revision directory manifest does not match reload request')
    if manifest.get('frame_id') != 'map' or manifest.get('units') != 'm':
        raise ValueError('map bundle must use frame_id=map and units=m')
    artifacts = manifest.get('artifacts') or {}
    required_roles = ('canonical_map', 'gazebo_world', 'gazebo_manifest',
                      'datamatrix_map', 'tag_graph', 'nav2_map')
    missing_roles = [role for role in required_roles if not artifacts.get(role)]
    if missing_roles:
        raise ValueError('map bundle lacks required artifacts: ' + ', '.join(missing_roles))
    hashes = manifest.get('sha256') or {}
    if not hashes:
        raise ValueError('map bundle has no artifact hashes')
    for relative, expected in hashes.items():
        path = (root / relative).resolve(strict=True)
        if not path.is_relative_to(root):
            raise ValueError(f'artifact path escapes revision directory: {relative}')
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != expected:
            raise ValueError(f'artifact hash mismatch: {relative}')
    canonical = json.loads((root / artifacts['canonical_map']).read_text(encoding='utf-8'))
    if canonical.get('frame_id') != 'map' or int(canonical.get('revision', -1)) != revision:
        raise ValueError('canonical map revision/frame does not match manifest')
    origin = canonical.get('origin') or {}
    try:
        expected_bounds = {
            'min_x': float(origin['x']), 'min_y': float(origin['y']),
            'max_x': float(origin['x']) + float(canonical['width']),
            'max_y': float(origin['y']) + float(canonical['height']),
        }
        actual_bounds = manifest['gazebo_bounds']
        if any(abs(float(actual_bounds[key]) - value) > 1e-6 for key, value in expected_bounds.items()):
            raise ValueError('Gazebo bounds do not match canonical map bounds')
    except (KeyError, TypeError, ValueError) as exc:
        if isinstance(exc, ValueError) and str(exc).startswith('Gazebo bounds'):
            raise
        raise ValueError(f'invalid canonical/Gazebo bounds in manifest: {exc}') from exc
    gazebo_manifest_path = (root / artifacts['gazebo_manifest']).resolve(strict=True)
    if not gazebo_manifest_path.is_relative_to(root):
        raise ValueError('Gazebo manifest path escapes revision directory')
    gazebo_manifest = json.loads(gazebo_manifest_path.read_text(encoding='utf-8'))
    world_path = (root / artifacts['gazebo_world']).resolve(strict=True)
    if not world_path.is_relative_to(root):
        raise ValueError('Gazebo world path escapes revision directory')
    world_xml = ET.parse(world_path).getroot()
    state_plugin = world_xml.find(".//plugin[@name='gazebo_ros_state'][@filename='libgazebo_ros_state.so']")
    if state_plugin is None:
        raise ValueError('Gazebo world lacks gazebo_ros_state plugin required for V30E simulation/readiness')
    canonical_floors = {str(item.get('id')): item for item in canonical.get('floors', [])}
    gazebo_floors = {str(item.get('id')): item for item in gazebo_manifest.get('floors', [])}
    if set(canonical_floors) != set(gazebo_floors):
        raise ValueError('Gazebo floor set does not match canonical map')
    for floor_id, floor in canonical_floors.items():
        if floor.get('boundary') != gazebo_floors[floor_id].get('boundary'):
            raise ValueError(f'Gazebo floor {floor_id} bounds do not match canonical map')
        if floor.get('holes', []) != gazebo_floors[floor_id].get('holes', []):
            raise ValueError(f'Gazebo floor {floor_id} holes do not match canonical map')
    robot = next((row for row in manifest.get('robots', []) if str(row.get('id')) == robot_id), None)
    if robot is None:
        raise ValueError(f'robot {robot_id} has no spawn pose in revision {revision}')
    floor_id = str(robot.get('floor_id', ''))
    nav2_relative = (manifest.get('nav2_maps') or {}).get(floor_id) or artifacts['nav2_map']
    nav2_path = (root / nav2_relative).resolve(strict=True)
    if not nav2_path.is_relative_to(root):
        raise ValueError('selected Nav2 map path escapes revision directory')
    floor = canonical_floors.get(floor_id)
    nav_bounds = (manifest.get('nav2_bounds') or {}).get(floor_id)
    if floor is None or nav_bounds is None:
        raise ValueError(f'Nav2 bounds are missing for robot floor {floor_id}')
    boundary = floor.get('boundary') or []
    try:
        xs = [float(point['x'] if isinstance(point, dict) else point[0]) for point in boundary]
        ys = [float(point['y'] if isinstance(point, dict) else point[1]) for point in boundary]
        resolution = float(nav_bounds['resolution'])
        nav_origin = nav_bounds['origin']
        expected_origin = (min(xs), min(ys))
        actual_origin = (float(nav_origin[0]), float(nav_origin[1]))
        width_m = float(nav_bounds['width']) * resolution
        height_m = float(nav_bounds['height']) * resolution
        if resolution <= 0 or any(abs(a - b) > 1e-6 for a, b in zip(expected_origin, actual_origin)):
            raise ValueError('Nav2 map origin/resolution do not match canonical floor')
        if width_m + 1e-6 < max(xs) - min(xs) or height_m + 1e-6 < max(ys) - min(ys):
            raise ValueError('Nav2 map bounds are smaller than canonical floor bounds')
        if width_m - (max(xs) - min(xs)) >= resolution + 1e-6 or height_m - (max(ys) - min(ys)) >= resolution + 1e-6:
            raise ValueError('Nav2 map bounds exceed canonical floor by more than one cell')
    except (KeyError, TypeError, ValueError, IndexError) as exc:
        raise ValueError(f'invalid Nav2 bounds for floor {floor_id}: {exc}') from exc
    return manifest, {
        'world': str(world_path),
        'map': str(nav2_path),
        'datamatrix': str((root / artifacts['datamatrix_map']).resolve(strict=True)),
        'graph': str((root / artifacts['tag_graph']).resolve(strict=True)),
        'spawn': robot.get('pose'),
    }


def command_for_revision(base: Sequence[str], root: Path, revision: int,
                         robot_id: str) -> list[str]:
    _manifest, selected = verify_bundle(root, revision, robot_id)
    replaced = ('world:=', 'map_file:=', 'datamatrix_map_file:=', 'tag_graph_file:=')
    command = [arg for arg in base if not arg.startswith(replaced)]
    command.extend((
        f"world:={selected['world']}", f"map_file:={selected['map']}",
        f"datamatrix_map_file:={selected['datamatrix']}",
        f"tag_graph_file:={selected['graph']}", 'allow_dev_world:=false',
    ))
    return command


def command_for_mode(base: Sequence[str], mode: str) -> list[str]:
    """Return the same launch command with one explicit runtime mode."""
    mode = str(mode).strip().lower()
    if mode not in ('mapping', 'navigation'):
        raise ValueError('mode must be mapping or navigation')
    command = [arg for arg in base if not arg.startswith('mode:=')
               and not arg.startswith('defer_nav2_start:=')]
    command.append(f'mode:={mode}')
    if mode == 'navigation':
        command.append('defer_nav2_start:=true')
    return command


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(payload, sort_keys=True), encoding='utf-8')
    os.replace(temporary, path)


def _write_mode_status(path: Path | None, request: dict, status: str,
                       message: str | None = None) -> None:
    if path is None:
        return
    _write_json(path, {
        'request_id': request.get('request_id'),
        'robot_id': request.get('robot_id'),
        'mode': request.get('mode'),
        'status': status,
        'message': message,
        'updated_at': datetime.now(timezone.utc).isoformat(),
    })


def _set_stack_mode(path: Path | None, mode: str) -> None:
    if path is None or not path.is_file():
        return
    rows = path.read_text(encoding='utf-8').splitlines()
    updated = False
    for index, row in enumerate(rows):
        if row.startswith('MODE='):
            rows[index] = f'MODE={mode}'
            updated = True
            break
    if updated:
        temporary = path.with_suffix(path.suffix + '.tmp')
        temporary.write_text('\n'.join(rows) + '\n', encoding='utf-8')
        os.replace(temporary, path)


def _run_readiness(root: Path, mode: str, robot_id: str, backend_url: str | None,
                   map_file: str | None, log_path: str | None,
                   deadline: float, child: subprocess.Popen,
                   env: dict[str, str] | None = None) -> tuple[bool, str]:
    script = root / 'scripts' / 'navigation_readiness.py'
    while time.monotonic() < deadline:
        if child.poll() is not None:
            return False, f'{mode} ROS launch exited with status {child.returncode}'
        remaining = deadline - time.monotonic()
        args = [sys.executable, str(script), '--mode', mode, '--model', 'swerve_base',
                '--robot-id', robot_id, '--timeout', str(min(30.0, max(2.0, remaining)))]
        if backend_url:
            args.extend(('--backend-url', backend_url))
        if log_path:
            args.extend(('--log-path', log_path))
        if mode == 'navigation' and map_file:
            args.extend(('--map-file', map_file))
        try:
            result = subprocess.run(args, cwd=root, env=env, capture_output=True, text=True,
                                    timeout=min(40.0, max(5.0, remaining + 5.0)))
            output = (result.stdout or '') + (result.stderr or '')
            if output:
                print(output.rstrip(), flush=True)
            if result.returncode == 0:
                return True, 'runtime readiness gate passed'
            reason = output.strip().splitlines()[-1] if output.strip() else 'readiness probe failed'
        except subprocess.TimeoutExpired:
            reason = 'readiness probe timed out'
        if child.poll() is not None:
            return False, f'{mode} ROS launch exited with status {child.returncode}'
        time.sleep(min(2.0, max(0.0, deadline - time.monotonic())))
    return False, reason


def _stop_process(process: subprocess.Popen, timeout: float = 25.0) -> None:
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def run_supervisor(request_file: Path, initial_revision: int | None,
                   robot_id: str, base_command: Sequence[str], *,
                   mode_request_file: Path | None = None,
                   mode_status_file: Path | None = None,
                   initial_mode: str | None = None,
                   stack_env_file: Path | None = None,
                   readiness_root: Path | None = None,
                   backend_url: str | None = None,
                   map_file: str | None = None,
                   mode_timeout: float = 600.0) -> int:
    active_revision = initial_revision
    command = list(base_command)
    active_mode = str(initial_mode or next(
        (arg.split(':=', 1)[1] for arg in command if arg.startswith('mode:=')), 'navigation')).lower()
    readiness_root = (readiness_root or Path(__file__).resolve().parent.parent).resolve()
    child: subprocess.Popen | None = None
    stopping = False

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True
        if child is not None:
            _stop_process(child, timeout=10)

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    while not stopping:
        if child is None:
            print(f'[MAP-SUPERVISOR] launching revision={active_revision or "development"}', flush=True)
            child = subprocess.Popen(command)
        code = child.poll()
        if code is not None:
            return code
        if mode_request_file is not None:
            mode_request = {}
            try:
                mode_request = json.loads(mode_request_file.read_text(encoding='utf-8'))
                if not isinstance(mode_request, dict):
                    raise ValueError('mode transition request must be a JSON object')
                target_mode = str(mode_request.get('mode') or '').lower()
                if str(mode_request.get('robot_id') or '') != robot_id:
                    raise ValueError('mode transition request robot_id does not match the active stack')
                if target_mode not in ('mapping', 'navigation'):
                    raise ValueError('mode transition target must be mapping or navigation')
                mode_request_file.unlink(missing_ok=True)
                if target_mode == active_mode:
                    _write_mode_status(mode_status_file, mode_request, 'READY',
                                       f'{target_mode} mode is already active')
                else:
                    previous_mode, previous_command = active_mode, command
                    target_command = command_for_mode(command, target_mode)
                    print(f'[MODE-SUPERVISOR] transition {previous_mode}->{target_mode}; restarting ROS/Gazebo and rechecking readiness', flush=True)
                    _write_mode_status(mode_status_file, mode_request, 'RESTARTING',
                                       'ROS/Gazebo restart in progress; simulated robot will respawn at its configured start pose')
                    _stop_process(child)
                    transition_env = os.environ.copy()
                    transition_env['WARETWIN_STACK_START_MONOTONIC_S'] = str(time.monotonic())
                    transition_env.pop('WARETWIN_GAZEBO_STARTED_MONOTONIC_S', None)
                    child = subprocess.Popen(target_command, env=transition_env)
                    ready, reason = _run_readiness(
                        readiness_root, target_mode, robot_id, backend_url,
                        map_file, None, time.monotonic() + mode_timeout, child,
                        env=transition_env)
                    if ready:
                        active_mode, command = target_mode, target_command
                        _set_stack_mode(stack_env_file, active_mode)
                        _write_mode_status(mode_status_file, mode_request, 'READY',
                                           'requested runtime mode passed the existing readiness gate')
                        print(f'[MODE-SUPERVISOR] mode={active_mode} readiness=PASS', flush=True)
                    else:
                        print(f'[MODE-SUPERVISOR] mode={target_mode} readiness=FAIL reason={reason}; restoring {previous_mode}', file=sys.stderr, flush=True)
                        _write_mode_status(mode_status_file, mode_request, 'ROLLING_BACK', reason)
                        _stop_process(child)
                        rollback_env = os.environ.copy()
                        rollback_env['WARETWIN_STACK_START_MONOTONIC_S'] = str(time.monotonic())
                        rollback_env.pop('WARETWIN_GAZEBO_STARTED_MONOTONIC_S', None)
                        child = subprocess.Popen(previous_command, env=rollback_env)
                        restored, restore_reason = _run_readiness(
                            readiness_root, previous_mode, robot_id, backend_url,
                            map_file, None, time.monotonic() + mode_timeout, child,
                            env=rollback_env)
                        if restored:
                            active_mode, command = previous_mode, previous_command
                            _set_stack_mode(stack_env_file, active_mode)
                            _write_mode_status(mode_status_file, mode_request, 'ROLLED_BACK',
                                               f'{target_mode} failed readiness ({reason}); {previous_mode} restored')
                        else:
                            _stop_process(child)
                            child = None
                            _write_mode_status(mode_status_file, mode_request, 'ERROR',
                                               f'{target_mode} failed readiness ({reason}); restoring {previous_mode} also failed ({restore_reason})')
                            return 1
                    continue
            except FileNotFoundError:
                pass
            except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
                print(f'[MODE-SUPERVISOR] transition request rejected: {exc}', file=sys.stderr, flush=True)
                try:
                    mode_request_file.unlink(missing_ok=True)
                    failed_request = mode_request if isinstance(mode_request, dict) else {}
                    _write_mode_status(mode_status_file, failed_request, 'ERROR', str(exc))
                except Exception:
                    pass
        try:
            request = json.loads(request_file.read_text(encoding='utf-8'))
            requested_revision = int(request.get('revision'))
            root = Path(str(request.get('artifact_dir', '')))
            if active_revision is not None and requested_revision <= active_revision:
                request_file.unlink(missing_ok=True)
            else:
                new_command = command_for_revision(command, root, requested_revision, robot_id)
                print(f'[MAP-SUPERVISOR] verified revision={requested_revision}; stopping navigation and restarting Gazebo/Nav2/tag consumers', flush=True)
                _stop_process(child)
                child = None
                active_revision = requested_revision
                command = new_command
                request_file.unlink(missing_ok=True)
                continue
        except FileNotFoundError:
            pass
        except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
            print(f'[MAP-SUPERVISOR] reload request retained; validation failed: {exc}', file=sys.stderr, flush=True)
        time.sleep(0.4)
    if child is not None:
        _stop_process(child, timeout=10)
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request-file', required=True, type=Path)
    parser.add_argument('--initial-revision', type=int)
    parser.add_argument('--robot-id', required=True)
    parser.add_argument('--mode-request-file', type=Path)
    parser.add_argument('--mode-status-file', type=Path)
    parser.add_argument('--initial-mode', choices=('mapping', 'navigation'))
    parser.add_argument('--stack-env-file', type=Path)
    parser.add_argument('--readiness-root', type=Path)
    parser.add_argument('--backend-url')
    parser.add_argument('--map-file')
    parser.add_argument('--mode-timeout', type=float, default=600.0)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    if args.mode_timeout <= 0:
        parser.error('--mode-timeout must be positive')
    command = args.command[1:] if args.command and args.command[0] == '--' else args.command
    if not command:
        parser.error('a ROS launch command is required after --')
    return run_supervisor(
        args.request_file, args.initial_revision, args.robot_id, command,
        mode_request_file=args.mode_request_file,
        mode_status_file=args.mode_status_file,
        initial_mode=args.initial_mode,
        stack_env_file=args.stack_env_file,
        readiness_root=args.readiness_root,
        backend_url=args.backend_url,
        map_file=args.map_file,
        mode_timeout=args.mode_timeout,
    )


if __name__ == '__main__':
    raise SystemExit(main())
