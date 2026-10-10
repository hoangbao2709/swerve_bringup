#!/usr/bin/env python3
"""Exercise real SLAM save, supervised Nav2 map load, localization and missions.

This is an opt-in Gazebo acceptance helper. It intentionally uses the running
backend, ROS Bridge, SLAM Toolbox, supervisor, Nav2 map_server and live ROS data;
it must not be treated as a mocked unit test or pointed at a physical robot.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import urllib.error
import urllib.request
import uuid

import rclpy
import websocket

sys.path.insert(0, str(Path(__file__).resolve().parent))
from end_to_end_acceptance import (MotionProbe, pose_pair_skew_sim_s, set_mode,
                                   web_navigation)  # noqa: E402


INITIAL_POSE_POSITION_TOLERANCE_M = 0.05
INITIAL_POSE_YAW_TOLERANCE_RAD = 0.05


def request_json(url: str, method: str = 'GET', body: dict | None = None,
                 timeout: float = 10.0) -> tuple[int, dict]:
    data = json.dumps(body).encode('utf-8') if body is not None else None
    headers = {'Content-Type': 'application/json'} if data is not None else {}
    request = urllib.request.Request(url, data=data, headers=headers, method=method)
    try:
        response = urllib.request.urlopen(request, timeout=timeout)
    except urllib.error.HTTPError as exc:
        response = exc
    with response:
        raw = response.read().decode('utf-8', errors='replace')
        try:
            payload = json.loads(raw) if raw else {}
        except json.JSONDecodeError:
            payload = {'raw': raw[:2000]}
        return int(response.status), payload


def operator_websocket(backend_url: str, frontend_origin: str):
    ws_url = backend_url.replace('https://', 'wss://').replace('http://', 'ws://') + '/ws'
    ws = websocket.create_connection(ws_url, origin=frontend_origin,
                                     timeout=5.0, enable_multithread=True)
    ws.settimeout(0.02)
    return ws


def write_report(path: str | None, report: dict) -> None:
    if not path:
        return
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + '.tmp')
    temporary.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    os.replace(temporary, destination)


def finite_pose(value) -> tuple[float, float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        pose = tuple(float(item) for item in value)
    except (TypeError, ValueError, OverflowError):
        return None
    return pose if all(math.isfinite(item) for item in pose) else None


def transition_status(payload: dict) -> str | None:
    """Return transition status without assuming an optional object is present."""
    transition = payload.get('transition')
    return transition.get('status') if isinstance(transition, dict) else None


def position_error(left, right) -> float:
    return math.hypot(float(left[0]) - float(right[0]), float(left[1]) - float(right[1]))


def yaw_error(left, right) -> float:
    return abs(math.atan2(math.sin(float(left[2]) - float(right[2])),
                          math.cos(float(left[2]) - float(right[2]))))


def initial_pose_confirmation(sample, requested_pose, request_stamp_s,
                              expected_epoch, current_epoch):
    """Require fresh, post-request TF in the current epoch at the requested pose."""
    current_pose = finite_pose(sample.get('pose') if isinstance(sample, dict) else None)
    try:
        sample_stamp_s = float(sample.get('stamp_sim_s')) if isinstance(sample, dict) else math.nan
        request_stamp_s = float(request_stamp_s)
    except (TypeError, ValueError, OverflowError):
        sample_stamp_s = math.nan
        request_stamp_s = math.nan
    position_error_m = (position_error(current_pose, requested_pose)
                        if current_pose is not None else None)
    yaw_error_rad = (yaw_error(current_pose, requested_pose)
                     if current_pose is not None else None)
    checks = {
        'fresh_transform': bool(isinstance(sample, dict) and sample.get('fresh') is True),
        'post_request_transform': (math.isfinite(sample_stamp_s)
                                  and math.isfinite(request_stamp_s)
                                  and sample_stamp_s >= request_stamp_s),
        'current_tf_epoch': (isinstance(sample, dict)
                             and sample.get('tf_epoch_generation') == expected_epoch
                             and expected_epoch == current_epoch),
        'position_error_within_0_05m': (position_error_m is not None
                                       and position_error_m <= INITIAL_POSE_POSITION_TOLERANCE_M),
        'yaw_error_within_0_05rad': (yaw_error_rad is not None
                                    and yaw_error_rad <= INITIAL_POSE_YAW_TOLERANCE_RAD),
    }
    failed = [name for name, passed in checks.items() if passed is not True]
    return {
        'passed': not failed,
        'checks': checks,
        'failed_checks': failed,
        'sample_stamp_sim_s': sample_stamp_s if math.isfinite(sample_stamp_s) else None,
        'position_error_m': position_error_m,
        'yaw_error_rad': yaw_error_rad,
        'position_tolerance_m': INITIAL_POSE_POSITION_TOLERANCE_M,
        'yaw_tolerance_rad': INITIAL_POSE_YAW_TOLERANCE_RAD,
    }


def initial_pose_after_supervised_load(pre_load_map_pose, pre_load_gazebo_pose,
                                       post_load_gazebo_pose):
    """Project the fresh saved-map pose to the robot's post-restart world pose."""
    if (position_error(post_load_gazebo_pose, pre_load_gazebo_pose) <= 0.05
            and yaw_error(post_load_gazebo_pose, pre_load_gazebo_pose) <= 0.05):
        return tuple(pre_load_map_pose), 'same_simulated_robot_pose_before_and_after_transition'
    return (map_pose_from_world(pre_load_map_pose, pre_load_gazebo_pose,
                                post_load_gazebo_pose),
            'measured_pre_load_map_to_gazebo_transform')


def _read_pgm(path: Path) -> tuple[int, int, bytes]:
    raw = path.read_bytes()
    offset = 0
    tokens = []
    while len(tokens) < 4:
        while offset < len(raw):
            if raw[offset] in b' \t\r\n\v\f':
                offset += 1
            elif raw[offset] == ord('#'):
                newline = raw.find(b'\n', offset)
                if newline < 0:
                    raise ValueError('PGM comment is not terminated')
                offset = newline + 1
            else:
                break
        start = offset
        while offset < len(raw) and raw[offset] not in b' \t\r\n\v\f#':
            offset += 1
        if start == offset:
            raise ValueError('PGM header is incomplete')
        tokens.append(raw[start:offset].decode('ascii'))
    magic, width_text, height_text, maximum_text = tokens
    width, height, maximum = int(width_text), int(height_text), int(maximum_text)
    if magic != 'P5' or width <= 0 or height <= 0 or maximum != 255:
        raise ValueError('saved map must use a valid 8-bit binary PGM')
    if offset >= len(raw) or raw[offset] not in b' \t\r\n\v\f':
        raise ValueError('PGM raster separator is missing')
    if raw[offset:offset + 2] == b'\r\n':
        offset += 2
    else:
        offset += 1
    pixels = raw[offset:]
    if len(pixels) != width * height:
        raise ValueError('PGM raster size does not match its dimensions')
    return width, height, pixels


def compare_active_grid_with_saved_map(grid, yaml_path: Path,
                                       image_path: Path) -> dict:
    """Independently decode the saved Nav2 trinary files and compare /map."""
    import yaml

    document = yaml.safe_load(yaml_path.read_text(encoding='utf-8')) or {}
    yaml_image = (yaml_path.parent / str(document.get('image') or '')).resolve()
    if yaml_image != image_path.resolve():
        return {'matches': False, 'reason': 'YAML image path does not select the saved PGM'}
    width, height, pixels = _read_pgm(image_path)
    resolution = float(document['resolution'])
    origin = [float(value) for value in document['origin']]
    info = grid.info
    orientation = info.origin.orientation
    loaded_yaw = math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))
    metadata_matches = (
        int(info.width) == width and int(info.height) == height
        and math.isclose(float(info.resolution), resolution, rel_tol=0.0, abs_tol=1e-6)
        and math.isclose(float(info.origin.position.x), origin[0], rel_tol=0.0, abs_tol=1e-5)
        and math.isclose(float(info.origin.position.y), origin[1], rel_tol=0.0, abs_tol=1e-5)
        and abs(math.atan2(math.sin(loaded_yaw - origin[2]),
                           math.cos(loaded_yaw - origin[2]))) <= 1e-5
    )
    if not metadata_matches:
        return {'matches': False, 'reason': 'active /map geometry differs from YAML/PGM'}
    free_threshold = float(document['free_thresh'])
    occupied_threshold = float(document['occupied_thresh'])
    negate = bool(int(document.get('negate', 0)))
    expected = []
    for row in range(height):
        image_row = height - row - 1
        for column in range(width):
            gray = pixels[image_row * width + column] / 255.0
            probability = gray if negate else 1.0 - gray
            expected.append(100 if occupied_threshold < probability else
                            0 if probability < free_threshold else -1)
    actual = [int(value) for value in grid.data]
    mismatches = [index for index, (left, right) in enumerate(zip(actual, expected))
                  if left != right]
    matches = len(actual) == len(expected) and not mismatches
    return {
        'matches': matches,
        'reason': None if matches else 'active /map occupancy differs from decoded PGM',
        'cell_count': len(expected), 'mismatch_count': len(mismatches)
            + abs(len(actual) - len(expected)),
        'first_mismatch_index': mismatches[0] if mismatches else None,
        'expected_occupied_cells': expected.count(100),
        'expected_free_cells': expected.count(0),
        'expected_unknown_cells': expected.count(-1),
        'actual_occupied_cells': actual.count(100),
        'actual_free_cells': actual.count(0),
        'actual_unknown_cells': actual.count(-1),
        'free_threshold': free_threshold,
        'occupied_threshold': occupied_threshold,
    }


def map_pose_from_world(map_pose, measured_world_pose, current_world_pose):
    """Apply the measured SLAM-map-to-Gazebo transform to a fresh world pose."""
    delta_yaw = math.atan2(math.sin(map_pose[2] - measured_world_pose[2]),
                           math.cos(map_pose[2] - measured_world_pose[2]))
    cosine, sine = math.cos(delta_yaw), math.sin(delta_yaw)
    translated_x = map_pose[0] - (cosine * measured_world_pose[0] - sine * measured_world_pose[1])
    translated_y = map_pose[1] - (sine * measured_world_pose[0] + cosine * measured_world_pose[1])
    return (
        cosine * current_world_pose[0] - sine * current_world_pose[1] + translated_x,
        sine * current_world_pose[0] + cosine * current_world_pose[1] + translated_y,
        math.atan2(math.sin(current_world_pose[2] + delta_yaw),
                   math.cos(current_world_pose[2] + delta_yaw)),
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend-url', required=True)
    parser.add_argument('--frontend-origin', required=True)
    parser.add_argument('--artifact-root', required=True)
    parser.add_argument('--runtime-dir', required=True)
    parser.add_argument('--robot-id', default='R01')
    parser.add_argument('--navigation-runs', type=int, default=5)
    parser.add_argument('--navigation-timeout', type=float, default=240.0)
    parser.add_argument('--json')
    args = parser.parse_args()

    report = {
        'test_id': 'REAL_GAZEBO_SLAM_SAVE_LOAD_NAVIGATION',
        'started_at_utc': datetime.now(timezone.utc).isoformat(),
        'source_commit': None,
        'robot_id': args.robot_id,
        'navigation_runs': [],
        'operator_ws_reconnections': [],
        'phases': {},
        'passed': False,
    }
    try:
        import subprocess
        source_root = Path(__file__).resolve().parents[1]
        report['source_commit'] = subprocess.run(
            ['git', '-C', str(source_root), 'rev-parse', 'HEAD'], check=True, capture_output=True,
            text=True, timeout=5).stdout.strip()
        ws = operator_websocket(args.backend_url, args.frontend_origin)
    except Exception as exc:
        report['reason'] = f'{type(exc).__name__}:{exc}'
        report['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        write_report(args.json, report)
        print(f'SAVE_LOAD_ACCEPTANCE=FAIL reason={report["reason"]}', flush=True)
        return 1

    rclpy.init()
    probe = MotionProbe()
    base = args.backend_url.rstrip('/')
    robot = args.robot_id
    local_url = f'{base}/api/robots/{robot}/local/maps'
    supervisor_status = Path(args.runtime_dir) / 'mode-switch-status.json'
    try:
        bridge_ready = probe.wait_until(
            lambda: probe.runtime_status is not None
            and robot in probe.runtime_status.get('connected_robot_ids', []), 20.0, ws)
        map_ready = probe.wait_until(
            lambda: probe.map is not None and probe.map_sample_count > 0
            and probe.map_pose() is not None, 20.0, ws)
        status_code, local_state = request_json(local_url)
        runtime_maps = ((probe.runtime_status or {}).get('local_active_maps') or {}).get(robot) or {}
        if not bridge_ready or not map_ready or status_code != 200:
            raise RuntimeError(f'pre-save readiness failed: bridge={bridge_ready}, map={map_ready}, '
                               f'HTTP={status_code}, state={local_state}')
        pose_pair = {}
        def capture_fresh_pose_pair():
            sample, gazebo_sample = probe.map_gazebo_pose_pair()
            gazebo_pose = (gazebo_sample or {}).get('pose')
            gazebo_age = (gazebo_sample or {}).get('sample_age_wall_s')
            pair_skew = (sample or {}).get('pose_pair_time_skew_sim_s')
            if (sample and sample.get('fresh') and gazebo_pose is not None
                    and gazebo_age is not None and gazebo_age <= 1.0
                    and pair_skew is not None and pair_skew <= 0.5):
                pose_pair.update(map_sample=sample, map_pose=sample.get('pose'),
                                 gazebo_pose=gazebo_pose,
                                 gazebo_sample=gazebo_sample,
                                 gazebo_age_wall_s=gazebo_age,
                                 map_tf_gazebo_skew_sim_s=pair_skew,
                                 gazebo_pose_sim_s=gazebo_sample.get('simulation_time_s'))
                return True
            return False
        if not probe.wait_until(capture_fresh_pose_pair, 20.0, ws):
            raise RuntimeError('fresh map/Gazebo pose pair unavailable before mapping handoff')
        if str(local_state.get('mapping_state') or '').upper() not in ('MAPPING', 'PAUSED'):
            raise RuntimeError(f'SLAM Toolbox mapping is not active: {local_state.get("mapping_state")}')
        if runtime_maps.get('map_source') != 'SLAM_TOOLBOX':
            raise RuntimeError(f'active ROS map is not from SLAM Toolbox: {runtime_maps}')
        pre_save_pose = finite_pose(pose_pair.get('map_pose'))
        if pre_save_pose is None:
            raise RuntimeError('fresh map-to-base pose unavailable before save')
        pre_save_gazebo_pose = finite_pose(pose_pair.get('gazebo_pose'))
        pre_save_gazebo_age = pose_pair.get('gazebo_age_wall_s')
        pre_save_map_sample = pose_pair['map_sample']
        report['slam_session_id'] = str(runtime_maps.get('active_map_id') or '').removeprefix('SLAM-') or None
        report['source_map_identity'] = {
            'active_map_id': runtime_maps.get('active_map_id'),
            'active_map_revision': runtime_maps.get('active_map_revision'),
            'mapping_session_id': report['slam_session_id'],
            'resolution': float(probe.map.info.resolution),
            'width': int(probe.map.info.width), 'height': int(probe.map.info.height),
            'known_cells': sum(1 for value in probe.map.data if int(value) >= 0),
        }
        if report['source_map_identity']['known_cells'] <= 0:
            raise RuntimeError('SLAM accumulated map has no known occupancy cells')
        initial_e2e = {}
        try:
            initial_e2e = json.loads((Path(args.runtime_dir) / 'end-to-end.json').read_text(encoding='utf-8'))
        except (OSError, ValueError):
            pass
        initial_e2e_pose = (finite_pose((initial_e2e.get('initial_pose_map') or {}).get('pose'))
                            if initial_e2e.get('initial_pose_ready') else None)
        initial_e2e_gazebo = finite_pose(initial_e2e.get('initial_pose_gazebo'))
        report['simulation_initial_pose'] = {
            'map_pose': initial_e2e_pose, 'gazebo_pose': initial_e2e_gazebo,
            'available': bool(initial_e2e_pose and initial_e2e_gazebo),
        }

        mode_ok, mode_reason = set_mode(probe, ws, robot, 'MANUAL')
        if not mode_ok:
            raise RuntimeError(f'could not enter MANUAL before map save: {mode_reason}')
        settling = probe.wait_mechanical_settling(ws, timeout=30.0, wall_timeout=120.0)
        if not settling.get('passed'):
            raise RuntimeError(f'robot did not settle before map save: {settling.get("reason")}')
        settled_world_pose = finite_pose(probe.gazebo_pose)
        settled_world_age = (
            time.monotonic() - probe.gazebo_pose_sample_monotonic
            if probe.gazebo_pose_sample_monotonic is not None else None)
        if (settled_world_pose is None or settled_world_age is None or settled_world_age > 1.0
                or position_error(settled_world_pose, pre_save_gazebo_pose) > 0.05
                or yaw_error(settled_world_pose, pre_save_gazebo_pose) > 0.05):
            raise RuntimeError('robot moved after the fresh map/Gazebo pose pair was captured')
        report['phases']['robot_stopped_in_manual'] = {
            'passed': True, 'settling': settling,
            'control_mode': 'MANUAL', 'pose_map': pre_save_pose,
            'pose_map_sample': pre_save_map_sample,
            'gazebo_pose': settled_world_pose,
            'gazebo_pose_age_wall_s': settled_world_age,
            'map_tf_gazebo_skew_sim_s': pose_pair['map_tf_gazebo_skew_sim_s'],
            'pose_unchanged': True,
        }

        pause_code, pause = request_json(f'{base}/api/robots/{robot}/local/mapping/stop',
                                         'POST', {}, timeout=15.0)
        if pause_code != 200 or pause.get('mapping_state') != 'PAUSED':
            raise RuntimeError(f'pausing live SLAM failed: HTTP {pause_code}, {pause}')
        status_code, paused_state = request_json(local_url)
        if status_code != 200 or paused_state.get('mapping_state') != 'PAUSED':
            raise RuntimeError(f'backend did not confirm paused mapping: {paused_state}')
        report['phases']['mapping_paused'] = {'passed': True, 'response': pause,
                                              'status': paused_state.get('mapping_state')}

        name = 'release-acceptance-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S') + '-' + uuid.uuid4().hex[:6]
        save_code, save_response = request_json(
            f'{base}/api/robots/{robot}/local/maps/save', 'POST', {'name': name}, timeout=60.0)
        if save_code != 200 or save_response.get('ok') is not True:
            raise RuntimeError(f'real SLAM map save failed: HTTP {save_code}, {save_response}')
        record = save_response.get('map') or {}
        if (record.get('name') != name or record.get('robot_id') != robot
                or not record.get('id') or not record.get('revision')
                or record.get('map_kind') != 'SAVED_LOCAL_MAP'
                or record.get('canonical_map_promoted') is not False
                or (record.get('slam_session_state') or {}).get('status') != 'AVAILABLE'):
            raise RuntimeError(f'save response lacks persisted map/session identity: {record}')

        import yaml
        map_dir = Path(args.artifact_root).resolve() / 'local_robot_maps' / robot
        yaml_path = map_dir / f'{name}.yaml'
        image_path = map_dir / f'{name}.pgm'
        posegraph_path = map_dir / f'{name}_slam_session.posegraph'
        session_data_path = map_dir / f'{name}_slam_session.data'
        for path in (yaml_path, image_path, posegraph_path, session_data_path):
            if not path.is_file() or path.stat().st_size <= 0:
                raise RuntimeError(f'real save omitted or emptied required artifact: {path.name}')
        yaml_doc = yaml.safe_load(yaml_path.read_text(encoding='utf-8')) or {}
        if Path(str(yaml_doc.get('image') or '')).name != image_path.name:
            raise RuntimeError('saved YAML image reference does not select the persisted PGM')
        expected_files = {
            path.name: {'size_bytes': path.stat().st_size,
                        'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
            for path in (yaml_path, image_path, posegraph_path, session_data_path)
        }
        if record.get('image_sha256') != expected_files[image_path.name]['sha256']:
            raise RuntimeError('saved-map registry checksum does not match the PGM artifact')
        if (int(record.get('width', -1)) <= 0 or int(record.get('height', -1)) <= 0
                or not math.isclose(float(record.get('resolution', 0)),
                                    float(yaml_doc.get('resolution', -1)), abs_tol=1e-6)
                or int(record['width']) != report['source_map_identity']['width']
                or int(record['height']) != report['source_map_identity']['height']):
            raise RuntimeError('saved map geometry does not match the accumulated SLAM map')
        status_code, listed = request_json(local_url)
        listed_record = next((row for row in listed.get('maps', []) if row.get('id') == record['id']), None)
        if status_code != 200 or listed_record is None:
            raise RuntimeError('saved map is not visible from the persisted backend registry')
        report.update({
            'saved_map_id': record['id'], 'saved_map_revision': record['revision'],
            'artifact_paths': {key: str(map_dir / key) for key in expected_files},
            'artifact_validation': expected_files,
            'saved_map_registry': record,
        })
        report['phases']['save_map'] = {
            'passed': True, 'http_status': save_code,
            'artifact_count': len(expected_files), 'registry_visible': True,
            'canonical_map_promoted': record['canonical_map_promoted'],
        }

        mode_ok, mode_reason = set_mode(probe, ws, robot, 'MANUAL')
        if not mode_ok:
            raise RuntimeError(f'could not ensure MANUAL before saved map load: {mode_reason}')
        settling = probe.wait_mechanical_settling(ws, timeout=30.0, wall_timeout=120.0)
        if not settling.get('passed'):
            raise RuntimeError(f'robot did not settle before saved-map load: {settling.get("reason")}')
        # SLAM Toolbox stops publishing map->odom while Mapping is paused. Carry
        # forward the last fresh SLAM pose, but only if Gazebo confirms that the
        # mechanically settled robot has not moved since that paired sample.
        # A stale TF timestamp must not be presented as a current localization.
        pre_load_pose = pre_save_pose
        pre_load_gazebo_pose = pre_save_gazebo_pose
        current_gazebo_pose = finite_pose(probe.gazebo_pose)
        current_gazebo_age = (
            time.monotonic() - probe.gazebo_pose_sample_monotonic
            if probe.gazebo_pose_sample_monotonic is not None else None)
        if (current_gazebo_pose is None or current_gazebo_age is None or current_gazebo_age > 1.0
                or position_error(current_gazebo_pose, pre_load_gazebo_pose) > 0.05
                or yaw_error(current_gazebo_pose, pre_load_gazebo_pose) > 0.05):
            raise RuntimeError('robot pose changed or Gazebo pose is stale before supervised map load')
        pre_load_tf_diagnostic = probe.map_pose_sample()
        status_code, state = request_json(local_url)
        if (status_code != 200 or state.get('robot_control_mode') != 'MANUAL'
                or not state.get('robot_stopped') or state.get('estop_active') is True):
            raise RuntimeError(f'load safety preconditions not confirmed by backend: {state}')
        report['pre_load_pose_map'] = pre_load_pose
        report['pre_load_pose_evidence'] = {
            'source': 'fresh_SLAM_TF_pair_before_pause',
            'map_tf_sample': pre_save_map_sample,
            'gazebo_pose_at_reference': pre_load_gazebo_pose,
            'gazebo_pose_sim_s_at_reference': pose_pair['gazebo_pose_sim_s'],
            'map_tf_gazebo_skew_sim_s': pose_pair['map_tf_gazebo_skew_sim_s'],
            'gazebo_pose_before_load': current_gazebo_pose,
            'gazebo_pose_age_wall_s': current_gazebo_age,
            'position_drift_m': position_error(current_gazebo_pose, pre_load_gazebo_pose),
            'yaw_drift_rad': yaw_error(current_gazebo_pose, pre_load_gazebo_pose),
            'map_tf_diagnostic_before_load': pre_load_tf_diagnostic,
            'accepted': True,
        }

        # Navigation mode uses Nav2's /navigation_map, not SLAM's /map.
        # /map may retain the final pre-transition SLAM snapshot; accepting it
        # would report success without proving the selected raster is active.
        map_count_before_load = probe.navigation_map_sample_count
        status_code, load_state = request_json(
            f'{base}/api/robots/{robot}/local/maps/load', 'POST',
            {'map_id': record['id']}, timeout=30.0)
        supervisor_transitions = []
        reapplied_manual = False

        def reconnect_operator_ws(phase):
            nonlocal ws
            try:
                ws.close()
            except (OSError, websocket.WebSocketException):
                pass
            ws = operator_websocket(args.backend_url, args.frontend_origin)
            probe.operator_ws_closed = False
            probe.runtime_status = None
            connected = probe.wait_until(
                lambda: probe.runtime_status is not None
                and robot in probe.runtime_status.get('connected_robot_ids', []),
                30.0, ws)
            if not connected:
                raise RuntimeError('operator WebSocket reconnected but the ROS bridge did not return')
            report['operator_ws_reconnections'].append({
                'phase': phase, 'robot_id': robot, 'connected_robot': True,
            })

        deadline = time.monotonic() + 600.0
        while status_code == 202 and load_state.get('status') == 'TRANSITIONING':
            try:
                status_text = supervisor_status.read_text(encoding='utf-8')
                supervisor = json.loads(status_text)
                supervisor_transitions.append({key: supervisor.get(key) for key in
                    ('status', 'mode', 'requested_mode', 'message', 'request_id')})
                report['supervisor_transitions'] = list(supervisor_transitions)
            except (OSError, ValueError):
                pass
            if (load_state.get('phase') == 'ROLLING_BACK'
                    or transition_status(load_state) in ('ERROR', 'ROLLED_BACK')):
                break
            if time.monotonic() >= deadline:
                raise RuntimeError(f'supervised load timed out: {load_state}')
            probe.pump(ws, 0.01)
            if probe.operator_ws_closed or not getattr(ws, 'connected', True):
                reconnect_operator_ws(load_state.get('phase'))
            if load_state.get('phase') == 'WAITING_FOR_MANUAL' and not reapplied_manual:
                first_error = None
                mode_transport_failure = False
                try:
                    mode_ok, mode_reason = set_mode(probe, ws, robot, 'MANUAL', timeout=15.0)
                except (OSError, websocket.WebSocketException) as exc:
                    mode_ok, mode_reason = False, f'{type(exc).__name__}:{exc}'
                    mode_transport_failure = True
                if (not mode_ok and (probe.operator_ws_closed
                                     or not getattr(ws, 'connected', True)
                                     or mode_transport_failure
                                     or mode_reason == 'no_accepted_ROBOT_CONTROL_STATUS')):
                    first_error = mode_reason
                    reconnect_operator_ws(load_state.get('phase'))
                    try:
                        mode_ok, mode_reason = set_mode(
                            probe, ws, robot, 'MANUAL', timeout=15.0)
                    except (OSError, websocket.WebSocketException) as exc:
                        mode_ok, mode_reason = False, f'{type(exc).__name__}:{exc}'
                if not mode_ok:
                    raise RuntimeError(
                        f'could not restore MANUAL after supervised Nav2 restart: '
                        f'{mode_reason}; first_attempt={first_error}')
                # The supervised transition destroys and recreates Gazebo and
                # the command arbiter. Do not rely on the pre-restart STOP or
                # a previous process's zero command. Reassert the priority
                # STOP in the new epoch, then prove physical and backend state
                # are stopped before retrying map load.
                ws.send(json.dumps({'type': 'ROBOT_MANUAL', 'robot_id': robot,
                                    'action': 'STOP'}))
                stopped_after_restart = probe.wait_mechanical_settling(
                    ws, timeout=30.0, wall_timeout=120.0)
                if not stopped_after_restart.get('passed'):
                    raise RuntimeError(
                        'robot did not mechanically settle after the supervised '
                        f'Nav2 restart: {stopped_after_restart.get("reason")}')
                stop_deadline = time.monotonic() + 15.0
                post_restart_state = {}
                while time.monotonic() < stop_deadline:
                    state_code, post_restart_state = request_json(local_url)
                    if (state_code == 200
                            and post_restart_state.get('robot_control_mode') == 'MANUAL'
                            and post_restart_state.get('robot_stopped') is True
                            and post_restart_state.get('estop_active') is False):
                        break
                    probe.pump(ws, 0.01)
                    time.sleep(0.1)
                else:
                    raise RuntimeError(
                        'fresh backend state did not confirm MANUAL, stopped, '
                        f'E-Stop-clear after restart: HTTP {state_code}, {post_restart_state}')
                reapplied_manual = True
                report['phases']['manual_restored_after_runtime_restart'] = {
                    'passed': True, 'mode': 'MANUAL', 'robot_id': robot,
                    'reconnected_after_missing_ack': first_error is not None,
                    'first_attempt_error': first_error,
                }
                report['phases']['robot_stopped_after_runtime_restart'] = {
                    'passed': True,
                    'robot_control_mode': post_restart_state.get('robot_control_mode'),
                    'robot_stopped': post_restart_state.get('robot_stopped'),
                    'estop_active': post_restart_state.get('estop_active'),
                    'mechanical_settling': stopped_after_restart,
                }
            time.sleep(0.5)
            status_code, load_state = request_json(
                f'{base}/api/robots/{robot}/local/maps/load', 'POST',
                {'map_id': record['id']}, timeout=30.0)
        if status_code != 200 or load_state.get('status') != 'LOADED':
            raise RuntimeError(f'backend did not confirm Nav2 map load: HTTP {status_code}, {load_state}')
        report['supervisor_transitions'] = supervisor_transitions

        # The supervised mapping->navigation handoff restarts Gazebo and
        # rewinds /clock. EpochTfObserver replaces its /tf subscription at the
        # rewind boundary, dropping queued messages from the prior world and
        # reacquiring transient-local /tf_static before localization evidence
        # is considered. A plain Buffer.clear() would leave those DDS samples
        # queued and allow TF_OLD_DATA to poison the new epoch.
        reset_deadline = time.monotonic() + 10.0
        while (len(probe.tf_observer.clock_resets) <= 0
               and time.monotonic() < reset_deadline):
            probe.pump(ws, 0.01)
        if not probe.tf_observer.clock_resets:
            raise RuntimeError('TF observer did not confirm the Gazebo simulation-clock rewind')
        report['simulation_clock_reset'] = {
            'observed': True,
            'epoch_generation': probe.tf_observer.epoch_generation,
            'events': list(probe.tf_observer.clock_resets),
            'listener_recreated': True,
        }

        gazebo_sample_before_load = probe.gazebo_pose_sample_monotonic
        fresh_gazebo = probe.wait_until(lambda: probe.gazebo_pose is not None
            and probe.gazebo_pose_sample_monotonic is not None
            and (gazebo_sample_before_load is None
                 or probe.gazebo_pose_sample_monotonic > gazebo_sample_before_load), 20.0, ws)
        if not fresh_gazebo:
            raise RuntimeError('Gazebo did not publish a fresh robot pose after supervised runtime restart')
        loaded_gazebo_pose = finite_pose(probe.gazebo_pose)
        # A prior startup pose is not reliable after SLAM has updated map->odom
        # (for example through map registration or loop closure). Project the
        # fresh saved-map-time map/Gazebo pair to the post-restart physical pose.
        initial_pose_for_map, initial_pose_method = initial_pose_after_supervised_load(
            pre_load_pose, pre_load_gazebo_pose, loaded_gazebo_pose)
        report['post_transition_pose'] = {
            'gazebo_pose': loaded_gazebo_pose,
            'initial_pose_method': initial_pose_method,
            'initial_pose_map': initial_pose_for_map,
            'pre_load_map_pose': pre_load_pose,
            'pre_load_gazebo_pose': pre_load_gazebo_pose,
            'fresh_gazebo_sample': fresh_gazebo,
        }

        # The load call is only one signal: require the actual Nav2-published
        # /navigation_map to match the selected raster, the active-map identity
        # in backend state, and the runtime capability. Do not inspect /map:
        # that topic is SLAM-owned and can remain latched after the handoff.
        def nav_grid_geometry_matches():
            grid = probe.navigation_map
            return bool(
                grid is not None
                and int(grid.info.width) == int(record['width'])
                and int(grid.info.height) == int(record['height'])
                and math.isclose(float(grid.info.resolution),
                                 float(record['resolution']), abs_tol=1e-6)
            )

        fresh_map = probe.wait_until(
            lambda: probe.navigation_map_sample_count > map_count_before_load
            and nav_grid_geometry_matches(), 30.0, ws)
        status_code, active_state = request_json(local_url)
        active_identity = (active_state.get('active_map_id') == record['id']
            and str(active_state.get('active_map_revision')) == str(record['revision'])
            and active_state.get('map_source') == 'LOCAL_MAP'
            and active_state.get('map_sync_status') == 'LOCAL_ONLY'
            and active_state.get('local_active_map_id') == record['id']
            and str(active_state.get('local_active_map_revision')) == str(record['revision']))
        nav_ready = bool(((probe.runtime_status or {}).get('robot_capabilities') or {})
                         .get(robot, {}).get('nav2_ready'))
        grid = probe.navigation_map
        grid_matches = bool(fresh_map and grid and int(grid.info.width) == int(record['width'])
            and int(grid.info.height) == int(record['height'])
            and math.isclose(float(grid.info.resolution), float(record['resolution']), abs_tol=1e-6))
        grid_content = (compare_active_grid_with_saved_map(grid, yaml_path, image_path)
                        if grid_matches else {'matches': False,
                                              'reason': 'active map metadata is not ready'})
        grid_matches = grid_matches and grid_content.get('matches') is True
        if status_code != 200 or not active_identity or not nav_ready or not grid_matches:
            raise RuntimeError('loaded map lacks independent active-state confirmation: '
                f'active_identity={active_identity}, nav2_ready={nav_ready}, fresh_map={fresh_map}, '
                f'grid_matches={grid_matches}, grid_content={grid_content}, state={active_state}')
        report['active_ros_map'] = {
            'topic': '/navigation_map', 'fresh_message': fresh_map,
            'message_count_after_load': probe.navigation_map_sample_count - map_count_before_load,
            'width': int(grid.info.width), 'height': int(grid.info.height),
            'resolution': float(grid.info.resolution),
            'origin': [float(grid.info.origin.position.x), float(grid.info.origin.position.y)],
            'active_map_id': active_state.get('active_map_id'),
            'active_map_revision': active_state.get('active_map_revision'),
            'map_source': active_state.get('map_source'), 'map_sync_status': active_state.get('map_sync_status'),
            'nav2_ready': nav_ready,
            'content_validation': grid_content,
        }

        tf_ready_after_reset = probe.wait_until(
            lambda: (lambda sample: bool(sample and sample.get('fresh')
                     and sample.get('tf_epoch_generation') == probe.tf_observer.epoch_generation))(
                         probe.map_pose_sample()), 15.0, ws)
        report['tf_after_clock_reset'] = {
            'passed': tf_ready_after_reset,
            'sample': probe.map_pose_sample() if tf_ready_after_reset else None,
            'epoch_generation': probe.tf_observer.epoch_generation,
        }
        if not tf_ready_after_reset:
            raise RuntimeError('fresh map-frame TF was not available from the post-restart epoch')

        mode_ok, mode_reason = set_mode(probe, ws, robot, 'MANUAL')
        if not mode_ok:
            raise RuntimeError(f'Nav2 load succeeded but MANUAL could not be confirmed: {mode_reason}')
        initial_pose = {'x': initial_pose_for_map[0], 'y': initial_pose_for_map[1],
                        'yaw': initial_pose_for_map[2], 'frame_id': 'map'}
        initial_request_start_sim_s = probe.sim_time()
        initial_request_epoch = probe.tf_observer.tf_snapshot()[1]
        initial_code, initial = request_json(
            f'{base}/api/robots/{robot}/local/initial-pose', 'POST', initial_pose, timeout=20.0)
        if (initial_code != 200 or initial.get('ok') is not True
                or initial.get('active_map_id') != record['id']
                or str(initial.get('active_map_revision')) != str(record['revision'])
                or not isinstance(initial.get('request_stamp_s'), (int, float))
                or not isinstance(initial.get('tf_stamp_s'), (int, float))
                or not isinstance(initial.get('tf_age_s'), (int, float))
                or float(initial.get('tf_stamp_s', -1)) < float(initial.get('request_stamp_s', 0))
                or not -0.1 <= float(initial.get('tf_age_s', -1)) <= 0.5
                or not isinstance(initial.get('tf_confirmation_duration_sim_s'), (int, float))
                or float(initial.get('tf_confirmation_duration_sim_s', 0.0)) < 0.25
                or not isinstance(initial.get('tf_confirmation_samples'), int)
                or initial.get('tf_confirmation_samples', 0) < 3
                or not isinstance(initial.get('position_error_m'), (int, float))
                or float(initial.get('position_error_m', math.inf)) > INITIAL_POSE_POSITION_TOLERANCE_M
                or not isinstance(initial.get('yaw_error_rad'), (int, float))
                or float(initial.get('yaw_error_rad', math.inf)) > INITIAL_POSE_YAW_TOLERANCE_RAD):
            raise RuntimeError(f'initial pose was not confirmed on the loaded map: {initial}')
        latest_pose_confirmation = {'sample': None, 'position_error_m': None,
                                    'yaw_error_rad': None,
                                    'request_start_sim_s': initial_request_start_sim_s,
                                    'bridge_request_stamp_s': initial.get('request_stamp_s'),
                                    'bridge_tf_stamp_s': initial.get('tf_stamp_s'),
                                    'bridge_tf_age_s': initial.get('tf_age_s')}

        def pose_confirmed_now():
            sample = probe.map_pose_sample()
            confirmation = initial_pose_confirmation(
                sample, initial_pose_for_map, initial.get('request_stamp_s'),
                initial_request_epoch, probe.tf_observer.tf_snapshot()[1])
            latest_pose_confirmation.update({
                'sample': sample,
                **confirmation,
            })
            return confirmation['passed']

        pose_confirmed = probe.wait_until(pose_confirmed_now, 20.0, ws)
        report['phases']['initial_pose'] = {
            'passed': pose_confirmed,
            'response': initial,
            'requested_pose': initial_pose,
            'external_tf_confirmation': latest_pose_confirmation,
            'simulation_pose_method': initial_pose_method,
            'tf_epoch_generation': probe.tf_observer.epoch_generation,
            'clock_resets': list(probe.tf_observer.clock_resets),
        }
        if not pose_confirmed:
            raise RuntimeError('map-frame transform did not confirm the requested initial pose')
        report['phases']['load_map'] = {
            'passed': True, 'http_status': status_code, 'status': load_state.get('status'),
            'active_identity': active_identity, 'active_ros_map': report['active_ros_map'],
            'supervisor_transition_count': len(supervisor_transitions),
        }

        # Keep the existing 5 cm / 0.05 rad release criteria. Each NAV2 mission
        # is independent and must pass individually; do not average away a miss.
        for run in range(1, max(1, args.navigation_runs) + 1):
            # Initial-pose confirmation is a state boundary. A perfectly fresh
            # Gazebo sample captured just before that request is still invalid
            # as the navigation start pose: AMCL/EKF may have changed map->odom
            # immediately after it. Require both sources at/after the bridge's
            # confirmed transform stamp before deriving the independent world
            # projection or authorizing the mission measurement.
            result = web_navigation(
                probe, ws, robot, args.navigation_timeout,
                minimum_pose_stamp_s=float(initial['tf_stamp_s']),
                verified_local_map_identity={
                    'active_map_id': record['id'],
                    'active_map_revision': str(record['revision']),
                })
            result['run'] = run
            report['navigation_runs'].append(result)
            print(f'SAVE_LOAD_NAV_RUN_{run}={"PASS" if result.get("passed") else "FAIL"} '
                  f'status={result.get("status")} translation_error_m={result.get("goal_distance_m")} '
                  f'yaw_error_rad={result.get("goal_yaw_error_rad")} '
                  f'translation_error_at_terminal_m={result.get("goal_error_at_terminal_m")}', flush=True)
            if not result.get('passed'):
                break
        navigation_passed = (len(report['navigation_runs']) == max(1, args.navigation_runs)
            and all(item.get('passed') is True
                    and item.get('goal_distance_m') is not None
                    and item['goal_distance_m'] <= 0.05
                    and item.get('goal_yaw_error_rad') is not None
                    and item['goal_yaw_error_rad'] <= 0.05
                    and item.get('status') == 'SUCCEEDED'
                    for item in report['navigation_runs']))
        report['phases']['navigation_after_load'] = {
            'passed': navigation_passed, 'requested_runs': max(1, args.navigation_runs),
            'completed_runs': len(report['navigation_runs']),
            'translation_tolerance_m': 0.05, 'yaw_tolerance_rad': 0.05,
        }
        report['passed'] = bool(navigation_passed)
        report['reason'] = None if report['passed'] else 'one_or_more_loaded_map_navigation_runs_failed'
    except Exception as exc:
        report['reason'] = f'{type(exc).__name__}:{exc}'
        print(f'SAVE_LOAD_ACCEPTANCE=FAIL reason={report["reason"]}', flush=True)
    finally:
        report['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        write_report(args.json, report)
        try:
            probe.destroy_node()
        finally:
            rclpy.shutdown()
            ws.close()

    print(f'SAVE_LOAD_ACCEPTANCE={"PASS" if report["passed"] else "FAIL"} '
          f'map_id={report.get("saved_map_id")} revision={report.get("saved_map_revision")} '
          f'nav_runs={len(report["navigation_runs"])}', flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
