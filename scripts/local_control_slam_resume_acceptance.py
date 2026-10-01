#!/usr/bin/env python3
"""Resume a saved SLAM session through the Web UI and prove it extends /map."""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'waretwin' / 'backend'))

os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'config.settings')
import django
django.setup()

import rclpy
import websocket
import end_to_end_acceptance as acceptance
from twin.local_control import slam_map_restoration_evidence


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2), encoding='utf-8')


def main() -> int:
    robot_id = os.environ.get('ROBOT_ID', 'R01')
    map_name = os.environ.get('SAVED_MAP_NAME', 'slam_accumulated_20261001_01')
    backend = os.environ.get('BACKEND_URL', '').rstrip('/')
    frontend = os.environ.get('FRONTEND_URL', '')
    artifact_root = Path(os.environ.get('WARETWIN_ARTIFACT_ROOT') or ROOT / 'generated' / 'maps').expanduser().resolve()
    registry_path = artifact_root / 'local_robot_maps' / robot_id / 'registry.json'
    records = json.loads(registry_path.read_text(encoding='utf-8'))
    record = next((row for row in records if row.get('name') == map_name
                   and row.get('robot_id') == robot_id), None)
    if not record:
        raise RuntimeError(f'{map_name} is not registered for {robot_id}')
    map_root = registry_path.parent.resolve()
    yaml_path = (map_root / str(record['_yaml'])).resolve(strict=True)
    image_path = (map_root / str(record['_image'])).resolve(strict=True)
    posegraph_path = (map_root / str(record['_slam_posegraph'])).resolve(strict=True)
    session_data_path = (map_root / str(record['_slam_data'])).resolve(strict=True)
    for artifact in (yaml_path, image_path, posegraph_path, session_data_path):
        if not artifact.is_relative_to(map_root) or not artifact.is_file() or artifact.stat().st_size <= 0:
            raise RuntimeError(f'saved local map/session artifact is missing or out of scope: {artifact.name}')
    if (record.get('slam_session_state') or {}).get('status') != 'AVAILABLE':
        raise RuntimeError('registered saved SLAM session is not AVAILABLE')

    evidence_dir = Path(tempfile.mkdtemp(prefix='slam-resume-', dir=ROOT / '.runtime'))
    browser_log = evidence_dir / 'browser.log'
    output = ROOT / '.runtime' / 'local-map-slam-resume-acceptance.json'
    result = {
        'passed': False, 'source': 'Mapping tab -> Django -> supervisor -> SLAM Toolbox configure -> live /map',
        'robot_id': robot_id, 'saved_map_name': map_name,
        'map_id': record['id'], 'map_revision': record['revision'],
        'source_mapping_session_id': record.get('source_mapping_session_id'),
        'saved_map': {
            'yaml': str(yaml_path), 'image': str(image_path),
            'posegraph_bytes': posegraph_path.stat().st_size,
            'session_data_bytes': session_data_path.stat().st_size,
            'dimensions': [record['width'], record['height']],
            'resolution': record['resolution'], 'origin': record['origin'],
            'known_cells': record.get('known_cells'),
            'image_sha256': record.get('image_sha256'),
        },
        'evidence_directory': str(evidence_dir),
    }
    rclpy.init()
    probe = acceptance.MotionProbe()
    token = acceptance.authenticate(backend)
    ws_url = backend.replace('https://', 'wss://').replace('http://', 'ws://') + '/ws?token=' + token
    ws = websocket.create_connection(ws_url, timeout=5.0, enable_multithread=True)
    ws.settimeout(0.02)
    browser = None
    try:
        ready = probe.wait_until(lambda: probe.odom is not None and probe.gazebo_pose is not None
            and probe.sim_time() > 0 and probe.runtime_status is not None
            and robot_id in (probe.runtime_status.get('connected_robot_ids') or []), 45.0, ws)
        if not ready:
            raise RuntimeError('live Gazebo pose, clock, or authenticated bridge was not ready')
        runtime_state = str(probe.runtime_status.get('runtime_state') or '').upper()
        if runtime_state not in ('NAVIGATION', 'MAPPING'):
            raise RuntimeError(
                f'bounded resume acceptance requires supervised NAVIGATION or MAPPING mode, got {runtime_state or "UNKNOWN"}')
        manual_ok, manual_reason = acceptance.set_mode(probe, ws, robot_id, 'MANUAL', timeout=20.0)
        if not manual_ok:
            raise RuntimeError(f'robot did not confirm MANUAL mode before resume: {manual_reason}')
        result['before_resume'] = {
            'gazebo_pose': probe.gazebo_pose, 'odom_pose': probe.odom,
            'runtime_mode': runtime_state,
        }

        child_env = os.environ.copy()
        child_env.update({
            'BACKEND_URL': backend, 'FRONTEND_URL': frontend,
            'ROBOT_ID': robot_id, 'SAVED_MAP_NAME': map_name,
            'SLAM_RESUME_ACCEPTANCE_DIR': str(evidence_dir),
        })
        for key in ('SLAM_RESUME_SKIP_REQUEST', 'SLAM_RESUME_RESTORE_EVIDENCE'):
            if os.environ.get(key):
                child_env[key] = os.environ[key]
        with browser_log.open('w', encoding='utf-8') as log_stream:
            browser = subprocess.Popen(
                ['node', str(ROOT / 'scripts' / 'local_control_slam_resume_browser.cjs')],
                cwd=ROOT, env=child_env, stdout=log_stream, stderr=subprocess.STDOUT)
            started = time.monotonic()
            teleop_start = teleop_end = None
            start_selected_index = start_owner_index = start_drive_index = None
            while browser.poll() is None:
                probe.pump(ws, 0.02)
                if teleop_start is None:
                    marker = evidence_dir / 'teleop-start.json'
                    if marker.is_file():
                        teleop_start = json.loads(marker.read_text(encoding='utf-8'))
                        start_selected_index = len(probe.selected_cmd_events)
                        start_owner_index = len(probe.command_owner_events)
                        start_drive_index = len(probe.drive_events)
                        result['teleop_start_pose'] = {
                            'gazebo_pose': probe.gazebo_pose, 'map_pose': probe.map_pose(),
                            'odom_pose': probe.odom, 'sim_time': probe.sim_time(),
                        }
                if teleop_end is None:
                    marker = evidence_dir / 'teleop-end.json'
                    if marker.is_file():
                        teleop_end = json.loads(marker.read_text(encoding='utf-8'))
                        result['teleop_end_pose'] = {
                            'gazebo_pose': probe.gazebo_pose, 'map_pose': probe.map_pose(),
                            'odom_pose': probe.odom, 'sim_time': probe.sim_time(),
                        }
                if time.monotonic() - started > 22 * 60:
                    browser.terminate()
                    browser.wait(timeout=10)
                    raise TimeoutError('Web saved-session resume acceptance exceeded 22 minutes')

        browser_code = browser.returncode
        result['browser_exit_code'] = browser_code
        browser_result_path = evidence_dir / 'browser-result.json'
        if not browser_result_path.is_file():
            raise RuntimeError('browser helper did not persist its result; see browser.log')
        browser_result = json.loads(browser_result_path.read_text(encoding='utf-8'))
        result['browser'] = browser_result
        if browser_code != 0 or browser_result.get('errors'):
            raise RuntimeError(f'Web resume/teleop browser acceptance failed: {browser_log.read_text(encoding="utf-8")[-2500:]}')
        if teleop_start is None or teleop_end is None:
            raise RuntimeError('independent ROS observer missed the bounded Web Teleop window')

        restored = browser_result.get('resume_responses') or []
        resume_result = next((item.get('body') for item in reversed(restored)
                              if (item.get('body') or {}).get('status') == 'RESUMED'), None)
        if resume_result is None:
            resume_result = (browser_result.get('restoration_carryover') or {}).get('body')
        initial_map = browser_result.get('initial_map_snapshot') or {}
        extended_map = browser_result.get('extended_map_snapshot') or {}
        initial_check = slam_map_restoration_evidence(record, yaml_path, image_path, initial_map)
        extended_check = slam_map_restoration_evidence(record, yaml_path, image_path, extended_map)
        result['session_restore_response'] = resume_result
        result['initial_map_restoration_check'] = initial_check
        result['post_motion_old_map_check'] = extended_check

        try:
            settled = probe.wait_mechanical_settling(ws, timeout=15.0, wall_timeout=60.0)
        except Exception as exc:
            settled = {'passed': False, 'reason': f'{type(exc).__name__}: {exc}'}
        final_gazebo_pose = probe.gazebo_pose
        start_pose = result.get('teleop_start_pose', {}).get('gazebo_pose')
        motion = acceptance.MotionProbe.body_displacement(start_pose, final_gazebo_pose)
        selected = probe.selected_cmd_events[start_selected_index or 0:]
        owners = probe.command_owner_events[start_owner_index or 0:]
        drives = probe.drive_events[start_drive_index or 0:]
        nonzero_selected = [row for row in selected
            if any(abs(value) > 1e-4 for value in row[1:])]
        result['teleop_motion'] = {
            'action': 'FORWARD', 'start_gazebo_pose': start_pose,
            'final_gazebo_pose': final_gazebo_pose,
            'gazebo_displacement': motion,
            'selected_nonzero_samples': len(nonzero_selected),
            'web_manual_owner_seen': any(owner == 'WEB_MANUAL' for _, owner in owners),
            'drive_command_active': any(any(abs(value) > 1e-4 for value in row[1:]) for row in drives),
            'stop_settled': settled,
        }
        nodes = subprocess.run(
            ['ros2', 'node', 'list', '--no-daemon', '--spin-time', '1'],
            cwd=ROOT, capture_output=True, text=True, timeout=15, check=False)
        node_names = [line.strip() for line in nodes.stdout.splitlines() if line.strip()]
        result['mapping_ownership'] = {
            'slam_toolbox_present': any(name.rstrip('/').endswith('/slam_toolbox') for name in node_names),
            'ekf_v30e_present': any('ekf_v30e' in name for name in node_names),
            'amcl_present': any('amcl' in name for name in node_names),
            'nodes': node_names,
        }

        saved_cells = int(record.get('known_cells') or 0)
        initial_cells = int(initial_map.get('known_cells') or 0)
        final_cells = int(extended_map.get('known_cells') or 0)
        dimensions_grew = (extended_map.get('width'), extended_map.get('height')) != (
            initial_map.get('width'), initial_map.get('height'))
        map_extended = final_cells > initial_cells or dimensions_grew
        result['acceptance'] = {
            'SLAM_SESSION_LOAD': bool(resume_result and resume_result.get('status') == 'RESUMED'),
            'OLD_MAP_RESTORED': bool(initial_check.get('passed')),
            'RESUME_MAPPING': bool(result['teleop_motion']['web_manual_owner_seen']
                and result['teleop_motion']['selected_nonzero_samples'] > 0
                and result['teleop_motion']['drive_command_active']
                and motion and motion['forward_m'] > 0.10 and settled.get('passed')),
            'RESUMED_MAP_EXTENDS': bool(map_extended and extended_check.get('passed')),
            'MAP_ODOM_OWNER_SLAM_TOOLBOX': bool(
                result['mapping_ownership']['slam_toolbox_present']
                and not result['mapping_ownership']['ekf_v30e_present']
                and not result['mapping_ownership']['amcl_present']),
        }
        result['mapping_extension'] = {
            'saved_known_cells': saved_cells, 'restored_before_teleop_known_cells': initial_cells,
            'after_teleop_known_cells': final_cells,
            'known_cell_delta': final_cells - initial_cells,
            'saved_map_dimensions': [record['width'], record['height']],
            'restored_dimensions': [initial_map.get('width'), initial_map.get('height')],
            'after_teleop_dimensions': [extended_map.get('width'), extended_map.get('height')],
            'extent_grew': dimensions_grew,
        }
        result['passed'] = all(result['acceptance'].values())
        if not result['passed']:
            result['failure_reason'] = 'one or more saved-session restoration/extension gates failed'
    except Exception as exc:
        result['failure_reason'] = f'{type(exc).__name__}: {exc}'
    finally:
        if browser is not None and browser.poll() is None:
            browser.terminate()
            browser.wait(timeout=10)
        try:
            ws.close()
        except Exception:
            pass
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        write_json(output, result)

    print(json.dumps({'passed': result.get('passed'),
                      'acceptance': result.get('acceptance'),
                      'failure_reason': result.get('failure_reason'),
                      'report': str(output), 'evidence_directory': str(evidence_dir)}, indent=2))
    return 0 if result.get('passed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
