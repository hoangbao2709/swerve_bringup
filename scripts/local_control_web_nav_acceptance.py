#!/usr/bin/env python3
"""Real Control Detail Web preview/goal acceptance with independent ROS evidence."""
import json, os, re, subprocess, sys, tempfile, time, math
from pathlib import Path

sys.path.insert(0, str(Path.cwd() / 'scripts'))
import end_to_end_acceptance as acceptance
import rclpy, websocket


def map_snapshot(status, robot_id):
    status = status or {}
    local = ((status.get('local_active_maps') or {}).get(robot_id) or {})
    sync = ((status.get('robot_map_sync') or {}).get(robot_id) or {})
    return {
        'canonical_map_id': 'CANONICAL' if not local.get('local_active_map_id') else None,
        'canonical_revision': status.get('published_revision'),
        'active_map_id': local.get('active_map_id'),
        'active_map_revision': local.get('active_map_revision'),
        'local_active_map_id': local.get('local_active_map_id'),
        'local_active_map_revision': local.get('local_active_map_revision'),
        'ros_revision': status.get('ros_revision'),
        'gazebo_revision': status.get('gazebo_revision'),
        'nav2_revision': status.get('nav2_revision'),
        'tag_map_revision': status.get('tag_map_revision'),
        'robot_sync': sync,
        'tf_health': status.get('tf_status'),
        'map_sync_status': local.get('map_sync_status', status.get('map_sync_status')),
        'map_sync_error': status.get('map_sync_error'),
    }


def transform_pose(probe, parent, child):
    try:
        transform = probe.tf.lookup_transform(parent, child, rclpy.time.Time()).transform
        return (transform.translation.x, transform.translation.y,
                acceptance.yaw_from_quaternion(transform.rotation))
    except Exception:
        return None


def load_map_world_alignment(robot_id, map_name):
    """Bind the saved SLAM frame to the Gazebo spawn recorded for its session."""
    world_file = Path(os.environ['WORLD_FILE']).expanduser().resolve()
    manifest_path = world_file.parent.parent / 'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    robot = next((row for row in manifest.get('robots', [])
                  if str(row.get('id')) == robot_id), None)
    if robot is None:
        raise RuntimeError(f'{robot_id} has no Gazebo spawn pose in {manifest_path}')
    spawn = robot.get('pose')
    if not isinstance(spawn, list) or len(spawn) != 4:
        raise RuntimeError(f'{robot_id} has an invalid Gazebo spawn pose in {manifest_path}')
    world_anchor = (float(spawn[0]), float(spawn[1]), float(spawn[3]))

    mapping_evidence = Path('.runtime/mapping-browser-probe.json')
    mapping_run = json.loads(mapping_evidence.read_text())
    mapping_text = str(mapping_run.get('ui_mapping_text') or '')
    lines = mapping_text.splitlines()
    try:
        pose_line = lines[next(i for i, line in enumerate(lines)
            if 'ROBOT POSE' in line and 'map' in line) + 1].strip()
    except (StopIteration, IndexError) as exc:
        raise RuntimeError('the proven SLAM map-frame origin is missing from mapping evidence') from exc
    match = re.fullmatch(r'([-+]?\d+(?:\.\d+)?),\s*([-+]?\d+(?:\.\d+)?)\s*m\s*·\s*([-+]?\d+(?:\.\d+)?)\s*rad', pose_line)
    if not match:
        raise RuntimeError(f'could not parse initial SLAM map pose from {mapping_evidence}: {pose_line!r}')
    map_anchor = tuple(float(match.group(index)) for index in (1, 2, 3))
    map_events = mapping_run.get('maps') or []
    mapping_session_id = str((map_events[0] if map_events else {}).get('session') or '')
    if not mapping_session_id:
        raise RuntimeError('mapping evidence does not identify its SLAM session')

    artifact_root = manifest_path.parents[2]
    registry_path = artifact_root / 'local_robot_maps' / robot_id / 'registry.json'
    registry = json.loads(registry_path.read_text())
    record = next((row for row in registry
                   if row.get('name') == map_name and row.get('robot_id') == robot_id), None)
    if record is None:
        raise RuntimeError(f'{map_name} is not registered for {robot_id} in {registry_path}')
    if record.get('source_mapping_session_id') != mapping_session_id:
        raise RuntimeError('saved map and SLAM frame anchor come from different mapping sessions')
    return {
        'map_anchor_pose': map_anchor,
        'gazebo_world_anchor_pose': world_anchor,
        'mapping_session_id': mapping_session_id,
        'mapping_anchor_source': str(mapping_evidence),
        'world_anchor_source': str(manifest_path),
        'map_registry_source': str(registry_path),
        'saved_map_id': record.get('id'),
        'saved_map_revision': record.get('revision'),
    }


def map_pose_from_world(world_pose, map_anchor, world_anchor):
    angle = acceptance.wrap_angle(world_anchor[2] - map_anchor[2])
    dx, dy = world_pose[0] - world_anchor[0], world_pose[1] - world_anchor[1]
    return (
        map_anchor[0] + math.cos(angle) * dx + math.sin(angle) * dy,
        map_anchor[1] - math.sin(angle) * dx + math.cos(angle) * dy,
        acceptance.wrap_angle(world_pose[2] - angle),
    )


def world_pose_from_map(map_pose, map_anchor, world_anchor):
    angle = acceptance.wrap_angle(world_anchor[2] - map_anchor[2])
    dx, dy = map_pose[0] - map_anchor[0], map_pose[1] - map_anchor[1]
    return (
        world_anchor[0] + math.cos(angle) * dx - math.sin(angle) * dy,
        world_anchor[1] + math.sin(angle) * dx + math.cos(angle) * dy,
        acceptance.wrap_angle(map_pose[2] + angle),
    )


def save(output, evidence, result):
    output.write_text(json.dumps(result, indent=2))
    evidence.mkdir(parents=True, exist_ok=True)
    (evidence / 'observer-state.json').write_text(json.dumps(result, indent=2))


def main():
    backend = os.environ['BACKEND_URL'].rstrip('/')
    robot_id = os.environ.get('ROBOT_ID', 'R01')
    output = Path(os.environ.get('NAV_ACCEPTANCE_OUTPUT', '.runtime/resume-web-navigation.json'))
    map_name = os.environ.get('SAVED_MAP_NAME', 'slam_accumulated_20261001_01')
    alignment = load_map_world_alignment(robot_id, map_name)
    map_anchor = tuple(alignment['map_anchor_pose'])
    world_anchor = tuple(alignment['gazebo_world_anchor_pose'])
    evidence = Path(tempfile.mkdtemp(prefix='web-nav-resume-', dir='.runtime'))
    result = {'passed': False, 'source': 'Control Detail UI -> Django -> R01 -> Nav2 -> Gazebo'}
    result['map_world_alignment'] = alignment
    token = acceptance.authenticate(backend)
    url = backend.replace('https://', 'wss://').replace('http://', 'ws://') + '/ws?token=' + token
    rclpy.init()
    probe = acceptance.MotionProbe()
    ws = websocket.create_connection(url, timeout=5.0, enable_multithread=True)
    ws.settimeout(0.02)
    browser = None
    try:
        ready = probe.wait_until(lambda: probe.odom is not None and probe.gazebo_pose is not None
            and probe.sim_time() > 0 and probe.runtime_status is not None
            and robot_id in (probe.runtime_status.get('connected_robot_ids') or []), 30.0, ws)
        assert ready, 'ROS pose, clock or R01 bridge is unavailable'
        mode_ok, mode_reason = acceptance.set_mode(probe, ws, robot_id, 'AUTONOMOUS')
        assert mode_ok, f'AUTONOMOUS was not applied before Web navigation: {mode_reason}'
        active = map_snapshot(probe.runtime_status, robot_id)
        assert active['map_sync_status'] in ('CANONICAL', 'LOCAL_ONLY'), json.dumps(active)
        tf_ready = probe.wait_until(lambda: probe.map_pose() is not None, 15.0, ws)
        assert tf_ready, 'map -> base_footprint TF unavailable after bounded observer wait'
        idle = probe.observe_idle_cmd_vel(0.8)
        assert idle['selected_velocity_zero'] and idle['selected_owner'] == 'NONE', json.dumps(idle)
        pre_navigation_settle = probe.wait_mechanical_settling(ws, timeout=45, wall_timeout=180)
        assert pre_navigation_settle.get('passed') is True, json.dumps(pre_navigation_settle)
        result['map_state_startup_ready'] = active
        result['tf_pose_startup'] = probe.map_pose()
        result['idle_before_negative'] = idle
        result['mechanical_settle_before_negative'] = pre_navigation_settle

        # Safe backend enforcement case: missing preview token must reject before
        # any bridge command, and the independent observer checks there was no motion.
        negative_pose = probe.map_pose()
        gazebo_before_negative = probe.gazebo_pose
        selected_index = len(probe.selected_cmd_events)
        owner_index = len(probe.command_owner_events)
        error_index = len(probe.ws_messages)
        ws.send(json.dumps({'type': 'NAV_GOAL', 'robot_id': robot_id,
            'x': negative_pose[0], 'y': negative_pose[1], 'yaw': negative_pose[2],
            'frame_id': 'map', 'active_map_id': active['active_map_id'],
            'active_map_revision': active['active_map_revision']}))
        rejected = probe.wait_until(lambda: any(message.get('type') == 'ERROR'
            and message.get('code') == 'PATH_PREVIEW_REQUIRED'
            for message in probe.ws_messages[error_index:]), 5.0, ws)
        assert rejected, 'missing-preview Send Goal was not rejected by Django'
        negative_end = time.monotonic() + 1.5
        while time.monotonic() < negative_end:
            probe.pump(ws, 0.02)
        selected_negative = probe.selected_cmd_events[selected_index:]
        owners_negative = probe.command_owner_events[owner_index:]
        negative_displacement = acceptance.dist(gazebo_before_negative, probe.gazebo_pose)
        no_motion = (not any(any(abs(value) > 1e-6 for value in row[1:])
            for row in selected_negative)
            and negative_displacement is not None and negative_displacement < 0.01
            and all(owner != 'NAV2' for _, owner in owners_negative))
        result['preview_enforcement_negative'] = {
            'passed': bool(rejected and no_motion), 'case': 'NAV_GOAL missing preview_request_id',
            'error': next(message for message in probe.ws_messages[error_index:]
                if message.get('type') == 'ERROR' and message.get('code') == 'PATH_PREVIEW_REQUIRED'),
            'gazebo_displacement_m': negative_displacement,
            'selected_nonzero_samples': sum(any(abs(value) > 1e-6 for value in row[1:])
                for row in selected_negative), 'nav_owner_seen': any(owner == 'NAV2'
                    for _, owner in owners_negative)}
        assert result['preview_enforcement_negative']['passed'], json.dumps(result['preview_enforcement_negative'])

        manual_ok, manual_reason = acceptance.set_mode(probe, ws, robot_id, 'MANUAL', timeout=20.0)
        assert manual_ok, f'MANUAL was not applied before saved-map loading: {manual_reason}'
        result['manual_mode_before_map_load'] = {
            'passed': True, 'control_status': probe.control_statuses[-1] if probe.control_statuses else None,
        }
        pose_seed = probe.gazebo_pose
        assert pose_seed is not None, 'Gazebo ground-truth pose is unavailable before initial-pose setup'
        pose_seed_payload = {'x': pose_seed[0], 'y': pose_seed[1], 'yaw': pose_seed[2]}
        (evidence / 'pose-seed.json').write_text(json.dumps(pose_seed_payload))
        map_pose_seed = map_pose_from_world(pose_seed, map_anchor, world_anchor)
        map_pose_seed_payload = {'x': map_pose_seed[0], 'y': map_pose_seed[1], 'yaw': map_pose_seed[2]}
        (evidence / 'map-pose-seed.json').write_text(json.dumps(map_pose_seed_payload))
        spawn_distance = acceptance.dist(pose_seed, world_anchor)
        spawn_yaw_error = abs(acceptance.wrap_angle(pose_seed[2] - world_anchor[2]))
        if spawn_distance > 0.05 or spawn_yaw_error > 0.05:
            raise RuntimeError('the simulator must be reset to the SLAM session Gazebo anchor before local-map acceptance: ' +
                json.dumps({'gazebo_pose': pose_seed, 'world_anchor': world_anchor,
                    'xy_error_m': spawn_distance, 'yaw_error_rad': spawn_yaw_error}))
        result['initial_pose_before'] = {
            'gazebo_pose': pose_seed,
            'map_pose': probe.map_pose(),
            'projected_saved_map_pose': map_pose_seed,
            'odom_pose': probe.odom,
            'map_to_odom': transform_pose(probe, 'map', 'odom'),
            'odom_to_base': transform_pose(probe, 'odom', 'base_footprint'),
        }

        browser = subprocess.Popen(['node', 'scripts/local_control_web_nav_browser.cjs'],
            env=dict(os.environ, NAV_ACCEPTANCE_DIR=str(evidence.resolve()),
                NAV_GOAL_HEADING=str(map_pose_seed[2]),
                SAVED_MAP_NAME=map_name))
        preview_file = evidence / 'path-preview.json'
        deadline = time.monotonic() + 180
        while time.monotonic() < deadline and not preview_file.exists():
            probe.pump(ws, 0.04)
            if browser.poll() is not None:
                raise RuntimeError(f'Control Detail browser exited before valid preview ({browser.returncode})')
        assert preview_file.exists(), 'Control Detail did not produce a valid nearby path preview'
        preview_ui = json.loads(preview_file.read_text())
        preview = preview_ui['preview']['result']
        preview_goal = preview.get('goal') or {}
        preview_path_length_m = float(preview.get('path_length_m', math.nan))
        pose_set_file = evidence / 'pose-set.json'
        assert pose_set_file.is_file(), 'Control Detail did not confirm the requested initial pose through Django'
        pose_set = json.loads(pose_set_file.read_text())
        expected_pose = tuple(float(pose_set['pose'][axis]) for axis in ('x', 'y', 'yaw'))
        target_distance_m = math.hypot(float(preview_goal.get('x', math.nan)) - expected_pose[0],
            float(preview_goal.get('y', math.nan)) - expected_pose[1])
        pose_confirmed = probe.wait_until(lambda: (
            probe.map_pose() is not None
            and acceptance.dist(probe.map_pose(), expected_pose) <= 0.05
            and abs(acceptance.wrap_angle(probe.map_pose()[2] - expected_pose[2])) <= 0.05),
            15.0, ws)
        pre_nav_map_pose = probe.map_pose()
        pre_nav_gazebo_pose = probe.gazebo_pose
        pre_nav_error = (acceptance.dist(pre_nav_map_pose, expected_pose)
            if pre_nav_map_pose is not None else None)
        pre_nav_yaw_error = (abs(acceptance.wrap_angle(pre_nav_map_pose[2] - expected_pose[2]))
            if pre_nav_map_pose is not None else None)
        active_after_load = map_snapshot(probe.runtime_status, robot_id)
        local_map_loaded = (active_after_load['map_sync_status'] == 'LOCAL_ONLY'
            and bool(active_after_load['local_active_map_id'])
            and active_after_load['active_map_id'] == preview.get('active_map_id')
            and str(active_after_load['active_map_revision']) == str(preview.get('active_map_revision')))
        result['init_pose_request'] = {
            'passed': bool(pose_set.get('localization_owner') == 'ekf_v30e' and pose_confirmed),
            'expected_pose': expected_pose, 'localization_owner': pose_set.get('localization_owner'),
            'map_pose_after': pre_nav_map_pose,
            'gazebo_pose_at_request': pose_seed,
            'map_to_odom_after': transform_pose(probe, 'map', 'odom'),
            'odom_to_base_after': transform_pose(probe, 'odom', 'base_footprint'),
        }
        result['pre_nav_localization_error_m'] = pre_nav_error
        result['pre_nav_localization'] = {
            'passed': bool(pose_confirmed and local_map_loaded and pre_nav_error is not None
                and pre_nav_error <= 0.05 and pre_nav_yaw_error is not None
                and pre_nav_yaw_error <= 0.05),
            'map_pose': pre_nav_map_pose, 'expected_map_pose': expected_pose,
            'gazebo_pose': pre_nav_gazebo_pose,
            'projected_map_pose_from_gazebo': map_pose_from_world(
                pre_nav_gazebo_pose, map_anchor, world_anchor) if pre_nav_gazebo_pose else None,
            'odom_pose': probe.odom,
            'map_to_odom': transform_pose(probe, 'map', 'odom'),
            'odom_to_base': transform_pose(probe, 'odom', 'base_footprint'),
            'xy_error_m': pre_nav_error, 'yaw_error_rad': pre_nav_yaw_error,
            'loaded_local_map': local_map_loaded,
            'map_state': active_after_load,
        }
        if not result['pre_nav_localization']['passed']:
            raise RuntimeError('pre-navigation localization/map gate failed: ' +
                json.dumps(result['pre_nav_localization']))
        before_send = map_snapshot(probe.runtime_status, robot_id)
        preview_matches_state = (preview.get('active_map_id') == before_send['active_map_id']
            and str(preview.get('active_map_revision')) == str(before_send['active_map_revision'])
            and preview_ui['initial_map'] == preview_ui['before_send_map']
            and before_send['map_sync_status'] in ('CANONICAL', 'LOCAL_ONLY'))
        path_pass = bool(preview.get('status') == 'VALID'
            and len(preview.get('path') or []) > 0 and preview_matches_state
            and math.isfinite(target_distance_m) and 1.1 <= target_distance_m <= 1.35
            and math.isfinite(preview_path_length_m) and preview_path_length_m <= 3.0)
        result['map_state_before_preview'] = active_after_load
        result['path_preview_ui'] = preview_ui
        result['map_state_immediately_before_send'] = before_send
        result['path_preview'] = {'passed': path_pass,
            'request_id': preview.get('request_id'), 'status': preview.get('status'),
            'path_length_m': preview.get('path_length_m'), 'path_points': len(preview.get('path') or []),
            'active_map_id': preview.get('active_map_id'),
            'active_map_revision': preview.get('active_map_revision'),
            'target_distance_m': target_distance_m,
            'preview_path_length_m': preview_path_length_m,
            'nearby_distance_range_m': [1.1, 1.35],
            'nearby_path_limit_m': 3.0,
            'map_state_matches': preview_matches_state}
        assert path_pass, json.dumps(result['path_preview'])

        pose0, gazebo0 = probe.map_pose(), probe.gazebo_pose
        odom0 = probe.odom
        nav_index, selected_index = len(probe.nav_cmd_events), len(probe.selected_cmd_events)
        owner_index, drive_index = len(probe.command_owner_events), len(probe.drive_events)
        nav_status_index = len(probe.nav_statuses)
        (evidence / 'send-goal-ready.json').write_text('{}')
        sent_file = evidence / 'goal-sent.json'
        deadline = time.monotonic() + 45
        while time.monotonic() < deadline and not sent_file.exists():
            probe.pump(ws, 0.04)
            if browser.poll() is not None:
                raise RuntimeError(f'Control Detail browser exited before Send Goal ({browser.returncode})')
        assert sent_file.exists(), 'Control Detail did not emit NAV_GOAL after Send Goal click'
        sent = json.loads(sent_file.read_text())
        target = (float(sent['frame']['x']), float(sent['frame']['y']), float(sent['frame']['yaw']))
        assert sent['frame']['preview_request_id'] == preview.get('request_id')
        assert sent['frame']['active_map_id'] == preview.get('active_map_id')
        assert str(sent['frame']['active_map_revision']) == str(preview.get('active_map_revision'))

        nav_wall_timeout = float(os.environ.get('NAV_WALL_TIMEOUT_S', '600'))
        nav_sim_timeout = float(os.environ.get('NAV_SIM_TIMEOUT_S', '45'))
        nav_sim_start = probe.sim_time()
        nav_wall_start = time.monotonic()
        nav_gazebo_start = probe.gazebo_pose
        deadline = nav_wall_start + nav_wall_timeout
        terminal = None
        accepted = False
        watchdog_reason = None
        while time.monotonic() < deadline:
            probe.pump(ws, 0.04)
            status = acceptance.find_goal_status(probe, target[0], target[1], robot_id)
            if status:
                state = str(status.get('status') or '').upper()
                accepted = accepted or state in ('ACTIVE', 'NAVIGATING', 'SUCCEEDED')
                if state in ('SUCCEEDED', 'FAILED', 'CANCELED', 'EMERGENCY_STOPPED'):
                    terminal = status
                    break
            if probe.sim_time() - nav_sim_start >= nav_sim_timeout:
                watchdog_reason = f'simulation-time acceptance watchdog {nav_sim_timeout:g}s'
                break
            physical_distance = (acceptance.dist(nav_gazebo_start, probe.gazebo_pose)
                if nav_gazebo_start is not None and probe.gazebo_pose is not None else None)
            if physical_distance is not None and physical_distance > 2.0:
                watchdog_reason = f'safe nearby-goal displacement cap exceeded: {physical_distance:.3f}m'
                break
            if browser.poll() is not None:
                raise RuntimeError(f'Control Detail browser exited during navigation ({browser.returncode})')
        if terminal is None:
            ws.send(json.dumps({'type': 'NAV_CANCEL', 'robot_id': robot_id}))
            terminal = {'status': 'TIMEOUT', 'reason': watchdog_reason or
                f'no terminal NAV_STATUS within {nav_wall_timeout:g} wall seconds'}

        terminal_pose1 = probe.map_pose()
        terminal_gazebo1 = probe.gazebo_pose
        terminal_odom1 = probe.odom
        nav_rows = probe.nav_cmd_events[nav_index:]
        selected_rows = probe.selected_cmd_events[selected_index:]
        owner_rows = probe.command_owner_events[owner_index:]
        drive_rows = probe.drive_events[drive_index:]
        nav_active = [row for row in nav_rows if any(abs(value) > 1e-4 for value in row[1:])]
        selected_active = [row for row in selected_rows if any(abs(value) > 1e-4 for value in row[1:])]
        nav_owner = any(owner == 'NAV2' for _, owner in owner_rows)
        drive_active = any(any(abs(value) > 1e-4 for value in row[1:]) for row in drive_rows)
        displacement = acceptance.dist(pose0, terminal_pose1) if terminal_pose1 else None
        gazebo_displacement = acceptance.dist(gazebo0, terminal_gazebo1) if terminal_gazebo1 else None
        odom_displacement = acceptance.dist(odom0, terminal_odom1) if terminal_odom1 else None
        target_world = world_pose_from_map(target, map_anchor, world_anchor)
        map_goal_xy_error = acceptance.dist(terminal_pose1, target) if terminal_pose1 else None
        goal_xy_error = acceptance.dist(terminal_gazebo1, target_world) if terminal_gazebo1 else None
        map_goal_yaw_error = (abs(acceptance.wrap_angle(terminal_pose1[2] - target[2]))
            if terminal_pose1 else None)
        goal_yaw_error = (abs(acceptance.wrap_angle(terminal_gazebo1[2] - target_world[2]))
            if terminal_gazebo1 else None)
        result['send_goal_validation'] = {'map_state_at_backend_acceptance': before_send,
            'preview_request_id': sent['frame']['preview_request_id'],
            'active_map_id': sent['frame']['active_map_id'],
            'active_map_revision': sent['frame']['active_map_revision'],
            'accepted_by_nav2': accepted, 'terminal': terminal}
        navigation_chain_passed = bool(terminal.get('status') == 'SUCCEEDED' and accepted and nav_active
                and selected_active and nav_owner and drive_active and displacement is not None
                and displacement > acceptance.TRANSLATION_TOLERANCE_M
                and gazebo_displacement is not None and gazebo_displacement > acceptance.TRANSLATION_TOLERANCE_M
                and odom_displacement is not None and odom_displacement > acceptance.TRANSLATION_TOLERANCE_M / 2)
        result['web_navigation'] = {
            # Nav2 terminal status and measured goal pose can arrive before
            # Gazebo wheel/chassis settling. Apply strict pose tolerances only
            # to the fresh stationary post-settle sample below.
            'passed': False, 'command_chain_passed': navigation_chain_passed,
            'accepted': accepted, 'result': terminal.get('status'), 'reason': terminal.get('reason'),
            'start_map_pose': pose0, 'start_gazebo_pose': gazebo0, 'start_odom_pose': odom0,
            'goal_pose': target, 'goal_gazebo_world_pose': target_world,
            'path_length_m': preview.get('path_length_m'),
            'nav2_terminal_map_pose': terminal_pose1,
            'nav2_terminal_gazebo_pose': terminal_gazebo1,
            'nav2_terminal_odom_pose': terminal_odom1,
            'final_map_pose': terminal_pose1, 'final_gazebo_pose': terminal_gazebo1,
            'final_odom_pose': terminal_odom1,
            'gazebo_displacement_m': gazebo_displacement, 'odom_displacement_m': odom_displacement,
            'map_frame_xy_error_m': map_goal_xy_error,
            'map_frame_yaw_error_rad': map_goal_yaw_error,
            'gazebo_ground_truth_error_m': goal_xy_error,
            'xy_error_m': goal_xy_error, 'yaw_error_rad': goal_yaw_error,
            'nav_cmd_nonzero_samples': len(nav_active), 'selected_nonzero_samples': len(selected_active),
            'nav_owner_seen': nav_owner, 'drive_nonzero_seen': drive_active,
            'action_status_samples': len(probe.action_status_events),
            'sim_elapsed_s': probe.sim_time() - nav_sim_start,
            'wall_elapsed_s': time.monotonic() - nav_wall_start,
            'dwb_cmd_samples': [{'wall_s': round(row[0] - nav_wall_start, 3),
                'vx': row[1], 'vy': row[2], 'wz': row[3]}
                for row in nav_rows[::max(1, len(nav_rows) // 80)]],
            'owner_samples': [{'wall_s': round(stamp - nav_wall_start, 3), 'owner': owner}
                for stamp, owner in owner_rows[::max(1, len(owner_rows) // 80)]],
            'web_nav_status_samples': len(probe.nav_statuses) - nav_status_index}
        def stopped_after_goal():
            if not probe.selected_cmd_events or not probe.command_owner_events:
                return False
            selected = probe.selected_cmd_events[-1]
            owner = probe.command_owner_events[-1]
            return (all(abs(value) <= 1e-6 for value in selected[1:])
                and owner[1] == 'NONE')
        stopped = probe.wait_until(stopped_after_goal, 12.0, ws)
        settling = (probe.wait_mechanical_settling(ws, timeout=45, wall_timeout=180)
            if stopped else {'passed': False, 'reason': 'selected command or owner did not stop'})
        result['web_navigation']['stop_after_result'] = stopped
        result['web_navigation']['settling_after_goal'] = settling
        if stopped and settling.get('passed') is True:
            # Preserve the Nav2 terminal pose above, then report final errors
            # from fresh stationary samples after mechanical settling.
            stable_end = time.monotonic() + 1.0
            while time.monotonic() < stable_end:
                probe.pump(ws, 0.04)
            stable_map, stable_gazebo, stable_odom = probe.map_pose(), probe.gazebo_pose, probe.odom
            result['web_navigation']['final_map_pose'] = stable_map
            result['web_navigation']['final_gazebo_pose'] = stable_gazebo
            result['web_navigation']['final_odom_pose'] = stable_odom
            result['web_navigation']['map_frame_xy_error_m'] = (
                acceptance.dist(stable_map, target) if stable_map else None)
            result['web_navigation']['map_frame_yaw_error_rad'] = (
                abs(acceptance.wrap_angle(stable_map[2] - target[2])) if stable_map else None)
            result['web_navigation']['gazebo_ground_truth_error_m'] = (
                acceptance.dist(stable_gazebo, target_world) if stable_gazebo else None)
            result['web_navigation']['xy_error_m'] = result['web_navigation']['gazebo_ground_truth_error_m']
            result['web_navigation']['yaw_error_rad'] = (
                abs(acceptance.wrap_angle(stable_gazebo[2] - target_world[2])) if stable_gazebo else None)
            result['web_navigation']['final_pose_sampled_after_settle'] = True
            result['web_navigation']['map_frame_yaw_within_0_05_rad'] = (
                result['web_navigation']['map_frame_yaw_error_rad'] is not None
                and result['web_navigation']['map_frame_yaw_error_rad'] <= 0.05)
            result['web_navigation']['gazebo_yaw_within_0_05_rad'] = (
                result['web_navigation']['yaw_error_rad'] is not None
                and result['web_navigation']['yaw_error_rad'] <= 0.05)
            result['web_navigation']['passed'] = bool(
                result['web_navigation']['command_chain_passed']
                and result['web_navigation']['map_frame_xy_error_m'] is not None
                and result['web_navigation']['map_frame_xy_error_m'] <= acceptance.NAV_GOAL_XY_TOLERANCE_M
                and result['web_navigation']['gazebo_ground_truth_error_m'] is not None
                and result['web_navigation']['gazebo_ground_truth_error_m'] <= acceptance.NAV_GOAL_XY_TOLERANCE_M
                and result['web_navigation']['xy_error_m'] is not None
                and result['web_navigation']['xy_error_m'] <= acceptance.NAV_GOAL_XY_TOLERANCE_M)
        result['web_navigation']['passed'] = (result['web_navigation']['passed']
            and stopped and settling.get('passed') is True)
        result['map_state_after_backend_validation'] = map_snapshot(probe.runtime_status, robot_id)
        result['path_preview_enforcement'] = result['preview_enforcement_negative']['passed'] and path_pass
        result['passed'] = result['path_preview_enforcement'] and result['web_navigation']['passed']
        (evidence / 'nav-finished.json').write_text(json.dumps({'passed': result['passed']}))
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline and browser.poll() is None:
            probe.pump(ws, 0.04)
        if browser.poll() is None:
            browser.terminate()
            browser.wait(timeout=5)
        result['browser_exit_code'] = browser.returncode
        result['browser_result'] = json.loads((evidence / 'browser-result.json').read_text()) if (evidence / 'browser-result.json').exists() else None
        result['evidence_dir'] = str(evidence)
    except Exception as error:
        result['reason'] = f'{type(error).__name__}: {error}'
        sent_file = evidence / 'goal-sent.json'
        if sent_file.is_file() and not (evidence / 'nav-finished.json').is_file():
            try:
                sent = json.loads(sent_file.read_text())
                frame = sent.get('frame') or {}
                target_x, target_y = float(frame['x']), float(frame['y'])
                current = acceptance.find_goal_status(probe, target_x, target_y, robot_id)
                current_state = str((current or {}).get('status') or '').upper()
                if current_state not in ('SUCCEEDED', 'FAILED', 'CANCELED', 'CANCELLED', 'EMERGENCY_STOPPED'):
                    ws.send(json.dumps({'type': 'NAV_CANCEL', 'robot_id': robot_id}))
                cancel_deadline = time.monotonic() + 20.0
                while time.monotonic() < cancel_deadline:
                    probe.pump(ws, 0.04)
                    current = acceptance.find_goal_status(probe, target_x, target_y, robot_id)
                    current_state = str((current or {}).get('status') or '').upper()
                    owner = probe.command_owner_events[-1][1] if probe.command_owner_events else None
                    selected = probe.selected_cmd_events[-1][1:] if probe.selected_cmd_events else None
                    if (current_state in ('SUCCEEDED', 'FAILED', 'CANCELED', 'CANCELLED', 'EMERGENCY_STOPPED')
                            and owner == 'NONE' and selected is not None
                            and all(abs(value) <= 1e-6 for value in selected)):
                        break
                result['safety_cancel_after_observer_error'] = {
                    'terminal_status': current_state,
                    'command_owner': owner if 'owner' in locals() else None,
                    'selected_velocity': selected if 'selected' in locals() else None,
                    'stopped': bool('owner' in locals() and owner == 'NONE'
                        and selected is not None and all(abs(value) <= 1e-6 for value in selected)),
                }
            except Exception as cancel_error:
                result['safety_cancel_after_observer_error'] = {
                    'error': f'{type(cancel_error).__name__}: {cancel_error}',
                }
        if browser is not None and browser.poll() is None:
            browser.terminate()
            try: browser.wait(timeout=3)
            except subprocess.TimeoutExpired: browser.kill(); browser.wait(timeout=3)
        print(result['reason'], flush=True)
        raise
    finally:
        save(output, evidence, result)
        ws.close()
        rclpy.shutdown()
        probe.destroy_node()
        if browser is not None and browser.poll() is None:
            browser.terminate()
            try: browser.wait(timeout=3)
            except subprocess.TimeoutExpired: browser.kill(); browser.wait(timeout=3)
    print('PATH_PREVIEW=' + ('PASS' if result.get('path_preview', {}).get('passed') else 'FAIL'), flush=True)
    print('PATH_PREVIEW_ENFORCEMENT=' + ('PASS' if result.get('path_preview_enforcement') else 'FAIL'), flush=True)
    print('WEB_NAV_GOAL_R01=' + ('PASS' if result.get('web_navigation', {}).get('passed') else 'FAIL'), flush=True)
    print('STAGE_3=' + ('PASS' if result['passed'] else 'FAIL'), flush=True)
    print('ACCEPTANCE_EVIDENCE=' + str(output), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
