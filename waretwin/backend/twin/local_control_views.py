from __future__ import annotations

import json
import fcntl
import math
import os
import threading
import time
import uuid
from types import SimpleNamespace
from pathlib import Path

from asgiref.sync import async_to_sync
from django.conf import settings
from django.db import transaction
from django.utils.module_loading import import_string
from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt
from django.views.decorators.http import require_http_methods

from .local_control import (
    get_robot_map, get_robot_slam_session, list_robot_maps, map_output_prefix,
    register_saved_map, slam_map_restoration_evidence,
    validate_map_name,
)
from .models import RobotVda5050Configuration
from .runtime import runtime
from .views import _body, _error, api_user_required
from .vda5050 import (
    apply_configuration, decrypt_password, encrypt_password, ensure_configuration_active,
    public_configuration, test_connection, validate_configuration,
)

_MODE_SWITCH_LOCK = threading.Lock()


def _mode_files():
    directory = Path(settings.WARETWIN_STACK_RUNTIME_DIR).resolve()
    return directory / 'mode-switch-request.json', directory / 'mode-switch-status.json'


def _read_mode_status(robot_id: str) -> dict:
    _request_path, status_path = _mode_files()
    try:
        status = json.loads(status_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return {'robot_id': robot_id, 'status': 'UNAVAILABLE', 'mode': None, 'message': None}
    if not isinstance(status, dict) or str(status.get('robot_id') or '') != robot_id:
        return {'robot_id': robot_id, 'status': 'UNAVAILABLE', 'mode': None, 'message': None}
    return status


def _write_mode_request(robot_id: str, mode: str, *, slam_session_file: str | None = None) -> dict:
    request_path, status_path = _mode_files()
    request_id = uuid.uuid4().hex
    request = {'request_id': request_id, 'robot_id': robot_id, 'mode': mode}
    if slam_session_file:
        request['slam_session_file'] = str(slam_session_file)
        request['force_restart'] = True
    request_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = request_path.with_suffix(request_path.suffix + '.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        os.chmod(temporary, 0o600)
        json.dump(request, stream, separators=(',', ':'))
        stream.flush()
        os.fsync(stream.fileno())
    # The queued launch argument contains an absolute robot-local session
    # prefix. Keep it in the mode-request file for the supervisor only; the
    # status file and API response are operator-facing and must not expose it.
    status = {
        'request_id': request_id, 'robot_id': robot_id, 'mode': mode,
        'status': 'REQUESTED', 'message': 'waiting for the ROS stack supervisor',
    }
    status_tmp = status_path.with_suffix(status_path.suffix + '.tmp')
    try:
        with status_tmp.open('w', encoding='utf-8') as stream:
            os.chmod(status_tmp, 0o600)
            json.dump(status, stream, separators=(',', ':'))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(status_tmp, status_path)
        os.replace(temporary, request_path)
    finally:
        temporary.unlink(missing_ok=True)
        status_tmp.unlink(missing_ok=True)
    return status


class SimulationRuntimeAdapter:
    """Mode transitions for the owned simulation stack supervisor."""

    def get_mode_status(self, robot_id: str) -> dict:
        return _read_mode_status(robot_id)

    def request_mode_change(self, robot_id: str, target: str, *,
                            slam_session_file: str | None = None) -> dict:
        with _MODE_SWITCH_LOCK:
            request_path, _status_path = _mode_files()
            lock_path = request_path.with_suffix('.lock')
            lock_fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o600)
            try:
                fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                os.close(lock_fd)
                return {'ok': False, 'http_status': 409,
                        'message': 'a runtime mode transition request is already being queued'}
            try:
                if request_path.exists():
                    return {'ok': False, 'http_status': 409,
                            'message': 'a runtime mode transition request is already queued'}
                status = _read_mode_status(robot_id)
                if status.get('status') == 'UNAVAILABLE':
                    return {'ok': False, 'http_status': 503,
                            'message': 'runtime supervisor status is unavailable'}
                if status.get('status') in ('STARTING', 'REQUESTED', 'RESTARTING', 'ROLLING_BACK'):
                    return {'ok': False, 'http_status': 409,
                            'message': 'a runtime mode transition is already in progress'}
                status = _write_mode_request(
                    robot_id, target.lower(), slam_session_file=slam_session_file)
            finally:
                fcntl.flock(lock_fd, fcntl.LOCK_UN)
                os.close(lock_fd)
        return {'ok': True, **status,
                'message': 'simulation runtime transition queued; readiness and rollback are managed by its supervisor'}


class RealRobotRuntimeAdapter:
    """Delegate transitions to a deployment-owned physical robot manager."""

    def _configured_adapter(self):
        entry_point = getattr(settings, 'WARETWIN_REAL_RUNTIME_ADAPTER', '')
        if not entry_point:
            return None
        try:
            return import_string(entry_point)()
        except Exception as exc:
            raise RuntimeError(f'physical robot runtime adapter could not load: {type(exc).__name__}') from exc

    def get_mode_status(self, robot_id: str) -> dict:
        try:
            adapter = self._configured_adapter()
            if adapter is None:
                return {'robot_id': robot_id, 'status': 'UNAVAILABLE', 'mode': None,
                        'message': 'no physical robot process adapter is configured'}
            result = adapter.get_mode_status(robot_id)
            if isinstance(result, dict):
                return result
        except Exception as exc:
            return {'robot_id': robot_id, 'status': 'ERROR', 'mode': None,
                    'message': f'physical robot runtime status failed: {type(exc).__name__}'}
        return {'robot_id': robot_id, 'status': 'ERROR', 'mode': None,
                'message': 'physical robot runtime adapter returned invalid status'}

    def request_mode_change(self, robot_id: str, target: str) -> dict:
        try:
            adapter = self._configured_adapter()
        except RuntimeError as exc:
            return {'ok': False, 'http_status': 502, 'code': 'REAL_RUNTIME_ADAPTER_FAILED',
                    'message': str(exc)}
        if adapter is None:
            return {
                'ok': False, 'http_status': 501,
                'code': 'REAL_RUNTIME_ADAPTER_UNAVAILABLE',
                'message': 'no physical robot process adapter is configured; no Gazebo or unmanaged ROS processes were started',
            }
        try:
            result = adapter.request_mode_change(robot_id, target)
        except Exception as exc:
            return {'ok': False, 'http_status': 502,
                    'code': 'REAL_RUNTIME_ADAPTER_FAILED',
                    'message': f'physical robot runtime adapter failed: {type(exc).__name__}'}
        return result if isinstance(result, dict) else {
            'ok': False, 'http_status': 502, 'code': 'REAL_RUNTIME_ADAPTER_FAILED',
            'message': 'physical robot runtime adapter returned an invalid result',
        }


def _runtime_adapter(runtime_mode: str):
    if runtime_mode == 'GAZEBO_ROS':
        return SimulationRuntimeAdapter()
    if runtime_mode == 'REAL_ROBOT':
        return RealRobotRuntimeAdapter()
    return None


def _robot_available(robot_id: str):
    if not runtime.is_external:
        return _error('local robot control requires an active ROS robot runtime', 503)
    if not runtime.robot_bridge_online(robot_id):
        return _error(f'authenticated ROS bridge for {robot_id} is offline', 503)
    return None


def _clear_pending_local_map_load(robot_id: str) -> None:
    getattr(runtime, 'pending_local_map_loads', {}).pop(robot_id, None)
    runtime.local_map_transitions.discard(robot_id)


def _clear_pending_slam_session_resume(robot_id: str) -> None:
    getattr(runtime, 'pending_slam_session_resumes', {}).pop(robot_id, None)
    runtime.local_map_transitions.discard(robot_id)


def _slam_session_resume_response(robot_id: str, record: dict, pending: dict,
                                  transition: dict, *, status: str = 'TRANSITIONING',
                                  evidence: dict | None = None):
    public_transition = ({key: transition[key] for key in (
        'ok', 'request_id', 'robot_id', 'mode', 'status', 'message') if key in transition}
        if isinstance(transition, dict) else None)
    return JsonResponse({
        'ok': True, 'status': status, 'robot_id': robot_id,
        'map': record, 'map_id': record['id'],
        'request_id': pending['request_id'], 'transition': public_transition,
        'mapping_state': 'MAPPING' if status == 'RESUMED' else 'RESTORING_SESSION',
        'restore_evidence': evidence,
        'message': ('SLAM Toolbox is restoring the saved pose graph before it accepts new scans.'
                    if status == 'TRANSITIONING' else
                    'The prior SLAM map is restored and live mapping has resumed.'),
    }, status=202 if status == 'TRANSITIONING' else 200)


def _require_manual_stopped(robot_id: str):
    robot = runtime.engine.state.get('robots', {}).get(robot_id)
    if not robot:
        return _error(f'no live robot state is available for {robot_id}', 409)
    if str(robot.get('control_mode') or '').upper() != 'MANUAL':
        return _error('switch the robot to MANUAL before changing its map or initial pose', 409)
    navigation_state = str(robot.get('navigation_state') or '').upper()
    if navigation_state in ('EMERGENCY_STOPPED', 'NAVIGATING', 'ACTIVE', 'PENDING', 'PAUSED'):
        return _error('stop the active navigation goal before changing the map or initial pose', 409)
    velocities = [robot.get('vx', 0.0), robot.get('vy', 0.0), robot.get('wz', 0.0)]
    try:
        measured = [float(value) for value in velocities]
    except (TypeError, ValueError):
        return _error('measured robot velocity is unavailable; stop the robot first', 409)
    if not all(math.isfinite(value) and abs(value) <= 0.05 for value in measured):
        return _error('stop the robot before changing its map or initial pose', 409)
    return None


def _bridge_request(robot_id: str, operation: str, payload: dict | None = None,
                    timeout: float = 20.0) -> dict:
    return async_to_sync(runtime.gateway().request_control)(
        robot_id, operation, payload or {}, timeout=timeout)


def _response_for_bridge_result(result: dict):
    if not result.get('ok'):
        return _error(str(result.get('error') or 'robot operation failed'), 502,
                      {'operation': result.get('operation')})
    return None


def _active_map_geometry(robot_id: str) -> dict | None:
    active = runtime.active_map_state(robot_id)
    geometry = runtime.robot_map_geometry.get(robot_id)
    if (geometry and geometry.get('active_map_id') == active.get('active_map_id')
            and str(geometry.get('active_map_revision') or '') == str(active.get('active_map_revision') or '')):
        return geometry
    local_id = active.get('local_active_map_id')
    if local_id:
        try:
            record, _yaml_path, _image_path = get_robot_map(robot_id, local_id)
        except (FileNotFoundError, ValueError):
            return None
        if record.get('width') and record.get('height'):
            return {
                'width': record['width'], 'height': record['height'],
                'resolution': record['resolution'],
                'origin': {'x': record['origin'][0], 'y': record['origin'][1],
                           'yaw': record['origin'][2]},
                'active_map_id': local_id,
                'active_map_revision': record['revision'],
            }
    return None


def _pose_within_active_map(robot_id: str, pose: dict[str, float]) -> bool:
    geometry = _active_map_geometry(robot_id)
    if not geometry:
        return False
    try:
        width = int(geometry['width']) * float(geometry['resolution'])
        height = int(geometry['height']) * float(geometry['resolution'])
        origin = geometry['origin']
        dx, dy = pose['x'] - float(origin['x']), pose['y'] - float(origin['y'])
        yaw = float(origin['yaw'])
        grid_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        grid_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        return 0.0 <= grid_x < width and 0.0 <= grid_y < height
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


@csrf_exempt
@api_user_required
@require_http_methods(['GET'])
def local_maps(request, robot_id: str):
    problem = _robot_available(robot_id)
    if problem:
        return problem
    try:
        maps = list_robot_maps(robot_id)
    except ValueError as exc:
        return _error(str(exc), 500)
    active_map = runtime.active_map_state(robot_id)
    robot = runtime.engine.state.get('robots', {}).get(robot_id, {})
    measured = []
    try:
        measured = [float(robot.get(axis, 0.0)) for axis in ('vx', 'vy', 'wz')]
    except (TypeError, ValueError):
        measured = []
    return JsonResponse({
        'robot_id': robot_id,
        'maps': maps,
        'runtime_mode': runtime.operation_mode,
        'mapping_state': getattr(runtime, 'robot_mapping_state', {}).get(robot_id, 'UNKNOWN'),
        'mapping_duration_s': getattr(runtime, 'robot_mapping_elapsed_s', {}).get(robot_id, 0.0),
        'robot_control_mode': str(robot.get('control_mode') or '').upper(),
        'robot_stopped': bool(len(measured) == 3 and all(math.isfinite(value) and abs(value) <= 0.05 for value in measured)),
        **active_map,
        'active_local_map_id': active_map['local_active_map_id'],
        'map_sync_status': active_map['map_sync_status'],
    })


@csrf_exempt
@api_user_required
@require_http_methods(['GET', 'POST'])
def local_runtime_mode(request, robot_id: str):
    adapter = _runtime_adapter(runtime.runtime_mode)
    if request.method == 'GET':
        transition = adapter.get_mode_status(robot_id) if adapter else {
            'robot_id': robot_id, 'status': 'UNAVAILABLE', 'mode': None,
            'message': f'no runtime adapter is available for {runtime.runtime_mode}',
        }
        pending = getattr(runtime, 'pending_local_map_loads', {}).get(robot_id)
        if (pending and pending.get('request_id')
                and transition.get('request_id') == pending['request_id']
                and transition.get('status') in ('ROLLED_BACK', 'ERROR')):
            _clear_pending_local_map_load(robot_id)
        return JsonResponse({
            'robot_id': robot_id,
            'current_mode': runtime.operation_mode,
            'transition': transition,
            'pending_local_map_id': pending.get('map_id') if pending else None,
        })
    if not runtime.is_external:
        return _error('mapping/navigation transition requires a connected robot runtime', 409)
    if not runtime.robot_bridge_online(robot_id):
        return _error(f'authenticated ROS bridge for {robot_id} is offline', 503)
    target = str(_body(request).get('mode') or '').strip().upper()
    if target not in ('MAPPING', 'NAVIGATION'):
        return _error('mode must be MAPPING or NAVIGATION')
    if robot_id in runtime.local_map_transitions:
        return _error('runtime mode cannot change during a saved local map load', 409)
    current = str(runtime.operation_mode).upper()
    if current not in ('MAPPING', 'NAVIGATION'):
        return _error(f'cannot switch runtime mode from {current}', 409)
    if target == current:
        return JsonResponse({'ok': True, 'robot_id': robot_id, 'current_mode': current,
                             'status': 'READY', 'message': 'requested mode is already active'})
    robot = runtime.engine.state.get('robots', {}).get(robot_id, {})
    if str(robot.get('control_mode') or '').upper() != 'MANUAL' or str(robot.get('navigation_state') or '').upper() != 'MANUAL':
        return _error('switch to MANUAL mode and stop the robot before changing SLAM/Nav2 runtime mode', 409)
    adapter = _runtime_adapter(runtime.runtime_mode)
    if adapter is None:
        return _error(f'no runtime adapter is available for {runtime.runtime_mode}', 409)
    state = adapter.get_mode_status(robot_id)
    if state.get('status') in ('STARTING', 'REQUESTED', 'RESTARTING', 'ROLLING_BACK'):
        return _error('a runtime mode transition is already in progress', 409)
    result = adapter.request_mode_change(robot_id, target)
    if not result.get('ok'):
        return _error(str(result.get('message') or 'runtime adapter rejected the transition'),
                      int(result.get('http_status') or 502), {'code': result.get('code')})
    status = result
    return JsonResponse({
        'ok': True, 'robot_id': robot_id, 'current_mode': current,
        'requested_mode': target, 'request_id': status['request_id'],
        'status': status['status'],
        'message': status.get('message'),
    }, status=202 if status.get('status') in ('REQUESTED', 'RESTARTING') else 200)


@csrf_exempt
@api_user_required
@require_http_methods(['POST'])
def mapping_command(request, robot_id: str, action: str):
    if action not in ('start', 'stop'):
        return _error('unsupported mapping action', 404)
    problem = _robot_available(robot_id)
    if problem:
        return problem
    if runtime.operation_mode != 'MAPPING':
        return _error(
            'SLAM Toolbox is not active in this runtime. Select mapping mode when starting the ROS stack; '
            'the current launch keeps SLAM and Nav2 mutually exclusive.', 409,
            {'runtime_mode': runtime.operation_mode},
        )
    if robot_id in getattr(runtime, 'pending_slam_session_resumes', {}):
        return _error('mapping controls are blocked while a saved SLAM session is restoring', 409)
    operation = 'MAPPING_START' if action == 'start' else 'MAPPING_STOP'
    try:
        result = _bridge_request(robot_id, operation, timeout=8.0)
    except Exception as exc:
        return _error(f'robot mapping transition failed: {type(exc).__name__}', 502)
    failure = _response_for_bridge_result(result)
    if failure:
        return failure
    state = 'MAPPING' if action == 'start' else 'PAUSED'
    runtime.robot_mapping_state[robot_id] = state
    return JsonResponse({'ok': True, 'robot_id': robot_id, 'mapping_state': state,
                         'runtime_result': result.get('result')})


@csrf_exempt
@api_user_required
@require_http_methods(['POST'])
def save_robot_map(request, robot_id: str):
    problem = _robot_available(robot_id)
    if problem:
        return problem
    if runtime.operation_mode != 'MAPPING':
        return _error('map save requires the active SLAM mapping runtime', 409)
    if robot_id in getattr(runtime, 'pending_slam_session_resumes', {}):
        return _error('map save is blocked until the saved SLAM session has been verified', 409)
    if str(getattr(runtime, 'robot_mapping_state', {}).get(robot_id) or '').upper() != 'PAUSED':
        return _error('stop mapping before saving so the accumulated map and serialized pose graph are consistent', 409)
    try:
        name = validate_map_name(_body(request).get('name'))
        prefix = map_output_prefix(robot_id, name)
    except ValueError as exc:
        return _error(str(exc))
    session_prefix = prefix.parent / f'{prefix.name}_slam_session'
    output_files = (
        prefix.with_suffix('.yaml'), prefix.with_suffix('.pgm'),
        session_prefix.with_suffix('.posegraph'), session_prefix.with_suffix('.data'),
    )
    if (any(row.get('name') == name for row in list_robot_maps(robot_id))
            or any(path.exists() for path in output_files)):
        return _error('a map with this name already exists for this robot', 409)
    saved_map = getattr(runtime, 'robot_map_snapshots', {}).get(robot_id, {})
    mapping = saved_map.get('map') if isinstance(saved_map, dict) else None
    session_id = str(getattr(runtime, 'robot_mapping_sessions', {}).get(robot_id) or '')
    try:
        known_cells = int((mapping or {}).get('known_cells') or 0)
    except (TypeError, ValueError, OverflowError):
        known_cells = 0
    if (not isinstance(mapping, dict) or mapping.get('map_source') != 'SLAM_TOOLBOX'
            or not session_id or mapping.get('mapping_session_id') != session_id
            or known_cells <= 0):
        return _error('save requires a live accumulated /map from the current SLAM Toolbox session', 409)
    try:
        result = _bridge_request(robot_id, 'MAP_SAVE', {
            'name': name, 'output_prefix': str(prefix),
            'session_output_prefix': str(session_prefix),
        }, timeout=45.0)
    except Exception as exc:
        return _error(f'map saver request failed: {type(exc).__name__}', 502)
    failure = _response_for_bridge_result(result)
    if failure:
        return failure
    try:
        bridge_result = result.get('result') if isinstance(result.get('result'), dict) else {}
        record = register_saved_map(
            robot_id, name, prefix.with_suffix('.yaml'),
            mapping_metadata=mapping,
            slam_session=bridge_result.get('slam_session'),
        )
    except FileExistsError as exc:
        return _error(str(exc), 409)
    except (OSError, ValueError) as exc:
        return _error(f'map saver returned without a valid persisted map: {exc}', 502)
    return JsonResponse({'ok': True, 'robot_id': robot_id, 'map': record})


@csrf_exempt
@api_user_required
@require_http_methods(['POST'])
def load_robot_map(request, robot_id: str):
    problem = _robot_available(robot_id)
    if problem:
        return problem
    stopped = _require_manual_stopped(robot_id)
    if stopped:
        return stopped
    if robot_id in getattr(runtime, 'pending_slam_session_resumes', {}):
        return _error('navigation-map loading is blocked while a saved SLAM session is restoring', 409)
    map_id = str(_body(request).get('map_id') or '').strip()
    try:
        record, yaml_path, image_path = get_robot_map(robot_id, map_id)
    except (FileNotFoundError, ValueError) as exc:
        return _error(str(exc), 404)
    pending_loads = runtime.pending_local_map_loads
    pending = pending_loads.get(robot_id)
    if pending and pending.get('map_id') != map_id:
        return _error('another saved local map is still loading for this robot', 409)

    mode = str(runtime.operation_mode or '').upper()
    if mode == 'MAPPING':
        if pending:
            adapter = _runtime_adapter(runtime.runtime_mode)
            status = adapter.get_mode_status(robot_id) if adapter else {}
            if (status.get('request_id') == pending.get('request_id')
                    and status.get('status') in ('ROLLED_BACK', 'ERROR')):
                _clear_pending_local_map_load(robot_id)
                return _error(status.get('message') or 'navigation runtime transition failed', 409)
            return JsonResponse({
                'ok': True, 'status': 'TRANSITIONING', 'robot_id': robot_id,
                'active_map': record, 'active_map_id': map_id,
                'active_map_revision': record['revision'],
                'request_id': pending.get('request_id'),
                'transition': status,
                'mapping_state': 'PAUSED',
                'message': 'SLAM is paused; waiting for the runtime supervisor to start Nav2.',
            }, status=202)

        adapter = _runtime_adapter(runtime.runtime_mode)
        if adapter is None:
            return _error(f'no runtime adapter is available for {runtime.runtime_mode}', 409)
        transition = adapter.get_mode_status(robot_id)
        if transition.get('status') != 'READY' or str(transition.get('mode') or '').lower() != 'mapping':
            return _error('mapping runtime is not ready for a supervised navigation handoff', 409,
                          {'transition': transition})

        runtime.local_map_transitions.add(robot_id)
        try:
            stopped_slam = _bridge_request(robot_id, 'MAPPING_STOP', timeout=8.0)
        except Exception as exc:
            runtime.local_map_transitions.discard(robot_id)
            return _error(f'SLAM could not be stopped before navigation handoff: {type(exc).__name__}', 502)
        failure = _response_for_bridge_result(stopped_slam)
        if failure:
            runtime.local_map_transitions.discard(robot_id)
            return failure
        runtime.robot_mapping_state[robot_id] = 'PAUSED'
        runtime.invalidate_path_previews(robot_id, 'runtime changed from mapping to navigation')

        result = adapter.request_mode_change(robot_id, 'NAVIGATION')
        if not result.get('ok'):
            runtime.local_map_transitions.discard(robot_id)
            return _error(str(result.get('message') or 'runtime supervisor rejected navigation handoff'),
                          int(result.get('http_status') or 502), {'code': result.get('code')})
        pending = {
            'map_id': map_id, 'map_revision': str(record['revision']),
            'request_id': result.get('request_id'),
            'yaml_path': str(yaml_path), 'image_path': str(image_path),
            'phase': 'MODE_TRANSITION',
        }
        pending_loads[robot_id] = pending
        return JsonResponse({
            'ok': True, 'status': 'TRANSITIONING', 'robot_id': robot_id,
            'active_map': record, 'active_map_id': map_id,
            'active_map_revision': record['revision'],
            'request_id': pending['request_id'],
            'transition': result, 'mapping_state': 'PAUSED',
            'map_sync_status': 'LOADING',
            'message': 'SLAM was paused; the supervisor is restarting the stack in Navigation mode.',
        }, status=202)

    if mode != 'NAVIGATION':
        return _error(f'saved local maps cannot load while runtime mode is {mode or "UNKNOWN"}', 409)

    if pending and pending.get('phase') == 'MODE_TRANSITION':
        adapter = _runtime_adapter(runtime.runtime_mode)
        transition = adapter.get_mode_status(robot_id) if adapter else {}
        if (transition.get('request_id') != pending.get('request_id')
                or transition.get('status') in ('ROLLED_BACK', 'ERROR')):
            _clear_pending_local_map_load(robot_id)
            return _error(transition.get('message') or 'saved map navigation transition is no longer active', 409)
        if (transition.get('status') != 'READY'
                or str(transition.get('mode') or '').lower() != 'navigation'):
            return JsonResponse({
                'ok': True, 'status': 'TRANSITIONING', 'robot_id': robot_id,
                'active_map': record, 'active_map_id': map_id,
                'active_map_revision': record['revision'],
                'request_id': pending.get('request_id'), 'transition': transition,
                'map_sync_status': 'LOADING',
                'message': 'waiting for Nav2 and its selected map server to pass readiness.',
            }, status=202)

    stopped = _require_manual_stopped(robot_id)
    if stopped:
        return stopped
    runtime.local_map_transitions.add(robot_id)
    if pending is None:
        pending = {
            'map_id': map_id, 'map_revision': str(record['revision']),
            'request_id': None, 'yaml_path': str(yaml_path),
            'image_path': str(image_path), 'phase': 'MAP_LOAD',
        }
        pending_loads[robot_id] = pending
    try:
        result = _bridge_request(robot_id, 'MAP_LOAD', {
            'map_id': map_id, 'map_revision': record['revision'],
            'map_yaml': str(yaml_path),
        }, timeout=30.0)
    except Exception as exc:
        _clear_pending_local_map_load(robot_id)
        return _error(f'Nav2 map load request failed: {type(exc).__name__}', 502)
    failure = _response_for_bridge_result(result)
    if failure:
        _clear_pending_local_map_load(robot_id)
        return failure
    applied = result.get('result') or {}
    if (str(applied.get('map_id') or '') != map_id
            or str(applied.get('active_map_revision') or '') != str(record['revision'])):
        _clear_pending_local_map_load(robot_id)
        return _error('ROS confirmed a different active map than the selected artifact', 502)
    runtime.local_map_overrides[robot_id] = map_id
    runtime.local_map_revisions[robot_id] = str(record['revision'])
    runtime.local_map_status[robot_id] = {
        'loaded': True, 'map_id': map_id,
        'active_map_revision': str(record['revision']),
        'canonical_map_revision': runtime.published_map_revision,
    }
    _clear_pending_local_map_load(robot_id)
    runtime.invalidate_path_previews(robot_id, 'active map changed')
    return JsonResponse({
        'ok': True, 'status': 'LOADED', 'robot_id': robot_id,
        'active_map': record, 'active_map_id': map_id,
        'active_map_revision': record['revision'],
        'local_active_map_id': map_id,
        'local_active_map_revision': record['revision'],
        'canonical_map_revision': runtime.published_map_revision,
        'map_source': 'SAVED_LOCAL', 'map_sync_status': 'LOCAL_ONLY',
        'message': 'Nav2 confirmed this robot-local map. Local navigation is enabled; fleet missions remain tied to the canonical map.',
    })


@csrf_exempt
@api_user_required
@require_http_methods(['POST'])
def resume_robot_slam_session(request, robot_id: str):
    """Restart the supervised Mapping runtime with one registered pose graph.

    SLAM Toolbox loads the graph during lifecycle configure, before its scan
    subscription is activated. This avoids replacing graph-owned sensor state
    while the synchronous scan-processing path is running.
    """
    pending_resumes = runtime.pending_slam_session_resumes
    pending = pending_resumes.get(robot_id)
    if pending is None:
        problem = _robot_available(robot_id)
        if problem:
            return problem
    if runtime.runtime_mode != 'GAZEBO_ROS':
        return _error('saved-session resume currently requires the supervised Gazebo/ROS runtime', 409)
    body = _body(request)
    map_id = str(body.get('map_id') or '').strip()
    try:
        record, session_prefix, _posegraph, _session_data = get_robot_slam_session(robot_id, map_id)
        _record, yaml_path, image_path = get_robot_map(robot_id, map_id)
    except (FileNotFoundError, ValueError, OSError) as exc:
        return _error(str(exc), 404)

    if pending and pending.get('map_id') != map_id:
        return _error('another saved SLAM session is already restoring for this robot', 409)
    if robot_id in runtime.pending_local_map_loads:
        return _error('saved SLAM resume is blocked while a navigation map is loading', 409)

    adapter = _runtime_adapter(runtime.runtime_mode)
    if adapter is None:
        return _error('no supervised runtime adapter is available for saved-session resume', 409)

    if pending is None:
        stopped = _require_manual_stopped(robot_id)
        if stopped:
            return stopped
        mode = str(runtime.operation_mode or '').upper()
        if mode not in ('MAPPING', 'NAVIGATION'):
            return _error(f'saved SLAM sessions cannot resume while runtime mode is {mode or "UNKNOWN"}', 409)
        transition = adapter.get_mode_status(robot_id)
        if transition.get('status') != 'READY' or str(transition.get('mode') or '').lower() != mode.lower():
            return _error('runtime supervisor is not ready for a saved-session handoff', 409,
                          {'transition': transition})
        if mode == 'MAPPING' and str(runtime.robot_mapping_state.get(robot_id) or '').upper() != 'PAUSED':
            try:
                stopped_slam = _bridge_request(robot_id, 'MAPPING_STOP', timeout=8.0)
            except Exception as exc:
                return _error(f'SLAM could not pause before session restore: {type(exc).__name__}', 502)
            failure = _response_for_bridge_result(stopped_slam)
            if failure:
                return failure
            runtime.robot_mapping_state[robot_id] = 'PAUSED'

        runtime.local_map_transitions.add(robot_id)
        runtime.invalidate_path_previews(robot_id, 'runtime changed to resume a saved SLAM session')
        result = adapter.request_mode_change(
            robot_id, 'MAPPING', slam_session_file=str(session_prefix))
        if not result.get('ok'):
            runtime.local_map_transitions.discard(robot_id)
            return _error(str(result.get('message') or 'runtime supervisor rejected SLAM session resume'),
                          int(result.get('http_status') or 502), {'code': result.get('code')})
        pending = {
            'map_id': map_id, 'map_revision': str(record['revision']),
            'session_artifact_id': (record.get('slam_session_state') or {}).get('artifact_id'),
            'request_id': result.get('request_id'), 'started_monotonic': time.monotonic(),
            'phase': 'MODE_TRANSITION', 'last_snapshot_marker': None,
            'restore_evidence': None,
        }
        pending_resumes[robot_id] = pending
        # Do not let a stale pre-restart map satisfy the restored-map gate.
        runtime.robot_map_snapshots.pop(robot_id, None)
        runtime.robot_runtime_map_snapshots.pop(robot_id, None)
        runtime.robot_slam_map_snapshots.pop(robot_id, None)
        runtime.robot_map_geometry.pop(robot_id, None)
        return _slam_session_resume_response(robot_id, record, pending, result)

    transition = adapter.get_mode_status(robot_id)
    if (transition.get('request_id') != pending.get('request_id')
            or transition.get('status') in ('ROLLED_BACK', 'ERROR')):
        _clear_pending_slam_session_resume(robot_id)
        return _error(transition.get('message') or 'saved SLAM session runtime transition failed', 409,
                      {'transition': transition})
    if (transition.get('status') != 'READY'
            or str(transition.get('mode') or '').lower() != 'mapping'
            or str(runtime.operation_mode or '').upper() != 'MAPPING'):
        return _slam_session_resume_response(robot_id, record, pending, transition)

    # SLAM mode is now authoritative; a previous Navigation local-map identity
    # must not remain presented as the active map while the restored /map is
    # being independently checked.
    runtime.local_map_overrides.pop(robot_id, None)
    runtime.local_map_revisions.pop(robot_id, None)
    runtime.local_map_status.pop(robot_id, None)
    snapshot = runtime.robot_map_snapshots.get(robot_id) or {}
    map_data = snapshot.get('map') if isinstance(snapshot, dict) else None
    evidence = None
    if isinstance(map_data, dict) and map_data.get('map_source') == 'SLAM_TOOLBOX':
        marker = (map_data.get('mapping_session_id'), map_data.get('map_version'),
                  map_data.get('timestamp'), map_data.get('stamp'))
        if marker != pending.get('last_snapshot_marker'):
            evidence = slam_map_restoration_evidence(record, yaml_path, image_path, map_data)
            pending['last_snapshot_marker'] = marker
            pending['restore_evidence'] = evidence
        else:
            evidence = pending.get('restore_evidence')
    if evidence and evidence.get('passed'):
        runtime.robot_mapping_state[robot_id] = 'MAPPING'
        runtime.robot_mapping_elapsed_s[robot_id] = 0.0
        runtime.robot_mapping_sessions[robot_id] = str(map_data.get('mapping_session_id') or '')
        _clear_pending_slam_session_resume(robot_id)
        return _slam_session_resume_response(robot_id, record, pending, transition,
                                             status='RESUMED', evidence=evidence)

    if time.monotonic() - float(pending.get('started_monotonic') or time.monotonic()) > 180.0:
        try:
            _bridge_request(robot_id, 'MAPPING_STOP', timeout=8.0)
            runtime.robot_mapping_state[robot_id] = 'PAUSED'
        except Exception:
            pass
        _clear_pending_slam_session_resume(robot_id)
        return _error('saved SLAM runtime started, but the live map did not prove restoration of the saved area',
                      502, {'restore_evidence': evidence or pending.get('restore_evidence')})
    return _slam_session_resume_response(robot_id, record, pending, transition,
                                         evidence=evidence or pending.get('restore_evidence'))


@csrf_exempt
@api_user_required
@require_http_methods(['POST'])
def initialize_robot_pose(request, robot_id: str):
    problem = _robot_available(robot_id)
    if problem:
        return problem
    stopped = _require_manual_stopped(robot_id)
    if stopped:
        return stopped
    body = _body(request)
    try:
        pose = {key: float(body[key]) for key in ('x', 'y', 'yaw')}
    except (KeyError, TypeError, ValueError):
        return _error('x, y, and yaw must be finite numbers')
    import math
    if not all(math.isfinite(value) for value in pose.values()):
        return _error('x, y, and yaw must be finite numbers')
    if pose['yaw'] < -math.pi or pose['yaw'] > math.pi:
        return _error('yaw must be within [-pi, pi]')
    if robot_id in runtime.local_map_transitions:
        return _error('initial pose is blocked while a saved local map is loading', 409)
    active_map = runtime.active_map_state(robot_id)
    if (not active_map.get('active_map_id') or not active_map.get('active_map_revision')
            or active_map.get('map_sync_status') not in ('CANONICAL', 'LOCAL_ONLY')):
        return _error('the selected robot has no confirmed active map', 409)
    if not _pose_within_active_map(robot_id, pose):
        return _error('initial pose must be inside the active map bounds', 400)
    frame_id = str(body.get('frame_id') or 'map')
    if frame_id != 'map':
        return _error('initial pose must use the map frame')
    try:
        result = _bridge_request(robot_id, 'INITIAL_POSE', {
            **pose, 'frame_id': frame_id,
            'active_map_id': active_map['active_map_id'],
            'active_map_revision': active_map['active_map_revision'],
        }, timeout=10.0)
    except Exception as exc:
        return _error(f'initial pose request failed: {type(exc).__name__}', 502)
    failure = _response_for_bridge_result(result)
    if failure:
        return failure
    applied = result.get('result') or {}
    if (not applied.get('runtime_pose_confirmed')
            or applied.get('active_map_id') != active_map['active_map_id']
            or str(applied.get('active_map_revision') or '') != str(active_map['active_map_revision'])):
        return _error('localization service did not confirm the requested pose on the active map', 502)
    return JsonResponse({'ok': True, 'robot_id': robot_id, 'frame_id': frame_id,
                         'pose': pose, 'active_map_id': active_map['active_map_id'],
                         'active_map_revision': active_map['active_map_revision'],
                         'localization_owner': 'ekf_v30e'})


def _get_vda_config(robot_id: str):
    config, _created = RobotVda5050Configuration.objects.get_or_create(
        robot_id=robot_id, defaults={'serial_number': robot_id})
    return config


@csrf_exempt
@api_user_required
@require_http_methods(['GET', 'PUT'])
def vda5050_configuration(request, robot_id: str):
    if request.method == 'GET':
        config = _get_vda_config(robot_id)
        state = ensure_configuration_active(config)
        return JsonResponse(public_configuration(config, state))

    body = _body(request)
    if not body:
        return _error('VDA5050 configuration body must be a non-empty JSON object')
    config = _get_vda_config(robot_id)
    old_values = {field: getattr(config, field) for field in (
        'enabled', 'mqtt_host', 'mqtt_port', 'mqtt_username', 'mqtt_password_ciphertext',
        'tls_enabled', 'topic_prefix', 'interface_name', 'manufacturer', 'serial_number',
        'protocol_version', 'mqtt_protocol_version', 'allow_task', 'allow_instant_actions',
        'auto_reconnect', 'reconnect_interval', 'connection_timeout', 'keepalive', 'client_id',
    )}
    try:
        values = validate_configuration(body)
        password = str(body.get('mqtt_password') or '')
        if password:
            if len(password) > 256:
                return _error('MQTT password is longer than 256 characters')
            ciphertext = encrypt_password(password)
        else:
            ciphertext = config.mqtt_password_ciphertext
        if values['enabled'] and values['mqtt_username'] and not ciphertext:
            return _error('MQTT password is required when a username is configured')
    except ValueError as exc:
        return _error(str(exc))
    for field, value in values.items():
        setattr(config, field, value)
    config.mqtt_password_ciphertext = ciphertext
    config.save()
    try:
        connection_state = apply_configuration(config)
    except Exception as exc:
        with transaction.atomic():
            for field, value in old_values.items():
                setattr(config, field, value)
            config.save()
        try:
            apply_configuration(config)
        except Exception:
            pass
        return _error(f'VDA5050 configuration was not applied: {type(exc).__name__}', 502)
    if values['enabled'] and connection_state.get('status') != 'CONNECTED':
        with transaction.atomic():
            for field, value in old_values.items():
                setattr(config, field, value)
            config.save()
        try:
            apply_configuration(config)
        except Exception:
            pass
        return JsonResponse({
            'success': False,
            'error': {'code': 'VDA5050_APPLY_FAILED',
                      'message': connection_state.get('last_error') or 'MQTT broker connection failed'},
            'connection': connection_state,
        }, status=502)
    return JsonResponse({
        'ok': True, 'configuration': public_configuration(config, connection_state),
        'applied': True,
    })


@csrf_exempt
@api_user_required
@require_http_methods(['POST'])
def vda5050_test_connection(request, robot_id: str):
    body = _body(request)
    if not body:
        config = _get_vda_config(robot_id)
        draft = {field: getattr(config, field) for field in (
            'enabled', 'mqtt_host', 'mqtt_port', 'mqtt_username', 'tls_enabled', 'topic_prefix',
            'interface_name', 'manufacturer', 'serial_number', 'protocol_version',
            'mqtt_protocol_version', 'allow_task', 'allow_instant_actions', 'auto_reconnect',
            'reconnect_interval', 'connection_timeout', 'keepalive', 'client_id',
        )}
        password = decrypt_password(config.mqtt_password_ciphertext)
    else:
        config = _get_vda_config(robot_id)
        draft = {field: getattr(config, field) for field in (
            'enabled', 'mqtt_host', 'mqtt_port', 'mqtt_username', 'tls_enabled', 'topic_prefix',
            'interface_name', 'manufacturer', 'serial_number', 'protocol_version',
            'mqtt_protocol_version', 'allow_task', 'allow_instant_actions', 'auto_reconnect',
            'reconnect_interval', 'connection_timeout', 'keepalive', 'client_id',
        )}
        draft.update(body)
        password = str(body.get('mqtt_password') or decrypt_password(config.mqtt_password_ciphertext))
    draft['enabled'] = True
    try:
        values = validate_configuration(draft)
    except ValueError as exc:
        return _error(str(exc))
    if values['mqtt_username'] and not password:
        return _error('MQTT password is required when a username is configured')
    client_config = SimpleNamespace(
        **values, robot_id=robot_id,
        mqtt_password_ciphertext=encrypt_password(password),
    )
    result = test_connection(client_config, password)
    return JsonResponse(result, status=200 if result['ok'] else 502)
