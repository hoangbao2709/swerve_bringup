from __future__ import annotations

import json
import fcntl
import math
import os
import threading
import uuid
from types import SimpleNamespace
from pathlib import Path

from asgiref.sync import async_to_sync
from django.conf import settings
from django.db import transaction
from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt
from django.views.decorators.http import require_http_methods

from .local_control import (
    get_robot_map, list_robot_maps, map_output_prefix, register_saved_map,
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


def _write_mode_request(robot_id: str, mode: str) -> dict:
    request_path, status_path = _mode_files()
    request_id = uuid.uuid4().hex
    request = {'request_id': request_id, 'robot_id': robot_id, 'mode': mode}
    request_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = request_path.with_suffix(request_path.suffix + '.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        os.chmod(temporary, 0o600)
        json.dump(request, stream, separators=(',', ':'))
        stream.flush()
        os.fsync(stream.fileno())
    status = {**request, 'status': 'REQUESTED', 'message': 'waiting for the ROS stack supervisor'}
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


def _robot_available(robot_id: str):
    if not runtime.is_external:
        return _error('local robot control requires the active ROS/Gazebo runtime', 503)
    if not runtime.robot_bridge_online(robot_id):
        return _error(f'authenticated ROS bridge for {robot_id} is offline', 503)
    return None


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
    return JsonResponse({
        'robot_id': robot_id,
        'maps': maps,
        'runtime_mode': runtime.operation_mode,
        'mapping_state': getattr(runtime, 'robot_mapping_state', {}).get(robot_id, 'UNKNOWN'),
        'mapping_duration_s': getattr(runtime, 'robot_mapping_elapsed_s', {}).get(robot_id, 0.0),
        'active_local_map_id': runtime.local_map_overrides.get(robot_id),
        'map_sync_status': runtime.runtime_status_message().get('map_sync_status'),
    })


@csrf_exempt
@api_user_required
@require_http_methods(['GET', 'POST'])
def local_runtime_mode(request, robot_id: str):
    if request.method == 'GET':
        return JsonResponse({
            'robot_id': robot_id,
            'current_mode': runtime.operation_mode,
            'transition': _read_mode_status(robot_id),
        })
    if not runtime.is_external or runtime.runtime_mode != 'GAZEBO_ROS':
        return _error('controlled mapping/navigation transition requires the owned GAZEBO_ROS stack', 409)
    if not runtime.robot_bridge_online(robot_id):
        return _error(f'authenticated ROS bridge for {robot_id} is offline', 503)
    target = str(_body(request).get('mode') or '').strip().upper()
    if target not in ('MAPPING', 'NAVIGATION'):
        return _error('mode must be MAPPING or NAVIGATION')
    current = str(runtime.operation_mode).upper()
    if current not in ('MAPPING', 'NAVIGATION'):
        return _error(f'cannot switch runtime mode from {current}', 409)
    if target == current:
        return JsonResponse({'ok': True, 'robot_id': robot_id, 'current_mode': current,
                             'status': 'READY', 'message': 'requested mode is already active'})
    robot = runtime.engine.state.get('robots', {}).get(robot_id, {})
    if str(robot.get('control_mode') or '').upper() != 'MANUAL' or str(robot.get('navigation_state') or '').upper() != 'MANUAL':
        return _error('switch to MANUAL mode and stop the robot before changing SLAM/Nav2 runtime mode', 409)
    state = _read_mode_status(robot_id)
    if state.get('status') in ('STARTING', 'REQUESTED', 'RESTARTING', 'ROLLING_BACK'):
        return _error('a runtime mode transition is already in progress', 409)
    with _MODE_SWITCH_LOCK:
        request_path, _status_path = _mode_files()
        lock_path = request_path.with_suffix('.lock')
        lock_fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o600)
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(lock_fd)
            return _error('a runtime mode transition request is already being queued', 409)
        try:
            if request_path.exists():
                return _error('a runtime mode transition request is already queued', 409)
            status = _read_mode_status(robot_id)
            if status.get('status') == 'UNAVAILABLE':
                return _error('ROS stack mode supervisor status is unavailable', 503)
            if status.get('status') in ('STARTING', 'REQUESTED', 'RESTARTING', 'ROLLING_BACK'):
                return _error('a runtime mode transition is already in progress', 409)
            status = _write_mode_request(robot_id, target.lower())
        finally:
            fcntl.flock(lock_fd, fcntl.LOCK_UN)
            os.close(lock_fd)
    return JsonResponse({
        'ok': True, 'robot_id': robot_id, 'current_mode': current,
        'requested_mode': target, 'request_id': status['request_id'],
        'status': status['status'],
        'message': 'ROS/Gazebo will restart; the simulated robot will respawn at its configured start pose.',
    }, status=202)


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
    try:
        name = validate_map_name(_body(request).get('name'))
        prefix = map_output_prefix(robot_id, name)
    except ValueError as exc:
        return _error(str(exc))
    if any(row.get('name') == name for row in list_robot_maps(robot_id)) or prefix.with_suffix('.yaml').exists():
        return _error('a map with this name already exists for this robot', 409)
    try:
        result = _bridge_request(robot_id, 'MAP_SAVE', {
            'name': name, 'output_prefix': str(prefix),
        }, timeout=45.0)
    except Exception as exc:
        return _error(f'map saver request failed: {type(exc).__name__}', 502)
    failure = _response_for_bridge_result(result)
    if failure:
        return failure
    try:
        record = register_saved_map(robot_id, name, prefix.with_suffix('.yaml'))
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
    if runtime.operation_mode != 'NAVIGATION':
        return _error('map load requires the active Nav2 navigation runtime', 409)
    stopped = _require_manual_stopped(robot_id)
    if stopped:
        return stopped
    if robot_id in runtime.local_map_transitions:
        return _error('a local map transition is already in progress', 409)
    map_id = str(_body(request).get('map_id') or '').strip()
    try:
        record, yaml_path, _image_path = get_robot_map(robot_id, map_id)
    except (FileNotFoundError, ValueError) as exc:
        return _error(str(exc), 404)
    runtime.local_map_transitions.add(robot_id)
    try:
        try:
            result = _bridge_request(robot_id, 'MAP_LOAD', {
                'map_id': map_id, 'map_yaml': str(yaml_path),
            }, timeout=30.0)
        except Exception as exc:
            return _error(f'Nav2 map load request failed: {type(exc).__name__}', 502)
        failure = _response_for_bridge_result(result)
        if failure:
            return failure
        runtime.local_map_overrides[robot_id] = map_id
        return JsonResponse({
            'ok': True, 'robot_id': robot_id, 'active_map': record,
            'map_sync_status': 'OUT_OF_SYNC',
            'message': 'Nav2 loaded this map. Autonomous goals remain blocked until the local map is published as the canonical warehouse map.',
        })
    finally:
        runtime.local_map_transitions.discard(robot_id)


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
    frame_id = str(body.get('frame_id') or 'map')
    if frame_id != 'map':
        return _error('initial pose must use the map frame')
    try:
        result = _bridge_request(robot_id, 'INITIAL_POSE', {**pose, 'frame_id': frame_id}, timeout=10.0)
    except Exception as exc:
        return _error(f'initial pose request failed: {type(exc).__name__}', 502)
    failure = _response_for_bridge_result(result)
    if failure:
        return failure
    return JsonResponse({'ok': True, 'robot_id': robot_id, 'frame_id': frame_id,
                         'pose': pose, 'localization_owner': 'ekf_v30e'})


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
