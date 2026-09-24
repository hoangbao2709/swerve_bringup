from __future__ import annotations

import json
import logging
import os
import re
import shutil
import time
from datetime import datetime, timezone
from functools import wraps
from typing import Any

from django.contrib.auth import authenticate
from django.conf import settings
from django.http import HttpRequest, JsonResponse, HttpResponse
from django.views.decorators.csrf import csrf_exempt
from django.views.decorators.http import require_http_methods
from django.db import IntegrityError
from django.db import connection
from asgiref.sync import async_to_sync
from channels.layers import get_channel_layer
from pydantic import TypeAdapter, ValidationError

from django.contrib.auth.models import User
from accounts.models import ApiToken, UserProfile, ensure_profile, public_user, role_of
from .auth import user_from_request
from .models import EventLog
from .warehouse_services import ensure_active_map, sync_from_layout, save_layout_to_map, publish_layout_to_map, master_to_layout
from .map_sync import published_map_payload, map_sync_status
from .runtime import runtime
from .schema import ScenarioInjection, WhatIfRequest
from .ai import copilot as copilot_ai
from .ai import vlm as vlm_ai
from .sim.whatif import run_whatif
from .canonical_map import canonicalize_layout, validate_canonical_layout

log = logging.getLogger(__name__)
inject_adapter = TypeAdapter(ScenarioInjection)


def _body(request: HttpRequest) -> dict[str, Any]:
    if not request.body:
        return {}
    try:
        data = json.loads(request.body.decode('utf-8'))
        return data if isinstance(data, dict) else {}
    except (UnicodeDecodeError, json.JSONDecodeError):
        return {}


def _error(message: str, status: int = 400, details: dict[str, Any] | None = None):
    return JsonResponse({
        'success': False,
        'error': {'code': f'HTTP_{status}', 'message': message, 'details': details or {}},
        # Keep the legacy field while clients migrate to the common envelope.
        'detail': message,
    }, status=status)


def _valid_password(password: str) -> str | None:
    if len(password or '') < 8:
        return 'password must be at least 8 characters'
    if not re.search(r'[A-Za-z]', password) or not re.search(r'\d', password):
        return 'password must contain letters and digits'
    return None


def api_user_required(view):
    @wraps(view)
    def wrapped(request, *args, **kwargs):
        user = user_from_request(request)
        if user is None:
            return _error('authentication required', 401)
        request.api_user = user
        return view(request, *args, **kwargs)
    return wrapped


def api_admin_required(view):
    @wraps(view)
    def wrapped(request, *args, **kwargs):
        user = user_from_request(request)
        if user is None:
            return _error('authentication required', 401)
        if role_of(user) != 'admin':
            return _error('admin privileges required', 403)
        request.api_user = user
        return view(request, *args, **kwargs)
    return wrapped


def _audit(user: User, action: str, target: str | None = None):
    runtime.engine.emit('ADMIN_ACTION', 'USER', 'INFO', f"ADMIN_ACTION admin={user.username} action={action}" + (f" target={target}" if target else ''), payload={'action': action, 'admin': user.username, 'target': target, 'result': 'SUCCESS'})


def _broadcast_layout_update(map_obj, source: str) -> None:
    runtime.replace_layout(map_obj.layout)
    # Keep scheduler work-points in the same database transaction domain as the
    # authoritative map. Shelf/conveyor/station coordinates therefore converge
    # immediately after edits from either Warehouse Data or Map Editor.
    try:
        from .schedule_services import sync_workpoints_from_layout, sync_robot_profiles
        sync_workpoints_from_layout(map_obj.warehouse, map_obj.layout, prune=False)
        sync_robot_profiles(map_obj.warehouse, runtime.engine.state.get('robots') or {})
    except Exception as exc:
        # Scheduler tables may not exist before migration; layout publishing must
        # remain available so initial migrations/seed can recover cleanly.
        log.exception(
            'Unable to synchronize scheduler data after layout update: %s',
            type(exc).__name__,
        )
    layer = get_channel_layer()
    if layer is None:
        return
    meta = {
        'type': 'LAYOUT_UPDATED',
        'source': source,
        'warehouse_id': map_obj.warehouse_id,
        'layout_id': map_obj.layout.get('id'),
        'revision': map_obj.revision,
        'published_version': map_obj.published_version,
        'is_active': map_obj.is_active,
        'updated_at': map_obj.updated_at.isoformat() if map_obj.updated_at else None,
    }
    async_to_sync(layer.group_send)('twin_clients', {'type': 'twin.message', 'payload': meta})
    async_to_sync(layer.group_send)('twin_clients', {'type': 'twin.message', 'payload': runtime.full_message()})


def root(request):
    return JsonResponse({
        'service': 'waretwin-django-backend',
        'ws': '/ws',
        'health': '/api/health/',
        'mode': settings.WARETWIN_RUNTIME_MODE,
    })


@api_user_required
@require_http_methods(['GET'])
def conveyors(request):
    """Authoritative PLC-backed conveyor state, including tracked belt items."""
    return JsonResponse(list(runtime.conveyor_snapshot().values()), safe=False)


@csrf_exempt
@api_user_required
@require_http_methods(['POST'])
def conveyor_command(request, conveyor_id: str):
    body = _body(request)
    action = str(body.get('action', '')).upper()
    if not action:
        return _error('action is required')
    try:
        result = async_to_sync(runtime.command_conveyor)(conveyor_id, action, **body)
    except (KeyError, ValueError) as exc:
        return _error(str(exc))
    return JsonResponse(result)


@csrf_exempt
@api_user_required
@require_http_methods(['POST'])
def conveyor_handshake(request, conveyor_id: str):
    body = _body(request)
    try:
        result = async_to_sync(runtime.conveyor_handshake)(
            conveyor_id,
            str(body.get('robot_id', '')),
            str(body.get('item_id', '')),
            str(body.get('phase', '')),
        )
    except (KeyError, ValueError) as exc:
        return _error(str(exc))
    return JsonResponse(result)


@csrf_exempt
@require_http_methods(['POST'])
def auth_register(request):
    d = _body(request)
    username = str(d.get('username', '')).strip()
    email = str(d.get('email', '')).strip()
    password = str(d.get('password', ''))
    if len(username) < 3:
        return _error('invalid username')
    if '@' not in email:
        return _error('invalid email')
    err = _valid_password(password)
    if err:
        return _error(err)
    if User.objects.filter(email__iexact=email).exists():
        return _error('email already exists', 409)
    try:
        user = User.objects.create_user(username=username, email=email, password=password)
    except IntegrityError:
        return _error('username or email already exists', 409)
    return JsonResponse(public_user(user), status=201)


@csrf_exempt
@require_http_methods(['POST'])
def auth_login(request):
    d = _body(request)
    user = authenticate(request, username=d.get('username', ''), password=d.get('password', ''))
    if user is None or not user.is_active:
        return _error('Invalid username or password', 401)
    token = ApiToken.issue(user)
    return JsonResponse({'access_token': token.key, 'token_type': 'bearer', 'user': public_user(user)})


@csrf_exempt
@api_user_required
@require_http_methods(['POST'])
def auth_logout(request):
    token = request.headers.get('Authorization', '').removeprefix('Bearer ').strip()
    if token:
        ApiToken.objects.filter(key=token).delete()
    return JsonResponse({'ok': True})


@api_user_required
@require_http_methods(['GET'])
def auth_me(request):
    return JsonResponse(public_user(request.api_user))


@csrf_exempt
@api_admin_required
@require_http_methods(['GET', 'POST'])
def admin_users(request):
    if request.method == 'GET':
        return JsonResponse([public_user(u) for u in User.objects.order_by('id')], safe=False)
    d = _body(request)
    err = _valid_password(str(d.get('password', '')))
    if err:
        return _error(err)
    if User.objects.filter(email__iexact=str(d.get('email', '')).strip()).exists():
        return _error('email already exists', 409)
    try:
        user = User.objects.create_user(
            username=str(d.get('username', '')).strip(),
            email=str(d.get('email', '')).strip(),
            password=str(d.get('password', '')),
            is_active=bool(d.get('is_active', True)),
        )
        ensure_profile(user, d.get('role', 'user') if d.get('role') in ('admin', 'user') else 'user')
    except IntegrityError:
        return _error('username or email already exists', 409)
    _audit(request.api_user, 'CREATE_USER', str(user.id))
    return JsonResponse(public_user(user), status=201)


@csrf_exempt
@api_admin_required
@require_http_methods(['GET', 'PATCH', 'DELETE'])
def admin_user_detail(request, user_id: int):
    try:
        user = User.objects.get(id=user_id)
    except User.DoesNotExist:
        return _error('user not found', 404)
    if request.method == 'GET':
        return JsonResponse(public_user(user))
    if request.method == 'DELETE':
        if user.id == request.api_user.id:
            return _error('cannot delete yourself')
        if role_of(user) == 'admin' and UserProfile.objects.filter(role='admin', user__is_active=True).count() <= 1:
            return _error('cannot remove the last admin')
        user.delete()
        _audit(request.api_user, 'DELETE_USER', str(user_id))
        return JsonResponse({'ok': True})
    d = _body(request)
    if d.get('role') == 'user' and role_of(user) == 'admin' and UserProfile.objects.filter(role='admin', user__is_active=True).count() <= 1:
        return _error('cannot remove the last admin')
    if d.get('is_active') is False and user.id == request.api_user.id:
        return _error('cannot disable yourself')
    if d.get('role') == 'user' and user.id == request.api_user.id:
        return _error('cannot demote yourself')
    if 'username' in d:
        user.username = str(d['username']).strip()
    if 'email' in d:
        new_email = str(d['email']).strip()
        if User.objects.exclude(id=user.id).filter(email__iexact=new_email).exists():
            return _error('email already exists', 409)
        user.email = new_email
    if d.get('role') in ('admin', 'user'):
        ensure_profile(user, d['role'])
    if 'is_active' in d:
        user.is_active = bool(d['is_active'])
    if d.get('password'):
        err = _valid_password(str(d['password']))
        if err:
            return _error(err)
        user.set_password(str(d['password']))
    try:
        user.save()
    except IntegrityError:
        return _error('username or email already exists', 409)
    _audit(request.api_user, 'UPDATE_USER', str(user_id))
    return JsonResponse(public_user(user))


@csrf_exempt
@api_admin_required
@require_http_methods(['POST'])
def admin_reset_password(request, user_id: int):
    try:
        user = User.objects.get(id=user_id)
    except User.DoesNotExist:
        return _error('user not found', 404)
    password = str(_body(request).get('password', ''))
    err = _valid_password(password)
    if err:
        return _error(err)
    user.set_password(password)
    user.save(update_fields=['password'])
    ApiToken.objects.filter(user=user).delete()
    _audit(request.api_user, 'RESET_PASSWORD', str(user_id))
    return JsonResponse(public_user(user))


def _database_probe() -> tuple[bool, str | None]:
    database_ok = False
    database_error = None
    try:
        with connection.cursor() as cursor:
            cursor.execute('SELECT 1')
            database_ok = cursor.fetchone() == (1,)
    except Exception as exc:
        database_error = f'{type(exc).__name__}: {exc}'
    return database_ok, database_error


def _robot_telemetry_alive() -> bool:
    last = runtime.last_telemetry_at
    if last is None:
        return False
    timeout = float(getattr(settings, 'WARETWIN_ROS_HEARTBEAT_TIMEOUT_S', 3.0))
    return (time.monotonic() - last) <= timeout


def _system_health_status(
    runtime_mode: str,
    components: dict[str, Any],
    *,
    database_ok: bool,
    robot_telemetry_alive: bool,
) -> str:
    """Return full-system health independently from backend readiness.

    The HTTP health endpoint is also used as a Django readiness probe, so the
    backend and the connected robot stack intentionally have separate fields.
    This helper keeps the overall status truthful for every runtime profile.
    """
    if not database_ok or not components.get('websocket'):
        return 'ERROR'
    if components.get('runtime_state') == 'ERROR':
        return 'ERROR'
    if runtime_mode == 'LOCAL_SIM':
        return 'OK'
    if not components.get('ros_bridge') or not components.get('ros'):
        return 'DISCONNECTED'
    if not robot_telemetry_alive:
        return 'DISCONNECTED'
    if runtime_mode == 'GAZEBO_ROS' and not components.get('gazebo'):
        return 'DEGRADED'
    return 'OK'


def health(request):
    database_ok, database_error = _database_probe()
    components = runtime.health_snapshot()
    backend_ready = database_ok and components['websocket']
    backend_status = 'READY' if backend_ready else 'NOT_READY'
    system_status = _system_health_status(
        settings.WARETWIN_RUNTIME_MODE,
        components,
        database_ok=database_ok,
        robot_telemetry_alive=_robot_telemetry_alive(),
    )
    overall_ok = backend_ready and system_status == 'OK'
    status = 'ok' if overall_ok else ('error' if system_status == 'ERROR' else 'degraded')
    task_alive = runtime._task is not None and not runtime._task.done()
    return JsonResponse({
        'status': status,
        'ok': overall_ok,
        'backend_status': backend_status,
        'backend_ready': backend_ready,
        'system_status': system_status,
        'system_ready': system_status == 'OK',
        'database': database_ok,
        'ros_bridge': components['ros_bridge'],
        'websocket': components['websocket'],
        'ros': components['ros'],
        'gazebo': components['gazebo'],
        'mode': components['runtime_state'],
        'runtime_mode': settings.WARETWIN_RUNTIME_MODE,
        'runtime_state': components['runtime_state'],
        'timestamp': datetime.now(timezone.utc).isoformat(),
        'version': getattr(settings, 'WARETWIN_VERSION', '0.1.0'),
        'components': components,
        'database_error': database_error,
        'configured_runtime_mode': settings.WARETWIN_RUNTIME_MODE,
        'run_id': runtime.run_id,
        'tick': runtime.engine.state['sim']['tick'],
        'speed': runtime.speed,
        'paused': runtime.paused,
        'clients': runtime.client_count,
        'tick_rate': round(runtime.tick_rate_actual, 1),
        'robots': len(runtime.engine.state['robots']),
        'sim_task_alive': task_alive,
        'standby': not task_alive,
        'db_ok': database_ok,
        'loop_errors': runtime.loop_errors,
        'last_error': runtime.last_error,
    })


def system_status(request):
    """Measured host + ROS diagnostics for the operator diagnostics page."""
    metrics: dict[str, Any] = {'cpu_load_1m': None, 'cpu_count': os.cpu_count() or 1, 'memory': {}, 'disk': {}, 'uptime_s': None}
    try:
        metrics['cpu_load_1m'] = os.getloadavg()[0]
    except (AttributeError, OSError):
        pass
    try:
        meminfo = {}
        with open('/proc/meminfo', encoding='utf-8') as stream:
            for line in stream:
                key, value = line.split(':', 1)
                meminfo[key] = int(value.strip().split()[0]) * 1024
        total = meminfo.get('MemTotal', 0)
        available = meminfo.get('MemAvailable', meminfo.get('MemFree', 0))
        metrics['memory'] = {
            'total_bytes': total,
            'available_bytes': available,
            'used_bytes': max(0, total - available),
            'used_percent': round((total - available) * 100 / total, 2) if total else None,
        }
    except (OSError, ValueError):
        pass
    try:
        usage = shutil.disk_usage(settings.BASE_DIR)
        metrics['disk'] = {
            'total_bytes': usage.total, 'used_bytes': usage.used,
            'free_bytes': usage.free, 'used_percent': round(usage.used * 100 / usage.total, 2),
        }
    except OSError:
        pass
    try:
        with open('/proc/uptime', encoding='utf-8') as stream:
            metrics['uptime_s'] = float(stream.read().split()[0])
    except (OSError, ValueError):
        pass
    database_ok, database_error = _database_probe()
    components = runtime.health_snapshot()
    backend_ready = database_ok and components['websocket']
    backend_status = 'READY' if backend_ready else 'NOT_READY'
    system_health = _system_health_status(
        settings.WARETWIN_RUNTIME_MODE,
        components,
        database_ok=database_ok,
        robot_telemetry_alive=_robot_telemetry_alive(),
    )
    overall_ok = backend_ready and system_health == 'OK'
    robot_state = {
        str(robot_id): {
            'status': robot.get('status'),
            'fsm': robot.get('fsm'),
            'position': robot.get('position'),
            'vx': robot.get('vx'),
            'vy': robot.get('vy'),
            'wz': robot.get('wz'),
            'last_telemetry_at': robot.get('last_telemetry_at'),
        }
        for robot_id, robot in runtime.engine.state.get('robots', {}).items()
    }
    return JsonResponse({
        'status': 'ok' if overall_ok else ('error' if system_health == 'ERROR' else 'degraded'),
        'ok': overall_ok,
        'backend_status': backend_status,
        'backend_ready': backend_ready,
        'system_status': system_health,
        'system_ready': system_health == 'OK',
        'database': database_ok,
        'database_error': database_error,
        'timestamp': datetime.now(timezone.utc).isoformat(),
        'version': getattr(settings, 'WARETWIN_VERSION', '0.1.0'),
        'mode': components['runtime_state'],
        'runtime_mode': settings.WARETWIN_RUNTIME_MODE,
        'system': metrics,
        'ros': components['diagnostics'],
        'runtime': {
            'bridge_state': components['bridge_state'],
            'ros_connected': components['ros_connected'],
            'nav2_state': runtime.nav2_state,
            'last_telemetry_at': runtime.last_telemetry_iso,
            'loop_errors': runtime.loop_errors,
            'last_error': runtime.last_error,
            'clients': runtime.client_count,
        },
        'simulation_state': runtime.engine.state.get('sim', {}),
        'controller_state': components['diagnostics'].get('controllers', []),
        'robots': robot_state,
    })


def state(request):
    return JsonResponse(runtime.full_message()['state'])


def state_validate(request):
    try:
        runtime.validate_state()
        return JsonResponse({'valid': True})
    except Exception as exc:
        log.exception('Runtime state validation failed: %s', type(exc).__name__)
        return _error('runtime state validation failed', 500, {'reason': str(exc), 'valid': False})


def layout_get(request):
    active = ensure_active_map(runtime.layout)
    if runtime.layout.get('id') != active.layout.get('id') or runtime.layout != active.layout:
        runtime.replace_layout(active.layout)
    # Geometry remains owned by the published map, while shelf occupancy is read
    # from relational master data so current_load is never stale in the live view.
    response_layout = canonicalize_layout(master_to_layout(active.warehouse, active.layout))
    response = JsonResponse(response_layout)
    response['X-Warehouse-Id'] = str(active.warehouse_id)
    response['X-Layout-Revision'] = str(active.revision)
    response['X-Layout-Version'] = str(active.published_version)
    response['X-Layout-Updated-At'] = active.updated_at.isoformat() if active.updated_at else ''
    return response


def _validate_layout_doc(doc: dict[str, Any]) -> list[str]:
    import math

    errors: list[str] = validate_canonical_layout(doc)
    required = [
        'schema_version', 'id', 'name', 'size', 'grid', 'floors', 'zones',
        'racks', 'conveyors', 'stations', 'charging_stations', 'docks',
        'lifts', 'obstacles',
    ]
    for key in required:
        if key not in doc:
            errors.append(f'missing field: {key}')

    try:
        width = float(doc['size']['width'])
        depth = float(doc['size']['depth'])
        height = float(doc['size']['height'])
        if width <= 0 or depth <= 0 or height <= 0:
            errors.append('warehouse size must be positive')
    except (TypeError, ValueError):
        width = depth = height = 0.0
        errors.append('invalid size')

    try:
        cs = float(doc['grid']['cell_size'])
        cols = int(doc['grid']['cols'])
        rows = int(doc['grid']['rows'])
        if cs <= 0 or cols <= 0 or rows <= 0:
            errors.append('grid cell_size/cols/rows must be positive')
        elif width > 0 and depth > 0:
            expected_cols = max(1, int(round(width / cs)))
            expected_rows = max(1, int(round(depth / cs)))
            if cols != expected_cols or rows != expected_rows:
                errors.append(
                    f'grid does not match warehouse size: expected {expected_cols}x{expected_rows}, got {cols}x{rows}'
                )
    except (TypeError, ValueError, KeyError):
        errors.append('invalid grid')

    ids: set[str] = set()
    for key in ('zones', 'racks', 'conveyors', 'stations', 'charging_stations', 'docks', 'lifts', 'obstacles'):
        for obj in doc.get(key, []):
            oid = str(obj.get('id') or '').strip()
            if not oid:
                errors.append(f'{key}: missing id')
            elif oid in ids:
                errors.append(f'duplicate object id: {oid}')
            else:
                ids.add(oid)

    zone_ids = {str(z.get('id')) for z in doc.get('zones', []) if z.get('id')}
    for z in doc.get('zones', []):
        zid = str(z.get('id') or '?')
        polygon = z.get('polygon') or []
        if len(polygon) < 3:
            errors.append(f'zone {zid}: polygon must contain at least 3 points')
            continue
        for point in polygon:
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                errors.append(f'zone {zid}: invalid polygon point')
                continue
            try:
                x, y = float(point[0]), float(point[1])
            except (TypeError, ValueError):
                errors.append(f'zone {zid}: polygon coordinates must be numeric')
                continue
            if width > 0 and depth > 0 and not (0 <= x <= width and 0 <= y <= depth):
                errors.append(f'zone {zid}: polygon point ({x:g},{y:g}) is outside warehouse bounds')

    zone_bound_collections = (
        'racks', 'conveyors', 'stations', 'charging_stations',
        'docks', 'parking', 'cameras', 'sensors', 'locations',
    )
    for key in zone_bound_collections:
        for obj in doc.get(key, []):
            zid = obj.get('zone')
            if zid and str(zid) not in zone_ids:
                errors.append(f"{key} {obj.get('id', '?')}: unknown zone {zid}")

    rack_ids: set[str] = set()
    for rack in doc.get('racks', []):
        rid = str(rack.get('id') or '?')
        rack_ids.add(rid)
        try:
            x, y, z = map(float, rack.get('position') or [])
            w, h, d = map(float, rack.get('size') or [])
        except (TypeError, ValueError):
            errors.append(f'rack {rid}: invalid position/size')
            continue
        if w <= 0 or h <= 0 or d <= 0:
            errors.append(f'rack {rid}: size must be positive')
            continue
        angle = math.radians(float(rack.get('rotation') or 0))
        cx, cz = x + w / 2, z + d / 2
        c, s = math.cos(angle), math.sin(angle)
        corners = []
        for px, pz in ((x, z), (x + w, z), (x + w, z + d), (x, z + d)):
            dx, dz = px - cx, pz - cz
            corners.append((cx + dx * c + dz * s, cz - dx * s + dz * c))
        if width > 0 and depth > 0:
            min_x = min(p[0] for p in corners); max_x = max(p[0] for p in corners)
            min_z = min(p[1] for p in corners); max_z = max(p[1] for p in corners)
            if min_x < -1e-6 or min_z < -1e-6 or max_x > width + 1e-6 or max_z > depth + 1e-6:
                errors.append(f'rack {rid}: rotated footprint is outside warehouse bounds')
        if y < -1e-6 or y + h > height + 1e-6:
            errors.append(f'rack {rid}: vertical extent is outside warehouse height')

    for loc in doc.get('locations', []):
        if str(loc.get('kind') or '').upper() == 'SHELF' and loc.get('rack_id') and str(loc['rack_id']) not in rack_ids:
            errors.append(f"location {loc.get('id', '?')}: unknown rack {loc.get('rack_id')}")
        ap = loc.get('access_point')
        if isinstance(ap, (list, tuple)) and len(ap) == 2 and width > 0 and depth > 0:
            try:
                ax, ay = float(ap[0]), float(ap[1])
                if not (0 <= ax <= width and 0 <= ay <= depth):
                    errors.append(f"location {loc.get('id', '?')}: access point is outside warehouse bounds")
            except (TypeError, ValueError):
                errors.append(f"location {loc.get('id', '?')}: invalid access point")

    return errors


@csrf_exempt
@api_admin_required
@require_http_methods(['POST'])
def layout_validate(request):
    errors = _validate_layout_doc(_body(request))
    return JsonResponse({'valid': not errors, 'errors': errors})


def _layout_revision_conflict(request, active):
    raw = request.headers.get('X-Layout-Revision')
    if not raw:
        return None
    try:
        expected = int(raw)
    except ValueError:
        return _error('X-Layout-Revision must be an integer')
    if expected != active.revision:
        return _error(
            f'layout changed by another session (expected r{expected}, current r{active.revision}); reload before saving',
            409,
            {'current_revision': active.revision},
        )
    return None


@csrf_exempt
@api_admin_required
@require_http_methods(['GET', 'PUT'])
def layout_draft(request):
    active = ensure_active_map(runtime.layout)
    if request.method == 'GET':
        # The editor always starts from the shared authoritative map.
        response = JsonResponse(canonicalize_layout(active.layout))
        response['X-Warehouse-Id'] = str(active.warehouse_id)
        response['X-Layout-Revision'] = str(active.revision)
        response['X-Layout-Version'] = str(active.published_version)
        return response

    conflict = _layout_revision_conflict(request, active)
    if conflict:
        return conflict
    body = canonicalize_layout(_body(request), active.layout)
    errors = _validate_layout_doc(body)
    if errors:
        return _error('; '.join(errors[:10]))
    try:
        active = save_layout_to_map(body, user=request.api_user, fallback_layout=runtime.layout)
    except IntegrityError as exc:
        return _error(f'layout/master-data conflict: {exc}', 409)
    _audit(request.api_user, 'SAVE_AND_SYNC_LAYOUT', str(body.get('id', 'layout')))
    _broadcast_layout_update(active, 'WAREHOUSE_EDITOR_SAVE')
    return JsonResponse({
        'ok': True, 'status': 'SYNCED', 'layout_id': body.get('id'),
        'revision': active.revision, 'published_version': active.published_version,
        'warehouse_id': active.warehouse_id,
    })


@csrf_exempt
@api_admin_required
@require_http_methods(['POST'])
def layout_publish(request):
    current = ensure_active_map(runtime.layout)
    conflict = _layout_revision_conflict(request, current)
    if conflict:
        return conflict
    body = canonicalize_layout(_body(request), current.layout)
    errors = _validate_layout_doc(body)
    if errors:
        return _error('; '.join(errors[:10]))
    try:
        active = publish_layout_to_map(body, user=request.api_user, fallback_layout=runtime.layout)
    except IntegrityError as exc:
        return _error(f'layout/master-data conflict: {exc}', 409)
    except ValueError as exc:
        return _error(str(exc), 400)
    except Exception as exc:
        # Artifact generation is part of publishing.  Do not report a
        # successful DB publish when an external artifact failed.
        log.exception('Map publish failed while generating artifacts: %s', type(exc).__name__)
        return _error(f'publish artifacts failed: {exc}', 500)
    _audit(request.api_user, 'PUBLISH_LAYOUT', f"{body.get('name', 'warehouse')}@{active.published_version}")
    _broadcast_layout_update(active, 'WAREHOUSE_EDITOR_PUBLISH')
    map_payload = published_map_payload(active)
    async_to_sync(runtime.map_published)(map_payload)
    return JsonResponse({
        'ok': True, 'status': 'PUBLISHED', 'version': active.published_version,
        'published_version': active.published_version, 'revision': active.revision,
        'warehouse_id': active.warehouse_id,
        'artifacts': map_payload.get('artifact_manifest') or getattr(active, '_artifact_manifest', None),
        'simulation_reset': True,
    })


@api_user_required
@require_http_methods(['GET'])
def map_sync_status_view(request):
    active = ensure_active_map(runtime.layout)
    payload = published_map_payload(active)
    runtime._aggregate_robot_map_sync()
    require_nav2, require_tag_map = runtime.map_sync_requirements()
    status = map_sync_status(
        published_revision=payload.get('map_revision'),
        ros_revision=runtime.ros_map_revision,
        gazebo_revision=runtime.gazebo_map_revision,
        nav2_revision=runtime.nav2_map_revision,
        tag_map_revision=runtime.tag_map_revision,
        tf_status=runtime.map_tf_status,
        require_nav2=require_nav2,
        require_tag_map=require_tag_map,
        ros_connected=runtime.ros_bridge_connected,
        error=runtime.map_sync_error,
        external=runtime.is_external,
    )
    return JsonResponse({
        'warehouse_id': payload.get('warehouse_id'),
        'published_revision': payload.get('map_revision'),
        'published_version': payload.get('published_version'),
        'map_revision': payload.get('map_revision'),
        'ros_revision': runtime.ros_map_revision,
        'gazebo_revision': runtime.gazebo_map_revision,
        'nav2_revision': runtime.nav2_map_revision,
        'tag_map_revision': runtime.tag_map_revision,
        'tf_status': runtime.map_tf_status,
        'robot_map_sync': {
            rid: {key: value for key, value in row.items() if key != 'received_monotonic'}
            for rid, row in runtime.robot_map_sync.items()
        },
        'status': status,
        'artifact_manifest': payload.get('artifact_manifest'),
        'error': runtime.map_sync_error,
    })


@api_admin_required
@require_http_methods(['GET'])
def layout_versions(request):
    active = ensure_active_map(runtime.layout)
    from .map_artifacts import artifact_revision_dir
    return JsonResponse([
        {
            'version': row.version,
            'revision': row.revision,
            'created_at': row.created_at.isoformat(),
            'created_by': row.created_by.username if row.created_by else None,
            'artifacts': str(artifact_revision_dir(active.warehouse.code, row.revision))
            if artifact_revision_dir(active.warehouse.code, row.revision).is_dir() else None,
        }
        for row in active.versions.all()[:100]
    ], safe=False)

def kpi(request):
    return JsonResponse(runtime.engine.state['kpi'])


def decisions(request):
    limit = min(int(request.GET.get('limit', '20')), 200)
    return JsonResponse(runtime.engine.state['recent_decisions'][:limit], safe=False)


def events(request):
    limit = min(int(request.GET.get('limit', '200')), 2000)
    types = set(request.GET.getlist('type'))
    severities = set(request.GET.getlist('severity'))
    robot_id = request.GET.get('robot_id')
    zone_id = request.GET.get('zone_id')
    since_tick = request.GET.get('since_tick')
    since_tick_i = int(since_tick) if since_tick and since_tick.isdigit() else None

    q = EventLog.objects.filter(run_id=runtime.run_id)
    if types:
        q = q.filter(type__in=types)
    if severities:
        q = q.filter(severity__in=severities)
    if robot_id:
        q = q.filter(robot_id=robot_id)
    if zone_id:
        q = q.filter(zone_id=zone_id)
    if since_tick_i is not None:
        q = q.filter(tick__gte=since_tick_i)
    persisted = [e.to_twin_dict() for e in q[:limit]]

    live = []
    for e in runtime.engine.state.get('recent_events', []):
        if types and e.get('type') not in types: continue
        if severities and e.get('severity') not in severities: continue
        if robot_id and e.get('robot_id') != robot_id: continue
        if zone_id and e.get('zone_id') != zone_id: continue
        if since_tick_i is not None and int(e.get('tick', 0)) < since_tick_i: continue
        live.append(e)
    merged = {e.get('id'): e for e in persisted}
    for e in live:
        merged[e.get('id')] = e
    result = sorted(merged.values(), key=lambda e: int(e.get('tick', 0)), reverse=True)[:limit]
    return JsonResponse(result, safe=False)


@csrf_exempt
@api_admin_required
@require_http_methods(['POST'])
def inject(request):
    try:
        inj = inject_adapter.validate_python(_body(request))
    except ValidationError as exc:
        return _error(str(exc)[:300])
    runtime.engine.inject(inj.model_dump(exclude_none=True))
    _audit(request.api_user, 'INJECT_SCENARIO', str(inj.kind))
    return JsonResponse({'ok': True})


@csrf_exempt
@api_admin_required
@require_http_methods(['POST'])
def inject_clear(request):
    d = _body(request)
    kind, target_id = d.get('kind'), d.get('target_id')
    if not kind or not target_id:
        return _error('kind and target_id are required')
    runtime.engine.clear_injection(kind, target_id)
    _audit(request.api_user, 'CLEAR_SCENARIO', str(target_id))
    return JsonResponse({'ok': True})


@csrf_exempt
@api_admin_required
@require_http_methods(['POST'])
def tasks(request):
    d = _body(request)
    try:
        task = runtime.engine.create_task(d.get('type', 'PICK'), d.get('priority', 'NORMAL'), d['source'], d['destination'], int(d.get('load_units', 1)))
        if d.get('deadline_s') is not None:
            task['deadline_tick'] = runtime.engine.state['sim']['tick'] + int(float(d['deadline_s']) * 10)
        if d.get('robot_id'):
            runtime.engine.assign_task(task['id'], d['robot_id'], source='USER')
    except (KeyError, ValueError) as exc:
        return _error(str(exc), 409)
    _audit(request.api_user, 'CREATE_TASK', task['id'])
    return JsonResponse(task, status=201)


@csrf_exempt
@api_admin_required
@require_http_methods(['POST'])
def task_assign(request, task_id: str):
    robot_id = _body(request).get('robot_id')
    if not robot_id:
        return _error('robot_id required')
    try:
        task = runtime.engine.assign_task(task_id, robot_id, source='USER')
    except ValueError as exc:
        return _error(str(exc), 409)
    _audit(request.api_user, 'ASSIGN_TASK', f'{task_id}->{robot_id}')
    return JsonResponse(task)


@csrf_exempt
@api_user_required
@require_http_methods(['POST'])
def copilot(request):
    question = str(_body(request).get('question', '')).strip()
    if not question:
        return _error('question required')
    snapshot = json.loads(json.dumps(runtime.engine.state))
    return JsonResponse(copilot_ai.answer(question, snapshot, runtime.layout))


@csrf_exempt
@api_user_required
@require_http_methods(['POST'])
def vlm_observe(request):
    d = _body(request)
    camera_id = d.get('camera_id')
    if camera_id not in runtime.engine.state['cameras']:
        return _error('unknown camera', 404)
    if runtime.engine.state['cameras'][camera_id]['status'] == 'OFFLINE':
        return _error('camera offline', 409)
    snapshot = json.loads(json.dumps(runtime.engine.state))
    obs = vlm_ai.observe(camera_id, d.get('image_b64'), snapshot, runtime.layout)
    runtime.engine.state['cameras'][camera_id]['last_observation'] = obs
    severity = obs['severity'] if obs['event'] != 'none' else 'INFO'
    runtime.engine.emit('VLM_OBSERVATION', 'VLM', severity, f"{camera_id}: {obs['event'].replace('_', ' ')} ({obs['confidence']:.0%}) — {obs.get('description', '')}", camera_id=camera_id, zone_id=obs['zone'], payload={'confidence': obs['confidence'], 'raw': obs.get('raw')})
    return JsonResponse(obs)


@csrf_exempt
@api_user_required
@require_http_methods(['POST'])
def whatif(request):
    try:
        req = WhatIfRequest.model_validate(_body(request)).model_dump(exclude_none=True)
    except ValidationError as exc:
        return _error(str(exc)[:300])
    start_tick = runtime.engine.state['sim']['tick']
    base, scen = runtime.engine.clone(), runtime.engine.clone()
    base.external_scheduler = False
    scen.external_scheduler = False
    result = run_whatif(base, scen, req, start_tick)
    return JsonResponse(result)


def ai_status(request):
    return JsonResponse({
        'llm': bool(os.getenv('OPENAI_API_KEY')),
        'model': os.getenv('OPENAI_MODEL', 'rule-based'),
        'vision_model': os.getenv('OPENAI_VISION_MODEL', 'simulated'),
        'vlm_acts': os.getenv('TWIN_VLM_ACTS', '0') == '1',
    })


@csrf_exempt
@api_user_required
@require_http_methods(['POST'])
def sim(request):
    d = _body(request)
    action = d.get('action')
    if action == 'PAUSE':
        runtime.paused = True
    elif action == 'PLAY':
        runtime.paused = False
    elif action == 'RESET':
        runtime.reset(d.get('seed'))
    else:
        return _error('action must be PLAY, PAUSE or RESET')
    if d.get('speed') is not None:
        speed = int(d['speed'])
        if speed not in (0, 1, 2, 5, 10):
            return _error('invalid speed')
        runtime.speed = speed
        runtime.paused = speed == 0
    _audit(request.api_user, f'SIM_{action}', None)
    return health(request)
