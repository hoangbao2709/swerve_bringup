from __future__ import annotations

import math
from typing import Any

from asgiref.sync import async_to_sync
from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt
from django.views.decorators.http import require_http_methods

from .auth import user_from_request
from .models import EventLog, RobotNavigationMission
from .runtime import runtime
from .tag_navigation import active_warehouse, create_mission, current_mission, get_tag_graph, mission_snapshot, shortest_tag_route, validate_target_tag
from .views import _body, _error


def _auth(view):
    def wrapped(request, *args, **kwargs):
        user = user_from_request(request)
        if user is None: return _error('authentication required', 401)
        request.api_user = user
        return view(request, *args, **kwargs)
    return wrapped


@_auth
@require_http_methods(['GET'])
def tags(request):
    return JsonResponse(get_tag_graph(request.GET.get('warehouse_id')).get('tags', []), safe=False)


@_auth
@require_http_methods(['GET'])
def tag_graph(request):
    try: return JsonResponse(get_tag_graph(request.GET.get('warehouse_id')))
    except Exception as exc: return _error(str(exc), 404)


@_auth
@require_http_methods(['GET'])
def missions(request):
    qs = RobotNavigationMission.objects.select_related('target_tag').all()
    robot_id = request.GET.get('robot_id')
    if robot_id: qs = qs.filter(robot_id=robot_id)
    return JsonResponse([mission_snapshot(m) for m in qs[:100]], safe=False)


@_auth
@require_http_methods(['GET'])
def current(request):
    m = current_mission(request.GET.get('robot_id', '')) if request.GET.get('robot_id') else None
    return JsonResponse(mission_snapshot(m) if m else None, safe=False)


def _nearest_tag(robot_id: str, warehouse_id: int | None = None) -> int | None:
    from .models import NavigationTag
    wh = active_warehouse(warehouse_id); robot = runtime.engine.state.get('robots', {}).get(robot_id)
    if not wh or not robot: return None
    x, y = robot.get('position', [0, 0, 0])[0], robot.get('position', [0, 0, 0])[2]
    return min(NavigationTag.objects.filter(warehouse=wh, enabled=True), key=lambda t: math.hypot(t.x-x, t.y-y), default=None).tag_id if NavigationTag.objects.filter(warehouse=wh, enabled=True).exists() else None


def _map_sync_error(robot_id: str | None = None) -> str | None:
    if not runtime.is_external:
        return None
    if robot_id and runtime.local_map_overrides.get(str(robot_id)):
        return f'Cannot start fleet mission: {robot_id} is using a local-only map'
    status = runtime.runtime_status_message().get('map_sync_status')
    if status != 'SYNCED':
        return f'Cannot start mission: map revision mismatch ({status})'
    return None


@csrf_exempt
@_auth
@require_http_methods(['POST'])
def start(request):
    body = _body(request)
    if any(key in body for key in ('x', 'y', 'yaw')): return _error('coordinates are not accepted; use target_tag_id')
    robot_id = str(body.get('robot_id', '')).strip()
    if not robot_id or body.get('target_tag_id') is None: return _error('robot_id and target_tag_id are required')
    if not runtime.ros_bridge_connected and runtime.is_external: return _error('ROS bridge is offline', 409)
    sync_error = _map_sync_error(robot_id)
    if sync_error: return _error(sync_error, 409)
    try:
        wh_id = int(body['warehouse_id']) if body.get('warehouse_id') else None
        current_tag = body.get('current_tag_id') or _nearest_tag(robot_id, wh_id)
        mission = create_mission(robot_id, int(body['target_tag_id']), wh_id, int(current_tag) if current_tag is not None else None)
        result = async_to_sync(runtime.tag_command)('GO_TO_TAG', mission, request.api_user)
        if not result.get('ok') and runtime.is_external: return _error('ROS bridge is offline', 503)
        return JsonResponse(mission_snapshot(mission), status=201)
    except (ValueError, TypeError) as exc: return _error(str(exc))


def _action(request, mission_id: int, action: str, status: str):
    try: mission = RobotNavigationMission.objects.select_related('target_tag').get(pk=mission_id)
    except RobotNavigationMission.DoesNotExist: return _error('mission not found', 404)
    if action in ('RESUME_TAG_NAVIGATION', 'REPLAN_TAG_NAVIGATION'):
        sync_error = _map_sync_error(mission.robot_id)
        if sync_error: return _error(sync_error, 409)
    if action == 'REPLAN_TAG_NAVIGATION':
        try:
            mission.route = shortest_tag_route(mission.current_tag_id, mission.target_tag.tag_id, mission.warehouse_id)
            mission.route_index = 0; mission.next_tag_id = mission.route[1] if len(mission.route) > 1 else mission.target_tag.tag_id
        except (ValueError, TypeError) as exc: return _error(str(exc))
    previous = {
        'status': mission.status,
        'route': mission.route,
        'route_index': mission.route_index,
        'next_tag_id': mission.next_tag_id,
    }
    mission.status = status; mission.save(update_fields=['status', 'route', 'route_index', 'next_tag_id', 'updated_at'])
    result = async_to_sync(runtime.tag_command)(action, mission, request.api_user)
    if runtime.is_external and not result.get('ok'):
        # Do not leave a persisted mission in PAUSED/CANCELLED/REPLAN state
        # when ROS never accepted the command. Restore the exact DB state and
        # expose a machine-readable failure to the browser.
        mission.status = previous['status']
        mission.route = previous['route']
        mission.route_index = previous['route_index']
        mission.next_tag_id = previous['next_tag_id']
        mission.save(update_fields=['status', 'route', 'route_index', 'next_tag_id', 'updated_at'])
        async_to_sync(runtime.broadcast)({'type': 'TAG_NAV_STATUS', **mission_snapshot(mission)})
        return _error('ROS bridge is offline; mission state was not changed', 503)
    async_to_sync(runtime.broadcast)({'type': 'TAG_NAV_STATUS', **mission_snapshot(mission)})
    return JsonResponse(mission_snapshot(mission) | {'command': result})


@csrf_exempt
@_auth
@require_http_methods(['POST'])
def mission_action(request, mission_id: int, action: str):
    mapping = {'pause': ('PAUSE_TAG_NAVIGATION', 'PAUSED'), 'resume': ('RESUME_TAG_NAVIGATION', 'NAVIGATING'),
               'cancel': ('CANCEL_TAG_NAVIGATION', 'CANCELLED'), 'replan': ('REPLAN_TAG_NAVIGATION', 'PLANNING')}
    if action not in mapping: return _error('unsupported mission action')
    return _action(request, mission_id, *mapping[action])


@csrf_exempt
@_auth
@require_http_methods(['POST'])
def emergency_stop(request, robot_id: str):
    mission = current_mission(robot_id)
    result = async_to_sync(runtime.gateway().send_command)(robot_id, 'EMERGENCY_STOP', {'mission_id': mission.id if mission else None})
    if not result.get('ok'):
        runtime.engine.emit('EMERGENCY_STOP_FAILED', 'USER', 'CRITICAL', f'Emergency stop failed: ROS bridge offline for {robot_id}', robot_id=robot_id)
        return _error('ROS bridge is offline; emergency stop was not acknowledged', 503)
    if mission: mission.status = 'EMERGENCY_STOPPED'; mission.save(update_fields=['status', 'updated_at'])
    runtime.engine.emit('EMERGENCY_STOP', 'USER', 'CRITICAL', f'Emergency stop {robot_id}', robot_id=robot_id)
    return JsonResponse({'ok': result.get('ok', False), 'mission': mission_snapshot(mission) if mission else None})


@csrf_exempt
@_auth
@require_http_methods(['POST'])
def clear_emergency_stop(request, robot_id: str):
    """Clear the ROS stop latch without resuming a mission automatically."""
    result = async_to_sync(runtime.gateway().send_command)(robot_id, 'CLEAR_EMERGENCY_STOP', {})
    if not result.get('ok'):
        return _error('ROS bridge is offline; emergency stop remains active', 503)
    runtime.engine.emit('EMERGENCY_STOP_CLEARED', 'USER', 'HIGH', f'Emergency stop cleared for {robot_id}', robot_id=robot_id)
    return JsonResponse({'ok': True, 'robot_id': robot_id, 'mission': mission_snapshot(current_mission(robot_id)) if current_mission(robot_id) else None})


@_auth
@require_http_methods(['GET'])
def mission_events(request, mission_id: int):
    rows = EventLog.objects.filter(run_id=f'tag-{mission_id}')[:100]
    return JsonResponse([e.to_twin_dict() for e in rows], safe=False)
