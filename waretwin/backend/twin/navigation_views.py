from __future__ import annotations

import math
from typing import Any

from asgiref.sync import async_to_sync
from django.db import transaction
from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt
from django.views.decorators.http import require_http_methods

from .models import EventLog, RobotMapRegistration, RobotNavigationMission, WarehouseMap
from .navigation_targets import navigation_tag_registry
from .runtime import runtime
from .tag_navigation import active_warehouse, create_mission, current_mission, get_tag_graph, mission_snapshot, shortest_tag_route, validate_target_tag
from .views import _body, _error


@require_http_methods(['GET'])
def tags(request):
    return JsonResponse(get_tag_graph(request.GET.get('warehouse_id')).get('tags', []), safe=False)


@require_http_methods(['GET'])
def robot_tags(request, robot_id: str):
    active_map = runtime.active_map_state(robot_id)
    registry = navigation_tag_registry(active_map, robot_id=robot_id)
    return JsonResponse({'robot_id': robot_id, **registry})


@csrf_exempt
@require_http_methods(['POST'])
def register_robot_map(request, robot_id: str):
    """Persist an audited canonical->active-map SE(2) registration.

    The request must name the exact live map identities. A later SLAM map or
    canonical map revision cannot inherit this transform accidentally.
    """
    body = _body(request)
    active = runtime.active_map_state(robot_id)
    active_map_id = str(active.get('active_map_id') or '')
    active_map_revision = str(active.get('active_map_revision') or '')
    if (not active_map_id or not active_map_revision
            or active.get('map_sync_status') != 'LOCAL_ONLY'
            or active_map_id == 'CANONICAL'):
        return _error('robot must have a confirmed active local/SLAM map', 409,
                      {'code': 'ACTIVE_LOCAL_MAP_REQUIRED'})
    if (str(body.get('active_map_id') or '') != active_map_id
            or str(body.get('active_map_revision') or '') != active_map_revision):
        return _error('registration must identify the exact active map and revision', 409,
                      {'code': 'ACTIVE_MAP_REVISION_MISMATCH'})
    try:
        canonical_revision = int(body.get('canonical_map_revision'))
    except (TypeError, ValueError):
        return _error('canonical_map_revision must be an integer', 400,
                      {'code': 'CANONICAL_MAP_REVISION_REQUIRED'})
    if canonical_revision != active.get('canonical_map_revision'):
        return _error('canonical revision does not match the robot map baseline', 409,
                      {'code': 'CANONICAL_MAP_REVISION_MISMATCH'})
    warehouse_map = WarehouseMap.objects.filter(is_active=True).order_by('id').first()
    if warehouse_map is None or warehouse_map.revision != canonical_revision:
        return _error('canonical map revision is not the active warehouse revision', 409,
                      {'code': 'CANONICAL_MAP_REVISION_MISMATCH'})
    try:
        transform = {key: float(body[key]) for key in ('tx', 'ty', 'yaw')}
    except (KeyError, TypeError, ValueError):
        return _error('tx, ty and yaw are required finite transform values', 400,
                      {'code': 'REGISTRATION_TRANSFORM_INVALID'})
    if not all(math.isfinite(value) for value in transform.values()):
        return _error('tx, ty and yaw must be finite', 400,
                      {'code': 'REGISTRATION_TRANSFORM_INVALID'})
    source = str(body.get('source') or '').strip()
    if not source or len(source) > 160:
        return _error('registration source is required and must be at most 160 characters', 400,
                      {'code': 'REGISTRATION_SOURCE_REQUIRED'})
    transform['yaw'] = math.atan2(math.sin(transform['yaw']), math.cos(transform['yaw']))

    with transaction.atomic():
        warehouse_map = WarehouseMap.objects.select_for_update().filter(
            pk=warehouse_map.pk, is_active=True, revision=canonical_revision).first()
        if warehouse_map is None:
            return _error('canonical map changed while the registration was being saved', 409,
                          {'code': 'CANONICAL_MAP_REVISION_MISMATCH'})
        prior = RobotMapRegistration.objects.select_for_update().filter(
            robot_id=robot_id, active_map_id=active_map_id,
            active_map_revision=active_map_revision)
        registration_revision = (prior.order_by('-registration_revision')
                                 .values_list('registration_revision', flat=True).first() or 0) + 1
        prior.filter(is_active=True).update(is_active=False)
        registration = RobotMapRegistration.objects.create(
            robot_id=robot_id, warehouse_map=warehouse_map,
            canonical_revision=canonical_revision,
            active_map_id=active_map_id, active_map_revision=active_map_revision,
            tx=transform['tx'], ty=transform['ty'], yaw=transform['yaw'],
            registration_revision=registration_revision, source=source,
            created_by=None,
        )
    runtime.invalidate_registration_path_previews(
        robot_id, 'canonical-to-active map registration revision changed')
    return JsonResponse({
        'robot_id': robot_id,
        'canonical_map_id': 'CANONICAL',
        'canonical_map_revision': canonical_revision,
        'active_map_id': active_map_id,
        'active_map_revision': active_map_revision,
        'tx': registration.tx, 'ty': registration.ty, 'yaw': registration.yaw,
        'registration_revision': registration.registration_revision,
        'source': registration.source,
        'is_active': registration.is_active,
    }, status=201)


@require_http_methods(['GET'])
def tag_graph(request):
    """Serve the graph derived from the exact active published map revision.

    The legacy NavigationTagEdge table is retained for historical missions,
    but it is not the authority for current autonomous routing: published
    aisle geometry is. Returning its generated graph here keeps diagnostics
    and operator tooling aligned with route previews and the published bundle.
    """
    try:
        maps = WarehouseMap.objects.filter(is_active=True).select_related('warehouse')
        warehouse_id = request.GET.get('warehouse_id')
        if warehouse_id:
            maps = maps.filter(warehouse_id=int(warehouse_id))
        warehouse_map = maps.order_by('id').first()
        if warehouse_map is None:
            return _error('no active published warehouse map is available', 404,
                          {'code': 'PUBLISHED_MAP_UNAVAILABLE'})
        from .navigation_graph import prepare_published_navigation
        published = prepare_published_navigation(warehouse_map.layout, warehouse_map.revision)
        return JsonResponse({
            'warehouse_id': warehouse_map.warehouse_id,
            'canonical_revision': warehouse_map.revision,
            'graph_revision': published['tag_graph_revision'],
            'frame_id': 'map',
            'units': 'm',
            'tags': published.get('navigation_tags', []),
            'edges': published.get('navigation_edges', []),
        })
    except (TypeError, ValueError) as exc:
        return _error(str(exc), 400, {'code': 'TAG_GRAPH_INVALID'})
    except Exception as exc:
        return _error(str(exc), 404)


@require_http_methods(['GET'])
def missions(request):
    qs = RobotNavigationMission.objects.select_related('target_tag').all()
    robot_id = request.GET.get('robot_id')
    if robot_id: qs = qs.filter(robot_id=robot_id)
    return JsonResponse([mission_snapshot(m) for m in qs[:100]], safe=False)


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
@require_http_methods(['POST'])
def start(request):
    body = _body(request)
    if any(key in body for key in ('x', 'y', 'yaw')): return _error('coordinates are not accepted; use target_tag_id')
    robot_id = str(body.get('robot_id', '')).strip()
    if not robot_id or body.get('target_tag_id') is None: return _error('robot_id and target_tag_id are required')
    robot_mode = str(runtime.robot_runtime_modes.get(robot_id) or runtime.operation_mode).upper()
    if runtime.is_external or robot_mode == 'UNIFIED':
        return _error('Tag navigation must use the shared NavigationTarget preview and Nav2 pipeline', 409,
                      {'code': 'TAG_NAVIGATION_SINGLE_PIPELINE_REQUIRED'})
    if not runtime.ros_bridge_connected and runtime.is_external: return _error('ROS bridge is offline', 409)
    sync_error = _map_sync_error(robot_id)
    if sync_error: return _error(sync_error, 409)
    try:
        wh_id = int(body['warehouse_id']) if body.get('warehouse_id') else None
        current_tag = body.get('current_tag_id') or _nearest_tag(robot_id, wh_id)
        mission = create_mission(robot_id, int(body['target_tag_id']), wh_id, int(current_tag) if current_tag is not None else None)
        result = async_to_sync(runtime.tag_command)('GO_TO_TAG', mission, None)
        if not result.get('ok') and runtime.is_external: return _error('ROS bridge is offline', 503)
        return JsonResponse(mission_snapshot(mission), status=201)
    except (ValueError, TypeError) as exc: return _error(str(exc))


def _action(request, mission_id: int, action: str, status: str):
    try: mission = RobotNavigationMission.objects.select_related('target_tag').get(pk=mission_id)
    except RobotNavigationMission.DoesNotExist: return _error('mission not found', 404)
    robot_mode = str(runtime.robot_runtime_modes.get(mission.robot_id) or runtime.operation_mode).upper()
    if (runtime.is_external or robot_mode == 'UNIFIED') \
            and action in ('RESUME_TAG_NAVIGATION', 'REPLAN_TAG_NAVIGATION'):
        return _error('Tag navigation must use the shared NavigationTarget preview and Nav2 pipeline', 409,
                      {'code': 'TAG_NAVIGATION_SINGLE_PIPELINE_REQUIRED'})
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
    result = async_to_sync(runtime.tag_command)(action, mission, None)
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
@require_http_methods(['POST'])
def mission_action(request, mission_id: int, action: str):
    mapping = {'pause': ('PAUSE_TAG_NAVIGATION', 'PAUSED'), 'resume': ('RESUME_TAG_NAVIGATION', 'NAVIGATING'),
               'cancel': ('CANCEL_TAG_NAVIGATION', 'CANCELLED'), 'replan': ('REPLAN_TAG_NAVIGATION', 'PLANNING')}
    if action not in mapping: return _error('unsupported mission action')
    return _action(request, mission_id, *mapping[action])


@csrf_exempt
@require_http_methods(['POST'])
def emergency_stop(request, robot_id: str):
    mission = current_mission(robot_id)
    result = async_to_sync(runtime.gateway().send_command)(robot_id, 'EMERGENCY_STOP', {'mission_id': mission.id if mission else None})
    if not result.get('ok'):
        runtime.engine.emit('EMERGENCY_STOP_FAILED', 'LOCAL', 'CRITICAL', f'Emergency stop failed: ROS bridge offline for {robot_id}', robot_id=robot_id)
        return _error('ROS bridge is offline; emergency stop was not acknowledged', 503)
    if mission: mission.status = 'EMERGENCY_STOPPED'; mission.save(update_fields=['status', 'updated_at'])
    runtime.engine.emit('EMERGENCY_STOP', 'LOCAL', 'CRITICAL', f'Emergency stop {robot_id}', robot_id=robot_id)
    return JsonResponse({'ok': result.get('ok', False), 'mission': mission_snapshot(mission) if mission else None})


@csrf_exempt
@require_http_methods(['POST'])
def clear_emergency_stop(request, robot_id: str):
    """Clear the ROS stop latch without resuming a mission automatically."""
    result = async_to_sync(runtime.gateway().request_control)(
        robot_id, 'CLEAR_ESTOP', {}, timeout=5.0)
    applied = result.get('result') if isinstance(result.get('result'), dict) else {}
    code = str(applied.get('code') or '')
    if not result.get('ok'):
        if code == 'CLEAR_ESTOP_REJECTED_GOAL_PENDING':
            status = 409
            message = str(applied.get('message') or 'E-STOP remains active until navigation is terminal')
        elif 'timed out' in str(result.get('error') or '').lower():
            code = 'CLEAR_ESTOP_TIMEOUT'
            status = 504
            message = 'bridge did not confirm that the E-STOP latch was cleared before the timeout'
        else:
            code = code or 'CLEAR_ESTOP_UNAVAILABLE'
            status = 503
            message = str(result.get('error') or 'ROS bridge did not acknowledge the E-STOP clear request')
        return JsonResponse({'ok': False, 'code': code, 'error': message,
            'emergency_stop_active': applied.get('emergency_stop_active', True),
            'pre_stop_navigation_terminal': applied.get('pre_stop_navigation_terminal', False)}, status=status)
    if (code != 'CLEAR_ESTOP_APPLIED' or applied.get('emergency_stop_active') is not False
            or applied.get('pre_stop_navigation_terminal') is not True):
        return JsonResponse({'ok': False, 'code': 'CLEAR_ESTOP_UNCONFIRMED',
            'error': 'bridge response did not confirm an applied clear and terminal pre-stop navigation',
            'emergency_stop_active': applied.get('emergency_stop_active', True)}, status=502)
    runtime.engine.emit('EMERGENCY_STOP_CLEARED', 'LOCAL', 'HIGH', f'Emergency stop cleared for {robot_id}', robot_id=robot_id)
    return JsonResponse({'ok': True, 'code': code, 'robot_id': robot_id,
        'emergency_stop_active': False, 'pre_stop_navigation_terminal': True,
        'message': applied.get('message'),
        'mission': mission_snapshot(current_mission(robot_id)) if current_mission(robot_id) else None})


@require_http_methods(['GET'])
def mission_events(request, mission_id: int):
    rows = EventLog.objects.filter(run_id=f'tag-{mission_id}')[:100]
    return JsonResponse([e.to_twin_dict() for e in rows], safe=False)
