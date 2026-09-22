from __future__ import annotations

import heapq
import uuid
from typing import Any

from django.db import transaction
from django.utils import timezone

from .models import EventLog, NavigationTag, NavigationTagEdge, RobotNavigationMission, Warehouse


def active_warehouse(warehouse_id: int | None = None) -> Warehouse:
    qs = Warehouse.objects.filter(status='ACTIVE')
    if warehouse_id:
        return qs.get(pk=warehouse_id)
    return qs.order_by('id').first() or Warehouse.objects.order_by('id').first()


def get_tag_graph(warehouse_id: int | None = None) -> dict[str, Any]:
    wh = active_warehouse(warehouse_id)
    if wh is None:
        return {'warehouse_id': None, 'tags': [], 'edges': []}
    tags = list(NavigationTag.objects.filter(warehouse=wh, enabled=True))
    edges = list(NavigationTagEdge.objects.filter(warehouse=wh, enabled=True).select_related('from_tag', 'to_tag'))
    return {
        'warehouse_id': wh.id,
        'tags': [
            {
                'id': t.id, 'tag_id': t.tag_id, 'family': t.family, 'size': t.size,
                'floor_id': t.floor_id, 'x': t.x, 'y': t.y, 'z': t.z, 'yaw': t.yaw,
                'lane_id': t.lane_id, 'zone_id': t.zone_id, 'metadata': t.metadata,
                'label': t.label, 'enabled': t.enabled,
            }
            for t in tags
        ],
        'edges': [{'from_tag_id': e.from_tag.tag_id, 'to_tag_id': e.to_tag.tag_id,
                   'cost': e.cost if e.cost is not None else ((e.from_tag.x-e.to_tag.x)**2 + (e.from_tag.y-e.to_tag.y)**2) ** 0.5,
                   'bidirectional': e.bidirectional} for e in edges],
    }


def validate_target_tag(target_tag_id: int, warehouse_id: int | None = None) -> NavigationTag:
    wh = active_warehouse(warehouse_id)
    if wh is None:
        raise ValueError('no warehouse configured')
    try:
        return NavigationTag.objects.get(warehouse=wh, tag_id=int(target_tag_id), enabled=True)
    except NavigationTag.DoesNotExist as exc:
        raise ValueError(f'target tag {target_tag_id} is not enabled in warehouse {wh.code}') from exc


def shortest_tag_route(current_tag_id: int, target_tag_id: int, warehouse_id: int | None = None) -> list[int]:
    wh = active_warehouse(warehouse_id)
    tags = {t.tag_id: t for t in NavigationTag.objects.filter(warehouse=wh, enabled=True)}
    if current_tag_id not in tags or target_tag_id not in tags:
        raise ValueError('current and target tags must exist and be enabled')
    adj: dict[int, list[tuple[float, int]]] = {key: [] for key in tags}
    for edge in NavigationTagEdge.objects.filter(warehouse=wh, enabled=True).select_related('from_tag', 'to_tag'):
        cost = edge.cost if edge.cost is not None else ((edge.from_tag.x-edge.to_tag.x)**2 + (edge.from_tag.y-edge.to_tag.y)**2) ** 0.5
        adj[edge.from_tag.tag_id].append((cost, edge.to_tag.tag_id))
        if edge.bidirectional:
            adj[edge.to_tag.tag_id].append((cost, edge.from_tag.tag_id))
    dist = {current_tag_id: 0.0}; previous: dict[int, int] = {}; heap = [(0.0, current_tag_id)]
    while heap:
        cost, node = heapq.heappop(heap)
        if node == target_tag_id: break
        if cost != dist.get(node): continue
        for weight, nxt in adj.get(node, []):
            candidate = cost + weight
            if candidate < dist.get(nxt, float('inf')):
                dist[nxt] = candidate; previous[nxt] = node; heapq.heappush(heap, (candidate, nxt))
    if target_tag_id not in dist:
        raise ValueError(f'no tag route from {current_tag_id} to {target_tag_id}')
    route = [target_tag_id]
    while route[-1] != current_tag_id: route.append(previous[route[-1]])
    return list(reversed(route))


def _event(mission: RobotNavigationMission, event_type: str, message: str, payload: dict[str, Any] | None = None, source='FLEET_MANAGER'):
    EventLog.objects.create(run_id=f'tag-{mission.id}', event_id=uuid.uuid4().hex[:16], tick=0,
        type=event_type, source=source, severity='INFO', message=message, robot_id=mission.robot_id,
        payload=payload or {})


def mission_snapshot(mission: RobotNavigationMission) -> dict[str, Any]:
    return {'id': mission.id, 'mission_id': mission.id, 'warehouse_id': mission.warehouse_id, 'robot_id': mission.robot_id,
            'target_tag_id': mission.target_tag.tag_id, 'current_tag_id': mission.current_tag_id, 'next_tag_id': mission.next_tag_id,
            'route': mission.route, 'status': mission.status, 'route_index': mission.route_index,
            'progress_percent': mission.progress_percent, 'failure_reason': mission.failure_reason,
            'started_at': mission.started_at.isoformat() if mission.started_at else None,
            'updated_at': mission.updated_at.isoformat() if mission.updated_at else None,
            'completed_at': mission.completed_at.isoformat() if mission.completed_at else None}


@transaction.atomic
def create_mission(robot_id: str, target_tag_id: int, warehouse_id: int | None = None, current_tag_id: int | None = None):
    wh = active_warehouse(warehouse_id); target = validate_target_tag(target_tag_id, wh.id)
    current = current_tag_id
    if current is None:
        current = NavigationTag.objects.filter(warehouse=wh, tag_id=target_tag_id).values_list('tag_id', flat=True).first()
    if current is None: raise ValueError('current_tag_id is required until robot tag localization is available')
    route = shortest_tag_route(current, target_tag_id, wh.id)
    RobotNavigationMission.objects.filter(robot_id=robot_id, status__in=['PENDING','PLANNING','NAVIGATING','PAUSED','DEAD_RECKONING','APPROACH_TAG','TAG_CORRECTION']).update(status='CANCELLED', completed_at=timezone.now())
    mission = RobotNavigationMission.objects.create(warehouse=wh, robot_id=robot_id, target_tag=target, current_tag_id=current,
        next_tag_id=route[1] if len(route) > 1 else target_tag_id, route=route, status='PLANNING', route_index=0)
    _event(mission, 'TAG_MISSION_CREATED', f'Mission target={target_tag_id} created', {'route': route})
    _event(mission, 'TAG_ROUTE_PLANNED', 'Tag route planned', {'route': route})
    return mission


def current_mission(robot_id: str, mission_id: int | None = None):
    qs = RobotNavigationMission.objects.select_related('target_tag', 'warehouse').filter(robot_id=robot_id)
    if mission_id is not None: qs = qs.filter(pk=mission_id)
    return (qs.first() if mission_id is not None else qs.exclude(status__in=['ARRIVED', 'CANCELLED', 'FAILED', 'EMERGENCY_STOPPED']).first())


def apply_ros_tag_status(data: dict[str, Any]):
    mission = current_mission(str(data.get('robot_id', '')), int(data['mission_id']) if data.get('mission_id') else None)
    if mission is None: return None
    state = str(data.get('state') or mission.status).upper()
    allowed = {choice[0] for choice in RobotNavigationMission.STATUSES}
    mission.current_tag_id = data.get('current_tag_id', mission.current_tag_id)
    mission.next_tag_id = data.get('next_tag_id', mission.next_tag_id)
    mission.route = data.get('route') or mission.route
    mission.route_index = int(data.get('route_index', mission.route_index) or 0)
    mission.progress_percent = float(data.get('progress_percent', mission.progress_percent) or 0)
    # Arrival is a tag-gated safety transition. A coordinate-only or stale ROS
    # success message can never complete a web mission.
    if state == 'ARRIVED' and data.get('detected_tag_id') != mission.target_tag.tag_id and data.get('current_tag_id') != mission.target_tag.tag_id:
        state = 'ROUTE_DEVIATION'
        mission.failure_reason = 'ARRIVED rejected: target tag was not detected'
    mission.status = state if state in allowed else 'NAVIGATING'
    if mission.status in ('ARRIVED', 'CANCELLED', 'FAILED', 'EMERGENCY_STOPPED'):
        mission.completed_at = timezone.now()
    mission.save()
    return mission


def apply_ros_tag_event(data: dict[str, Any]):
    mission = current_mission(str(data.get('robot_id', '')), int(data['mission_id']) if data.get('mission_id') else None)
    if mission is not None:
        event = str(data.get('event') or 'TAG_NAV_EVENT')
        _event(mission, event, str(data.get('details') or event), data.get('details') if isinstance(data.get('details'), dict) else {}, 'ROS')
    return mission


def log_ros_tag_detection(data: dict[str, Any]):
    mission = current_mission(str(data.get('robot_id', '')), int(data['mission_id']) if data.get('mission_id') else None)
    if mission is not None and data.get('visible'):
        tag_id = data.get('tag_id')
        _event(mission, 'TAG_DETECTED', f'Tag {tag_id} detected', {'tag_id': tag_id, 'offset_x': data.get('offset_x'), 'offset_y': data.get('offset_y')}, 'ROS')
    return mission


def apply_ros_localization(data: dict[str, Any]):
    mission = current_mission(str(data.get('robot_id', '')), int(data['mission_id']) if data.get('mission_id') else None)
    if mission is not None:
        mission.current_tag_id = data.get('last_tag_id', mission.current_tag_id)
        state = str(data.get('state') or '').upper()
        if state in ('DEAD_RECKONING', 'APPROACH_TAG', 'TAG_CORRECTION'):
            mission.status = state
        mission.save(update_fields=['current_tag_id', 'status', 'updated_at'])
    return mission
