from __future__ import annotations

import math
import uuid
from datetime import timedelta
from typing import Any, Iterable

from django.db import transaction
from asgiref.sync import async_to_sync
from channels.layers import get_channel_layer
from django.db.models import Q
from django.utils import timezone
from django.utils.dateparse import parse_datetime

from .models import (
    ResourceReservation,
    RobotProfile,
    RobotSchedule,
    ScheduleStop,
    Warehouse,
    WarehouseMap,
    WarehouseOrder,
    WorkPoint,
    Zone,
    Shelf,
    ShelfInventoryItem,
)

ACTIVE_SCHEDULE_STATUSES = ('PLANNED', 'QUEUED', 'RUNNING')
RESERVED_STATUSES = ('HELD', 'ACTIVE')
PRIORITY_RANK = {'CRITICAL': 4, 'HIGH': 3, 'NORMAL': 2, 'LOW': 1}


def _dt(value: Any, *, default=None):
    if value in (None, ''):
        return default
    if hasattr(value, 'tzinfo'):
        dt = value
    else:
        dt = parse_datetime(str(value))
    if dt is None:
        raise ValueError('invalid datetime')
    if timezone.is_naive(dt):
        dt = timezone.make_aware(dt, timezone.get_current_timezone())
    return dt


def active_warehouse() -> Warehouse:
    m = WarehouseMap.objects.filter(is_active=True).select_related('warehouse').first()
    if m:
        return m.warehouse
    w = Warehouse.objects.filter(status='ACTIVE').order_by('id').first()
    if not w:
        raise ValueError('no active warehouse')
    return w


def workpoint_to_dict(w: WorkPoint) -> dict[str, Any]:
    return {
        'id': w.id, 'warehouse_id': w.warehouse_id, 'zone_id': w.zone_id,
        'code': w.code, 'name': w.name, 'kind': w.kind, 'floor': w.floor,
        'x': w.x, 'y': w.y, 'z': w.z, 'yaw': w.yaw,
        'resource_type': w.resource_type, 'resource_id': w.resource_id,
        'enabled': w.enabled, 'capacity': w.capacity, 'metadata': w.metadata,
        'updated_at': w.updated_at.isoformat() if w.updated_at else None,
    }


def robot_to_dict(p: RobotProfile, runtime_state: dict[str, Any] | None = None) -> dict[str, Any]:
    r = (runtime_state or {}).get(p.robot_id) or {}
    return {
        'id': p.id, 'warehouse_id': p.warehouse_id, 'robot_id': p.robot_id,
        'name': p.name or p.robot_id, 'enabled': p.enabled,
        'payload_capacity': p.payload_capacity, 'min_dispatch_battery': p.min_dispatch_battery,
        'capabilities': p.capabilities, 'metadata': p.metadata,
        'telemetry': {
            'status': r.get('status', 'OFFLINE'), 'fsm': r.get('fsm', 'OFFLINE'),
            'battery': float(r.get('battery', 0)), 'floor': int(r.get('floor', 1) or 1),
            'position': r.get('position', [0, 0, 0]), 'zone': r.get('zone'),
            'current_task_id': r.get('current_task_id'),
            'load': r.get('load', {'current': 0, 'capacity': p.payload_capacity}),
            'carrying_load': float((r.get('load') or {}).get('current') or 0) > 0,
        },
    }


def order_to_dict(o: WarehouseOrder) -> dict[str, Any]:
    return {
        'id': o.id, 'order_no': o.order_no, 'external_ref': o.external_ref,
        'warehouse_id': o.warehouse_id, 'type': o.type, 'priority': o.priority, 'status': o.status,
        'source': workpoint_to_dict(o.source), 'destination': workpoint_to_dict(o.destination),
        'quantity': o.quantity, 'load_units': o.load_units, 'payload_weight_kg': o.payload_weight_kg,
        'due_at': o.due_at.isoformat() if o.due_at else None, 'notes': o.notes, 'metadata': o.metadata,
        'created_at': o.created_at.isoformat() if o.created_at else None,
        'updated_at': o.updated_at.isoformat() if o.updated_at else None,
    }


def schedule_to_dict(s: RobotSchedule, *, include_stops=True) -> dict[str, Any]:
    data = {
        'id': s.id, 'schedule_id': s.schedule_id, 'warehouse_id': s.warehouse_id,
        'order_id': s.order_id, 'order_no': s.order.order_no, 'priority': s.order.priority,
        'robot_id': s.robot.robot_id, 'mode': s.mode, 'status': s.status,
        'planned_start': s.planned_start.isoformat(),
        'planned_end': s.planned_end.isoformat() if s.planned_end else None,
        'actual_start': s.actual_start.isoformat() if s.actual_start else None,
        'actual_end': s.actual_end.isoformat() if s.actual_end else None,
        'estimated_distance_m': s.estimated_distance_m,
        'estimated_duration_s': s.estimated_duration_s,
        'score': s.score, 'current_leg': s.current_leg, 'engine_task_id': s.engine_task_id,
        'failure_reason': s.failure_reason,
        'created_at': s.created_at.isoformat() if s.created_at else None,
    }
    if include_stops:
        data['stops'] = [
            {
                'id': x.id, 'sequence': x.sequence, 'action': x.action,
                'service_seconds': x.service_seconds, 'status': x.status,
                'workpoint': workpoint_to_dict(x.workpoint),
                'arrived_at': x.arrived_at.isoformat() if x.arrived_at else None,
                'completed_at': x.completed_at.isoformat() if x.completed_at else None,
            }
            for x in s.stops.select_related('workpoint').order_by('sequence')
        ]
    return data


def _zone_for(warehouse: Warehouse, zone_code: str | None) -> Zone | None:
    if not zone_code:
        return None
    return Zone.objects.filter(warehouse=warehouse).filter(Q(code=zone_code) | Q(layout_zone_id=zone_code)).first()


def sync_workpoints_from_layout(warehouse: Warehouse, layout: dict[str, Any], *, prune=False) -> dict[str, int]:
    """Make scheduler work-points follow the database-backed warehouse map.

    Existing operational references remain stable because codes are deterministic.
    Layout locations are imported verbatim; conveyor endpoints are additionally
    exposed as CVxx-IN / CVxx-OUT points so they can be used in order routes.
    """
    wanted: dict[str, dict[str, Any]] = {}
    for loc in layout.get('locations') or []:
        code = str(loc.get('id') or '').strip()
        ap = loc.get('access_point') or []
        if not code or len(ap) < 2:
            continue
        kind = str(loc.get('kind') or 'CUSTOM').upper()
        if kind not in dict(WorkPoint.KINDS):
            kind = 'STATION'
        wanted[code] = {
            'name': code.replace('-', ' ').title(), 'kind': kind, 'floor': int(loc.get('floor') or 1),
            'x': float(ap[0]), 'y': float(ap[1]), 'z': 0.0, 'yaw': float(loc.get('yaw') or 0.0),
            'zone': _zone_for(warehouse, loc.get('zone')), 'resource_type': kind,
            'resource_id': str(loc.get('rack_id') or code), 'enabled': True,
            'capacity': 1, 'metadata': {'layout_location': True, **{k: v for k, v in loc.items() if k not in ('access_point',)}},
        }
    for cv in layout.get('conveyors') or []:
        path = cv.get('path') or []
        if len(path) < 2:
            continue
        for suffix, point, kind in [('IN', path[0], 'CONVEYOR_IN'), ('OUT', path[-1], 'CONVEYOR_OUT')]:
            code = f"{cv['id']}-{suffix}"
            wanted[code] = {
                'name': f"{cv.get('name') or cv['id']} {suffix.title()}", 'kind': kind, 'floor': int(cv.get('floor') or 1),
                'x': float(point[0]), 'y': float(point[1]), 'z': 0.0, 'yaw': 0.0,
                'zone': _zone_for(warehouse, cv.get('zone')), 'resource_type': 'CONVEYOR', 'resource_id': str(cv['id']),
                'enabled': True, 'capacity': 1, 'metadata': {'conveyor': cv['id'], 'endpoint': suffix},
            }
    for dock in layout.get('docks') or []:
        # Docks can be operational INBOUND / OUTBOUND points.  Older code
        # overwrote the matching layout location with the generic DOCK kind,
        # which made it impossible for the dashboard to distinguish receiving
        # from shipping.  Preserve the dock's semantic kind when available.
        p = dock.get('access_point') or dock.get('position') or dock.get('cell') or dock.get('door') or []
        if len(p) < 2:
            continue
        code = str(dock.get('id') or f'DOCK-{len(wanted)+1}')
        raw_kind = str(dock.get('kind') or '').upper()
        dock_kind = raw_kind if raw_kind in ('INBOUND', 'OUTBOUND') else 'DOCK'
        # A location with the same code usually has a navigation access point
        # just inside the warehouse. Prefer that coordinate over the door edge.
        previous = wanted.get(code)
        if previous and str(previous.get('kind') or '') in ('INBOUND', 'OUTBOUND'):
            dock_kind = str(previous['kind'])
            p = [previous['x'], previous['y']]
        wanted[code] = {
            'name': str(dock.get('name') or code), 'kind': dock_kind, 'floor': int(dock.get('floor') or 1),
            'x': float(p[0]), 'y': float(p[1]), 'z': 0.0, 'yaw': float(dock.get('yaw') or 0),
            'zone': _zone_for(warehouse, dock.get('zone')), 'resource_type': dock_kind, 'resource_id': code,
            'enabled': True, 'capacity': 1, 'metadata': {'dock': dock, 'dock_kind': dock_kind},
        }

    created = updated = 0
    with transaction.atomic():
        for code, values in wanted.items():
            obj, was_created = WorkPoint.objects.get_or_create(warehouse=warehouse, code=code, defaults=values)
            if was_created:
                created += 1
            else:
                changed_fields = []
                for key, value in values.items():
                    current = getattr(obj, key)
                    current_cmp = current.pk if key == 'zone' and current is not None else current
                    value_cmp = value.pk if key == 'zone' and value is not None else value
                    if current_cmp != value_cmp:
                        setattr(obj, key, value)
                        changed_fields.append(key)
                if changed_fields:
                    obj.save(update_fields=[*changed_fields, 'updated_at'])
                    updated += 1
        if prune:
            WorkPoint.objects.filter(warehouse=warehouse).exclude(code__in=wanted.keys()).update(enabled=False)
    return {'created': created, 'updated': updated, 'total': len(wanted)}


def sync_robot_profiles(warehouse: Warehouse, runtime_robots: dict[str, dict[str, Any]]) -> dict[str, int]:
    created = updated = 0
    ids = set()
    for rid, state in runtime_robots.items():
        ids.add(rid)
        obj, was_created = RobotProfile.objects.get_or_create(
            warehouse=warehouse, robot_id=rid,
            defaults={'name': rid, 'payload_capacity': int((state.get('load') or {}).get('capacity') or 4), 'capabilities': ['MOVE', 'PICK', 'TRANSFER']},
        )
        if was_created: created += 1
        else:
            changed_fields = []
            if not obj.enabled:
                obj.enabled = True
                changed_fields.append('enabled')
            if not obj.name:
                obj.name = rid
                changed_fields.append('name')
            capacity = int((state.get('load') or {}).get('capacity') or obj.payload_capacity)
            if obj.payload_capacity != capacity:
                obj.payload_capacity = capacity
                changed_fields.append('payload_capacity')
            if changed_fields:
                obj.save(update_fields=[*changed_fields, 'updated_at'])
                updated += 1
    RobotProfile.objects.filter(warehouse=warehouse).exclude(robot_id__in=ids).update(enabled=False)
    return {'created': created, 'updated': updated, 'total': len(ids)}


def ensure_scheduler_master_data(runtime) -> Warehouse:
    """Return the scheduler warehouse without mutating the database.

    IMPORTANT: request handlers call this function concurrently (overview, orders,
    robots, schedules, workpoints).  Synchronizing hundreds of work-points here
    caused every GET request to become a writer and made SQLite fail with
    "database is locked" under normal dashboard parallel loading.

    Master data is synchronized explicitly by ``seed_demo``, the warehouse/layout
    publish path, or POST ``/api/scheduler/sync``.  Read APIs must stay read-only.
    """
    warehouse = active_warehouse()
    if not WorkPoint.objects.filter(warehouse=warehouse).exists():
        raise ValueError('scheduler work-points are not initialized; run: python manage.py seed_demo')
    if not RobotProfile.objects.filter(warehouse=warehouse).exists():
        raise ValueError('scheduler robot profiles are not initialized; run: python manage.py seed_demo')
    return warehouse


def _flow_kind(point: WorkPoint) -> str:
    """Return the operational kind for a work-point, including legacy dock rows."""
    kind = str(point.kind or '').upper()
    if kind != 'DOCK':
        return kind
    meta = point.metadata if isinstance(point.metadata, dict) else {}
    dock = meta.get('dock') if isinstance(meta.get('dock'), dict) else {}
    legacy = str(meta.get('dock_kind') or dock.get('kind') or '').upper()
    if legacy in ('INBOUND', 'OUTBOUND'):
        return legacy
    code = str(point.code or '').upper()
    if code.startswith('INBOUND'):
        return 'INBOUND'
    if code.startswith('OUTBOUND'):
        return 'OUTBOUND'
    return kind


def _shelf_for_workpoint(point: WorkPoint) -> Shelf | None:
    if _flow_kind(point) != 'SHELF':
        return None
    rack_id = str(point.resource_id or '').strip()
    q = Shelf.objects.filter(zone__warehouse=point.warehouse)
    if rack_id:
        shelf = q.filter(layout_rack_id=rack_id).first()
        if shelf:
            return shelf
    # Fallback for hand-created scheduler points.
    return q.filter(code=point.code).first()


def workpoint_for_shelf(shelf: Shelf) -> WorkPoint | None:
    """Return one enabled scheduler point for a physical shelf."""
    rack_id = str(shelf.layout_rack_id or shelf.code)
    q = WorkPoint.objects.filter(warehouse=shelf.zone.warehouse, enabled=True)
    exact = q.filter(kind='SHELF', resource_id__iexact=rack_id).order_by('id').first()
    if exact:
        return exact
    # Legacy layouts may expose SHELF semantics through the resource metadata only.
    for point in q.filter(Q(code__iexact=shelf.code) | Q(resource_id__iexact=rack_id)).order_by('id'):
        if _flow_kind(point) == 'SHELF':
            return point
    return None


def inventory_item_to_dict(item: ShelfInventoryItem) -> dict[str, Any]:
    shelf = item.shelf
    origin = item.origin_order
    reserved = item.reserved_by_order
    last = item.last_movement_order
    return {
        'id': item.id,
        'item_uid': item.item_uid,
        'warehouse_id': item.warehouse_id,
        'shelf_id': item.shelf_id,
        'shelf_code': (shelf.layout_rack_id or shelf.code) if shelf else None,
        'status': item.status,
        'item_code': item.item_code,
        'item_name': item.item_name,
        'external_ref': item.external_ref,
        'quantity': item.quantity,
        'load_units': item.load_units,
        'payload_weight_kg': item.payload_weight_kg,
        'origin_order_id': item.origin_order_id,
        'origin_order_no': origin.order_no if origin else None,
        'last_movement_order_id': item.last_movement_order_id,
        'last_movement_order_no': last.order_no if last else None,
        'reserved_by_order_id': item.reserved_by_order_id,
        'reserved_by_order_no': reserved.order_no if reserved else None,
        'metadata': item.metadata,
        'stored_at': item.stored_at.isoformat() if item.stored_at else None,
        'updated_at': item.updated_at.isoformat() if item.updated_at else None,
    }


def _new_inventory_uid(order: WarehouseOrder | None = None) -> str:
    prefix = (order.order_no if order else 'ITEM').replace(' ', '-')[:36]
    return f'{prefix}-{uuid.uuid4().hex[:12].upper()}'


def reserve_inventory_for_order(
    order: WarehouseOrder, *, item_id: int | None = None, item_code: str = '', external_ref: str = ''
) -> ShelfInventoryItem:
    """Reserve one exact shelf item for OUTBOUND/TRANSFER.

    File-based outbound orders can identify goods by SKU/reference. Shelf-click
    actions pass item_id, which is stronger and prevents the same item being moved
    by two operators at the same time.
    """
    if order.type not in ('OUTBOUND', 'TRANSFER'):
        raise ValueError('inventory reservation is only valid for OUTBOUND/TRANSFER orders')
    shelf = _shelf_for_workpoint(order.source)
    if shelf is None:
        raise ValueError('source shelf is not linked to warehouse shelf data')
    with transaction.atomic():
        q = ShelfInventoryItem.objects.select_for_update().filter(
            warehouse=order.warehouse, shelf=shelf, status='STORED', reserved_by_order__isnull=True,
        )
        if item_id is not None:
            q = q.filter(pk=int(item_id))
        else:
            item_code = str(item_code or '').strip()
            external_ref = str(external_ref or '').strip()
            if item_code:
                q = q.filter(item_code__iexact=item_code)
            if external_ref:
                q = q.filter(external_ref__iexact=external_ref)
        item = q.order_by('stored_at', 'id').first()
        if item is None:
            token = f' item {item_id}' if item_id is not None else (f' SKU {item_code}' if item_code else '')
            raise ValueError(f'no available{token} on shelf {shelf.code}')
        item.status = 'RESERVED'
        item.reserved_by_order = order
        item.last_movement_order = order
        item.save(update_fields=['status', 'reserved_by_order', 'last_movement_order', 'updated_at'])
        metadata = dict(order.metadata or {})
        metadata.update({
            'inventory_item_id': item.id,
            'inventory_item_uid': item.item_uid,
            'item_code': item.item_code,
            'item_name': item.item_name,
            'source_shelf_code': shelf.layout_rack_id or shelf.code,
        })
        WarehouseOrder.objects.filter(pk=order.pk).update(metadata=metadata)
        order.metadata = metadata
        return item


def release_inventory_reservation(order: WarehouseOrder) -> bool:
    """Make a reserved item selectable again when an order is cancelled/failed."""
    item = ShelfInventoryItem.objects.filter(reserved_by_order=order, status='RESERVED').first()
    if item is None:
        return False
    item.status = 'STORED'
    item.reserved_by_order = None
    item.save(update_fields=['status', 'reserved_by_order', 'updated_at'])
    return True


def sync_inventory_placeholders(warehouse: Warehouse) -> dict[str, int]:
    """Backfill lightweight demo items so every current_load slot is inspectable.

    This is called by seed_demo after layout/master-data sync. Real imported items
    are never overwritten or deleted.
    """
    created = 0
    for shelf in Shelf.objects.filter(zone__warehouse=warehouse).select_related('zone').order_by('id'):
        tracked = ShelfInventoryItem.objects.filter(
            warehouse=warehouse, shelf=shelf, status__in=('STORED', 'RESERVED')
        ).count()
        needed = max(0, min(8, int(shelf.current_load)) - tracked)
        for offset in range(needed):
            slot = tracked + offset + 1
            ShelfInventoryItem.objects.create(
                item_uid=_new_inventory_uid(), warehouse=warehouse, shelf=shelf, status='STORED',
                item_code=f'DEMO-{shelf.code}-{slot:02d}',
                item_name=f'Demo item {slot} on {shelf.code}', quantity=1, load_units=1,
                metadata={'seeded_from_shelf_load': True, 'slot': slot},
            )
            created += 1
    return {'created': created}


def validate_flow_route(type_: str, source: WorkPoint, destination: WorkPoint, *, exclude_order_id: int | None = None) -> None:
    """Enforce the receiving/shipping contract and shelf slot availability.

    One completed INBOUND order occupies one of the physical shelf's eight order
    slots; one completed OUTBOUND order releases one slot. Active orders reserve
    those slots so operators cannot overbook a shelf from the dashboard.
    """
    kind = str(type_ or '').upper()
    source_kind = _flow_kind(source)
    destination_kind = _flow_kind(destination)
    active = ('NEW', 'PLANNED', 'QUEUED', 'RUNNING')

    if kind == 'INBOUND':
        if source_kind != 'INBOUND':
            raise ValueError('INBOUND order source must be an inbound dock')
        if destination_kind != 'SHELF':
            raise ValueError('INBOUND order destination must be a shelf')
        shelf = _shelf_for_workpoint(destination)
        if shelf is None:
            raise ValueError('destination shelf is not linked to warehouse shelf data')
        pending = WarehouseOrder.objects.filter(
            warehouse=source.warehouse, type='INBOUND', destination__resource_id=destination.resource_id, status__in=active,
        )
        if exclude_order_id:
            pending = pending.exclude(pk=exclude_order_id)
        if int(shelf.current_load) + pending.count() >= 8:
            raise ValueError(f'shelf {shelf.code} has no free order slot (8/8 including inbound reservations)')

    elif kind == 'OUTBOUND':
        if source_kind != 'SHELF':
            raise ValueError('OUTBOUND order source must be a shelf')
        if destination_kind != 'OUTBOUND':
            raise ValueError('OUTBOUND order destination must be an outbound dock')
        shelf = _shelf_for_workpoint(source)
        if shelf is None:
            raise ValueError('source shelf is not linked to warehouse shelf data')
        pending = WarehouseOrder.objects.filter(
            warehouse=source.warehouse, type='OUTBOUND', source__resource_id=source.resource_id, status__in=active,
        )
        if exclude_order_id:
            pending = pending.exclude(pk=exclude_order_id)
        if int(shelf.current_load) - pending.count() <= 0:
            raise ValueError(f'shelf {shelf.code} has no unreserved order available for outbound')

    elif kind == 'TRANSFER':
        if source_kind != 'SHELF' or destination_kind != 'SHELF':
            raise ValueError('TRANSFER order must move from one shelf to another shelf')
        if source.id == destination.id or (source.resource_id and source.resource_id == destination.resource_id):
            raise ValueError('TRANSFER destination shelf must differ from source shelf')
        source_shelf = _shelf_for_workpoint(source)
        destination_shelf = _shelf_for_workpoint(destination)
        if source_shelf is None or destination_shelf is None:
            raise ValueError('TRANSFER shelf is not linked to warehouse shelf data')
        if destination_shelf.status in ('DISABLED', 'MAINTENANCE'):
            raise ValueError(f'destination shelf {destination_shelf.code} is unavailable')
        pending_out = WarehouseOrder.objects.filter(
            warehouse=source.warehouse, type__in=('OUTBOUND', 'TRANSFER'),
            source__resource_id=source.resource_id, status__in=active,
        )
        pending_in = WarehouseOrder.objects.filter(
            warehouse=source.warehouse, type__in=('INBOUND', 'TRANSFER'),
            destination__resource_id=destination.resource_id, status__in=active,
        )
        if exclude_order_id:
            pending_out = pending_out.exclude(pk=exclude_order_id)
            pending_in = pending_in.exclude(pk=exclude_order_id)
        if int(source_shelf.current_load) - pending_out.count() <= 0:
            raise ValueError(f'source shelf {source_shelf.code} has no unreserved order available')
        if int(destination_shelf.current_load) + pending_in.count() >= 8:
            raise ValueError(f'destination shelf {destination_shelf.code} has no free order slot (8/8 including reservations)')


def _set_shelf_counter(shelf: Shelf, value: int) -> None:
    shelf.current_load = max(0, min(8, int(value)))
    shelf.capacity = 8
    shelf.status = 'FULL' if shelf.current_load >= 8 else 'OCCUPIED' if shelf.current_load > 0 else 'AVAILABLE'
    shelf.save(update_fields=['current_load', 'capacity', 'status', 'updated_at'])


def _notify_shelf_layout_refresh(order: WarehouseOrder) -> None:
    # The live 2D/3D shelf render reads current_load from /api/layout. Trigger the
    # normal refresh path without changing geometry/revision.
    active_map = WarehouseMap.objects.filter(warehouse=order.warehouse, is_active=True).first()
    layer = get_channel_layer()
    if active_map is not None and layer is not None:
        async_to_sync(layer.group_send)('twin_clients', {
            'type': 'twin.message',
            'payload': {
                'type': 'LAYOUT_UPDATED', 'source': 'shelf-inventory',
                'warehouse_id': active_map.warehouse_id,
                'layout_id': (active_map.layout or {}).get('id') or active_map.warehouse.layout_id,
                'revision': active_map.revision, 'published_version': active_map.published_version,
                'is_active': True,
                'updated_at': active_map.updated_at.isoformat() if active_map.updated_at else None,
            },
        })


def apply_completed_flow_to_shelf(order: WarehouseOrder) -> bool:
    """Apply one completed INBOUND/OUTBOUND/TRANSFER exactly once.

    Besides the lightweight shelf counter used by the renderer, this updates the
    exact ShelfInventoryItem selected by the operator so the next shelf click shows
    the real item location.
    """
    if order.type not in ('INBOUND', 'OUTBOUND', 'TRANSFER'):
        return False

    changed = False
    with transaction.atomic():
        locked_order = WarehouseOrder.objects.select_for_update().select_related(
            'source', 'destination', 'warehouse'
        ).get(pk=order.pk)
        metadata = dict(locked_order.metadata or {})
        if metadata.get('shelf_occupancy_applied'):
            return False

        if locked_order.type == 'INBOUND':
            shelf = _shelf_for_workpoint(locked_order.destination)
            if shelf is None:
                return False
            shelf = Shelf.objects.select_for_update().get(pk=shelf.pk)
            before = max(0, min(8, int(shelf.current_load)))
            if before >= 8:
                raise ValueError(f'shelf {shelf.code} is already full')
            _set_shelf_counter(shelf, before + 1)

            item = ShelfInventoryItem.objects.filter(origin_order=locked_order).first()
            if item is None:
                item = ShelfInventoryItem.objects.create(
                    item_uid=_new_inventory_uid(locked_order),
                    warehouse=locked_order.warehouse,
                    shelf=shelf,
                    status='STORED',
                    item_code=str(metadata.get('item_code') or ''),
                    item_name=str(metadata.get('item_name') or ''),
                    external_ref=locked_order.external_ref,
                    quantity=max(1, int(locked_order.quantity)),
                    load_units=max(1, int(locked_order.load_units)),
                    payload_weight_kg=max(0.0, float(locked_order.payload_weight_kg)),
                    origin_order=locked_order,
                    last_movement_order=locked_order,
                    metadata={
                        'source': metadata.get('created_from') or 'inbound_order',
                        'import_batch_id': metadata.get('import_batch_id'),
                        'import_payload': metadata.get('import_payload') or {},
                    },
                )
            metadata.update({
                'inventory_item_id': item.id, 'inventory_item_uid': item.item_uid,
                'shelf_occupancy_before': before, 'shelf_occupancy_after': shelf.current_load,
                'shelf_occupancy_rack_id': shelf.layout_rack_id or shelf.code,
            })

        elif locked_order.type == 'OUTBOUND':
            shelf = _shelf_for_workpoint(locked_order.source)
            if shelf is None:
                return False
            shelf = Shelf.objects.select_for_update().get(pk=shelf.pk)
            before = max(0, min(8, int(shelf.current_load)))
            if before <= 0:
                raise ValueError(f'shelf {shelf.code} is already empty')

            item_q = ShelfInventoryItem.objects.select_for_update().filter(warehouse=locked_order.warehouse, shelf=shelf)
            item = item_q.filter(reserved_by_order=locked_order).first()
            if item is None and metadata.get('inventory_item_id'):
                item = item_q.filter(pk=metadata['inventory_item_id']).first()
            if item is None and metadata.get('item_code'):
                item = item_q.filter(status='STORED', item_code__iexact=str(metadata['item_code'])).order_by('stored_at', 'id').first()
            if item is None:
                item = item_q.filter(status__in=('STORED', 'RESERVED')).order_by('stored_at', 'id').first()
            if item is None:
                raise ValueError(f'no tracked inventory item is available on shelf {shelf.code}')

            _set_shelf_counter(shelf, before - 1)
            item.status = 'OUTBOUND'
            item.shelf = None
            item.reserved_by_order = None
            item.last_movement_order = locked_order
            item.save(update_fields=['status', 'shelf', 'reserved_by_order', 'last_movement_order', 'updated_at'])
            metadata.update({
                'inventory_item_id': item.id, 'inventory_item_uid': item.item_uid,
                'shelf_occupancy_before': before, 'shelf_occupancy_after': shelf.current_load,
                'shelf_occupancy_rack_id': shelf.layout_rack_id or shelf.code,
            })

        else:  # TRANSFER
            source_shelf = _shelf_for_workpoint(locked_order.source)
            destination_shelf = _shelf_for_workpoint(locked_order.destination)
            if source_shelf is None or destination_shelf is None:
                return False
            # Stable lock order avoids deadlocks if two operators swap shelves.
            shelf_by_id = {x.pk: x for x in Shelf.objects.select_for_update().filter(pk__in=[source_shelf.pk, destination_shelf.pk]).order_by('pk')}
            source_shelf = shelf_by_id[source_shelf.pk]
            destination_shelf = shelf_by_id[destination_shelf.pk]
            source_before = max(0, min(8, int(source_shelf.current_load)))
            destination_before = max(0, min(8, int(destination_shelf.current_load)))
            if source_before <= 0:
                raise ValueError(f'source shelf {source_shelf.code} is already empty')
            if destination_before >= 8:
                raise ValueError(f'destination shelf {destination_shelf.code} is already full')

            item_q = ShelfInventoryItem.objects.select_for_update().filter(warehouse=locked_order.warehouse, shelf=source_shelf)
            item = item_q.filter(reserved_by_order=locked_order).first()
            if item is None and metadata.get('inventory_item_id'):
                item = item_q.filter(pk=metadata['inventory_item_id']).first()
            if item is None:
                raise ValueError('TRANSFER has no reserved inventory item')

            _set_shelf_counter(source_shelf, source_before - 1)
            _set_shelf_counter(destination_shelf, destination_before + 1)
            item.shelf = destination_shelf
            item.status = 'STORED'
            item.reserved_by_order = None
            item.last_movement_order = locked_order
            item.stored_at = timezone.now()
            item.save(update_fields=['shelf', 'status', 'reserved_by_order', 'last_movement_order', 'stored_at', 'updated_at'])
            metadata.update({
                'inventory_item_id': item.id, 'inventory_item_uid': item.item_uid,
                'source_shelf_code': source_shelf.layout_rack_id or source_shelf.code,
                'destination_shelf_code': destination_shelf.layout_rack_id or destination_shelf.code,
                'source_shelf_load_before': source_before, 'source_shelf_load_after': source_shelf.current_load,
                'destination_shelf_load_before': destination_before, 'destination_shelf_load_after': destination_shelf.current_load,
            })

        metadata['shelf_occupancy_applied'] = True
        metadata['shelf_occupancy_applied_at'] = timezone.now().isoformat()
        locked_order.metadata = metadata
        locked_order.save(update_fields=['metadata', 'updated_at'])
        order.metadata = metadata
        changed = True

    if changed:
        _notify_shelf_layout_refresh(order)
    return changed


def create_order(data: dict[str, Any], user=None) -> WarehouseOrder:
    warehouse = Warehouse.objects.get(pk=int(data.get('warehouse_id') or active_warehouse().id))
    source = WorkPoint.objects.get(pk=int(data['source_id']), warehouse=warehouse, enabled=True)
    destination = WorkPoint.objects.get(pk=int(data['destination_id']), warehouse=warehouse, enabled=True)
    if source.id == destination.id:
        raise ValueError('source and destination must differ')
    order_no = str(data.get('order_no') or f"ORD-{timezone.now().strftime('%Y%m%d')}-{uuid.uuid4().hex[:6].upper()}")
    priority = str(data.get('priority') or 'NORMAL').upper()
    type_ = str(data.get('type') or 'MOVE').upper()
    if priority not in dict(WarehouseOrder.PRIORITIES): raise ValueError('invalid priority')
    if type_ not in dict(WarehouseOrder.TYPES): raise ValueError('invalid order type')
    validate_flow_route(type_, source, destination)
    load_units = int(data.get('load_units') or 1)
    if load_units < 1 or load_units > 4: raise ValueError('load_units must be between 1 and 4')
    return WarehouseOrder.objects.create(
        warehouse=warehouse, order_no=order_no, external_ref=str(data.get('external_ref') or ''),
        type=type_, priority=priority, source=source, destination=destination,
        quantity=max(1, int(data.get('quantity') or 1)), load_units=load_units,
        payload_weight_kg=max(0.0, float(data.get('payload_weight_kg') or 0.0)),
        due_at=_dt(data.get('due_at')), notes=str(data.get('notes') or ''),
        metadata=data.get('metadata') if isinstance(data.get('metadata'), dict) else {}, created_by=user,
    )


def _point_dist(a: tuple[float,float,int], b: WorkPoint) -> float:
    floor_penalty = 35.0 if int(a[2]) != int(b.floor) else 0.0
    return math.hypot(float(a[0]) - b.x, float(a[1]) - b.y) + floor_penalty


def _route_distance(robot_state: dict[str, Any], stops: Iterable[WorkPoint]) -> float:
    pos = robot_state.get('position') or [0, 0, 0]
    cur = (float(pos[0]), float(pos[2]), int(robot_state.get('floor') or 1))
    total = 0.0
    for p in stops:
        total += _point_dist(cur, p)
        cur = (p.x, p.y, p.floor)
    return total


def _conflicts(robot: RobotProfile, start, end, exclude_schedule_id: int | None = None) -> bool:
    q = RobotSchedule.objects.filter(robot=robot, status__in=ACTIVE_SCHEDULE_STATUSES).filter(planned_start__lt=end).filter(Q(planned_end__isnull=True) | Q(planned_end__gt=start))
    if exclude_schedule_id: q = q.exclude(pk=exclude_schedule_id)
    return q.exists()


def candidate_scores(order: WarehouseOrder, planned_start, runtime_state: dict[str, Any], route_points: list[WorkPoint] | None = None) -> list[dict[str, Any]]:
    route = route_points or [order.source, order.destination]
    candidates: list[dict[str, Any]] = []
    profiles = RobotProfile.objects.filter(warehouse=order.warehouse, enabled=True).order_by('robot_id')
    raw: list[dict[str, Any]] = []
    for p in profiles:
        r = runtime_state.get(p.robot_id) or {}
        battery = float(r.get('battery') or 0)
        fsm = str(r.get('fsm') or 'OFFLINE')
        status = str(r.get('status') or 'OFFLINE')
        reasons = []
        rejected = None
        if status in ('OFFLINE', 'ERROR') or not r:
            rejected = f'robot is {status.lower()}'
        elif battery < p.min_dispatch_battery:
            rejected = f'battery {battery:.0f}% below dispatch minimum {p.min_dispatch_battery:.0f}%'
        elif order.load_units > p.payload_capacity:
            rejected = f'load {order.load_units} exceeds capacity {p.payload_capacity}'
        dist = _route_distance(r, route) if r else 1e6
        duration = dist / 1.1 + sum(20 for _ in route[1:])
        end = planned_start + timedelta(seconds=duration)
        conflict = _conflicts(p, planned_start, end)
        if conflict and not rejected: rejected = 'schedule conflict'
        raw.append({'profile': p, 'state': r, 'battery': battery, 'distance_m': dist, 'duration_s': duration, 'rejected': rejected, 'fsm': fsm})
    valid = [x for x in raw if not x['rejected']]
    max_dist = max([x['distance_m'] for x in valid], default=1.0)
    for x in raw:
        r = x['state']; p = x['profile']
        distance_score = max(0.0, 1.0 - x['distance_m'] / max(max_dist, 1.0)) if not x['rejected'] else 0.0
        battery_score = min(1.0, x['battery'] / 100.0)
        availability = 1.0 if x['fsm'] == 'IDLE' else 0.55 if x['fsm'] in ('COMPLETED', 'CHARGING') else 0.30
        active_count = RobotSchedule.objects.filter(robot=p, status__in=ACTIVE_SCHEDULE_STATUSES).count()
        workload = 1.0 / (1.0 + active_count)
        score = 0.40 * distance_score + 0.25 * availability + 0.20 * battery_score + 0.15 * workload
        reasons = [f"distance {x['distance_m']:.1f} m", f"battery {x['battery']:.0f}%", f"runtime {x['fsm']}", f"planned jobs {active_count}"]
        candidates.append({
            'robot_id': p.robot_id, 'robot_profile_id': p.id, 'score': round(score * 100, 1),
            'distance_m': round(x['distance_m'], 2), 'duration_s': round(x['duration_s'], 1),
            'battery': round(x['battery'], 1), 'fsm': x['fsm'], 'eligible': not bool(x['rejected']),
            'rejected_reason': x['rejected'], 'reasons': reasons,
        })
    return sorted(candidates, key=lambda x: (not x['eligible'], -x['score'], x['distance_m'], x['robot_id']))


def _resource_windows(
    route: list[WorkPoint], planned_start, service_seconds: list[int] | None = None,
    start_point: tuple[float, float, int] | None = None,
) -> list[dict[str, Any]]:
    """Estimate shared-resource occupancy windows along an ordered route.

    `start_point` is the assigned robot pose on the navigation plane. When it is
    known, source-resource reservation begins at the robot's estimated arrival
    instead of immediately at planned_start.
    """
    service_seconds = service_seconds or []
    cursor = planned_start
    cur = start_point or (route[0].x, route[0].y, route[0].floor)
    out: list[dict[str, Any]] = []
    for i, wp in enumerate(route):
        cursor += timedelta(seconds=max(1, _point_dist(cur, wp) / 1.1))
        default_service = 20 if i in (0, len(route) - 1) else 0
        secs = max(0, int(service_seconds[i])) if i < len(service_seconds) else default_service
        if wp.kind in ('CONVEYOR_IN','CONVEYOR_OUT','INBOUND','OUTBOUND','PACKING','SORTING','DOCK','STATION','CHARGING'):
            start = cursor
            end = start + timedelta(seconds=max(20, secs or 20))
            out.append({
                'workpoint': wp,
                'resource_type': wp.resource_type or wp.kind,
                'resource_id': wp.resource_id or wp.code,
                'starts_at': start,
                'ends_at': end,
            })
            cursor = end
        else:
            cursor += timedelta(seconds=secs)
        cur = (wp.x, wp.y, wp.floor)
    return out


def _resource_conflicts(
    route: list[WorkPoint], planned_start, service_seconds: list[int] | None = None,
    start_point: tuple[float, float, int] | None = None,
) -> list[dict[str, Any]]:
    conflicts: list[dict[str, Any]] = []
    for window in _resource_windows(route, planned_start, service_seconds, start_point):
        clash = ResourceReservation.objects.filter(
            resource_type=window['resource_type'], resource_id=window['resource_id'],
            status__in=RESERVED_STATUSES, starts_at__lt=window['ends_at'], ends_at__gt=window['starts_at'],
        ).select_related('schedule').order_by('starts_at').first()
        if clash:
            conflicts.append({
                'workpoint': window['workpoint'].code,
                'resource_type': window['resource_type'], 'resource_id': window['resource_id'],
                'starts_at': window['starts_at'].isoformat(), 'ends_at': window['ends_at'].isoformat(),
                'conflicting_schedule': clash.schedule.schedule_id,
            })
    return conflicts


def preview_schedule(data: dict[str, Any], runtime) -> dict[str, Any]:
    order = WarehouseOrder.objects.select_related('source','destination','warehouse').get(pk=int(data['order_id']))
    planned_start = _dt(data.get('planned_start'), default=timezone.now())
    route_ids = [int(x) for x in (data.get('route_workpoint_ids') or [])]
    middle = list(WorkPoint.objects.filter(id__in=route_ids, warehouse=order.warehouse, enabled=True))
    by_id = {x.id: x for x in middle}
    route = [order.source] + [by_id[x] for x in route_ids if x in by_id and x not in (order.source_id, order.destination_id)] + [order.destination]
    candidates = candidate_scores(order, planned_start, runtime.engine.state.get('robots') or {}, route)
    best = next((c for c in candidates if c['eligible']), None)
    requested_robot = str(data.get('robot_id') or '').strip()
    resource_candidate = next((c for c in candidates if c['robot_id'] == requested_robot and c['eligible']), None) if requested_robot else best
    service = data.get('service_seconds') if isinstance(data.get('service_seconds'), list) else []
    start_point = None
    if resource_candidate:
        rs = (runtime.engine.state.get('robots') or {}).get(resource_candidate['robot_id']) or {}
        pos = rs.get('position') or [0, 0, 0]
        start_point = (float(pos[0]), float(pos[2]), int(rs.get('floor') or 1))
    conflicts = _resource_conflicts(route, planned_start, service, start_point)
    return {
        'order': order_to_dict(order), 'planned_start': planned_start.isoformat(),
        'route': [workpoint_to_dict(x) for x in route], 'candidates': candidates,
        'recommended': best, 'resource_conflicts': conflicts,
    }


def _next_schedule_id() -> str:
    return f"SCH-{timezone.now().strftime('%Y%m%d')}-{uuid.uuid4().hex[:6].upper()}"


@transaction.atomic
def create_schedule(data: dict[str, Any], runtime, user=None) -> RobotSchedule:
    order = WarehouseOrder.objects.select_for_update().select_related('source','destination','warehouse').get(pk=int(data['order_id']))
    if order.status in ('COMPLETED','CANCELLED','FAILED'):
        raise ValueError(f'order is {order.status.lower()}')
    if order.schedules.filter(status__in=ACTIVE_SCHEDULE_STATUSES).exists():
        raise ValueError('order already has an active schedule')
    planned_start = _dt(data.get('planned_start'), default=timezone.now())
    mode = str(data.get('mode') or 'AUTO').upper()
    if mode not in dict(RobotSchedule.MODES): raise ValueError('invalid scheduler mode')
    route_ids = [int(x) for x in (data.get('route_workpoint_ids') or [])]
    mids = list(WorkPoint.objects.filter(id__in=route_ids, warehouse=order.warehouse, enabled=True))
    by_id = {x.id: x for x in mids}
    route = [order.source] + [by_id[x] for x in route_ids if x in by_id and x not in (order.source_id, order.destination_id)] + [order.destination]
    scores = candidate_scores(order, planned_start, runtime.engine.state.get('robots') or {}, route)
    requested_robot = str(data.get('robot_id') or '').strip()
    candidate = None
    if requested_robot:
        candidate = next((x for x in scores if x['robot_id'] == requested_robot), None)
        if not candidate: raise ValueError('unknown robot')
        if not candidate['eligible']: raise ValueError(f"robot {requested_robot} unavailable: {candidate['rejected_reason']}")
    else:
        candidate = next((x for x in scores if x['eligible']), None)
        if not candidate: raise ValueError('no eligible robot is available for this schedule')
    robot = RobotProfile.objects.get(pk=candidate['robot_profile_id'])
    duration = float(candidate['duration_s']) + sum(int(x) for x in (data.get('service_seconds') or []))
    planned_end = planned_start + timedelta(seconds=max(30, duration))
    if _conflicts(robot, planned_start, planned_end): raise ValueError(f'robot {robot.robot_id} has a conflicting schedule')

    schedule = RobotSchedule.objects.create(
        schedule_id=_next_schedule_id(), warehouse=order.warehouse, order=order, robot=robot,
        mode=mode, status='PLANNED', planned_start=planned_start, planned_end=planned_end,
        estimated_distance_m=float(candidate['distance_m']), estimated_duration_s=max(30, duration),
        score={'selected': candidate, 'candidates': scores[:8]}, created_by=user,
    )
    service = data.get('service_seconds') if isinstance(data.get('service_seconds'), list) else []
    for i, wp in enumerate(route):
        action = 'PICKUP' if i == 0 else 'DROPOFF' if i == len(route)-1 else 'TRANSIT'
        secs = int(service[i]) if i < len(service) else (20 if action in ('PICKUP','DROPOFF') else 0)
        ScheduleStop.objects.create(schedule=schedule, sequence=i, workpoint=wp, action=action, service_seconds=max(0, secs))
    # Reserve shared resources using the selected robot's estimated arrival times.
    rs = (runtime.engine.state.get('robots') or {}).get(robot.robot_id) or {}
    pos = rs.get('position') or [0, 0, 0]
    start_point = (float(pos[0]), float(pos[2]), int(rs.get('floor') or 1))
    for window in _resource_windows(route, planned_start, service, start_point):
        wp = window['workpoint']
        clash = ResourceReservation.objects.filter(
            resource_type=window['resource_type'], resource_id=window['resource_id'],
            status__in=RESERVED_STATUSES, starts_at__lt=window['ends_at'], ends_at__gt=window['starts_at'],
        ).exists()
        if clash:
            raise ValueError(f'resource conflict at {wp.code}')
        ResourceReservation.objects.create(
            schedule=schedule, workpoint=wp, resource_type=window['resource_type'],
            resource_id=window['resource_id'], starts_at=window['starts_at'], ends_at=window['ends_at'],
        )
    order.status = 'PLANNED'; order.save(update_fields=['status','updated_at'])
    return schedule


def cancel_schedule(schedule: RobotSchedule) -> RobotSchedule:
    if schedule.status in ('COMPLETED','FAILED','CANCELLED'):
        return schedule
    schedule.status = 'CANCELLED'; schedule.actual_end = timezone.now(); schedule.failure_reason = 'cancelled by operator'; schedule.save(update_fields=['status','actual_end','failure_reason','updated_at'])
    schedule.order.status = 'CANCELLED'; schedule.order.save(update_fields=['status','updated_at'])
    release_inventory_reservation(schedule.order)
    schedule.reservations.filter(status__in=RESERVED_STATUSES).update(status='CANCELLED')
    return schedule


def _register_engine_location(engine, wp: WorkPoint) -> None:
    engine.loc[wp.code] = {'id': wp.code, 'kind': wp.kind, 'zone': wp.zone.code if wp.zone else None, 'floor': wp.floor, 'rack_id': wp.resource_id if wp.kind == 'SHELF' else None, 'level_range': None, 'access_point': [wp.x, wp.y]}


def process_runtime_schedules(engine, plc=None) -> bool:
    """Advance persistent schedules against the simulation engine.

    Returns True if database/runtime state changed. It deliberately dispatches only
    explicitly planned schedules; SimEngine.external_scheduler suppresses random
    demo task generation/assignment.
    """
    now = timezone.now()
    changed = False
    qs = RobotSchedule.objects.filter(status__in=ACTIVE_SCHEDULE_STATUSES).select_related('order','robot').prefetch_related('stops__workpoint').order_by('planned_start','id')
    for sched in qs:
        stops = list(sched.stops.select_related('workpoint').order_by('sequence'))
        if len(stops) < 2:
            sched.status='FAILED'; sched.failure_reason='schedule requires at least two stops'; sched.actual_end=now; sched.save(); sched.order.status='FAILED'; sched.order.save(update_fields=['status','updated_at']); release_inventory_reservation(sched.order); changed=True; continue
        r = engine.state.get('robots', {}).get(sched.robot.robot_id)
        if not r:
            if sched.status == 'RUNNING':
                sched.status='FAILED'; sched.failure_reason='robot missing from runtime'; sched.actual_end=now; sched.save(); sched.order.status='FAILED'; sched.order.save(update_fields=['status','updated_at']); release_inventory_reservation(sched.order); changed=True
            continue
        if sched.status in ('PLANNED','QUEUED'):
            if sched.planned_start > now:
                continue
            if r.get('status') in ('OFFLINE','ERROR') or r.get('battery',0) < sched.robot.min_dispatch_battery:
                if sched.status != 'QUEUED':
                    sched.status='QUEUED'; sched.save(update_fields=['status','updated_at'])
                    sched.order.status='QUEUED'; sched.order.save(update_fields=['status','updated_at']); changed=True
                continue
            if r.get('fsm') not in ('IDLE','COMPLETED') or r.get('current_task_id'):
                if sched.status != 'QUEUED':
                    sched.status='QUEUED'; sched.save(update_fields=['status','updated_at'])
                    sched.order.status='QUEUED'; sched.order.save(update_fields=['status','updated_at']); changed=True
                continue
            # Start first leg.
            sched.status='RUNNING'; sched.actual_start = sched.actual_start or now; sched.order.status='RUNNING'
            sched.order.save(update_fields=['status','updated_at'])
            sched.reservations.filter(status='HELD').update(status='ACTIVE')
            sched.save(update_fields=['status', 'actual_start', 'updated_at'])
        if sched.status != 'RUNNING':
            continue
        leg = int(sched.current_leg)
        if leg >= len(stops)-1:
            sched.status='COMPLETED'; sched.actual_end=now; sched.engine_task_id=''; sched.save(update_fields=['status','actual_end','engine_task_id','updated_at'])
            sched.order.status='COMPLETED'; sched.order.save(update_fields=['status','updated_at'])
            apply_completed_flow_to_shelf(sched.order)
            sched.reservations.filter(status__in=RESERVED_STATUSES).update(status='RELEASED')
            for stop in stops:
                if stop.status != 'COMPLETED':
                    stop.status='COMPLETED'; stop.completed_at=now; stop.save(update_fields=['status','completed_at'])
            changed=True; continue
        # If a task is active, wait for its authoritative engine result.
        if sched.engine_task_id:
            task = engine.state.get('tasks', {}).get(sched.engine_task_id)
            if task and task.get('status') == 'COMPLETED':
                destination = stops[leg + 1].workpoint
                # Conveyor handoff is an explicit PLC/robot handshake. The
                # scheduler only requests the transport; it never drives the
                # motor or silently completes a conveyor stop.
                if plc and destination.kind == 'CONVEYOR_IN':
                    conveyor_id = destination.resource_id
                    try:
                        item = plc.command(conveyor_id, 'ENQUEUE', order_id=sched.order.order_no, load_units=sched.order.load_units, position_m=0.0)
                        item_id = item['items'][-1]['item_id']
                        plc.request_handshake(conveyor_id, sched.robot.robot_id, item_id, 'REQUEST_DROP')
                        plc.request_handshake(conveyor_id, sched.robot.robot_id, item_id, 'DROP_CONFIRMED')
                    except (KeyError, ValueError):
                        continue
                stops[leg].status='COMPLETED'; stops[leg].completed_at=now; stops[leg].save(update_fields=['status','completed_at'])
                stops[leg+1].status='COMPLETED'; stops[leg+1].arrived_at=now; stops[leg+1].completed_at=now; stops[leg+1].save(update_fields=['status','arrived_at','completed_at'])
                sched.current_leg = leg + 1; sched.engine_task_id=''; sched.save(update_fields=['current_leg','engine_task_id','updated_at']); changed=True
                continue
            if task and task.get('status') in ('FAILED','CANCELLED','TRANSFERRED'):
                sched.status='FAILED'; sched.failure_reason=f"engine task {sched.engine_task_id} ended as {task.get('status')}"; sched.actual_end=now; sched.save()
                sched.order.status='FAILED'; sched.order.save(update_fields=['status','updated_at']); release_inventory_reservation(sched.order); sched.reservations.filter(status__in=RESERVED_STATUSES).update(status='CANCELLED'); changed=True
                continue
            if not task:
                # Avoid false failure while just created/pruned; if robot is free, re-dispatch current leg.
                if r.get('fsm') not in ('IDLE','COMPLETED'):
                    continue
                sched.engine_task_id=''; sched.save(update_fields=['engine_task_id','updated_at'])
            else:
                continue
        # Dispatch next leg only when same robot is ready.
        if r.get('fsm') not in ('IDLE','COMPLETED') or r.get('current_task_id'):
            continue
        src, dst = stops[leg].workpoint, stops[leg+1].workpoint
        _register_engine_location(engine, src); _register_engine_location(engine, dst)
        if plc and src.kind == 'CONVEYOR_OUT':
            conveyor_id = src.resource_id
            belt = plc.snapshot().get(conveyor_id) or {}
            item = next((x for x in belt.get('items', []) if x.get('status') == 'AT_EXIT'), None)
            if item is None:
                continue
            try:
                plc.request_handshake(conveyor_id, sched.robot.robot_id, item['item_id'], 'REQUEST_PICKUP')
                plc.request_handshake(conveyor_id, sched.robot.robot_id, item['item_id'], 'PICKUP_CONFIRMED')
            except (KeyError, ValueError):
                continue
        try:
            task = engine.create_task('TRANSPORT', sched.order.priority, src.code, dst.code, sched.order.load_units, validate_rules=False)
            engine.assign_task(task['id'], sched.robot.robot_id, source='SCHEDULER')
        except ValueError:
            continue
        sched.engine_task_id = task['id']; sched.save(update_fields=['engine_task_id','updated_at'])
        stops[leg].status='ACTIVE'; stops[leg].arrived_at = stops[leg].arrived_at or now; stops[leg].save(update_fields=['status','arrived_at'])
        changed=True
    return changed


def prepare_external_dispatches(runtime) -> tuple[list[dict[str, Any]], bool]:
    """Service one stop at a time and return exact-stop Nav2 goals.

    ``current_leg`` is the index of the next stop to visit in external mode.
    Nav2 success only changes that stop to ARRIVED.  This scheduler performs its
    pickup/dropoff service, marks that same stop COMPLETED, then advances to the
    next index.  SimEngine is never asked to move the robot.
    """
    if not runtime.ros_bridge_connected:
        return [], False
    wh = ensure_scheduler_master_data(runtime)
    now = timezone.now()
    goals: list[dict[str, Any]] = []
    changed = False
    states = runtime.engine.state.get('robots', {})
    schedules = (RobotSchedule.objects.filter(warehouse=wh, status__in=ACTIVE_SCHEDULE_STATUSES)
                 .select_related('order', 'robot').prefetch_related('stops__workpoint').order_by('planned_start', 'id'))
    for sched in schedules:
        if sched.planned_start and sched.planned_start > now:
            continue
        robot = states.get(sched.robot.robot_id) or {}
        if robot.get('status') in ('OFFLINE', 'ERROR'):
            continue
        if sched.status in ('PLANNED', 'QUEUED'):
            sched.status = 'RUNNING'; sched.actual_start = sched.actual_start or now
            sched.order.status = 'RUNNING'
            sched.order.save(update_fields=['status', 'updated_at'])
            sched.save(update_fields=['status', 'actual_start', 'updated_at'])
            sched.reservations.filter(status='HELD').update(status='ACTIVE')
            changed = True
        stops = list(sched.stops.all().order_by('sequence'))
        leg = int(sched.current_leg or 0)
        if leg >= len(stops):
            _complete_external_schedule(sched, now)
            changed = True
            continue
        stop = stops[leg]

        if stop.status == 'ARRIVED':
            service_done_at = (stop.arrived_at or now) + timedelta(seconds=stop.service_seconds)
            if now < service_done_at:
                continue
            _apply_external_stop_service(runtime, sched, stop)
            stop.status = 'COMPLETED'
            stop.completed_at = now
            stop.save(update_fields=['status', 'completed_at'])
            sched.current_leg = leg + 1
            sched.engine_task_id = ''
            sched.save(update_fields=['current_leg', 'engine_task_id', 'updated_at'])
            changed = True
            leg += 1
            if leg >= len(stops):
                _complete_external_schedule(sched, now)
                continue
            stop = stops[leg]

        if sched.engine_task_id:
            continue
        if stop.status == 'COMPLETED':
            sched.current_leg = leg + 1
            sched.save(update_fields=['current_leg', 'updated_at'])
            changed = True
            continue
        if stop.status not in ('PENDING', 'ACTIVE'):
            continue

        sched.engine_task_id = f'ros:{sched.schedule_id}:{stop.id}'
        sched.save(update_fields=['engine_task_id', 'updated_at'])
        stop.status = 'ACTIVE'
        stop.save(update_fields=['status'])
        changed = True
        goals.append({'robot_id': sched.robot.robot_id, 'schedule_id': sched.schedule_id,
                      'stop_id': stop.id, 'frame_id': 'map', 'x': float(stop.workpoint.x),
                      'y': float(stop.workpoint.y), 'yaw': float(stop.workpoint.yaw or 0.0)})
    return goals, changed


def _apply_external_stop_service(runtime, sched: RobotSchedule, stop: ScheduleStop) -> None:
    """Reflect the completed pickup/dropoff in the external robot state cache."""
    robot = runtime.engine.state.get('robots', {}).get(sched.robot.robot_id)
    if robot is None:
        return
    load = robot.setdefault('load', {'current': 0, 'capacity': sched.robot.payload_capacity})
    if stop.action == 'PICKUP':
        load['current'] = min(int(load.get('capacity') or sched.robot.payload_capacity), sched.order.load_units)
    elif stop.action == 'DROPOFF':
        load['current'] = 0


def _complete_external_schedule(sched: RobotSchedule, now) -> None:
    if sched.status == 'COMPLETED':
        return
    sched.status = 'COMPLETED'
    sched.actual_end = now
    sched.engine_task_id = ''
    sched.current_leg = sched.stops.count()
    sched.save(update_fields=['status', 'actual_end', 'engine_task_id', 'current_leg', 'updated_at'])
    sched.order.status = 'COMPLETED'
    sched.order.save(update_fields=['status', 'updated_at'])
    apply_completed_flow_to_shelf(sched.order)
    sched.reservations.filter(status__in=RESERVED_STATUSES).update(status='RELEASED')


def release_external_dispatch(schedule_id: str, stop_id: int) -> bool:
    """Undo an unsent goal after a bridge race/disconnect; never advance it."""
    try:
        sched = RobotSchedule.objects.get(schedule_id=schedule_id)
        stop = sched.stops.get(pk=stop_id)
    except (RobotSchedule.DoesNotExist, ScheduleStop.DoesNotExist, TypeError, ValueError):
        return False
    expected = f'ros:{sched.schedule_id}:{stop.id}'
    if sched.engine_task_id != expected:
        return False
    sched.engine_task_id = ''
    sched.save(update_fields=['engine_task_id', 'updated_at'])
    if stop.status == 'ACTIVE':
        stop.status = 'PENDING'
        stop.save(update_fields=['status'])
    return True


def apply_external_nav_status(data: dict[str, Any]) -> bool:
    """Apply an authoritative Nav2 result to the existing scheduler lifecycle."""
    schedule_id = str(data.get('schedule_id') or '')
    if not schedule_id:
        return False
    try:
        sched = (RobotSchedule.objects.select_related('order')
                 .prefetch_related('stops__workpoint').get(schedule_id=schedule_id))
    except RobotSchedule.DoesNotExist:
        return False
    status = str(data.get('status') or '').upper()
    now = timezone.now()
    try:
        stop_id = int(data.get('stop_id'))
    except (TypeError, ValueError):
        return False
    if str(data.get('robot_id') or '') != sched.robot.robot_id:
        return False
    stops = list(sched.stops.all().order_by('sequence'))
    leg = int(sched.current_leg or 0)
    if leg >= len(stops):
        return False
    stop = stops[leg]
    expected_task = f'ros:{sched.schedule_id}:{stop.id}'
    if stop.id != stop_id or sched.engine_task_id != expected_task:
        return False
    if status == 'ACTIVE':
        if sched.status in ('PLANNED', 'QUEUED'):
            sched.status = 'RUNNING'; sched.actual_start = sched.actual_start or now
            sched.order.status = 'RUNNING'; sched.order.save(update_fields=['status', 'updated_at'])
            sched.save(update_fields=['status', 'actual_start', 'updated_at'])
        return True
    if status in ('FAILED', 'CANCELED', 'CANCELLED'):
        stop.status = 'FAILED'
        stop.save(update_fields=['status'])
        sched.status = 'FAILED' if status == 'FAILED' else 'CANCELLED'
        reason = str(data.get('reason') or f'Nav2 {status.lower()}')
        sched.failure_reason = reason[:1000]
        sched.actual_end = now; sched.engine_task_id = ''
        sched.save(update_fields=['status', 'failure_reason', 'actual_end', 'engine_task_id', 'updated_at'])
        sched.order.status = 'FAILED' if status == 'FAILED' else 'CANCELLED'
        sched.order.save(update_fields=['status', 'updated_at'])
        release_inventory_reservation(sched.order)
        sched.reservations.filter(status__in=RESERVED_STATUSES).update(status='CANCELLED')
        return True
    if status != 'SUCCEEDED':
        return False
    stop.status = 'ARRIVED'
    stop.arrived_at = now
    stop.save(update_fields=['status', 'arrived_at'])
    sched.engine_task_id = ''
    sched.save(update_fields=['engine_task_id', 'updated_at'])
    return True


def overview(runtime) -> dict[str, Any]:
    wh = ensure_scheduler_master_data(runtime)
    now = timezone.now()
    orders = WarehouseOrder.objects.filter(warehouse=wh)
    schedules = RobotSchedule.objects.filter(warehouse=wh)
    return {
        'warehouse_id': wh.id,
        'orders': {
            'total': orders.count(), 'new': orders.filter(status='NEW').count(), 'planned': orders.filter(status__in=('PLANNED','QUEUED')).count(),
            'running': orders.filter(status='RUNNING').count(), 'completed': orders.filter(status='COMPLETED').count(), 'failed': orders.filter(status='FAILED').count(),
        },
        'flows': {
            'inbound': {
                'active': orders.filter(type='INBOUND', status__in=('NEW','PLANNED','QUEUED','RUNNING')).count(),
                'running': orders.filter(type='INBOUND', status='RUNNING').count(),
                'completed': orders.filter(type='INBOUND', status='COMPLETED').count(),
            },
            'outbound': {
                'active': orders.filter(type='OUTBOUND', status__in=('NEW','PLANNED','QUEUED','RUNNING')).count(),
                'running': orders.filter(type='OUTBOUND', status='RUNNING').count(),
                'completed': orders.filter(type='OUTBOUND', status='COMPLETED').count(),
            },
        },
        'schedules': {
            'active': schedules.filter(status__in=ACTIVE_SCHEDULE_STATUSES).count(),
            'running': schedules.filter(status='RUNNING').count(),
            'next': [schedule_to_dict(x, include_stops=False) for x in schedules.filter(status__in=('PLANNED','QUEUED'), planned_start__gte=now).select_related('order','robot').order_by('planned_start')[:5]],
        },
        'workpoints': WorkPoint.objects.filter(warehouse=wh, enabled=True).count(),
        'robots': RobotProfile.objects.filter(warehouse=wh, enabled=True).count(),
    }
