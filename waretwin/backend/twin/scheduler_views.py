from __future__ import annotations

import json
import uuid
from typing import Any

from asgiref.sync import async_to_sync
from channels.layers import get_channel_layer
from django.db import IntegrityError, transaction
from django.db.models import Count, Q
from django.http import JsonResponse
from django.utils import timezone
from django.views.decorators.csrf import csrf_exempt
from django.views.decorators.http import require_http_methods

from .models import RobotProfile, RobotSchedule, Shelf, ShelfInventoryItem, WarehouseOrder, WorkPoint
from .runtime import runtime
from .schedule_services import (
    active_warehouse,
    _flow_kind,
    cancel_schedule,
    create_order,
    create_schedule,
    ensure_scheduler_master_data,
    inventory_item_to_dict,
    order_to_dict,
    overview,
    preview_schedule,
    release_inventory_reservation,
    reserve_inventory_for_order,
    robot_to_dict,
    schedule_to_dict,
    sync_robot_profiles,
    sync_workpoints_from_layout,
    validate_flow_route,
    workpoint_for_shelf,
    workpoint_to_dict,
)
from .order_import_services import auto_schedule_order, import_and_auto_schedule
from .views import _audit, _body, _error, api_admin_required, api_user_required


def _notify(source: str = 'scheduler') -> None:
    layer = get_channel_layer()
    if layer is None:
        return
    async_to_sync(layer.group_send)('twin_clients', {
        'type': 'twin.message',
        'payload': {'type': 'SCHEDULE_UPDATED', 'source': source},
    })


@csrf_exempt
@api_user_required
@require_http_methods(['GET'])
def scheduler_overview(request):
    return JsonResponse(overview(runtime))


@csrf_exempt
@api_admin_required
@require_http_methods(['POST'])
def scheduler_sync(request):
    try:
        wh = active_warehouse()
        wp = sync_workpoints_from_layout(wh, runtime.layout, prune=False)
        rp = sync_robot_profiles(wh, runtime.engine.state.get('robots') or {})
    except ValueError as exc:
        return _error(str(exc), 409)
    _audit(request.api_user, 'SCHEDULER_SYNC', f'warehouse={wh.id}')
    _notify('scheduler-sync')
    return JsonResponse({'warehouse_id': wh.id, 'workpoints': wp, 'robots': rp})


@csrf_exempt
@api_user_required
@require_http_methods(['GET'])
def workpoints(request):
    try:
        wh = ensure_scheduler_master_data(runtime)
    except ValueError as exc:
        return _error(str(exc), 409)
    q = WorkPoint.objects.filter(warehouse=wh)
    if request.GET.get('enabled') != 'all': q = q.filter(enabled=True)
    kind = request.GET.get('kind')
    if kind: q = q.filter(kind=kind.upper())
    search = (request.GET.get('search') or '').strip()
    if search: q = q.filter(Q(code__icontains=search) | Q(name__icontains=search))
    return JsonResponse([workpoint_to_dict(x) for x in q.order_by('kind','code')], safe=False)


@csrf_exempt
@api_user_required
@require_http_methods(['GET'])
def scheduler_robots(request):
    try:
        wh = ensure_scheduler_master_data(runtime)
    except ValueError as exc:
        return _error(str(exc), 409)
    profiles = RobotProfile.objects.filter(warehouse=wh, enabled=True).order_by('robot_id')
    state = runtime.engine.state.get('robots') or {}
    return JsonResponse([robot_to_dict(x, state) for x in profiles], safe=False)


def _shelf_by_rack(warehouse, rack_id: str) -> Shelf | None:
    token = str(rack_id or '').strip()
    if not token:
        return None
    return Shelf.objects.filter(zone__warehouse=warehouse).filter(
        Q(layout_rack_id__iexact=token) | Q(code__iexact=token)
    ).select_related('zone').first()


def _destination_shelf_rows(warehouse, current_shelf: Shelf) -> list[dict[str, Any]]:
    active = ('NEW', 'PLANNED', 'QUEUED', 'RUNNING')
    shelf_points: dict[str, WorkPoint] = {}
    for point in WorkPoint.objects.filter(warehouse=warehouse, enabled=True, kind='SHELF').order_by('id'):
        shelf_points.setdefault(str(point.resource_id or point.code), point)
    pending_by_resource = {
        str(row['destination__resource_id'] or ''): int(row['total'])
        for row in WarehouseOrder.objects.filter(
            warehouse=warehouse, type__in=('INBOUND', 'TRANSFER'), status__in=active
        ).values('destination__resource_id').annotate(total=Count('id'))
    }
    rows: list[dict[str, Any]] = []
    for shelf in Shelf.objects.filter(zone__warehouse=warehouse).exclude(pk=current_shelf.pk).select_related('zone').order_by('code'):
        if shelf.status in ('DISABLED', 'MAINTENANCE'):
            continue
        rack_id = str(shelf.layout_rack_id or shelf.code)
        point = shelf_points.get(rack_id)
        if point is None:
            continue
        pending = pending_by_resource.get(str(point.resource_id or point.code), 0)
        effective = max(0, min(8, int(shelf.current_load))) + pending
        rows.append({
            'shelf_id': shelf.id, 'shelf_code': rack_id, 'zone': shelf.zone.code,
            'floor': shelf.floor, 'current_load': int(shelf.current_load),
            'reserved_in': pending, 'available_slots': max(0, 8 - effective),
            'enabled': effective < 8,
        })
    return rows


def _nearest_outbound_point(warehouse, source: WorkPoint, requested: str = '') -> WorkPoint:
    token = str(requested or '').strip()
    points = [p for p in WorkPoint.objects.filter(warehouse=warehouse, enabled=True).order_by('code') if _flow_kind(p) == 'OUTBOUND']
    if token:
        for point in points:
            if token.lower() in {point.code.lower(), str(point.resource_id or '').lower(), point.name.lower()}:
                return point
        raise ValueError(f'unknown outbound dock {token}')
    if not points:
        raise ValueError('no enabled OUTBOUND dock is configured')
    return min(points, key=lambda p: ((float(p.x)-float(source.x))**2 + (float(p.y)-float(source.y))**2, p.code))


@csrf_exempt
@api_user_required
@require_http_methods(['GET'])
def shelf_inventory(request, rack_id: str):
    try:
        warehouse = ensure_scheduler_master_data(runtime)
    except ValueError as exc:
        return _error(str(exc), 409)
    shelf = _shelf_by_rack(warehouse, rack_id)
    if shelf is None:
        return _error('shelf not found', 404)
    items = ShelfInventoryItem.objects.filter(
        warehouse=warehouse, shelf=shelf, status__in=('STORED', 'RESERVED')
    ).select_related('shelf', 'origin_order', 'last_movement_order', 'reserved_by_order').order_by('stored_at', 'id')
    shelf_point = workpoint_for_shelf(shelf)
    outbound_points = []
    if shelf_point is not None:
        outbound_points = [
            workpoint_to_dict(p) for p in WorkPoint.objects.filter(warehouse=warehouse, enabled=True).order_by('code')
            if _flow_kind(p) == 'OUTBOUND'
        ]
    return JsonResponse({
        'shelf': {
            'id': shelf.id, 'code': shelf.code, 'rack_id': shelf.layout_rack_id or shelf.code,
            'zone': shelf.zone.code, 'floor': shelf.floor, 'current_load': int(shelf.current_load),
            'capacity': 8, 'percent': round(max(0, min(8, int(shelf.current_load))) * 12.5, 1),
        },
        'items': [inventory_item_to_dict(x) for x in items],
        'outbound_docks': outbound_points,
        'destination_shelves': _destination_shelf_rows(warehouse, shelf),
    })


@csrf_exempt
@api_admin_required
@require_http_methods(['POST'])
def inventory_item_dispatch(request, item_id: int):
    data = _body(request)
    action = str(data.get('action') or '').upper().strip()
    if action not in ('OUTBOUND', 'TRANSFER'):
        return _error('action must be OUTBOUND or TRANSFER', 400)
    prefer_unloaded = bool(data.get('prefer_unloaded_robot', False))
    priority = str(data.get('priority') or 'NORMAL').upper()
    if priority not in dict(WarehouseOrder.PRIORITIES):
        return _error('invalid priority', 400)
    try:
        warehouse = ensure_scheduler_master_data(runtime)
        with transaction.atomic():
            item = ShelfInventoryItem.objects.select_for_update().select_related('shelf__zone').get(
                pk=item_id, warehouse=warehouse
            )
            if item.status != 'STORED' or item.shelf_id is None or item.reserved_by_order_id:
                raise ValueError('this item is already reserved, moving, or no longer on a shelf')
            source_shelf = item.shelf

            if action == 'OUTBOUND':
                # Serialize actions from the same shelf while route/capacity is
                # validated and the exact item is reserved.
                source_shelf = Shelf.objects.select_for_update().select_related('zone').get(pk=source_shelf.pk)
                item.shelf = source_shelf
                source = workpoint_for_shelf(source_shelf)
                if source is None:
                    raise ValueError(f'shelf {source_shelf.code} has no enabled robot access work-point')
                destination = _nearest_outbound_point(warehouse, source, str(data.get('outbound_code') or ''))
                prefix = 'OUT'
                notes = f'Operator shelf action: {item.item_code or item.item_uid} to {destination.code}'
            else:
                target_code = str(data.get('destination_shelf_code') or '').strip()
                if not target_code:
                    raise ValueError('destination_shelf_code is required for TRANSFER')
                target_shelf = _shelf_by_rack(warehouse, target_code)
                if target_shelf is None:
                    raise ValueError(f'unknown destination shelf {target_code}')
                if target_shelf.pk == source_shelf.pk:
                    raise ValueError('destination shelf must differ from source shelf')
                # Lock both shelves in a stable primary-key order. This prevents
                # deadlocks when two operators try opposite A→B / B→A transfers.
                locked = {
                    shelf.pk: shelf for shelf in Shelf.objects.select_for_update().select_related('zone')
                    .filter(pk__in=[source_shelf.pk, target_shelf.pk]).order_by('pk')
                }
                source_shelf = locked[source_shelf.pk]
                target_shelf = locked[target_shelf.pk]
                item.shelf = source_shelf
                source = workpoint_for_shelf(source_shelf)
                if source is None:
                    raise ValueError(f'shelf {source_shelf.code} has no enabled robot access work-point')
                destination = workpoint_for_shelf(target_shelf)
                if destination is None:
                    raise ValueError(f'shelf {target_code} has no enabled robot access work-point')
                prefix = 'MOV'
                notes = f'Operator shelf action: {item.item_code or item.item_uid} to {target_shelf.layout_rack_id or target_shelf.code}'

            order = create_order({
                'warehouse_id': warehouse.id,
                'order_no': f'{prefix}-{timezone.now().strftime("%Y%m%d-%H%M%S")}-{item.id}-{uuid.uuid4().hex[:5].upper()}',
                'external_ref': item.external_ref,
                'type': action,
                'priority': priority,
                'source_id': source.id,
                'destination_id': destination.id,
                'quantity': max(1, int(item.quantity)),
                'load_units': max(1, int(item.load_units)),
                'payload_weight_kg': max(0.0, float(item.payload_weight_kg)),
                'notes': notes,
                'metadata': {
                    'created_from': 'shelf_item_action',
                    'inventory_item_id': item.id,
                    'inventory_item_uid': item.item_uid,
                    'item_code': item.item_code,
                    'item_name': item.item_name,
                    'source_shelf_code': item.shelf.layout_rack_id or item.shelf.code,
                    'prefer_unloaded_robot': prefer_unloaded,
                },
            }, request.api_user)
            reserve_inventory_for_order(order, item_id=item.id)
            schedule = auto_schedule_order(
                order, runtime, request.api_user, earliest_start=timezone.now(),
                prefer_unloaded_robot=prefer_unloaded,
            )
            order.refresh_from_db()
            item.refresh_from_db()

        _audit(request.api_user, 'DISPATCH_SHELF_ITEM', f'item={item.item_uid} action={action} robot={schedule.robot.robot_id}')
        _notify('shelf-item-dispatch')
        return JsonResponse({
            'ok': True, 'action': action, 'prefer_unloaded_robot': prefer_unloaded,
            'item': inventory_item_to_dict(item),
            'order': order_to_dict(order),
            'schedule': schedule_to_dict(schedule),
            'selected_robot': robot_to_dict(schedule.robot, runtime.engine.state.get('robots') or {}),
        }, status=201)
    except ShelfInventoryItem.DoesNotExist:
        return _error('inventory item not found', 404)
    except (ValueError, KeyError, TypeError, IntegrityError) as exc:
        return _error(str(exc), 409)


@csrf_exempt
@api_user_required
@require_http_methods(['GET', 'POST'])
def orders(request):
    if request.method == 'GET':
        try: wh = ensure_scheduler_master_data(runtime)
        except ValueError as exc: return _error(str(exc), 409)
        q = WarehouseOrder.objects.filter(warehouse=wh).select_related('source','destination','warehouse')
        status = request.GET.get('status')
        if status: q = q.filter(status=status.upper())
        priority = request.GET.get('priority')
        if priority: q = q.filter(priority=priority.upper())
        limit = min(500, max(1, int(request.GET.get('limit') or 200)))
        return JsonResponse([order_to_dict(x) for x in q[:limit]], safe=False)
    if getattr(request.api_user, 'waretwin_role', None) != 'admin':
        return _error('admin privileges required', 403)
    try:
        ensure_scheduler_master_data(runtime)
        obj = create_order(_body(request), request.api_user)
    except (ValueError, KeyError, WorkPoint.DoesNotExist) as exc:
        return _error(str(exc), 400)
    except IntegrityError:
        return _error('order_no already exists', 409)
    _audit(request.api_user, 'CREATE_ORDER', obj.order_no)
    _notify('order-created')
    return JsonResponse(order_to_dict(obj), status=201)


@csrf_exempt
@api_admin_required
@require_http_methods(['POST'])
def order_import(request):
    """Import one INBOUND *or* one OUTBOUND file, then auto-schedule.

    The two flows are intentionally sequential. INBOUND must complete first so
    exact ShelfInventoryItem records exist before OUTBOUND tries to reserve them.
    """
    try:
        wh = ensure_scheduler_master_data(runtime)
    except ValueError as exc:
        return _error(str(exc), 409)

    files = []
    inbound = request.FILES.get('inbound_file')
    outbound = request.FILES.get('outbound_file')
    if inbound is not None and outbound is not None:
        return _error('import INBOUND first and wait for completion, then import OUTBOUND in a separate request', 400)
    if inbound is not None:
        files.append(('INBOUND', inbound))
    if outbound is not None:
        files.append(('OUTBOUND', outbound))
    if not files:
        return _error('attach one inbound_file or outbound_file (.xlsx or .json)', 400)

    try:
        result = import_and_auto_schedule(warehouse=wh, runtime=runtime, user=request.api_user, files=files)
    except ValueError as exc:
        return _error(str(exc), 400)

    summary = result.get('summary') or {}
    _audit(request.api_user, 'IMPORT_AUTO_SCHEDULE_ORDERS', f"batch={result.get('batch_id')} rows={summary.get('rows', 0)} scheduled={summary.get('scheduled', 0)}")
    _notify('order-import-auto-schedule')
    return JsonResponse(result, status=201)


@csrf_exempt
@api_admin_required
@require_http_methods(['GET', 'PATCH', 'DELETE'])
def order_detail(request, order_id: int):
    try:
        obj = WarehouseOrder.objects.select_related('source','destination','warehouse').get(pk=order_id)
    except WarehouseOrder.DoesNotExist:
        return _error('order not found', 404)
    if request.method == 'GET': return JsonResponse(order_to_dict(obj))
    if request.method == 'DELETE':
        if obj.status == 'COMPLETED':
            return _error('completed orders are retained as operation history', 409)
        if obj.schedules.exclude(status__in=('CANCELLED','FAILED','COMPLETED')).exists():
            return _error('cannot delete an order with an active schedule', 409)
        target = obj.order_no
        # Deleting an unscheduled OUTBOUND/TRANSFER must not strand the exact
        # shelf item in RESERVED forever.
        release_inventory_reservation(obj)
        obj.delete(); _audit(request.api_user, 'DELETE_ORDER', target); _notify('order-deleted'); return JsonResponse({'ok': True})
    d = _body(request)
    if obj.status in ('RUNNING','COMPLETED'):
        return _error('running/completed orders cannot be edited', 409)
    active_schedule = obj.schedules.filter(status__in=('PLANNED','QUEUED','RUNNING')).exists()
    plan_fields = {'source_id','destination_id','type','priority','quantity','load_units','payload_weight_kg','due_at'}
    if active_schedule and any(k in d for k in plan_fields):
        return _error('cancel the active schedule before changing order planning fields', 409)
    # Shelf-click movements reserve one exact item. Changing the route/type under
    # that reservation could move the right item to the wrong location, so force
    # an explicit cancel/re-dispatch instead.
    if any(k in d for k in {'source_id','destination_id','type'}) and ShelfInventoryItem.objects.filter(
        reserved_by_order=obj, status='RESERVED'
    ).exists():
        return _error('cancel this shelf-item movement before changing its route or type', 409)
    try:
        if 'source_id' in d:
            obj.source = WorkPoint.objects.get(pk=int(d['source_id']), warehouse=obj.warehouse, enabled=True)
        if 'destination_id' in d:
            obj.destination = WorkPoint.objects.get(pk=int(d['destination_id']), warehouse=obj.warehouse, enabled=True)
        if obj.source_id == obj.destination_id:
            raise ValueError('source and destination must differ')
        if 'type' in d:
            value = str(d['type']).upper()
            if value not in dict(WarehouseOrder.TYPES): raise ValueError('invalid order type')
            obj.type = value
        if 'priority' in d:
            value = str(d['priority']).upper()
            if value not in dict(WarehouseOrder.PRIORITIES): raise ValueError('invalid priority')
            obj.priority = value
        if 'quantity' in d:
            obj.quantity = max(1, int(d['quantity']))
        if 'load_units' in d:
            value = int(d['load_units'])
            if value < 1 or value > 4: raise ValueError('load_units must be between 1 and 4')
            obj.load_units = value
        if 'payload_weight_kg' in d:
            obj.payload_weight_kg = max(0.0, float(d['payload_weight_kg']))
        if 'due_at' in d:
            from .schedule_services import _dt
            obj.due_at = _dt(d.get('due_at'))
        if 'notes' in d: obj.notes = str(d.get('notes') or '')
        if 'external_ref' in d: obj.external_ref = str(d.get('external_ref') or '')[:128]
        validate_flow_route(obj.type, obj.source, obj.destination, exclude_order_id=obj.id)
        obj.save()
    except WorkPoint.DoesNotExist:
        return _error('unknown or disabled work-point', 400)
    except (ValueError, TypeError) as exc:
        return _error(str(exc), 400)
    _audit(request.api_user, 'UPDATE_ORDER', obj.order_no); _notify('order-updated')
    return JsonResponse(order_to_dict(obj))


@csrf_exempt
@api_admin_required
@require_http_methods(['POST'])
def order_cancel(request, order_id: int):
    try:
        obj = WarehouseOrder.objects.select_related('source','destination','warehouse').get(pk=order_id)
    except WarehouseOrder.DoesNotExist:
        return _error('order not found', 404)
    active = obj.schedules.filter(status__in=('PLANNED','QUEUED','RUNNING')).select_related('order','robot')
    for schedule in active:
        if schedule.engine_task_id and not runtime.is_external:
            runtime.engine.cancel_task(schedule.engine_task_id, source='SCHEDULER')
        cancel_schedule(schedule)
        if runtime.is_external:
            from asgiref.sync import async_to_sync
            async_to_sync(runtime.gateway().send_command)(schedule.robot.robot_id, 'CANCEL_NAVIGATION',
                                                           {'schedule_id': schedule.schedule_id})
    if obj.status not in ('COMPLETED','FAILED'):
        obj.status = 'CANCELLED'; obj.save(update_fields=['status','updated_at'])
        release_inventory_reservation(obj)
    _audit(request.api_user, 'CANCEL_ORDER', obj.order_no)
    _notify('order-cancelled')
    return JsonResponse(order_to_dict(obj))


@csrf_exempt
@api_user_required
@require_http_methods(['GET'])
def schedules(request):
    try: wh = ensure_scheduler_master_data(runtime)
    except ValueError as exc: return _error(str(exc), 409)
    q = RobotSchedule.objects.filter(warehouse=wh).select_related('order','robot','warehouse')
    status = request.GET.get('status')
    if status: q = q.filter(status=status.upper())
    robot_id = request.GET.get('robot_id')
    if robot_id: q = q.filter(robot__robot_id=robot_id)
    limit = min(500, max(1, int(request.GET.get('limit') or 200)))
    return JsonResponse([schedule_to_dict(x) for x in q[:limit]], safe=False)


@csrf_exempt
@api_admin_required
@require_http_methods(['POST'])
def schedule_preview(request):
    try:
        ensure_scheduler_master_data(runtime)
        result = preview_schedule(_body(request), runtime)
        return JsonResponse(result)
    except (ValueError, KeyError, WarehouseOrder.DoesNotExist) as exc:
        return _error(str(exc), 400)


@csrf_exempt
@api_admin_required
@require_http_methods(['POST'])
def schedule_create(request):
    try:
        ensure_scheduler_master_data(runtime)
        obj = create_schedule(_body(request), runtime, request.api_user)
    except (ValueError, KeyError, WarehouseOrder.DoesNotExist, RobotProfile.DoesNotExist) as exc:
        return _error(str(exc), 409)
    _audit(request.api_user, 'CREATE_SCHEDULE', obj.schedule_id)
    _notify('schedule-created')
    return JsonResponse(schedule_to_dict(obj), status=201)


@csrf_exempt
@api_user_required
@require_http_methods(['GET'])
def schedule_detail(request, schedule_id: int):
    try:
        obj = RobotSchedule.objects.select_related('order','robot','warehouse').get(pk=schedule_id)
    except RobotSchedule.DoesNotExist:
        return _error('schedule not found', 404)
    return JsonResponse(schedule_to_dict(obj))


@csrf_exempt
@api_admin_required
@require_http_methods(['POST'])
def schedule_cancel(request, schedule_id: int):
    try:
        obj = RobotSchedule.objects.select_related('order','robot').get(pk=schedule_id)
    except RobotSchedule.DoesNotExist:
        return _error('schedule not found', 404)
    if obj.engine_task_id and not runtime.is_external:
        runtime.engine.cancel_task(obj.engine_task_id, source='SCHEDULER')
    cancel_schedule(obj)
    if runtime.is_external:
        from asgiref.sync import async_to_sync
        async_to_sync(runtime.gateway().send_command)(obj.robot.robot_id, 'CANCEL_NAVIGATION',
                                                       {'schedule_id': obj.schedule_id, 'stop_id': obj.current_leg})
    _audit(request.api_user, 'CANCEL_SCHEDULE', obj.schedule_id)
    _notify('schedule-cancelled')
    return JsonResponse(schedule_to_dict(obj))
