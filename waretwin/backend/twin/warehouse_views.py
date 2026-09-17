from __future__ import annotations

import json
from typing import Any

from django.db import IntegrityError
from asgiref.sync import async_to_sync
from channels.layers import get_channel_layer
from django.db.models import Count, Q
from django.db.models.deletion import ProtectedError
from django.core.exceptions import ValidationError as DjangoValidationError
from django.http import HttpRequest, JsonResponse
from django.views.decorators.csrf import csrf_exempt
from django.views.decorators.http import require_http_methods

from .auth import user_from_request
from .models import Warehouse, Zone, Shelf, WarehouseMap
from .runtime import runtime
from .warehouse_services import (
    apply_warehouse_data, apply_zone_data, apply_shelf_data,
    warehouse_to_dict, zone_to_dict, shelf_to_dict, sync_from_layout,
    commit_master_to_map, ensure_active_map, ensure_warehouse_map, set_active_map,
)
from accounts.models import role_of


def _body(request: HttpRequest) -> dict[str, Any]:
    try:
        data = json.loads(request.body.decode('utf-8')) if request.body else {}
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def _error(message: str, status: int = 400):
    return JsonResponse({'detail': message}, status=status)


def _user(request: HttpRequest, *, admin: bool = False):
    user = user_from_request(request)
    if user is None:
        return None, _error('authentication required', 401)
    if admin and role_of(user) != 'admin':
        return None, _error('admin privileges required', 403)
    return user, None


def _audit(user, action: str, target: str):
    runtime.engine.emit('ADMIN_ACTION', 'USER', 'INFO', f'ADMIN_ACTION admin={user.username} action={action} target={target}', payload={'action': action, 'admin': user.username, 'target': target, 'result': 'SUCCESS'})


def _map_meta(map_obj: WarehouseMap) -> dict[str, Any]:
    return {
        'warehouse_id': map_obj.warehouse_id,
        'layout_id': (map_obj.layout or {}).get('id') or map_obj.warehouse.layout_id,
        'revision': map_obj.revision,
        'published_version': map_obj.published_version,
        'is_active': map_obj.is_active,
        'updated_at': map_obj.updated_at.isoformat() if map_obj.updated_at else None,
    }


def _broadcast_map(map_obj: WarehouseMap, source: str) -> None:
    """Update runtime if this is the active map and notify every connected page."""
    if map_obj.is_active:
        runtime.replace_layout(map_obj.layout)
    try:
        from .schedule_services import sync_workpoints_from_layout, sync_robot_profiles
        sync_workpoints_from_layout(map_obj.warehouse, map_obj.layout, prune=False)
        if map_obj.is_active:
            sync_robot_profiles(map_obj.warehouse, runtime.engine.state.get('robots') or {})
    except Exception:
        pass
    layer = get_channel_layer()
    if layer is None:
        return
    payload = {'type': 'LAYOUT_UPDATED', 'source': source, **_map_meta(map_obj)}
    async_to_sync(layer.group_send)('twin_clients', {'type': 'twin.message', 'payload': payload})
    if map_obj.is_active:
        async_to_sync(layer.group_send)('twin_clients', {'type': 'twin.message', 'payload': runtime.full_message()})


def _commit_map(warehouse: Warehouse, user, source: str) -> WarehouseMap:
    fallback = None if source == 'CREATE_WAREHOUSE' else runtime.layout
    obj = commit_master_to_map(warehouse, user=user, source=source, fallback_layout=fallback)
    _broadcast_map(obj, source)
    return obj


def _save(obj, apply, data, **kwargs):
    try:
        apply(obj, data, **kwargs)
        obj.full_clean(validate_unique=False, validate_constraints=False)
        obj.save()
        return None
    except ValueError as exc:
        return _error(str(exc))
    except DjangoValidationError as exc:
        if hasattr(exc, 'message_dict'):
            detail = '; '.join(f"{k}: {', '.join(map(str, v))}" for k, v in exc.message_dict.items())
        else:
            detail = '; '.join(map(str, exc.messages))
        return _error(detail)
    except IntegrityError:
        return _error('duplicate code in the selected parent', 409)


@csrf_exempt
@require_http_methods(['GET', 'POST'])
def warehouses(request: HttpRequest):
    user, error = _user(request, admin=request.method == 'POST')
    if error: return error
    if request.method == 'GET':
        q = Warehouse.objects.annotate(zone_count=Count('zones', distinct=True), shelf_count=Count('zones__shelves', distinct=True))
        search = request.GET.get('search', '').strip()
        status = request.GET.get('status', '').strip().upper()
        if search: q = q.filter(Q(code__icontains=search) | Q(name__icontains=search))
        if status: q = q.filter(status=status)
        return JsonResponse([warehouse_to_dict(x) for x in q], safe=False)
    obj = Warehouse()
    err = _save(obj, apply_warehouse_data, _body(request))
    if err: return err
    _audit(user, 'CREATE_WAREHOUSE', obj.code)
    _commit_map(obj, user, 'CREATE_WAREHOUSE')
    return JsonResponse(warehouse_to_dict(obj), status=201)


@csrf_exempt
@require_http_methods(['GET', 'PATCH', 'DELETE'])
def warehouse_detail(request: HttpRequest, warehouse_id: int):
    user, error = _user(request, admin=request.method != 'GET')
    if error: return error
    try: obj = Warehouse.objects.get(id=warehouse_id)
    except Warehouse.DoesNotExist: return _error('warehouse not found', 404)
    if request.method == 'GET': return JsonResponse(warehouse_to_dict(obj))
    if request.method == 'DELETE':
        try:
            if hasattr(obj, 'map') and obj.map.is_active and Warehouse.objects.filter(status='ACTIVE').exclude(pk=obj.pk).exists():
                return _error('cannot delete the active warehouse map; activate another warehouse first', 409)
        except WarehouseMap.DoesNotExist:
            pass
        try: obj.delete()
        except ProtectedError: return _error('cannot delete warehouse because it contains zones', 409)
        _audit(user, 'DELETE_WAREHOUSE', str(warehouse_id))
        return JsonResponse({'ok': True})
    err = _save(obj, apply_warehouse_data, _body(request))
    if err: return err
    _audit(user, 'UPDATE_WAREHOUSE', obj.code)
    _commit_map(obj, user, 'UPDATE_WAREHOUSE')
    return JsonResponse(warehouse_to_dict(obj))


@csrf_exempt
@require_http_methods(['GET', 'POST'])
def zones(request: HttpRequest):
    user, error = _user(request, admin=request.method == 'POST')
    if error: return error
    if request.method == 'GET':
        q = Zone.objects.select_related('warehouse').annotate(shelf_count=Count('shelves'))
        warehouse_id = request.GET.get('warehouse')
        search = request.GET.get('search', '').strip()
        status = request.GET.get('status', '').strip().upper()
        ztype = request.GET.get('type', '').strip().upper()
        floor = request.GET.get('floor')
        if warehouse_id: q = q.filter(warehouse_id=warehouse_id)
        if search: q = q.filter(Q(code__icontains=search) | Q(name__icontains=search))
        if status: q = q.filter(status=status)
        if ztype: q = q.filter(type=ztype)
        if floor and floor.isdigit(): q = q.filter(floor=int(floor))
        return JsonResponse([zone_to_dict(x) for x in q], safe=False)
    data = _body(request)
    try: warehouse = Warehouse.objects.get(id=int(data.get('warehouse_id')))
    except (TypeError, ValueError, Warehouse.DoesNotExist): return _error('valid warehouse_id is required')
    obj = Zone(warehouse=warehouse)
    err = _save(obj, apply_zone_data, data, warehouse=warehouse)
    if err: return err
    _audit(user, 'CREATE_ZONE', f'{warehouse.code}/{obj.code}')
    _commit_map(warehouse, user, 'CREATE_ZONE')
    return JsonResponse(zone_to_dict(obj), status=201)


@csrf_exempt
@require_http_methods(['GET', 'PATCH', 'DELETE'])
def zone_detail(request: HttpRequest, zone_id: int):
    user, error = _user(request, admin=request.method != 'GET')
    if error: return error
    try: obj = Zone.objects.select_related('warehouse').get(id=zone_id)
    except Zone.DoesNotExist: return _error('zone not found', 404)
    if request.method == 'GET': return JsonResponse(zone_to_dict(obj))
    if request.method == 'DELETE':
        warehouse = obj.warehouse
        map_obj = ensure_warehouse_map(warehouse, runtime.layout)
        zid = obj.layout_zone_id or obj.code
        refs = []
        for key in ('docks', 'conveyors', 'stations', 'charging_stations', 'parking', 'cameras', 'sensors', 'locations'):
            for item in (map_obj.layout or {}).get(key, []):
                if str(item.get('zone') or '') == zid:
                    refs.append(f"{key}:{item.get('id', '?')}")
        if refs:
            preview = ', '.join(refs[:5])
            suffix = f" (+{len(refs)-5} more)" if len(refs) > 5 else ''
            return _error(
                f'cannot delete zone because map objects still reference it: {preview}{suffix}; remove or reassign them in Warehouse Editor first',
                409,
            )
        try: obj.delete()
        except ProtectedError: return _error('cannot delete zone because it contains shelves', 409)
        _audit(user, 'DELETE_ZONE', str(zone_id))
        _commit_map(warehouse, user, 'DELETE_ZONE')
        return JsonResponse({'ok': True})
    data = _body(request)
    warehouse = obj.warehouse
    if 'warehouse_id' in data:
        try: warehouse = Warehouse.objects.get(id=int(data['warehouse_id']))
        except (TypeError, ValueError, Warehouse.DoesNotExist): return _error('invalid warehouse_id')
    err = _save(obj, apply_zone_data, data, warehouse=warehouse)
    if err: return err
    _audit(user, 'UPDATE_ZONE', f'{warehouse.code}/{obj.code}')
    _commit_map(warehouse, user, 'UPDATE_ZONE')
    return JsonResponse(zone_to_dict(obj))


@csrf_exempt
@require_http_methods(['GET', 'POST'])
def shelves(request: HttpRequest):
    user, error = _user(request, admin=request.method == 'POST')
    if error: return error
    if request.method == 'GET':
        q = Shelf.objects.select_related('zone', 'zone__warehouse')
        warehouse_id = request.GET.get('warehouse')
        zone_id = request.GET.get('zone')
        search = request.GET.get('search', '').strip()
        status = request.GET.get('status', '').strip().upper()
        stype = request.GET.get('type', '').strip().upper()
        floor = request.GET.get('floor')
        if warehouse_id: q = q.filter(zone__warehouse_id=warehouse_id)
        if zone_id: q = q.filter(zone_id=zone_id)
        if search: q = q.filter(Q(code__icontains=search) | Q(name__icontains=search) | Q(layout_rack_id__icontains=search))
        if status: q = q.filter(status=status)
        if stype: q = q.filter(type=stype)
        if floor and floor.isdigit(): q = q.filter(floor=int(floor))
        return JsonResponse([shelf_to_dict(x) for x in q], safe=False)
    data = _body(request)
    try: zone = Zone.objects.select_related('warehouse').get(id=int(data.get('zone_id')))
    except (TypeError, ValueError, Zone.DoesNotExist): return _error('valid zone_id is required')
    obj = Shelf(zone=zone, floor=zone.floor)
    err = _save(obj, apply_shelf_data, data, zone=zone)
    if err: return err
    _audit(user, 'CREATE_SHELF', f'{zone.code}/{obj.code}')
    _commit_map(zone.warehouse, user, 'CREATE_SHELF')
    return JsonResponse(shelf_to_dict(obj), status=201)


@csrf_exempt
@require_http_methods(['GET', 'PATCH', 'DELETE'])
def shelf_detail(request: HttpRequest, shelf_id: int):
    user, error = _user(request, admin=request.method != 'GET')
    if error: return error
    try: obj = Shelf.objects.select_related('zone', 'zone__warehouse').get(id=shelf_id)
    except Shelf.DoesNotExist: return _error('shelf not found', 404)
    if request.method == 'GET': return JsonResponse(shelf_to_dict(obj))
    if request.method == 'DELETE':
        code = obj.code
        warehouse = obj.zone.warehouse
        obj.delete()
        _audit(user, 'DELETE_SHELF', code)
        _commit_map(warehouse, user, 'DELETE_SHELF')
        return JsonResponse({'ok': True})
    data = _body(request)
    zone = obj.zone
    if 'zone_id' in data:
        try: zone = Zone.objects.select_related('warehouse').get(id=int(data['zone_id']))
        except (TypeError, ValueError, Zone.DoesNotExist): return _error('invalid zone_id')
    err = _save(obj, apply_shelf_data, data, zone=zone)
    if err: return err
    _audit(user, 'UPDATE_SHELF', f'{zone.code}/{obj.code}')
    _commit_map(zone.warehouse, user, 'UPDATE_SHELF')
    return JsonResponse(shelf_to_dict(obj))


@require_http_methods(['GET'])
def warehouse_tree(request: HttpRequest):
    _, error = _user(request)
    if error: return error
    warehouses_q = Warehouse.objects.annotate(zone_count=Count('zones', distinct=True), shelf_count=Count('zones__shelves', distinct=True))
    zones_q = Zone.objects.annotate(shelf_count=Count('shelves')).order_by('floor', 'code')
    by_wh: dict[int, list[dict[str, Any]]] = {}
    for z in zones_q:
        by_wh.setdefault(z.warehouse_id, []).append(zone_to_dict(z))
    out = []
    for w in warehouses_q:
        row = warehouse_to_dict(w)
        row['zones'] = by_wh.get(w.id, [])
        out.append(row)
    return JsonResponse(out, safe=False)


@csrf_exempt
@require_http_methods(['POST'])
def warehouse_sync_from_layout(request: HttpRequest):
    user, error = _user(request, admin=True)
    if error: return error
    active = ensure_active_map(runtime.layout)
    try:
        result = sync_from_layout(active.layout, warehouse=active.warehouse, prune=False)
    except IntegrityError as exc:
        return _error(f'layout sync failed: {exc}', 409)
    _audit(user, 'SYNC_WAREHOUSE_FROM_LAYOUT', active.layout.get('id', 'layout'))
    _broadcast_map(active, 'SYNC_WAREHOUSE_FROM_LAYOUT')
    return JsonResponse({'ok': True, **result})


@require_http_methods(['GET'])
def warehouse_maps(request: HttpRequest):
    _, error = _user(request)
    if error: return error
    ensure_active_map(runtime.layout)
    for warehouse in Warehouse.objects.all():
        ensure_warehouse_map(warehouse, None)
    rows = WarehouseMap.objects.select_related('warehouse').order_by('warehouse__code')
    return JsonResponse([
        {**_map_meta(m), 'warehouse_code': m.warehouse.code, 'warehouse_name': m.warehouse.name}
        for m in rows
    ], safe=False)


@csrf_exempt
@require_http_methods(['POST'])
def warehouse_map_activate(request: HttpRequest, warehouse_id: int):
    user, error = _user(request, admin=True)
    if error: return error
    try:
        warehouse = Warehouse.objects.get(pk=warehouse_id)
    except Warehouse.DoesNotExist:
        return _error('warehouse not found', 404)
    obj = set_active_map(warehouse, runtime.layout)
    runtime.replace_layout(obj.layout)
    _audit(user, 'ACTIVATE_WAREHOUSE_MAP', warehouse.code)
    _broadcast_map(obj, 'ACTIVATE_WAREHOUSE_MAP')
    return JsonResponse({'ok': True, **_map_meta(obj)})
