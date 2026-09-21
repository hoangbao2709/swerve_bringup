from __future__ import annotations

import logging
from typing import Any
from django.db import transaction

from .models import Warehouse, Zone, Shelf, NavigationTag, NavigationTagEdge
from .canonical_map import canonicalize_layout, validate_canonical_layout
from .map_artifacts import build_revision_artifacts, cleanup_artifact_dir

log = logging.getLogger(__name__)


def warehouse_to_dict(w: Warehouse, *, counts: bool = True) -> dict[str, Any]:
    data = {
        'id': w.id, 'code': w.code, 'name': w.name, 'description': w.description,
        'status': w.status, 'width': w.width, 'depth': w.depth, 'height': w.height,
        'units': w.units, 'layout_id': w.layout_id,
        'created_at': w.created_at.isoformat() if w.created_at else None,
        'updated_at': w.updated_at.isoformat() if w.updated_at else None,
    }
    if counts:
        data['zone_count'] = getattr(w, 'zone_count', w.zones.count())
        data['shelf_count'] = getattr(w, 'shelf_count', Shelf.objects.filter(zone__warehouse=w).count())
    return data


def zone_to_dict(z: Zone, *, counts: bool = True) -> dict[str, Any]:
    data = {
        'id': z.id, 'warehouse_id': z.warehouse_id, 'code': z.code, 'name': z.name,
        'type': z.type, 'status': z.status, 'floor': z.floor, 'color': z.color,
        'polygon': z.polygon, 'description': z.description, 'layout_zone_id': z.layout_zone_id,
        'created_at': z.created_at.isoformat() if z.created_at else None,
        'updated_at': z.updated_at.isoformat() if z.updated_at else None,
    }
    if counts:
        data['shelf_count'] = getattr(z, 'shelf_count', z.shelves.count())
    return data


def shelf_to_dict(s: Shelf) -> dict[str, Any]:
    return {
        'id': s.id, 'zone_id': s.zone_id, 'warehouse_id': s.zone.warehouse_id,
        'code': s.code, 'name': s.name, 'type': s.type, 'status': s.status, 'floor': s.floor,
        'position': {'x': s.position_x, 'y': s.position_y, 'z': s.position_z},
        'size': {'width': s.width, 'depth': s.depth, 'height': s.height},
        'rotation_deg': s.rotation_deg, 'levels': s.levels,
        'capacity': s.capacity, 'current_load': s.current_load,
        'access_point': {'x': s.access_x, 'y': s.access_y, 'yaw': s.access_yaw},
        'layout_rack_id': s.layout_rack_id, 'description': s.description, 'metadata': s.metadata,
        'created_at': s.created_at.isoformat() if s.created_at else None,
        'updated_at': s.updated_at.isoformat() if s.updated_at else None,
    }


def _num(data: dict[str, Any], key: str, current: float | None = None) -> float:
    if key not in data:
        if current is None:
            raise ValueError(f'{key} is required')
        return float(current)
    try:
        return float(data[key])
    except (TypeError, ValueError):
        raise ValueError(f'{key} must be a number')


def _pos_int(data: dict[str, Any], key: str, current: int | None = None, *, allow_zero: bool = False) -> int:
    if key not in data:
        if current is None:
            raise ValueError(f'{key} is required')
        return int(current)
    try:
        value = int(data[key])
    except (TypeError, ValueError):
        raise ValueError(f'{key} must be an integer')
    if value < (0 if allow_zero else 1):
        raise ValueError(f'{key} must be >= {0 if allow_zero else 1}')
    return value


def validate_polygon(value: Any) -> list[list[float]]:
    if value in (None, ''):
        return []
    if not isinstance(value, list):
        raise ValueError('polygon must be an array of [x, y] points')
    if value and len(value) < 3:
        raise ValueError('polygon must contain at least 3 points')
    out: list[list[float]] = []
    for p in value:
        if not isinstance(p, (list, tuple)) or len(p) != 2:
            raise ValueError('polygon points must be [x, y]')
        try:
            out.append([float(p[0]), float(p[1])])
        except (TypeError, ValueError):
            raise ValueError('polygon coordinates must be numbers')
    return out


def apply_warehouse_data(obj: Warehouse, data: dict[str, Any]) -> Warehouse:
    if 'code' in data:
        obj.code = str(data['code']).strip()
    if 'name' in data:
        obj.name = str(data['name']).strip()
    if not obj.code or not obj.name:
        raise ValueError('code and name are required')
    if 'description' in data: obj.description = str(data.get('description') or '')
    if 'status' in data:
        status = str(data['status']).upper()
        if status not in dict(Warehouse.STATUS): raise ValueError('invalid warehouse status')
        obj.status = status
    for field in ('width', 'depth', 'height'):
        value = _num(data, field, getattr(obj, field))
        if value <= 0: raise ValueError(f'{field} must be > 0')
        setattr(obj, field, value)
    if 'units' in data: obj.units = str(data.get('units') or 'm').strip()[:16]
    if 'layout_id' in data: obj.layout_id = str(data.get('layout_id') or '').strip()
    return obj


def apply_zone_data(obj: Zone, data: dict[str, Any], *, warehouse: Warehouse | None = None) -> Zone:
    if warehouse is not None: obj.warehouse = warehouse
    if 'code' in data: obj.code = str(data['code']).strip()
    if 'name' in data: obj.name = str(data['name']).strip()
    if not obj.code or not obj.name: raise ValueError('code and name are required')
    if 'type' in data:
        value = str(data['type']).upper()
        if value not in dict(Zone.TYPES): raise ValueError('invalid zone type')
        obj.type = value
    if 'status' in data:
        value = str(data['status']).upper()
        if value not in dict(Zone.STATUS): raise ValueError('invalid zone status')
        obj.status = value
    obj.floor = _pos_int(data, 'floor', obj.floor)
    if 'color' in data:
        color = str(data['color']).strip()
        if not color.startswith('#') or len(color) not in (4, 7): raise ValueError('color must be a hex value')
        obj.color = color
    if 'polygon' in data: obj.polygon = validate_polygon(data['polygon'])
    if 'description' in data: obj.description = str(data.get('description') or '')
    if 'layout_zone_id' in data: obj.layout_zone_id = str(data.get('layout_zone_id') or '').strip()
    return obj


def apply_shelf_data(obj: Shelf, data: dict[str, Any], *, zone: Zone | None = None) -> Shelf:
    if zone is not None: obj.zone = zone
    if 'code' in data: obj.code = str(data['code']).strip()
    if 'name' in data: obj.name = str(data['name']).strip()
    if not obj.code or not obj.name: raise ValueError('code and name are required')
    if 'type' in data:
        value = str(data['type']).upper()
        if value not in dict(Shelf.TYPES): raise ValueError('invalid shelf type')
        obj.type = value
    if 'status' in data:
        value = str(data['status']).upper()
        if value not in dict(Shelf.STATUS): raise ValueError('invalid shelf status')
        obj.status = value
    obj.floor = _pos_int(data, 'floor', obj.floor)

    position = data.get('position') if isinstance(data.get('position'), dict) else {}
    size = data.get('size') if isinstance(data.get('size'), dict) else {}
    access = data.get('access_point') if isinstance(data.get('access_point'), dict) else {}
    aliases = {
        'position_x': position.get('x', data.get('position_x')),
        'position_y': position.get('y', data.get('position_y')),
        'position_z': position.get('z', data.get('position_z')),
        'width': size.get('width', data.get('width')),
        'depth': size.get('depth', data.get('depth')),
        'height': size.get('height', data.get('height')),
        'access_x': access.get('x', data.get('access_x')),
        'access_y': access.get('y', data.get('access_y')),
        'access_yaw': access.get('yaw', data.get('access_yaw')),
    }
    for field in ('position_x', 'position_y', 'position_z', 'rotation_deg', 'access_x', 'access_y', 'access_yaw'):
        if field in data or aliases.get(field) is not None:
            raw = aliases.get(field) if aliases.get(field) is not None else data.get(field)
            try: setattr(obj, field, float(raw))
            except (TypeError, ValueError): raise ValueError(f'{field} must be a number')
    for field in ('width', 'depth', 'height'):
        raw = aliases.get(field)
        value = float(raw) if raw is not None else float(getattr(obj, field))
        if value <= 0: raise ValueError(f'{field} must be > 0')
        setattr(obj, field, value)
    obj.levels = _pos_int(data, 'levels', obj.levels)
    # WareTwin shelf occupancy is standardized: every physical shelf has 8 order slots.
    # One order is therefore 12.5% of shelf capacity in both the 2D and 3D views.
    if 'capacity' in data:
        try:
            requested_capacity = int(data['capacity'])
        except (TypeError, ValueError):
            raise ValueError('capacity must be an integer')
        if requested_capacity != 8:
            raise ValueError('shelf capacity is fixed at 8 orders')
    obj.capacity = 8
    obj.current_load = _pos_int(data, 'current_load', obj.current_load, allow_zero=True)
    if obj.current_load > 8: raise ValueError('current_load cannot exceed 8 orders')

    wh = obj.zone.warehouse
    if obj.position_x < 0 or obj.position_y < 0 or obj.position_x > wh.width or obj.position_y > wh.depth:
        raise ValueError('shelf position must be inside warehouse bounds')
    if obj.access_x < 0 or obj.access_y < 0 or obj.access_x > wh.width or obj.access_y > wh.depth:
        raise ValueError('access point must be inside warehouse bounds')
    if 'layout_rack_id' in data: obj.layout_rack_id = str(data.get('layout_rack_id') or '').strip()
    if 'description' in data: obj.description = str(data.get('description') or '')
    if 'metadata' in data:
        if not isinstance(data['metadata'], dict): raise ValueError('metadata must be an object')
        obj.metadata = data['metadata']
    return obj



# ---------------------------------------------------------------------------
# Database-backed map synchronization
# ---------------------------------------------------------------------------
def _deepcopy_json(value: Any) -> Any:
    import copy
    return copy.deepcopy(value)


def _default_layout_for_warehouse(warehouse: Warehouse, template: dict[str, Any] | None = None) -> dict[str, Any]:
    base = _deepcopy_json(template or {})
    base.setdefault('schema_version', 2)
    base['id'] = warehouse.layout_id or warehouse.code.lower()
    base['name'] = warehouse.name
    base['units'] = warehouse.units or 'm'
    base['size'] = {'width': float(warehouse.width), 'depth': float(warehouse.depth), 'height': float(warehouse.height)}
    cell = float((base.get('grid') or {}).get('cell_size') or 1.0)
    base['grid'] = {
        'cell_size': cell,
        'cols': max(1, int(round(float(warehouse.width) / cell))),
        'rows': max(1, int(round(float(warehouse.depth) / cell))),
    }
    base.setdefault('floors', [{'id': 1, 'name': 'Floor 1', 'elevation': 0.0}])
    for key in ('columns', 'lifts', 'zones', 'docks', 'racks', 'conveyors', 'stations',
                'charging_stations', 'parking', 'restricted_areas', 'walkways',
                'cameras', 'sensors', 'locations', 'obstacles'):
        base.setdefault(key, [])
    base.setdefault('spawn', {'robots': []})
    return canonicalize_layout(base)


def master_to_layout(warehouse: Warehouse, base_layout: dict[str, Any] | None = None) -> dict[str, Any]:
    """Project Warehouse/Zone/Shelf master data into the map document.

    Non-master geometry (walls, docks, conveyors, chargers, lifts, cameras, etc.)
    is preserved from the existing map. Zones/racks/shelf locations are rebuilt
    from the relational master data so CRUD and the editor cannot drift apart.
    """
    layout = _default_layout_for_warehouse(warehouse, base_layout)
    zones = list(warehouse.zones.all().order_by('floor', 'code'))
    zone_ids: dict[int, str] = {}
    layout['zones'] = []
    for z in zones:
        zid = z.layout_zone_id or z.code
        zone_ids[z.id] = zid
        layout['zones'].append({
            'id': zid,
            'name': z.name,
            'color': z.color,
            'polygon': _deepcopy_json(z.polygon or []),
            'floor': int(z.floor),
        })

    existing_locations = list((base_layout or {}).get('locations') or [])
    existing_by_rack: dict[str, list[dict[str, Any]]] = {}
    non_shelf_locations: list[dict[str, Any]] = []
    for loc in existing_locations:
        if str(loc.get('kind') or '').upper() == 'SHELF' and loc.get('rack_id'):
            existing_by_rack.setdefault(str(loc['rack_id']), []).append(_deepcopy_json(loc))
        else:
            non_shelf_locations.append(_deepcopy_json(loc))

    racks: list[dict[str, Any]] = []
    shelf_locations: list[dict[str, Any]] = []
    for z in zones:
        zid = zone_ids[z.id]
        for s in z.shelves.all().order_by('code'):
            rid = s.layout_rack_id or s.code
            racks.append({
                'id': rid,
                'zone': zid,
                'position': [float(s.position_x), float(s.position_z), float(s.position_y)],
                'size': [float(s.width), float(s.height), float(s.depth)],
                'rotation': float(s.rotation_deg),
                'levels': int(s.levels),
                'model': str((s.metadata or {}).get('model') or 'standard'),
                'blocks_grid': bool((s.metadata or {}).get('blocks_grid', True)),
                'floor': int(s.floor),
                # Operational occupancy travels with the rack document so the live
                # 2D/3D map can render one consistent shelf-level state.
                'capacity': 8,
                'current_load': max(0, min(8, int(s.current_load))),
            })

            access_points = (s.metadata or {}).get('access_points')
            if not isinstance(access_points, list) or not access_points:
                access_points = [[float(s.access_x), float(s.access_y)]]
            else:
                clean = []
                for p in access_points:
                    if isinstance(p, (list, tuple)) and len(p) >= 2:
                        clean.append([float(p[0]), float(p[1])])
                access_points = clean or [[float(s.access_x), float(s.access_y)]]
                access_points[0] = [float(s.access_x), float(s.access_y)]

            old = existing_by_rack.get(rid) or []
            for index, point in enumerate(access_points):
                if index < len(old):
                    loc = old[index]
                    loc.update({
                        'kind': 'SHELF', 'zone': zid, 'floor': int(s.floor),
                        'rack_id': rid, 'access_point': point,
                    })
                    loc.setdefault('level_range', [1, int(s.levels)])
                else:
                    loc = {
                        'id': f'SHELF-{rid}-{index + 1}',
                        'kind': 'SHELF',
                        'zone': zid,
                        'floor': int(s.floor),
                        'rack_id': rid,
                        'level_range': [1, int(s.levels)],
                        'access_point': point,
                    }
                shelf_locations.append(loc)

    layout['racks'] = racks
    layout['locations'] = non_shelf_locations + shelf_locations
    return layout


@transaction.atomic
def sync_from_layout(layout: dict[str, Any], *, warehouse: Warehouse | None = None, prune: bool = False) -> dict[str, int]:
    """Synchronize layout geometry into relational master data.

    When ``prune=True`` (editor publish), zones/shelves removed from the layout are
    removed from master data too. Normal imports keep historical master rows.
    """
    size = layout.get('size') or {}
    layout_id = str(layout.get('id') or 'warehouse')
    if warehouse is None:
        warehouse = Warehouse.objects.filter(layout_id=layout_id).first()
        if warehouse is None:
            warehouse = Warehouse.objects.filter(code__iexact=layout_id.upper()).first()
        if warehouse is None:
            warehouse = Warehouse(code=layout_id.upper(), layout_id=layout_id)
    warehouse.layout_id = layout_id
    warehouse.name = str(layout.get('name') or warehouse.name or 'Warehouse')
    warehouse.width = float(size.get('width') or warehouse.width or 1)
    warehouse.depth = float(size.get('depth') or warehouse.depth or 1)
    warehouse.height = float(size.get('height') or warehouse.height or 1)
    warehouse.units = str(layout.get('units') or warehouse.units or 'm')
    warehouse.save()

    zone_map: dict[str, Zone] = {}
    keep_zone_ids: set[int] = set()
    for z in layout.get('zones') or []:
        zid = str(z.get('id') or '').strip()
        if not zid:
            continue
        zone = (Zone.objects.filter(warehouse=warehouse, layout_zone_id=zid).first()
                or Zone.objects.filter(warehouse=warehouse, code=zid).first()
                or Zone(warehouse=warehouse, code=zid))
        zone.name = str(z.get('name') or zone.name or zid)
        zone.floor = int(z.get('floor') or zone.floor or 1)
        zone.color = str(z.get('color') or zone.color or '#3b82f6')
        zone.polygon = z.get('polygon') or []
        zone.layout_zone_id = zid
        zone.save()
        keep_zone_ids.add(zone.id)
        zone_map[zid] = zone

    access_by_rack: dict[str, list[list[float]]] = {}
    for loc in layout.get('locations') or []:
        if str(loc.get('kind') or '').upper() != 'SHELF' or not loc.get('rack_id'):
            continue
        ap = loc.get('access_point')
        if isinstance(ap, list) and len(ap) == 2:
            access_by_rack.setdefault(str(loc['rack_id']), []).append([float(ap[0]), float(ap[1])])

    keep_shelf_ids: set[int] = set()
    shelf_count = 0
    for rack in layout.get('racks') or []:
        rid = str(rack.get('id') or '').strip()
        zone = zone_map.get(str(rack.get('zone') or ''))
        if not rid or zone is None:
            continue
        pos = rack.get('position') or [0, 0, 0]
        sz = rack.get('size') or [1, 1, 1]
        levels = max(1, int(rack.get('levels') or 1))
        access_points = access_by_rack.get(rid) or [[float(pos[0]), float(pos[2])]]
        primary_access = access_points[0]
        shelf = (Shelf.objects.filter(zone=zone, layout_rack_id=rid).first()
                 or Shelf.objects.filter(zone=zone, code=rid).first()
                 or Shelf(zone=zone, code=rid))
        is_new = shelf.pk is None
        shelf.name = shelf.name or f'Shelf {rid}'
        shelf.floor = int(rack.get('floor') or zone.floor)
        shelf.position_x = float(pos[0]); shelf.position_y = float(pos[2]); shelf.position_z = float(pos[1])
        shelf.width = float(sz[0]); shelf.height = float(sz[1]); shelf.depth = float(sz[2])
        shelf.rotation_deg = float(rack.get('rotation') or 0); shelf.levels = levels
        # Fixed shelf contract: 8 orders maximum (12.5% per order).
        shelf.capacity = 8
        if 'current_load' in rack:
            try:
                shelf.current_load = max(0, min(8, int(rack.get('current_load') or 0)))
            except (TypeError, ValueError):
                shelf.current_load = 0 if is_new else max(0, min(8, int(shelf.current_load)))
        elif is_new:
            # Starter/demo state stays deliberately light: only 0, 1, 2 or 3 orders.
            # This makes the 2D progress bars and exact 3D box counts easy to inspect.
            starter_pattern = (0, 1, 0, 2, 0, 1, 0, 3)
            shelf.current_load = starter_pattern[shelf_count % len(starter_pattern)]
        else:
            shelf.current_load = max(0, min(8, int(shelf.current_load)))
        if is_new:
            shelf.status = 'OCCUPIED' if shelf.current_load > 0 else 'AVAILABLE'
            shelf.type = 'STORAGE'
        shelf.access_x = float(primary_access[0]); shelf.access_y = float(primary_access[1])
        shelf.layout_rack_id = rid
        shelf.metadata = {
            **(shelf.metadata or {}),
            'model': rack.get('model'),
            'blocks_grid': bool(rack.get('blocks_grid', True)),
            'access_points': access_points,
        }
        shelf.save()
        keep_shelf_ids.add(shelf.id)
        shelf_count += 1

    if prune:
        # Delete shelves first because Zone -> Shelf is PROTECT.
        Shelf.objects.filter(zone__warehouse=warehouse).exclude(id__in=keep_shelf_ids).delete()
        Zone.objects.filter(warehouse=warehouse).exclude(id__in=keep_zone_ids).delete()

    # Navigation tags/edges are part of the canonical map contract.  The
    # existing relational models use tag_id as their physical identity and
    # endpoint foreign keys; UUIDs remain in the raw layout for editor
    # matching and are not regenerated here.
    keep_tag_ids: set[int] = set()
    tags_by_id: dict[int, NavigationTag] = {}
    for raw_tag in layout.get('navigation_tags') or []:
        try:
            tag_id = int(raw_tag.get('tag_id'))
            x, y = float(raw_tag.get('x')), float(raw_tag.get('y'))
            yaw = float(raw_tag.get('yaw', 0))
        except (TypeError, ValueError):
            continue
        tag, _ = NavigationTag.objects.get_or_create(
            warehouse=warehouse, tag_id=tag_id,
            defaults={
                'family': str(raw_tag.get('family') or 'DATAMATRIX').upper(),
                'size': max(0.001, float(raw_tag.get('size', 0.15) or 0.15)),
                'floor_id': str(raw_tag.get('floor_id', 1)),
                'x': x, 'y': y, 'z': float(raw_tag.get('z', 0) or 0), 'yaw': yaw,
            },
        )
        tag.family = str(raw_tag.get('family') or 'DATAMATRIX').upper()
        tag.size = max(0.001, float(raw_tag.get('size', 0.15) or 0.15))
        tag.floor_id = str(raw_tag.get('floor_id', 1))
        tag.x, tag.y = x, y
        tag.z, tag.yaw = float(raw_tag.get('z', 0) or 0), yaw
        tag.lane_id = str(raw_tag.get('lane_id') or raw_tag.get('aisle_id') or '')
        tag.zone = zone_map.get(str(raw_tag.get('zone_id') or raw_tag.get('zone') or ''))
        raw_metadata = raw_tag.get('metadata')
        tag.metadata = dict(raw_metadata) if isinstance(raw_metadata, dict) else {}
        tag.enabled = bool(raw_tag.get('enabled', True))
        tag.label = str(raw_tag.get('label') or raw_tag.get('uuid') or '')
        tag.save()
        keep_tag_ids.add(tag.pk)
        tags_by_id[tag_id] = tag

    keep_edge_ids: set[int] = set()
    edge_count = 0
    for raw_edge in layout.get('navigation_edges') or []:
        try:
            from_id = int(raw_edge.get('from_tag_id'))
            to_id = int(raw_edge.get('to_tag_id'))
        except (TypeError, ValueError):
            # Older drafts may only carry UUID endpoints.  Resolve those via
            # the canonical tag list before falling back to a safe skip.
            by_uuid = {str(tag.get('uuid')): int(tag.get('tag_id')) for tag in layout.get('navigation_tags') or [] if tag.get('uuid')}
            try:
                from_id, to_id = by_uuid[str(raw_edge.get('from_tag_uuid'))], by_uuid[str(raw_edge.get('to_tag_uuid'))]
            except KeyError:
                continue
        source, target = tags_by_id.get(from_id), tags_by_id.get(to_id)
        if source is None or target is None or source.pk == target.pk:
            continue
        defaults = {
            'cost': float(raw_edge.get('cost')) if raw_edge.get('cost') is not None else None,
            'enabled': bool(raw_edge.get('enabled', True)),
            'bidirectional': str(raw_edge.get('direction') or '') == 'bidirectional' or bool(raw_edge.get('bidirectional', False)),
        }
        edge, _ = NavigationTagEdge.objects.update_or_create(
            warehouse=warehouse, from_tag=source, to_tag=target, defaults=defaults,
        )
        keep_edge_ids.add(edge.pk)
        edge_count += 1

    if prune:
        NavigationTagEdge.objects.filter(warehouse=warehouse).exclude(pk__in=keep_edge_ids).delete()
        NavigationTag.objects.filter(warehouse=warehouse).exclude(pk__in=keep_tag_ids).delete()

    return {
        'warehouses': 1, 'zones': len(zone_map), 'shelves': shelf_count,
        'navigation_tags': len(tags_by_id), 'navigation_edges': edge_count,
        'warehouse_id': warehouse.id,
    }


def ensure_warehouse_map(warehouse: Warehouse, fallback_layout: dict[str, Any] | None = None):
    from .models import WarehouseMap
    initial = master_to_layout(warehouse, fallback_layout)
    obj, created = WarehouseMap.objects.get_or_create(
        warehouse=warehouse,
        defaults={'layout': initial, 'draft': _deepcopy_json(initial), 'revision': 1},
    )
    if created and not WarehouseMap.objects.exclude(pk=obj.pk).filter(is_active=True).exists():
        obj.is_active = True
        obj.save(update_fields=['is_active', 'updated_at'])
    return obj


def ensure_active_map(fallback_layout: dict[str, Any]):
    from .models import WarehouseMap
    active = WarehouseMap.objects.select_related('warehouse').filter(is_active=True).first()
    if active:
        return active
    warehouse = Warehouse.objects.filter(layout_id=str(fallback_layout.get('id') or '')).first() or Warehouse.objects.first()
    if warehouse is None:
        result = sync_from_layout(fallback_layout)
        warehouse = Warehouse.objects.get(pk=result['warehouse_id'])
    obj = ensure_warehouse_map(warehouse, fallback_layout)
    WarehouseMap.objects.exclude(pk=obj.pk).update(is_active=False)
    if not obj.is_active:
        obj.is_active = True
        obj.save(update_fields=['is_active', 'updated_at'])
    return obj


def set_active_map(warehouse: Warehouse, fallback_layout: dict[str, Any] | None = None):
    from .models import WarehouseMap
    obj = ensure_warehouse_map(warehouse, fallback_layout)
    WarehouseMap.objects.exclude(pk=obj.pk).update(is_active=False)
    if not obj.is_active:
        obj.is_active = True
        obj.save(update_fields=['is_active', 'updated_at'])
    return obj


def commit_master_to_map(warehouse: Warehouse, *, user=None, source: str = 'WAREHOUSE_CRUD',
                         fallback_layout: dict[str, Any] | None = None):
    """Make master data authoritative for geometry and bump map revision."""
    from .models import WarehouseMap
    obj = ensure_warehouse_map(warehouse, fallback_layout)
    next_layout = master_to_layout(warehouse, obj.layout or fallback_layout)
    obj.layout = next_layout
    obj.draft = _deepcopy_json(next_layout)
    obj.revision = int(obj.revision or 0) + 1
    obj.updated_by = user if getattr(user, 'pk', None) else None
    obj.save()
    return obj


@transaction.atomic
def save_layout_to_map(layout: dict[str, Any], *, user=None, fallback_layout: dict[str, Any] | None = None):
    """Save editor geometry as the shared current map without creating a published version."""
    layout = canonicalize_layout(layout, fallback_layout)
    active = ensure_active_map(fallback_layout or layout)
    sync_from_layout(layout, warehouse=active.warehouse, prune=True)
    active.refresh_from_db()
    active.layout = _deepcopy_json(layout)
    active.draft = _deepcopy_json(layout)
    active.revision = int(active.revision or 0) + 1
    active.updated_by = user if getattr(user, 'pk', None) else None
    active.save()
    return active


def publish_layout_to_map(layout: dict[str, Any], *, user=None, fallback_layout: dict[str, Any] | None = None):
    from .models import WarehouseMap, WarehouseMapVersion
    layout = canonicalize_layout(layout, fallback_layout)
    validation_errors = validate_canonical_layout(layout)
    if validation_errors:
        raise ValueError('; '.join(validation_errors[:20]))
    active = ensure_active_map(fallback_layout or layout)
    staging = final = None
    moved = False
    manifest = None
    try:
        with transaction.atomic():
            locked = WarehouseMap.objects.select_for_update().select_related('warehouse').get(pk=active.pk)
            next_revision = int(locked.revision or 0) + 1
            next_version = int(locked.published_version or 0) + 1
            # Build every external artifact while the map row is locked.  Any
            # exporter failure therefore leaves both the DB and artifact tree
            # untouched.
            staging, final, manifest = build_revision_artifacts(
                layout,
                warehouse_id=locked.warehouse.code,
                revision=next_revision,
                published_version=next_version,
            )
            sync_from_layout(layout, warehouse=locked.warehouse, prune=True)
            locked.layout = _deepcopy_json(layout)
            locked.draft = _deepcopy_json(layout)
            locked.revision = next_revision
            locked.published_version = next_version
            locked.updated_by = user if getattr(user, 'pk', None) else None
            locked.save()
            WarehouseMapVersion.objects.create(
                warehouse_map=locked,
                version=next_version,
                revision=next_revision,
                layout=_deepcopy_json(layout),
                created_by=locked.updated_by,
            )
            # The destination is immutable and must not already exist.
            final.parent.mkdir(parents=True, exist_ok=True)
            if final.exists():
                raise ValueError(f'artifact revision already exists: {final}')
            import os
            os.replace(staging, final)
            staging = None
            moved = True
        active = locked
    except Exception as exc:
        log.exception('Warehouse map publish transaction failed: %s', type(exc).__name__)
        cleanup_artifact_dir(staging)
        if moved:
            cleanup_artifact_dir(final)
        raise
    active._artifact_manifest = manifest
    active._artifact_dir = str(final)
    return active
