"""Canonical Warehouse Map V2 geometry and coordinate convention.

The legacy layout document is intentionally still accepted.  Normalization is
non-destructive: it adds V2 metadata and derives a rectangular floor boundary
only when a legacy floor has no boundary.
"""
from __future__ import annotations

import copy
import math
from typing import Any, Iterable


def warehouse_to_screen(x: float, y: float, origin=(0.0, 0.0), scale=1.0):
    return ((float(x) - origin[0]) * scale, (float(y) - origin[1]) * scale)


def screen_to_warehouse(x: float, y: float, origin=(0.0, 0.0), scale=1.0):
    if not math.isfinite(float(scale)) or float(scale) == 0: raise ValueError('scale must be finite and non-zero')
    return (float(x) / scale + origin[0], float(y) / scale + origin[1])


def warehouse_to_three(x: float, y: float, z: float = 0.0):
    return (float(x), float(z), float(y))


def three_to_warehouse(x: float, y: float, z: float):
    return (float(x), float(z), float(y))


def warehouse_to_ros(x: float, y: float, z: float = 0.0):
    return (float(x), float(y), float(z))


def ros_to_warehouse(x: float, y: float, z: float):
    return (float(x), float(y), float(z))


def _point(raw: Any):
    if isinstance(raw, dict): return (float(raw['x']), float(raw['y']))
    if isinstance(raw, (list, tuple)) and len(raw) >= 2: return (float(raw[0]), float(raw[1]))
    raise ValueError('point must contain x/y')


def points(raw: Any) -> list[tuple[float, float]]:
    if not isinstance(raw, list): raise ValueError('polygon must be an array')
    result = []
    for item in raw:
        x, y = _point(item)
        if not math.isfinite(x) or not math.isfinite(y): raise ValueError('coordinates must be finite')
        result.append((x, y))
    return result


def signed_area(poly: Iterable[tuple[float, float]]) -> float:
    p = list(poly)
    return sum(p[i][0] * p[(i + 1) % len(p)][1] - p[(i + 1) % len(p)][0] * p[i][1] for i in range(len(p))) / 2


def orientation(a, b, c):
    return (b[0]-a[0]) * (c[1]-a[1]) - (b[1]-a[1]) * (c[0]-a[0])


def on_segment(a, b, p, eps=1e-9):
    return abs(orientation(a, b, p)) <= eps and min(a[0], b[0])-eps <= p[0] <= max(a[0], b[0])+eps and min(a[1], b[1])-eps <= p[1] <= max(a[1], b[1])+eps


def segments_intersect(a, b, c, d, eps=1e-9):
    o1, o2, o3, o4 = orientation(a, b, c), orientation(a, b, d), orientation(c, d, a), orientation(c, d, b)
    if ((o1 > eps and o2 < -eps) or (o1 < -eps and o2 > eps)) and ((o3 > eps and o4 < -eps) or (o3 < -eps and o4 > eps)): return True
    return (abs(o1) <= eps and on_segment(a, b, c, eps)) or (abs(o2) <= eps and on_segment(a, b, d, eps)) or (abs(o3) <= eps and on_segment(c, d, a, eps)) or (abs(o4) <= eps and on_segment(c, d, b, eps))


def self_intersects(poly: list[tuple[float, float]]) -> bool:
    if len(poly) < 3: return False
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        for j in range(i + 1, len(poly)):
            if j in (i - 1, i + 1) or (i == 0 and j == len(poly)-1): continue
            if segments_intersect(a, b, poly[j], poly[(j + 1) % len(poly)]): return True
    return False


def point_in_polygon(point, poly, include_boundary=True):
    if include_boundary and any(on_segment(poly[i], poly[(i+1) % len(poly)], point) for i in range(len(poly))): return True
    x, y = point; inside = False
    for i in range(len(poly)):
        a, b = poly[i], poly[(i+1) % len(poly)]
        if (a[1] > y) != (b[1] > y) and x < (b[0]-a[0]) * (y-a[1]) / ((b[1]-a[1]) or 1e-30) + a[0]: inside = not inside
    return inside


def validate_polygon(raw: Any, label='polygon') -> list[str]:
    errors = []
    try: poly = points(raw)
    except (TypeError, ValueError) as exc: return [f'{label}: {exc}']
    if len(poly) < 3: errors.append(f'{label}: requires at least 3 vertices')
    if len(set(poly)) != len(poly): errors.append(f'{label}: duplicate point')
    if len(poly) >= 3 and abs(signed_area(poly)) <= 1e-9: errors.append(f'{label}: area must be non-zero')
    if len(poly) >= 3 and self_intersects(poly): errors.append(f'{label}: self-intersects')
    return errors


def validate_floor_polygon(raw: Any, label='floor') -> list[str]:
    return validate_polygon(raw, label)


def validate_hole_polygon(raw: Any, label='hole') -> list[str]:
    return validate_polygon(raw, label)


def validate_aisle_centerline(raw: Any, width: Any, label='aisle') -> list[str]:
    errors: list[str] = []
    try:
        line = points(raw)
    except (TypeError, ValueError) as exc:
        return [f'{label}: {exc}']
    try:
        numeric_width = float(width)
    except (TypeError, ValueError):
        numeric_width = float('nan')
    if len(line) < 2: errors.append(f'{label}: centerline requires at least 2 points')
    if not math.isfinite(numeric_width) or numeric_width <= 0: errors.append(f'{label}: width must be positive and finite')
    for a, b in zip(line, line[1:]):
        if math.hypot(b[0] - a[0], b[1] - a[1]) <= 1e-9: errors.append(f'{label}: consecutive points must not duplicate')
    return errors


def _line_intersection(a, ad, b, bd):
    cross = ad[0] * bd[1] - ad[1] * bd[0]
    if abs(cross) <= 1e-10: return None
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = (dx * bd[1] - dy * bd[0]) / cross
    return (a[0] + ad[0] * t, a[1] + ad[1] * t)


def aisle_footprint(raw: Any, width: float) -> list[tuple[float, float]]:
    line = points(raw)
    if len(validate_aisle_centerline(line, width)):
        return []
    half = float(width) / 2
    directions = []
    for a, b in zip(line, line[1:]):
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        directions.append(((b[0] - a[0]) / length, (b[1] - a[1]) / length))

    def side(sign):
        out = []
        for i, point in enumerate(line):
            prev = directions[max(0, i - 1)]; nxt = directions[min(len(directions) - 1, i)]
            def normal(direction): return (-direction[1] * half * sign, direction[0] * half * sign)
            if i == 0: offset = normal(nxt); out.append((point[0] + offset[0], point[1] + offset[1])); continue
            if i == len(line) - 1: offset = normal(prev); out.append((point[0] + offset[0], point[1] + offset[1])); continue
            pn, nn = normal(prev), normal(nxt)
            intersection = _line_intersection((line[i - 1][0] + pn[0], line[i - 1][1] + pn[1]), prev, (point[0] + nn[0], point[1] + nn[1]), nxt)
            if intersection is None or math.hypot(intersection[0] - point[0], intersection[1] - point[1]) > half * 4:
                out.append((point[0] + (pn[0] + nn[0]) / 2, point[1] + (pn[1] + nn[1]) / 2))
            else: out.append(intersection)
        return out
    return side(1) + list(reversed(side(-1)))


def polygon_contained_in_usable_floor(polygon, boundary, holes=()) -> bool:
    if len(polygon) < 3 or len(boundary) < 3: return False
    # Strict containment prevents an aisle half-width from sitting on a wall.
    if not all(point_in_polygon(point, boundary, include_boundary=False) and not any(point_in_polygon(point, hole, include_boundary=True) for hole in holes) for point in polygon): return False
    for i, a in enumerate(polygon):
        b = polygon[(i + 1) % len(polygon)]
        if any(segments_intersect(a, b, boundary[j], boundary[(j + 1) % len(boundary)]) for j in range(len(boundary))): return False
        if any(segments_intersect(a, b, hole[j], hole[(j + 1) % len(hole)]) for hole in holes for j in range(len(hole))): return False
    return True


def canonicalize_layout(value: dict[str, Any], fallback: dict[str, Any] | None = None) -> dict[str, Any]:
    source = copy.deepcopy(value if isinstance(value, dict) else (fallback or {}))
    size = source.get('size') or {}
    width, depth = float(size.get('width') or 1), float(size.get('depth') or 1)
    source['schema_version'] = 2
    source['coordinate_system'] = {'unit': 'meter', 'frame': 'warehouse_map', 'yaw_unit': 'radian'}
    floors = source.get('floors') if isinstance(source.get('floors'), list) else []
    if not floors: floors = [{'id': 1, 'name': 'Floor 1', 'elevation': 0.0}]
    normalized_floors = []
    for floor in floors:
        item = dict(floor)
        item.setdefault('name', f"Floor {item.get('id', 1)}")
        item.setdefault('elevation', 0.0)
        boundary = item.get('boundary') or item.get('footprint')
        if not boundary: boundary = [[0, 0], [width, 0], [width, depth], [0, depth]]
        item['boundary'] = boundary
        item.setdefault('holes', [])
        normalized_floors.append(item)
    source['floors'] = normalized_floors
    for key in ('aisles', 'navigation_tags', 'navigation_edges', 'stations', 'holes'):
        if not isinstance(source.get(key), list): source[key] = []
    return source


def validate_canonical_layout(value: dict[str, Any]) -> list[str]:
    doc = canonicalize_layout(value)
    errors: list[str] = []
    floors = doc.get('floors', [])
    floor_ids = [str(f.get('id')) for f in floors]
    if len(set(floor_ids)) != len(floor_ids): errors.append('floors: duplicate id')
    floor_polys: dict[str, list[tuple[float, float]]] = {}
    for floor in floors:
        fid = str(floor.get('id'))
        errors += validate_floor_polygon(floor.get('boundary'), f'floor {fid} boundary')
        try: boundary = points(floor.get('boundary'))
        except (TypeError, ValueError): boundary = []
        floor_polys[fid] = boundary
        holes = floor.get('holes') or []
        if not isinstance(holes, list): errors.append(f'floor {fid}: holes must be an array'); continue
        for index, hole in enumerate(holes):
            errors += validate_hole_polygon(hole, f'floor {fid} hole {index + 1}')
            try: hp = points(hole)
            except (TypeError, ValueError): continue
            if boundary and not all(point_in_polygon(p, boundary, include_boundary=False) for p in hp): errors.append(f'floor {fid} hole {index + 1}: outside floor')
            if boundary and any(segments_intersect(hp[i], hp[(i+1) % len(hp)], boundary[j], boundary[(j+1) % len(boundary)]) for i in range(len(hp)) for j in range(len(boundary))): errors.append(f'floor {fid} hole {index + 1}: intersects boundary')
            for other in holes[:index]:
                try: op = points(other)
                except (TypeError, ValueError): continue
                if any(segments_intersect(hp[i], hp[(i+1) % len(hp)], op[j], op[(j+1) % len(op)]) for i in range(len(hp)) for j in range(len(op))) or point_in_polygon(hp[0], op, False): errors.append(f'floor {fid}: holes intersect'); break
    ids: set[str] = set()
    for key in ('aisles', 'navigation_tags', 'navigation_edges', 'racks', 'zones', 'conveyors', 'stations'):
        for obj in doc.get(key, []):
            oid = str(obj.get('uuid') or obj.get('id') or '')
            if oid and oid in ids: errors.append(f'duplicate object id: {oid}')
            if oid: ids.add(oid)
            floor_ref = obj.get('floor_id', obj.get('floor'))
            if floor_ref is not None and str(floor_ref) not in floor_ids: errors.append(f'{key} {oid}: invalid floor reference')
    tag_ids = [str(t.get('tag_id')) for t in doc.get('navigation_tags', []) if t.get('tag_id') is not None]
    if len(tag_ids) != len(set(tag_ids)): errors.append('navigation_tags: duplicate tag_id')
    for aisle in doc.get('aisles', []):
        aid = aisle.get('uuid') or aisle.get('id') or '?'
        centerline = aisle.get('centerline')
        width = aisle.get('width')
        aisle_errors = validate_aisle_centerline(centerline, width, f'aisle {aid}')
        errors += aisle_errors
        try: centerline_points = points(centerline); numeric_width = float(width)
        except (TypeError, ValueError, KeyError): continue
        floor_ref = aisle.get('floor_id', aisle.get('floor'))
        if floor_ref is not None and str(floor_ref) in floor_polys and centerline_points and numeric_width > 0:
            boundary = floor_polys[str(floor_ref)]
            floor = next((item for item in floors if str(item.get('id')) == str(floor_ref)), {})
            hole_points = []
            for hole in floor.get('holes') or []:
                try: hole_points.append(points(hole))
                except (TypeError, ValueError): pass
            footprint = aisle_footprint(centerline_points, numeric_width)
            if boundary and not polygon_contained_in_usable_floor(footprint, boundary, hole_points): errors.append(f'aisle {aid}: width footprint is outside usable floor or intersects a hole')
    return errors
