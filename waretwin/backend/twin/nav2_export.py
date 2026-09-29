"""Rasterize canonical warehouse geometry into a Nav2 trinary PGM map."""
from __future__ import annotations

import math
import re
from pathlib import Path
from typing import Any

from .canonical_map import canonicalize_layout, point_in_polygon, points


def _distance_to_segment(x: float, y: float, a: tuple[float, float], b: tuple[float, float]) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-18:
        return math.hypot(x - a[0], y - a[1])
    t = max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / length_sq))
    return math.hypot(x - (a[0] + t * dx), y - (a[1] + t * dy))


def _inside_rotated_rect(x: float, y: float, cx: float, cy: float,
                         width: float, height: float, yaw: float) -> bool:
    dx, dy = x - cx, y - cy
    c, s = math.cos(yaw), math.sin(yaw)
    local_x, local_y = c * dx + s * dy, -s * dx + c * dy
    return abs(local_x) <= width / 2 and abs(local_y) <= height / 2


def _floor_obstacle(x: float, y: float, floor: dict[str, Any], doc: dict[str, Any],
                    default_floor: str) -> bool:
    boundary = points(floor['boundary'])
    holes = [points(hole) for hole in floor.get('holes') or []]
    if not point_in_polygon((x, y), boundary):
        return True
    if any(point_in_polygon((x, y), hole) for hole in holes):
        return True

    half_wall = float((doc.get('gazebo') or {}).get('wall_thickness', 0.2)) / 2
    if any(_distance_to_segment(x, y, a, b) <= half_wall
           for ring in [boundary, *holes]
           for a, b in zip(ring, ring[1:] + ring[:1])):
        return True

    fid = str(floor.get('id'))
    for rack in doc.get('racks') or []:
        if str(rack.get('floor_id', rack.get('floor', default_floor))) != fid:
            continue
        position, size = rack.get('position') or [0, 0, 0], rack.get('size') or [1, 1, 1]
        if len(position) < 3 or len(size) < 3:
            continue
        cx, cy = float(position[0]) + float(size[0]) / 2, float(position[2]) + float(size[2]) / 2
        if _inside_rotated_rect(x, y, cx, cy, float(size[0]), float(size[2]),
                                math.radians(float(rack.get('rotation', 0)))):
            return True

    for key in ('stations', 'obstacles'):
        for item in doc.get(key) or []:
            if str(item.get('floor_id', item.get('floor', default_floor))) != fid:
                continue
            rect = item.get('rect') or [0, 0, 1, 1]
            x0, y0, x1, y1 = map(float, rect)
            if min(x0, x1) <= x <= max(x0, x1) and min(y0, y1) <= y <= max(y0, y1):
                return True

    if fid == default_floor:
        for column in doc.get('columns') or []:
            cx, cy = map(float, column)
            if math.hypot(x - cx, y - cy) <= 0.45:
                return True

    for conveyor in doc.get('conveyors') or []:
        if str(conveyor.get('floor_id', conveyor.get('floor', default_floor))) != fid:
            continue
        width = float(conveyor.get('width', 1)) / 2
        path = conveyor.get('path') or []
        if any(_distance_to_segment(x, y, tuple(map(float, a)), tuple(map(float, b))) <= width
               for a, b in zip(path, path[1:])):
            return True
    return False


def _scanline_fill(pixels: bytearray, width: int, height: int, resolution: float,
                   min_x: float, max_y: float, polygon: list[tuple[float, float]],
                   value: int) -> None:
    """Fill pixel centers inside a world-space polygon, in north-up PGM rows."""
    if len(polygon) < 3:
        return
    min_py = min(point[1] for point in polygon)
    max_py = max(point[1] for point in polygon)
    first_row = max(0, math.ceil((max_y - max_py) / resolution - 0.5 - 1e-12))
    stop_row = min(height, math.floor((max_y - min_py) / resolution - 0.5 + 1e-12) + 1)
    edges = list(zip(polygon, polygon[1:] + polygon[:1]))
    row_value = bytes((value,))
    for row in range(first_row, stop_row):
        y = max_y - (row + 0.5) * resolution
        crossings = []
        for (x1, y1), (x2, y2) in edges:
            if (y1 > y) != (y2 > y):
                crossings.append(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
        crossings.sort()
        for left, right in zip(crossings[0::2], crossings[1::2]):
            first_col = max(0, math.ceil((left - min_x) / resolution - 0.5 - 1e-12))
            stop_col = min(width, math.floor((right - min_x) / resolution - 0.5 + 1e-12) + 1)
            if stop_col > first_col:
                start = row * width + first_col
                pixels[start:start + stop_col - first_col] = row_value * (stop_col - first_col)


def _oriented_rectangle(cx: float, cy: float, length: float, breadth: float,
                        yaw: float) -> list[tuple[float, float]]:
    c, s = math.cos(yaw), math.sin(yaw)
    return [
        (cx + c * x - s * y, cy + s * x + c * y)
        for x, y in ((-length / 2, -breadth / 2), (length / 2, -breadth / 2),
                     (length / 2, breadth / 2), (-length / 2, breadth / 2))
    ]


def _draw_disk(pixels: bytearray, width: int, height: int, resolution: float,
               min_x: float, max_y: float, cx: float, cy: float,
               radius: float, value: int) -> None:
    first_row = max(0, math.ceil((max_y - (cy + radius)) / resolution - 0.5 - 1e-12))
    stop_row = min(height, math.floor((max_y - (cy - radius)) / resolution - 0.5 + 1e-12) + 1)
    for row in range(first_row, stop_row):
        y = max_y - (row + 0.5) * resolution
        dy = y - cy
        if abs(dy) > radius:
            continue
        dx = math.sqrt(max(0.0, radius * radius - dy * dy))
        left, right = cx - dx, cx + dx
        first_col = max(0, math.ceil((left - min_x) / resolution - 0.5 - 1e-12))
        stop_col = min(width, math.floor((right - min_x) / resolution - 0.5 + 1e-12) + 1)
        if stop_col > first_col:
            start = row * width + first_col
            pixels[start:start + stop_col - first_col] = bytes((value,)) * (stop_col - first_col)


def render_nav2_map(layout: dict[str, Any], floor: dict[str, Any], *, resolution: float = 0.05,
                    max_cells: int = 16_000_000) -> tuple[bytes, str, dict[str, Any]]:
    """Return P5 image bytes, YAML metadata and measured raster bounds."""
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('Nav2 map resolution must be finite and positive')
    doc = canonicalize_layout(layout)
    boundary = points(floor.get('boundary'))
    min_x, max_x = min(p[0] for p in boundary), max(p[0] for p in boundary)
    min_y, max_y = min(p[1] for p in boundary), max(p[1] for p in boundary)
    width = max(1, math.ceil((max_x - min_x) / resolution))
    height = max(1, math.ceil((max_y - min_y) / resolution))
    if width * height > max_cells:
        raise ValueError(f'Nav2 map raster is too large: {width}x{height} cells')
    default_floor = str((doc.get('floors') or [floor])[0].get('id'))
    floor_id = str(floor.get('id'))
    pixels = bytearray(width * height)
    # Start occupied and fill usable floor cells free. Raster work is bounded
    # to each polygon's scanlines instead of testing every cell against every
    # warehouse object (which scales as cells x racks).
    _scanline_fill(pixels, width, height, resolution, min_x, max_y, boundary, 254)
    holes = [points(hole) for hole in floor.get('holes') or []]
    for hole in holes:
        _scanline_fill(pixels, width, height, resolution, min_x, max_y, hole, 0)

    wall_thickness = float((doc.get('gazebo') or {}).get('wall_thickness', 0.2))
    if not math.isfinite(wall_thickness) or wall_thickness <= 0:
        raise ValueError('Gazebo wall_thickness must be finite and positive')
    for ring in [boundary, *holes]:
        for a, b in zip(ring, ring[1:] + ring[:1]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = math.hypot(dx, dy)
            if length <= 1e-12:
                continue
            # Match the rectangular wall box generated in the Gazebo world.
            yaw = math.atan2(dy, dx)
            wall = _oriented_rectangle((a[0] + b[0]) / 2, (a[1] + b[1]) / 2,
                                       length, wall_thickness, yaw)
            _scanline_fill(pixels, width, height, resolution, min_x, max_y, wall, 0)

    for rack in doc.get('racks') or []:
        if str(rack.get('floor_id', rack.get('floor', default_floor))) != floor_id:
            continue
        position = rack.get('position') or [0, 0, 0]
        size = rack.get('size') or [1, 1, 1]
        if len(position) < 3 or len(size) < 3:
            raise ValueError(f'rack {rack.get("id", "?")} requires 3D position and size')
        values = [float(value) for value in (*position[:3], *size[:3])]
        if not all(math.isfinite(value) for value in values) or any(value <= 0 for value in values[3:]):
            raise ValueError(f'rack {rack.get("id", "?")} has invalid position or dimensions')
        x, _z, y = values[:3]
        sx, _sz, sy = values[3:]
        yaw = math.radians(float(rack.get('rotation', 0)))
        if not math.isfinite(yaw):
            raise ValueError(f'rack {rack.get("id", "?")} has invalid rotation')
        polygon = _oriented_rectangle(x + sx / 2, y + sy / 2, sx, sy, yaw)
        _scanline_fill(pixels, width, height, resolution, min_x, max_y, polygon, 0)

    for key in ('stations', 'obstacles'):
        for item in doc.get(key) or []:
            if str(item.get('floor_id', item.get('floor', default_floor))) != floor_id:
                continue
            rect = item.get('rect') or [0, 0, 1, 1]
            if len(rect) != 4:
                raise ValueError(f'{key} {item.get("id", "?")} requires a 4-value rectangle')
            x0, y0, x1, y1 = map(float, rect)
            if not all(math.isfinite(value) for value in (x0, y0, x1, y1)) or x1 <= x0 or y1 <= y0:
                raise ValueError(f'{key} {item.get("id", "?")} has invalid rectangle geometry')
            _scanline_fill(pixels, width, height, resolution, min_x, max_y,
                           [(x0, y0), (x1, y0), (x1, y1), (x0, y1)], 0)

    if floor_id == default_floor:
        for index, column in enumerate(doc.get('columns') or []):
            if len(column) < 2:
                raise ValueError(f'column {index} requires x/y coordinates')
            cx, cy = map(float, column[:2])
            if not math.isfinite(cx) or not math.isfinite(cy):
                raise ValueError(f'column {index} has invalid coordinates')
            _draw_disk(pixels, width, height, resolution, min_x, max_y,
                       cx, cy, 0.45, 0)

    for conveyor in doc.get('conveyors') or []:
        if str(conveyor.get('floor_id', conveyor.get('floor', default_floor))) != floor_id:
            continue
        path = conveyor.get('path') or []
        width_m = float(conveyor.get('width', 1))
        if not math.isfinite(width_m) or width_m <= 0:
            raise ValueError(f'conveyor {conveyor.get("id", "?")} width must be positive')
        for start, end in zip(path, path[1:]):
            ax, ay = map(float, start[:2])
            bx, by = map(float, end[:2])
            if not all(math.isfinite(value) for value in (ax, ay, bx, by)):
                raise ValueError(f'conveyor {conveyor.get("id", "?")} has invalid path coordinates')
            dx, dy = bx - ax, by - ay
            length = math.hypot(dx, dy)
            if length <= 1e-12:
                continue
            polygon = _oriented_rectangle((ax + bx) / 2, (ay + by) / 2,
                                          length, width_m, math.atan2(dy, dx))
            _scanline_fill(pixels, width, height, resolution, min_x, max_y, polygon, 0)
    image = f'P5\n{width} {height}\n255\n'.encode('ascii') + bytes(pixels)
    metadata = {
        'image': '',
        'mode': 'trinary',
        'resolution': resolution,
        'origin': [min_x, min_y, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }
    yaml_text = '\n'.join([
        'image: {image}', 'mode: trinary', f'resolution: {resolution:.12g}',
        f'origin: [{min_x:.12g}, {min_y:.12g}, 0.0]',
        'negate: 0', 'occupied_thresh: 0.65', 'free_thresh: 0.196',
        'frame_id: map', 'units: m',
    ]) + '\n'
    return image, yaml_text, {
        'floor_id': floor.get('id'), 'width': width, 'height': height,
        'resolution': resolution, 'origin': metadata['origin'],
        'bounds': {'min_x': min_x, 'min_y': min_y,
                   'max_x': min_x + width * resolution,
                   'max_y': min_y + height * resolution},
    }


def floor_artifact_name(floor_id: Any) -> str:
    return re.sub(r'[^A-Za-z0-9_.-]+', '_', str(floor_id)).strip('._-') or 'floor'
