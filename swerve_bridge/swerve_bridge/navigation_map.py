"""Geometry helpers for a full canonical navigation raster in the live map frame.

This module deliberately has no ROS dependencies so the exact same coverage and
resampling rules can be tested without bringing up Gazebo or Nav2.
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np


MAX_NAVIGATION_GRID_CELLS = 16_000_000


def _finite(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f'{name} must be finite')
    return result


def transform_occupancy_grid(*, width: int, height: int, resolution: float,
                              origin_x: float, origin_y: float, origin_yaw: float,
                              data, transform: dict[str, Any],
                              max_cells: int = MAX_NAVIGATION_GRID_CELLS) -> dict[str, Any]:
    """Resample a complete canonical grid into ``T_active_from_canonical``.

    The output is an axis-aligned grid in the active map frame. Unknown source
    cells remain unknown; no cells are added to the live SLAM grid. Inverse
    nearest-cell sampling avoids holes in a rotated raster while retaining the
    source's occupied/free/unknown classification.
    """
    width, height = int(width), int(height)
    resolution = _finite(resolution, 'resolution')
    origin_x = _finite(origin_x, 'origin_x')
    origin_y = _finite(origin_y, 'origin_y')
    origin_yaw = _finite(origin_yaw, 'origin_yaw')
    tx = _finite(transform['tx'], 'registration.tx')
    ty = _finite(transform['ty'], 'registration.ty')
    registration_yaw = _finite(transform['yaw'], 'registration.yaw')
    if width <= 0 or height <= 0 or resolution <= 0:
        raise ValueError('source grid dimensions and resolution must be positive')
    if width * height > max_cells:
        raise ValueError(f'source navigation grid is too large: {width}x{height}')
    source = np.asarray(data, dtype=np.int16)
    if source.size != width * height:
        raise ValueError(f'source data size mismatch: {source.size} != {width * height}')
    if np.any((source < -1) | (source > 100)):
        raise ValueError('source occupancy cells must be in [-1, 100]')
    source = source.reshape((height, width))

    cr, sr = math.cos(registration_yaw), math.sin(registration_yaw)
    co, so = math.cos(origin_yaw), math.sin(origin_yaw)

    def to_active(local_x: float, local_y: float) -> tuple[float, float]:
        canonical_x = origin_x + co * local_x - so * local_y
        canonical_y = origin_y + so * local_x + co * local_y
        return (tx + cr * canonical_x - sr * canonical_y,
                ty + sr * canonical_x + cr * canonical_y)

    corners = [to_active(x, y) for x, y in (
        (0.0, 0.0), (width * resolution, 0.0),
        (0.0, height * resolution), (width * resolution, height * resolution),
    )]
    min_x = min(point[0] for point in corners)
    max_x = max(point[0] for point in corners)
    min_y = min(point[1] for point in corners)
    max_y = max(point[1] for point in corners)
    output_origin_x = math.floor(min_x / resolution + 1e-10) * resolution
    output_origin_y = math.floor(min_y / resolution + 1e-10) * resolution
    output_width = max(1, math.ceil((max_x - output_origin_x) / resolution - 1e-10))
    output_height = max(1, math.ceil((max_y - output_origin_y) / resolution - 1e-10))
    if output_width * output_height > max_cells:
        raise ValueError(f'transformed navigation grid is too large: {output_width}x{output_height}')

    rows, columns = np.indices((output_height, output_width), dtype=np.float64)
    active_x = output_origin_x + (columns + 0.5) * resolution
    active_y = output_origin_y + (rows + 0.5) * resolution
    dx, dy = active_x - tx, active_y - ty
    canonical_x = cr * dx + sr * dy
    canonical_y = -sr * dx + cr * dy
    canonical_dx, canonical_dy = canonical_x - origin_x, canonical_y - origin_y
    local_x = co * canonical_dx + so * canonical_dy
    local_y = -so * canonical_dx + co * canonical_dy
    source_columns = np.floor(local_x / resolution).astype(np.int64)
    source_rows = np.floor(local_y / resolution).astype(np.int64)
    inside = ((source_columns >= 0) & (source_columns < width)
              & (source_rows >= 0) & (source_rows < height))
    transformed = np.full((output_height, output_width), -1, dtype=np.int16)
    transformed[inside] = source[source_rows[inside], source_columns[inside]]
    cells = transformed.ravel()
    known = cells >= 0
    return {
        'resolution': resolution,
        'width': output_width,
        'height': output_height,
        'origin_x': output_origin_x,
        'origin_y': output_origin_y,
        'origin_yaw': 0.0,
        'min_x': output_origin_x,
        'max_x': output_origin_x + output_width * resolution,
        'min_y': output_origin_y,
        'max_y': output_origin_y + output_height * resolution,
        'known_cells': int(np.count_nonzero(known)),
        'free_cells': int(np.count_nonzero((cells >= 0) & (cells < 50))),
        'occupied_cells': int(np.count_nonzero(cells >= 50)),
        'unknown_cells': int(np.count_nonzero(~known)),
        'data': cells.tolist(),
    }


def validate_target_coverage(grid: dict[str, Any] | None, *, x: float, y: float,
                             occupied_threshold: int = 50) -> dict[str, Any] | None:
    """Return authoritative rejection details, or ``None`` for traversable cells."""
    x, y = _finite(x, 'goal_x'), _finite(y, 'goal_y')
    if not isinstance(grid, dict) or not grid.get('ready'):
        return {'code': 'NAVIGATION_MAP_NOT_READY', 'goal_x': x, 'goal_y': y}
    try:
        width, height = int(grid['width']), int(grid['height'])
        resolution = _finite(grid['resolution'], 'navigation_map.resolution')
        origin_x = _finite(grid['origin_x'], 'navigation_map.origin_x')
        origin_y = _finite(grid['origin_y'], 'navigation_map.origin_y')
        origin_yaw = _finite(grid.get('origin_yaw', 0.0), 'navigation_map.origin_yaw')
        cells = grid['data']
        if width <= 0 or height <= 0 or resolution <= 0 or len(cells) != width * height:
            raise ValueError('navigation map geometry/data is incomplete')
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        return {'code': 'NAVIGATION_MAP_INVALID', 'goal_x': x, 'goal_y': y,
                'detail': str(exc)}

    dx, dy = x - origin_x, y - origin_y
    local_x = math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy
    local_y = -math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy
    column, row = math.floor(local_x / resolution), math.floor(local_y / resolution)
    bounds = {
        'goal_x': x, 'goal_y': y,
        'map_min_x': float(grid.get('min_x', origin_x)),
        'map_max_x': float(grid.get('max_x', origin_x + width * resolution)),
        'map_min_y': float(grid.get('min_y', origin_y)),
        'map_max_y': float(grid.get('max_y', origin_y + height * resolution)),
        'navigation_map_id': grid.get('navigation_map_id'),
        'navigation_map_revision': grid.get('navigation_map_revision'),
    }
    if not (0 <= column < width and 0 <= row < height):
        return {'code': 'TARGET_OUTSIDE_NAVIGATION_MAP', **bounds}
    value = int(cells[row * width + column])
    details = {**bounds, 'cell_x': column, 'cell_y': row, 'cell_value': value}
    if value < 0:
        return {'code': 'TARGET_IN_UNKNOWN_SPACE', **details}
    if value >= int(occupied_threshold):
        return {'code': 'TARGET_OCCUPIED', **details}
    return None
