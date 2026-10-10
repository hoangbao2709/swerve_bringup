"""Safely persist a paused SLAM OccupancyGrid as Nav2 trinary map files.

The conversion deliberately matches Nav2 Humble's map_saver thresholds and
row ordering. Files are staged beside their destinations and installed with
no-clobber hard links, so an existing operator artifact is never replaced.
"""
from __future__ import annotations

import math
import os
from pathlib import Path
import uuid


MAX_MAP_CELLS = 20_000_000
FREE_THRESHOLD = 0.25
OCCUPIED_THRESHOLD = 0.65
YAML_FREE_THRESHOLD = 0.196


def _finite(value, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"map {label} is not numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"map {label} must be finite")
    return result


def _grid_values(grid):
    frame_id = str(getattr(getattr(grid, "header", None), "frame_id", "") or "")
    if frame_id != "map":
        raise ValueError("SLAM map frame must be 'map'")
    info = getattr(grid, "info", None)
    if info is None:
        raise ValueError("SLAM map metadata is missing")
    try:
        width, height = int(info.width), int(info.height)
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError("SLAM map dimensions are invalid") from exc
    if width <= 0 or height <= 0 or width * height > MAX_MAP_CELLS:
        raise ValueError("SLAM map dimensions are outside the supported range")
    resolution = _finite(getattr(info, "resolution", None), "resolution")
    if resolution <= 0:
        raise ValueError("SLAM map resolution must be positive")
    origin = getattr(info, "origin", None)
    position = getattr(origin, "position", None)
    orientation = getattr(origin, "orientation", None)
    x = _finite(getattr(position, "x", None), "origin x")
    y = _finite(getattr(position, "y", None), "origin y")
    qx = _finite(getattr(orientation, "x", None), "origin quaternion x")
    qy = _finite(getattr(orientation, "y", None), "origin quaternion y")
    qz = _finite(getattr(orientation, "z", None), "origin quaternion z")
    qw = _finite(getattr(orientation, "w", None), "origin quaternion w")
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        raise ValueError("SLAM map origin quaternion has zero norm")
    # Normalize before extracting yaw, matching the geometry represented by a
    # valid ROS quaternion even if a producer's values are slightly rounded.
    qx, qy, qz, qw = (component / norm for component in (qx, qy, qz, qw))
    yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                     1.0 - 2.0 * (qy * qy + qz * qz))
    try:
        cells = tuple(grid.data)
    except (AttributeError, TypeError) as exc:
        raise ValueError("SLAM map occupancy data is missing") from exc
    if len(cells) != width * height:
        raise ValueError("SLAM map dimensions do not match occupancy data")
    normalized = []
    for cell in cells:
        if isinstance(cell, bool):
            raise ValueError("SLAM map occupancy data contains a non-integer cell")
        try:
            numeric = int(cell)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("SLAM map occupancy data contains a non-integer cell") from exc
        if numeric != cell or numeric < -1 or numeric > 100:
            raise ValueError("SLAM map occupancy cells must be -1 or in [0, 100]")
        normalized.append(numeric)
    known_cells = sum(cell >= 0 for cell in normalized)
    if known_cells == 0:
        raise ValueError("SLAM map contains no known occupancy cells")
    return width, height, resolution, x, y, yaw, normalized, known_cells


def _format_yaml_number(value: float) -> str:
    return format(value, ".12g")


def _stage_file(directory: Path, content: bytes, suffix: str) -> Path:
    path = directory / f".waretwin-map-{uuid.uuid4().hex}{suffix}.tmp"
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    descriptor = os.open(path, flags, 0o600)
    try:
        view = memoryview(content)
        while view:
            written = os.write(descriptor, view)
            view = view[written:]
        os.fsync(descriptor)
    except BaseException:
        os.close(descriptor)
        path.unlink(missing_ok=True)
        raise
    else:
        os.close(descriptor)
    return path


def write_trinary_map(grid, output_prefix: str | Path) -> dict:
    """Write ``.pgm``/``.yaml`` for a ROS OccupancyGrid without overwriting.

    Returns geometry and occupancy statistics only after both output files are
    durably linked into place. Any files created by a failed two-file commit
    are removed; pre-existing targets are never removed.
    """
    prefix = Path(output_prefix).expanduser().resolve()
    if not prefix.name or prefix.name in (".", "..") or not prefix.parent.is_dir():
        raise ValueError("map output directory or prefix is invalid")
    width, height, resolution, origin_x, origin_y, yaw, cells, known = _grid_values(grid)
    pgm_path = prefix.with_suffix(".pgm")
    yaml_path = prefix.with_suffix(".yaml")
    if pgm_path.exists() or yaml_path.exists():
        raise FileExistsError("map output already exists; refusing to overwrite operator data")

    free_int = round(FREE_THRESHOLD * 100)
    occupied_int = round(OCCUPIED_THRESHOLD * 100)
    raster = bytearray()
    for image_y in range(height):
        source_row = height - image_y - 1
        for x in range(width):
            cell = cells[source_row * width + x]
            if cell < 0 or cell > 100:
                shade = 205  # unknown/ambiguous, as Nav2's trinary map saver
            elif cell <= free_int:
                shade = 254
            elif occupied_int <= cell:
                shade = 0
            else:
                shade = 205
            raster.append(shade)
    pgm_content = (f"P5\n# WareTwin SLAM OccupancyGrid snapshot\n{width} {height}\n255\n"
                   .encode("ascii") + bytes(raster))
    yaml_content = (
        f"image: {pgm_path.name}\n"
        "mode: trinary\n"
        f"resolution: {_format_yaml_number(resolution)}\n"
        f"origin: [{_format_yaml_number(origin_x)}, {_format_yaml_number(origin_y)}, "
        f"{_format_yaml_number(yaw)}]\n"
        "negate: 0\n"
        f"occupied_thresh: {_format_yaml_number(OCCUPIED_THRESHOLD)}\n"
        # Nav2's trinary saver encodes unknown/ambiguous cells as gray 205.
        # A 0.25 free threshold decodes that value as free (1 - 205/255 < .25),
        # silently changing unexplored space during Load Map. 0.196 keeps the
        # standard 205 gray value in the unknown interval while preserving the
        # normal .25 occupancy threshold used to choose free PGM pixels.
        f"free_thresh: {_format_yaml_number(YAML_FREE_THRESHOLD)}\n"
    ).encode("utf-8")

    staged = []
    installed = []
    try:
        staged.append(_stage_file(prefix.parent, pgm_content, ".pgm"))
        staged.append(_stage_file(prefix.parent, yaml_content, ".yaml"))
        for temporary, destination in zip(staged, (pgm_path, yaml_path)):
            # A hard-link is an atomic no-clobber install on the same filesystem.
            os.link(temporary, destination)
            installed.append(destination)
        directory_fd = os.open(prefix.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except BaseException:
        for destination in installed:
            destination.unlink(missing_ok=True)
        raise
    finally:
        for temporary in staged:
            temporary.unlink(missing_ok=True)

    return {
        "yaml_path": str(yaml_path), "image_path": str(pgm_path),
        "width": width, "height": height, "resolution": resolution,
        "origin": [origin_x, origin_y, yaw], "known_cells": known,
        "occupied_cells": sum(cell >= occupied_int for cell in cells),
        "free_cells": sum(0 <= cell <= free_int for cell in cells),
        "unknown_cells": len(cells) - known,
    }
