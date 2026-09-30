"""Bounded ROS-to-Web map preparation helpers.

The ROS bridge owns TF and sensor-message parsing. This module provides pure,
testable filtering, voxel sampling, range conversion, and path metrics; the
browser only receives bounded numeric arrays and frame metadata.
"""
from __future__ import annotations

import base64
import itertools
import math
import threading
import time
import zlib
from typing import Iterable, Sequence


def compress_occupancy_grid(values: Iterable[int], max_cells: int = 4_000_000) -> str:
    """Return bounded zlib/base64 OccupancyGrid bytes with -1 encoded as 0."""
    raw = bytearray()
    for value in values:
        cell = int(value)
        if cell < -1 or cell > 100:
            raise ValueError('occupancy cells must be in [-1, 100]')
        raw.append(cell + 1)
        if len(raw) > max_cells:
            raise ValueError(f'occupancy grid exceeds {max_cells} cells')
    return base64.b64encode(zlib.compress(raw, level=6)).decode('ascii')


class LatestFrameBuffer:
    """A one-slot handoff for visualization frames.

    A producer never waits for a slow WebSocket sender. Offering a newer frame
    replaces the unsent one and increments a measurable drop counter.
    """

    def __init__(self):
        self._condition = threading.Condition()
        self._pending = None
        self._dropped_frames = 0

    def offer(self, frame) -> None:
        with self._condition:
            if self._pending is not None:
                self._dropped_frames += 1
            self._pending = frame
            self._condition.notify()

    def take(self, timeout: float | None = None):
        deadline = None if timeout is None else time.monotonic() + max(0.0, timeout)
        with self._condition:
            while self._pending is None:
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0.0:
                    return None
                self._condition.wait(remaining)
            frame, self._pending = self._pending, None
            return frame

    def clear(self) -> None:
        with self._condition:
            self._pending = None

    @property
    def dropped_frames(self) -> int:
        with self._condition:
            return self._dropped_frames

    @property
    def pending(self) -> bool:
        with self._condition:
            return self._pending is not None


def quaternion_rotate_xyz(point: Sequence[float], quaternion: Sequence[float]) -> tuple[float, float, float]:
    x, y, z = (float(value) for value in point)
    qx, qy, qz, qw = (float(value) for value in quaternion)
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def transform_points_xyz(
    points: Iterable[Sequence[float]],
    translation: Sequence[float],
    quaternion: Sequence[float],
) -> list[tuple[float, float, float]]:
    tx, ty, tz = (float(value) for value in translation)
    transformed = []
    for point in points:
        x, y, z = quaternion_rotate_xyz(point, quaternion)
        transformed.append((x + tx, y + ty, z + tz))
    return transformed


def point_xyz(point: object) -> tuple[float, float, float]:
    """Read XYZ from sensor_msgs_py tuples or structured NumPy rows."""
    names = getattr(getattr(point, 'dtype', None), 'names', None)
    if names and {'x', 'y', 'z'}.issubset(names):
        return float(point['x']), float(point['y']), float(point['z'])
    return float(point[0]), float(point[1]), float(point[2])


def successful_path_result(action_status: int, succeeded_status: int,
                           pose_count: int) -> bool:
    """A planner path is usable only when Nav2 succeeded and returned poses."""
    return action_status == succeeded_status and pose_count > 0


def bounded_voxel_points(
    points: Iterable[Sequence[float]],
    *,
    max_points: int = 4000,
    min_range: float = 0.15,
    max_range: float = 25.0,
    min_height: float = -1.0,
    max_height: float = 3.0,
    voxel_size: float = 0.04,
    max_input_points: int = 200_000,
) -> list[tuple[float, float, float]]:
    """Filter and voxel downsample without retaining an unbounded cloud."""
    limit = max(1, int(max_points))
    voxel = max(0.005, float(voxel_size))
    selected: dict[tuple[int, int, int], tuple[float, float, float]] = {}
    for raw in itertools.islice(points, max(1, int(max_input_points))):
        try:
            x, y, z = (float(value) for value in raw[:3])
        except (TypeError, ValueError, IndexError):
            continue
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue
        distance = math.sqrt(x * x + y * y + z * z)
        if distance < min_range or distance > max_range or z < min_height or z > max_height:
            continue
        key = (math.floor(x / voxel), math.floor(y / voxel), math.floor(z / voxel))
        selected.setdefault(key, (x, y, z))
    values = list(selected.values())
    if len(values) <= limit:
        return values
    stride = math.ceil(len(values) / limit)
    return values[::stride][:limit]


def laser_scan_xy(ranges: Iterable[float], angle_min: float, angle_increment: float,
                  range_min: float, range_max: float, max_points: int = 720) -> list[tuple[float, float]]:
    valid: list[tuple[float, float]] = []
    for index, raw in enumerate(ranges):
        distance = float(raw)
        if not math.isfinite(distance) or distance < range_min or distance > range_max:
            continue
        angle = angle_min + index * angle_increment
        valid.append((distance * math.cos(angle), distance * math.sin(angle)))
    limit = max(1, int(max_points))
    stride = max(1, math.ceil(len(valid) / limit))
    return valid[::stride][:limit]


def path_length(points: Iterable[Sequence[float]]) -> float:
    previous = None
    total = 0.0
    for raw in points:
        try:
            current = float(raw[0]), float(raw[1])
        except (TypeError, ValueError, IndexError):
            continue
        if not all(math.isfinite(value) for value in current):
            continue
        if previous is not None:
            total += math.hypot(current[0] - previous[0], current[1] - previous[1])
        previous = current
    return total
