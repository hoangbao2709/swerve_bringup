"""Single ROS <-> WareTwin coordinate adapter.

ROS uses X/Y on the warehouse floor and Z up. WareTwin's renderer uses
Three.js X/Y/Z where Y is up, so ROS (x, y, z) becomes (x, z, y).
Yaw is kept positive and the renderer already applies its historical -heading
rotation around Three.js Y.
"""

from __future__ import annotations

import math
from typing import Any


def ros_pose_to_waretwin(x: float, y: float, z: float, yaw: float) -> dict[str, Any]:
    return {
        'position': [float(x), float(z), float(y)],
        'heading': ((float(yaw) + math.pi) % (2.0 * math.pi)) - math.pi,
    }


def ros_twist_to_waretwin(vx: float, vy: float, wz: float) -> dict[str, float]:
    return {'vx': float(vx), 'vy': float(vy), 'wz': float(wz),
            'velocity': math.hypot(float(vx), float(vy))}


def validated_canonical_pose(value: Any, revision: Any, runtime_mode: str) -> dict | None:
    """Accept a separately measured pose only for the displayed canonical revision."""
    if not isinstance(value, dict) or value.get('valid') is not True:
        return None
    if (value.get('frame_id') != 'map' or value.get('map_id') != 'CANONICAL'
            or revision is None or str(value.get('map_revision')) != str(revision)
            or value.get('map_source') != 'CANONICAL'):
        return None
    if (runtime_mode != 'GAZEBO_ROS' or value.get('pose_source') != 'GAZEBO_MODEL_STATES'
            or value.get('source_frame_id') != 'world'
            or value.get('transform_source') != 'VALIDATED_CANONICAL_WORLD_BUNDLE'):
        return None
    try:
        if not all(math.isfinite(float(value[key])) for key in ('x', 'y', 'yaw')):
            return None
    except (KeyError, ValueError, TypeError):
        return None
    return {key: value[key] for key in ('x', 'y', 'yaw', 'frame_id', 'map_id', 'map_revision',
            'map_source', 'pose_source', 'source_frame_id', 'transform_source', 'timestamp', 'valid') if key in value}
