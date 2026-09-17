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
