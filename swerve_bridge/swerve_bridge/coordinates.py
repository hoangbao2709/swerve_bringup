"""Small ROS-independent helpers for ROS frame/pose conversion."""
from __future__ import annotations

import math
from typing import Any


def quaternion_yaw(quaternion: Any) -> float:
    """Return CCW yaw from a quaternion-like object or (x, y, z, w) tuple."""
    if hasattr(quaternion, 'x'):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    else:
        x, y, z, w = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rotate_translate_xy(x: float, y: float, translation: Any, quaternion: Any) -> tuple[float, float]:
    """Transform a planar point by a 3D translation/quaternion."""
    if hasattr(quaternion, 'x'):
        qx, qy, qz, qw = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    else:
        qx, qy, qz, qw = quaternion
    tx = 2.0 * (qy * 0.0 - qz * y)
    ty = 2.0 * (qz * x - qx * 0.0)
    tz = 2.0 * (qx * y - qy * x)
    rx = x + qw * tx + qy * tz - qz * ty
    ry = y + qw * ty + qz * tx - qx * tz
    return float(translation[0]) + rx, float(translation[1]) + ry


def pose_from_transform(transform: Any) -> dict[str, float]:
    """Convert a TransformStamped map->base pose to x/y/z/yaw in metres/radians."""
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return {
        'x': float(translation.x),
        'y': float(translation.y),
        'z': float(translation.z),
        'yaw': quaternion_yaw(rotation),
    }


def is_small_future_tf_skew(requested_stamp: Any, available_stamp: Any,
                            tolerance_s: float = 0.1) -> bool:
    """Allow a latest-TF fallback only for a narrowly newer requested time.

    Nav2 can stamp a path just ahead of the most recently published map->odom
    transform. This helper deliberately rejects old requests and large gaps;
    it is not a general substitute for time-correct TF lookup.
    """
    def nanoseconds(value: Any) -> int:
        if hasattr(value, 'nanoseconds'):
            return int(value.nanoseconds)
        return int(getattr(value, 'sec', 0)) * 1_000_000_000 + int(
            getattr(value, 'nanosec', 0))

    requested_ns = nanoseconds(requested_stamp)
    available_ns = nanoseconds(available_stamp)
    allowed_ns = max(0, int(float(tolerance_s) * 1_000_000_000))
    skew_ns = requested_ns - available_ns
    return 0 < skew_ns <= allowed_ns


def is_fresh_tf_sample(now_s: float, stamp_s: float, max_age_s: float,
                       future_tolerance_s: float = 0.1) -> bool:
    """Return whether a dynamic TF sample belongs to the current clock epoch.

    A negative age larger than the small publication skew allowance means the
    transform is from the future. In simulation this commonly happens when a
    long-lived node's TF buffer survives a Gazebo ``/clock`` rewind. Treating
    such a transform as fresh can falsely confirm localization or an initial
    pose from the previous simulation run.
    """
    try:
        now = float(now_s)
        stamp = float(stamp_s)
        max_age = float(max_age_s)
        future_tolerance = float(future_tolerance_s)
    except (TypeError, ValueError, OverflowError):
        return False
    if (not all(math.isfinite(value) for value in (now, stamp, max_age, future_tolerance))
            or stamp <= 0.0 or max_age < 0.0 or future_tolerance < 0.0):
        return False
    age = now - stamp
    return -future_tolerance <= age <= max_age
