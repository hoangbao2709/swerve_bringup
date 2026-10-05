"""Canonical-to-active map registration providers.

Navigation target resolution consumes a versioned SE(2) registration without
knowing where it came from.  This module contains the Gazebo-only provider;
physical robots must use a calibrated/localization provider instead.
"""
from __future__ import annotations

import math
from datetime import datetime, timezone
from typing import Any

from django.db import transaction
from django.db.models import Max

from .models import RobotMapRegistration, WarehouseMap

GAZEBO_REGISTRATION_SOURCE = 'GAZEBO_CANONICAL_ALIGNMENT'
# SLAM/canonical pose correspondence is exact in the current Gazebo contract,
# but tolerate small estimator jitter. A larger discrepancy must persist over
# three telemetry samples before replacing the registration.
REGISTRATION_TRANSLATION_TOLERANCE_M = 0.15
REGISTRATION_YAW_TOLERANCE_RAD = math.radians(5.0)
REGISTRATION_DRIFT_SAMPLES_REQUIRED = 3
REGISTRATION_POSE_PAIR_MAX_SKEW_S = 0.25


def normalize_yaw(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))


def derive_active_from_canonical(canonical_pose: dict[str, Any],
                                 active_pose: dict[str, Any]) -> dict[str, float]:
    """Derive T_active_from_canonical from paired poses of the same robot."""
    cx, cy, cyaw = (float(canonical_pose[key]) for key in ('x', 'y', 'yaw'))
    ax, ay, ayaw = (float(active_pose[key]) for key in ('x', 'y', 'yaw'))
    if not all(math.isfinite(value) for value in (cx, cy, cyaw, ax, ay, ayaw)):
        raise ValueError('registration poses must contain finite x, y and yaw')
    yaw = normalize_yaw(ayaw - cyaw)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return {
        'tx': ax - cosine * cx + sine * cy,
        'ty': ay - sine * cx - cosine * cy,
        'yaw': yaw,
    }


def transform_canonical_pose(pose: dict[str, Any], transform: dict[str, Any]) -> dict[str, float]:
    """Apply T_active_from_canonical to an SE(2) pose."""
    x, y, yaw = (float(pose[key]) for key in ('x', 'y', 'yaw'))
    tx, ty, angle = (float(transform[key]) for key in ('tx', 'ty', 'yaw'))
    values = (x, y, yaw, tx, ty, angle)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('pose and registration must be finite')
    cosine, sine = math.cos(angle), math.sin(angle)
    return {
        'x': tx + cosine * x - sine * y,
        'y': ty + sine * x + cosine * y,
        'yaw': normalize_yaw(yaw + angle),
    }


def _valid_pose(pose: Any) -> bool:
    if not isinstance(pose, dict) or pose.get('valid') is not True:
        return False
    try:
        return all(math.isfinite(float(pose[key])) for key in ('x', 'y', 'yaw'))
    except (KeyError, TypeError, ValueError):
        return False


def _pose_pair_is_time_aligned(canonical_pose: dict[str, Any],
                               active_pose: dict[str, Any]) -> bool:
    """Require fresh wall timestamps so both coordinates describe one pose."""
    try:
        stamps = []
        for pose in (canonical_pose, active_pose):
            raw = str(pose.get('timestamp') or '')
            stamp = datetime.fromisoformat(raw.replace('Z', '+00:00'))
            if stamp.tzinfo is None:
                stamp = stamp.replace(tzinfo=timezone.utc)
            stamps.append(stamp)
        skew = abs((stamps[0] - stamps[1]).total_seconds())
        age = abs((datetime.now(timezone.utc) - max(stamps)).total_seconds())
    except (TypeError, ValueError, OverflowError):
        return False
    return skew <= REGISTRATION_POSE_PAIR_MAX_SKEW_S and age <= REGISTRATION_POSE_PAIR_MAX_SKEW_S


def update_gazebo_registration(*, robot_id: str, active_map: dict[str, Any],
                               canonical_pose: dict[str, Any], active_pose: dict[str, Any],
                               drift_state: dict[tuple[str, str, str, int], int]) -> dict[str, Any] | None:
    """Create/update a stable simulation registration for one active SLAM map.

    Returns the active registration plus ``changed``. Invalid or stale pose
    correspondence is rejected without mutating registration state.
    """
    robot_id = str(robot_id or '').strip()
    active_map_id = str(active_map.get('active_map_id') or '')
    active_revision = str(active_map.get('active_map_revision') or '')
    if (not robot_id or not active_map_id.startswith('SLAM-') or not active_revision
            or active_map.get('map_source') != 'SLAM_TOOLBOX'
            or active_map.get('map_sync_status') != 'LOCAL_ONLY'
            or not _valid_pose(canonical_pose) or not _valid_pose(active_pose)
            or not _pose_pair_is_time_aligned(canonical_pose, active_pose)):
        return None
    if (canonical_pose.get('map_id') != 'CANONICAL'
            or canonical_pose.get('frame_id') != 'map'
            or canonical_pose.get('pose_source') != 'GAZEBO_MODEL_STATES'
            or canonical_pose.get('transform_source') != 'VALIDATED_CANONICAL_WORLD_BUNDLE'
            or str(canonical_pose.get('map_revision') or '')
            != str(active_map.get('canonical_map_revision') or '')
            or active_pose.get('map_id') != active_map_id
            or str(active_pose.get('map_revision') or '') != active_revision
            or active_pose.get('frame_id') != 'map'
            or active_pose.get('pose_source') != 'TF'):
        return None
    try:
        canonical_revision = int(str(active_map.get('canonical_map_revision') or ''))
    except (TypeError, ValueError):
        return None
    warehouse_map = WarehouseMap.objects.filter(is_active=True, revision=canonical_revision).order_by('id').first()
    if warehouse_map is None:
        return None
    candidate = derive_active_from_canonical(canonical_pose, active_pose)
    key = (robot_id, active_map_id, active_revision, canonical_revision)
    with transaction.atomic():
        prior = RobotMapRegistration.objects.select_for_update().filter(
            robot_id=robot_id, active_map_id=active_map_id,
            active_map_revision=active_revision,
        )
        current = prior.filter(is_active=True).order_by('-registration_revision').first()
        source_matches = bool(current and current.source == GAZEBO_REGISTRATION_SOURCE
                              and current.canonical_revision == canonical_revision)
        materially_changed = bool(current and (
            math.hypot(candidate['tx'] - current.tx, candidate['ty'] - current.ty)
            > REGISTRATION_TRANSLATION_TOLERANCE_M
            or abs(normalize_yaw(candidate['yaw'] - current.yaw))
            > REGISTRATION_YAW_TOLERANCE_RAD
        ))
        if source_matches and not materially_changed:
            drift_state.pop(key, None)
            return {
                'registration': current,
                'changed': False,
                'created': False,
            }
        if source_matches and materially_changed:
            drift_state[key] = drift_state.get(key, 0) + 1
            if drift_state[key] < REGISTRATION_DRIFT_SAMPLES_REQUIRED:
                return {'registration': current, 'changed': False, 'created': False}
        drift_state.pop(key, None)
        next_revision = (prior.aggregate(value=Max('registration_revision'))['value'] or 0) + 1
        prior.filter(is_active=True).update(is_active=False)
        registration = RobotMapRegistration.objects.create(
            robot_id=robot_id, warehouse_map=warehouse_map,
            canonical_revision=canonical_revision,
            active_map_id=active_map_id, active_map_revision=active_revision,
            tx=candidate['tx'], ty=candidate['ty'], yaw=candidate['yaw'],
            registration_revision=next_revision, source=GAZEBO_REGISTRATION_SOURCE,
            created_by=None,
        )
        return {'registration': registration, 'changed': True,
                'created': current is None or not source_matches}
