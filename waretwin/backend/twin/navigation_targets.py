from __future__ import annotations

import hashlib
import json
import math
from dataclasses import asdict, dataclass, field
from typing import Any

from .models import NavigationTag, RobotMapRegistration, WarehouseMap
from .map_registration import transform_canonical_pose


class NavigationTargetError(ValueError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class NavigationTarget:
    robot_id: str
    source_type: str
    source_id: str | int | None
    frame_id: str
    map_id: str
    map_revision: str
    x: float
    y: float
    yaw: float
    metadata: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def _digest(value: Any) -> str:
    encoded = json.dumps(_json_safe(value), sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def _json_safe(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _finite_or_none(value: Any) -> float | None:
    try:
        converted = float(value)
    except (TypeError, ValueError):
        return None
    return converted if math.isfinite(converted) else None


def _canonical_map(active_map: dict[str, Any]) -> tuple[WarehouseMap | None, str | None]:
    if (str(active_map.get('active_map_id') or '') != 'CANONICAL'
            or str(active_map.get('map_sync_status') or '') != 'CANONICAL'):
        return None, 'the robot is not using the synchronized canonical warehouse map'
    try:
        revision = int(str(active_map.get('active_map_revision') or ''))
    except (TypeError, ValueError):
        return None, 'the robot canonical map revision is missing or invalid'
    warehouse_map = (WarehouseMap.objects.select_related('warehouse')
                     .filter(is_active=True).order_by('id').first())
    if warehouse_map is None:
        return None, 'no active warehouse map is registered'
    if int(warehouse_map.revision) != revision:
        return None, 'the robot canonical map revision is stale relative to the active warehouse map'
    return warehouse_map, None


def _tag_navigation_pose(tag: NavigationTag, map_revision: str) -> tuple[dict[str, float] | None, str | None, str | None]:
    metadata = tag.metadata if isinstance(tag.metadata, dict) else {}
    pose_key = next((key for key in ('approach_pose', 'navigation_pose', 'goal_pose') if key in metadata), None)
    if pose_key is None:
        raw_pose: dict[str, Any] = {'x': tag.x, 'y': tag.y, 'yaw': tag.yaw}
        pose_source = 'REGISTERED_NAVIGATION_NODE'
    else:
        raw_pose = metadata.get(pose_key)
        pose_source = f'METADATA:{pose_key}'
        if not isinstance(raw_pose, dict):
            return None, pose_source, f'{pose_key} must contain a pose object'
        pose_frame = str(raw_pose.get('frame_id') or 'map')
        if pose_frame != 'map':
            return None, pose_source, f'{pose_key} must be expressed in map frame'
        pose_revision = raw_pose.get('map_revision')
        if pose_revision is not None and str(pose_revision) != map_revision:
            return None, pose_source, f'{pose_key} belongs to a different map revision'
    try:
        pose = {axis: float(raw_pose.get(axis, 0.0) if axis == 'yaw' else raw_pose[axis])
                for axis in ('x', 'y', 'yaw')}
    except (KeyError, TypeError, ValueError):
        return None, pose_source, 'registered navigation pose is incomplete or invalid'
    if not all(math.isfinite(value) for value in pose.values()):
        return None, pose_source, 'registered navigation pose contains non-finite coordinates'
    return pose, pose_source, None


def _tag_record(tag: NavigationTag, map_id: str, map_revision: str, map_error: str | None = None) -> dict[str, Any]:
    metadata = tag.metadata if isinstance(tag.metadata, dict) else {}
    pose, pose_source, pose_error = _tag_navigation_pose(tag, map_revision)
    enabled_by_metadata = metadata.get('navigation_allowed') is not False and metadata.get('navigable') is not False
    enabled = bool(tag.enabled)
    reason = map_error
    if reason is None and not enabled:
        reason = 'tag is disabled in the authoritative registry'
    if reason is None and not enabled_by_metadata:
        reason = 'tag metadata disallows navigation'
    if reason is None and pose_error:
        reason = pose_error
    serializable_metadata = {
        'id': tag.pk,
        'tag_id': tag.tag_id,
        'label': tag.label,
        'family': tag.family,
        'floor_id': tag.floor_id,
        'lane_id': tag.lane_id,
        'zone_id': tag.zone_id,
        'x': _finite_or_none(tag.x),
        'y': _finite_or_none(tag.y),
        'z': _finite_or_none(tag.z),
        'yaw': _finite_or_none(tag.yaw),
        'enabled': enabled,
        'registry_metadata': _json_safe(metadata),
        'updated_at': tag.updated_at.isoformat() if tag.updated_at else None,
        'navigation_pose': pose,
        'navigation_pose_source': pose_source,
        'map_id': map_id,
        'map_revision': map_revision,
    }
    tag_revision = _digest(serializable_metadata)
    return {
        'id': tag.pk,
        'tag_id': tag.tag_id,
        'label': tag.label or f'Tag {tag.tag_id}',
        'family': tag.family,
        'floor_id': tag.floor_id,
        'lane_id': tag.lane_id,
        'zone_id': tag.zone_id,
        'x': _finite_or_none(tag.x),
        'y': _finite_or_none(tag.y),
        'z': _finite_or_none(tag.z),
        'yaw': _finite_or_none(tag.yaw),
        'enabled': enabled,
        'navigable': reason is None,
        'reason': reason,
        'frame_id': 'map',
        'map_id': map_id,
        'map_revision': map_revision,
        'navigation_pose': pose,
        'navigation_pose_source': pose_source,
        'tag_revision': tag_revision,
        'metadata': _json_safe(metadata),
    }


def _active_registration(robot_id: str, active_map: dict[str, Any],
                         canonical_revision: int,
                         warehouse_map: WarehouseMap) -> RobotMapRegistration | None:
    registration = RobotMapRegistration.objects.filter(
        robot_id=str(robot_id), canonical_revision=canonical_revision,
        warehouse_map=warehouse_map,
        active_map_id=str(active_map.get('active_map_id') or ''),
        active_map_revision=str(active_map.get('active_map_revision') or ''),
        is_active=True,
    ).order_by('-registration_revision').first()
    if registration is not None and not all(math.isfinite(value) for value in (
            registration.tx, registration.ty, registration.yaw)):
        return None
    return registration


def navigation_tag_registry(active_map: dict[str, Any], robot_id: str | None = None) -> dict[str, Any]:
    """Return canonical Tags, mapped only by an exact versioned registration.

    A SLAM/local ``map`` frame is not implicitly the canonical warehouse
    frame. For local maps, Tags remain fail-closed until an administrator has
    registered the exact robot, canonical revision, active map identity and
    active map revision with a versioned SE(2) transform.
    """
    map_id = str(active_map.get('active_map_id') or '')
    map_revision = str(active_map.get('active_map_revision') or '')
    local_only = map_id != 'CANONICAL' and str(active_map.get('map_sync_status') or '') == 'LOCAL_ONLY'
    if local_only:
        try:
            canonical_revision = int(str(active_map.get('canonical_map_revision') or ''))
        except (TypeError, ValueError):
            canonical_revision = None
        warehouse_map = (WarehouseMap.objects.select_related('warehouse')
                         .filter(is_active=True).order_by('id').first())
        map_error = None
        if warehouse_map is None:
            map_error = 'no active canonical warehouse map is registered'
        elif canonical_revision is None or int(warehouse_map.revision) != canonical_revision:
            warehouse_map = None
            map_error = 'the local map canonical revision does not match the active warehouse registry'
    else:
        warehouse_map, map_error = _canonical_map(active_map)
    if warehouse_map is None:
        return {
            'source': 'WAREHOUSE_NAVIGATION_TAG_REGISTRY',
            'warehouse_id': None,
            'map_id': map_id or None,
            'map_revision': map_revision or None,
            'active_map_id': map_id or None,
            'active_map_revision': map_revision or None,
            'frame_id': 'map',
            'compatible': False,
            'reason': map_error,
            'registry_revision': None,
            'tags': [],
        }

    source_revision = str(warehouse_map.revision)
    tags = [_tag_record(tag, 'CANONICAL', source_revision)
            for tag in NavigationTag.objects.filter(warehouse=warehouse_map.warehouse).order_by('tag_id')]
    registration = None
    if local_only and robot_id:
        registration = _active_registration(robot_id, active_map, int(source_revision), warehouse_map)

    if local_only and registration is not None:
        for tag in tags:
            canonical_pose = tag.get('navigation_pose')
            tag['canonical_map_id'] = 'CANONICAL'
            tag['canonical_map_revision'] = source_revision
            tag['canonical_navigation_pose'] = canonical_pose
            tag['registration_revision'] = registration.registration_revision
            tag['registration_source'] = registration.source
            tag['map_id'] = map_id
            tag['map_revision'] = map_revision
            if canonical_pose is not None:
                active_pose = transform_canonical_pose(canonical_pose, {
                    'tx': registration.tx, 'ty': registration.ty, 'yaw': registration.yaw,
                })
                if all(math.isfinite(value) for value in active_pose.values()):
                    tag['navigation_pose'] = active_pose
                else:
                    tag['navigation_pose'] = None
                    tag['navigable'] = False
                    tag['reason'] = 'registered Tag pose is non-finite'
            tag['tag_revision'] = _digest({
                'canonical_tag_revision': tag['tag_revision'],
                'canonical_map_revision': source_revision,
                'active_map_id': map_id,
                'active_map_revision': map_revision,
                'registration_revision': registration.registration_revision,
                'registration_source': registration.source,
                'tx': registration.tx, 'ty': registration.ty, 'yaw': registration.yaw,
                'navigation_pose': tag.get('navigation_pose'),
            })
    elif local_only:
        for tag in tags:
            tag['navigable'] = False
            tag['reason'] = 'TAG_MAP_REGISTRATION_REQUIRED'
    registry_revision = _digest({
        'warehouse_id': warehouse_map.warehouse_id,
        'map_id': 'CANONICAL',
        'map_revision': source_revision,
        'active_map_id': map_id,
        'active_map_revision': map_revision,
        'navigation_policy': ('REGISTERED_LOCAL' if local_only and registration
                              else 'REGISTRATION_REQUIRED' if local_only else 'CANONICAL'),
        'registration_revision': registration.registration_revision if registration else None,
        'registration_source': registration.source if registration else None,
        'registration_transform': ({'tx': registration.tx, 'ty': registration.ty,
                                    'yaw': registration.yaw} if registration else None),
        'tags': [tag['tag_revision'] for tag in tags],
    })
    registered_local = bool(local_only and registration)
    return {
        'source': 'WAREHOUSE_NAVIGATION_TAG_REGISTRY',
        'warehouse_id': warehouse_map.warehouse_id,
        'warehouse_code': warehouse_map.warehouse.code,
        'map_id': map_id if registered_local else 'CANONICAL',
        'map_revision': map_revision if registered_local else source_revision,
        'canonical_map_id': 'CANONICAL',
        'canonical_map_revision': source_revision,
        'active_map_id': map_id,
        'active_map_revision': map_revision,
        'frame_id': 'map',
        'compatible': not local_only or registered_local,
        'reason': 'TAG_MAP_REGISTRATION_REQUIRED' if local_only and not registered_local else None,
        'registration_required': local_only and not registered_local,
        'transform_source': registration.source if registered_local else None,
        'registration_revision': registration.registration_revision if registered_local else None,
        'registration': ({
            'canonical_map_id': 'CANONICAL', 'canonical_map_revision': source_revision,
            'active_map_id': map_id, 'active_map_revision': map_revision,
            'tx': registration.tx, 'ty': registration.ty, 'yaw': registration.yaw,
            'registration_revision': registration.registration_revision,
            'source': registration.source,
        } if registered_local else None),
        'registry_revision': registry_revision,
        'tags': tags,
    }


def resolve_navigation_target(*, robot_id: str, source_type: str, active_map: dict[str, Any],
                              tag_id: int | str | None = None, x: float | None = None,
                              y: float | None = None, yaw: float | None = None,
                              source_map_id: str | None = None,
                              source_map_revision: str | int | None = None) -> dict[str, Any]:
    """Resolve either target source to the shared frame/map-bound contract."""
    rid = str(robot_id or '').strip()
    map_id = str(active_map.get('active_map_id') or '')
    map_revision = str(active_map.get('active_map_revision') or '')
    if not rid:
        raise NavigationTargetError('ROBOT_REQUIRED', 'robot_id is required')
    if not map_id or not map_revision:
        raise NavigationTargetError('MAP_REQUIRED', 'the robot active map is not confirmed')
    source = str(source_type or '').upper()
    if source in ('MAP_POINT', 'ACTIVE_MAP_POINT'):
        if active_map.get('map_sync_status') not in ('CANONICAL', 'LOCAL_ONLY'):
            raise NavigationTargetError('MAP_NOT_READY', 'the robot active map is not ready for navigation')
        if source_map_id is not None and str(source_map_id) != map_id:
            raise NavigationTargetError('MAP_POINT_SOURCE_MISMATCH',
                                         'active-map point does not identify the current active map')
        if source_map_revision is not None and str(source_map_revision) != map_revision:
            raise NavigationTargetError('MAP_POINT_SOURCE_MISMATCH',
                                         'active-map point belongs to a different stable map revision')
        try:
            point = {'x': float(x), 'y': float(y), 'yaw': float(yaw)}
        except (TypeError, ValueError):
            raise NavigationTargetError('MAP_POINT_INVALID', 'map point x, y and yaw are required') from None
        if not all(math.isfinite(value) for value in point.values()):
            raise NavigationTargetError('MAP_POINT_INVALID', 'map point coordinates must be finite')
        return NavigationTarget(rid, 'ACTIVE_MAP_POINT', None, 'map', map_id, map_revision,
                                point['x'], point['y'], point['yaw'], {
                                    'source_map_id': map_id,
                                    'source_map_revision': map_revision,
                                    'source_pose': point,
                                    'map_content_revision': active_map.get('map_content_revision'),
                                }).as_dict()
    if source == 'CANONICAL_MAP_POINT':
        if active_map.get('map_sync_status') not in ('CANONICAL', 'LOCAL_ONLY'):
            raise NavigationTargetError('MAP_NOT_READY', 'the robot active map is not ready for navigation')
        try:
            point = {'x': float(x), 'y': float(y), 'yaw': float(yaw)}
            canonical_revision = int(str(source_map_revision or ''))
        except (TypeError, ValueError):
            raise NavigationTargetError('MAP_POINT_INVALID',
                'canonical point x, y, yaw and source map revision are required') from None
        if not all(math.isfinite(value) for value in point.values()):
            raise NavigationTargetError('MAP_POINT_INVALID', 'canonical point coordinates must be finite')
        if str(source_map_id or '') != 'CANONICAL':
            raise NavigationTargetError('MAP_POINT_SOURCE_MISMATCH',
                'GLOBAL warehouse points must identify the CANONICAL source map')
        warehouse_map = WarehouseMap.objects.filter(is_active=True).order_by('id').first()
        if (warehouse_map is None or int(warehouse_map.revision) != canonical_revision
                or str(active_map.get('canonical_map_revision') or '') != str(canonical_revision)):
            raise NavigationTargetError('CANONICAL_MAP_REVISION_MISMATCH',
                'canonical point does not match the active warehouse and robot map baseline')
        registration = None
        if map_id == 'CANONICAL':
            if map_revision != str(canonical_revision) or active_map.get('map_sync_status') != 'CANONICAL':
                raise NavigationTargetError('CANONICAL_MAP_REVISION_MISMATCH',
                    'robot canonical map identity does not match the selected GLOBAL map')
            resolved = point
        else:
            registration = _active_registration(rid, active_map, canonical_revision, warehouse_map)
            if registration is None:
                raise NavigationTargetError('MAP_REGISTRATION_REQUIRED',
                    'canonical map point cannot be resolved until the exact active map has a current registration')
            resolved = transform_canonical_pose(point, {
                'tx': registration.tx, 'ty': registration.ty, 'yaw': registration.yaw,
            })
        metadata = {
            'source_map_id': 'CANONICAL',
            'source_map_revision': str(canonical_revision),
            'source_pose': point,
            'active_map_id': map_id,
            'active_map_revision': map_revision,
            'registration_revision': registration.registration_revision if registration else None,
            'registration_source': registration.source if registration else 'IDENTITY_CANONICAL_MAP',
            'map_content_revision': active_map.get('map_content_revision'),
        }
        return NavigationTarget(rid, source, None, 'map', map_id, map_revision,
                                resolved['x'], resolved['y'], resolved['yaw'], metadata).as_dict()
    if source != 'TAG':
        raise NavigationTargetError('SOURCE_TYPE_INVALID',
            'source_type must be ACTIVE_MAP_POINT, CANONICAL_MAP_POINT or TAG')

    registry = navigation_tag_registry(active_map, robot_id=rid)
    registration_required = registry.get('reason') == 'TAG_MAP_REGISTRATION_REQUIRED'
    if not registry['compatible'] and not registration_required:
        code = 'TAG_MAP_REVISION_MISMATCH' if 'revision' in str(registry.get('reason') or '').lower() else 'TAG_MAP_MISMATCH'
        raise NavigationTargetError(code, str(registry.get('reason') or 'Tag registry is incompatible with the robot map'))
    try:
        resolved_tag_id = int(tag_id)
    except (TypeError, ValueError):
        raise NavigationTargetError('TAG_UNKNOWN', 'tag_id must identify a registered Tag') from None
    tag = NavigationTag.objects.filter(warehouse_id=registry['warehouse_id'], tag_id=resolved_tag_id).first()
    if tag is None:
        raise NavigationTargetError('TAG_UNKNOWN', f'tag {resolved_tag_id} is not in the active warehouse registry')
    record = next((item for item in registry['tags'] if int(item['tag_id']) == resolved_tag_id), None)
    if record is None:
        raise NavigationTargetError('TAG_UNKNOWN', f'tag {resolved_tag_id} is not in the active warehouse registry')
    if not tag.enabled:
        raise NavigationTargetError('TAG_DISABLED', f'tag {resolved_tag_id} is disabled')
    if registration_required:
        raise NavigationTargetError('TAG_MAP_REGISTRATION_REQUIRED',
            'canonical Tag navigation is disabled on this local map until a versioned map registration is configured')
    if not record['navigable']:
        raise NavigationTargetError('TAG_NOT_NAVIGABLE', str(record['reason'] or 'Tag is not navigable'))
    target_pose = record['navigation_pose']
    target = NavigationTarget(
        robot_id=rid,
        source_type='TAG',
        source_id=resolved_tag_id,
        frame_id='map',
        map_id=map_id,
        map_revision=map_revision,
        x=target_pose['x'],
        y=target_pose['y'],
        yaw=target_pose['yaw'],
        metadata={
            'tag_id': resolved_tag_id,
            'label': record['label'],
            'family': record['family'],
            'floor_id': record['floor_id'],
            'lane_id': record['lane_id'],
            'zone_id': record['zone_id'],
            'tag_revision': record['tag_revision'],
            'registry_revision': registry['registry_revision'],
            'pose_source': record['navigation_pose_source'],
            'canonical_map_id': registry.get('canonical_map_id'),
            'canonical_map_revision': registry.get('canonical_map_revision'),
            'registration_revision': registry.get('registration_revision'),
            'registration_source': registry.get('transform_source'),
            'source_map_id': registry.get('canonical_map_id'),
            'source_map_revision': registry.get('canonical_map_revision'),
            'source_pose': record.get('canonical_navigation_pose') or record['navigation_pose'],
            'active_map_id': map_id,
            'active_map_revision': map_revision,
            'map_content_revision': active_map.get('map_content_revision'),
        },
    )
    return target.as_dict()
