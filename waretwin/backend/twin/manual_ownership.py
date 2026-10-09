"""Exclusive, expiring per-robot manual-control lease helpers."""
from __future__ import annotations

MANUAL_OWNER_TTL_S = 0.55


def active_owner(owners: dict, robot_id: str, now: float) -> dict | None:
    owner = owners.get(robot_id)
    if not isinstance(owner, dict) or float(owner.get('expires_at', 0.0)) <= now:
        if owner is not None:
            owners.pop(robot_id, None)
        return None
    return owner


def acquire(owners: dict, *, robot_id: str, channel_name: str, lease_id: str,
            identity: str, now: float, ttl: float = MANUAL_OWNER_TTL_S) -> tuple[bool, str | None]:
    current = active_owner(owners, robot_id, now)
    if current and (current.get('channel_name') != channel_name or current.get('lease_id') != lease_id):
        return False, 'MANUAL_CONTROL_OWNED'
    owners[robot_id] = {
        'channel_name': channel_name,
        'lease_id': lease_id,
        'identity': identity,
        'expires_at': now + ttl,
    }
    return True, None


def refresh(owners: dict, *, robot_id: str, channel_name: str, lease_id: str,
            now: float, ttl: float = MANUAL_OWNER_TTL_S) -> tuple[bool, str | None]:
    current = active_owner(owners, robot_id, now)
    if not current:
        return False, 'MANUAL_LEASE_EXPIRED'
    if current.get('channel_name') != channel_name or current.get('lease_id') != lease_id:
        return False, 'MANUAL_CONTROL_NOT_OWNER'
    current['expires_at'] = now + ttl
    return True, None


def release(owners: dict, *, robot_id: str, channel_name: str,
            lease_id: str | None = None, require_owner: bool = True) -> tuple[bool, str | None]:
    current = owners.get(robot_id)
    if current is None:
        return True, None
    if isinstance(current, str):  # compatibility for an in-flight pre-upgrade owner
        matches = current == channel_name
    else:
        matches = current.get('channel_name') == channel_name and (
            lease_id is None or current.get('lease_id') == lease_id)
    if not matches and require_owner:
        return False, 'MANUAL_CONTROL_NOT_OWNER'
    owners.pop(robot_id, None)
    return True, None
