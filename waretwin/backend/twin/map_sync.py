"""Published map revision metadata shared by HTTP, WebSocket and ROS bridge."""
from __future__ import annotations

import json
from typing import Any

from .map_artifacts import artifact_root
from .models import WarehouseMap


def published_map_payload(active: WarehouseMap | None = None) -> dict[str, Any]:
    """Return the immutable revision currently selected for runtime use."""
    if active is None:
        active = WarehouseMap.objects.select_related("warehouse").filter(is_active=True).first()
    if active is None:
        return {
            "warehouse_id": None, "warehouse_code": None, "revision": None,
            "published_version": 0, "map_revision": None, "artifact_manifest": None,
            "artifact_dir": None,
        }
    version = active.versions.order_by("-version").first()
    revision = int(version.revision) if version else (int(active.revision) if active.published_version else None)
    root = artifact_root() / str(active.warehouse.code) / str(revision) if revision is not None else None
    manifest = None
    if root is not None:
        path = root / "manifest.json"
        try:
            manifest = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            manifest = None
    return {
        "warehouse_id": active.warehouse_id,
        "warehouse_code": active.warehouse.code,
        "revision": revision,
        "published_version": int(active.published_version or 0),
        "map_revision": revision,
        "artifact_manifest": manifest,
        "artifact_dir": str(root) if root is not None else None,
    }


def map_sync_status(*, published_revision: int | None, ros_revision: int | None,
                    gazebo_revision: int | None, ros_connected: bool,
                    error: str | None = None, external: bool = True) -> str:
    if not external:
        return "SYNCED"
    if not ros_connected:
        return "ROS_OFFLINE"
    if error:
        return "ERROR"
    if published_revision is None:
        return "NO_PUBLISHED_MAP"
    if ros_revision != published_revision or gazebo_revision != published_revision:
        return "OUT_OF_SYNC"
    return "SYNCED"
