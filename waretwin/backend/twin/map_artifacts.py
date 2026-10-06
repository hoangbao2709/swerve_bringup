"""Deterministic publish artifacts for a canonical warehouse map.

This module deliberately contains only serialization/orchestration helpers.  The
canonical navigation graph remains the ``navigation_edges`` collection and the
Gazebo geometry is delegated to ``tools.export_gazebo_world``.
"""
from __future__ import annotations

import hashlib
import json
import logging
import math
import re
import shutil
import sys
import tempfile
from pathlib import Path
from typing import Any
from datetime import datetime, timezone

from django.conf import settings

from .canonical_map import canonicalize_layout
from .nav2_export import floor_artifact_name, render_nav2_map
from .navigation_graph import prepare_published_navigation

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.export_gazebo_world import export_gazebo_world  # noqa: E402

log = logging.getLogger(__name__)


def _safe_name(value: Any) -> str:
    text = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value)).strip("._-")
    return text or "warehouse"


def artifact_root() -> Path:
    configured = getattr(settings, "WARETWIN_ARTIFACT_ROOT", None)
    return Path(configured) if configured else ROOT / "generated" / "maps"


def artifact_revision_dir(warehouse_id: Any, revision: int) -> Path:
    """Return the canonical on-disk directory for one immutable revision.

    Every producer and consumer must use the same sanitized warehouse key.  In
    particular, a warehouse code containing spaces or slashes must not publish
    into one directory and later be looked up in another directory.
    """
    return artifact_root() / _safe_name(warehouse_id) / str(int(revision))


def _num(value: Any, default: float = 0.0) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        number = default
    if not math.isfinite(number):
        number = default
    if number == 0:
        return "0"
    return format(number, ".12g")


def _quoted(value: Any) -> str:
    return json.dumps(str(value), ensure_ascii=False)


def _tag_sort_key(tag: dict[str, Any]) -> tuple[int, str]:
    try:
        tag_id = int(tag.get("tag_id"))
    except (TypeError, ValueError):
        tag_id = 2**63 - 1
    return tag_id, str(tag.get("uuid") or "")


def render_datamatrix_yaml(layout: dict[str, Any]) -> str:
    """Render both the legacy ``markers`` list and the richer ``tags`` list."""
    tags = sorted(layout.get("navigation_tags") or [], key=_tag_sort_key)
    lines = ["frame_id: map", "units: m", "markers:"]
    for tag in tags:
        lines.append(
            "  - {tag_id: %s, x: %s, y: %s, yaw: %s, uuid: %s, floor_id: %s, z: %s}"
            % (
                _num(tag.get("tag_id")), _num(tag.get("x")), _num(tag.get("y")),
                _num(tag.get("yaw")), _quoted(tag.get("uuid", "")),
                _quoted(tag.get("floor_id", 1)), _num(tag.get("z")),
            )
        )
    lines.append("tags:")
    for tag in tags:
        lines.append(
            "  - {id: %s, uuid: %s, floor_id: %s, x: %s, y: %s, z: %s, yaw: %s}"
            % (
                _num(tag.get("tag_id")), _quoted(tag.get("uuid", "")),
                _quoted(tag.get("floor_id", 1)), _num(tag.get("x")),
                _num(tag.get("y")), _num(tag.get("z")), _num(tag.get("yaw")),
            )
        )
    return "\n".join(lines) + "\n"


def _tag_id_by_uuid(layout: dict[str, Any]) -> dict[str, int]:
    result: dict[str, int] = {}
    for tag in layout.get("navigation_tags") or []:
        if tag.get("uuid") is None:
            continue
        try:
            result[str(tag["uuid"])] = int(tag["tag_id"])
        except (KeyError, TypeError, ValueError):
            continue
    return result


def render_tag_graph_yaml(layout: dict[str, Any]) -> str:
    """Render the existing ROS ``tags`` mapping plus canonical edge metadata."""
    tags = sorted(layout.get("navigation_tags") or [], key=_tag_sort_key)
    by_uuid = _tag_id_by_uuid(layout)
    by_id = {int(tag["tag_id"]): tag for tag in tags if str(tag.get("tag_id", "")).lstrip("-").isdigit()}
    neighbours: dict[int, set[int]] = {tag_id: set() for tag_id in by_id}
    edges: list[dict[str, Any]] = []
    for raw in layout.get("navigation_edges") or []:
        try:
            from_id = int(raw.get("from_tag_id"))
            to_id = int(raw.get("to_tag_id"))
        except (TypeError, ValueError):
            from_id = by_uuid.get(str(raw.get("from_tag_uuid")), -1)
            to_id = by_uuid.get(str(raw.get("to_tag_uuid")), -1)
        if from_id not in by_id or to_id not in by_id:
            continue
        direction = str(raw.get("direction") or ("bidirectional" if raw.get("bidirectional") else "forward")).lower()
        bidirectional = direction == "bidirectional" or bool(raw.get("bidirectional", False))
        enabled = bool(raw.get("enabled", True))
        if enabled:
            neighbours[from_id].add(to_id)
            if bidirectional:
                neighbours[to_id].add(from_id)
        edges.append({
            "from": from_id,
            "to": to_id,
            "aisle_id": raw.get("aisle_id", raw.get("aisle")),
            "axis": raw.get("axis"),
            "lane_id": raw.get("lane_id", raw.get("aisle_id", raw.get("aisle"))),
            "distance": float(raw.get("distance", 0) or 0),
            "cost": float(raw.get("cost", raw.get("distance", 0)) or 0),
            "clearance_m": float(raw.get("clearance_m", 0) or 0),
            "direction": direction,
            "bidirectional": bidirectional,
            "enabled": enabled,
        })
    edges.sort(key=lambda edge: (edge["from"], edge["to"], str(edge.get("aisle_id") or ""), edge["direction"]))
    lines = ["frame_id: map", "units: m",
             f"canonical_revision: {int(layout.get('revision') or 0)}",
             f"graph_revision: {_quoted(layout.get('tag_graph_revision') or '')}", "tags:"]
    for tag in tags:
        try:
            tag_id = int(tag["tag_id"])
        except (KeyError, TypeError, ValueError):
            continue
        ids = sorted(neighbours.get(tag_id, set()))
        neighbour_text = "[" + ", ".join(str(item) for item in ids) + "]"
        lines.append(f"  {_quoted(tag_id)}:")
        lines.append(f"    uuid: {_quoted(tag.get('uuid', ''))}")
        lines.append(f"    floor_id: {_quoted(tag.get('floor_id', 1))}")
        lines.append(f"    x: {_num(tag.get('x'))}")
        lines.append(f"    y: {_num(tag.get('y'))}")
        lines.append(f"    z: {_num(tag.get('z'))}")
        lines.append(f"    yaw: {_num(tag.get('yaw'))}")
        metadata = tag.get('metadata') if isinstance(tag.get('metadata'), dict) else {}
        lines.append(f"    orientation_policy: {_quoted(metadata.get('orientation_policy', 'EXPLICIT'))}")
        lines.append(f"    neighbors: {neighbour_text}")
    lines.append("edges:")
    for edge in edges:
        aisle = "null" if edge["aisle_id"] is None else _quoted(edge["aisle_id"])
        lines.append(
            "  - {from: %d, to: %d, aisle_id: %s, axis: %s, lane_id: %s, distance: %s, cost: %s, clearance_m: %s, direction: %s, bidirectional: %s, enabled: %s}"
            % (
                edge["from"], edge["to"], aisle,
                _quoted(edge.get("axis") or ""), _quoted(edge.get("lane_id") or edge.get("aisle_id") or ""),
                _num(edge["distance"]), _num(edge["cost"]), _num(edge.get("clearance_m", 0)),
                _quoted(edge["direction"]), "true" if edge["bidirectional"] else "false",
                "true" if edge["enabled"] else "false",
            )
        )
    return "\n".join(lines) + "\n"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _required_files(root: Path) -> list[Path]:
    return [root / "canonical_map.json", root / "datamatrix_map.yaml", root / "tag_graph.yaml",
            root / "gazebo" / "warehouse.world", root / "gazebo" / "manifest.json",
            root / "nav2" / "warehouse.yaml"]


def build_revision_artifacts(layout: dict[str, Any], *, warehouse_id: Any, revision: int,
                             published_version: int) -> tuple[Path, Path, dict[str, Any]]:
    """Build a complete revision in a hidden sibling directory.

    The caller atomically renames the returned staging directory only after all
    files have been verified and the database transaction succeeds.
    """
    doc = prepare_published_navigation(layout, int(revision))
    doc['revision'] = int(revision)
    root = artifact_root()
    warehouse_dir = root / _safe_name(warehouse_id)
    warehouse_dir.mkdir(parents=True, exist_ok=True)
    final = warehouse_dir / str(int(revision))
    if final.exists():
        raise ValueError(f"artifact revision already exists: {final}")
    staging = Path(tempfile.mkdtemp(prefix=f".{int(revision)}.", dir=str(warehouse_dir)))
    try:
        doc['revision'] = int(revision)
        doc['frame_id'] = 'map'
        (staging / "canonical_map.json").write_text(json.dumps(doc, indent=2, sort_keys=True, ensure_ascii=False) + "\n", encoding="utf-8")
        (staging / "datamatrix_map.yaml").write_text(render_datamatrix_yaml(doc), encoding="utf-8")
        (staging / "tag_graph.yaml").write_text(render_tag_graph_yaml(doc), encoding="utf-8")
        export_manifest = export_gazebo_world(doc, staging / "gazebo")
        # PART 7 writes absolute mesh URIs into the world.  Its output is
        # generated in our hidden staging directory, so rewrite only that
        # prefix to the immutable final revision path before hashing/rename.
        world_path = staging / "gazebo" / "warehouse.world"
        world_text = world_path.read_text(encoding="utf-8")
        world_text = world_text.replace(str((staging / "gazebo").resolve()), str((final / "gazebo").resolve()))
        world_path.write_text(world_text, encoding="utf-8")
        nav_dir = staging / 'nav2'
        nav_dir.mkdir(parents=True, exist_ok=True)
        nav2_maps: dict[str, str] = {}
        nav2_bounds: dict[str, dict[str, Any]] = {}
        for index, floor in enumerate(doc.get('floors') or []):
            name = floor_artifact_name(floor.get('id', index + 1))
            yaml_rel = f'nav2/warehouse_{name}.yaml'
            image_rel = f'nav2/warehouse_{name}.pgm'
            image_bytes, yaml_text, raster = render_nav2_map(doc, floor)
            (staging / image_rel).write_bytes(image_bytes)
            (staging / yaml_rel).write_text(yaml_text.format(image=Path(image_rel).name), encoding='utf-8')
            nav2_maps[str(floor.get('id', index + 1))] = yaml_rel
            nav2_bounds[str(floor.get('id', index + 1))] = raster
        if not nav2_maps:
            raise ValueError('canonical map must contain at least one floor')
        primary_nav2 = next(iter(nav2_maps.values()))
        (staging / 'nav2' / 'warehouse.yaml').write_text(
            (staging / primary_nav2).read_text(encoding='utf-8').replace(
                Path(primary_nav2).name, 'warehouse.pgm'), encoding='utf-8')
        primary_image = staging / Path(primary_nav2).with_suffix('.pgm')
        (staging / 'nav2' / 'warehouse.pgm').write_bytes(primary_image.read_bytes())
        missing = [str(path.relative_to(staging)) for path in _required_files(staging) if not path.is_file()]
        if missing:
            raise RuntimeError("Gazebo/artifact export incomplete: " + ", ".join(missing))
        rel_artifacts = {
            "canonical_map": "canonical_map.json",
            "datamatrix_map": "datamatrix_map.yaml",
            "tag_graph": "tag_graph.yaml",
            "gazebo_world": "gazebo/warehouse.world",
            "gazebo_manifest": "gazebo/manifest.json",
            "nav2_map": "nav2/warehouse.yaml",
            "nav2_image": "nav2/warehouse.pgm",
        }
        # Hash every generated source/consumer artifact (including Gazebo meshes
        # and the per-floor Nav2 maps), not just a small role-based subset.
        hashes = {
            path.relative_to(staging).as_posix(): _sha256(path)
            for path in sorted(staging.rglob('*')) if path.is_file()
        }
        origin = doc.get('origin') or {'x': 0.0, 'y': 0.0}
        width, height = float(doc.get('width') or 0.0), float(doc.get('height') or 0.0)
        world_bounds = {
            'min_x': float(origin.get('x', 0.0)),
            'min_y': float(origin.get('y', 0.0)),
            'max_x': float(origin.get('x', 0.0)) + width,
            'max_y': float(origin.get('y', 0.0)) + height,
        }
        manifest = {
            "schema_version": 1,
            "warehouse_id": str(warehouse_id),
            "revision": int(revision),
            "published_version": int(published_version),
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "frame_id": "map",
            "units": "m",
            "origin": doc.get('origin'),
            "width": doc.get('width'),
            "height": doc.get('height'),
            "artifacts": rel_artifacts,
            "sha256": hashes,
            "nav2_maps": nav2_maps,
            "nav2_bounds": nav2_bounds,
            "nav2_resolution": 0.05,
            "canonical_bounds": world_bounds,
            "gazebo_bounds": world_bounds,
            "tag_map_revision": int(revision),
            "tag_graph_revision": doc.get('tag_graph_revision'),
            # Gazebo spawn resolution reads the same canonical robot records
            # from the immutable manifest; do not discard them when wrapping
            # the exporter output in the backend artifact manifest.
            "robots": export_manifest.get("robots", []),
        }
        (staging / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return staging, final, manifest
    except Exception as exc:
        log.exception('Warehouse artifact staging failed: %s', type(exc).__name__)
        shutil.rmtree(staging, ignore_errors=True)
        try:
            if warehouse_dir.exists() and not any(warehouse_dir.iterdir()):
                warehouse_dir.rmdir()
        except OSError as cleanup_exc:
            log.debug('Unable to remove empty artifact directory %s: %s', warehouse_dir, cleanup_exc)
        raise


def cleanup_artifact_dir(path: Path | None) -> None:
    if path and path.exists():
        shutil.rmtree(path, ignore_errors=True)
