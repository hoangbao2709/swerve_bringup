#!/usr/bin/env python3
"""Export a canonical WareTwin layout to a deterministic Gazebo Classic world.

The canonical map uses X/Z for its 2-D floor plane in frontend data structures
(``P3 = [x, vertical, floor_y]``).  This module keeps that conversion in one
place: :func:`p3_to_gazebo` maps it to Gazebo ``(x, y, z)`` while floor points
and tags already use ``(x, y)`` in the warehouse plane.
"""
from __future__ import annotations

import argparse
import hashlib
import html
import json
import math
import os
import re
import shutil
import sys
import tempfile
from pathlib import Path
from typing import Any, Iterable, Sequence


ROOT = Path(__file__).resolve().parents[1]
BACKEND = ROOT / "waretwin" / "backend"
if str(BACKEND) not in sys.path:
    sys.path.insert(0, str(BACKEND))

from twin.canonical_map import canonicalize_layout, validate_canonical_layout  # noqa: E402


Point = tuple[float, float]
EPS = 1e-9


class ExportValidationError(ValueError):
    """Raised when a canonical map cannot be exported safely."""

    def __init__(self, errors: Sequence[str]):
        self.errors = list(errors)
        super().__init__("; ".join(self.errors))


def _number(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ExportValidationError([f"{label}: expected a finite number"]) from exc
    if not math.isfinite(result):
        raise ExportValidationError([f"{label}: expected a finite number"])
    return result


def _point(raw: Any) -> Point:
    if isinstance(raw, dict):
        return (_number(raw.get("x"), "point.x"), _number(raw.get("y"), "point.y"))
    if isinstance(raw, (list, tuple)) and len(raw) >= 2:
        return (_number(raw[0], "point.x"), _number(raw[1], "point.y"))
    raise ValueError("point must contain x/y")


def _points(raw: Any) -> list[Point]:
    if not isinstance(raw, list):
        return []
    try:
        return [_point(value) for value in raw]
    except (TypeError, ValueError, ExportValidationError):
        return []


def _area(poly: Sequence[Point]) -> float:
    return sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1] for i in range(len(poly))) / 2


def _cross(a: Point, b: Point, c: Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a: Point, b: Point, p: Point) -> bool:
    return abs(_cross(a, b, p)) <= EPS and min(a[0], b[0]) - EPS <= p[0] <= max(a[0], b[0]) + EPS and min(a[1], b[1]) - EPS <= p[1] <= max(a[1], b[1]) + EPS


def _segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    o1, o2, o3, o4 = _cross(a, b, c), _cross(a, b, d), _cross(c, d, a), _cross(c, d, b)
    if ((o1 > EPS and o2 < -EPS) or (o1 < -EPS and o2 > EPS)) and ((o3 > EPS and o4 < -EPS) or (o3 < -EPS and o4 > EPS)):
        return True
    return abs(o1) <= EPS and _on_segment(a, b, c) or abs(o2) <= EPS and _on_segment(a, b, d) or abs(o3) <= EPS and _on_segment(c, d, a) or abs(o4) <= EPS and _on_segment(c, d, b)


def _point_in_polygon(point: Point, polygon: Sequence[Point]) -> bool:
    inside = False
    for i, a in enumerate(polygon):
        b = polygon[(i + 1) % len(polygon)]
        if _on_segment(a, b, point):
            return True
        if (a[1] > point[1]) != (b[1] > point[1]) and point[0] < (b[0] - a[0]) * (point[1] - a[1]) / ((b[1] - a[1]) or 1e-30) + a[0]:
            inside = not inside
    return inside


def _triangle_contains(point: Point, triangle: Sequence[Point]) -> bool:
    signs = [_cross(triangle[i], triangle[(i + 1) % 3], point) for i in range(3)]
    return not (min(signs) < -EPS and max(signs) > EPS)


def _clean_ring(ring: Sequence[Point]) -> list[Point]:
    result: list[Point] = []
    for point in ring:
        if not result or math.hypot(point[0] - result[-1][0], point[1] - result[-1][1]) > EPS:
            result.append(point)
    if len(result) > 1 and math.hypot(result[0][0] - result[-1][0], result[0][1] - result[-1][1]) <= EPS:
        result.pop()
    return result


def _visible_bridge(hole_point: Point, outer: Sequence[Point], holes: Sequence[Sequence[Point]], candidate_index: int) -> bool:
    candidate = outer[candidate_index]
    midpoint = ((hole_point[0] + candidate[0]) / 2, (hole_point[1] + candidate[1]) / 2)
    if not _point_in_polygon(midpoint, outer) or any(_point_in_polygon(midpoint, hole) for hole in holes):
        return False
    for ring in [outer, *holes]:
        for i, start in enumerate(ring):
            end = ring[(i + 1) % len(ring)]
            if start in (hole_point, candidate) or end in (hole_point, candidate):
                continue
            if _segments_intersect(hole_point, candidate, start, end):
                return False
    return True


def _bridge_hole(outer: list[Point], hole: list[Point], all_holes: Sequence[Sequence[Point]]) -> list[Point]:
    """Connect one clockwise hole to the outer ring with a deterministic bridge."""
    if _area(outer) < 0:
        outer = list(reversed(outer))
    if _area(hole) > 0:
        hole = list(reversed(hole))
    hole_index = min(range(len(hole)), key=lambda i: (-hole[i][0], hole[i][1], i))
    hole_point = hole[hole_index]
    # Prefer the right-most equally-near outer vertex.  This avoids a bridge
    # that runs back through a concave notch when a hole has symmetric choices.
    candidates = sorted(range(len(outer)), key=lambda i: (round(math.hypot(outer[i][0] - hole_point[0], outer[i][1] - hole_point[1]), 12), -outer[i][0], outer[i][1], i))
    outer_index = next((i for i in candidates if _visible_bridge(hole_point, outer, all_holes, i)), None)
    if outer_index is None:
        raise ExportValidationError(["floor polygon hole cannot be connected for deterministic triangulation"])
    # Walk the hole in its clockwise order, return over the same bridge, then
    # continue the outer CCW ring. Duplicate bridge vertices are intentional.
    walked_hole = hole[hole_index:] + hole[:hole_index] + [hole[hole_index]]
    return outer[: outer_index + 1] + walked_hole + [outer[outer_index]] + outer[outer_index + 1 :]


def triangulate_polygon(boundary: Sequence[Point], holes: Sequence[Sequence[Point]] = ()) -> tuple[list[Point], list[tuple[int, int, int]]]:
    """Deterministically triangulate a simple polygon with optional holes."""
    outer = _clean_ring(boundary)
    if len(outer) < 3:
        raise ExportValidationError(["floor boundary requires at least 3 vertices"])
    if _area(outer) < 0:
        outer.reverse()
    clean_holes = [_clean_ring(hole) for hole in holes if len(_clean_ring(hole)) >= 3]
    clean_holes.sort(key=lambda ring: (-max(point[0] for point in ring), min(point[1] for point in ring)))
    merged = outer
    for hole in clean_holes:
        merged = _bridge_hole(merged, hole, clean_holes)

    vertices = merged[:]
    remaining = list(range(len(vertices)))
    triangles: list[tuple[int, int, int]] = []
    guard = 0
    while len(remaining) > 3 and guard < len(vertices) * len(vertices) * 2:
        guard += 1
        ear_found = False
        for position, current in enumerate(remaining):
            previous = remaining[position - 1]
            following = remaining[(position + 1) % len(remaining)]
            a, b, c = vertices[previous], vertices[current], vertices[following]
            if _cross(a, b, c) <= EPS:
                continue
            triangle = (a, b, c)
            # Bridge vertices occur twice by design.  A duplicate coordinate
            # is the same topological point, not an interior vertex that can
            # invalidate an ear.
            if any(index not in (previous, current, following) and vertices[index] not in triangle and _triangle_contains(vertices[index], triangle) for index in remaining):
                continue
            triangles.append((previous, current, following))
            remaining.pop(position)
            ear_found = True
            break
        if not ear_found:
            raise ExportValidationError(["floor polygon triangulation failed"])
    if len(remaining) == 3:
        triangles.append((remaining[0], remaining[1], remaining[2]))
    if not triangles:
        raise ExportValidationError(["floor polygon produced no triangles"])
    return vertices, triangles


def canonical_to_gazebo(x: float, y: float, z: float = 0.0) -> tuple[float, float, float]:
    """Convert a canonical floor-plane point to Gazebo XYZ without swapping elsewhere."""
    return (float(x), float(y), float(z))


def p3_to_gazebo(position: Sequence[Any], floor_elevation: float) -> tuple[float, float, float]:
    """Convert canonical ``P3=[x, vertical, floor_y]`` to Gazebo XYZ."""
    if len(position) < 3:
        raise ValueError("P3 position requires three coordinates")
    return (float(position[0]), float(position[2]), float(floor_elevation) + float(position[1]))


def _safe_name(value: Any) -> str:
    text = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value)).strip("._-") or "unnamed"
    return text


def _fmt(value: float) -> str:
    return f"{float(value):.9g}"


def _pose(x: float, y: float, z: float, yaw: float = 0.0) -> str:
    return f"{_fmt(x)} {_fmt(y)} {_fmt(z)} 0 0 {_fmt(yaw)}"


def _xml(value: Any) -> str:
    return html.escape(str(value), quote=True)


def _box_model(name: str, pose: str, size: Sequence[float], color: str = "0.35 0.38 0.42 1", static: bool = True) -> str:
    sx, sy, sz = (_fmt(float(item)) for item in size)
    return f'''    <model name="{_xml(name)}">
      <static>{str(static).lower()}</static>
      <pose>{pose}</pose>
      <link name="link">
        <visual name="visual"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
        <collision name="collision"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
      </link>
    </model>'''


def _cylinder_model(name: str, pose: str, radius: float, height: float) -> str:
    return f'''    <model name="{_xml(name)}">
      <static>true</static><pose>{pose}</pose>
      <link name="link">
        <visual name="visual"><geometry><cylinder><radius>{_fmt(radius)}</radius><length>{_fmt(height)}</length></cylinder></geometry><material><ambient>0.32 0.35 0.40 1</ambient><diffuse>0.32 0.35 0.40 1</diffuse></material></visual>
        <collision name="collision"><geometry><cylinder><radius>{_fmt(radius)}</radius><length>{_fmt(height)}</length></cylinder></geometry></collision>
      </link>
    </model>'''


def _tag_model(name: str, pose: str, size: float = 0.15, family: str = "DATAMATRIX") -> str:
    # A small inline marker keeps generated worlds standalone; tag_id/family are
    # also carried by the model name and manifest for camera/ROS integration.
    side = max(0.01, float(size))
    inner = side * 0.75
    return f'''    <model name="{_xml(name)}">
      <static>true</static><pose>{pose}</pose>
      <link name="link">
        <visual name="substrate"><pose>0 0 0.003 0 0 0</pose><geometry><box><size>{_fmt(side)} {_fmt(side)} 0.006</size></box></geometry><material><ambient>0.95 0.95 0.95 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material></visual>
        <visual name="id_marker"><pose>0 0 0.007 0 0 0</pose><geometry><box><size>{_fmt(inner)} {_fmt(inner)} 0.004</size></box></geometry><material><ambient>0.01 0.01 0.01 1</ambient><diffuse>0.01 0.01 0.01 1</diffuse></material></visual>
      </link>
    </model>'''


def _write_floor_mesh(path: Path, boundary: Sequence[Point], holes: Sequence[Sequence[Point]], elevation: float, thickness: float = 0.1) -> dict[str, Any]:
    vertices, triangles = triangulate_polygon(boundary, holes)
    lines = ["# deterministic WareTwin floor mesh", "o floor"]
    for x, y in vertices:
        lines.append(f"v {_fmt(x)} {_fmt(y)} {_fmt(elevation)}")
    for x, y in vertices:
        lines.append(f"v {_fmt(x)} {_fmt(y)} {_fmt(elevation - thickness)}")
    top_offset = len(vertices)
    for a, b, c in triangles:
        lines.append(f"f {a + 1} {b + 1} {c + 1}")
        lines.append(f"f {top_offset + c + 1} {top_offset + b + 1} {top_offset + a + 1}")
    # Side walls are emitted from the original rings so a hole remains an open
    # cut-out in the floor mesh rather than being filled by a bounding box.
    # Top/bottom vertices are written first, then each side contributes four
    # deterministic vertices and one quad face.
    next_index = len(vertices) * 2 + 1
    for ring in [list(boundary), *[list(hole) for hole in holes]]:
        ring = _clean_ring(ring)
        for a, b in zip(ring, ring[1:] + ring[:1]):
            side_indices = [next_index, next_index + 1, next_index + 2, next_index + 3]
            lines.extend([f"v {_fmt(a[0])} {_fmt(a[1])} {_fmt(elevation)}", f"v {_fmt(b[0])} {_fmt(b[1])} {_fmt(elevation)}", f"v {_fmt(b[0])} {_fmt(b[1])} {_fmt(elevation - thickness)}", f"v {_fmt(a[0])} {_fmt(a[1])} {_fmt(elevation - thickness)}"])
            lines.append("f " + " ".join(str(index) for index in side_indices))
            next_index += 4
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return {"vertices": len(vertices), "triangles": len(triangles), "holes": len(holes), "mesh": path.as_posix()}


def _floor_map(layout: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {str(floor.get("id")): floor for floor in layout.get("floors", [])}


def _floor_id(obj: dict[str, Any], default: str) -> str:
    return str(obj.get("floor_id", obj.get("floor", default)))


def _spawn_robot_records(doc: dict[str, Any], floors: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    """Validate canonical spawn records and return deterministic Gazebo poses."""
    raw_robots = (doc.get("spawn") or {}).get("robots") or []
    if not isinstance(raw_robots, list):
        raise ExportValidationError(["spawn.robots: expected a list"])
    records: list[dict[str, Any]] = []
    seen: set[str] = set()
    for index, robot in enumerate(raw_robots):
        if not isinstance(robot, dict):
            raise ExportValidationError([f"spawn.robots[{index}]: expected an object"])
        robot_id = str(robot.get("id", "")).strip()
        if not robot_id:
            raise ExportValidationError([f"spawn.robots[{index}]: id is required"])
        if robot_id in seen:
            raise ExportValidationError([f"spawn robot {robot_id}: duplicate id"])
        seen.add(robot_id)
        floor_id = _floor_id(robot, next(iter(floors), ""))
        floor = floors.get(floor_id)
        if floor is None:
            raise ExportValidationError([f"spawn robot {robot_id}: invalid floor reference {floor_id}"])
        position = robot.get("position")
        if not isinstance(position, (list, tuple)) or len(position) < 3:
            raise ExportValidationError([f"spawn robot {robot_id}: position requires [x, vertical, floor_y]"])
        canonical_position = [_number(position[i], f"spawn robot {robot_id} position[{i}]") for i in range(3)]
        heading = _number(robot.get("heading", 0), f"spawn robot {robot_id} heading")
        elevation = _number(floor.get("elevation", 0), f"floor {floor_id} elevation")
        point = (canonical_position[0], canonical_position[2])
        boundary = _points(floor.get("boundary"))
        holes = [_points(hole) for hole in floor.get("holes") or []]
        if len(boundary) < 3 or not _point_in_polygon(point, boundary):
            raise ExportValidationError([f"spawn robot {robot_id}: spawn point outside floor {floor_id}"])
        if any(_point_in_polygon(point, hole) for hole in holes if len(hole) >= 3):
            raise ExportValidationError([f"spawn robot {robot_id}: spawn point inside hole on floor {floor_id}"])
        gx, gy, gz = p3_to_gazebo(canonical_position, elevation)
        records.append({
            "id": robot_id,
            "floor_id": floor.get("id"),
            "pose": [gx, gy, gz, heading],
            "battery": robot.get("battery", 100),
        })
    return sorted(records, key=lambda record: str(record["id"]))


def validate_export_layout(layout: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(layout, dict):
        raise ExportValidationError(["layout: expected a JSON object"])
    doc = canonicalize_layout(layout)
    errors = list(validate_canonical_layout(doc))
    floors = _floor_map(doc)
    if not floors:
        errors.append("floors: at least one floor is required")
    names: set[str] = set()

    def add_name(name: str, label: str) -> None:
        if name in names:
            errors.append(f"duplicate Gazebo model name: {name} ({label})")
        names.add(name)

    for floor in doc.get("floors", []):
        fid = str(floor.get("id"))
        boundary = _points(floor.get("boundary"))
        if len(boundary) < 3:
            errors.append(f"floor {fid}: invalid boundary")
        add_name(f"floor_{_safe_name(fid)}", f"floor {fid}")
        for index in range(len(boundary)):
            add_name(f"floor_{_safe_name(fid)}_wall_{index}", f"floor {fid} boundary wall")
        for hole_index, hole in enumerate(floor.get("holes") or []):
            for edge_index in range(len(_points(hole))):
                add_name(f"floor_{_safe_name(fid)}_hole_{hole_index}_wall_{edge_index}", f"floor {fid} hole wall")
    for key, prefix in (("racks", "rack"), ("stations", "station"), ("conveyors", "conveyor"), ("obstacles", "obstacle")):
        for index, obj in enumerate(doc.get(key, [])):
            ident = obj.get("id", index)
            add_name(f"{prefix}_{_safe_name(ident)}", f"{key} {ident}")
            if _floor_id(obj, str(doc["floors"][0].get("id"))) not in floors:
                errors.append(f"{key} {ident}: invalid floor reference")
    for index, _ in enumerate(doc.get("columns", [])):
        add_name(f"column_{index}", "column")
    for tag in doc.get("navigation_tags", []):
        uid = str(tag.get("uuid", ""))
        add_name(f"nav_tag_{_safe_name(tag.get('tag_id', 'invalid'))}", f"navigation tag {uid}")
    _spawn_robot_records(doc, floors)
    if errors:
        raise ExportValidationError(sorted(set(errors)))
    return doc


def export_gazebo_world(layout: dict[str, Any], output: Path) -> dict[str, Any]:
    """Validate and export ``warehouse.world`` plus meshes and manifest."""
    doc = validate_export_layout(layout)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{output.name}.", dir=str(output.parent)))
    try:
        models_dir = staging / "models"
        models_dir.mkdir(parents=True, exist_ok=True)
        floors = _floor_map(doc)
        floor_ids = [str(floor.get("id")) for floor in doc.get("floors", [])]
        default_floor = floor_ids[0]
        floor_info: list[dict[str, Any]] = []
        world_models: list[str] = []
        for floor in doc.get("floors", []):
            fid = str(floor.get("id")); elevation = _number(floor.get("elevation", 0), f"floor {fid} elevation")
            boundary = _points(floor.get("boundary")); holes = [_points(hole) for hole in floor.get("holes") or []]
            mesh_path = models_dir / f"floor_{_safe_name(fid)}" / "mesh.obj"
            mesh_path.parent.mkdir(parents=True, exist_ok=True)
            mesh_meta = _write_floor_mesh(mesh_path, boundary, holes, elevation)
            floor_info.append({"id": floor.get("id"), "name": floor.get("name", f"Floor {fid}"), "elevation": elevation, "boundary": boundary, "holes": holes, "mesh": f"models/floor_{_safe_name(fid)}/mesh.obj", "triangles": mesh_meta["triangles"]})
            # The staging directory is renamed atomically after generation;
            # point the final world at the stable output path, not the hidden
            # temporary directory used during validation.
            final_mesh_path = output / "models" / f"floor_{_safe_name(fid)}" / "mesh.obj"
            mesh_uri = f"file://{final_mesh_path.resolve().as_posix()}"
            world_models.append(f'''    <model name="floor_{_safe_name(fid)}">
      <static>true</static><pose>0 0 0 0 0 0</pose>
      <link name="link">
        <visual name="visual"><geometry><mesh><uri>{_xml(mesh_uri)}</uri><scale>1 1 1</scale></mesh></geometry><material><ambient>0.35 0.37 0.40 1</ambient><diffuse>0.35 0.37 0.40 1</diffuse></material></visual>
        <collision name="collision"><geometry><mesh><uri>{_xml(mesh_uri)}</uri><scale>1 1 1</scale></mesh></geometry></collision>
      </link>
    </model>''')
            wall_height = _number(doc.get("gazebo", {}).get("wall_height", 3.0), "wall_height")
            wall_thickness = _number(doc.get("gazebo", {}).get("wall_thickness", 0.2), "wall_thickness")
            for ring_name, ring in [("boundary", boundary), *[(f"hole_{i}", hole) for i, hole in enumerate(holes)]]:
                for index, (a, b) in enumerate(zip(ring, ring[1:] + ring[:1])):
                    length = math.hypot(b[0] - a[0], b[1] - a[1]); yaw = math.atan2(b[1] - a[1], b[0] - a[0])
                    world_models.append(_box_model(f"floor_{_safe_name(fid)}_{ring_name}_wall_{index}", _pose((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, elevation + wall_height / 2, yaw), (length, wall_thickness, wall_height), "0.62 0.65 0.68 1"))

        object_manifest: list[dict[str, Any]] = []
        for rack in doc.get("racks", []):
            ident = str(rack.get("id")); fid = _floor_id(rack, default_floor); elevation = _number(floors[fid].get("elevation", 0), f"floor {fid} elevation")
            position = rack.get("position", [0, 0, 0]); size = rack.get("size", [1, 1, 1]); x, y, z = p3_to_gazebo(position, elevation)
            yaw = math.radians(_number(rack.get("rotation", 0), f"rack {ident} rotation")); model = f"rack_{_safe_name(ident)}"
            world_models.append(_box_model(model, _pose(x + float(size[0]) / 2, y + float(size[2]) / 2, z + float(size[1]) / 2, yaw), (float(size[0]), float(size[2]), float(size[1])), "0.72 0.42 0.12 1"))
            object_manifest.append({"kind": "rack", "id": rack.get("id"), "model": model, "floor_id": rack.get("floor", rack.get("floor_id", default_floor)), "pose": [x, y, z, yaw], "size": [float(size[0]), float(size[2]), float(size[1])]})
        for station in doc.get("stations", []):
            ident = str(station.get("id")); fid = _floor_id(station, default_floor); elevation = _number(floors[fid].get("elevation", 0), f"floor {fid} elevation"); rect = station.get("rect", [0, 0, 1, 1]); x0, y0, x1, y1 = map(float, rect)
            model = f"station_{_safe_name(ident)}"; world_models.append(_box_model(model, _pose((x0 + x1) / 2, (y0 + y1) / 2, elevation + 0.75), (x1 - x0, y1 - y0, 1.5), "0.22 0.47 0.63 1")); object_manifest.append({"kind": "station", "id": station.get("id"), "model": model, "floor_id": station.get("floor", station.get("floor_id", default_floor)), "rect": [x0, y0, x1, y1]})
        for obstacle in doc.get("obstacles", []):
            ident = str(obstacle.get("id")); fid = _floor_id(obstacle, default_floor); elevation = _number(floors[fid].get("elevation", 0), f"floor {fid} elevation"); rect = obstacle.get("rect", [0, 0, 1, 1]); x0, y0, x1, y1 = map(float, rect); model = f"obstacle_{_safe_name(ident)}"; world_models.append(_box_model(model, _pose((x0 + x1) / 2, (y0 + y1) / 2, elevation + 1), (x1 - x0, y1 - y0, 2), "0.30 0.32 0.35 1")); object_manifest.append({"kind": "obstacle", "id": obstacle.get("id"), "model": model, "floor_id": obstacle.get("floor", obstacle.get("floor_id", default_floor)), "rect": [x0, y0, x1, y1]})
        for index, column in enumerate(doc.get("columns", [])):
            x, y = map(float, column); elevation = _number(floors[default_floor].get("elevation", 0), f"floor {default_floor} elevation"); model = f"column_{index}"; world_models.append(_cylinder_model(model, _pose(x, y, elevation + 1.5), 0.45, 3.0)); object_manifest.append({"kind": "column", "id": index, "model": model, "floor_id": doc["floors"][0].get("id"), "pose": [x, y, elevation + 1.5]})
        for conveyor in doc.get("conveyors", []):
            ident = str(conveyor.get("id")); width = float(conveyor.get("width", 1)); fid = _floor_id(conveyor, default_floor); elevation = _number(floors[fid].get("elevation", 0), f"floor {fid} elevation")
            for index, (a, b) in enumerate(zip(conveyor.get("path", [])[0:], conveyor.get("path", [])[1:])):
                ax, ay = map(float, a); bx, by = map(float, b); length = math.hypot(bx - ax, by - ay); yaw = math.atan2(by - ay, bx - ax); model = f"conveyor_{_safe_name(ident)}_{index}"; world_models.append(_box_model(model, _pose((ax + bx) / 2, (ay + by) / 2, elevation + 0.35, yaw), (length, width, 0.7), "0.15 0.55 0.65 1")); object_manifest.append({"kind": "conveyor", "id": ident, "segment": index, "model": model, "floor_id": conveyor.get("floor", conveyor.get("floor_id", default_floor))})
        tag_manifest: list[dict[str, Any]] = []
        for tag in doc.get("navigation_tags", []):
            uid = str(tag.get("uuid")); fid = str(tag.get("floor_id", default_floor)); elevation = _number(floors[fid].get("elevation", 0), f"floor {fid} elevation"); x = _number(tag.get("x"), f"tag {uid} x"); y = _number(tag.get("y"), f"tag {uid} y"); z = elevation + _number(tag.get("z", 0), f"tag {uid} z"); yaw = _number(tag.get("yaw"), f"tag {uid} yaw"); tag_id = int(tag.get("tag_id")); family = str(tag.get("family") or "DATAMATRIX").upper(); size = _number(tag.get("size", 0.15), f"tag {uid} size");
            if size <= 0.0 or size > 2.0:
                raise ExportValidationError([f"tag {uid} size must be in (0, 2] m"])
            model = f"nav_tag_{_safe_name(tag_id)}"; world_models.append(_tag_model(model, _pose(x, y, z, yaw), size, family)); tag_manifest.append({"uuid": uid, "tag_id": tag_id, "family": family, "size": size, "floor_id": tag.get("floor_id", default_floor), "lane_id": tag.get("lane_id"), "zone_id": tag.get("zone_id"), "model": model, "pose": [x, y, z, yaw]})

        robot_manifest = _spawn_robot_records(doc, floors)
        canonical_json = json.dumps(doc, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
        revision = str(doc.get("map_revision") or doc.get("revision") or hashlib.sha256(canonical_json).hexdigest()[:16])
        world = "<?xml version=\"1.0\"?>\n<sdf version=\"1.6\">\n  <world name=\"warehouse_generated\">\n    <plugin name=\"gazebo_ros_state\" filename=\"libgazebo_ros_state.so\"/>\n    <gravity>0 0 -9.81</gravity>\n    <include><uri>model://sun</uri></include>\n" + "\n".join(world_models) + "\n  </world>\n</sdf>\n"
        (staging / "warehouse.world").write_text(world, encoding="utf-8")
        manifest = {"schema_version": 1, "map_revision": revision, "world": "warehouse.world", "floors": floor_info, "tags": tag_manifest, "objects": object_manifest, "robots": robot_manifest}
        (staging / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        if output.exists():
            if output.is_dir(): shutil.rmtree(output)
            else: output.unlink()
        os.replace(staging, output)
        return manifest
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", required=True, type=Path, help="canonical warehouse layout JSON")
    parser.add_argument("--output", required=True, type=Path, help="generated output directory")
    args = parser.parse_args(argv)
    try:
        layout = json.loads(args.layout.read_text(encoding="utf-8"))
        manifest = export_gazebo_world(layout, args.output)
    except (OSError, json.JSONDecodeError, ExportValidationError) as exc:
        print(f"Gazebo export failed: {exc}", file=sys.stderr)
        return 2
    print(f"Exported {manifest['world']} ({len(manifest['floors'])} floors, {len(manifest['tags'])} tags) to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
