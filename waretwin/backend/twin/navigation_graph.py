"""Geometry-derived, orthogonal navigation graph for published warehouse Tags.

Graph edges are generated from the aisle geometry in the canonical map, not
from Tag identifier patterns.  Each edge is checked against the published
floor/rack/obstacle geometry with a small swept-clearance sample before it is
published.  The helper is deliberately ROS- and frontend-independent.
"""
from __future__ import annotations

import hashlib
import heapq
import json
import math
from typing import Any

from .canonical_map import canonicalize_layout
from .nav2_export import _floor_obstacle

AXIS_TOLERANCE_M = 0.05
AISLE_PROJECTION_TOLERANCE_M = 0.35
ROBOT_ROUTE_CLEARANCE_M = 0.30
ROUTE_SAMPLE_STEP_M = 0.05
# A robot localized within this distance of a valid lane node is treated as
# already at the graph entry. Issuing a separate Nav2 pose for a few-centimeter
# connector can demand an arbitrary heading at effectively the same position;
# the entry Tag pose is a better-defined, already approved route target.
TAG_ENTRY_CAPTURE_RADIUS_M = 0.15
# Shelf-service associations are only inferred at a clearly aligned rack
# endcap, on a published aisle, and within this stand-off band.  The current
# WH-TEST-01 bundle places its service markers 2.5 m beyond rack end faces.
# These limits reject unrelated aisle intersections and leave a documented
# allowance for a half-metre tag-grid offset on the short rack axis.
SHELF_SERVICE_MIN_STANDOFF_M = 0.5
SHELF_SERVICE_MAX_STANDOFF_M = 3.0
SHELF_SERVICE_ALIGNMENT_TOLERANCE_M = 0.6
SHELF_SERVICE_CLEARANCE_M = 0.35


def _tag_sources(tag: dict[str, Any]) -> set[str]:
    values = tag.get('source_aisles')
    if not isinstance(values, list):
        values = str(tag.get('generated_from') or '').split(',')
    return {str(value).strip() for value in values if str(value).strip()}


def _project(centerline: list[dict[str, Any]], x: float, y: float) -> tuple[float, float] | None:
    """Return perpendicular distance and cumulative distance along polyline."""
    along = 0.0
    best: tuple[float, float] | None = None
    def xy(point: Any) -> tuple[float, float]:
        if isinstance(point, dict):
            return float(point['x']), float(point['y'])
        return float(point[0]), float(point[1])
    for raw_start, raw_end in zip(centerline, centerline[1:]):
        ax, ay = xy(raw_start)
        bx, by = xy(raw_end)
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            continue
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (length * length)))
        distance = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
        candidate = (distance, along + t * length)
        if best is None or candidate[0] < best[0]:
            best = candidate
        along += length
    return best


def _segment_is_clear(layout: dict[str, Any], floor: dict[str, Any], a: tuple[float, float],
                      b: tuple[float, float], clearance: float = ROBOT_ROUTE_CLEARANCE_M) -> bool:
    """Conservatively sample the robot center and footprint sides along a lane."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return False
    nx, ny = -dy / length, dx / length
    steps = max(1, math.ceil(length / ROUTE_SAMPLE_STEP_M))
    default_floor = str((layout.get('floors') or [floor])[0].get('id'))
    for index in range(steps + 1):
        ratio = index / steps
        x, y = a[0] + dx * ratio, a[1] + dy * ratio
        for offset in (-clearance, 0.0, clearance):
            if _floor_obstacle(x + nx * offset, y + ny * offset, floor, layout, default_floor):
                return False
    return True


def _lane_for_segment(layout: dict[str, Any], floor_id: str, a: tuple[float, float],
                      b: tuple[float, float]) -> tuple[str, str] | None:
    """Return (lane_id, axis) only when the complete segment lies in one aisle."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    if length <= AXIS_TOLERANCE_M:
        return ('', 'X')
    if abs(dx) <= AXIS_TOLERANCE_M and abs(dy) > AXIS_TOLERANCE_M:
        axis = 'Y'
    elif abs(dy) <= AXIS_TOLERANCE_M and abs(dx) > AXIS_TOLERANCE_M:
        axis = 'X'
    else:
        return None
    # Check aisle membership before the more expensive obstacle-footprint
    # samples. Most candidate connectors are rejected by lane geometry alone.
    samples = max(1, math.ceil(length / 0.25))
    for aisle in layout.get('aisles') or []:
        if str(aisle.get('floor_id', aisle.get('floor', 1))) != floor_id:
            continue
        width = float(aisle.get('width') or 0.0)
        if width < 2 * ROBOT_ROUTE_CLEARANCE_M:
            continue
        centerline = aisle.get('centerline') or []
        projections = [_project(centerline, a[0] + dx * index / samples,
                                a[1] + dy * index / samples) for index in range(samples + 1)]
        if all(item is not None and item[0] <= width / 2 - ROBOT_ROUTE_CLEARANCE_M + 1e-9
               for item in projections):
            floor = next((item for item in layout.get('floors') or [] if str(item.get('id')) == floor_id), None)
            if floor is not None and _segment_is_clear(layout, floor, a, b):
                return str(aisle.get('id') or aisle.get('uuid') or ''), axis
    return None


def _orthogonal_connector(layout: dict[str, Any], floor_id: str, start: tuple[float, float],
                          end: tuple[float, float]) -> list[tuple[float, float, str, str]] | None:
    """Find a lane-constrained Manhattan connector; no free-space diagonal."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    candidates: list[list[tuple[float, float]]] = []
    if abs(dx) <= AXIS_TOLERANCE_M or abs(dy) <= AXIS_TOLERANCE_M:
        candidates.append([start, end])
    else:
        candidates.extend(([start, (start[0], end[1]), end],
                           [start, (end[0], start[1]), end]))
    result = []
    for points in candidates:
        legs = []
        valid = True
        for a, b in zip(points, points[1:]):
            lane = _lane_for_segment(layout, floor_id, a, b)
            if lane is None:
                valid = False
                break
            if math.hypot(b[0] - a[0], b[1] - a[1]) > AXIS_TOLERANCE_M:
                legs.append((b[0], b[1], lane[0], lane[1]))
        if valid:
            return legs
    return None


def validate_orthogonal_route(points: list[dict[str, Any]] | list[tuple[float, float]],
                              tolerance: float = AXIS_TOLERANCE_M) -> bool:
    """Reject any nominal route leg whose two coordinates both change."""
    def xy(point: Any) -> tuple[float, float]:
        if isinstance(point, dict):
            return float(point['x']), float(point['y'])
        return float(point[0]), float(point[1])
    for first, second in zip(points, points[1:]):
        ax, ay = xy(first)
        bx, by = xy(second)
        if abs(ax - bx) > tolerance and abs(ay - by) > tolerance:
            return False
    return True


def plan_orthogonal_tag_route(layout: dict[str, Any], start_pose: dict[str, Any],
                              destination_tag_id: int,
                              canonical_revision: int | None = None) -> dict[str, Any]:
    """Plan a lane-constrained graph route from an arbitrary start pose to Tag.

    The start connector is also required to lie within aisle geometry. The
    planner chooses a reachable graph entry, then runs Dijkstra with a small
    turn penalty so near-equal routes favor fewer turns.
    """
    # Normalize and generate the graph against the exact published revision.
    # WarehouseMap.layout can retain an earlier embedded revision after a
    # publish increments the authoritative model revision; letting that value
    # leak into graph_revision makes preview and per-leg reauthorization
    # disagree even though the graph geometry is unchanged.
    doc = prepare_published_navigation(layout, canonical_revision)
    try:
        start = (float(start_pose['x']), float(start_pose['y']))
        start_yaw = float(start_pose.get('yaw', 0.0))
        target_id = int(destination_tag_id)
    except (KeyError, TypeError, ValueError):
        raise ValueError('start pose and destination Tag must be valid') from None
    if not all(math.isfinite(value) for value in (*start, start_yaw)):
        raise ValueError('start pose must contain finite coordinates')
    # Swerve can traverse either warehouse axis with the same cardinal body
    # heading (the perpendicular axis uses lateral velocity). Keep one
    # cardinal transit heading through connector corners and intermediate
    # graph nodes; rotating to every segment tangent creates tiny rotate-stop
    # goals at L connectors and turns. Only the final destination restores its
    # semantic service heading.
    transit_yaw = _cardinal_yaw(start_yaw)
    tags = {int(tag['tag_id']): tag for tag in doc.get('navigation_tags') or []
            if tag.get('enabled', True)}
    target = tags.get(target_id)
    if target is None:
        raise ValueError(f'destination Tag {target_id} is missing or disabled')
    target_metadata = target.get('metadata') if isinstance(target.get('metadata'), dict) else {}
    final_key = next((key for key in ('service_pose', 'approach_pose', 'navigation_pose', 'goal_pose')
                      if key in target_metadata), None)
    if final_key:
        raw_final_pose = target_metadata.get(final_key)
        if not isinstance(raw_final_pose, dict):
            raise ValueError(f'Tag {target_id} {final_key} must be a pose object')
        destination_pose = {axis: float(raw_final_pose.get(axis, 0.0) if axis == 'yaw' else raw_final_pose[axis])
                            for axis in ('x', 'y', 'yaw')}
        if not all(math.isfinite(value) for value in destination_pose.values()):
            raise ValueError(f'Tag {target_id} {final_key} must contain finite pose values')
    else:
        destination_pose = {'x': float(target['x']), 'y': float(target['y']), 'yaw': float(target['yaw'])}
    adjacency: dict[int, list[dict[str, Any]]] = {tag_id: [] for tag_id in tags}
    incoming: dict[int, list[int]] = {tag_id: [] for tag_id in tags}
    for edge in doc.get('navigation_edges') or []:
        if not edge.get('enabled', True):
            continue
        source, destination = int(edge['from_tag_id']), int(edge['to_tag_id'])
        if source not in tags or destination not in tags:
            continue
        dx = float(tags[destination]['x']) - float(tags[source]['x'])
        dy = float(tags[destination]['y']) - float(tags[source]['y'])
        if not validate_orthogonal_route([(0.0, 0.0), (dx, dy)]):
            continue
        edge_record = {**edge, 'axis': 'X' if abs(dy) <= AXIS_TOLERANCE_M else 'Y'}
        adjacency[source].append(edge_record)
        incoming[destination].append(source)
        if bool(edge.get('bidirectional')) or str(edge.get('direction')) == 'bidirectional':
            adjacency[destination].append({**edge_record, 'from_tag_id': destination,
                                           'to_tag_id': source, 'direction': 'reverse'})
            incoming[source].append(destination)

    floor_ids = {tag_id: str(tag.get('floor_id', 1)) for tag_id, tag in tags.items()}
    floor_of_target = floor_ids[target_id]
    # Find graph nodes that can actually reach this destination first, then
    # choose the nearest geometrically connected lane entry. This avoids
    # repeatedly checking long connectors for every node in a large warehouse.
    reachable = {target_id}
    frontier = [target_id]
    while frontier:
        node = frontier.pop()
        for previous_node in incoming.get(node, []):
            if previous_node not in reachable:
                reachable.add(previous_node)
                frontier.append(previous_node)
    entries = sorted((tag_id for tag_id in reachable if floor_ids[tag_id] == floor_of_target),
                     key=lambda tag_id: (abs(float(tags[tag_id]['x']) - start[0])
                                         + abs(float(tags[tag_id]['y']) - start[1]), tag_id))
    chosen_entry = None
    chosen_connector = None
    for entry_id in entries:
        entry = tags[entry_id]
        connector = _orthogonal_connector(doc, floor_ids[entry_id], start,
                                           (float(entry['x']), float(entry['y'])))
        if connector is not None:
            chosen_entry, chosen_connector = entry_id, connector
            break
    if chosen_entry is None or chosen_connector is None:
        raise ValueError(f'no orthogonal aisle entry reaches Tag {target_id} from the current pose')

    original_connector = chosen_connector
    entry_x, entry_y = float(tags[chosen_entry]['x']), float(tags[chosen_entry]['y'])
    entry_dx, entry_dy = abs(entry_x - start[0]), abs(entry_y - start[1])
    connector_distance = math.hypot(entry_dx, entry_dy)
    if (connector_distance <= TAG_ENTRY_CAPTURE_RADIUS_M
            and (entry_dx <= AXIS_TOLERANCE_M or entry_dy <= AXIS_TOLERANCE_M)):
        # The short connector remains included in route cost/preview geometry,
        # but the robot navigates directly to the graph entry's semantic Tag
        # pose. This avoids a separate micro-goal with an unstable arbitrary
        # yaw. A real L-shaped connector (both axes exceed tolerance) remains
        # explicit and is still executed as its own orthogonal turn points.
        chosen_connector = []

    turn_penalty_m = 0.15
    # Dijkstra state includes the prior axis, so turns are explicit and
    # deterministic rather than hidden inside a free-space planner.
    initial_axis = chosen_connector[-1][3] if chosen_connector else ''
    start_state = (chosen_entry, initial_axis)
    start_cost = (connector_distance if not chosen_connector and original_connector
                  else sum(math.hypot((item[0] - (start[0] if i == 0 else chosen_connector[i-1][0])),
                                      (item[1] - (start[1] if i == 0 else chosen_connector[i-1][1])))
                           for i, item in enumerate(chosen_connector)))
    distances = {start_state: start_cost}
    previous: dict[tuple[int, str], tuple[tuple[int, str], dict[str, Any]]] = {}
    queue = [(start_cost, chosen_entry, initial_axis)]
    terminal: tuple[int, str] | None = None
    while queue:
        cost, node, previous_axis = heapq.heappop(queue)
        state = (node, previous_axis)
        if cost != distances.get(state):
            continue
        if node == target_id:
            terminal = state
            break
        for edge in adjacency.get(node, []):
            nxt, axis = int(edge['to_tag_id']), str(edge['axis'])
            if floor_ids.get(nxt) != floor_ids.get(node):
                continue
            length = float(edge.get('distance') or math.hypot(
                float(tags[nxt]['x']) - float(tags[node]['x']),
                float(tags[nxt]['y']) - float(tags[node]['y'])))
            candidate = cost + length + (turn_penalty_m if previous_axis and previous_axis != axis else 0.0)
            next_state = (nxt, axis)
            if candidate < distances.get(next_state, math.inf):
                distances[next_state] = candidate
                previous[next_state] = (state, edge)
                heapq.heappush(queue, (candidate, nxt, axis))
    if terminal is None:
        raise ValueError(f'no orthogonal aisle route reaches Tag {target_id} from the current pose')
    route_edges = []
    route_nodes = [terminal[0]]
    cursor = terminal
    while cursor != start_state:
        parent, edge = previous[cursor]
        route_edges.append(edge)
        route_nodes.append(parent[0])
        cursor = parent
    route_edges.reverse()
    route_nodes.reverse()
    route_cost = distances[terminal]
    orientation_policy = str(target_metadata.get('orientation_policy') or '').upper()
    if orientation_policy == 'SHELF_WIDTH_PARALLEL':
        # The width-axis constraint is modulo pi: both robot headings whose
        # base_link Y axes lie along the rack's long axis are valid service
        # poses. Prefer the one matching the stable transit heading so the
        # robot does not perform an unnecessary 180-degree spin at the shelf.
        # For a rotated rack these candidates remain non-cardinal as required
        # by its actual geometry; only the direction is selected here.
        base_yaw = float(destination_pose['yaw'])
        alternate_yaw = math.atan2(math.sin(base_yaw + math.pi),
                                   math.cos(base_yaw + math.pi))
        def heading_error(candidate: float) -> float:
            return abs(math.atan2(math.sin(candidate - transit_yaw),
                                  math.cos(candidate - transit_yaw)))
        destination_pose = {
            **destination_pose,
            'yaw': min((base_yaw, alternate_yaw), key=heading_error),
        }
    best_connector = chosen_connector
    entry_id = chosen_entry
    points: list[dict[str, Any]] = [{'x': start[0], 'y': start[1], 'yaw': start_yaw,
                                     'kind': 'START', 'tag_id': None}]
    current = start
    segments: list[dict[str, Any]] = []
    for x, y, lane_id, axis in best_connector:
        distance = math.hypot(x - current[0], y - current[1])
        if distance <= AXIS_TOLERANCE_M:
            current = (x, y)
            continue
        point = {'x': x, 'y': y, 'yaw': transit_yaw,
                 'kind': 'LANE_CONNECTOR', 'tag_id': None}
        segments.append({'from': points[-1].get('tag_id') or 'ROBOT_START', 'to': 'LANE_CONNECTOR',
                         'axis': axis, 'lane_id': lane_id, 'length_m': distance,
                         'direction': 'forward'})
        points.append(point)
        current = (x, y)
    for index, tag_id in enumerate(route_nodes):
        tag = tags[tag_id]
        point = {'x': float(tag['x']), 'y': float(tag['y']), 'yaw': transit_yaw,
                 'kind': 'TAG', 'tag_id': tag_id}
        if math.hypot(point['x'] - current[0], point['y'] - current[1]) > AXIS_TOLERANCE_M:
            edge = route_edges[index - 1] if index else None
            axis = str(edge.get('axis')) if edge else ('X' if abs(point['y'] - current[1]) <= AXIS_TOLERANCE_M else 'Y')
            if abs(point['x'] - current[0]) > AXIS_TOLERANCE_M and abs(point['y'] - current[1]) > AXIS_TOLERANCE_M:
                raise ValueError('route contains a diagonal Tag segment')
            segments.append({'from': route_nodes[index - 1] if index else
                             ('LANE_CONNECTOR' if best_connector else 'ROBOT_START'),
                             'to': tag_id, 'axis': axis,
                             'lane_id': edge.get('aisle_id') if edge else None,
                             'length_m': math.hypot(point['x'] - current[0], point['y'] - current[1]),
                             'direction': edge.get('direction', 'forward') if edge else 'forward'})
        points.append(point)
        current = (point['x'], point['y'])
    if math.hypot(destination_pose['x'] - current[0], destination_pose['y'] - current[1]) > AXIS_TOLERANCE_M:
        service_connector = _orthogonal_connector(doc, floor_ids[target_id], current,
                                                   (destination_pose['x'], destination_pose['y']))
        if service_connector is None:
            raise ValueError('Tag service/approach pose is not connected to the route by an orthogonal lane segment')
        for x, y, lane_id, axis in service_connector:
            distance = math.hypot(x - current[0], y - current[1])
            if distance <= AXIS_TOLERANCE_M:
                current = (x, y)
                continue
            segments.append({'from': target_id, 'to': 'TAG_SERVICE_POSE', 'axis': axis,
                             'lane_id': lane_id, 'length_m': distance, 'direction': 'forward'})
            points.append({'x': x, 'y': y, 'yaw': transit_yaw,
                           'kind': 'SERVICE_APPROACH', 'tag_id': target_id})
            current = (x, y)
    # Final service heading is applied at the final physical/approach position.
    points[-1] = {**points[-1], 'x': destination_pose['x'], 'y': destination_pose['y'],
                  'yaw': destination_pose['yaw'], 'kind': 'TAG_SERVICE', 'tag_id': target_id}
    if not validate_orthogonal_route(points):
        raise ValueError('route contains a diagonal nominal segment')
    return {
        'route_nodes': route_nodes,
        'route_segments': segments,
        'route_points': points,
        'route_cost_m': route_cost,
        'destination_pose': destination_pose,
        'orientation_policy': orientation_policy or None,
        'graph_revision': doc.get('tag_graph_revision'),
        'turn_penalty_m': turn_penalty_m,
    }


def _cardinal_yaw(yaw: float) -> float:
    return math.atan2(math.sin(round(yaw / (math.pi / 2)) * (math.pi / 2)),
                      math.cos(round(yaw / (math.pi / 2)) * (math.pi / 2)))


def _aisle_at(tag: dict[str, Any], aisles: list[dict[str, Any]]) -> dict[str, Any] | None:
    sources = _tag_sources(tag)
    for aisle in aisles:
        aisle_id = str(aisle.get('id') or aisle.get('uuid') or '')
        if aisle_id in sources:
            return aisle
    return None


def _rack_service_geometry(tag: dict[str, Any], rack: dict[str, Any],
                           layout: dict[str, Any]) -> dict[str, Any] | None:
    """Validate an endcap service pose against rack, floor and aisle geometry.

    Rack map geometry follows the same lower-corner-plus-half-size convention
    used by the canonical Nav2 rasterizer.  A valid shelf service point must
    be outside the rack, close to exactly one longitudinal end face, aligned
    with that rack's centreline, and inside one of the Tag's published aisles.
    """
    try:
        metadata = tag.get('metadata') if isinstance(tag.get('metadata'), dict) else {}
        pose_key = next((key for key in ('service_pose', 'approach_pose',
                                         'navigation_pose', 'goal_pose')
                         if key in metadata), None)
        pose = metadata.get(pose_key) if pose_key else tag
        if not isinstance(pose, dict):
            return None
        x, y = float(pose['x']), float(pose['y'])
        floor_id = str(tag.get('floor_id', 1))
        if str(rack.get('floor_id', rack.get('floor', 1))) != floor_id:
            return None
        position, size = rack.get('position') or [], rack.get('size') or []
        if len(position) < 3 or len(size) < 3:
            return None
        sx, sy = float(size[0]), float(size[2])
        if not all(math.isfinite(value) and value > 0 for value in (sx, sy)):
            return None
        # Position is the canonical lower corner; raster and route collision
        # checks rotate the rectangle about this derived centre.
        cx, cy = float(position[0]) + sx / 2, float(position[2]) + sy / 2
        rack_yaw = math.radians(float(rack.get('rotation') or 0.0))
        dx, dy = x - cx, y - cy
        c, s = math.cos(rack_yaw), math.sin(rack_yaw)
        local_x, local_y = c * dx + s * dy, -s * dx + c * dy
        if sx >= sy:
            along, across, half_long = local_x, local_y, sx / 2
            long_yaw = rack_yaw
        else:
            along, across, half_long = local_y, local_x, sy / 2
            long_yaw = rack_yaw + math.pi / 2
        standoff = abs(along) - half_long
        if not (SHELF_SERVICE_MIN_STANDOFF_M <= standoff <= SHELF_SERVICE_MAX_STANDOFF_M):
            return None
        if abs(across) > SHELF_SERVICE_ALIGNMENT_TOLERANCE_M:
            return None

        source_aisles = _tag_sources(tag)
        aisle_matches = []
        for aisle in layout.get('aisles') or []:
            aisle_id = str(aisle.get('id') or aisle.get('uuid') or '')
            if source_aisles and aisle_id not in source_aisles:
                continue
            if str(aisle.get('floor_id', aisle.get('floor', 1))) != floor_id:
                continue
            centerline = aisle.get('centerline') or []
            projected = _project(centerline, x, y)
            width = float(aisle.get('width') or 0.0)
            if (projected is not None and width >= 2 * SHELF_SERVICE_CLEARANCE_M
                    and projected[0] <= width / 2 - SHELF_SERVICE_CLEARANCE_M + 1e-9):
                aisle_matches.append(aisle_id)
        if not aisle_matches:
            return None

        floor = next((item for item in layout.get('floors') or []
                      if str(item.get('id')) == floor_id), None)
        if floor is None:
            return None
        default_floor = str((layout.get('floors') or [floor])[0].get('id'))
        # A small radial footprint check prevents an otherwise aligned point
        # from being classified as service when it overlaps a rack/wall.
        for index in range(8):
            angle = index * math.pi / 4
            if _floor_obstacle(x + SHELF_SERVICE_CLEARANCE_M * math.cos(angle),
                               y + SHELF_SERVICE_CLEARANCE_M * math.sin(angle),
                               floor, layout, default_floor):
                return None
        return {
            'rack_id': str(rack.get('id') or rack.get('uuid') or ''),
            'service_face': ('LONG_AXIS_POSITIVE_END' if along >= 0
                             else 'LONG_AXIS_NEGATIVE_END'),
            'service_standoff_m': round(standoff, 6),
            'rack_long_yaw': math.atan2(math.sin(long_yaw), math.cos(long_yaw)),
            'aisle_id': sorted(aisle_matches)[0],
        }
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


def _rack_service_candidates(tag: dict[str, Any], layout: dict[str, Any]) -> list[tuple[dict[str, Any], dict[str, Any]]]:
    matches = []
    for rack in layout.get('racks') or []:
        geometry = _rack_service_geometry(tag, rack, layout)
        if geometry is not None:
            matches.append((rack, geometry))
    return matches


def _associate_shelf_service_tags(layout: dict[str, Any],
                                  canonical_revision: int | None) -> None:
    """Set shelf semantics only for unambiguous, geometry-validated tags."""
    racks = {str(rack.get('id') or rack.get('uuid') or ''): rack
             for rack in layout.get('racks') or []}
    for tag in layout.get('navigation_tags') or []:
        metadata = dict(tag.get('metadata') or {})
        role = str(tag.get('semantic_role') or '').strip().lower()
        policy = str(metadata.get('orientation_policy') or '').strip().upper()
        explicit_id = metadata.get('rack_id') or metadata.get('shelf_id')
        declared_service = role in ('shelf_service', 'service') or bool(explicit_id) \
            or policy == 'SHELF_WIDTH_PARALLEL'

        if explicit_id:
            rack = racks.get(str(explicit_id))
            if rack is None:
                raise ValueError(f"Tag {tag.get('tag_id')} references missing rack_id {explicit_id}")
            geometry = _rack_service_geometry(tag, rack, layout)
            if geometry is None:
                raise ValueError(
                    f"Tag {tag.get('tag_id')} is not a validated service point for rack {explicit_id}")
            candidates = [(rack, geometry)]
        else:
            candidates = _rack_service_candidates(tag, layout)
            if len(candidates) > 1:
                raise ValueError(
                    f"Tag {tag.get('tag_id')} has ambiguous shelf-service geometry; specify rack_id")
            if declared_service and len(candidates) != 1:
                raise ValueError(
                    f"Tag {tag.get('tag_id')} declares shelf service but has no unique valid rack service point")
            if not candidates:
                if policy == 'SHELF_WIDTH_PARALLEL':
                    raise ValueError(f"Tag {tag.get('tag_id')} requests shelf orientation without valid rack service geometry")
                continue
            # An authored non-shelf orientation policy is meaningful input,
            # not missing metadata.  Do not silently reinterpret that Tag as
            # a shelf service point just because it happens to lie near a rack.
            if policy and policy not in ('SHELF_WIDTH_PARALLEL', 'EXPLICIT'):
                continue
            if role and role not in ('intersection', 'shelf_service', 'service'):
                continue

        rack, geometry = candidates[0]
        if policy and policy not in ('SHELF_WIDTH_PARALLEL', 'EXPLICIT') and declared_service:
            raise ValueError(
                f"Tag {tag.get('tag_id')} shelf-service geometry conflicts with orientation_policy {policy}")
        metadata['rack_id'] = geometry['rack_id']
        metadata['orientation_policy'] = 'SHELF_WIDTH_PARALLEL'
        metadata['service_face'] = geometry['service_face']
        metadata['service_standoff_m'] = geometry['service_standoff_m']
        metadata['service_aisle_id'] = geometry['aisle_id']
        service_pose = dict(metadata.get('service_pose') or {})
        service_pose.update({'frame_id': 'map', 'x': float(service_pose.get('x', tag['x'])),
                             'y': float(service_pose.get('y', tag['y']))})
        if canonical_revision is not None:
            service_pose['map_revision'] = str(int(canonical_revision))
        metadata['service_pose'] = service_pose
        tag['metadata'] = metadata
        tag['semantic_role'] = 'shelf_service'


def _rack_for_tag(tag: dict[str, Any], layout: dict[str, Any]) -> dict[str, Any] | None:
    metadata = tag.get('metadata') if isinstance(tag.get('metadata'), dict) else {}
    explicit = metadata.get('rack_id') or metadata.get('shelf_id')
    racks = layout.get('racks') or []
    if explicit:
        rack = next((item for item in racks
                     if str(item.get('id') or item.get('uuid') or '') == str(explicit)), None)
        if rack is None or _rack_service_geometry(tag, rack, layout) is None:
            raise ValueError(f"Tag {tag.get('tag_id')} has invalid rack_id/service geometry")
        return rack
    if str(tag.get('semantic_role') or '').lower() not in ('shelf_service', 'service'):
        return None
    candidates = _rack_service_candidates(tag, layout)
    if len(candidates) != 1:
        raise ValueError(f"Tag {tag.get('tag_id')} requires one unambiguous rack_id service association")
    return candidates[0][0]


def _resolve_tag_yaw(tag: dict[str, Any], layout: dict[str, Any]) -> tuple[float, str]:
    metadata = dict(tag.get('metadata') or {})
    policy = str(metadata.get('orientation_policy') or '').upper()
    if not policy:
        if _rack_for_tag(tag, layout):
            policy = 'SHELF_WIDTH_PARALLEL'
        else:
            # Legacy Tags without any lane geometry preserve their explicit
            # authored yaw. Published Tags with aisle associations are always
            # converted to a semantic cardinal lane heading.
            policy = 'LANE_FORWARD' if _aisle_at(tag, layout.get('aisles') or []) else 'EXPLICIT'
    if policy == 'EXPLICIT':
        return float(tag.get('yaw', 0.0)), policy
    if policy == 'AXIS_Y_PARALLEL':
        return math.pi / 2, policy
    if policy == 'AXIS_Y_PERPENDICULAR':
        return 0.0, policy
    if policy == 'SHELF_WIDTH_PARALLEL':
        rack = _rack_for_tag(tag, layout)
        if rack is None:
            raise ValueError(f"Tag {tag.get('tag_id')} requests shelf orientation without rack geometry")
        size = rack.get('size') or [1, 1, 1]
        long_axis_yaw = math.radians(float(rack.get('rotation') or 0.0))
        if float(size[2]) > float(size[0]):
            long_axis_yaw += math.pi / 2
        # Robot base_link Y is yaw + pi/2; align that axis with the rack's
        # actual long dimension. A rotated shelf is allowed to require a
        # non-cardinal service heading; snapping would break the geometry rule.
        yaw = long_axis_yaw - math.pi / 2
        return math.atan2(math.sin(yaw), math.cos(yaw)), policy
    if policy not in ('LANE_FORWARD', 'LANE_REVERSE'):
        raise ValueError(f"Tag {tag.get('tag_id')} has unsupported orientation policy {policy}")
    aisle = _aisle_at(tag, layout.get('aisles') or [])
    if aisle is None or len(aisle.get('centerline') or []) < 2:
        raise ValueError(f"Tag {tag.get('tag_id')} has no aisle geometry for lane orientation")
    point = (float(tag['x']), float(tag['y']))
    projected = _project(aisle['centerline'], *point)
    if projected is None:
        raise ValueError(f"Tag {tag.get('tag_id')} aisle centerline is invalid")
    along = projected[1]
    accumulated = 0.0
    direction_yaw = 0.0
    def xy(point: Any) -> tuple[float, float]:
        if isinstance(point, dict):
            return float(point['x']), float(point['y'])
        return float(point[0]), float(point[1])
    for raw_start, raw_end in zip(aisle['centerline'], aisle['centerline'][1:]):
        start, end = xy(raw_start), xy(raw_end)
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy)
        if length > 1e-9 and accumulated + length >= along - 1e-9:
            direction_yaw = math.atan2(dy, dx)
            break
        accumulated += length
    if policy == 'LANE_REVERSE' or str(aisle.get('direction') or '').lower() == 'reverse':
        direction_yaw += math.pi
    return _cardinal_yaw(direction_yaw), policy


def _edge_key(edge: dict[str, Any]) -> tuple[str, str, str]:
    return (str(edge.get('aisle_id') or ''), str(edge.get('from_tag_uuid') or ''),
            str(edge.get('to_tag_uuid') or ''))


def generate_orthogonal_edges(layout: dict[str, Any]) -> list[dict[str, Any]]:
    """Generate adjacent Tag edges from aisle association and clear geometry."""
    doc = canonicalize_layout(layout)
    tag_by_floor: dict[str, list[dict[str, Any]]] = {}
    for tag in doc.get('navigation_tags') or []:
        if tag.get('enabled', True):
            tag_by_floor.setdefault(str(tag.get('floor_id', 1)), []).append(tag)
    generated: dict[tuple[str, str, str], dict[str, Any]] = {}
    for aisle in doc.get('aisles') or []:
        aisle_id = str(aisle.get('id') or aisle.get('uuid') or '')
        centerline = aisle.get('centerline') or []
        if not aisle_id or len(centerline) < 2:
            continue
        floor_id = str(aisle.get('floor_id', aisle.get('floor', 1)))
        floor = next((item for item in doc.get('floors') or [] if str(item.get('id')) == floor_id), None)
        if floor is None:
            continue
        candidates = []
        for tag in tag_by_floor.get(floor_id, []):
            projection = _project(centerline, float(tag['x']), float(tag['y']))
            if projection is None:
                continue
            if aisle_id not in _tag_sources(tag) and projection[0] > AISLE_PROJECTION_TOLERANCE_M:
                continue
            candidates.append((projection[1], tag))
        candidates.sort(key=lambda item: (item[0], int(item[1].get('tag_id', 0))))
        for (_, first), (_, second) in zip(candidates, candidates[1:]):
            dx = float(second['x']) - float(first['x'])
            dy = float(second['y']) - float(first['y'])
            x_axis, y_axis = abs(dy) <= AXIS_TOLERANCE_M, abs(dx) <= AXIS_TOLERANCE_M
            # Exactly one axis must be constant; no diagonal and no duplicate
            # coincident node is admitted to the route graph.
            if x_axis == y_axis:
                continue
            if not _segment_is_clear(doc, floor, (float(first['x']), float(first['y'])),
                                     (float(second['x']), float(second['y']))):
                continue
            direction = str(aisle.get('direction') or 'bidirectional').lower()
            if direction not in ('forward', 'reverse', 'bidirectional'):
                direction = 'bidirectional'
            source, target = (second, first) if direction == 'reverse' else (first, second)
            distance = math.hypot(dx, dy)
            edge = {
                'uuid': f"edge-{aisle_id}-{source['uuid']}-{target['uuid']}",
                'from_tag_uuid': str(source['uuid']), 'to_tag_uuid': str(target['uuid']),
                'from_tag_id': int(source['tag_id']), 'to_tag_id': int(target['tag_id']),
                'aisle_id': aisle_id, 'floor_id': source.get('floor_id', 1),
                'distance': distance, 'cost': distance,
                'direction': direction, 'bidirectional': direction == 'bidirectional',
                'enabled': True, 'placement': 'auto', 'locked': False,
                'axis': 'X' if x_axis else 'Y', 'lane_id': aisle_id,
                'clearance_m': ROBOT_ROUTE_CLEARANCE_M,
            }
            generated[_edge_key(edge)] = edge
    return sorted(generated.values(), key=lambda edge: (edge['from_tag_id'], edge['to_tag_id'], edge['aisle_id']))


def validate_orthogonal_edges(layout: dict[str, Any], edges: list[dict[str, Any]] | None = None) -> list[str]:
    doc = canonicalize_layout(layout)
    by_uuid = {str(tag.get('uuid')): tag for tag in doc.get('navigation_tags') or []}
    errors = []
    for edge in edges if edges is not None else doc.get('navigation_edges') or []:
        source, target = by_uuid.get(str(edge.get('from_tag_uuid'))), by_uuid.get(str(edge.get('to_tag_uuid')))
        if source is None or target is None:
            errors.append(f"edge {edge.get('uuid', '?')} references a missing Tag")
            continue
        dx, dy = abs(float(source['x']) - float(target['x'])), abs(float(source['y']) - float(target['y']))
        if (dx > AXIS_TOLERANCE_M) and (dy > AXIS_TOLERANCE_M):
            errors.append(f"edge {edge.get('uuid', '?')} is diagonal ({dx:.3f}m, {dy:.3f}m)")
        elif dx <= AXIS_TOLERANCE_M and dy <= AXIS_TOLERANCE_M:
            errors.append(f"edge {edge.get('uuid', '?')} has coincident endpoints")
    return errors


def prepare_published_navigation(layout: dict[str, Any], canonical_revision: int | None = None) -> dict[str, Any]:
    """Return layout with semantic cardinal Tag poses and a generated graph."""
    doc = canonicalize_layout(layout)
    _associate_shelf_service_tags(doc, canonical_revision)
    authored_errors = validate_orthogonal_edges(doc)
    if authored_errors:
        raise ValueError('; '.join(authored_errors))
    for tag in doc.get('navigation_tags') or []:
        metadata = dict(tag.get('metadata') or {})
        yaw, policy = _resolve_tag_yaw(tag, doc)
        metadata['orientation_policy'] = policy
        if policy == 'SHELF_WIDTH_PARALLEL':
            service_pose = dict(metadata.get('service_pose') or {})
            service_pose.update({'frame_id': 'map', 'x': float(service_pose.get('x', tag['x'])),
                                 'y': float(service_pose.get('y', tag['y'])), 'yaw': yaw})
            if canonical_revision is not None:
                service_pose['map_revision'] = str(int(canonical_revision))
            metadata['service_pose'] = service_pose
        tag['metadata'] = metadata
        tag['yaw'] = yaw
    edges = generate_orthogonal_edges(doc)
    existing_manual = [edge for edge in doc.get('navigation_edges') or []
                       if str(edge.get('placement') or '').lower() == 'manual' or edge.get('locked') is True]
    if existing_manual:
        merged = {_edge_key(edge): edge for edge in edges}
        by_uuid = {str(tag.get('uuid')): tag for tag in doc.get('navigation_tags') or []}
        for edge in existing_manual:
            source = by_uuid.get(str(edge.get('from_tag_uuid')))
            target = by_uuid.get(str(edge.get('to_tag_uuid')))
            if source is None or target is None:
                continue
            dx, dy = abs(float(source['x']) - float(target['x'])), abs(float(source['y']) - float(target['y']))
            if ((dx <= AXIS_TOLERANCE_M) == (dy <= AXIS_TOLERANCE_M)):
                continue
            aisle = next((item for item in doc.get('aisles') or []
                          if str(item.get('id') or item.get('uuid')) == str(edge.get('aisle_id'))), None)
            floor_id = str(edge.get('floor_id', 1))
            floor = next((item for item in doc.get('floors') or [] if str(item.get('id')) == floor_id), None)
            if aisle is None or floor is None or not _segment_is_clear(doc, floor,
                    (float(source['x']), float(source['y'])), (float(target['x']), float(target['y']))):
                continue
            enriched = {**edge, 'axis': 'X' if dy <= AXIS_TOLERANCE_M else 'Y',
                        'distance': math.hypot(dx, dy), 'lane_id': str(edge.get('aisle_id'))}
            merged[_edge_key(enriched)] = enriched
        edges = sorted(merged.values(), key=lambda edge: (int(edge['from_tag_id']), int(edge['to_tag_id']), str(edge['aisle_id'])))
    errors = validate_orthogonal_edges(doc, edges)
    if errors:
        raise ValueError('; '.join(errors))
    doc['navigation_edges'] = edges
    revision_input = {
        'canonical_revision': int(canonical_revision) if canonical_revision is not None else doc.get('revision'),
        'tags': [{'tag_id': tag['tag_id'], 'x': tag['x'], 'y': tag['y'], 'yaw': tag['yaw'],
                  'semantic_role': tag.get('semantic_role'),
                  'rack_id': (tag.get('metadata') or {}).get('rack_id'),
                  'service_face': (tag.get('metadata') or {}).get('service_face'),
                  'service_pose': (tag.get('metadata') or {}).get('service_pose'),
                  'orientation_policy': (tag.get('metadata') or {}).get('orientation_policy')}
                 for tag in sorted(doc.get('navigation_tags') or [], key=lambda item: int(item['tag_id']))],
        'edges': [{key: edge.get(key) for key in ('from_tag_id', 'to_tag_id', 'aisle_id', 'axis', 'distance', 'direction', 'enabled')}
                  for edge in edges],
    }
    encoded = json.dumps(revision_input, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    doc['tag_graph_revision'] = hashlib.sha256(encoded).hexdigest()
    return doc
