"""Robot-scoped local map registry helpers.

Files live below the configured map artifact root and are only addressed by
opaque map IDs over the API. A registry row never exposes its host paths.
"""
from __future__ import annotations

import json
import hashlib
import base64
import math
import os
import re
import threading
import uuid
import zlib
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from django.conf import settings

from .map_artifacts import artifact_root

_ROBOT_ID = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$')
_MAP_NAME = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$')
_registry_lock = threading.RLock()


def _pgm_dimensions(path: Path) -> tuple[int, int]:
    """Validate a map_saver 8-bit PGM and return its declared dimensions."""
    data = path.read_bytes()
    index = 0
    tokens: list[bytes] = []
    while len(tokens) < 4:
        while index < len(data):
            if data[index] in b' \t\r\n':
                index += 1
            elif data[index:index + 1] == b'#':
                newline = data.find(b'\n', index)
                if newline < 0:
                    raise ValueError('PGM comment is not terminated')
                index = newline + 1
            else:
                break
        start = index
        while index < len(data) and data[index] not in b' \t\r\n#':
            index += 1
        if start == index:
            raise ValueError('PGM header is incomplete')
        tokens.append(data[start:index])
    try:
        width, height, maximum = int(tokens[1]), int(tokens[2]), int(tokens[3])
    except ValueError as exc:
        raise ValueError('PGM dimensions are invalid') from exc
    if tokens[0] not in (b'P5', b'P2') or width <= 0 or height <= 0 or maximum != 255:
        raise ValueError('Nav2 map image must be an 8-bit P2 or P5 PGM')
    if tokens[0] == b'P5':
        if index >= len(data) or data[index] not in b' \t\r\n':
            raise ValueError('PGM header has no raster delimiter')
        delimiter = data[index]
        index += 1
        if delimiter == 13 and index < len(data) and data[index] == 10:
            index += 1
        if len(data) - index != width * height:
            raise ValueError('PGM raster length does not match its dimensions')
    else:
        values: list[int] = []
        while index < len(data):
            while index < len(data) and data[index] in b' \t\r\n':
                index += 1
            if index >= len(data):
                break
            if data[index:index + 1] == b'#':
                newline = data.find(b'\n', index)
                if newline < 0:
                    break
                index = newline + 1
                continue
            start = index
            while index < len(data) and data[index] not in b' \t\r\n#':
                index += 1
            try:
                value = int(data[start:index])
            except ValueError as exc:
                raise ValueError('PGM raster contains an invalid pixel') from exc
            if not 0 <= value <= 255:
                raise ValueError('PGM pixel value is outside 0-255')
            values.append(value)
        if len(values) != width * height:
            raise ValueError('PGM raster length does not match its dimensions')
    return width, height


def robot_map_dir(robot_id: str) -> Path:
    if not _ROBOT_ID.fullmatch(str(robot_id or '')):
        raise ValueError('invalid robot_id')
    root = (Path(artifact_root()) / 'local_robot_maps').resolve()
    path = (root / robot_id).resolve()
    if not path.is_relative_to(root):
        raise ValueError('invalid robot map directory')
    return path


def validate_map_name(value: Any) -> str:
    name = str(value or '').strip()
    if not _MAP_NAME.fullmatch(name):
        raise ValueError('map name must use 1-64 letters, numbers, underscores, or hyphens')
    return name


def _registry_path(robot_id: str) -> Path:
    return robot_map_dir(robot_id) / 'registry.json'


def list_robot_maps(robot_id: str) -> list[dict[str, Any]]:
    path = _registry_path(robot_id)
    try:
        rows = json.loads(path.read_text(encoding='utf-8'))
    except FileNotFoundError:
        return []
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f'map registry cannot be read: {type(exc).__name__}') from exc
    if not isinstance(rows, list):
        raise ValueError('map registry has an invalid format')
    result = []
    for row in rows:
        if not isinstance(row, dict):
            continue
        yaml_path = (robot_map_dir(robot_id) / str(row.get('_yaml', ''))).resolve()
        image_path = (robot_map_dir(robot_id) / str(row.get('_image', ''))).resolve()
        base = robot_map_dir(robot_id).resolve()
        if not yaml_path.is_relative_to(base) or not image_path.is_relative_to(base):
            continue
        if not yaml_path.is_file() or not image_path.is_file():
            continue
        public = {key: value for key, value in row.items() if not key.startswith('_')}
        state = row.get('slam_session_state')
        if isinstance(state, dict) and state.get('status') == 'AVAILABLE':
            posegraph = (robot_map_dir(robot_id) / str(row.get('_slam_posegraph', ''))).resolve()
            session_data = (robot_map_dir(robot_id) / str(row.get('_slam_data', ''))).resolve()
            root = robot_map_dir(robot_id).resolve()
            try:
                session_files_valid = (
                    posegraph.is_relative_to(root) and session_data.is_relative_to(root)
                    and posegraph.is_file() and session_data.is_file()
                    and posegraph.stat().st_size > 0 and session_data.stat().st_size > 0
                )
            except OSError:
                session_files_valid = False
            if not session_files_valid:
                public['slam_session_state'] = {
                    'status': 'MISSING', 'engine': 'SLAM_TOOLBOX',
                    'artifact_id': state.get('artifact_id'),
                }
        result.append(public)
    return result


def get_robot_map(robot_id: str, map_id: str) -> tuple[dict[str, Any], Path, Path]:
    for row in list_robot_maps(robot_id):
        if row.get('id') != map_id:
            continue
        directory = robot_map_dir(robot_id).resolve()
        raw_rows = json.loads(_registry_path(robot_id).read_text(encoding='utf-8'))
        raw = next((item for item in raw_rows if item.get('id') == map_id), None)
        if not raw:
            break
        yaml_path = (directory / str(raw['_yaml'])).resolve()
        image_path = (directory / str(raw['_image'])).resolve()
        if not yaml_path.is_relative_to(directory) or not image_path.is_relative_to(directory):
            break
        return row, yaml_path, image_path
    raise FileNotFoundError('map not found for this robot')


def validate_robot_map_artifacts(robot_id: str, map_id: str) -> tuple[dict[str, Any], Path, Path]:
    """Validate the registered YAML/PGM pair immediately before Nav2 loads it."""
    record, yaml_path, image_path = get_robot_map(robot_id, map_id)
    if (record.get('map_kind') != 'SAVED_LOCAL_MAP'
            or record.get('canonical_map_promoted') is not False
            or record.get('frame_id') != 'map'):
        raise ValueError('selected map is not a robot-local saved navigation map')
    artifacts = record.get('navigation_artifacts')
    if not isinstance(artifacts, dict) or artifacts.get('yaml') is not True or artifacts.get('image') is not True:
        raise ValueError('selected saved map registry entry has incomplete navigation artifacts')

    try:
        import yaml
    except ImportError as exc:
        raise ValueError('YAML support is unavailable for saved map validation') from exc
    try:
        document = yaml.safe_load(yaml_path.read_text(encoding='utf-8')) or {}
        image_reference = Path(str(document['image']))
        resolved_image = (image_reference if image_reference.is_absolute()
                          else yaml_path.parent / image_reference).resolve(strict=True)
        resolution = float(document['resolution'])
        origin = [float(value) for value in document['origin']]
        negate = int(document['negate'])
        occupied_threshold = float(document['occupied_thresh'])
        free_threshold = float(document['free_thresh'])
        width, height = _pgm_dimensions(image_path)
        image_hash = hashlib.sha256(image_path.read_bytes()).hexdigest()
        registered_origin = [float(value) for value in record['origin']]
        registered_resolution = float(record['resolution'])
        registered_width, registered_height = int(record['width']), int(record['height'])
    except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError,
            OverflowError) as exc:
        raise ValueError(f'selected saved map artifacts are invalid: {type(exc).__name__}') from exc

    root = robot_map_dir(robot_id).resolve(strict=True)
    if (not yaml_path.is_relative_to(root) or not resolved_image.is_relative_to(root)
            or resolved_image != image_path.resolve(strict=True)):
        raise ValueError('selected map YAML does not reference its registered robot-local PGM')
    if (not math.isfinite(resolution) or resolution <= 0.0
            or len(origin) != 3 or not all(math.isfinite(value) for value in origin)
            or negate not in (0, 1)
            or not math.isfinite(occupied_threshold) or not math.isfinite(free_threshold)
            or not 0.0 <= free_threshold < occupied_threshold <= 1.0
            or not math.isclose(resolution, registered_resolution, rel_tol=0.0, abs_tol=1e-6)
            or any(not math.isclose(value, registered, rel_tol=0.0, abs_tol=1e-6)
                   for value, registered in zip(origin, registered_origin))
            or width != registered_width or height != registered_height):
        raise ValueError('selected saved map YAML, PGM dimensions, and registry geometry do not match')
    registered_hash = str(record.get('image_sha256') or '').lower()
    if not registered_hash or image_hash != registered_hash:
        raise ValueError('selected saved map PGM hash does not match its registry entry')
    return record, yaml_path, image_path


def get_robot_slam_session(robot_id: str, map_id: str) -> tuple[dict[str, Any], Path, Path, Path]:
    """Resolve a registered SLAM Toolbox session without exposing its paths in the API."""
    record, _yaml_path, _image_path = get_robot_map(robot_id, map_id)
    state = record.get('slam_session_state') or {}
    if (str(state.get('engine') or '').upper() != 'SLAM_TOOLBOX'
            or str(state.get('status') or '').upper() != 'AVAILABLE'):
        raise FileNotFoundError('saved map has no available SLAM Toolbox session')
    directory = robot_map_dir(robot_id).resolve(strict=True)
    raw_rows = json.loads(_registry_path(robot_id).read_text(encoding='utf-8'))
    raw = next((item for item in raw_rows if item.get('id') == map_id), None)
    if not isinstance(raw, dict):
        raise FileNotFoundError('saved SLAM session registry entry is missing')
    posegraph = (directory / str(raw.get('_slam_posegraph') or '')).resolve(strict=True)
    session_data = (directory / str(raw.get('_slam_data') or '')).resolve(strict=True)
    if (not posegraph.is_relative_to(directory) or not session_data.is_relative_to(directory)
            or posegraph.suffix != '.posegraph' or session_data.suffix != '.data'
            or posegraph.with_suffix('.data') != session_data
            or not posegraph.is_file() or not session_data.is_file()
            or posegraph.stat().st_size <= 0 or session_data.stat().st_size <= 0):
        raise FileNotFoundError('saved SLAM Toolbox session artifacts are incomplete or invalid')
    return record, posegraph.with_suffix(''), posegraph, session_data


def _read_pgm_pixels(path: Path) -> tuple[int, int, bytes]:
    data = path.read_bytes()
    index = 0
    tokens: list[bytes] = []
    while len(tokens) < 4:
        while index < len(data):
            if data[index] in b' \t\r\n':
                index += 1
            elif data[index:index + 1] == b'#':
                newline = data.find(b'\n', index)
                if newline < 0:
                    raise ValueError('PGM comment is not terminated')
                index = newline + 1
            else:
                break
        start = index
        while index < len(data) and data[index] not in b' \t\r\n#':
            index += 1
        if start == index:
            raise ValueError('PGM header is incomplete')
        tokens.append(data[start:index])
    try:
        width, height, maximum = int(tokens[1]), int(tokens[2]), int(tokens[3])
    except ValueError as exc:
        raise ValueError('PGM dimensions are invalid') from exc
    if tokens[0] not in (b'P5', b'P2') or width <= 0 or height <= 0 or maximum != 255:
        raise ValueError('SLAM resume comparison requires an 8-bit P2 or P5 PGM')
    if tokens[0] == b'P5':
        if index >= len(data) or data[index] not in b' \t\r\n':
            raise ValueError('PGM header has no raster delimiter')
        delimiter = data[index]
        index += 1
        if delimiter == 13 and index < len(data) and data[index] == 10:
            index += 1
        pixels = data[index:]
        if len(pixels) != width * height:
            raise ValueError('PGM raster length does not match its dimensions')
        return width, height, pixels

    values: list[int] = []
    while index < len(data):
        while index < len(data) and data[index] in b' \t\r\n':
            index += 1
        if index >= len(data):
            break
        if data[index:index + 1] == b'#':
            newline = data.find(b'\n', index)
            if newline < 0:
                break
            index = newline + 1
            continue
        start = index
        while index < len(data) and data[index] not in b' \t\r\n#':
            index += 1
        try:
            value = int(data[start:index])
        except ValueError as exc:
            raise ValueError('PGM raster contains an invalid pixel') from exc
        if not 0 <= value <= 255:
            raise ValueError('PGM pixel value is outside 0-255')
        values.append(value)
    if len(values) != width * height:
        raise ValueError('PGM raster length does not match its dimensions')
    return width, height, bytes(values)


def slam_map_restoration_evidence(record: dict[str, Any], yaml_path: Path,
                                  image_path: Path, map_data: dict[str, Any]) -> dict[str, Any]:
    """Compare the live SLAM OccupancyGrid against known cells in its saved image.

    SLAM Toolbox's deserialize service response is empty in the installed ROS
    package, so service/startup acknowledgement is not treated as restoration
    proof. This compares the map's world-coordinate cell classes instead.
    """
    evidence: dict[str, Any] = {
        'passed': False, 'saved_map_id': record.get('id'),
        'saved_image_sha256': record.get('image_sha256'),
        'live_mapping_session_id': map_data.get('mapping_session_id'),
    }
    try:
        import yaml

        document = yaml.safe_load(yaml_path.read_text(encoding='utf-8')) or {}
        saved_width, saved_height, pixels = _read_pgm_pixels(image_path)
        saved_resolution = float(document['resolution'])
        saved_origin = [float(value) for value in document['origin']]
        negate = int(document['negate'])
        occupied_thresh = float(document['occupied_thresh'])
        free_thresh = float(document['free_thresh'])
        width, height = int(map_data['width']), int(map_data['height'])
        resolution = float(map_data['resolution'])
        origin = map_data['origin']
        origin_x, origin_y, origin_yaw = (float(origin[key]) for key in ('x', 'y', 'yaw'))
        if (map_data.get('map_source') != 'SLAM_TOOLBOX'
                or not map_data.get('mapping_session_id')
                or width <= 0 or height <= 0 or width * height > 4_000_000
                or resolution <= 0 or saved_resolution <= 0
                or saved_width != int(record['width']) or saved_height != int(record['height'])
                or len(saved_origin) != 3 or negate not in (0, 1)
                or not all(math.isfinite(value) for value in
                           (saved_resolution, *saved_origin, resolution,
                            origin_x, origin_y, origin_yaw, occupied_thresh, free_thresh))
                or not math.isclose(saved_resolution, resolution, rel_tol=0.0, abs_tol=1e-6)
                or abs(math.sin(origin_yaw - saved_origin[2])) > 1e-5):
            evidence['reason'] = 'live map identity or geometry is not compatible with the saved map'
            return evidence
        compressed = map_data.get('data_zlib_base64')
        if map_data.get('data_encoding') != 'zlib-base64-offset1' or not isinstance(compressed, str):
            evidence['reason'] = 'live OccupancyGrid payload is missing or unsupported'
            return evidence
        occupancy_bytes = zlib.decompress(base64.b64decode(compressed, validate=True))
        if len(occupancy_bytes) != width * height:
            evidence['reason'] = 'live OccupancyGrid payload length does not match its geometry'
            return evidence

        saved_yaw = saved_origin[2]
        saved_cos, saved_sin = math.cos(saved_yaw), math.sin(saved_yaw)
        live_cos, live_sin = math.cos(origin_yaw), math.sin(origin_yaw)
        saved_known = compared = live_known = matched = 0
        saved_occupied = matched_occupied = 0
        occupied_match_distances: list[float] = []
        missing_occupied: list[dict[str, Any]] = []
        # Two equal-resolution square grids can have arbitrary phase offsets.
        # The nearest phase-equivalent centers differ by at most half a cell
        # on each axis, so their Euclidean correspondence bound is the half
        # diagonal: sqrt((r/2)^2 + (r/2)^2) = r/sqrt(2).
        raster_tolerance = math.sqrt(2.0) * max(saved_resolution, resolution) / 2.0
        for image_y in range(saved_height):
            map_y = saved_height - image_y - 1
            row = image_y * saved_width
            for map_x in range(saved_width):
                pixel = pixels[row + map_x]
                # SLAM Toolbox's trinary map_saver writes its unknown cells as
                # gray 205. Given the YAML thresholds that shade can otherwise
                # be misclassified as free by a generic image threshold test.
                if pixel == 205:
                    continue
                probability = (pixel if negate else 255 - pixel) / 255.0
                if probability > occupied_thresh:
                    expected = 1
                elif probability < free_thresh:
                    expected = 0
                else:
                    continue
                saved_known += 1
                world_x = saved_origin[0] + saved_cos * ((map_x + 0.5) * saved_resolution) - saved_sin * ((map_y + 0.5) * saved_resolution)
                world_y = saved_origin[1] + saved_sin * ((map_x + 0.5) * saved_resolution) + saved_cos * ((map_y + 0.5) * saved_resolution)
                dx, dy = world_x - origin_x, world_y - origin_y
                local_x = live_cos * dx + live_sin * dy
                local_y = -live_sin * dx + live_cos * dy
                containing_x = math.floor(local_x / resolution)
                containing_y = math.floor(local_y / resolution)
                candidates: list[tuple[float, int, int, int, int]] = []
                # A half-diagonal search can reach only the containing cell or
                # one of its immediate neighbors on either axis.
                for cell_y in range(containing_y - 1, containing_y + 2):
                    if not 0 <= cell_y < height:
                        continue
                    for cell_x in range(containing_x - 1, containing_x + 2):
                        if not 0 <= cell_x < width:
                            continue
                        live_local_x = (cell_x + 0.5) * resolution
                        live_local_y = (cell_y + 0.5) * resolution
                        live_world_x = origin_x + live_cos * live_local_x - live_sin * live_local_y
                        live_world_y = origin_y + live_sin * live_local_x + live_cos * live_local_y
                        distance = math.hypot(live_world_x - world_x, live_world_y - world_y)
                        if distance <= raster_tolerance:
                            value = occupancy_bytes[cell_y * width + cell_x] - 1
                            probability = value / 100.0 if value >= 0 else -1.0
                            actual = (1 if probability > occupied_thresh else
                                      0 if probability >= 0 and probability < free_thresh else -1)
                            candidates.append((distance, cell_x, cell_y, value, actual))
                if not candidates:
                    if expected == 1:
                        saved_occupied += 1
                        missing_occupied.append({
                            'saved_row': map_y, 'saved_col': map_x,
                            'world_x': world_x, 'world_y': world_y,
                        })
                    continue
                compared += 1
                nearest = min(candidates)
                if any(value >= 0 for _, _, _, value, _ in candidates):
                    live_known += 1
                if any(actual == expected for _, _, _, _, actual in candidates):
                    matched += 1
                if expected == 1:
                    saved_occupied += 1
                    occupied_matches = [candidate for candidate in candidates if candidate[4] == 1]
                    if occupied_matches:
                        matched_occupied += 1
                        occupied_match_distances.append(min(occupied_matches)[0])
                    else:
                        missing_occupied.append({
                            'saved_row': map_y, 'saved_col': map_x,
                            'world_x': world_x, 'world_y': world_y,
                            'nearest_live_value': nearest[3],
                            'nearest_live_class': (
                                'UNKNOWN' if nearest[3] < 0 else
                                'FREE' if nearest[4] == 0 else 'AMBIGUOUS'),
                            'nearest_center_distance_m': nearest[0],
                        })

        coverage = compared / saved_known if saved_known else 0.0
        known_overlap = live_known / compared if compared else 0.0
        agreement = matched / live_known if live_known else 0.0
        expected_known = int(record.get('known_cells') or saved_known)
        live_known_cells = int(map_data.get('known_cells') or 0)
        evidence.update({
            'saved_dimensions': [saved_width, saved_height],
            'live_dimensions': [width, height],
            'saved_known_cells': saved_known,
            'registered_known_cells': expected_known,
            'live_known_cells': live_known_cells,
            'covered_saved_cells': compared,
            'known_overlap_cells': live_known,
            'matched_saved_cells': matched,
            'saved_coverage_ratio': coverage,
            'known_overlap_ratio': known_overlap,
            'cell_class_agreement_ratio': agreement,
            'raster_correspondence_tolerance_m': raster_tolerance,
            'raster_correspondence_tolerance_derivation': (
                'equal-resolution square-grid phase: sqrt((resolution/2)^2 + '
                '(resolution/2)^2) = resolution/sqrt(2)'),
            'saved_occupied_cells': saved_occupied,
            'matched_occupied_cells': matched_occupied,
            'missing_occupied_cells': saved_occupied - matched_occupied,
            'max_occupied_match_distance_m': max(occupied_match_distances, default=None),
            'missing_occupied_examples': missing_occupied[:25],
            'live_grid_sha256': hashlib.sha256(
                f'{width}:{height}:{resolution:.9f}:{origin_x:.6f}:{origin_y:.6f}:{origin_yaw:.6f}'.encode()
                + occupancy_bytes).hexdigest(),
        })
        evidence['passed'] = bool(
            saved_known > 0 and live_known_cells >= expected_known * 0.9
            and coverage >= 0.9 and known_overlap >= 0.9 and agreement >= 0.9
            and saved_occupied == matched_occupied)
        if not evidence['passed']:
            evidence['reason'] = (
                'live SLAM map does not preserve saved-map world-space geometry '
                'or one or more saved occupied cells have no occupied correspondence')
        return evidence
    except Exception as exc:
        evidence['reason'] = f'map restoration comparison failed: {type(exc).__name__}'
        return evidence


def register_saved_map(robot_id: str, name: str, yaml_path: Path, *,
                       mapping_metadata: dict[str, Any] | None = None,
                       slam_session: dict[str, Any] | None = None) -> dict[str, Any]:
    """Register a map saver result only after the YAML and image exist."""
    name = validate_map_name(name)
    directory = robot_map_dir(robot_id).resolve()
    yaml_path = yaml_path.resolve(strict=True)
    if not yaml_path.is_relative_to(directory) or yaml_path.suffix.lower() != '.yaml':
        raise ValueError('map saver YAML path is outside the robot map store')
    try:
        import yaml
    except ImportError as exc:
        raise ValueError('map saver output has invalid metadata') from exc
    try:
        document = yaml.safe_load(yaml_path.read_text(encoding='utf-8')) or {}
        image_name = str(document['image'])
        resolution = float(document['resolution'])
        origin = [float(item) for item in document['origin']]
        negate = int(document['negate'])
        occupied_thresh = float(document['occupied_thresh'])
        free_thresh = float(document['free_thresh'])
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise ValueError('map saver output has invalid metadata') from exc
    image_path = Path(image_name)
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image_path = image_path.resolve(strict=True)
    if not image_path.is_relative_to(directory) or not image_path.is_file():
        raise ValueError('map saver image is outside the robot map store')
    if (not math.isfinite(resolution) or resolution <= 0 or len(origin) != 3
            or not all(math.isfinite(item) for item in origin)
            or negate not in (0, 1)
            or not math.isfinite(occupied_thresh) or not math.isfinite(free_thresh)
            or not 0.0 <= free_thresh < occupied_thresh <= 1.0):
        raise ValueError('map saver output has invalid resolution, origin, or occupancy thresholds')
    if image_path.suffix.lower() != '.pgm':
        raise ValueError('map saver image must be a supported PGM file')
    width, height = _pgm_dimensions(image_path)
    if mapping_metadata:
        try:
            map_width = int(mapping_metadata['width'])
            map_height = int(mapping_metadata['height'])
            map_resolution = float(mapping_metadata['resolution'])
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            raise ValueError('saved map metadata does not match the current SLAM map') from exc
        if (map_width != width or map_height != height or not math.isfinite(map_resolution)
                or not math.isclose(map_resolution, resolution, rel_tol=0.0, abs_tol=1e-6)):
            raise ValueError('saved map artifacts do not match the current SLAM map geometry')
    now = datetime.now(timezone.utc).isoformat()
    map_id = uuid.uuid4().hex
    mapping_metadata = mapping_metadata if isinstance(mapping_metadata, dict) else {}
    total_cells = width * height

    def cell_count(field):
        value = mapping_metadata.get(field)
        try:
            value = int(value)
        except (TypeError, ValueError, OverflowError):
            return None
        return value if 0 <= value <= total_cells else None

    known_cells = cell_count('known_cells')
    unknown_cells = cell_count('unknown_cells')
    free_cells = cell_count('free_cells')
    occupied_cells = cell_count('occupied_cells')
    if known_cells is not None and unknown_cells is None:
        unknown_cells = total_cells - known_cells
    try:
        resolution_for_area = float(mapping_metadata.get('resolution', resolution))
        explored_area = (float(mapping_metadata.get('explored_area_m2'))
                         if mapping_metadata.get('explored_area_m2') is not None
                         else known_cells * resolution_for_area ** 2
                         if known_cells is not None else None)
    except (TypeError, ValueError, OverflowError):
        explored_area = None
    if explored_area is not None and (not math.isfinite(explored_area) or explored_area < 0):
        explored_area = None

    session_posegraph = session_data = None
    session_metadata = {'status': 'NOT_SAVED', 'engine': 'SLAM_TOOLBOX', 'artifact_id': None}
    if slam_session is not None:
        if str(slam_session.get('engine') or '').upper() != 'SLAM_TOOLBOX' or str(slam_session.get('status') or '').upper() != 'AVAILABLE':
            raise ValueError('SLAM Toolbox session state was not confirmed')
        session_posegraph = Path(str(slam_session.get('posegraph_path') or '')).resolve(strict=True)
        session_data = Path(str(slam_session.get('data_path') or '')).resolve(strict=True)
        if (not session_posegraph.is_relative_to(directory) or not session_data.is_relative_to(directory)
                or session_posegraph.suffix != '.posegraph' or session_data.suffix != '.data'
                or not session_posegraph.is_file() or not session_data.is_file()
                or session_posegraph.stat().st_size <= 0 or session_data.stat().st_size <= 0):
            raise ValueError('SLAM Toolbox session artifacts are invalid or outside the local map store')
        artifact_id = uuid.uuid4().hex
        session_metadata = {'status': 'AVAILABLE', 'engine': 'SLAM_TOOLBOX', 'artifact_id': artifact_id}

    row = {
        'id': map_id, 'map_id': map_id, 'name': name, 'robot_id': robot_id,
        'created_at': now, 'resolution': resolution, 'origin': origin,
        'revision': map_id[:12], 'frame_id': 'map', 'width': width, 'height': height,
        'map_kind': 'SAVED_LOCAL_MAP', 'canonical_map_promoted': False,
        'known_cells': known_cells, 'unknown_cells': unknown_cells,
        'free_cells': free_cells, 'occupied_cells': occupied_cells,
        'explored_area_m2': explored_area,
        'source_mapping_session_id': str(mapping_metadata.get('mapping_session_id') or '') or None,
        'source_map_version': mapping_metadata.get('map_version'),
        'navigation_artifacts': {'yaml': True, 'image': True, 'image_format': image_path.suffix.lower()},
        'slam_session_state': session_metadata,
        'image_sha256': hashlib.sha256(image_path.read_bytes()).hexdigest(),
        '_yaml': yaml_path.relative_to(directory).as_posix(),
        '_image': image_path.relative_to(directory).as_posix(),
    }
    if session_posegraph is not None and session_data is not None:
        row['_slam_posegraph'] = session_posegraph.relative_to(directory).as_posix()
        row['_slam_data'] = session_data.relative_to(directory).as_posix()
    registry_path = _registry_path(robot_id)
    with _registry_lock:
        try:
            rows = json.loads(registry_path.read_text(encoding='utf-8'))
        except FileNotFoundError:
            rows = []
        if any(item.get('name') == name for item in rows):
            raise FileExistsError('a map with this name already exists for this robot')
        rows.append(row)
        registry_path.parent.mkdir(parents=True, exist_ok=True)
        temp = registry_path.with_suffix(f'.{uuid.uuid4().hex}.tmp')
        try:
            with temp.open('w', encoding='utf-8') as stream:
                json.dump(rows, stream, ensure_ascii=False, indent=2)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp, registry_path)
        finally:
            temp.unlink(missing_ok=True)
    return {key: value for key, value in row.items() if not key.startswith('_')}


def map_output_prefix(robot_id: str, name: str) -> Path:
    name = validate_map_name(name)
    directory = robot_map_dir(robot_id).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    return (directory / f'{name}').resolve()


def map_yaml_path(robot_id: str, map_id: str) -> Path:
    _row, yaml_path, _image_path = get_robot_map(robot_id, map_id)
    return yaml_path
