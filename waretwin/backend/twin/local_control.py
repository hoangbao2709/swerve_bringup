"""Robot-scoped local map registry helpers.

Files live below the configured map artifact root and are only addressed by
opaque map IDs over the API. A registry row never exposes its host paths.
"""
from __future__ import annotations

import json
import hashlib
import math
import os
import re
import threading
import uuid
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
