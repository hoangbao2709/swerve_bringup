"""Robot-scoped local map registry helpers.

Files live below the configured map artifact root and are only addressed by
opaque map IDs over the API. A registry row never exposes its host paths.
"""
from __future__ import annotations

import json
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
        result.append({key: value for key, value in row.items() if not key.startswith('_')})
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


def register_saved_map(robot_id: str, name: str, yaml_path: Path) -> dict[str, Any]:
    """Register a map saver result only after the YAML and image exist."""
    name = validate_map_name(name)
    directory = robot_map_dir(robot_id).resolve()
    yaml_path = yaml_path.resolve(strict=True)
    if not yaml_path.is_relative_to(directory) or yaml_path.suffix.lower() != '.yaml':
        raise ValueError('map saver YAML path is outside the robot map store')
    try:
        import yaml
        document = yaml.safe_load(yaml_path.read_text(encoding='utf-8')) or {}
        image_name = str(document['image'])
        resolution = float(document['resolution'])
        origin = [float(item) for item in document['origin']]
    except (ImportError, OSError, KeyError, TypeError, ValueError) as exc:
        raise ValueError('map saver output has invalid metadata') from exc
    image_path = Path(image_name)
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image_path = image_path.resolve(strict=True)
    if not image_path.is_relative_to(directory) or not image_path.is_file():
        raise ValueError('map saver image is outside the robot map store')
    if resolution <= 0 or len(origin) != 3:
        raise ValueError('map saver output has invalid resolution or origin')
    now = datetime.now(timezone.utc).isoformat()
    map_id = uuid.uuid4().hex
    row = {
        'id': map_id, 'name': name, 'robot_id': robot_id,
        'created_at': now, 'resolution': resolution, 'origin': origin,
        'revision': map_id[:12], 'frame_id': 'map',
        '_yaml': yaml_path.relative_to(directory).as_posix(),
        '_image': image_path.relative_to(directory).as_posix(),
    }
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
