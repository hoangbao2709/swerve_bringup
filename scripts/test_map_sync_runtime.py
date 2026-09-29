import hashlib
import json
from pathlib import Path

import pytest

from ros_stack_supervisor import command_for_revision, verify_bundle


def _bundle(root: Path, revision: int = 12):
    files = {
        'canonical_map.json': json.dumps({
            'frame_id': 'map', 'revision': revision, 'origin': {'x': 0, 'y': 0},
            'width': 1, 'height': 1,
            'floors': [{'id': 'F1', 'boundary': [[0, 0], [1, 0], [1, 1], [0, 1]], 'holes': []}],
        }),
        'gazebo/warehouse.world': '<sdf><world name="warehouse"><plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"/></world></sdf>',
        'gazebo/manifest.json': json.dumps({
            'floors': [{'id': 'F1', 'boundary': [[0, 0], [1, 0], [1, 1], [0, 1]], 'holes': []}],
        }),
        'datamatrix_map.yaml': 'frame_id: map\n',
        'tag_graph.yaml': 'frame_id: map\n',
        'nav2/warehouse_F1.yaml': 'image: warehouse_F1.pgm\nresolution: 1\norigin: [0, 0, 0]\nframe_id: map\n',
        'nav2/warehouse_F1.pgm': 'P5\n1 1\n255\n\xfe',
    }
    for relative, value in files.items():
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(value if isinstance(value, bytes) else value.encode())
    manifest = {
        'revision': revision, 'frame_id': 'map', 'units': 'm',
        'artifacts': {
            'canonical_map': 'canonical_map.json', 'gazebo_world': 'gazebo/warehouse.world',
            'gazebo_manifest': 'gazebo/manifest.json',
            'datamatrix_map': 'datamatrix_map.yaml', 'tag_graph': 'tag_graph.yaml',
            'nav2_map': 'nav2/warehouse_F1.yaml',
        },
        'gazebo_bounds': {'min_x': 0, 'min_y': 0, 'max_x': 1, 'max_y': 1},
        'nav2_maps': {'F1': 'nav2/warehouse_F1.yaml'},
        'nav2_bounds': {'F1': {'width': 1, 'height': 1, 'resolution': 1, 'origin': [0, 0, 0]}},
        'robots': [{'id': 'R02', 'floor_id': 'F1', 'pose': [0.5, 0.5, 0.1, 0.5]}],
        'sha256': {name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in files},
    }
    (root / 'manifest.json').write_text(json.dumps(manifest))


def test_verified_bundle_selects_world_nav2_tags_and_robot_spawn(tmp_path):
    _bundle(tmp_path)
    manifest, selected = verify_bundle(tmp_path, 12, 'R02')
    assert manifest['frame_id'] == 'map'
    assert selected['map'].endswith('nav2/warehouse_F1.yaml')
    assert selected['spawn'] == [0.5, 0.5, 0.1, 0.5]
    command = command_for_revision(
        ['ros2', 'launch', 'system.launch.py', 'world:=old', 'map_file:=old-map', 'mode:=navigation'],
        tmp_path, 12, 'R02',
    )
    assert 'world:=old' not in command
    assert 'map_file:=old-map' not in command
    assert any(arg.endswith('gazebo/warehouse.world') for arg in command)
    assert any(arg.endswith('nav2/warehouse_F1.yaml') for arg in command)


def test_bundle_revision_and_hash_mismatch_fail_closed(tmp_path):
    _bundle(tmp_path)
    with pytest.raises(ValueError, match='revision'):
        verify_bundle(tmp_path, 13, 'R02')
    (tmp_path / 'gazebo/warehouse.world').write_text('<sdf>mutated</sdf>')
    with pytest.raises(ValueError, match='hash mismatch'):
        verify_bundle(tmp_path, 12, 'R02')


def test_bundle_rejects_gazebo_bounds_that_disagree_with_canonical_map(tmp_path):
    _bundle(tmp_path)
    manifest = json.loads((tmp_path / 'manifest.json').read_text())
    manifest['gazebo_bounds']['max_x'] = 2
    (tmp_path / 'manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='Gazebo bounds'):
        verify_bundle(tmp_path, 12, 'R02')
