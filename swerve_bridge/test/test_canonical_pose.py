import hashlib
import json
import math
from types import SimpleNamespace as NS

import pytest
from swerve_bridge.canonical_pose import (GazeboCanonicalAlignment,
                                          interpolate_canonical_pose_at)


def pose(x, y, yaw=0):
    return NS(position=NS(x=x, y=y, z=0), orientation=NS(x=0, y=0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)))


@pytest.fixture
def bundle(tmp_path):
    world = tmp_path / 'gazebo' / 'warehouse.world'
    world.parent.mkdir()
    anchors = [(5, 0, 0), (10, 5, math.pi/2), (5, 10, math.pi), (0, 5, -math.pi/2)]
    world.write_text('<sdf><world>' + ''.join(
        f'<model name="floor_1_boundary_wall_{i}"><static>true</static><pose>{x} {y} 0 0 0 {yaw}</pose></model>'
        for i, (x, y, yaw) in enumerate(anchors)) + '</world></sdf>')
    floor = dict(id=1, boundary=[[0, 0], [10, 0], [10, 10], [0, 10]], holes=[])
    (tmp_path / 'canonical.json').write_text(json.dumps(dict(revision=21, frame_id='map', floors=[floor],
                                      origin=dict(x=0, y=0), width=10, height=10)))
    (world.parent / 'manifest.json').write_text(json.dumps(dict(floors=[floor])))
    artifacts = dict(gazebo_world='gazebo/warehouse.world', canonical_map='canonical.json', gazebo_manifest='gazebo/manifest.json')
    (tmp_path / 'manifest.json').write_text(json.dumps(dict(revision=21, frame_id='map', units='m',
        robots=[dict(id='R01')], artifacts=artifacts, gazebo_bounds=dict(min_x=0, min_y=0, max_x=10, max_y=10),
        sha256={p: hashlib.sha256((tmp_path / p).read_bytes()).hexdigest() for p in artifacts.values()})))
    return world


def test_live_world_pose_and_alignment_fail_closed(bundle):
    alignment = GazeboCanonicalAlignment(bundle, 'R01', 'robot')
    states = NS(name=['floor_1_boundary_wall_0', 'floor_1_boundary_wall_1', 'robot'],
                pose=[pose(5, 0), pose(10, 5, math.pi/2), pose(6.2, 4.3, .7)])
    result = alignment.pose(states, 'now', source_timestamp_s=21.25)
    assert (result['x'], result['y'], result['yaw']) == pytest.approx((6.2, 4.3, .7))
    assert result['transform_source'] == 'VALIDATED_CANONICAL_WORLD_BUNDLE'
    assert result['source_timestamp_s'] == pytest.approx(21.25)
    states.pose[0] = pose(5.1, 0)
    assert alignment.pose(states, 'now') is None
    states.name = ['robot']
    states.pose = [pose(6.2, 4.3)]
    assert alignment.pose(states, 'now') is None


def test_tampered_world_is_not_a_canonical_transform(bundle):
    bundle.write_text(bundle.read_text().replace('10 5', '11 5'))
    with pytest.raises(ValueError, match='hash mismatch'):
        GazeboCanonicalAlignment(bundle, 'R01', 'robot')


def test_canonical_pose_interpolates_at_active_tf_acquisition_time():
    first = {'x': 1.0, 'y': 2.0, 'z': 0.0, 'yaw': math.radians(170),
             'map_id': 'CANONICAL', 'map_revision': '23'}
    second = {'x': 3.0, 'y': 4.0, 'z': 0.2, 'yaw': math.radians(-170),
              'map_id': 'CANONICAL', 'map_revision': '23'}

    pose = interpolate_canonical_pose_at(
        [(10.0, first), (10.2, second)], 10.1)

    assert pose is not None
    assert (pose['x'], pose['y'], pose['z']) == pytest.approx((2.0, 3.0, 0.1))
    assert abs(abs(pose['yaw']) - math.pi) < 1e-9
    assert pose['source_timestamp_s'] == pytest.approx(10.1)
    assert pose['map_id'] == 'CANONICAL'
    assert pose['map_revision'] == '23'


@pytest.mark.parametrize('samples,stamp', [
    ([(10.0, {'x': 0.0, 'y': 0.0, 'yaw': 0.0})], 10.1),
    ([(10.0, {'x': 0.0, 'y': 0.0, 'yaw': 0.0}),
      (10.4, {'x': 1.0, 'y': 0.0, 'yaw': 0.0})], 10.2),
])
def test_canonical_pose_interpolation_fails_closed_without_close_bracket(samples, stamp):
    assert interpolate_canonical_pose_at(samples, stamp, max_gap_s=0.25) is None
