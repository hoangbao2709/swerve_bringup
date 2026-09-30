import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from swerve_bridge.web_map_renderer import (
    bounded_voxel_points,
    laser_scan_xy,
    path_length,
    point_xyz,
    quaternion_rotate_xyz,
    successful_path_result,
    transform_points_xyz,
)


def test_laser_scan_converts_only_finite_in_range_samples():
    points = laser_scan_xy(
        [1.0, float('nan'), 2.0, 0.1, float('inf')],
        -math.pi / 2, math.pi / 4, 0.2, 3.0,
    )
    assert len(points) == 2
    assert math.isclose(points[0][0], 0.0, abs_tol=1e-9)
    assert math.isclose(points[0][1], -1.0, abs_tol=1e-9)
    assert math.isclose(points[1][0], 2.0, abs_tol=1e-9)
    assert math.isclose(points[1][1], 0.0, abs_tol=1e-9)


def test_point_cloud_transform_rotates_and_translates_ros_coordinates():
    half = math.pi / 4
    quaternion = (0.0, 0.0, math.sin(half), math.cos(half))
    rotated = quaternion_rotate_xyz((1, 0, 0), quaternion)
    assert all(math.isclose(value, expected, abs_tol=1e-9)
               for value, expected in zip(rotated, (0.0, 1.0, 0.0)))
    transformed = transform_points_xyz([(1, 0, 0)], (2, -1, 0.5), quaternion)[0]
    assert all(math.isclose(value, expected, abs_tol=1e-9)
               for value, expected in zip(transformed, (2.0, 0.0, 0.5)))


def test_point_cloud_reader_accepts_sensor_msgs_tuple_and_structured_rows():
    class StructuredPoint:
        dtype = type('DType', (), {'names': ('x', 'y', 'z')})()
        values = {'x': 1.25, 'y': -2.5, 'z': 0.75}

        def __getitem__(self, key):
            return self.values[key]

    assert point_xyz((1, 2, 3)) == (1.0, 2.0, 3.0)
    assert point_xyz(StructuredPoint()) == (1.25, -2.5, 0.75)


def test_nav2_preview_requires_successful_action_and_nonempty_path():
    assert successful_path_result(4, 4, 2)
    assert not successful_path_result(6, 4, 2)
    assert not successful_path_result(4, 4, 0)


def test_voxel_filter_removes_nan_height_and_range_and_respects_point_budget():
    points = [(0.2 + index * 0.1, 0, 0.1) for index in range(80)]
    points.extend([(float('nan'), 0, 0), (1, 0, 9), (40, 0, 0)])
    result = bounded_voxel_points(
        points, max_points=9, min_range=0.1, max_range=20,
        min_height=-0.2, max_height=2.0, voxel_size=0.01,
    )
    assert len(result) <= 9
    assert all(math.isfinite(value) for point in result for value in point)
    assert all(-0.2 <= z <= 2.0 for _x, _y, z in result)
    assert all(math.hypot(x, y) <= 20 for x, y, _z in result)


def test_path_length_uses_route_points_and_ignores_non_finite_vertices():
    assert math.isclose(path_length([(0, 0), (3, 4), (float('nan'), 0), (6, 8)]), 10.0)
