"""The LiDAR pipeline must bound backlog so transforms remain usable."""
from pathlib import Path
import sys

import yaml
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'swerve_controller'))
from lidar_preprocessor_node import LIDAR_QOS, filter_xyz_points  # noqa: E402


def test_pointcloud_preprocessor_keeps_only_the_latest_sensor_sample():
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

    assert LIDAR_QOS.history == HistoryPolicy.KEEP_LAST
    assert LIDAR_QOS.depth == 1
    assert LIDAR_QOS.reliability == ReliabilityPolicy.BEST_EFFORT
    assert LIDAR_QOS.durability == DurabilityPolicy.VOLATILE


def test_scan_conversion_and_slam_do_not_queue_stale_sensor_frames():
    scan = yaml.safe_load(
        (ROOT / 'config/pointcloud_to_laserscan.yaml').read_text(encoding='utf-8'))
    slam = yaml.safe_load((ROOT / 'config/slam_toolbox.yaml').read_text(encoding='utf-8'))

    assert scan['pointcloud_to_laserscan']['ros__parameters']['queue_size'] == 1
    assert slam['slam_toolbox']['ros__parameters']['scan_queue_size'] == 1


def test_vectorized_cloud_filter_preserves_range_height_self_and_voxel_rules():
    points = np.array([
        (1.01, 2.01, 0.20),       # first point in voxel; retained
        (1.02, 2.02, 0.20),       # same voxel; dropped as before
        (1.06, 2.02, 0.20),       # next x voxel
        (0.00, 0.00, 0.20),       # exactly minimum range
        (0.00, 0.00, 3.01),       # outside range
        (1.00, 0.00, 1.01),       # outside height
        (0.10, 0.10, 0.10),       # inside the robot self-filter box
        (-0.01, 1.00, 0.20),      # negative voxel index uses floor
        (float('nan'), 1.00, 0.20),
        (float('inf'), 1.00, 0.20),
    ], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
    kwargs = {
        'min_range': 0.2, 'max_range': 3.0, 'voxel_size': 0.05,
        'min_height': -1.0, 'max_height': 1.0,
        'remove_floor': False, 'floor_z': -0.35, 'floor_band': 0.05,
        'self_min': (-0.65, -0.4, -0.4), 'self_max': (0.65, 0.4, 0.15),
    }
    actual = filter_xyz_points(points, **kwargs)
    expected = np.array([
        (1.01, 2.01, 0.20),
        (1.06, 2.02, 0.20),
        (0.00, 0.00, 0.20),
        (-0.01, 1.00, 0.20),
    ], dtype=np.float32)
    np.testing.assert_array_equal(actual, expected)


def test_vectorized_cloud_filter_preserves_floor_toggle_and_disables_voxels():
    points = np.array([
        (0.00, 0.00, 0.30),
        (0.01, 0.00, 0.31),
        (0.80, 0.00, -0.35),
    ], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
    kwargs = {
        'min_range': 0.0, 'max_range': 2.0, 'voxel_size': 0.0,
        'min_height': -1.0, 'max_height': 1.0,
        'remove_floor': False, 'floor_z': -0.35, 'floor_band': 0.05,
        'self_min': (-0.65, -0.4, -0.4), 'self_max': (0.65, 0.4, 0.15),
    }
    actual = filter_xyz_points(points, **kwargs)
    assert actual.shape == (3, 3)
    np.testing.assert_array_equal(actual[:2, 0], np.array([0.0, 0.01], dtype=np.float32))
    assert actual[2, 2] == np.float32(-0.35)
    kwargs['remove_floor'] = True
    actual = filter_xyz_points(points, **kwargs)
    assert actual.shape == (2, 3)
    np.testing.assert_array_equal(actual[:, 2], np.array([0.30, 0.31], dtype=np.float32))
