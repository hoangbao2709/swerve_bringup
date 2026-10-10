import math
import unittest
from types import SimpleNamespace

from swerve_bridge.coordinates import (is_small_future_tf_skew, pose_from_transform,
                                       is_fresh_tf_sample, quaternion_yaw,
                                       rotate_translate_xy)


class CoordinateConversionTest(unittest.TestCase):
    def test_quaternion_yaw_is_ros_ccw_radians(self):
        half = math.pi / 4.0
        self.assertAlmostEqual(quaternion_yaw((0.0, 0.0, math.sin(half), math.cos(half))), math.pi / 2.0)

    def test_point_transform_applies_rotation_and_translation(self):
        half = math.pi / 4.0
        x, y = rotate_translate_xy(
            1.0, 0.0, (2.0, -3.0, 0.0),
            (0.0, 0.0, math.sin(half), math.cos(half)),
        )
        self.assertAlmostEqual(x, 2.0)
        self.assertAlmostEqual(y, -2.0)

    def test_pose_is_read_from_map_transform(self):
        half = math.pi / 4.0
        transform = SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=2.35, y=-1.8, z=0.04),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half)),
        ))
        pose = pose_from_transform(transform)
        self.assertEqual((pose['x'], pose['y'], pose['z']), (2.35, -1.8, 0.04))
        self.assertAlmostEqual(pose['yaw'], math.pi / 2.0)

    def test_small_future_tf_skew_allows_only_bounded_latest_transform_fallback(self):
        available = SimpleNamespace(sec=40, nanosec=761_000_000)
        requested = SimpleNamespace(sec=40, nanosec=773_000_000)
        self.assertTrue(is_small_future_tf_skew(requested, available, 0.1))
        self.assertFalse(is_small_future_tf_skew(requested, available, 0.01))

    def test_tf_fallback_rejects_old_or_missing_transform_time(self):
        available = SimpleNamespace(sec=40, nanosec=761_000_000)
        old = SimpleNamespace(sec=40, nanosec=700_000_000)
        missing = SimpleNamespace(sec=0, nanosec=0)
        self.assertFalse(is_small_future_tf_skew(old, available, 0.1))
        self.assertFalse(is_small_future_tf_skew(missing, available, 0.1))

    def test_dynamic_tf_freshness_rejects_previous_simulation_epoch(self):
        self.assertFalse(is_fresh_tf_sample(20.0, 60.0, 2.0, 0.1))

    def test_dynamic_tf_freshness_accepts_only_bounded_future_skew(self):
        self.assertTrue(is_fresh_tf_sample(20.0, 20.05, 2.0, 0.1))
        self.assertFalse(is_fresh_tf_sample(20.0, 20.11, 2.0, 0.1))

    def test_dynamic_tf_freshness_rejects_stale_zero_and_non_finite_stamps(self):
        self.assertFalse(is_fresh_tf_sample(20.0, 17.9, 2.0, 0.1))
        self.assertFalse(is_fresh_tf_sample(20.0, 0.0, 2.0, 0.1))
        self.assertFalse(is_fresh_tf_sample(float('nan'), 20.0, 2.0, 0.1))


if __name__ == '__main__':
    unittest.main()
