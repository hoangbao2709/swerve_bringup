"""Numerical sign tests for the two in-line swerve modules."""
import math
import unittest


def module_velocity(vx, vy, wz, x, y):
    return vx - wz * y, vy + wz * x


def command_equivalent(vx, vy, wz, x, y, radius=0.0675):
    mx, my = module_velocity(vx, vy, wz, x, y)
    speed = math.hypot(mx, my)
    angle = math.atan2(my, mx)
    return angle, speed / radius


class SwerveKinematicsTest(unittest.TestCase):
    modules = ((0.300042, 0.0), (-0.300042, 0.0))

    def test_translation_signs(self):
        for vx, vy, expected_x, expected_y in ((1, 0, 1, 0), (-1, 0, -1, 0), (0, 1, 0, 1), (0, -1, 0, -1)):
            for x, y in self.modules:
                mx, my = module_velocity(vx, vy, 0, x, y)
                self.assertEqual(math.copysign(1, mx) if mx else 0, expected_x)
                self.assertEqual(math.copysign(1, my) if my else 0, expected_y)

    def test_pure_rotation_has_opposite_module_velocity(self):
        front = module_velocity(0, 0, 1, *self.modules[0])
        rear = module_velocity(0, 0, 1, *self.modules[1])
        self.assertAlmostEqual(front[0], 0.0)
        self.assertGreater(front[1], 0.0)
        self.assertAlmostEqual(rear[0], 0.0)
        self.assertLess(rear[1], 0.0)
        self.assertAlmostEqual(math.hypot(*front), math.hypot(*rear))

    def test_diagonal_and_combined_rotation(self):
        angle, speed = command_equivalent(1, 1, 0, *self.modules[0])
        self.assertAlmostEqual(angle, math.pi / 4)
        self.assertAlmostEqual(speed, math.hypot(1, 1) / 0.0675)
        front = module_velocity(1, 0, 1, *self.modules[0])
        rear = module_velocity(1, 0, 1, *self.modules[1])
        self.assertGreater(front[1], 0.0)
        self.assertLess(rear[1], 0.0)


if __name__ == '__main__':
    unittest.main()
