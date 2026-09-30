import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'swerve_controller'))
from swerve_odometry_node import SwerveOdometry
from rclpy.time import Time


def odometry(prefer):
    return SimpleNamespace(
        prefer_position_velocity=prefer,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=1.0)),
        modules=('front', 'rear'), steer_names={'front': 'steer_front_joint', 'rear': 'steer_rear_joint'},
        wheel_names={'front': 'wheel_front_drive_joint', 'rear': 'wheel_rear_drive_joint'},
        steer={'front': 0., 'rear': 0.}, wheel_velocity={'front': 0., 'rear': 0.},
        previous_wheel_position={'front': None, 'rear': None}, previous_state_time=None,
        compute_body_velocity=lambda: (0., 0., 0.))


def frame(stamp, positions, velocities):
    return SimpleNamespace(name=['wheel_front_drive_joint', 'wheel_rear_drive_joint'],
        position=positions, velocity=velocities,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp, nanosec=0)))


def test_gazebo_position_differences_not_contact_velocity_drive_odometry():
    node = odometry(True)
    SwerveOdometry.joint_state_callback(node, frame(1, [2., 3.], [-.154, -.107]))
    SwerveOdometry.joint_state_callback(node, frame(2, [2.001, 3.002], [-.154, -.107]))
    assert node.wheel_velocity == pytest.approx({'front': .001, 'rear': .002})


def test_actual_wheel_rotation_is_not_zeroed_or_hidden():
    node = odometry(True)
    SwerveOdometry.joint_state_callback(node, frame(1, [2., 3.], [0., 0.]))
    SwerveOdometry.joint_state_callback(node, frame(2, [5., 0.], [0., 0.]))
    assert node.wheel_velocity == pytest.approx({'front': 3., 'rear': -3.})


def test_hardware_default_keeps_reported_velocity_and_position_fallback():
    node = odometry(False)
    SwerveOdometry.joint_state_callback(node, frame(1, [2., 3.], [-.154, -.107]))
    assert node.wheel_velocity == {'front': -.154, 'rear': -.107}
    SwerveOdometry.joint_state_callback(node, frame(2, [2.1, 3.2], []))
    assert node.wheel_velocity == pytest.approx({'front': .1, 'rear': .2})
