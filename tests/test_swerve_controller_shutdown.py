import pytest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'swerve_controller'))
import swerve_controller_node as controller


def test_executor_conversion_error_is_suppressed_only_after_ros_shutdown(monkeypatch):
    monkeypatch.setattr(controller.rclpy, 'spin', lambda _node: (_ for _ in ()).throw(
        RuntimeError('Unable to convert call argument to Python object')))
    monkeypatch.setattr(controller.rclpy, 'ok', lambda: False)

    controller.spin_until_shutdown(object())


def test_executor_runtime_error_while_ros_is_live_propagates(monkeypatch):
    monkeypatch.setattr(controller.rclpy, 'spin', lambda _node: (_ for _ in ()).throw(
        RuntimeError('live executor failure')))
    monkeypatch.setattr(controller.rclpy, 'ok', lambda: True)

    with pytest.raises(RuntimeError, match='live executor failure'):
        controller.spin_until_shutdown(object())
