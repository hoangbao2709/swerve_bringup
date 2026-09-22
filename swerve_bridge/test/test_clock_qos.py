import sys
from pathlib import Path

from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

# The ROS package is installed with ``swerve_bridge/`` as its Python root;
# make the source-tree test use the same import layout without requiring an
# editable install first.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from swerve_bridge.qos import gazebo_clock_qos_profile


def test_gazebo_clock_qos_matches_runtime_contract():
    profile = gazebo_clock_qos_profile()
    assert profile.reliability == ReliabilityPolicy.BEST_EFFORT
    assert profile.durability == DurabilityPolicy.VOLATILE
    assert profile.depth == 10
