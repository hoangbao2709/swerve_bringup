"""QoS profiles shared by the ROS bridge and its focused tests."""

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def gazebo_clock_qos_profile() -> QoSProfile:
    """Return the Gazebo Classic /clock subscriber contract."""
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def canonical_map_qos_profile() -> QoSProfile:
    """Receive the latched canonical map even when the bridge starts late."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
