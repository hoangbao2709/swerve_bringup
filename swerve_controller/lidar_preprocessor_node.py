#!/usr/bin/env python3
"""Filter the native 3D LiDAR cloud without changing its message type."""

import signal

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


# Large point clouds are time-sensitive sensor data. A deeper queue lets a
# CPU-bound callback publish old scans after odometry/TF has already advanced;
# keep only the latest sample instead of feeding Nav2 stale measurements.
LIDAR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def filter_xyz_points(points, *, min_range, max_range, voxel_size, min_height,
                     max_height, remove_floor, floor_z, floor_band,
                     self_min, self_max):
    """Vectorized equivalent of the point-wise range, height and self filters.

    The simulator emits 11k+ points per cloud. Iterating structured NumPy
    records and hashing every point in Python consumed a material fraction of
    a core, starving the low-RTF Nav2 control loop. Preserve input order and
    the original first-point-per-voxel rule while moving the per-point work to
    NumPy's compiled operations.
    """
    if not len(points):
        return np.empty((0, 3), dtype=np.float32)

    # Work in float64 to retain the previous Python-float boundary behavior
    # for min/max range, height, and voxel calculations.
    x = points['x'].astype(np.float64, copy=False)
    y = points['y'].astype(np.float64, copy=False)
    z = points['z'].astype(np.float64, copy=False)
    distance_squared = x * x + y * y + z * z
    keep = (np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            & (z >= min_height) & (z <= max_height)
            & (distance_squared >= min_range * min_range)
            & (distance_squared <= max_range * max_range))
    if remove_floor:
        keep &= np.abs(z - floor_z) > floor_band
    in_self_box = ((x >= self_min[0]) & (x <= self_max[0])
                   & (y >= self_min[1]) & (y <= self_max[1])
                   & (z >= self_min[2]) & (z <= self_max[2]))
    keep &= ~in_self_box
    xyz = np.column_stack((x[keep], y[keep], z[keep]))

    if voxel_size > 0.0 and len(xyz):
        voxels = np.floor(xyz / voxel_size).astype(np.int64)
        _, first_indices = np.unique(voxels, axis=0, return_index=True)
        # np.unique sorts voxel keys; restore the source point ordering so the
        # retained point is exactly the first valid point as before.
        xyz = xyz[np.sort(first_indices)]
    return xyz.astype(np.float32, copy=False)


class LidarPreprocessor(Node):
    def __init__(self) -> None:
        super().__init__('lidar_preprocessor')
        self.declare_parameter('input_topic', '/lidar/points')
        self.declare_parameter('output_topic', '/lidar/points_filtered')
        self.declare_parameter('min_range', 0.20)
        self.declare_parameter('max_range', 30.0)
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('min_height', -1.0)
        self.declare_parameter('max_height', 4.0)
        self.declare_parameter('remove_floor', False)
        self.declare_parameter('floor_z', -0.35)
        self.declare_parameter('floor_band', 0.05)

        # Bounds are expressed in lidar_link. The sensor is mounted at z=0.35 m,
        # so this box covers the robot body below/around the sensor.
        self.declare_parameter('self_filter.min_x', -0.65)
        self.declare_parameter('self_filter.max_x', 0.65)
        self.declare_parameter('self_filter.min_y', -0.40)
        self.declare_parameter('self_filter.max_y', 0.40)
        self.declare_parameter('self_filter.min_z', -0.40)
        self.declare_parameter('self_filter.max_z', 0.15)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.publisher = self.create_publisher(PointCloud2, output_topic, LIDAR_QOS)
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.callback, LIDAR_QOS)

        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.min_height = float(self.get_parameter('min_height').value)
        self.max_height = float(self.get_parameter('max_height').value)
        self.remove_floor = bool(self.get_parameter('remove_floor').value)
        self.floor_z = float(self.get_parameter('floor_z').value)
        self.floor_band = float(self.get_parameter('floor_band').value)
        self.self_min = tuple(float(self.get_parameter(f'self_filter.min_{axis}').value)
                              for axis in ('x', 'y', 'z'))
        self.self_max = tuple(float(self.get_parameter(f'self_filter.max_{axis}').value)
                              for axis in ('x', 'y', 'z'))
        self.get_logger().info(
            f'Filtering {input_topic} -> {output_topic}; '
            f'range={self.min_range:.2f}..{self.max_range:.2f} m, '
            f'voxel={self.voxel_size:.3f} m')

    def callback(self, msg: PointCloud2) -> None:
        fields = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=False)
        points = filter_xyz_points(
            fields, min_range=self.min_range, max_range=self.max_range,
            voxel_size=self.voxel_size, min_height=self.min_height,
            max_height=self.max_height, remove_floor=self.remove_floor,
            floor_z=self.floor_z, floor_band=self.floor_band,
            self_min=self.self_min, self_max=self.self_max,
        ).tolist()

        # The native sensor cloud remains untouched; this is a new XYZ cloud
        # in the same lidar_link frame and timestamp for downstream consumers.
        filtered = point_cloud2.create_cloud_xyz32(msg.header, points)
        self.publisher.publish(filtered)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarPreprocessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
