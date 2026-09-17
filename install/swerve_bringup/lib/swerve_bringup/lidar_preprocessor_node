#!/usr/bin/env python3
"""Filter the native 3D LiDAR cloud without changing its message type."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


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
        self.publisher = self.create_publisher(PointCloud2, output_topic, 10)
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.callback, qos_profile_sensor_data)

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
        points = []
        occupied = set()
        min_r2 = self.min_range * self.min_range
        max_r2 = self.max_range * self.max_range

        for point in point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = (float(point[0]), float(point[1]), float(point[2]))
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            if z < self.min_height or z > self.max_height:
                continue
            if self.remove_floor and abs(z - self.floor_z) <= self.floor_band:
                continue
            if x * x + y * y + z * z < min_r2 or x * x + y * y + z * z > max_r2:
                continue
            if all(self.self_min[i] <= value <= self.self_max[i]
                   for i, value in enumerate((x, y, z))):
                continue

            if self.voxel_size > 0.0:
                key = (math.floor(x / self.voxel_size),
                       math.floor(y / self.voxel_size),
                       math.floor(z / self.voxel_size))
                if key in occupied:
                    continue
                occupied.add(key)
            points.append((x, y, z))

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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
