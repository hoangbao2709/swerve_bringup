# Phase 11 — Sim to Real interface

The common ROS contract is:

| Signal | Standard topic/frame |
|---|---|
| 3D LiDAR | `/lidar/points`, `lidar_link` |
| IMU | `/imu/data`, `imu_link` |
| wheel odometry | `/odom` |
| filtered odometry | `/odometry/filtered` |

The downstream nodes do not change between Gazebo and the physical robot:

`/lidar/points` → preprocessing → `/lidar/points_filtered` → `/scan`

`/odom` + `/imu/data` → EKF → `/odometry/filtered`

## Launch

Gazebo:

```bash
source /opt/ros/humble/setup.bash
source ~/swerve_bringup/install/local_setup.bash
ros2 launch swerve_bringup system.launch.py use_sim:=true use_sim_time:=true
```

Real robot:

```bash
ros2 launch swerve_bringup system.launch.py \
  use_sim:=false use_sim_time:=false \
  real_sensor_launch:=/absolute/path/to/real_sensors.launch.py \
  real_lidar_topic:=/vendor/lidar/cloud \
  real_imu_topic:=/vendor/imu/data \
  real_odom_topic:=/vendor/odom \
  bridge_token:=<WARETWIN_ROS_BRIDGE_TOKEN>
```

The vendor launch should publish/remap its data to the standard topics. No full
PointCloud2 is sent to Django; the ROS bridge only sends telemetry and status.

## Calibration before real deployment

Update `config/sim_real_interface.yaml` and rebuild/relaunch after measuring:

- `lidar_extrinsics.xyz/rpy`: measured `base_link → lidar_link` translation and rotation.
- `imu_extrinsics.xyz/rpy`: measured `base_link → imu_link` translation and rotation.
- LiDAR channels, vertical/horizontal FOV, scan frequency, min/max range and noise.
- IMU update rate, gyro noise and accelerometer noise.
- Driver frame IDs: clouds must use `lidar_link`, IMU data must use `imu_link`.
- Driver timestamps and `use_sim_time` (`false` on the real robot).
- Wheel radius, wheelbase/track geometry and encoder sign/scale in `config/swerve_odometry.yaml`.
- EKF covariances and sensor timeout in `config/ekf.yaml`.
- `pointcloud_to_laserscan` height slice and preprocessing self-filter bounds.

Only the vendor driver launch and the interface calibration should differ between
simulation and the real robot. Nav2, SLAM, EKF, filtering and WareTwin bridge
consume the same topic/frame interface.
