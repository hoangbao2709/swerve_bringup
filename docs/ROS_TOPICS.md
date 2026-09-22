# ROS 2 topics and TF contract

All distances are metres, angles are radians and timestamps use ROS time. In
simulation `use_sim_time=true` is required and `/clock` comes from Gazebo.

## Sensor and state pipeline

| Topic | Type | Producer / consumer |
|---|---|---|
| `/lidar/points` | `sensor_msgs/PointCloud2` | Gazebo LiDAR or real driver |
| `/lidar/points_filtered` | `sensor_msgs/PointCloud2` | `lidar_preprocessor_node` |
| `/scan` | `sensor_msgs/LaserScan` | `pointcloud_to_laserscan` |
| `/imu/data` | `sensor_msgs/Imu` | Gazebo/real IMU |
| `/joint_states` | `sensor_msgs/JointState` | joint state broadcaster |
| `/odom` | `nav_msgs/Odometry` | swerve odometry |
| `/odometry/filtered` | `nav_msgs/Odometry` | robot_localization EKF |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo |
| `/cmd_vel` | `geometry_msgs/Twist` | manual/Nav2/bridge to controller |
| `/emergency_stop` | `std_msgs/Bool` | bridge safety command |

The bridge measures LiDAR age, frame, stamp and frequency. DDS does not expose a
portable dropped-message counter through this node, so dropped messages remain
`null`/unknown rather than being fabricated as zero.

## Control topics and services

| Topic / service | Purpose |
|---|---|
| `/steering_controller/commands` | four steering position commands |
| `/drive_controller/commands` | drive wheel velocity commands |
| `/controller_manager/list_controllers` | measured controller state |
| `/go_to_tag` | project `GoToTag` action used by the bridge |
| `/navigate_to_pose` | Nav2 action boundary when configured |

The controller enforces max linear/angular velocity, wheel velocity,
acceleration, steering rate/angle and the command timeout. Manual and
autonomous commands must not be sent concurrently by the UI.

The browser sends `ROBOT_MODE` and repeated `ROBOT_MANUAL` frames through
Django; the bridge converts them to bounded `/cmd_vel` commands. Manual actions
are `FORWARD`, `BACKWARD`, `LEFT`, `RIGHT`, `ROTATE_LEFT`, `ROTATE_RIGHT` and
`STOP`. Commands expire after `manual_command_timeout` (0.4 s by default), so
lost WebSocket/keyboard focus stops the robot. `EMERGENCY_STOP` publishes a
latched `/emergency_stop` Boolean and zero velocity; clear it explicitly before
resuming control.

For a multi-robot deployment, launch one bridge per robot with a unique
`robot_id` and set its `namespace` parameter (for example `robot_1`). The bridge
scopes its configured topics, controller-manager service and GoToTag action to
that namespace. The default empty namespace keeps the existing single-robot
topic contract backward-compatible.

## Mapping graph

```text
/lidar/points
  -> lidar_preprocessor_node
/lidar/points_filtered
  -> pointcloud_to_laserscan
/scan + /odometry/filtered TF
  -> slam_toolbox
map -> odom
```

Only SLAM owns `map -> odom` in mapping mode.

## Navigation graph

```text
saved map YAML -> nav2_map_server
/scan + /lidar/points_filtered -> Nav2 costmaps
/odometry/filtered -> local odom/base state
V30E/tag localization -> map -> odom correction
Nav2 -> /cmd_vel -> swerve_controller
```

SLAM is not started in navigation mode. The selected `map_file` is validated
and passed to Nav2's map server. The common EKF owns `odom -> base_footprint`;
`robot_state_publisher` owns fixed links below the base; the navigation
localization filter owns the map correction.

## TF tree

```text
map
└── odom
    └── base_footprint
        └── base_link
            ├── lidar_link
            ├── imu_link
            └── v30e_link (when enabled)
```

Use `ros2 run tf2_tools view_frames` and `ros2 run tf2_ros tf2_echo` to verify
the live graph. Duplicate `map -> odom` or `odom -> base_footprint` publishers
are a configuration error.

## WebSocket bridge messages

ROS → Django: `ROBOT_STATE`, `HEARTBEAT`, `ROS_DIAGNOSTICS`, `NAV_STATUS`,
`ROBOT_CONTROL_STATUS`, `MAP_REVISION_STATUS`, `TAG_NAV_STATUS` and
localization/tag events.

Django → ROS: `NAV_GOAL`, `CANCEL_NAVIGATION`, `TAG_NAV_GOAL`, tag pause/resume/
cancel/replan, `CONTROL_MODE`, `MANUAL_CMD`, `MAP_PUBLISHED`, `EMERGENCY_STOP`
and `CLEAR_EMERGENCY_STOP`.

`MAP_REVISION_STATUS.gazebo_revision` is measured from the manifest beside the
world file supplied to the current launch. Receiving `MAP_PUBLISHED` does not
pretend that Gazebo Classic hot-reloaded geometry; a new published world needs a
stack restart before the status can become `SYNCED`.
