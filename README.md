# swerve_bringup

Muốn chạy toàn bộ frontend + Django backend + Gazebo + ROS bridge + RViz,
xem [SETUP_A_Z.md](SETUP_A_Z.md). Lệnh build trong tài liệu đã xử lý package
lồng `swerve_bridge` và môi trường ROS overlay của máy.

Package ROS 2 (ament_cmake) chứa mô tả URDF + launch hiển thị cho AGV đa hướng:
- 2 cụm swerve (steer_front, steer_rear): mỗi cụm có khớp xoay đứng (steer) + khớp lăn bánh chủ động (drive)
- 4 caster bị động ở 4 góc: physics mặc định là collision cylinder đơn giản, ít ma sát
- Visual caster mặc định chỉ là xấp xỉ mount + fork + wheel; `proper_caster_test:=true` mới thêm DOF swivel + roll thực nghiệm
- Bộ gá đỡ (board điều khiển + bracket) gắn cố định vào base_link

Toàn bộ hình học & vị trí khớp được trích xuất trực tiếp từ CAD gốc:
`096626040039-1.STEP` (khung + bố trí tổng thể) và
`HZ-CS95-____-DL-200-600-BK48-35-N7-KG.STEP` (chi tiết cụm swerve, dùng để xác định
chính xác tâm trục xoay đứng và trục bánh chủ động qua phân tích mặt trụ hình học).

## Cấu trúc

```
swerve_bringup/
├── package.xml
├── CMakeLists.txt
├── urdf/swerve_base.urdf
├── meshes/*.stl              (CAD gốc, đơn vị mm, không bị overwrite)
├── meshes/visual/*.obj       (visual đã optimize, đơn vị m, có normals)
├── config/controllers.yaml
├── config/ekf.yaml
├── config/lidar_preprocessing.yaml
├── config/pointcloud_to_laserscan.yaml
├── config/slam_toolbox.yaml
├── swerve_navigation/config/nav2_params.yaml
├── swerve_navigation/launch/navigation.launch.py
├── swerve_navigation/maps/
├── worlds/test.world
├── worlds/warehouse.world
├── launch/display.launch.py
├── launch/gazebo.launch.py
└── rviz/swerve.rviz
```

## Phase 2: ros2_control joint control

Phase 2 chỉ expose bốn joint chủ động trong Gazebo Classic qua
`gazebo_ros2_control/GazeboSystem`:

| Joint | Command interface | State interfaces | Axis |
|---|---|---|---|
| `steer_front_joint` | `position` | `position`, `velocity` | +Z |
| `steer_rear_joint` | `position` | `position`, `velocity` | +Z |
| `wheel_front_drive_joint` | `velocity` | `position`, `velocity` | +Y |
| `wheel_rear_drive_joint` | `velocity` | `position`, `velocity` | +Y |

Các controller được load tự động sau khi spawn entity:

```text
joint_state_broadcaster  joint_state_broadcaster/JointStateBroadcaster  ACTIVE
steering_controller      position_controllers/JointGroupPositionController ACTIVE
drive_controller         velocity_controllers/JointGroupVelocityController ACTIVE
```

Chạy và kiểm tra:

```bash
colcon build --packages-select swerve_bringup
source install/setup.bash
ros2 launch swerve_bringup gazebo.launch.py
ros2 control list_controllers --claimed-interfaces
ros2 topic echo /joint_states
```

## Phase 6: robot_localization EKF

`robot_localization` fuses the planar swerve wheel odometry from `/odom` with
the IMU yaw rate from `/imu/data` and publishes `/odometry/filtered`. The EKF
uses `two_d_mode: true`; LiDAR and ground truth are not localization inputs.

Fields fused:

| Input | Fields |
|---|---|
| `/odom` (`nav_msgs/msg/Odometry`) | `x`, `y`, `yaw`, `vx`, `vy`, `wz` |
| `/imu/data` (`sensor_msgs/msg/Imu`) | `angular_velocity.z` |

TF ownership is deliberately single-source: `ekf_filter_node` publishes
`odom -> base_footprint`; `swerve_odometry_node` publishes `/odom` but has
`publish_tf: false`. `robot_state_publisher` owns the fixed transforms
`base_footprint -> base_link` and the sensor links:

```text
odom -> base_footprint -> base_link
                         ├── lidar_link
                         └── imu_link
```

Run and debug:

```bash
ros2 launch swerve_bringup gazebo.launch.py
ros2 topic hz /odometry/filtered
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_tools view_frames
```

For evaluation, record `/ground_truth/odom`, `/odom`, and
`/odometry/filtered` separately and compare position error and yaw error over
forward 5 m, reverse, strafe, diagonal, 90/360 degree rotations, and mixed
`vx + vy + wz` motions. `/ground_truth/odom` is evaluation-only and is never
fused by the EKF.

## Phase 7: warehouse world for 3D LiDAR

`worlds/warehouse.world` is a 30 m x 20 m warehouse envelope. It contains a
central drive aisle, two multi-level rack rows, inbound/outbound pallet areas,
a conveyor, packing workstation, safety barrier, mobile obstacle, perimeter
walls and four pillars. Every warehouse object has a separate visual and
collision geometry; rack shelves, posts and boxes are separate collision
models rather than one oversized rack collision.

Layout coordinates (metres):

| Object | Coordinates / size |
|---|---|
| Floor | center `(0, 0, -0.05)`, `30 x 20 x 0.1` |
| Walls | envelope `x=[-15,15]`, `y=[-10,10]`, height `4` |
| Rack rows | `y=-4.5` and `y=+4.5`; posts `x=-9..9`; shelf levels `z=0.70, 1.70, 2.70` |
| Inbound pallets/boxes | `y=-7.0`, `x=-13.0,-11.8,-10.6` |
| Outbound pallets/boxes | `y=+7.0`, `x=-13.0,-11.8,-10.6` |
| Conveyor | center `(0,-7.0,1.05)`, deck `10 x 1.25 x 0.22` |
| Packing workstation | center `(7,-7.0,1.0)`, table `3 x 1.2 x 0.18` |

Coordinate convention is intentionally one-to-one:

```text
Gazebo world origin = ROS map origin = WareTwin origin = (0, 0, 0)
+X: warehouse aisle/east direction
+Y: north/left direction
+Z: up
```

No WareTwin coordinate file was present in this package, so all Phase 7
warehouse dimensions, rack positions, conveyor position and obstacle positions
are explicitly marked as estimates until the actual layout is supplied. No
additional map offset or hidden transform is introduced. The robot starts at
`(-12, 0, 0.001)` in the central aisle, heading `+X`.

Run the warehouse and inspect the 3D cloud:

```bash
ros2 launch swerve_bringup gazebo.launch.py
ros2 topic hz /lidar/points
ros2 topic echo /lidar/points --once
ros2 run tf2_ros tf2_echo base_link lidar_link
rviz2
```

In RViz set `Fixed Frame` to `odom` and add `PointCloud2` on
`/lidar/points`. The floor should produce low-Z points, rack upper levels
should produce higher-Z points, and walls/conveyor/boxes should appear as
separate surfaces. To run the old empty test world for regression only:

```bash
ros2 launch swerve_bringup gazebo.launch.py \
  world:=/home/yahboom/swerve_bringup/install/swerve_bringup/share/swerve_bringup/worlds/test.world
```

Remaining estimates: warehouse dimensions/layout, shelf and pallet dimensions,
conveyor/workstation dimensions, pillar positions, obstacle positions, and the
robot sensor mounting values already identified in `config/lidar.yaml`.

### Visual assets and fallback mode

`use_cad_visuals:=true` loads the decimated CAD-derived OBJ files in
`meshes/visual/`. `use_cad_visuals:=false` selects lightweight URDF primitives
for visual debugging. Both choices leave collision geometry, inertial values,
joint axes and controller physics unchanged. The fallback drive-wheel cylinder
is rotated onto the existing +Y axle inside its `<visual><origin>`.

`tools/optimize_visual_mesh.py` reads the original binary STL, removes
degenerate/duplicate geometry, repairs orientable winding, writes explicit OBJ
vertex normals, and validates finite vertices and source bounding-box extent.
It writes only under `meshes/visual/`; the CAD STL files remain unchanged and
are not used as collision geometry.

## Phase 8: 3D LiDAR preprocessing and SLAM

The native `/lidar/points` `sensor_msgs/msg/PointCloud2` remains the raw sensor
stream. `lidar_preprocessor_node` creates `/lidar/points_filtered` with NaN,
range, height, robot-body self-filter and voxel filtering. Floor removal is
available but disabled by default so the filtered cloud remains useful for 3D
perception.

`pointcloud_to_laserscan` projects only the configured slice
`min_height=-0.15` to `max_height=0.30` in `lidar_link` into `/scan`. This
height slice sees walls, rack legs and conveyor surfaces at robot height while
preventing upper rack tiers from becoming 2D SLAM obstacles.

`slam_toolbox` consumes `/scan`, uses the EKF odometry topic through the
existing `odom -> base_footprint -> base_link` TF chain, and publishes
`map -> odom`. No Nav2 is started in this phase.

Start the pipeline after the robot/Gazebo launch:

```bash
ros2 launch swerve_bringup gazebo.launch.py
ros2 launch swerve_bringup slam.launch.py
```

Inspect all three representations in RViz:

```bash
ros2 topic hz /lidar/points
ros2 topic hz /lidar/points_filtered
ros2 topic hz /scan
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo map base_link
```

Add `PointCloud2` `/lidar/points_filtered`, `LaserScan` `/scan`, and `Map`
`/map` to RViz. Save a completed map with:

```bash
ros2 run nav2_map_server map_saver_cli -f warehouse_map \
  --ros-args -p map_subscribe_transient_local:=true
```

Required runtime packages are `pointcloud_to_laserscan` and `slam_toolbox`.
The filter node and all configuration files are included in this package; if
`ros2 pkg prefix pointcloud_to_laserscan` or `ros2 pkg prefix slam_toolbox`
fails on another machine, install the matching ROS distribution packages
before running `slam.launch.py`.

## Phase 9: holonomic Nav2 with 3D LiDAR

Nav2 configuration is in `swerve_navigation/config/nav2_params.yaml` and uses
`dwb_core::DWBLocalPlanner`. It explicitly samples both X and Y velocities:
`vx=-0.55..0.55 m/s`, `vy=-0.55..0.55 m/s`, and `wz=-1.20..1.20 rad/s`;
`vy` is not forced to zero. Acceleration limits are `0.8 m/s²` in X/Y and
`1.8 rad/s²` in yaw, with deceleration limits `1.0 m/s²` and `2.2 rad/s²`.

The robot footprint is a real-base polygon with 5 cm padding:

```text
[[ 0.60,  0.30], [ 0.60, -0.30],
 [-0.60, -0.30], [-0.60,  0.30]]
```

The local costmap uses a VoxelLayer with `/lidar/points_filtered` as
`PointCloud2` (`marking: true`, `clearing: true`) and `/scan` as an additional
observation. Its obstacle height is limited to `0.05..1.30 m`, so rack points
above the robot clearance do not automatically block the 2D planner. The
global costmap uses the selected static Nav2 `/map` plus `/scan` updates.

Start in this order:

```bash
./scripts/start_stack.sh mapping
./scripts/start_stack.sh navigation --map /absolute/path/to/saved_map.yaml
```

The navigation launch starts the static `map_server`, controller, planner,
behavior, BT navigator, waypoint follower and lifecycle manager. The
controller's `/cmd_vel` goes directly to the existing swerve controller, which
converts `vx`, `vy`, `wz` to steering and drive commands. SLAM Toolbox is not
started in navigation mode; the selected saved map plus the V30E/tag
localization filter own the `map -> odom` correction. Navigation still starts
the LiDAR preprocessing and point-cloud-to-scan stages so
`/lidar/points_filtered` and `/scan` remain live; only SLAM Toolbox is
disabled in this mode.

Navigation checks:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /local_costmap/costmap --once
ros2 topic echo /local_costmap/voxel_grid --once
ros2 action list | rg navigate
```

Test forward, left/right goals, pure strafe, in-place rotation, diagonal
goals, narrow aisles, static obstacles, and an obstacle inserted while moving.
Acceptance requires a real `/cmd_vel` path through the swerve controller,
costmap marking/clearing from PointCloud2, replanning or stopping when blocked,
and no teleport or animation-based motion. The runtime acceptance below covers
startup, live data, TF, bridge health and Nav2 lifecycle; goal-motion and
obstacle-behaviour tests remain a separate deliberate test session.

Steering controller nhận `std_msgs/msg/Float64MultiArray` theo thứ tự
`[steer_front_joint, steer_rear_joint]`; drive controller theo thứ tự
`[wheel_front_drive_joint, wheel_rear_drive_joint]`:

```bash
# steer front +30, -30; steer rear +30, -30 (radian)
ros2 topic pub --once /steering_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.523599, 0.0]}"
ros2 topic pub --once /steering_controller/commands std_msgs/msg/Float64MultiArray "{data: [-0.523599, 0.0]}"
ros2 topic pub --once /steering_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.523599]}"
ros2 topic pub --once /steering_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, -0.523599]}"

# front/rear drive forward and reverse; stop by publishing zero
ros2 topic pub --once /drive_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0, 0.0]}"
ros2 topic pub --once /drive_controller/commands std_msgs/msg/Float64MultiArray "{data: [-1.0, 0.0]}"
ros2 topic pub --once /drive_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 1.0]}"
ros2 topic pub --once /drive_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, -1.0]}"
ros2 topic pub --once /drive_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}"
```

Không dùng `diff_drive_controller`, `/cmd_vel`, odometry, cảm biến hoặc Nav2
trong Phase 2. Publisher joint-state Gazebo cũ đã được bỏ để tránh conflict
với `joint_state_broadcaster`. Physics caster mặc định vẫn là collision model
đơn giản thụ động; phần visual mount + fork + wheel chỉ là approximation. Nhánh
`proper_caster_test:=true` là test thực nghiệm riêng, thêm swivel + roll DOFs
và không phải physics mặc định.

## Phase 3: swerve kinematics + `/cmd_vel`

Node `swerve_controller_node` subscribe `/cmd_vel` và publish vào
`/steering_controller/commands` và `/drive_controller/commands`. Với mỗi module:

```text
module_vx = vx - wz*y
module_vy = vy + wz*x
angle = atan2(module_vy, module_vx)
wheel_angular_velocity = hypot(module_vx, module_vy) / wheel_radius
```

Node có normalize góc, shortest-path, wheel reverse khi cần quay hơn 90 độ,
deadband/giữ góc lúc đứng yên, giới hạn tốc độ bánh, steering rate, gia tốc
bánh và timeout 0.5 s. Geometry nằm trong `config/swerve_controller.yaml`.

Test nhanh:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0}, angular: {z: 0.0}}"
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.5}, angular: {z: 0.0}}"
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.5}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}"
```

Dừng publisher để kiểm tra watchdog; sau 0.5 s drive command được ramp về 0,
steering giữ nguyên góc cuối. Phase 3 không thêm odometry, LiDAR hoặc Nav2.

## Phase 4: encoder odometry + TF

`swerve_odometry_node` chỉ đọc `/joint_states`:

```text
wheel_v = wheel_velocity_sign * wheel_radius * wheel_joint_velocity
u_i = wheel_v*cos(steer_i)
v_i = wheel_v*sin(steer_i)
u_i = vx - wz*y_i
v_i = vy + wz*x_i
```

Hai module được giải bằng least-squares cho `(vx, vy, wz)`. Vận tốc thân
được đổi từ frame robot sang frame `odom` và tích phân thành `(x, y, yaw)`.
Odometry encoder publish `/odom` nhưng không publish TF. `robot_localization`
fuses `/odom` + IMU and is the single owner of `odom -> base_footprint`;
`robot_state_publisher` publishes `base_footprint -> base_link`.

Frame convention: ROS REP-103, `x` tiến, `y` trái, `z` lên; yaw dương ngược
chiều kim đồng hồ nhìn từ +Z. Dấu wheel hiện tại là
`wheel_velocity_sign=1.0`; nếu thay đổi mô hình joint hoặc driver thực, hiệu
chuẩn dấu ở cùng một tham số thay vì đổi UI.

Debug:

```bash
ros2 topic echo /odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic echo /joint_states
```

Node không dùng ground truth và không publish `/ground_truth/odom`; ground truth
nếu cần đánh giá phải do một node/plugin riêng cung cấp. Các bài test forward
5 m, reverse, strafe 3 m, quay 90/360 độ, diagonal và square path cần chạy
sau khi Gazebo/ros2_control khởi động ổn định.

## Phase 5: 3D LiDAR + IMU

Đã thêm hai fixed frame từ `base_link`:

```text
base_link
├── lidar_link
└── imu_link
```

Pose và sensor parameters được quản lý bằng [lidar.yaml](file:///home/yahboom/swerve_bringup/config/lidar.yaml)
và [imu.yaml](file:///home/yahboom/swerve_bringup/config/imu.yaml), sau đó
launch truyền chúng vào xacro arguments. Tất cả pose hiện tại đều là ESTIMATE.

LiDAR dùng Gazebo Classic multi-ray (`type="ray"`) với 720 mẫu ngang × 16
kênh đứng, FOV ngang 360 độ, FOV đứng 30 độ, 10 Hz, range 0.2–30 m và
Gaussian range noise 0.01 m. Plugin `libgazebo_ros_ray_sensor.so` publish:

```text
/lidar/points       sensor_msgs/msg/PointCloud2
frame_id            lidar_link
```

IMU dùng `libgazebo_ros_imu_sensor.so`, publish:

```text
/imu/data           sensor_msgs/msg/Imu
frame_id            imu_link
```

Debug/RViz:

```bash
ros2 topic list
ros2 topic hz /lidar/points
ros2 topic echo /lidar/points --once
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 run tf2_ros tf2_echo base_link imu_link
rviz2
```

Trong RViz đặt `Fixed Frame` là `odom` hoặc `base_link`, thêm display
`PointCloud2` với topic `/lidar/points`. Không thêm EKF, SLAM, Nav2,
WareTwin hoặc ground-truth sensor.

## Cây kinematic (10 link / 9 joint)

```
base_footprint (ao, khong hinh hoc - z=0, mat dat)
└── base_footprint_joint (fixed, z=+0.245) -> base_link
      ├── steer_front_joint (continuous, Z)      -> steer_front_link
      │     └── wheel_front_drive_joint (Y)      -> wheel_front_drive_link   [banh chu dong, mesh that]
      ├── steer_rear_joint  (continuous, Z)      -> steer_rear_link
      │     └── wheel_rear_drive_joint  (Y)      -> wheel_rear_drive_link    [banh chu dong, mesh that]
      ├── wheel_front_left_joint  (fixed) -> wheel_front_left_link   [collision thụ động + visual mount/fork/wheel]
      ├── wheel_front_right_joint (fixed) -> wheel_front_right_link  [collision thụ động + visual mount/fork/wheel]
      ├── wheel_rear_left_joint   (fixed) -> wheel_rear_left_link    [collision thụ động + visual mount/fork/wheel]
      └── wheel_rear_right_joint  (fixed) -> wheel_rear_right_link   [collision thụ động + visual mount/fork/wheel]
```

Đây là cây physics mặc định. `proper_caster_test:=true` thay nhánh caster
bằng fork + wheel có thêm swivel và roll joints để thử nghiệm; visual
approximation không phải bằng chứng rằng physics mặc định có các DOF đó.

`base_footprint` la link ao (khong mesh/inertial) theo chuan REP-105/REP-120, dat tai
hinh chieu cua robot xuong mat dat (z=0) - can cho Nav2 (costmap, AMCL, controller...).
Do cao 0.245m tu base_footprint len base_link duoc uoc luong tu diem tiep dat thap nhat
cua banh chu dong va banh bi dong (2 uoc luong lech nhau ~3mm, da lay trung binh) -
NEN DO LAI THUC TE sau khi lap rap de hieu chinh chinh xac.

## Build & chạy (ROS 2, workspace colcon)

```bash
# copy thư mục swerve_bringup/ vào <ws>/src/, sau đó:
cd <ws>
colcon build --packages-select swerve_bringup
source install/setup.bash
ros2 launch swerve_bringup display.launch.py
```

Mặc định bật `joint_state_publisher_gui` để bạn kéo thanh trượt test các khớp
steer và drive. Với caster, chỉ dùng `proper_caster_test:=true` khi cần test
swivel/roll thực nghiệm. Tắt GUI bằng:

```bash
ros2 launch swerve_bringup display.launch.py use_joint_state_gui:=false
```

## Các giả định / hạn chế CẦN KIỂM TRA LẠI trước khi dùng thật

1. **Khối lượng & quán tính**: ước lượng từ thể tích hình học × mật độ giả định
   (nhôm 2700 kg/m³ cho khung/cụm xoay/bracket, 1500 kg/m³ cho bánh bị động).
   Không phải số liệu cân đo thật.
2. **`wheel_front_drive_link` / `wheel_rear_drive_link`**: mesh là hình trụ
   placeholder Ø135mm (đo từ mặt trụ hình học lớn nhất tìm được trong khối
   HZ-CS95), KHÔNG phải mesh bánh xe thật (không có gân lốp/rãnh). Nếu có
   file STEP/STL riêng của bánh xe, thay `meshes/drive_wheel.stl`.
3. **`steer_front_link` / `steer_rear_link`**: mesh vẫn là toàn bộ khối
   HZ-CS95 (housing + phần quay dính liền) vì file CAD xuất dạng 1 solid
   duy nhất — không tách được vỏ đứng yên (nếu có) khỏi phần quay bên trong.
4. **4 bánh caster**: physics mặc định là collision cylinder thụ động đơn
   giản và không có swivel/roll DOF; visual mount + fork + wheel chỉ là
   approximation. Nhánh `proper_caster_test:=true` là mô hình thực nghiệm có
   swivel + roll và zero-trail mặc định, chưa phải calibration cơ khí.
5. CAD STL gốc là **mm** và không bị ghi đè. OBJ trong `meshes/visual/` đã
   được chuyển sang **m** (có normals và scale hình học giữ nguyên), nên URDF
   dùng `scale="1 1 1"`. Không dùng visual OBJ cho collision.
