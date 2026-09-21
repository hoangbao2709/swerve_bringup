# Setup A–Z: WareTwin + ROS 2 + Gazebo + RViz

Tài liệu này chạy profile tích hợp:

```text
React/Vite :5173
      │ REST + WebSocket /ws
Django/Channels :8000
      │ WebSocket /ws/ros
ROS 2 bridge ── Gazebo Classic + ros2_control + EKF + LiDAR/SLAM
      │
RViz2
```

Frontend không nói chuyện trực tiếp với ROS. Django là gateway; `swerve_bridge`
là node ROS duy nhất kết nối tới `/ws/ros`.

## 1. Điều kiện máy

- Ubuntu 22.04, ROS 2 Humble.
- Gazebo Classic 11 (`gazebo`, `gazebo_ros`), không dùng Gazebo Sim/GZ cho
  launch hiện tại.
- Python 3, `python3-venv`, Node.js 18+ và npm.
- Có desktop/display nếu muốn mở Gazebo GUI và RViz.

Nếu máy đã tự source một workspace khác trong `.bashrc`, luôn dùng
`source scripts/ros_env.sh` trước các lệnh ROS. Script này xoá overlay đó,
nạp `/opt/ros/humble`, rồi nạp workspace này.

## 2. Cài dependency hệ thống

```bash
cd /home/yahboom/swerve_bringup
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-gazebo-plugins \
  ros-humble-ros2-controllers \
  ros-humble-robot-localization \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-rviz2 \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  python3-colcon-common-extensions python3-rosdep python3-venv \
  python3-yaml python3-websocket
```

Các package bắt buộc nhất cho Gazebo là `gazebo_ros2_control` và
`ros2-controllers`; cho pipeline cảm biến là `robot_localization`,
`pointcloud_to_laserscan`, `slam_toolbox`.

## 3. Chạy script setup một lần

Script cài Python/Node dependencies, tạo database, seed dữ liệu warehouse và
build cả hai package ROS (kể cả package lồng `swerve_bridge`):

```bash
cd /home/yahboom/swerve_bringup
./scripts/setup_full_stack.sh
```

Nếu không muốn script cài apt, chạy riêng:

```bash
cd waretwin/backend
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
cp -n .env.example .env
.venv/bin/python manage.py migrate
.venv/bin/python manage.py seed_demo

cd ../frontend
npm ci

cd ../..
./scripts/build_ros.sh
```

## 4. Cấu hình backend và token bridge

`waretwin/backend/.env` phải có ít nhất:

```env
WARETWIN_RUNTIME_MODE=GAZEBO_ROS
WARETWIN_ROS_BRIDGE_TOKEN=đặt-một-token-chung
WARETWIN_ARTIFACT_ROOT=/home/yahboom/swerve_bringup/generated/maps
CORS_ALLOWED_ORIGINS=http://localhost:5173,http://127.0.0.1:5173
TWIN_ADMIN_USERNAME=admin
TWIN_ADMIN_PASSWORD=admin12345
```

`WARETWIN_ROS_BRIDGE_TOKEN` phải giống token truyền cho ROS launch. Không commit
`.env` lên Git. Sau khi đổi `.env`, chạy lại migrate/seed nếu cần:

```bash
cd waretwin/backend
.venv/bin/python manage.py migrate
.venv/bin/python manage.py seed_demo
```

## 5. Mở 4 terminal

### Terminal 1 — Django backend

```bash
cd /home/yahboom/swerve_bringup/waretwin/backend
./run.sh
```

Backend chạy tại `http://127.0.0.1:8000` và WebSocket tại
`ws://127.0.0.1:8000/ws` và `ws://127.0.0.1:8000/ws/ros`.

Kiểm tra:

```bash
curl http://127.0.0.1:8000/
curl http://127.0.0.1:8000/api/health
```

### Terminal 2 — frontend

```bash
cd /home/yahboom/swerve_bringup/waretwin/frontend
npm run dev -- --host 0.0.0.0
```

Mở `http://127.0.0.1:5173`, đăng nhập bằng tài khoản seed:

```text
username: admin
password: admin12345
```

Sau khi đăng nhập, runtime đúng sẽ là `GAZEBO_ROS`; trước khi ROS bridge kết
nối, robot có thể hiện `OFFLINE`.

### Terminal 3 — Gazebo + ROS bridge + SLAM + RViz

```bash
cd /home/yahboom/swerve_bringup
source scripts/ros_env.sh
ros2 launch swerve_bringup system.launch.py \
  use_sim:=true \
  use_sim_time:=true \
  mode:=mapping \
  gui:=true \
  start_rviz:=true \
  bridge_ws_url:=ws://127.0.0.1:8000/ws/ros \
  bridge_token="$WARETWIN_ROS_BRIDGE_TOKEN"
```

Lệnh này khởi động Gazebo, robot state publisher, spawn robot, ros2_control,
swerve controller, odometry, EKF, LiDAR, IMU, cloud filter, `/scan`, SLAM
Toolbox, `swerve_bridge` và RViz.

Không chạy thêm `display.launch.py` cùng lệnh này vì nó sẽ tạo một
`robot_state_publisher` thứ hai. `display.launch.py` chỉ dùng để kiểm tra URDF
không có Gazebo:

```bash
source scripts/ros_env.sh
ros2 launch swerve_bringup display.launch.py
```

### Terminal 4 — kiểm tra ROS

```bash
cd /home/yahboom/swerve_bringup
source scripts/ros_env.sh
ros2 control list_controllers
ros2 topic list | sort
ros2 topic hz /joint_states
ros2 topic hz /odom
ros2 topic hz /odometry/filtered
ros2 topic hz /lidar/points
ros2 topic hz /lidar/points_filtered
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom base_link
```

Kỳ vọng: `joint_state_broadcaster`, `steering_controller`,
`drive_controller` đều `active`; các topic sensor có message; RViz không còn
báo thiếu `map` sau khi SLAM nhận scan.

## 6. Test frontend ↔ backend ↔ ROS

Đăng nhập frontend và kiểm tra badge runtime. Backend WebSocket `/ws` phải nhận
`FULL` rồi `RUNTIME_STATUS`; ROS bridge `/ws/ros` phải làm backend báo
`ros_connected=true`.

Kiểm tra login nhanh:

```bash
cd waretwin/backend
.venv/bin/python - <<'PY'
import json, urllib.request
req = urllib.request.Request(
    'http://127.0.0.1:8000/api/auth/login',
    data=json.dumps({'username': 'admin', 'password': 'admin12345'}).encode(),
    headers={'Content-Type': 'application/json'},
)
print(json.load(urllib.request.urlopen(req)))
PY
```

## 7. Mapping rồi chạy navigation

Profile mặc định là `mode:=mapping`; SLAM Toolbox sở hữu `map -> odom`. Khi đã
quét đủ warehouse, lưu map:

```bash
ros2 run nav2_map_server map_saver_cli -f /tmp/warehouse_map \
  --ros-args -p map_subscribe_transient_local:=true
```

Navigation dùng map tĩnh và không chạy SLAM đồng thời:

```bash
ros2 launch swerve_bringup system.launch.py \
  use_sim:=true use_sim_time:=true mode:=navigation \
  gui:=true start_rviz:=true \
  bridge_ws_url:=ws://127.0.0.1:8000/ws/ros \
  bridge_token="$WARETWIN_ROS_BRIDGE_TOKEN"
```

Launch navigation mặc định kiểm tra `swerve_navigation/maps/warehouse.yaml` và
`warehouse.pgm`. Nếu dùng map mới, truyền YAML tương ứng:

```bash
ros2 launch swerve_bringup system.launch.py mode:=navigation \
  map_file:=/absolute/path/warehouse_map.yaml
```

## 8. Nếu cổng 8000 hoặc 5173 đang bị chiếm

Không kill service lạ. Dùng cổng khác trong lúc phát triển:

Backend:

```bash
cd waretwin/backend
BACKEND_PORT=8001 ./run.sh
```

Frontend:

```bash
cd waretwin/frontend
VITE_API_URL=http://127.0.0.1:8001 \
VITE_WS_URL=ws://127.0.0.1:8001/ws \
npm run dev -- --host 0.0.0.0 --port 5174
```

ROS bridge:

```bash
ros2 launch swerve_bringup system.launch.py \
  bridge_ws_url:=ws://127.0.0.1:8001/ws/ros
```

## 9. Chạy test/build

```bash
cd /home/yahboom/swerve_bringup
./scripts/build_ros.sh

cd waretwin/backend
.venv/bin/python manage.py test

cd ../frontend
npm run build
npm test
```

## 10. Lỗi thường gặp

| Lỗi | Cách xử lý |
|---|---|
| `Package 'gazebo_ros2_control' not found` | Cài `ros-humble-gazebo-ros2-control`. |
| `Package 'robot_localization' not found` | Cài `ros-humble-robot-localization`. |
| `pointcloud_to_laserscan` hoặc `slam_toolbox` not found | Cài hai package tương ứng trong bước 2. |
| Controller spawner timeout | Kiểm tra `ros2_control` packages và build lại bằng `scripts/build_ros.sh`. |
| Bridge báo `Invalid WebSocket Header` | Dùng source mới có bridge không gửi subprotocol `json`; build lại ROS. |
| Frontend `OFFLINE` | Kiểm tra backend URL, token login và `ros_connected`; đừng để `.env.local` trỏ IP cũ. |
| RViz thiếu `map` | Chờ SLAM nhận `/scan`, đặt Fixed Frame đúng (`map` khi mapping). |
| Build lấy nhầm Python/Cartoros2 | Không build bằng overlay cũ; chạy `source scripts/ros_env.sh` hoặc `scripts/build_ros.sh`. |
