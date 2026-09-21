# Setup A–Z: WareTwin + ROS 2 + Gazebo + RViz

Tài liệu này chạy profile tích hợp:

```text
React/Vite :5173 (hoặc fallback tự động)
      │ REST + WebSocket /ws
Django/Channels :8000 (hoặc fallback tự động)
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
- Python 3.10+, `python3-venv`, Node.js 20+ (khuyến nghị Node 22 LTS) và npm
  10+.
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
  ros-humble-controller-manager \
  ros-humble-robot-localization \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-controller ros-humble-nav2-planner ros-humble-nav2-map-server \
  ros-humble-nav2-behaviors ros-humble-nav2-bt-navigator \
  ros-humble-nav2-waypoint-follower ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-msgs ros-humble-ros2-control \
  ros-humble-rviz2 \
  ros-humble-tf2-tools ros-humble-diagnostic-updater \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  python3-colcon-common-extensions python3-rosdep python3-venv \
  python3-yaml python3-websocket ripgrep curl lsof
```

Các package bắt buộc nhất cho Gazebo là `gazebo_ros2_control` và
`ros2-controllers`; cho pipeline cảm biến là `robot_localization`,
`pointcloud_to_laserscan`, `slam_toolbox`.

## 3. Cài đặt tự động từ máy Ubuntu sạch

Lệnh dưới đây có thể chạy lại an toàn. Nó kiểm tra Ubuntu/ROS/Python/Node/npm,
cài package Ubuntu còn thiếu, chạy `rosdep`, tạo virtualenv, migrate/seed
database, cài và build frontend, build ROS, rồi chạy preflight. Script ghi log
ở `logs/setup.log`.

```bash
cd /home/yahboom/swerve_bringup
./scripts/setup_full_stack.sh
./scripts/preflight_check.sh
```

Nếu preflight báo thiếu package ROS mà máy không cho `sudo` không mật khẩu, cài
theo phần 2 rồi chạy lại; script không bỏ qua dependency.

### Chạy toàn stack bằng một lệnh

```bash
./scripts/start_stack.sh mapping --headless
./scripts/status_stack.sh
./scripts/smoke_test.sh
./scripts/stop_stack.sh
```

Các lựa chọn hỗ trợ: `navigation`, `--headless`, `--no-rviz`,
`--no-gazebo-gui`, `--backend-port PORT`, `--frontend-port PORT`, và
`--map /absolute/path/map.yaml`, `--world /absolute/path/world.sdf`,
`--robot-id R01`, `--namespace robot_2`. Khi 8000 hoặc 5173 bị chiếm, launcher chỉ
chọn fallback (8001/5174 trở lên), ghi URL đã chọn vào `.runtime/stack.env`
và truyền đúng URL đó cho frontend cùng ROS bridge. Nó không kill process lạ.

Sau khi Warehouse Editor publish một revision, launcher tự tìm artifact đang
active và dùng cùng revision cho `gazebo/warehouse.world`, `datamatrix_map.yaml`
và `tag_graph.yaml`. `--world` ghi đè lựa chọn tự động cho test/dev. Gazebo
Classic không hot-reload world nguyên tử, nên hãy restart stack sau khi publish
geometry mới.

Ví dụ launch một bridge namespaced cho robot thứ hai (các node ROS producer
khác cũng phải dùng cùng namespace trước khi chạy fleet thật):

```bash
./scripts/start_stack.sh navigation --robot-id R02 --namespace robot_2 \
  --map /absolute/path/map.yaml --headless
```

`status_stack.sh` và `/api/system/status/` báo trạng thái đo được; ROS/Gazebo
không kết nối thì hiện `DISCONNECTED`, không được coi là xanh giả.

## 4. Chạy script setup một lần

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

## 5. Cấu hình backend và token bridge

`waretwin/backend/.env` phải có ít nhất:

```env
WARETWIN_RUNTIME_MODE=GAZEBO_ROS
WARETWIN_ROS_BRIDGE_TOKEN=đặt-một-token-chung
WARETWIN_ARTIFACT_ROOT=../generated/maps
CORS_ALLOWED_ORIGINS=http://localhost:5173,http://127.0.0.1:5173
TWIN_ADMIN_USERNAME=admin
TWIN_ADMIN_PASSWORD=admin12345
```

`WARETWIN_ROS_BRIDGE_TOKEN` phải giống token truyền cho ROS launch. Không commit
`.env` lên Git. Các biến chính nằm trong `waretwin/backend/.env.example` và
`waretwin/frontend/.env.example`: `BACKEND_HOST`, `BACKEND_PORT`,
`FRONTEND_HOST`, `FRONTEND_PORT`, `ROS_DOMAIN_ID`, `ROS_WS_URL`,
`WARETWIN_ROS_BRIDGE_TOKEN`, `VITE_API_BASE_URL`, `VITE_WS_BASE_URL`.
Để frontend tự dùng hostname mà trình duyệt đang mở (kể cả truy cập LAN), để
hai biến `VITE_*_BASE_URL` trống và chỉ đặt `VITE_BACKEND_PORT` nếu cần.

Sau khi đổi `.env`, chạy lại migrate/seed nếu cần:

```bash
cd waretwin/backend
.venv/bin/python manage.py migrate
.venv/bin/python manage.py seed_demo
```

## 6. Mở 4 terminal thủ công (debug chi tiết)

### Terminal 1 — Django backend

```bash
cd /home/yahboom/swerve_bringup/waretwin/backend
./run.sh
```

Backend chạy tại `BACKEND_HOST:BACKEND_PORT` và WebSocket tại `/ws` cùng
`/ws/ros`. Nếu dùng `start_stack.sh`, đọc URL thực tế từ
`.runtime/stack.env` hoặc output của script.

Kiểm tra:

```bash
curl http://127.0.0.1:8000/
curl http://127.0.0.1:8000/api/health/
curl http://127.0.0.1:8000/api/system/status/
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

## 7. Test frontend ↔ backend ↔ ROS

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

## 8. Mapping rồi chạy navigation

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

`system.launch.py` truyền `map_file` xuyên suốt xuống
`swerve_navigation/launch/navigation.launch.py` và `nav2_map_server`. YAML
được kiểm tra tại thời điểm launch, bao gồm image PGM, resolution và kích
thước; map custom hợp lệ không bị thay bằng `warehouse.yaml`.

Trên trang Robot Control, chuyển `AUTONOMOUS`/`MANUAL` trước khi gửi lệnh.
Manual dùng W/A/S/D, phím mũi tên hoặc nút giữ trên màn hình; bridge lặp lệnh
trong lúc giữ và tự gửi zero sau 0.4 giây mất heartbeat (dead-man). Không thể
gửi manual và Nav2 đồng thời; `EMERGENCY STOP` đi qua backend tới ROS thật.

Map Web canonical được lưu qua Warehouse Editor → **Save & Sync** → **Validate**
→ **Publish**. Bản publish tạo version bất biến gồm JSON, tag graph, map
metadata và Gazebo world; backend gửi revision sang bridge. Mission external
chỉ chạy khi ROS/Gazebo cùng revision (`SYNCED`).

## 9. Nếu cổng 8000 hoặc 5173 đang bị chiếm

Không kill service lạ. Dùng cổng khác trong lúc phát triển:

Backend:

```bash
cd waretwin/backend
BACKEND_PORT=8001 ./run.sh
```

Frontend:

```bash
cd waretwin/frontend
VITE_BACKEND_PORT=8001 \
VITE_API_BASE_URL= \
VITE_WS_BASE_URL= \
npm run dev -- --host 0.0.0.0 --port 5174
```

ROS bridge:

```bash
ros2 launch swerve_bringup system.launch.py \
  bridge_ws_url:=ws://127.0.0.1:8001/ws/ros
```

## 10. Chạy test/build

```bash
cd /home/yahboom/swerve_bringup
./scripts/build_ros.sh

cd waretwin/backend
.venv/bin/python manage.py test

cd ../frontend
npm run build
npm test

cd ../..
./scripts/smoke_test.sh
```

## 11. Health, diagnostics và recovery

- `/api/health/` là health contract ngắn: database, WebSocket, ROS bridge,
  ROS, Gazebo, mode và timestamp.
- `/api/system/status/` lấy CPU/RAM/disk/uptime và graph diagnostics được bridge
  đo từ ROS; khi bridge offline các trường ROS để false/null.
- Frontend **Diagnostics** hiển thị `CONNECTING`, `CONNECTED`, `RECONNECTING`,
  `DISCONNECTED`, `ERROR`, sensor age/frequency, TF, controllers, nodes/topics.
- `EMERGENCY STOP` gửi lệnh ROS thật; controller chặn command và publish zero
  velocity cho tới khi có command mới/clear state. Nếu bridge offline API trả
  lỗi, không đánh dấu UI là đã dừng.
- Sau crash: `./scripts/status_stack.sh`, đọc `logs/*.log`, rồi
  `./scripts/stop_stack.sh`; process group thuộc project được dọn, process
  ngoài project không bị đụng tới.

## 12. Lỗi thường gặp

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
| `Node.js >=20 required` | Cài Node 22 LTS (`nvm install 22 && nvm use 22`), kiểm tra `node --version` rồi chạy preflight. |
| `map_file` không hợp lệ | Kiểm tra YAML có `image`, `resolution`, file PGM cùng thư mục và truyền đường dẫn tuyệt đối. |
| Port fallback không đồng bộ | Không tự mở frontend/backend riêng; dùng `start_stack.sh` hoặc cập nhật cùng `BACKEND_PORT`, `VITE_BACKEND_PORT`, `ROS_WS_URL`. |
| `/api/health/` ROS false | Đây là trạng thái thật khi bridge chưa kết nối; kiểm tra token, URL `/ws/ros`, `ROS_DOMAIN_ID`, rồi xem `logs/ros.log`. |
| `sudo` bị từ chối trong setup | Đăng nhập account có quyền apt hoặc cài package thủ công, sau đó chạy lại `./scripts/setup_full_stack.sh`. |
