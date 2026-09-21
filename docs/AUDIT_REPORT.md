# WareTwin / AGV Digital Twin audit

Audit được thực hiện trên branch `web-simulation` trước khi sửa. Các mục dưới
đây ghi lại nguyên nhân gốc và trạng thái sau khi sửa; đây không phải là danh
sách các lỗi được che bằng UI.

## Đã xác nhận và đã sửa

| Khu vực | Lỗi/nguy cơ | Xử lý |
| --- | --- | --- |
| Setup | `backend/run.sh` không bảo đảm executable; setup chỉ kiểm tra `command -v node` | Giữ mode Git `100755`; kiểm tra Node/npm bằng version thực tế, yêu cầu Node >=20/npm >=10 và khuyến nghị Node 22 LTS. |
| Git | ROS build/log, Python cache, `.env`, frontend dependencies và database runtime bị theo dõi | Mở rộng `.gitignore`; gỡ generated/runtime artifacts khỏi Git index nhưng giữ file local để không phá môi trường hiện tại. |
| Environment | URL, port và bridge token không có contract thống nhất | Chuẩn hóa `.env.example`, backend/frontend URL, `ROS_WS_URL`, `VITE_*`, token và runtime mode; Vite tự dùng hostname trình duyệt cho truy cập LAN. |
| Port | Fallback port có thể làm frontend vẫn gọi backend cũ | `start_stack.sh` chọn port rảnh, ghi `.runtime/stack.env`, truyền cùng URL cho frontend và ROS bridge; không tự kill process ngoài stack. |
| ROS launch | `map_file` bị hard-code và validation kiểm tra map mặc định | `map_file` được declare ở system/navigation launch, resolve/validate đúng file operator truyền vào và truyền tới Nav2 map server. |
| Runtime mode | Mapping và navigation có thể cùng chạy; thiếu state rõ ràng | Guard `mapping`/`navigation`, SLAM và Nav2 mutually exclusive; state `IDLE/SIMULATION/MAPPING/NAVIGATION/ERROR` đi qua bridge và WebSocket. |
| TF/time | Có nguy cơ nhiều owner cho `map->odom`/`odom->base`; `use_sim_time` không nhất quán | SLAM hoặc V30E sở hữu `map->odom`, EKF sở hữu `odom->base_footprint`, odometry không publish TF; các node bringup nhận `use_sim_time`. |
| Gazebo | Spawn/controller ordering và mesh path phụ thuộc máy phát triển | Serialize spawn trước controller spawners; resolve robot từ canonical manifest; đổi mesh sang `package://`. |
| Controller | Thiếu timeout/dead-man, limit và e-stop thật | Thêm velocity/acceleration/steering limits, command timeout, latch e-stop và safe shutdown; web e-stop gửi lệnh ROS thật. |
| ROS bridge | Một message lỗi có thể làm bridge chết; reconnect/token/rate-limit chưa đủ | Thêm authenticated connection manager, exponential backoff, heartbeat, validation, rate limiting, isolation/logging, robot registry theo `robot_id`, manual/autonomous/pause/resume/replan. |
| Health | Có trạng thái xanh giả khi ROS/Gazebo chưa chạy | `/api/health/` và `/api/system/status/` dùng DB, heartbeat và diagnostics đo được; mất ROS trả `false`/`DISCONNECTED`. |
| Map | Web/Gazebo/ROS dùng các artifact có thể lệch revision | Canonical map publish tạo revision bất biến, manifest/hash, Gazebo SDF, map YAML, tag graph; bridge ACK `SYNCED`/`OUT_OF_SYNC`. |
| Warehouse | Editor và tag metadata chưa đủ contract backend | Đồng bộ polygon/holes/aisle/waypoint/edge/tag; thêm `NavigationTag.family/size/z/lane/zone/metadata` và export Gazebo tag có kích thước thực. |
| Backend | Error response không đồng nhất; map export rollback chưa chặt | Chuẩn hóa error envelope, validate state, rollback artifact/version khi export lỗi, thêm health/diagnostic endpoints. |
| Frontend | WebSocket reconnect đơn giản; control panel có health hard-code | Reconnect exponential backoff + jitter/cleanup; Robot Control và Diagnostics chỉ hiện trạng thái ROS đã đo, chưa đo thì `UNKNOWN`. |
| Persistence | Workpoint seed không phản ánh layout cũ; mission/control status không đồng nhất | Backfill workpoints từ station/charging/parking/shelf; migration `0010_navigationtag_landmark_metadata`; mapping status ROS ↔ DB `CANCELLED/PAUSED/PLANNING`. |
| Quality | Thiếu script lifecycle/test/docs | Thêm preflight, start/stop/status, smoke test, tài liệu architecture/API/topics/coordinates/troubleshooting và audit này. |

## Nguy cơ còn có chủ ý / cần môi trường đầy đủ

- `Channels` đang dùng in-memory layer cho Ubuntu development single-process; môi
  trường production nhiều process cần Redis hoặc channel layer phân tán.
- DDS không cung cấp số sample LiDAR bị drop qua API hiện dùng; diagnostics trả
  `null` cho metric này thay vì bịa số 0. Frequency, age, frame và timestamp vẫn
  được đo.
- Namespace multi-robot đã có contract bridge/registry/topic, nhưng mọi node
  producer Gazebo/controller/TF phải được launch cùng namespace trước khi chạy
  fleet thật; graph multi-robot đầy đủ chưa thể xác nhận trên máy hiện tại.
- Robot telemetry realtime là in-memory cache; order/mission/schedule/map/event
  được lưu DB. High-rate `RobotState` history chưa được thêm để tránh phình DB.

## Blocker môi trường hiện tại

Preflight còn fail vì máy đang thiếu các package ROS Humble:

```text
gazebo_ros2_control robot_localization pointcloud_to_laserscan
slam_toolbox nav2_bringup
```

`setup_full_stack.sh` đã phát hiện và cố cài đúng package nhưng phiên hiện tại
không có sudo authentication tương tác (`sudo: a terminal is required`). Vì vậy
Gazebo/SLAM/Nav2 integration smoke test chưa thể coi là pass; không được diễn đạt
thành “đã chạy end-to-end” cho đến khi cài package và chạy lại test.
