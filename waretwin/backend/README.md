# WareTwin Django Backend Base

> Để chạy profile tích hợp với Gazebo/ROS 2, xem
> [../../SETUP_A_Z.md](../../SETUP_A_Z.md). Backend hỗ trợ `LOCAL_SIM` và
> `GAZEBO_ROS`; profile đầy đủ dùng `GAZEBO_ROS` cùng node `swerve_bridge`.

Django backend base for the current WareTwin frontend. It is designed for the target architecture:

```text
Frontend (React)
      |
      | REST + WebSocket
      v
Central Backend (Django on PC)   <-- this project
      |
      | ROS bridge WebSocket (/ws/ros)
      v
swerve_bridge (ROS 2 node)
      |
      v
ROS2
```

`LOCAL_SIM` vẫn giữ `SimEngine` làm state provider cho frontend-only development.
Ở profile `GAZEBO_ROS`, state robot đến từ ROS bridge; frontend không tự chạy
browser simulation khi bridge mất kết nối.

## What is already supported

- Authentication: register, login, logout, current user.
- Admin user management: create, update, disable, delete, reset password.
- Django Channels WebSocket at `/ws`.
- `FULL`, `PATCH`, `HEATMAP`, `ERROR`, `COPILOT_REPLY`, `WHATIF_RESULT` messages.
- Simulation controls: play, pause, reset, speed.
- Robot/task/lift/zone/conveyor/camera/sensor/people/alert mock state.
- Create and manually assign tasks.
- Scenario injection and clear.
- Alert acknowledgement.
- KPI, decisions and event/audit history.
- Warehouse Editor draft save, validation, publish and layout versions.
- What-if simulation.
- Copilot rule-based fallback without any external AI API.
- VLM simulated observation without any external AI API.
- SQLite persistence for users, tokens, warehouse master data, orders, missions,
  reservations, audit/events and robot profiles.
- `RobotGateway` boundary for the Central Django -> authenticated ROS bridge -> ROS 2 path.

## Chưa nằm trong profile này

- A separate Robot Server HTTP API for physical hardware.
- Redis/Celery/multi-process state distribution; the included Channels layer is
  intentionally single-process for the Ubuntu development stack.

`LOCAL_SIM` remains an in-process UI/demo provider. In `GAZEBO_ROS` and
`REAL_ROBOT`, robot pose/status is accepted only from the authenticated ROS
bridge; a lost bridge marks robots offline and does not simulate movement.
The bridge registry routes commands by `robot_id`, while the default launch
still runs one robot for backward compatibility.

## Quick start

Ubuntu / Linux:

```bash
cd django_backend_base
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env
python manage.py migrate
python manage.py seed_demo
python manage.py runserver 0.0.0.0:8000
```

Or simply:

```bash
./run.sh
```

Development account from `.env.example`:

```text
username: admin
password: admin12345
```

Change this password before any non-local deployment.

## Frontend configuration

Change the frontend `.env` to:

```env
VITE_DEMO_MODE=false
VITE_BACKEND_PORT=8000
VITE_API_BASE_URL=
VITE_WS_BASE_URL=
```

Then run the frontend:

```bash
npm install
npm run dev
```

Open the frontend and log in with the development admin account.

When the base URLs are empty, the browser hostname plus
`VITE_BACKEND_PORT` is used automatically. Legacy `VITE_API_URL` and
`VITE_WS_URL` remain supported for existing installations.

## Main REST endpoints

```text
POST   /api/auth/register
POST   /api/auth/login
POST   /api/auth/logout
GET    /api/auth/me

GET    /api/admin/users
POST   /api/admin/users
GET    /api/admin/users/:id
PATCH  /api/admin/users/:id
DELETE /api/admin/users/:id
POST   /api/admin/users/:id/reset-password

GET    /api/health
GET    /api/health/
GET    /api/system/status/
GET    /api/map/sync-status
GET    /api/state
GET    /api/state/validate
GET    /api/kpi
GET    /api/events
GET    /api/decisions

GET    /api/layout
GET    /api/layout/draft
PUT    /api/layout/draft
POST   /api/layout/validate
POST   /api/layout/publish
GET    /api/layout/versions

POST   /api/sim
POST   /api/inject
POST   /api/inject/clear
POST   /api/tasks
POST   /api/tasks/:task_id/assign

POST   /api/copilot
POST   /api/vlm/observe
POST   /api/whatif
GET    /api/ai/status
```

## WebSocket client messages

The backend keeps the current frontend contract:

```text
RESYNC
SIM_CONTROL
INJECT
CLEAR_INJECTION
CREATE_TASK
ASSIGN_TASK
ACK_ALERT
SELECT_ROBOT
WHATIF_RUN
COPILOT_ASK
```

## WebSocket server messages

```text
FULL
PATCH
HEATMAP
WHATIF_RESULT
COPILOT_REPLY
ERROR
```

## Folder structure

```text
config/
  settings.py
  urls.py
  asgi.py

accounts/
  models.py
  management/commands/seed_demo.py

twin/
  consumers.py          # WebSocket contract
  runtime.py            # mock realtime twin runtime
  views.py              # frontend-compatible REST endpoints
  models.py             # events + future robot/missions
  schema.py             # TwinState contract
  warehouse_layout.json

  sim/                   # existing Python simulation engine
  ai/                    # local/rule-based Copilot + simulated VLM
  gateways/
    base.py
    robot_server.py      # future Central -> Robot Server integration point

layouts/
  draft.json             # created at runtime
  active.json            # created on publish
  layout-v*.json         # versions
```

## Runtime modes

React không nói chuyện trực tiếp với ROS2. Chọn state provider bằng
`WARETWIN_RUNTIME_MODE`:

```text
LOCAL_SIM
Frontend -> Django -> SimEngine

GAZEBO_ROS / REAL_ROBOT
Frontend -> Django -> swerve_bridge -> ROS2 -> robot/simulator
```

For LIVE mode, robot telemetry should become authoritative. The frontend must not invent position/state after communication loss; use `ONLINE / STALE / OFFLINE` and wait for telemetry before confirming a command state.


## Frontend API-connected build

Use the paired frontend. Its backend mode is authoritative: WebSocket loss shows
OFFLINE and does not fall back to a browser simulation. REST uses
`VITE_API_BASE_URL` and Channels uses `VITE_WS_BASE_URL`; the legacy aliases
remain backward compatible.

For another browser PC, add its Vite origin to `CORS_ALLOWED_ORIGINS` and set the frontend URLs to this Django PC IP.

## Warehouse / Zone / Shelf master data

The Django backend now includes persistent warehouse master data with the hierarchy:

`Warehouse 1 -> N Zone 1 -> N Shelf`

Main API endpoints (Bearer auth required; mutations require admin):

- `GET/POST /api/warehouses`
- `GET/PATCH/DELETE /api/warehouses/{id}`
- `GET/POST /api/zones`
- `GET/PATCH/DELETE /api/zones/{id}`
- `GET/POST /api/shelves`
- `GET/PATCH/DELETE /api/shelves/{id}`
- `GET /api/warehouse-tree`
- `POST /api/warehouse-sync/from-layout`

Deletion is protected: a warehouse with zones cannot be deleted, and a zone with shelves cannot be deleted. Shelf records contain physical position/size plus a robot access point (`access_x`, `access_y`, `access_yaw`).

After pulling this version run:

```bash
python manage.py migrate
python manage.py seed_demo
```

`seed_demo` also synchronizes the currently published layout into master data. With the bundled layout this creates 1 warehouse, 5 zones and 184 physical shelf/rack records. Existing matching records are updated geometrically without resetting their operational status/load.

Backend tests are in `twin/tests/test_warehouse_crud.py` and can be run with:

```bash
python manage.py test twin.tests.test_warehouse_crud
```
