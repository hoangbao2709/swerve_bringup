# WareTwin Django Backend Base

Django backend base for the current WareTwin frontend. It is designed for the target architecture:

```text
Frontend (React)
      |
      | REST + WebSocket
      v
Central Backend (Django on PC)   <-- this project
      |
      | future Robot Server API
      v
Robot Server (Django/Python on robot)
      |
      v
ROS2
```

At this stage **no Robot Server or ROS2 API is connected**. The existing Python `SimEngine` is kept as the local/mock state provider so the frontend can exercise its complete UI contract before hardware integration.

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
- SQLite persistence for users, tokens, audit/events, future robot endpoints and missions.
- Future `RobotGateway` abstraction for Central Django -> Robot Server -> ROS2.

## Not connected yet

- Robot Server HTTP/WebSocket API.
- ROS2 topics/actions/services.
- Real robot telemetry.
- Real robot commands.
- Redis/Celery/multi-process state distribution.

The current runtime is intentionally **single-process and in-memory**. This is appropriate for frontend/backend development. When LIVE robot integration begins, replace the mock runtime source with `RobotServerGateway` and move realtime state to the production architecture.

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
VITE_WS_URL=ws://127.0.0.1:8000/ws
```

Then run the frontend:

```bash
npm install
npm run dev
```

Open the frontend and log in with the development admin account.

The frontend derives the REST base URL from `VITE_WS_URL`, so:

```text
ws://127.0.0.1:8000/ws
```

becomes:

```text
http://127.0.0.1:8000
```

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

## Future LIVE mode

Do not make React talk directly to ROS2. Keep the frontend contract stable and change only the backend state provider:

```text
Current development
Frontend -> Django -> SimEngine

Future live system
Frontend -> Django -> RobotServerGateway -> Robot Django/Python -> ROS2 -> Robot
```

For LIVE mode, robot telemetry should become authoritative. The frontend must not invent position/state after communication loss; use `ONLINE / STALE / OFFLINE` and wait for telemetry before confirming a command state.


## Frontend API-connected build

Use the paired `frontend_api_connected` package. Its backend mode is authoritative: WebSocket loss shows OFFLINE and does not fall back to a browser simulation. REST uses `VITE_API_URL`; Channels uses `VITE_WS_URL`.

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
