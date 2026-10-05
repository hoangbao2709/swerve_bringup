# Robot Control backend

The `robot-real-sim` profile keeps the browser and backend control path shared
with a future real robot. Django owns authentication, control APIs, map identity,
runtime aggregation, and the authenticated WebSocket. ROS 2 is authoritative for
robot movement and telemetry; the browser never acts as a simulator.

```text
Browser → Django REST / WebSocket → ROS bridge → ROS 2
        → Command Arbiter / Nav2 → Robot Interface → Gazebo
```

`GAZEBO_ROS` is the simulation runtime selected by `scripts/start_stack.sh`.
`REAL_ROBOT` uses the same upper layers and replaces only the Gazebo hardware
interface. The configured runtime modes do not include the old `LOCAL_SIM`
profile.

## Control-facing endpoints

- `POST /api/auth/login`, `GET /api/auth/me`, `POST /api/auth/logout`
- `GET /api/health/`, `GET /api/system/status/`
- `GET /api/state`, authenticated Channels WebSocket at `/ws`
- `GET /api/layout`, `GET /api/map/sync-status`
- Robot-local map, localization, and runtime controls under `/api/robots/:id/local/`
- Robot navigation-tag registry and Emergency Stop / Clear Stop under
  `/api/robots/:id/` and `/api/navigation/`
- Robot commands, telemetry, path previews, and navigation goals over `/ws`
- Authenticated ROS bridge at `/ws/ros`

Navigation is rejected unless the backend confirms the active map identity and
revision, runtime readiness, applied control mode, fresh localization, and a
clear Emergency Stop state. The WebSocket carries runtime state to the UI and
manual commands back through the ROS bridge; the command arbiter and its
watchdog remain authoritative.

## Local development

Use the repository-level setup and runtime so the backend, ROS workspace, map
artifacts, and Gazebo launch use one configuration:

```bash
cd ../..
./scripts/setup_full_stack.sh
./scripts/start_stack.sh unified --gui --rviz
```

The unified launch starts SLAM Toolbox and Nav2. The active canonical map is
read from the database and its generated revision bundle; do not substitute a
checked-in development map when runtime synchronization is unavailable.

Run backend tests from this directory with:

```bash
./.venv/bin/python manage.py test twin.tests
```
