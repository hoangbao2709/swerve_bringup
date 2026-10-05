# Django runtime connection

Django is the frontend's runtime authority. The local frontend calls Django
REST endpoints and consumes runtime state over its WebSocket without a browser
user identity. It does not fall back to local simulation when disconnected.

## Runtime path

```text
Gazebo → ROS 2 → ROS bridge → Django/WebSocket → frontend store and views
```

The bridge reports robot telemetry, active-map pose, ROS/TF diagnostics, map
and LiDAR snapshots, and navigation results. Manual commands, E-stop, and
validated Nav2 path-preview/goal requests flow back through Django and the ROS
bridge. The browser does not generate motion or synthetic telemetry.

## Configuration

- `VITE_API_BASE_URL`: optional REST base URL.
- `VITE_WS_BASE_URL`: optional WebSocket base URL.
- `VITE_BACKEND_PORT`: backend port when the hostname is derived from the
  browser location.

REST and WebSocket requests do not send user credentials or browser tokens.
Backend disconnection leaves last-confirmed state visible and marks the runtime
offline; unknown measurements remain unknown. The separate ROS bridge
connection keeps its robot-service handshake.

## Development

```bash
# backend
python manage.py migrate
python manage.py runserver 127.0.0.1:8000

# frontend
npm ci
npm run dev
```
