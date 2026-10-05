# Robot Control UI

This frontend is a live control client, not a robot simulator. Runtime state is
received from Django/ROS; if the connection is lost, the UI shows unavailable
or last-confirmed data and sends STOP when an active manual command owner exits.
Motion commands never update a browser-side robot pose.

The only application routes are:

- `/login`
- `/control` — runtime-backed robot overview
- `/robots/:robotId/control` — selected robot, live map/LiDAR, manual and
  autonomous controls, safety state, and robot-local runtime tools

```text
Browser → Django REST / WebSocket → ROS bridge → ROS 2
        → Command Arbiter / Nav2 → Robot Interface → Gazebo
```

## Run

For the integrated Gazebo + ROS runtime, use the repository-level setup and
launcher from the project root:

```bash
./scripts/setup_full_stack.sh
./scripts/start_stack.sh unified --gui --rviz
```

For frontend-only iteration against an already-running backend, install
dependencies and run Vite:

```bash
npm ci
npm run dev
```

Set `VITE_BACKEND_PORT` if Django is on a non-default port. Optional
`VITE_API_BASE_URL` and `VITE_WS_BASE_URL` allow an explicit host or reverse
proxy. There is no demo-mode or local movement fallback.

## Verify

```bash
npm run build
npm test
```

Tests cover authentication/route guards, dynamic robot selection, manual command
semantics, map identity and revision gates, LiDAR views, path preview, and
Emergency Stop fail-closed behavior.
