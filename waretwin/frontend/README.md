# WareTwin Frontend

The frontend preserves the existing WareTwin Login, Overview, Robot Control,
and Robot Control Detail screens. Authenticated routes are `/`, `/control`,
and `/robots/:robotId/control`; `/login` is the guest entry point. Unknown
authenticated routes return to `/`.

The browser is a visualization and control client, never a robot simulator.
Runtime data flows from Gazebo through ROS 2 and the ROS bridge to Django and
its authenticated WebSocket, then into the existing frontend store and views.
When the backend is unavailable, live values remain unavailable; the browser
does not generate robot motion, sensor data, or telemetry.

Warehouse geometry is refreshed from Django's active layout on WebSocket
connect and after layout updates. Robot identity, pose, heading, online state,
map identity, LiDAR, and diagnostics come from backend/ROS reports. Missing
measurements are shown as unknown or unavailable rather than inferred.

## Development

```bash
npm ci
npm run dev
npm run build
npm test -- --run
```

Use `VITE_API_BASE_URL` and `VITE_WS_BASE_URL` when Django is not reachable at
the frontend's derived hostname and default backend port. Login uses Django
authentication; the WebSocket requires the authenticated session token.
