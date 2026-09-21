# WareTwin API contract

Base URL is configured by `VITE_API_BASE_URL`; the frontend can derive it from
the browser hostname and `VITE_BACKEND_PORT` when the base is blank.

## Health and runtime

```http
GET /api/health/
GET /api/system/status/
GET /api/state
GET /api/map/sync-status
```

`/api/health/` returns `status`, `database`, `ros_bridge`, `websocket`, `ros`,
`gazebo`, `mode`, `runtime_state`, `timestamp`, `version` and measured
`components`. `/api/system/status/` adds CPU/RAM/disk/uptime and ROS nodes,
topics, controllers, runtime and sensor diagnostics. It never claims an
external ROS component is healthy when no bridge report exists.

## Authentication and WebSockets

```http
POST /api/auth/login
POST /api/auth/logout
GET  /api/auth/me
```

The browser WebSocket is `/ws?token=<session token>` and receives `FULL`,
`PATCH`, `RUNTIME_STATUS`, `LAYOUT_UPDATED`, map/schedule events and `ERROR`.
The ROS bridge is `/ws/ros?token=<WARETWIN_ROS_BRIDGE_TOKEN>`. The bridge token
is never committed and is never returned by diagnostics/logging.

## Canonical map and warehouse

```http
GET  /api/layout
GET  /api/layout/draft
PUT  /api/layout/draft
POST /api/layout/validate
POST /api/layout/publish
GET  /api/layout/versions
POST /api/warehouse/{id}/export/gazebo/
GET  /api/warehouses
GET/POST/PATCH/DELETE /api/warehouses/{id}
GET/POST/PATCH/DELETE /api/zones/{id}
GET/POST/PATCH/DELETE /api/shelves/{id}
GET  /api/warehouse-tree
POST /api/warehouse-sync/from-layout
```

The layout uses `warehouse_map`, metres and radians. Publish is optimistic-lock
protected by `X-Layout-Revision`, validates geometry, creates an immutable map
version and exports Gazebo/tag/navigation artifacts. The frontend must not
invent pixel-to-metre conversions or overwrite a deployed version.

Navigation landmarks in the canonical JSON use `tag_id`, `family` (`DATAMATRIX`,
`APRILTAG`, `QR` or another declared family), `size` in metres, `x/y/z`, `yaw`,
`floor_id`, `lane_id`, `zone_id` and free-form `metadata`. The relational tag
record stores the same values for mission routing and diagnostics.

`POST /api/warehouse/{id}/export/gazebo/` publishes the current validated draft
as an immutable revision and returns canonical JSON, the Gazebo world, map
metadata, tag graph and manifest paths. Passing `{"version": N}` only inspects
an existing immutable version. `start_stack.sh` automatically selects the
latest published world; an explicit `--world` overrides it. Gazebo Classic
cannot atomically hot-reload a world, so restart the ROS stack after publishing
new geometry.

## Orders, missions and fleet

```http
GET/POST /api/orders
GET/PATCH/DELETE /api/orders/{id}
POST /api/orders/{id}/cancel
GET/POST /api/schedules
GET /api/scheduler/overview
POST /api/scheduler/sync
GET /api/navigation/missions
POST /api/navigation/missions/start
POST /api/navigation/missions/{id}/{pause|resume|cancel|replan}
POST /api/robots/{robot_id}/emergency-stop
POST /api/robots/{robot_id}/clear-emergency-stop
```

Manual/autonomous control is sent over the authenticated browser WebSocket:

```json
{"type":"ROBOT_MODE","robot_id":"R01","mode":"MANUAL"}
{"type":"ROBOT_MANUAL","robot_id":"R01","action":"FORWARD"}
```

Actions are `FORWARD`, `BACKWARD`, `LEFT`, `RIGHT`, `ROTATE_LEFT`,
`ROTATE_RIGHT` and `STOP`. The ROS bridge applies a dead-man expiry (0.4 s by
default), rejects navigation while in manual mode, and returns
`ROBOT_CONTROL_STATUS`. The browser repeats held commands and sends `STOP` on
release/unmount.

Order/schedule/mission state is persisted. Assignment is performed by backend
scheduler/reservation services; React only requests an operation. E-stop
returns an error (503) if the external bridge did not acknowledge it.

## Error envelope

All new/updated API errors use:

```json
{
  "success": false,
  "error": {
    "code": "HTTP_409",
    "message": "human-readable message",
    "details": {}
  },
  "detail": "human-readable message"
}
```

`detail` is retained for existing frontend clients during migration.
