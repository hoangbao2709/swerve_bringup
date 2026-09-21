# Troubleshooting

## First response

```bash
./scripts/preflight_check.sh
./scripts/status_stack.sh
./scripts/smoke_test.sh
tail -f logs/{backend,frontend,ros}.log
```

Stop only the project stack with `./scripts/stop_stack.sh`. Do not use
`pkill -f python`, `pkill -f node` or a broad Gazebo kill; the launcher records
project process groups and leaves unrelated services alone.

## Setup failures

- **Node <20 / npm <10**: install Node 22 LTS, verify `node --version` and
  `npm --version`, then rerun setup.
- **Missing ROS package**: install the exact `ros-humble-*` package listed by
  preflight. The critical runtime packages are `gazebo_ros2_control`,
  `robot_localization`, `pointcloud_to_laserscan`, `slam_toolbox`, Nav2,
  controller manager and RViz.
- **sudo failure**: setup deliberately stops before apt when the account cannot
  authenticate. Install packages with an authorized Ubuntu account and rerun.
- **wrong overlay**: run `source scripts/ros_env.sh`; it removes unrelated ROS
  overlays before sourcing Humble and this workspace.

## Ports and LAN

8000 and 5173 are defaults, not assumptions. `start_stack.sh` reports the
owner, selects 8001/5174 or the next free port, and writes the selected values
to `.runtime/stack.env`. Frontend base URLs are blank by default, so the browser
uses its own hostname. For a reverse proxy or HTTPS deployment, set explicit
`VITE_API_BASE_URL` and `VITE_WS_BASE_URL`.

## Backend / WebSocket

- `/api/health/` with `ros_bridge=false` means the authenticated ROS bridge has
  not sent a heartbeat; it is not a Django database failure.
- Check that `WARETWIN_ROS_BRIDGE_TOKEN` matches the launch argument and that
  `ROS_WS_URL` points to `/ws/ros`, not the browser `/ws` endpoint.
- WebSocket states `RECONNECTING` and `DISCONNECTED` are expected during a ROS
  restart. Inspect backend and bridge logs; no token is printed.
- `MESSAGE_HANDLER_ERROR` identifies a bad bridge packet while preserving the
  connection. Fix the packet schema rather than disabling validation.
- If the Control page shows manual buttons disabled, verify the runtime is
  `GAZEBO_ROS`/`REAL_ROBOT`, the browser WebSocket is `CONNECTED`, and the
  Diagnostics page reports a live ROS bridge. `LOCAL_SIM` intentionally cannot
  drive a real robot.

## ROS / Gazebo / TF

```bash
source scripts/ros_env.sh
ros2 node list
ros2 topic list
ros2 control list_controllers
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_link
```

Check that `/clock`, `/odom`, `/odometry/filtered`, `/lidar/points`,
`/lidar/points_filtered` and `/scan` have data. In mapping, SLAM must be the
only `map -> odom` owner. In navigation, do not launch `slam.launch.py` in
parallel; the static map and localization path own the map correction.

## Mapping and navigation

- Missing `/map` in mapping: verify LiDAR → filter → scan and TF
  `base_link -> lidar_link`, then wait for SLAM to receive scans.
- Navigation rejects a map: use an absolute YAML path with a valid `image`,
  positive `resolution` and existing PGM; use `--map` in `start_stack.sh`.
- `warehouse.yaml` is intentionally guarded as a development placeholder. A
  measured saved map or a published canonical artifact must be supplied.
- `OUT_OF_SYNC` at `/api/map/sync-status` means publish/reconnect the same
  immutable revision before starting external missions. When a new canonical
  map is published, restart `start_stack.sh`: Gazebo Classic does not hot-reload
  world geometry while the process is running.

## Safety

If the robot does not stop, use the physical safety system first. Then verify
`/emergency_stop` and `/cmd_vel`:

```bash
ros2 topic echo /emergency_stop
ros2 topic echo /cmd_vel
```

The controller command timeout is a second stop mechanism. An API response is
successful only when the bridge accepted the E-stop command; offline requests
return HTTP 503 and do not mark a mission stopped.
