# WareTwin architecture

## Runtime graph

```text
Browser (React/Vite)
  ├─ REST /api/* ───────────────► Django + Channels
  └─ WebSocket /ws ◄─────────────┘
                                    └─ authenticated /ws/ros
                                       ▲
ROS 2 swerve_bridge ◄─────────────────┘
  ├─ /cmd_vel, /emergency_stop ─► controller
  ├─ /odometry/filtered ─────────► telemetry
  ├─ /lidar/points ─► filter ─► /scan ─► SLAM/Nav2
  └─ /clock, controller, TF ────► diagnostics
             │
             └─ Gazebo Classic + ros2_control + sensors + RViz2
```

The browser never publishes ROS messages directly. Django is the authenticated
command and persistence boundary; the ROS bridge is the only ROS process that
opens `/ws/ros`. A bridge packet is isolated at the consumer boundary, logged,
validated and cannot take down the connection by itself.

Robot control follows the same boundary: the Control page sends mode changes
and held manual actions over `/ws`; Django validates and forwards them to the
bridge; the bridge owns the dead-man timer and publishes bounded
`/cmd_vel_manual`. Nav2 and tag approach publish on separate inputs. The
`command_arbiter` selects one fresh input according to the latched
MANUAL/AUTONOMOUS mode and tag-route state, then publishes
`/cmd_vel_selected` to the swerve controller. Autonomous navigation is rejected
while manual mode is active, and E-STOP overrides every source through the real
ROS safety input.

## Runtime modes

| Mode | Robot motion source | Map owner | Expected graph |
|---|---|---|---|
| `LOCAL_SIM` | Django `SimEngine` | local layout | useful for UI-only development |
| `MAPPING` | Gazebo/controller | SLAM Toolbox (`map -> odom`) | Gazebo, EKF, LiDAR, `/scan`, SLAM |
| `NAVIGATION` | Gazebo/controller/Nav2 | saved map + localization | Gazebo, EKF, LiDAR, map server, Nav2 |
| `ERROR` | none | none | entered on external bridge loss/fault |

`system.launch.py` accepts only `mapping` or `navigation` and passes
`use_sim_time` to every relevant node. It does not start SLAM and Nav2 as two
competing map owners. Navigation rejects an invalid selected map before nodes
are created.

## Canonical warehouse data

The database-backed draft layout is the source for floors, polygon holes, walls,
shelves, stations, conveyors, charging/waiting/restricted zones, lanes,
waypoints, navigation tags and graph edges. Publishing validates the draft and
creates an immutable `WarehouseMapVersion` artifact directory:

```text
generated/maps/<warehouse-code>/<revision>/
  canonical_map.json
  datamatrix_map.yaml
  tag_graph.yaml
  manifest.json
  gazebo/warehouse.world
```

Web JSON, Gazebo SDF and ROS tag/navigation metadata come from this one
published revision. Draft changes cannot overwrite a deployed version. The
bridge acknowledges the revision and the backend exposes `SYNCED`,
`OUT_OF_SYNC`, `ROS_OFFLINE` or `ERROR` at `/api/map/sync-status`.

`start_stack.sh` asks Django for the active published artifact and launches its
Gazebo world, DataMatrix map and tag graph automatically. `--world` can select
an explicit development/test world. Since Gazebo Classic has no atomic live
world reload, publishing new geometry requires a ROS/Gazebo restart.

## Persistence and scheduling

Orders, inventory, work points, robot profiles, schedules, stops, missions,
events and reservations are Django models. The scheduler selects eligible
robots server-side (nearest available is the current strategy), persists the
dispatch and reserves graph edges. React does not contain assignment logic.
The runtime reloads database state after a backend restart; transient ROS
telemetry is intentionally marked offline until a fresh heartbeat arrives.

## Safety and recovery

- Controller command timeout is 0.5 s; stale commands become zero velocity.
- Emergency stop publishes zero `Twist` and a latched-in-process stop flag to
  the controller; a fresh command is required after clearing the stop.
- Bridge reconnect uses exponential backoff up to 30 s and sends heartbeat and
  diagnostics. Frontend reconnection states are rendered as real state.
- Stack scripts store only project PIDs/process groups under `.runtime/` and
  never kill arbitrary `python`, `node`, Gazebo or ROS processes.
- Health endpoints report measured external state. An offline bridge is not
  converted into a fake green ROS/Gazebo result.

## Multi-robot boundary

`RobotProfile`, `RobotEndpoint`, namespaced identity and per-robot telemetry are
represented in the persistence/API boundary. The Django bridge registry routes
commands by `robot_id`, so one authenticated bridge can be connected per robot
and a published map is broadcast to all of them. The current default simulation
launches one robot (`R01`); a full multi-Gazebo launch still requires every
robot-side producer (controller, odometry, sensors, TF and Nav2 action server)
to use the same ROS namespace before it is considered production-ready.
