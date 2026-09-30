# Local Robot Control Architecture

The Robot Control Detail route (`/robots/:robotId/control`) is the local HMI
for one selected robot. Control, Mapping, Localization, VDA5050, and
Diagnostics stay inside `RobotControlDetailPage`. Every REST and WebSocket
operation is robot-scoped. The browser consumes processed bridge frames; it
does not subscribe to ROS, parse PointCloud2, publish `/cmd_vel`, or plan a
route independently.

```text
React Robot Control Detail (R01)
        | REST + authenticated WebSocket
        v
WareTwin Django API / Channels runtime
        | robot_id-scoped command or bridge RPC
        v
Authenticated ROS Bridge (R01)
        |
        +-- Nav2 / SLAM / TF-localization / LiDAR
        +-- command arbiter / VDA5050 adapter
        |
        v
ros2_control -> simulation or physical robot runtime
```

Mode transitions are selected by a runtime adapter behind the REST contract.
The simulation adapter uses the owned stack supervisor and its readiness and
rollback gate. A physical adapter must manage only its configured robot ROS
processes; it must not start or restart Gazebo. The UI says “runtime adapter”
and does not encode Gazebo behavior. `WARETWIN_REAL_RUNTIME_ADAPTER` is the
deployment plug-in entry point. No physical runtime manager is supplied or
configured in this checkout; without one, physical mode transitions fail
closed with `REAL_RUNTIME_ADAPTER_UNAVAILABLE` instead of launching unmanaged
ROS or Gazebo processes.

## Robot detail sections

- **CONTROL** provides Global Map and LiDAR Map 2D/3D, measured pose and
  heading, goal selection, the planner-generated path preview, Send Goal, and
  MANUAL teleop. These are views of one robot and one active map.
- **MAPPING** calls robot-scoped mapping and map APIs. The backend updates the
  visible state only after the bridge confirms Start/Stop. Save registers an
  artifact only after a valid image and YAML exist. Runtime-mode transitions
  are adapter-owned; start/stop of SLAM does not kill unrelated processes.
- **LOCALIZATION** shows the measured map-frame pose and applies a selected
  initial pose through the existing `/ekf_v30e/set_pose` interface. This
  preserves `ekf_v30e` as the single `map -> odom` authority.
- **VDA5050** stores one configuration per robot. MQTT passwords are encrypted
  at rest and API responses expose only `password_configured`. Test Connection
  performs a backend broker attempt. Save & Apply persists then applies the
  adapter configuration and rolls persisted values back if an enabled broker
  cannot be connected. `allow_task=false` removes the order subscription and
  rejects queued orders while leaving authorized local teleop, telemetry,
  heartbeat, and diagnostics available. Instant-action execution is explicitly
  `NOT_IMPLEMENTED`: it is forced off, not subscribed to, and not forwarded.
- **DIAGNOSTICS** includes ROS, runtime, Gazebo where applicable, bridge,
  WebSocket, controllers, Nav2, SLAM, localization, TF, LiDAR, map identity and
  synchronization, command ownership, MQTT/VDA5050, and runtime errors. Secrets
  are excluded.

## Map views and ROS-to-Web rendering

The Global Map overlays the active robot occupancy map on warehouse context
when the active map is canonical. A local-only map is clearly labelled and
canonical warehouse overlays are hidden. Robot pose, heading, footprint, goal,
planner path, current navigation path, and available occupancy/costmap data
use map-frame metres. Revision and synchronization state remain visible.

The Python bridge owns sensor parsing and TF:

```text
/scan or PointCloud2
      -> ROS callback
      -> TF to configured visualization frame
      -> finite/range/height filtering and bounded sampling
      -> numeric web frame with timestamp and frame metadata
      -> Django WebSocket
      -> 2D canvas or 3D WebGL
```

LiDAR 2D uses `/scan`, removes invalid ranges, converts polar samples to XY,
transforms with actual TF, and caps the web output independently of ROS sensor
fidelity. LiDAR 3D prefers `/lidar/points_filtered` and falls back to
`/lidar/points` only when the filtered input is stale. Python parses and
transforms PointCloud2, removes non-finite/out-of-range points, applies height
and voxel filtering, and enforces a point budget.

Global OccupancyGrid snapshots are also bounded before transport. The bridge
encodes occupancy values as `value + 1` bytes, zlib-compresses them, and sends
the compact `zlib-base64-offset1` representation (up to four million cells).
The browser asynchronously decodes and validates the exact grid size before
painting; it never receives a large integer-per-cell JSON array. The bridge
replays its cached map when Global Map is selected, so opening the view after
the initial ROS map publication does not leave it blank.

For 3D, a one-slot latest-frame buffer separates ROS callbacks from WebSocket
I/O. One sender worker serializes frames; a new unsent frame replaces stale
pending data. Disconnect and view changes clear pending frames. Diagnostics
include source/send timestamps, frame ID, point count, source/output FPS, and
dropped-frame count. Browser JSON remains compact and bounded. ROS horizontal
samples, vertical channels, and sensor update rate are unchanged.

## Nav2 path preview and authorization

```text
map click
  -> PATH_PREVIEW_REQUEST(robot, target, active map ID/revision)
  -> Django identity/readiness gate
  -> R01 bridge
  -> Nav2 ComputePathToPose
  -> real map-frame nav_msgs/Path
  -> same path projected on Global, LiDAR 2D, and LiDAR 3D
  -> operator presses Send Goal
  -> Django validates one-time preview authorization
  -> NavigateToPose
```

Nav2's configured planner/costmap is the source of truth; this is its selected
lowest-cost path, not a claim of mathematical shortest distance. Preview never
sends a navigation action. Send Goal requires a non-empty valid preview ID
belonging to this robot, with the same target and active map identity/revision,
and an unexpired approval. Invalid, expired, map-mismatched, or no-path results
are rejected with explicit errors. There is no silent recomputation at execute
time. A map change invalidates existing approvals.

Global Map draws the canonical map-frame path directly. LiDAR 2D transforms
that same path to its selected base visualization frame; LiDAR 3D projects its
XY coordinates to the ground plane. The path is not recalculated per view.

## Local active maps and mapping lifecycle

`LOCAL ACTIVE MAP` is distinct from `FLEET/CANONICAL MAP`. The runtime status
exposes local active map ID/revision, canonical revision, and `LOCAL_ONLY`,
`SYNCED`, `OUT_OF_SYNC`, or `CANONICAL` status. A robot may save a map, load it
into its Nav2 map server, initialize localization on it, and navigate locally
without publishing it to Fleet. Fleet schedules/tag missions remain tied to
the canonical warehouse map and are blocked for the robot using a local-only
map.

Saved maps are under the configured artifact root in
`local_robot_maps/<robot_id>`. The registry uses opaque IDs and records name,
robot, creation time, resolution, origin, content revision/hash, and image
dimensions; it does not reveal host paths. Names and resolved artifact paths
are checked against traversal. Load requires a stopped MANUAL robot and a
validated artifact. The bridge calls `map_server/load_map`, then confirms the
new `/map` dimensions, resolution, origin, and cell data before reporting the
local map active. Motion remains blocked during that transition.

Mapping workflow states describe actual operation transitions:
`IDLE -> STARTING -> MAPPING -> STOPPING -> SAVING/LOADING -> READY`, with
`ERROR` on an unconfirmed operation. SLAM pause/resume is used when SLAM is
already managed by the selected runtime. Simulation mode transitions may
restart the owned simulation stack; physical operation must use its own
configured process manager.

## Localization initialization

The UI displays measured X/Y/yaw, frame, localization state, and active map.
Operators can enter coordinates or pick X/Y from the robot's map, adjust yaw,
preview the pose, then explicitly apply. The backend validates finite values,
robot identity, active map, map bounds, and bridge/service availability. The
bridge applies through the existing localization owner and confirms the
resulting map-frame TF pose where supported. No second `map -> odom` publisher
is introduced. Applying an initial pose changes localization; it does not
teleport the simulation entity.

## Command ownership, teleop, and Nav2

Web motion uses one path:

```text
Web key/button lease -> Django -> robot bridge -> /cmd_vel_manual --+
/cmd_vel ------------------------------------------------------------+-> command arbiter
/cmd_vel_nav -------------------------------------------------------+       |
/cmd_vel_tag -------------------------------------------------------+       v
E-STOP -------------------------------------------------------------> /cmd_vel_selected
                                                                            |
                                                                            v
                                                                  swerve controller
```

The arbiter is part of the production launch. MANUAL selects the fresh Web
manual lease, then the direct/manual source; AUTONOMOUS selects Nav2, except
tag approach owns while `/tag_navigation/state` is `APPROACH_TAG`. Stale or
missing selected sources output zero. E-STOP clears buffered sources and
overrides all of them with zero. Mode changes away from MANUAL, key/button
release, WebSocket/bridge disconnect, and stale manual heartbeat stop motion.
The diagnostics stream reports source, mode, command age, manual/Nav/tag
activity, and E-STOP state.

## Evidence boundary

Frontend/backend/Python tests and source inspection establish implementation
behavior only. They do not establish Gazebo rendering, physical LiDAR freshness,
map-server transitions, localization convergence, command-arbiter runtime
ownership, robot movement, goal completion, or broker connectivity. Those
results belong in `LOCAL_ROBOT_CONTROL_ACCEPTANCE.md` and are marked PASS only
after direct live observation.
