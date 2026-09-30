# Local Robot Control Architecture

The Robot Control Detail route (`/robots/R01/control`) is the local HMI for one
selected robot. Section changes stay in the same detail route and every REST
and WebSocket command carries that robot's `robot_id`. The browser consumes
processed bridge frames and never opens ROS topics or publishes `/cmd_vel`.

```text
React Robot Control Detail (R01)
        | REST + authenticated WebSocket
        v
WareTwin Django API / Channels runtime
        | robot_id scoped command or bridge RPC
        v
Authenticated ROS Bridge (R01)
        |
        +------------------------------+
        | ROS 2                        |
        |                              |
        | SLAM Toolbox   Nav2   TF     |
        | map / scan     EKF    LiDAR  |
        | command_arbiter              |
        +------------------------------+
        |
        v
ros2_control / Gazebo / Robot
```

## Control detail sections

- **CONTROL** keeps the existing robot selector, MANUAL/AUTONOMOUS gate,
  emergency stop, measured pose and velocity, global warehouse map, LiDAR
  status, teleop, navigation status, and pause/resume/cancel controls.
- **MAPPING** invokes robot-scoped bridge RPCs. SLAM Toolbox and Nav2 are
  mutually exclusive in `system.launch.py`. The section can request a
  supervised mode change after R01 is in MANUAL and idle. The ROS stack
  supervisor restarts only its owned ROS/Gazebo child, runs the existing
  readiness gate, and restores the previous mode if the requested mode fails
  readiness. The simulated robot respawns at its configured start pose.
  Start/stop within mapping toggles SLAM Toolbox's pause-measurement service;
  it does not kill a node. Save calls SLAM Toolbox's map saver and registers a
  map only after its YAML and image files exist.
- **LOCALIZATION** shows the measured map-frame pose and applies a selected
  initial pose through the configured `/ekf_v30e/set_pose` service. This keeps
  `ekf_v30e` as the existing `map -> odom` authority.
- **VDA5050** stores one MQTT configuration per robot. Fernet ciphertext is
  persisted for the password; API responses expose only
  `password_configured`. Test Connection uses a temporary MQTT client. Apply
  replaces the selected robot's live MQTT client and rolls the DB values back
  if an enabled broker connection cannot be established. Opening the section
  restores persisted enabled broker settings after a backend restart. The
  task gate changes before the old client is retired, so queued orders use the
  new `allow_task` policy. `allow_task=false`
  removes the order subscription and rejects a queued order before it can reach
  the authenticated ROS bridge. Local manual operation and telemetry are not
  gated by that flag.
- **DIAGNOSTICS** shows the bridge's runtime, ROS, Gazebo, controller, Nav2,
  SLAM, localization, TF, LiDAR, MQTT, map synchronization, and reported error
  state.

## Map and LiDAR web data

The Global Map uses the canonical warehouse layout and the active robot-scoped
occupancy snapshot (`/slam/map` in mapping mode, `/map` in navigation mode) in
the shared map coordinate convention. The LiDAR
2D view uses `/scan`; the bridge transforms it with TF to `base_footprint`,
filters invalid/out-of-range returns, and sends a bounded numeric payload. The
3D view prefers `/lidar/points_filtered`, falls back to `/lidar/points` when the
filtered stream is stale, transforms PointCloud2 in Python, removes invalid and
out-of-range points, applies height and voxel filtering, and caps web frames at
4,000 points and 3 frames per second. The bridge retains the newest ROS cloud
instead of queueing cloud frames. React only renders the bounded 2D/3D payload.

Source stamp, render timestamp, frame ID, point count, epoch/revision, and
render FPS travel with 3D frames. The 2D view reports frame age and point count.
ROS sensor configuration and production LiDAR fidelity are unchanged.

## Path preview and goal execution

```text
Map click or LiDAR 2D click
        | screen -> canonical map metres (LiDAR points use measured robot pose)
        v
PATH_PREVIEW_REQUEST (R01, map frame)
        v
Django map/identity gates -> authenticated ROS bridge
        v
Nav2 ComputePathToPose -> map-frame nav_msgs/Path
        | bounded path + base_footprint projection
        v
Global Map and LiDAR views render the same plan
        |
        | operator presses SEND GOAL
        v
NAV_GOAL with the approved preview request ID
        v
Django validates preview identity, target, age, and map revision
        v
NavigateToPose -> controller output -> existing swerve/Gazebo motion path
```

Preview alone does not send a navigation goal. The UI requires a valid response
for its current request ID. It disables preview and Send Goal when the robot's
local map override or ROS map synchronization status does not match the
canonical warehouse map. A locally loaded Nav2 map becomes active in
`map_server`; it is shown as out of sync and autonomous goals remain blocked
until the canonical map bundle is published again.

The LiDAR representations use `base_footprint`; their route and destination
come from the same map-frame Nav2 result transformed by the ROS bridge. The 3D
view projects the route onto the ground plane. Selecting a target is available
in Global Map and LiDAR 2D; the 3D view is for point-cloud inspection.

## Map storage and localization

Saved maps are stored under the configured artifact root in a robot-specific
`local_robot_maps/<robot_id>` directory. The registry uses opaque IDs and
excludes absolute paths from browser responses. Names and resolved paths are
validated before writes and loads. Loading a registered map requires MANUAL
mode and a stopped robot, then calls Nav2's `map_server/load_map`; a bridge-side
transition gate rejects motion until the service finishes. It is a runtime
map change, not a React image swap.

Initial pose is submitted in the ROS `map` frame to the existing
`robot_localization/SetPose` interface for `/ekf_v30e`. The UI asks for
confirmation and waits for the backend RPC response. Applying pose changes
localization state; it does not move the Gazebo entity.

## VDA5050 and task policy

The configured MQTT topic root is robot-specific and subscribes to `order`
only when `allow_task` is enabled. The bridge accepts released, map-frame node
positions while in AUTONOMOUS mode and executes them with the existing
NavigateToPose action, rejecting an order if E-STOP, map sync, another goal, or
invalid nodes prevent safe execution. Instant-action messages are gated by their own
subscription setting; the current bridge reports them as unsupported rather
than silently treating them as executed. Broker and physical fleet
interoperability require a real broker/adapter runtime test.

## Command ownership and safety

The existing control protocol remains the only web motion path. MANUAL mode is
accepted by the bridge before `MANUAL_CMD` can update its short dead-man lease;
key/button release sends STOP and a missed lease publishes a final zero. At the
swerve controller input, the direct/manual ROS topic (`/cmd_vel`), leased Web
manual topic (`/cmd_vel_manual`), Nav2 topic (`/cmd_vel_nav`), and tag approach
topic (`/cmd_vel_tag`) remain separate. The `command_arbiter` ROS component
selects one fresh source according to the latched robot mode and tag route
state, publishes `/cmd_vel_selected` and `/command_owner`, then the swerve
controller computes actuator commands from the selected stream. MANUAL selects
Web input while its lease is fresh,
then direct ROS input; AUTONOMOUS selects Nav2, with the tag-relative approach
input owning only while `/tag_navigation/state` is `APPROACH_TAG`. Mode changes
and E-STOP clear buffered commands. E-STOP also sets the existing ROS stop
input, cancels an active goal, and is cleared only by the explicit clear-stop
action.

The Web UI and Python/ROS tests validate the command protocol and data
processing paths. Global/LiDAR data rendering, mapping services, localization,
physical motion, Nav2 execution, MQTT broker communication, and emergency-stop
behavior still require the relevant live ROS/Gazebo or broker test before they
can be described as runtime-validated.
