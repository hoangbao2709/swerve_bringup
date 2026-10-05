# robot-real-sim

## Goal

Create a simulation environment that is as close as possible to the real robot software architecture.

Gazebo replaces only the physical hardware layer.

Required architecture:

Browser
-> Django / WebSocket
-> ROS Bridge
-> ROS 2
-> Command Arbiter / Nav2 / SLAM
-> Robot Interface
-> Gazebo

The future real robot path must remain:

Browser
-> Django / WebSocket
-> ROS Bridge
-> ROS 2
-> Command Arbiter / Nav2 / Localization
-> Hardware Driver
-> Real Robot

## User-facing application

The authenticated frontend routes are:

- /login
- / -> WareTwin Overview
- /control -> Robot Control overview/list
- /robots/:robotId/control

Unauthenticated `/` redirects to `/login`. Successful login opens `/`.
Unknown authenticated routes redirect to `/`. The sidebar contains only
Workspace (Overview, Robot Control) and Account (authenticated username,
logout). Do not expose removed warehouse operations/product pages.

Overview keeps the dark WareTwin operations-console layout (sidebar, status
header, 3D View and Map View), but renders warehouse geometry only from the
active backend canonical map. Robot identity and pose only come from runtime
telemetry; position/heading require fresh ROS TF pose data matching the active
map identity and revision. Backend diagnostics and bridge state are displayed
as reported, or UNKNOWN / UNAVAILABLE / NOT REPORTED when absent. Selecting a
robot opens the same backend runtime robot ID at
`/robots/:robotId/control`.

## Mandatory behavior

The remaining functions must not be degraded.

Preserve:

- authentication
- robot overview
- robot detail
- manual control
- autonomous control
- Emergency Stop
- command watchdog
- ROS bridge
- real ROS telemetry
- LiDAR 2D
- LiDAR 3D
- odometry
- TF
- SLAM Toolbox
- Nav2
- map identity
- map revision synchronization
- path preview
- NavigateToPose
- MAP POINT and TAG goals pass a validated current-map path preview before a
  Nav2 goal is sent
- external runtime never dispatches legacy database schedules
- only Robot Control/auth/runtime/map APIs remain publicly routed

## Forbidden

Do not:

- fake robot motion in frontend
- fake LiDAR
- fake telemetry
- bypass ROS bridge
- bypass command arbiter
- bypass Nav2
- remove safety checks
- remove watchdog
- weaken tests to make them pass
- hard-code success
- silently use LOCAL_SIM

The runtime motion paths are MANUAL (Django/WebSocket -> ROS bridge -> command
arbiter -> selected velocity -> controller -> Gazebo) and AUTONOMOUS (validated
path preview -> NAV_GOAL -> ROS bridge -> Nav2 -> arbiter/controller -> Gazebo).
The browser is never a simulator. Legacy database schedules/orders must not
issue external NAVIGATE commands.

LiDAR map overlays must use the `/scan` source frame and exact LaserScan stamp
for `map <- lidar_link` TF. `LIDAR_SCAN.frame_id` and points are map-framed;
`sensor_pose` is the lidar pose in map at that same stamp. Non-finite/no-return
ranges remain absent, not synthetic obstacles.
