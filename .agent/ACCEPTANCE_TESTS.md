# Acceptance Tests

## Frontend
- Login works
- unauthenticated `/` redirects to `/login`
- successful login opens `/`
- authenticated `/` renders WareTwin Overview from active backend layout/runtime
- 3D View and Map View show active floor/rack/warehouse geometry
- no robot marker appears before backend telemetry and fresh matching ROS TF pose
- runtime status is reported or explicitly UNKNOWN / UNAVAILABLE / NOT REPORTED
- clicking a runtime robot opens its exact dynamic `/robots/:robotId/control` URL
- /control loads
- `/control` lists only backend-reported runtime robots
- /robots/:robotId/control loads
- removed product routes redirect to `/`
- manual safety, E-stop unknown fail-closed, stale path preview and map revision checks pass

## Runtime
- Backend ready
- Frontend ready
- ROS bridge connected
- Gazebo running
- Robot spawned
- controllers ACTIVE
- /joint_states live
- /odom live
- /odometry/filtered live
- /lidar/points live
- /scan live
- map -> odom -> base_footprint TF valid
- SLAM live
- Nav2 ACTIVE
- /navigate_to_pose available

## Manual
- MANUAL mode works
- forward/backward/left/right/rotate work
- STOP produces zero velocity
- watchdog produces zero velocity
- E-stop blocks motion

## Autonomous
- path preview VALID
- NavigateToPose accepted
- MAP POINT target is resolved against the current active map
- TAG target and tag-registry revision are revalidated before goal dispatch
- robot physically moves in Gazebo
- goal finishes successfully

## LiDAR
- /scan frame is correct
- LiDAR 2D rotates correctly with robot
- map-frame overlay uses scan timestamp TF
- `LIDAR_SCAN.source_frame_id=lidar_link`, `frame_id=map`, scan timestamp, sensor pose,
  sensor yaw, and points agree with the same timestamped TF
- a ~90 degree yaw change rotates current scan points in map coordinates
- INF/no-return ranges do not create points
- LiDAR 3D remains consistent

## Backend/API
- external runtime never calls legacy schedule dispatch or sends scheduler `NAVIGATE`
- stale schedule/order records cannot cause robot motion
- obsolete scheduler/orders/conveyors/admin/inject/tasks/AI/simulation routes return 404
- auth, health/runtime, WebSocket, map, map sync, local robot maps, localization,
  manual/E-stop/navigation, LiDAR, and diagnostics required by Robot Control remain

## Repository safety
- current branch is `robot-real-sim`
- only `origin/robot-real-sim` receives commits/pushes
- `web-simulation` remains untouched
- clean final worktree and `git diff --check`
