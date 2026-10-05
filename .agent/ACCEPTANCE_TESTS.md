# Acceptance Tests

## Frontend
- Login works
- /control loads
- Robot can be selected
- /robots/:robotId/control loads
- Removed routes are inaccessible

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
- robot physically moves in Gazebo
- goal finishes successfully

## LiDAR
- /scan frame is correct
- LiDAR 2D rotates correctly with robot
- map-frame overlay uses scan timestamp TF
- LiDAR 3D remains consistent
