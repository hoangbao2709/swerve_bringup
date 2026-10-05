# Implementation Plan

- [ ] M1 - Baseline audit and protect `web-simulation`; work only on `robot-real-sim`
- [ ] M2 - Restore `/` WareTwin Overview using backend map/runtime data only
- [ ] M3 - Keep final routes `/login`, `/`, `/control`, `/robots/:robotId/control`
- [ ] M4 - Remove dead frontend scripts/modules without restoring browser simulation
- [ ] M5 - Reduce REST routing to auth, health/runtime, canonical map and Robot Control APIs
- [ ] M6 - Prove legacy external scheduler/order records cannot dispatch motion
- [ ] M7 - Verify MAP POINT and TAG preview -> Nav2 goal with map identity/revision checks
- [ ] M8 - Verify timestamped `map <- lidar_link` scan transform and 2D rotation
- [ ] M9 - Verify manual command, STOP, watchdog, and fail-closed E-stop
- [ ] M10 - Build frontend, run frontend/backend tests, and build ROS workspace
- [ ] M11 - Full Gazebo, SLAM, Nav2, manual/autonomous, Overview, and physical LiDAR acceptance
- [ ] M12 - Final code audit, branch/status/diff checks, and push only to origin/robot-real-sim

Do not mark a milestone complete without a recorded command result or runtime
evidence. A passing unit/API test does not substitute for physical Gazebo
acceptance.
