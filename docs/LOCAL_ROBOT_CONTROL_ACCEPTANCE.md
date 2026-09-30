# Local Robot Control Acceptance

Date: 2026-09-30 (Asia/Ho_Chi_Minh)
Branch: `web-simulation`

This report separates source/test evidence from live runtime acceptance. A
passing test or source inspection is not evidence of a rendered map, robot
motion, a completed Nav2 goal, or successful localization on a running robot.

## Runtime acceptance

The currently running project stack is stopped. On the earlier healthy
production `navigation --headless` run (canonical revision 21), the browser
opened `/robots/R01/control` with no page errors, rendered the 600x1200 `map`
grid and measured robot pose, received real `/scan` and filtered point-cloud
frames over the bridge/WebSocket, and displayed the selected 2D/3D views. A
real Nav2 `ComputePathToPose` preview returned VALID with 22 map-frame points
and enabled Send Goal. These observations establish the view/preview rows
below, not successful motion or navigation.

That run also observed 60 selected-velocity samples at idle with zero
non-zero samples. A direct/manual arbiter source test selected
`DIRECT_MANUAL`, produced 626 non-zero selected samples, and changed Gazebo
pose by 0.422 m (odometry 0.401 m). Its old watchdog assertion incorrectly
waited for a motor-controller zero in Gazebo time during low RTF; the harness
now checks the arbiter's wall-time selected output instead. That revised gate
has not been runtime-retested.

The Web-manual test failed before the latest immediate-publish fix: R01
acknowledged the mode/commands and reported `WEB_MANUAL` ownership, but the
manual source emitted only zero twists, selected output had zero non-zero
samples, and measured movement was below tolerance. A subsequent focused
pre-fix probe reproduced this. The bridge now publishes each accepted command
immediately while retaining the leased refresh/expiry timer; it was rebuilt,
but no post-fix motion test has passed the new startup gate.

Send Goal was attempted with a valid preview (same `CANONICAL` map revision
21), but Django rejected the request as `MAP_OUT_OF_SYNC` before
`NavigateToPose`; no navigation action or movement occurred. The synchronization
state was later observed as `SYNCED`, so this is an unresolved runtime-state
transition, not a successful goal. Preview enforcement has not had a live
negative test.

The earlier post-build restart failed its controller readiness gate: the
broadcaster activated, steering/drive were not confirmed, and
`/controller_manager/list_controllers` timed out. No controller YAML defect
was established. In the Phase 1 production run below, controller activation
was successful and every expected interface was claimed; the full navigation
stack still failed later at Nav2 lifecycle startup. No motion command was sent.

### Phase 1: controller startup reliability

ROOT_CAUSE=The launch chain advanced on any spawner process exit, even a
non-zero exit. Separately, readiness queried `list_controllers` every 0.2 s
while spawners were activating and replaced a prior valid controller snapshot
with an empty state after a transient service timeout. The previous run showed
the manager timeout during steering activation; this run found no controller
configuration defect.

FIX=Controller spawners now advance only on exit code 0, in dependency order;
failed activation aborts dependent startup. The command arbiter starts after
drive activation, then the selected-velocity consumer starts after the arbiter
process. Controller-manager verification waits for that settled chain, keeps
one request in flight, and spaces retries by at least 2 s. Readiness reports
manager/controller/node outcomes separately. Spawner manager-availability
timeout is 180 s; controller configuration was unchanged.

START_1=FAIL (controller readiness passed; Nav2 lifecycle readiness failed)
START_2=UNVERIFIED (not run because START_1 did not reach production READY)
GAZEBO_READY=PASS
ROBOT_SPAWNED=PASS
CONTROLLER_MANAGER_READY=PASS
JOINT_STATE_BROADCASTER_ACTIVE=PASS
STEERING_CONTROLLER_ACTIVE=PASS
DRIVE_CONTROLLER_ACTIVE=PASS
COMMAND_ARBITER_READY=PASS
SWERVE_CONTROLLER_READY=PASS
ODOM_READY=PASS
LIDAR_READY=PASS
TF_READY=PASS
NAV2_READY=FAIL
BRIDGE_READY=UNVERIFIED
CONTROLLER_INTERFACES=PASS: steer_front_joint/position, steer_rear_joint/position,
wheel_front_drive_joint/velocity, wheel_rear_drive_joint/velocity were all
reported claimed by the domain-0 readiness service. Duplicate ownership was
not separately enumerated after the matching-domain CLI check was omitted.
IDLE_SELECTED_CMD_ZERO=UNVERIFIED (follow-up CLI was initially bound to ROS
domain 12 rather than the managed run's domain 0; no matching-domain sample
was taken).

TESTS=PASS: `./scripts/build_ros.sh` built `swerve_bringup` and `swerve_bridge`;
10 targeted launch/readiness tests passed; changed Python files compiled;
`bash -n scripts/start_stack.sh` and `git diff --check` passed.

REMAINING_ISSUES=Nav2 lifecycle manager returned `success=false` after
`map_server` activation logged `transition invoked while in transition`,
leaving planner/controller/behavior/BT/waypoint lifecycle nodes inactive. The
readiness gate stopped before it could verify the R01 bridge heartbeat. The
bridge WebSocket connected in the same run, but that alone is not heartbeat
evidence. No second start was attempted and no motion test was run.

The Phase 1 test used the production command `source scripts/ros_env.sh &&
./scripts/start_stack.sh navigation`, after `stop_stack.sh` and process
verification. Kernel checks before and during the run found no new storage,
blocked-task, or OOM signatures; 5.3 GiB memory was available and root storage
had 6.0 GiB free. The launch reported canonical map revision 21. Stack-owned
processes were stopped after the failed Nav2 gate; unrelated port-8000 service
was not touched.

| Item | Result | Evidence / reason |
|---|---|---|
| CONTROL_TAB | PASS | Live R01 detail route loaded with controls and no browser/page errors. |
| GLOBAL_MAP | PASS | Live canonical revision 21 occupancy map, map-frame pose/heading and map status rendered in browser. |
| LIDAR_2D | PASS | Real bridge/WebSocket scan frame observed: 624 points, `base_footprint`; 2D mode selected in browser. |
| LIDAR_3D | PASS | Real filtered cloud frames observed: about 2.75k points with changing counts, `base_footprint`; 3D canvas stayed responsive with no page errors. |
| PATH_PREVIEW | PASS | Live Nav2 preview VALID, 22 map-frame points, length 0.58156 m; Send Goal enabled for matching map identity. |
| PATH_PREVIEW_ENFORCEMENT | UNVERIFIED | Source/backend tests enforce missing, stale, invalid, wrong-robot, target, and map-revision cases; no live negative authorization test. |
| SEND_GOAL | FAIL | The valid-preview request was rejected with `MAP_OUT_OF_SYNC` before Nav2; no action result or movement. |
| MAPPING_START | UNVERIFIED | No live SLAM state transition observed. |
| MAPPING_STOP | UNVERIFIED | No live SLAM stop/pause observed. |
| MAP_SAVE | UNVERIFIED | Tests create and validate map artifacts; no live runtime map save observed. |
| MAP_LOAD | UNVERIFIED | Tests cover bridge confirmation; no live map_server transition observed. |
| LOCAL_ACTIVE_MAP | UNVERIFIED | No live local map activation and subsequent local Nav2 use observed. |
| INIT_ROBOT_STATE | UNVERIFIED | Tests cover the existing localization interface contract; no live TF/localization update observed. |
| COMMAND_ARBITER_IDLE | PASS | 60 live selected-velocity samples, all zero, owner `NONE`. |
| COMMAND_ARBITER_MANUAL | UNVERIFIED | Direct source works; Web source failed pre-fix. The immediate-publish change has not passed a runtime gate yet. |
| COMMAND_ARBITER_NAV | UNVERIFIED | No live Nav2 ownership and pose-change evidence. |
| COMMAND_ARBITER_ESTOP | UNVERIFIED | No live immediate-zero observation. |
| TELEOP_FORWARD | UNVERIFIED | Pre-fix Web command had no non-zero selected velocity and movement below tolerance; post-fix gate was not reached. |
| TELEOP_BACKWARD | UNVERIFIED | No live robot pose change observed. |
| TELEOP_LEFT | UNVERIFIED | No live robot pose change observed. |
| TELEOP_RIGHT | UNVERIFIED | No live robot pose change observed. |
| TELEOP_ROTATE | UNVERIFIED | No live robot pose change observed. |
| TELEOP_STOP | UNVERIFIED | No live stop response observed. |
| VDA5050_CONFIG | UNVERIFIED | Backend tests cover configuration persistence/redaction; no live adapter status observed. |
| VDA5050_TEST_CONNECTION | UNVERIFIED | No live MQTT broker attempt observed. |
| VDA5050_ALLOW_TASK | UNVERIFIED | Tests cover disabled-order behavior; no live broker/order subscription observed. |
| VDA5050_INSTANT_ACTIONS | UNVERIFIED | Capability is honestly exposed as NOT_IMPLEMENTED; no live broker session observed. |
| WEB_MANUAL_R01 | UNVERIFIED | Web commands reached R01 and were acknowledged, but the pre-fix run produced no non-zero manual source. The final immediate-publish source could not be motion-tested. |
| WEB_NAV_GOAL_R01 | FAIL | Web Nav2 preview was valid; backend rejected Send Goal as `MAP_OUT_OF_SYNC` before action acceptance. |

## Source and automated validation

These results describe source-level behavior only:

- Frontend interaction tests: `npm run test -- tests/robot_control_workflow.test.tsx` — 11 passed.
- Frontend typecheck/build: `npm run build` — `tsc --noEmit` and Vite build passed; Vite emitted the existing large-chunk warning. There is no `typecheck` npm script.
- Backend: `manage.py check`, `makemigrations --check`, and the targeted ROS bridge/local-control/map-sync test set — 33 passed.
- Python renderer/command-arbiter/readiness tests — 23 passed; changed ROS/Python source passed `py_compile`.
- ROS package build: passed for `swerve_bringup` and `swerve_bridge`.
- `git diff --check`: passed.

## Remaining runtime gates

- Diagnose the Nav2 `transition invoked while in transition` startup failure,
  then pass the complete production readiness gate and perform one clean
  reproducibility restart.
- Confirm `/swerve_bridge` plus the backend-observed R01 heartbeat and capture a
  matching-domain idle `/cmd_vel_selected` zero sample.
- Retest Web manual forward/STOP after immediate bridge publication; establish
  MANUAL, NAV, and E-STOP ownership before testing further directions/goals.
- Record start pose, selected goal, planner path, action result, final pose,
  and final error for successful navigation.
- Verify real `/scan` and `/lidar/points_filtered` frames, map transitions,
  localization TF, and MQTT broker response in the live system.
- A deployment-specific physical runtime adapter is not included/configured;
  real-robot mode transitions therefore remain unavailable in this checkout.
