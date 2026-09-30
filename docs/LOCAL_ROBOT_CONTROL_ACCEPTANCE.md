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

### Phase 1B: deferred Nav2 lifecycle startup

NAV2_LIFECYCLE_ROOT_CAUSE=The prior runtime log proves that
`lifecycle_manager_navigation` failed first while activating `map_server`,
with `transition invoked while in transition`; the prior trace does not identify
which actor/source caused that overlap. Source audit found one production Nav2 lifecycle
manager, the mapping manager is conditional on mapping mode, and the readiness
probe is the only source-tree caller of `ManageLifecycleNodes.STARTUP`. The
previous trace did not record the manager parameters or pre-transition node
states, so a duplicate/autostart/local-map caller is not established as the
cause.

NAV2_LIFECYCLE_FIX=Readiness now checks the deferred manager contract at
runtime (`autostart=false`, expected node list), verifies exactly one
`map_server` and no mapping lifecycle manager, matches the selected map YAML,
and confirms a stable all-UNCONFIGURED snapshot again immediately before
startup. It atomically claims one STARTUP per launch generation across probe
retries, records `NOT_REQUESTED/REQUESTED/IN_PROGRESS/ACTIVE/FAILED`, waits
for transitions to settle, and captures lifecycle states and manager logs on
failure. The supervisor resets the state only after stopping the previous
launch child.

START_1=PASS (production `source scripts/ros_env.sh && ./scripts/start_stack.sh navigation`; full READY, headless; no motion test)
START_2=FAIL (fresh production launch spawned the robot, but did not pass the controller activation gate; no Nav2 startup request was made)
GAZEBO_READY=PASS (both starts)
ROBOT_SPAWNED=PASS (both starts)
CONTROLLER_MANAGER_READY=PASS (START_1; START_2 did not reach readiness confirmation after a controller-manager response timeout)
JOINT_STATE_BROADCASTER_ACTIVE=PASS (START_1 readiness and START_2 launch log)
STEERING_CONTROLLER_ACTIVE=PASS (START_1 readiness and START_2 launch log)
DRIVE_CONTROLLER_ACTIVE=PASS (START_1); FAIL (START_2 spawner blocked before activation)
COMMAND_ARBITER_READY=PASS (START_1); UNVERIFIED (START_2 did not reach dependent startup)
SWERVE_CONTROLLER_READY=PASS (START_1); UNVERIFIED (START_2 did not reach dependent startup)
ODOM_READY=PASS (START_1); UNVERIFIED (START_2 readiness stopped at controller gate)
LIDAR_READY=PASS (START_1); UNVERIFIED (START_2 readiness stopped at controller gate)
TF_READY=PASS (START_1); UNVERIFIED (START_2 readiness stopped at controller gate)
NAV2_READY=PASS (START_1 one STARTUP response success=true; all six lifecycle nodes ACTIVE; `/navigate_to_pose` present); UNVERIFIED (START_2 never reached Nav2)
BRIDGE_READY=PASS (START_1 `/swerve_bridge` and fresh backend robot_id=R01 heartbeat); UNVERIFIED (START_2 never reached bridge readiness)
TEST_ROS_DOMAIN_ID=0 (sourced active `.runtime/stack.env`; backend health reports R01 online, ROS bridge and ROS healthy)
CONTROLLER_INTERFACES=PASS (claimed: steer_front_joint/position, steer_rear_joint/position, wheel_front_drive_joint/velocity, wheel_rear_drive_joint/velocity; controller listing shows no overlap)
IDLE_SELECTED_CMD_ZERO=PASS (30 selected Twist samples, 0 non-zero; owner and active source NONE; manual/nav/tag flags false; command-arbiter E-STOP diagnostic known false; no motion command sent)

Phase 1B runtime evidence: before the single STARTUP request, all six Nav2
lifecycle nodes reported `unconfigured(1)`. The live lifecycle graph reported
`map_server=1`, `lifecycle_manager_navigation=1`,
`lifecycle_manager_mapping_map=0`; the manager reported `autostart=false` and
the expected six-node list. The selected YAML was canonical revision 21 and
matched `/map_server`'s `yaml_filename`. The one request returned
`success=true`; all six states were ACTIVE immediately afterward. The bridge
heartbeat was tied to R01 in the backend's fresh `online_robot_ids` list, not
inferred from WebSocket connectivity. The ROS CLI follow-up printed
`TEST_ROS_DOMAIN_ID=0` and confirmed controllers, claimed interfaces, nodes,
and actions. The bridge's `/emergency_stop` publisher had no initial Bool
sample, but the command arbiter's own diagnostic reported `estop_active=false`;
that diagnostic was used as the known runtime E-STOP state.

Phase 1B tests before the reproducibility restart: 23 targeted Python readiness,
command ownership, and map-sync tests passed; 6 Django health tests passed;
`manage.py makemigrations --check` passed; changed Python sources compiled;
`bash -n scripts/start_stack.sh` and `git diff --check` passed; `colcon build
--symlink-install` passed for the discovered `swerve_bringup` package. The
pre-START_1 VM gate found 5.3 GiB available RAM, 17 MiB swap use, 6.0 GiB free
on `/`, and no matching current-boot storage, blocked-task, or OOM errors.

START_2 runtime evidence: after a clean stop and process check, the second
production `start_stack.sh navigation` run again started Gazebo and spawned
`swerve_base`. `joint_state_broadcaster` and `steering_controller` logged
successful activation, but `drive_controller`'s spawner remained blocked in
`futex_wait_queue` for more than three minutes. The controller manager logged
`failed to send response to /controller_manager/list_controllers (timeout)`;
readiness ended with
`required ros2_control controllers not ready: controller_activation_sequence_not_complete`.
No Nav2 lifecycle request, bridge heartbeat check, or idle sample was reached
on this run. The stack was stopped cleanly. The current test window had no new
SCSI/I/O, blocked-task, or OOM errors; available RAM was 3.4 GiB while running
and 5.2 GiB after stop, swap remained 17 MiB used, and `/` retained 6.0 GiB.
Thus START_2 is a runtime FAIL, not an environment-blocked storage result.
No third startup cycle was run.

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

## Phase 1C / Phase 2 — current gated acceptance

This section supersedes the previous remaining-startup checklist for this
attempt. Historical Phase 1B results above remain historical evidence; they
are not counted as a fresh PASS for the changed controller startup path.

RESTART_ROOT_CAUSE=The retained Phase 1B START_2 log shows JSB and steering
activation and clean spawner exits. The drive spawner starts next, but no
`Loading controller 'drive_controller'` appears. Instead, controller_manager
logs `failed to send response to /controller_manager/list_controllers
(timeout): client will not receive response`. Installed controller-manager
2.40.0's `service_caller` creates a new client and calls
`rclpy.spin_until_future_complete(node, future)` without a response deadline.
Its `service_timeout` covers endpoint availability only. The failed read-only
response therefore strands the drive spawner before loading, blocks its exit,
and prevents every dependent layer from starting. This proves the immediate
stall mechanism, not a drive-controller configuration or resource claim error.
Fast DDS's [Humble response implementation](https://github.com/ros2/rmw_fastrtps/blob/humble/rmw_fastrtps_shared_cpp/src/rmw_response.cpp)
returns this timeout when the service's response writer cannot confirm the
requester's response reader during discovery. Why that specific endpoint match
missed its deadline is not established by the retained logs.

RESTART_FIX=Project controller spawner now creates and retains all four service
clients before its first request, bounds each response wait, removes timed-out
pending futures, and retries only read-only `list_controllers` requests (at most
three, serially, on the same client). Load/configure/switch mutations are sent
once; a missing acknowledgement is reconciled against the resulting controller
state rather than replayed. Each spawner checks its predecessor is ACTIVE and
confirms its own ACTIVE state and exact non-overlapping command claims before
exiting zero. The existing success-only launch event chain is preserved.
Request/response timing, initial/resulting state, lost replies, and confirmed
claims are logged. Controller parameters, sensor fidelity, arbitration, and
deferred Nav2 ownership are unchanged. This fix is source-tested, but has not
yet passed a production run.

Audit: before this attempt, `stop_stack.sh` found no owned stack processes and
process inspection found no surviving ROS/Gazebo stack. Controller startup is
serial; readiness sends no controller-manager queries until arbiter and swerve
nodes appear. Each Gazebo process constructs a fresh plugin/resource manager;
the previous log contains no already-loaded or duplicate-interface error.
Service callers in the spawners use rclpy directly, not the ROS CLI daemon.
No stack/DDS process cleanup fault was demonstrated, so no cleanup or daemon
reset was introduced as a speculative fix.

START_1=FAIL
STOP_CLEAN=PASS (after failed START_1; all managed processes stopped, no stack ROS/Gazebo survivors)
START_2=UNVERIFIED (not run after the Stage A failure)
GAZEBO_READY=PASS
ROBOT_SPAWNED=PASS
CONTROLLER_MANAGER_READY=UNVERIFIED (plugin manager created, service gate not reached)
JOINT_STATE_BROADCASTER_ACTIVE=FAIL (new spawner exited before loading)
STEERING_CONTROLLER_ACTIVE=UNVERIFIED
DRIVE_CONTROLLER_ACTIVE=UNVERIFIED
COMMAND_ARBITER_READY=UNVERIFIED
SWERVE_CONTROLLER_READY=UNVERIFIED
ODOM_READY=UNVERIFIED
LIDAR_READY=UNVERIFIED (raw/filtered/scan samples appeared, full gate did not finish)
TF_READY=UNVERIFIED
NAV2_READY=UNVERIFIED (no STARTUP request sent)
BRIDGE_READY=UNVERIFIED
TEST_ROS_DOMAIN_ID=0 (explicit before start; readiness sourced active stack.env and printed domain 0)
CONTROLLER_INTERFACES=UNVERIFIED
IDLE_SELECTED_CMD_ZERO=UNVERIFIED
STAGE_A_GATE=FAIL

Exact fresh runtime failure: the new JSB spawner attempted to assign its client
dictionary to `rclpy.Node.clients`, a read-only property, raising
`AttributeError: can't set attribute 'clients'`. The spawner exited 1 and the
launch success gate correctly aborted dependent startup. The implementation
was corrected to `service_clients`, and a real Humble-node constructor test
now confirms all four persistent clients can be created. This correction was
compiled and rebuilt but was not production-retested: the requested Stage A
stop-on-failure gate was honored. This is a new implementation defect exposed
by the test, separate from the previous drive-spawner DDS response loss.

VM health before/after the attempt: current-boot journal had no matching
SCSI/DID_TIME_OUT/I/O/ext4/blocked-task/OOM errors. RAM available was about
5.2–5.3 GiB, swap use 17 MiB, and root filesystem free space 6.0 GiB.
Storage did not block this attempt. The production path was
`stop_stack.sh`, `source scripts/ros_env.sh`, `start_stack.sh navigation`,
headless with canonical revision 21 and robot R01. Gazebo clock advanced;
`swerve_base` appeared among 56 models; spawn completed in 43.822 seconds.
The final readiness result was `controller_activation_sequence_not_complete`.
The failed stack was stopped and no robot motion command was sent.

COMMAND_ARBITER_MANUAL=UNVERIFIED
WEB_MANUAL_FORWARD=UNVERIFIED
WEB_MANUAL_BACKWARD=UNVERIFIED
WEB_MANUAL_LEFT=UNVERIFIED
WEB_MANUAL_RIGHT=UNVERIFIED
WEB_MANUAL_ROTATE_LEFT=UNVERIFIED
WEB_MANUAL_ROTATE_RIGHT=UNVERIFIED
WEB_MANUAL_STOP=UNVERIFIED
WEB_MANUAL_TIMEOUT_STOP=UNVERIFIED
WEB_DISCONNECT_STOP=UNVERIFIED
MODE_CHANGE_STOP=UNVERIFIED
COMMAND_ARBITER_ESTOP=UNVERIFIED
ESTOP_CLEAR_NO_RESUME=UNVERIFIED
WEB_MANUAL_R01=UNVERIFIED
FORWARD_GAZEBO_DISPLACEMENT=UNVERIFIED
FORWARD_ODOM_DISPLACEMENT=UNVERIFIED
MANUAL_COMMAND_LATENCY_MS=UNVERIFIED
STAGE_B_GATE=NOT_RUN (Stage A failed)

TESTS=PASS: 19 targeted controller-spawner, launch-chain and readiness pytest
tests; changed Python compile; bash syntax for startup/stop/common scripts;
`colcon build --symlink-install` (swerve_bringup); `git diff --check`.
Regression tests cover lost read responses, reuse of the same client, removal
of pending futures, bounded exhaustion, mutation acknowledgement loss without
replay, predecessor ACTIVE requirements, exact/unique claims, and actual Humble
node construction. Backend/frontend command source did not change, so their
unrelated suites were not rerun.

REMAINING_ISSUES=The corrected spawner still needs a fresh production START_1,
STOP_CLEAN and START_2 with all readiness, interface and idle samples PASS.
Stage B remains unexecuted until that complete gate passes. No phase-2 Web
manual acceptance or navigation goal was attempted.
