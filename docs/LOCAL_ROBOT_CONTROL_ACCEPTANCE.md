# Local Robot Control Acceptance

Updated: 2026-10-01 (Asia/Ho_Chi_Minh)
Branch: `web-simulation`

This report separates source/test evidence from live runtime acceptance. A
passing test or source inspection is not evidence of a rendered map, robot
motion, a completed Nav2 goal, or successful localization on a running robot.

## Evidence ordering

Sections below retain dated historical failures for traceability; they are not
the current status. Phase 1 startup is CLOSED/PASS (two production starts,
patched bondcpp loaded by actual processes, clean stop, bridge and idle zero).
The latest resume evidence is recorded at the end of this report. Overnight
rows using a relaxed wheel limit do not satisfy the current Stage B gate.

## Historical runtime acceptance (2026-09-30; superseded)

At the end of that historical session the project stack was stopped. On the earlier healthy
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

## Earlier runtime checklist (superseded by latest results below)

- Deferred single-owner Nav2 startup was verified by Phase 1B START_1.
  Current restart and dependency results are recorded in the latest run below.
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

## Corrected-spawner production retest — latest attempt

Latest attempt began from clean `web-simulation` commit `d310527`. Source uses
`service_clients`, contains no assignment to `Node.clients`, and byte comparison
confirmed the installed spawner matches the corrected source. No rebuild was
needed before this production retest. Both pre-start and post-failure managed
stops/process inspections found no stack-owned ROS/Gazebo survivors.

FIRST_FAILING_LAYER=Production startup log guard, before new ROS/Gazebo launch.
The 20:31 production `start_stack.sh navigation` invocation selected R01,
canonical revision 21, headless mode, backend port 8001, and ROS domain 0.
It immediately reported `ROS/Gazebo launch failed`. `logs/ros.log` was still
dated 20:14:34 and contained the previous run's JSB constructor failure and
its `[ERROR] ... failed with exit code 1` marker. No current supervisor/launch
marker was written. Backend/frontend logs had current 20:31 timestamps.
The parent checks ROS logs immediately after starting an asynchronous shell,
but the shell sources `ros_env.sh` before opening its `tee` outputs. Thus the
parent interpreted the old error before tee could reset the log and stopped
the new process group. The corrected controller spawner never ran in this
attempt; this was not a newly observed controller or Nav2 failure.

FIX=Initialize current ROS and bridge logs synchronously in the parent before
spawning the asynchronous managed ROS shell. Preserve each nonempty old log
under a timestamped `.previous` name, then create an empty current log. Startup
probes can no longer read old errors or old Gazebo PIDs while the child loads
its environment. Controller startup and readiness safety gates are unchanged.

START_1=FAIL (stale previous-run log interpreted as current startup failure)
STOP_CLEAN=PASS (managed failure cleanup plus explicit stop and process inspection)
START_2=UNVERIFIED (not run after Stage A failure)
GAZEBO_READY=UNVERIFIED
ROBOT_SPAWNED=UNVERIFIED
CONTROLLER_MANAGER_READY=UNVERIFIED
JSB_ACTIVE=UNVERIFIED
STEERING_ACTIVE=UNVERIFIED
DRIVE_ACTIVE=UNVERIFIED
DRIVE_CONTROLLER_ACTIVE=UNVERIFIED
COMMAND_ARBITER_READY=UNVERIFIED
SWERVE_CONTROLLER_READY=UNVERIFIED
ODOM_READY=UNVERIFIED
LIDAR_READY=UNVERIFIED
TF_READY=UNVERIFIED
NAV2_READY=UNVERIFIED
BRIDGE_READY=UNVERIFIED
TEST_ROS_DOMAIN_ID=0 (explicit before start, confirmed in production launch output)
CONTROLLER_INTERFACES=UNVERIFIED
IDLE_SELECTED_CMD_ZERO=UNVERIFIED
STAGE_A_GATE=FAIL

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
STAGE_B_GATE=NOT_RUN (Stage A failed; no motion command sent)

STORAGE_HEALTH=PASS: current-boot journal before/after startup had no matching
SCSI/DID_TIME_OUT/I/O/ext4/blocked-task/OOM errors. Pre-start RAM available
5.3 GiB, swap use 17 MiB, root filesystem free space 6.0 GiB.

TESTS=PASS: 21 targeted log-initialization, controller-spawner, launch-chain and
readiness pytest tests; changed Python test compile; `bash -n` for changed
startup/common shell scripts; `git diff --check`. New regression coverage
proves previous error/PID text is archived and absent before the next writer
starts, and ensures log initialization precedes asynchronous ROS startup and
the error probe. No ROS/backend/frontend source changed in this correction,
so no unrelated builds or suites were rerun.

REMAINING_ISSUES=The corrected spawner and startup-log fix still need two clean
production READY runs with controller claims, fresh R01 heartbeat and at least
30 idle zero samples. The requested diagnose/fix/STOP rule was honored after
the first failing layer; no additional production run or Stage B motion test
was performed in this attempt.

## Production two-start retest — historical outcome before library-selection fix

Started from clean commit `63cdc08`; corrected source and installed spawner
matched byte-for-byte. The synchronous log fix was present. Both runs used
the production `start_stack.sh navigation` path, headless, robot R01,
canonical revision 21, and managed ROS domain 0. No motion command was sent.

START_1=PASS (complete production READY plus controller/idle/heartbeat checks)
STOP_CLEAN=PASS (between starts and after START_2 failure; no stack ROS/Gazebo survivors)
START_2=FAIL (Nav2 manager bond formation after map_server activation)
GAZEBO_READY=PASS (both)
ROBOT_SPAWNED=PASS (both)
CONTROLLER_MANAGER_READY=PASS (both)
JSB_ACTIVE=PASS (both)
STEERING_ACTIVE=PASS (both)
DRIVE_ACTIVE=PASS (both)
COMMAND_ARBITER_READY=PASS (both)
SWERVE_CONTROLLER_READY=PASS (both)
ODOM_READY=PASS (both)
LIDAR_READY=PASS (both raw cloud, filtered cloud and scan)
TF_READY=PASS (both map->odom->base_link and odom->base_footprint)
NAV2_READY=PASS (START_1); FAIL (START_2)
BRIDGE_READY=PASS (START_1); UNVERIFIED (START_2 stopped before bridge gate)
CONTROLLER_INTERFACES=PASS (both)
IDLE_SELECTED_CMD_ZERO=PASS (START_1); UNVERIFIED (START_2 stopped before idle probe)
TEST_ROS_DOMAIN_ID=0 (both production runs; START_1 follow-up sourced managed stack.env)
STAGE_A_GATE=FAIL

START_1 evidence: full READY after 98.209 seconds, Gazebo PID 88401,
spawn duration 30.712 seconds. All three spawners confirmed exact ACTIVE
states/interface claims and exited cleanly in sequence. One deferred Nav2
STARTUP (`f0e22eb4-b6a4-4900-b752-386559d11243`) returned success=true;
all six lifecycle nodes became ACTIVE and `/navigate_to_pose` was available.
`ros2 control list_controllers --claimed-interfaces` and
`list_hardware_interfaces` confirmed the two steering position and two wheel
velocity command interfaces, claimed without overlap. Django health reported
fresh `online_robot_ids=["R01"]`, `ros_bridge=true`, `ros=true`.
The read-only idle subscriber received 101 selected Twist samples over
2.001 seconds, with zero nonzero values. The only selected-command publisher
was `command_arbiter`; diagnostics reported owner NONE, mode AUTONOMOUS,
manual/nav/tag inactive, E-STOP false, and no last-command age.

After START_1, `stop_stack.sh` confirmed the ROS/frontend/backend groups
stopped, and process inspection found no survivors. Pre-START_2 storage checks
were clean. START_2 spawned in 21.206 seconds with Gazebo PID 90338. The drive
spawner encountered one lost first `list_controllers` reply, logged
`CONTROLLER_RESPONSE_LOST`, retried the read-only request on its persistent
client, then loaded/configured/activated drive and exited cleanly. This directly
verifies the spawner recovery that the previous second start lacked.

FIRST_FAILING_LAYER=Nav2 lifecycle manager's bond formation, not controller
activation or duplicate global lifecycle startup. START_2 recorded one STARTUP
(`b885475f-0774-4de3-93a8-9c04c8ea42db`), with all six nodes UNCONFIGURED
beforehand, autostart=false, one map_server, one navigation manager, and no
mapping manager. The manager configured every node, activated map_server,
then returned success=false with `transition invoked while in transition`.
Post-failure states were map_server ACTIVE(3), all other five nodes INACTIVE(2).
No second STARTUP was sent; the one-shot guard stopped the failed stack.

NAV2_BOND_ROOT_CAUSE=Installed bondcpp 3.0.2's `waitUntilFormed` reads
`sm_.getState()` without taking the FSM mutex while heartbeat callbacks perform
state transitions under that mutex. The FSM temporarily clears its state during
the transition, and the unlocked read can throw smclib's
`StateUndefinedException` with exactly the observed message. Nav2's
`changeStateForNode` calls `createBondConnection` after a successful lifecycle
activation, which explains map_server ACTIVE despite the manager exception.
This is separate from the previously verified deferred startup ownership fix.
Primary sources:
[bondcpp 3.0.2 wait implementation](https://github.com/ros/bond_core/blob/3.0.2/bondcpp/src/bond.cpp)
and [Nav2 lifecycle manager](https://github.com/ros-navigation/navigation2/blob/humble/nav2_lifecycle_manager/src/lifecycle_manager.cpp).
Managed-environment loader inspection confirmed /opt/ros/humble Nav2 and
bondcpp were used, not the unrelated cartoros2 overlay.

NAV2_BOND_FIX=Backport synchronized state reads in waitUntilFormed and
waitUntilBroken against the installed bondcpp 3.0.2 headers. Condition-variable
waiting releases the same mutex so heartbeat callbacks can finish. Exact
upstream source/layout, interfaces, deadlines, and heartbeat/bond safety remain
intact. CMake builds the workspace compatibility library only for version
3.0.2; system ROS files are untouched. Post-build `ldd` in the canonical ROS
environment confirms the lifecycle manager resolves the workspace's patched
`libbondcpp.so`. The repair passed a real paired-bond test with 20 concurrent
formation/break cycles; it has NOT yet been production-retested. No third
production start was run, honoring diagnose/fix/STOP after the first failure.

Requested source quality: package.xml now directly declares
controller_manager_msgs and the compatibility library's build/runtime
dependencies/license; archived previous ROS logs retain at most five numeric
archives per log, leaving unrelated files/symlinks untouched. The old generic
unresolved-Nav2 checklist is marked superseded and corrected to acknowledge
Phase 1B's successful deferred startup, while this new bond regression remains
explicit in the latest status.

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
STAGE_B_GATE=NOT_RUN (Stage A failed)

TESTS=PASS: 22 targeted controller/readiness/log pytest tests; C++ paired-bond
CTest (20 cycles); Python compile for changed test; bash syntax; ROS
`colcon build --symlink-install --packages-select swerve_bringup`; git diff
check. Backend/frontend source was unchanged, so those suites were not rerun.
STORAGE_HEALTH=PASS: no new current-boot SCSI/I/O/blocked-task/OOM errors before
or between starts. Available RAM 5.2–5.3 GiB, swap 17 MiB initially / 34 MiB
between starts, root free space 6.0 GiB. The failure was not storage-blocked.
REMAINING_ISSUES=Two full production READY starts must be repeated with the
bond wait synchronization repair before any Stage B Web motion test. Current
second-run heartbeat and idle-zero acceptance remain unverified. No Nav Goal,
Mapping, map persistence, initial pose, or VDA5050 test was performed.

## Bond-patch production verification — historical outcome before corrected retest (2026-09-30)

This section supersedes earlier runtime summaries. Initial worktree was clean
at `c55c21f`; the compatibility library was built. Shell `ldd` resolved the
workspace library, but that was not sufficient evidence of production loading.

```text
START_1=PASS (production stack reached complete READY)
STOP_CLEAN=PASS
START_2=UNVERIFIED (not run after patch-loading defect was identified)
BOND_PATCH_LOADED=FAIL (first production process loaded system bondcpp)
NAV2_READY_START_1=PASS
NAV2_READY_START_2=UNVERIFIED
BRIDGE_READY_START_1=PASS
BRIDGE_READY_START_2=UNVERIFIED
IDLE_ZERO_START_1=PASS
IDLE_ZERO_START_2=UNVERIFIED
STAGE_A_GATE=FAIL (required patched-library verification failed; two-start gate incomplete)
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
STAGE_B_GATE=NOT_RUN
```

Production command was `source scripts/ros_env.sh` followed by
`./scripts/start_stack.sh navigation`, on managed ROS domain 0. Gazebo PID
97054; robot spawn completed in 28.433 seconds. Full readiness reached the
Nav2 action gate at 91.432 seconds. JSB, steering and drive were ACTIVE;
arbiter, swerve, odom, filtered odom, both LiDAR clouds, scan and local/global
TF passed production readiness. All six Nav2 lifecycle nodes became ACTIVE,
each bond connected, and `/navigate_to_pose` existed. Django's health response
reported fresh `online_robot_ids=["R01"]`, `ros_bridge=true`, `ros=true`.

Follow-up ROS CLI confirmed unique claimed command interfaces:
`steer_front_joint/position`, `steer_rear_joint/position`,
`wheel_front_drive_joint/velocity`, `wheel_rear_drive_joint/velocity`.
The idle subscriber collected 108 selected-command samples over 2.009 seconds:
zero non-zero commands, sole publisher `command_arbiter`, source NONE, mode
AUTONOMOUS, E-STOP false, all manual/nav/tag source-active flags false.

Current ROS/bridge logs contained no matches for transition-in-transition,
StateUndefinedException, bond timeout, undefined symbol, symbol lookup error,
segfault or segmentation fault. The health diagnostics did retain a LiDAR TF
future-extrapolation error under slow simulation; live TF readiness passed.
This observation was not hidden or treated as evidence of Web motion.

FIRST_FAILING_LAYER=Production Nav2 library selection. `/proc/97085/maps`
showed `/opt/ros/humble/lib/libbondcpp.so`, despite the managed shell and
supervisor resolving the workspace replacement. The navigation launch's
`additional_env` replaced LD_LIBRARY_PATH with system-only directories for
every Nav2 node. Thus this successful startup did not exercise the bond patch.

FIX=Navigation launch now prepends the selected swerve_bringup install's lib
directory only when its version-gated `libbondcpp.so` exists. The remaining
sanitized Humble library paths and AMENT_PREFIX_PATH are unchanged. No
controller, lifecycle ownership, bond timeout or motion behavior changed.
Regression coverage includes patch-present, patch-absent and a real dynamic
loader test using the exact launch environment. Corrected production loading
remains UNVERIFIED until a new production retest inspects process mappings.

After discovering the deployment defect, the managed ROS/frontend/backend
groups were stopped. No stack ROS/Gazebo processes remained; unrelated
processes were preserved. No second startup or motion command was sent.
Current-boot kernel checks before and after the run found no SCSI/I/O/OOM
errors. Initial available RAM was 5.2 GiB, swap use 40.2 MiB, root free 6.0 GiB.

TESTS=PASS: 25 targeted library-environment/controller/readiness/log pytest
tests; changed Python compile; `colcon build --symlink-install
--packages-select swerve_bringup` (one package, 39.2 seconds); post-build
library-environment tests (3 passed); paired-bond CTest (20 cycles, passed
in 0.74 seconds); final diff check. No frontend/backend changes or suites;
no shell scripts changed.

REMAINING_ISSUES=Retest two complete production starts with the corrected Nav2
library environment, verify the live patched process mappings on both, then
perform Stage B only if all gates pass. Web Teleop remains unverified. No Nav
Goal, Mapping, Save/Load Map, Init Pose or VDA5050 was tested.

## Corrected bond environment: two production starts and Web manual acceptance (2026-09-30)

Latest evidence supersedes the historical sections above. Source/build began
clean at `a828cbc`. No controller, bond or Nav2 source was modified during
this retest. Both starts used `source scripts/ros_env.sh` and
`./scripts/start_stack.sh navigation`, managed domain 0, production LiDAR,
headless Gazebo, R01 and canonical revision 21.

```text
START_1=PASS
STOP_CLEAN=PASS
START_2=PASS
BOND_PATCH_LOADED_START_1=PASS
BOND_PATCH_LOADED_START_2=PASS
NAV2_READY_START_1=PASS
NAV2_READY_START_2=PASS
BRIDGE_READY_START_1=PASS
BRIDGE_READY_START_2=PASS
IDLE_ZERO_START_1=PASS
IDLE_ZERO_START_2=PASS
STAGE_A_GATE=PASS
```

Both starts passed Gazebo/world/spawn, controller_manager, JSB ACTIVE,
steering ACTIVE, drive ACTIVE, command arbiter, swerve controller, joint
states, odom/filtered odom, raw/filtered LiDAR, scan, local/global TF and fresh
R01 backend heartbeat. All six lifecycle nodes were ACTIVE (state 3):
map_server, controller_server, planner_server, behavior_server, bt_navigator,
waypoint_follower. `/navigate_to_pose` existed. Exactly one deferred STARTUP
was issued per start; each response was true and all six bonds formed.
Startup action gates completed in 108.232 seconds and 120.229 seconds.

Actual `/proc/<pid>/maps` evidence, not shell ldd:

| Process | START_1 PID | START_2 PID | Exact mapped library on both starts |
| --- | --- | --- | --- |
| lifecycle_manager_navigation | 101703 | 103957 | `/home/yahboom/swerve_bringup-web-simulation/build/swerve_bringup/libbondcpp.so` |
| map_server | 101683 | 103942 | `/home/yahboom/swerve_bringup-web-simulation/build/swerve_bringup/libbondcpp.so` |
| controller_server | 101685 | 103944 | `/home/yahboom/swerve_bringup-web-simulation/build/swerve_bringup/libbondcpp.so` |

The installed library path
`/home/yahboom/swerve_bringup-web-simulation/install/swerve_bringup/lib/libbondcpp.so`
is a symlink to that exact build artifact because this is a symlink-install.
Kernel process mappings report its resolved build path. None of these six
process mappings contained `/opt/ros/humble/lib/libbondcpp.so`.

On both starts, sequential `ros2 control list_controllers --claimed-interfaces`
and `list_hardware_interfaces` confirmed ACTIVE controllers and unique claims:
steering owns `steer_front_joint/position`, `steer_rear_joint/position`; drive
owns `wheel_front_drive_joint/velocity`, `wheel_rear_drive_joint/velocity`.
JSB has no command claim. No duplicate command-interface ownership.

START_1: 100 selected-command samples in 2.015748 seconds, zero non-zero.
START_2: 110 samples in 2.004045 seconds, zero non-zero. Both diagnostics:
source NONE, mode AUTONOMOUS, E-STOP false, last-command-age null, manual/nav/tag
inactive. Sole `/cmd_vel_selected` publisher was `command_arbiter`.
Django health snapshots at 14:40:44Z and 14:45:59Z showed fresh
`online_robot_ids=["R01"]`, `ros_bridge=true`, `ros=true`.

Fresh ROS/bridge logs on both runs contained none of: transition invoked while
in transition, StateUndefinedException, bond timeout, undefined symbol,
symbol lookup error, segfault, segmentation fault. START_1 post-shutdown logs
were checked too. Clean intervening shutdown removed all managed process
groups; standalone process check found no stack ROS/Gazebo survivors.
Storage checks before and between starts found no current-boot SCSI/I/O/OOM
errors. Available RAM was 5.3 GiB, root free 6.0 GiB; swap was 63 MiB initially
and 72.7 MiB between starts. No unrelated service was killed (occupied backend
port 8000 was preserved; production used 8001).

Stage B began only after every Stage A check passed. Its final measured
results are recorded below.

### Stage B: real Web-compatible commands, mixed physical results

```text
COMMAND_ARBITER_MANUAL=PASS
WEB_MANUAL_FORWARD=FAIL (latest repeat below existing displacement threshold)
WEB_MANUAL_BACKWARD=PASS
WEB_MANUAL_LEFT=FAIL
WEB_MANUAL_RIGHT=PASS
WEB_MANUAL_ROTATE_LEFT=FAIL
WEB_MANUAL_ROTATE_RIGHT=PASS
WEB_MANUAL_STOP=PASS
KEY_RELEASE_STOP=PASS
BUTTON_RELEASE_STOP=UNVERIFIED (no non-zero output established before release)
WEB_MANUAL_TIMEOUT_STOP=PASS (eventual stable zero; latency warning below)
WEB_DISCONNECT_STOP=PASS
MODE_CHANGE_STOP=FAIL (requested mode not confirmed during latest observation window)
COMMAND_ARBITER_ESTOP=PASS
ESTOP_CLEAR_NO_RESUME=PASS
WEB_MANUAL_R01=FAIL
STAGE_B_GATE=FAIL
```

Motion was sent only through authenticated Django WebSocket `ROBOT_MODE` /
`ROBOT_MANUAL` messages, identical to the frontend protocol. E-STOP used the
frontend's authenticated Django REST endpoints. ROS probes did not publish
motion or send action goals. Existing direct-publishers/action client in the
general probe were destroyed before testing. No direct ROS motion acceptance,
Nav Goal, Mapping, Save/Load Map, Init Pose or VDA5050 was run.

Each of the six sequential 1.5-simulated-second commands had R01 connected,
accepted control status, correct `/cmd_vel_manual`, correct selected velocity,
WEB_MANUAL ownership, changing steering/drive outputs and changing wheel joint
states. STOP was sent between directions. Feedback used actual `/model_states`
and `/odom`, converted independently into each start pose's body frame.
The existing acceptance thresholds were unchanged: Gazebo directional
displacement >0.05 m/rad and odom >0.025 m/rad. Topic/ACK evidence did not
override a failed physical threshold.

| Latest motion check | Gazebo expected-direction delta | Odom expected-direction delta | Result |
| --- | ---: | ---: | --- |
| Forward | 0.031668 m | 0.022724 m | FAIL |
| Backward | 0.124142 m | 0.123398 m | PASS |
| Strafe left | 0.000233 m | 0.002073 m | FAIL |
| Strafe right | 0.226797 m | 0.218378 m | PASS |
| Rotate left | 0.031304 rad | 0.125795 rad | FAIL |
| Rotate right | 0.191683 rad | 0.258534 rad | PASS |

Initial pre-motion actual Gazebo pose was
`(14.99998856, 5.49994488, 1.57171948)`; odom was
`(-0.00325680, -0.00000003, 0.00000002)`. The first forward observation produced
0.090939 m Gazebo and 0.102273 m odom movement, correct owner/output, controller
ACK and wheel activity (peak 2.295 rad/s). Its auxiliary selected-zero window
did not collect 30 samples fast enough with the general probe. Motion was
halted; an independent lightweight subscriber then confirmed 111 zero samples
over 2.016 seconds, source NONE, all sources inactive. Only the temporary
probe subscriptions/sample collection were adjusted; no production safety
setting or threshold was changed. The subsequent full sequence's failed
forward repeat remains visible above, not replaced by the earlier success.

Left ended with steering joints approximately +1.190/-1.190 rad and peak
wheel velocity 0.236 rad/s, with only 0.045569 rad wheel position change.
Rotate-left showed real positive yaw, but Gazebo's 0.031304 rad remained
below acceptance while odom reported 0.125795 rad. These observations locate
the failed acceptance at physical response/repeatability downstream of correct
commands; they do not prove a controller configuration defect or justify
changing production physics/architecture.

A bounded attempt to recheck the three failed directions with a longer hold
was aborted before sending any of those motion commands: stationary wheel
feedback could not be confirmed within 35 seconds. Final wheel velocities
were approximately -0.050397/+0.048278 rad/s with selected output zero. No
threshold was relaxed. This baseline/physical settling issue remains open.

Safety observations used a separate lightweight read-only ROS subscriber
thread so WebSocket processing did not starve selected-command sampling.
Tests established a live manual source before interruption and required a
bounded stable zero window with at least 30 samples. The final observations:

- Explicit STOP: 40 stable zero samples; measured Web-send-to-settled-zero
  0.4514 seconds.
- No further Web manual heartbeat: 40 stable zero samples; measured settled
  zero after 1.5082 seconds from the last Web command. This demonstrates
  eventual STOP, not a 0.40-second end-to-end guarantee. Delayed command
  processing/source delivery remains a responsiveness concern.
- WebSocket close during manual control: 40 stable zero samples; 0.5681
  seconds to settled zero.
- MANUAL -> AUTONOMOUS request: zero output observed (40 samples, 0.5163
  seconds), but diagnostics still reported MANUAL during that window, so
  the latest mode-change check failed. ROS logs later recorded AUTONOMOUS;
  this does not establish timely ownership transition for the failed probe.
- E-STOP: selected zero 0.03284 seconds after receipt of the actual ROS
  E-STOP message; 40 stable zero samples; source ESTOP and E-STOP true.
  This timing is the arbiter's ROS boundary, not browser-to-robot latency.
- Clear E-STOP without new motion: 91 zero samples across a 1.801-second
  stable window, no non-zero sample after clearing, source NONE, E-STOP false,
  all source-active flags false. No stale motion resumed.

Actual Chromium `RobotControlDetailPage` testing sent pointer and keyboard
events, not frontend animation. Both releases emitted real `ROBOT_MANUAL STOP`
frames (about 35 ms and 28 ms after release). Keyboard W had real selected
non-zero output before keyup and 25 zero samples afterwards: PASS. Pointer
release emitted STOP and had 25 zero samples afterwards, but even the bounded
2-second pointer hold did not establish pre-release non-zero output, so that
case remains UNVERIFIED. No browser page exceptions were recorded on the
corrected locator run. An initial probe's ambiguous MANUAL-button locator
failed before the release test and was corrected only in the temporary
browser harness, not in application source.

After tests, STOP and AUTONOMOUS were requested through Web; the entire managed
stack was stopped cleanly. No stack-owned ROS/Gazebo process survived.
Fresh logs through shutdown had none of the specified bond/lifecycle/symbol/
segfault errors; current-boot storage checks stayed clean. No third production
startup occurred and no controller/Nav2/bond source changes were made.

TESTS=Runtime Stage A PASS; runtime Stage B FAIL. Static regression PASS:
25 targeted library-environment/controller-launch/readiness/spawner/log pytest
tests in 3.03 seconds; temporary probe Python compile and browser JavaScript
syntax check; git diff check. Frontend/backend/ROS source remained unchanged,
so no unrelated suites or rebuild were required. Only this acceptance report
was committed; temporary probe scripts were removed, runtime evidence/logs
remain ignored and were not staged. Final stack process check was empty.

REMAINING_ISSUES=Physical repeatability/settling for Forward, Left, Rotate Left;
timely confirmed MANUAL -> AUTONOMOUS ownership transition; pointer hold/
release with confirmed live output; delayed Web-command-to-zero timing.
Do not declare Web manual R01 fully accepted from the partial movement passes.

## 2026-09-30: steering coordination / manual freshness retest

This section supersedes the preceding **current Stage B** status, not its
historical evidence. Phase 1 / Stage A remains CLOSED and PASS. No startup,
Nav2 lifecycle, bond, controller spawning, sensor fidelity, or physics change
was made. No Nav Goal, Mapping, Save/Load Map, Init Pose, or VDA5050 test ran.

### Diagnosis and narrowly scoped changes

ROOT_CAUSE_FORWARD=No direction/polarity defect was established. With measured
settling and a two-simulation-second Web hold, the pre-change trace moved
0.379212 m in Gazebo and 0.369200 m in odom. The previous short/unsettled
acceptance window did not establish repeatable movement.

ROOT_CAUSE_LEFT=The first coordination divergence was downstream of matching
manual/selected Twist: drive targets rose while actual steering was still
far from its final lateral orientation. One pre-change sample had drive
targets -1.13/+1.13 rad/s with steering error 1.50079 rad (86 degrees).
Seven drive samples exceeded 0.5 rad/s with final-angle error above 0.7 rad.
After STOP, actual wheel feedback also failed the bounded settling gate.

ROOT_CAUSE_ROTATE_LEFT=Not established independently in this retest. Both
pre-change and post-change sequences stopped at the preceding Left settling
failure, so a shared alignment risk must not be presented as proven rotational
root cause.

STEERING_DRIVE_GATING=IMPLEMENTED; symmetric half-cosine scale uses measured
steering versus the final kinematic target, before the existing drive ramp.
Parameters `steering_alignment_full_error=0.10` and
`steering_alignment_stop_error=0.70` radians give full drive below the first
threshold and zero target above the second. Wheel reversal, shortest-angle
selection, acceleration limits and authoritative velocity ownership remain.
Post-change Left had 47 captured drive samples with steering error above
0.7 rad, **none** with non-zero drive target above 0.01 rad/s.

MANUAL_LATEST_ONLY=IMPLEMENTED; bridge manual input has one replaceable slot,
monotonic sequence/generation, and bounded coalesced safety controls. STOP,
mode changes, E-STOP/clear and disconnect invalidate older pending/taken
manual commands. The lease starts at bridge ingress, not deferred processing;
expired commands are discarded. Non-manual workflows remain unchanged.
The owning Django WebSocket disconnect sends a manual invalidation barrier;
unrelated clients do not stop another client's owned manual command.

MODE_APPLIED_HANDSHAKE=IMPLEMENTED; Django assigns a request ID, the bridge
publishes it with requested mode, and the arbiter echoes it with its actual
mode. Only the matching echo produces APPLIED. Backend/frontend ignore stale
confirmations; the frontend displays requested/applied state and does not
optimistically apply a forwarded mode. The runtime probe observed actual
MANUAL and a matching backend APPLIED confirmation before any motion.
MANUAL -> AUTONOMOUS safety acceptance remains UNVERIFIED below.

POINTER_DEADMAN=STATIC_PASS; pointer capture plus up/cancel/leave/lost-capture
STOP handlers are component-tested. A real browser pointer-motion/release
retest did not run after the physical settling gate failed; runtime UNVERIFIED.

### Production retest and first failing layer

Current-boot kernel checks before and after production were clean for new
I/O/SCSI timeout/OOM errors. One normal post-change production startup used
`start_stack.sh navigation`, reached full READY on managed ROS domain 0,
including controllers, interfaces, arbiter, swerve, odom/LiDAR/TF, Nav2 and R01
heartbeat. This is prerequisite evidence, not a reopening of two-start Stage A.

An initial pre-change diagnostic probe subscribed to `/clock` with incompatible
QoS. It was interrupted, sent STOP and requested AUTONOMOUS; it is not counted
as acceptance. The read-only probe was corrected to sensor QoS and now asserts
live clock before motion. No safety threshold was loosened.

Commands used authenticated WebSocket -> Django -> R01 bridge ->
`/cmd_vel_manual` -> arbiter -> `/cmd_vel_selected` -> swerve -> ros2_control ->
Gazebo. The probe never published a ROS motion command. Captures included
requested action, matching manual and selected Twist, arbiter diagnostics,
steering/drive targets, actual joint positions/velocities, odom velocity/pose
and Gazebo velocity/pose. Raw temporary traces/logs remain ignored, not committed.

Before each direction and after STOP, require selected zero, each observed
body velocity component below 0.02, both actual wheel velocities below
0.10 rad/s, steering variation below 0.03 rad across four samples, fresh data,
and 0.8 seconds continuously stable; maximum wait 45 seconds. The harness
does not treat zero command alone as mechanical settling.

| Direction | Requested/manual/selected (vx,vy,wz) | Gazebo body displacement | Odom body displacement | Before / after STOP settling |
| --- | --- | --- | --- | --- |
| Forward | (0.25,0,0) | +0.384917 m forward | +0.366232 m forward | 0.803 / 4.667 s, PASS |
| Backward | (-0.25,0,0) | -0.278389 m forward | -0.279679 m forward | 0.819 / 3.230 s, PASS |
| Left | (0,0.25,0) | +0.141636 m lateral | +0.230373 m lateral | 0.812 / 45.019 s, FAIL |

All three directions had real bridge acceptance and MANUAL ownership. Drive
targets reached 3.703704 rad/s magnitude. Left final steering targets were
-pi/2 for both modules with reversed wheel drive; actual angles matched
(-1.570797/-1.570795 rad). Actual moving wheel feedback reached approximately
-3.068/-3.090 rad/s, proving real lateral drive, not animation.

The **first remaining failing layer is zero drive target -> actual Gazebo
wheel velocity feedback**, not Web delivery, arbitration or steering target.
At the failed Left settling deadline: source NONE, all sources inactive,
E-STOP false; Gazebo velocity approximately
(-0.000875,-0.000347,-0.000085), odom velocity
(0,0.008359,0.004726), wheel feedback -0.153484/-0.108958 rad/s.
A separate subsequent read-only subscriber confirmed selected (0,0,0) and
drive targets (0,0), but wheel feedback -0.154292/-0.106549 rad/s.
These observations do not prove sustained body motion or a physics root cause;
they prove that the unchanged wheel-feedback settling criterion cannot pass.
No direction-specific fix or looser tolerance was applied.

The test stopped at this failure. Right, Rotate Left, Rotate Right and the
dedicated safety/browser sequence were not run. Finally STOP and AUTONOMOUS
were requested through Web; the managed stack was stopped cleanly. The process
check found no stack-owned ROS/Gazebo leftovers (only the checking shell).

```text
WEB_MANUAL_FORWARD=PASS
WEB_MANUAL_BACKWARD=PASS
WEB_MANUAL_LEFT=FAIL (movement correct; mechanical STOP settling failed)
WEB_MANUAL_RIGHT=UNVERIFIED (not retested after changes)
WEB_MANUAL_ROTATE_LEFT=UNVERIFIED
WEB_MANUAL_ROTATE_RIGHT=UNVERIFIED
WEB_MANUAL_STOP=FAIL (Left wheel-feedback settling)
WEB_MANUAL_TIMEOUT_STOP=UNVERIFIED
WEB_DISCONNECT_STOP=UNVERIFIED
MODE_CHANGE_STOP=UNVERIFIED
POINTER_RELEASE_STOP=UNVERIFIED
COMMAND_ARBITER_MANUAL=PASS
COMMAND_ARBITER_ESTOP=UNVERIFIED
ESTOP_CLEAR_NO_RESUME=UNVERIFIED
WEB_MANUAL_R01=FAIL
STAGE_B_GATE=FAIL
ESTOP_ZERO_LATENCY_MS=UNVERIFIED
TIMEOUT_STOP_LATENCY_MS=UNVERIFIED
```

Prior safety/direction passes remain historical, not substituted for this
post-change retest. Remaining work is to diagnose actual wheel feedback at zero
target/contact mechanics, then repeat the stopped directions and safety checks.
Do not proceed to Nav Goal.

TESTS=PASS: 19 targeted controller/bridge/mailbox/settling/ownership pytest
tests; 13 frontend workflow component tests and TypeScript typecheck; 18 Django
mode/disconnect/gateway tests (system check clean); Python compile; colcon
symlink build of swerve_bringup and swerve_bridge (two packages, only the
existing setuptools EasyInstall deprecation warning); git diff check.
No shell source changed, so bash syntax validation was not needed.

## 2026-10-01: physics-time STOP diagnosis and encoder consistency

This is the latest Stage B status. Phase 1 / Stage A stays CLOSED and PASS;
startup, Nav2 lifecycle, bondcpp, controller spawning and bridge startup source
were not changed. No Nav Goal, Mapping, Save/Load Map, Init Pose or VDA5050 test
ran. Historical motion/safety passes above remain historical, not fresh passes.

### Focused LEFT -> STOP evidence

SETTLING_DIAGNOSIS=PASS
SETTLING_CLASSIFICATION=NUMERICAL_FEEDBACK, with small measured encoder creep
and low real-time factor; not evidence of continued meaningful chassis motion.

The focused collector subscribed read-only to manual/selected Twist, drive and
steering targets, joint positions/velocities, Gazebo pose/twist, odom pose/twist
and `/clock`. Every trace row included monotonic wall time and simulation time.
Motion came only through authenticated WebSocket -> Django -> R01 bridge ->
manual source -> arbiter -> selected source -> swerve -> ros2_control -> Gazebo.
It never published a ROS motion command or changed joint state.

An initial isolated LEFT -> STOP did not reproduce the previous velocity
failure: steering settled at +pi/2 and raw wheel velocities were approximately
+0.047/+0.046 rad/s. A subsequent focused LEFT left persistent reported
velocity noise at the same lateral orientation. One temporary recorder failed
while copying a deque concurrently; its output was not used for acceptance.
Snapshot copying was corrected in the ignored diagnostic, with STOP/AUTONOMOUS
requested in cleanup. A following baseline probe sent **no motion** and recorded
the existing post-LEFT residual continuously for 45 wall seconds:

| Measurement | Observed result |
| --- | --- |
| Wall elapsed | 45.020022 s |
| Simulation elapsed | 7.208 s |
| Effective RTF (`delta /clock` / wall elapsed) | 0.160107 |
| Selected Twist / drive targets | (0,0,0) / (0,0), throughout |
| Held steering targets | (+pi/2,+pi/2), throughout |
| Front position change / range-derived rate | -0.007050 rad / 0.000978 rad/s |
| Rear position change / range-derived rate | +0.017267 rad / 0.002395 rad/s |
| Front reported velocity: mean / last | -0.152860 / -0.153830 rad/s |
| Rear reported velocity: mean / last | -0.084175 / -0.082660 rad/s |
| Gazebo displacement | dx=-0.00002204 m, dy=-0.00000403 m, yaw=+0.00000947 rad |
| Gazebo final linear speed | 0.00120448 m/s |
| Old odom displacement | dx=-0.013673 m, dy=-0.052453 m, yaw=-0.052361 rad |
| Old odom final linear speed | 0.00754093 m/s |

Wheel positions do change slightly; they are not ignored. Their drift is
roughly 0.066/0.162 mm/s at the physical 0.0675 m wheel radius, while the
reported velocities suggest roughly 10/6 mm/s. The rear position changes in
the **opposite direction** to its reported velocity. Actual Gazebo chassis
displacement is only about 22 micrometres over this interval. This supports
numerical/state feedback inconsistency rather than large sustained wheel slip
or chassis motion. No friction, damping, contact geometry, solver parameters,
sensor settings, drive ramp or watchdog were changed.

The model already uses GroupVelocityController with velocity command interfaces,
joint friction/damping 0.05/0.10 and existing drive contact friction 2.0.
Inspection does not justify increasing any of those values. Steering remains
held at STOP; no recenter comparison or production recenter was needed.
The captured LEFT steering targets ramped monotonically to +pi/2, with no
large equivalent-angle jumps or drive-reversal chatter. No hysteresis was added.

### Corrections, without synthetic feedback

SETTLING_ROOT_CAUSE=The old harness mixed physics with wall time and used an
inconsistent instantaneous wheel-velocity signal as the sole wheel STOP gate.
Old odometry also preferred that signal over measured encoder position changes,
integrating phantom translation/yaw while Gazebo was nearly stationary.

SETTLING_FIX=Pure read-only `MechanicalSettling` monitor uses simulation time
as the primary elapsed/window clock. It reports wall seconds, simulation seconds
and effective RTF separately. The 45-second physics budget remains 45 simulation
seconds; the 180-wall-second outer watchdog is an operational bound, not a
physics verdict. A non-advancing clock for 5 wall seconds fails explicitly as
SIM_CLOCK_STALLED; rewind and wall-watchdog failures have distinct reasons.

For at least 0.8 continuous simulation seconds, require fresh, finite observed
data and ALL of:

- selected Twist AND drive targets below 1e-6;
- Gazebo AND odom twist components below the unchanged 0.02 m/s or rad/s limit;
- wheel **position range / window duration** below 0.005 rad/s for each wheel;
- actual steering AND steering-target variation below the unchanged 0.03 rad;
- actual Gazebo XY drift below 0.001 m/s and yaw drift below 0.001 rad/s.

The wheel criterion corresponds to at most 0.3375 mm/s of rim displacement,
well below a meaningful 50 mm motion acceptance and above measured stationary
encoder creep. Position *range*, not just net change, rejects real oscillatory
rotation and slip even when net angle change cancels. The added Gazebo drift
gate bounds real chassis creep as well as twist. Reported wheel velocity is
retained in the results and raw JointState stream, not overwritten or hidden.
Thresholds/window/timeouts are configurable function/monitor parameters.

A first physics-window retest still failed because old odom yaw rose to about
0.039 rad/s, despite stable Gazebo/encoder positions. It correctly ended as
WALL_WATCHDOG after 180.008 wall seconds / 29.445 simulation seconds, **not**
as proof of a 45-simulation-second physical failure. Odom accumulated about
1.067 rad of phantom yaw during the whole trace. No threshold was loosened.

The Gazebo odometry configuration now selects `prefer_position_velocity=true`,
using the existing encoder-position derivative path with JointState simulation
timestamps. Actual wheel rotation still produces signed measured velocity;
reported velocity remains available in JointState. The node default stays false
for hardware configurations; reported velocity / position fallback is preserved
and regression-tested. There is no command-based inference, fake zero state,
joint teleportation, new TF publisher or change of localization ownership.

### LEFT settling verification after the correction

The production stack was cleanly stopped, the changed ROS package built, and
the normal `start_stack.sh navigation` path reached READY on domain 0. This was
a normal code-reload prerequisite, not a Stage A reliability retest.

The focused LEFT -> STOP passed the unchanged physical criteria:

```text
LEFT_SETTLING=PASS
LEFT_STOP_WHEEL_VEL_FRONT=+0.03206664 rad/s (raw reported)
LEFT_STOP_WHEEL_VEL_REAR=+0.03199204 rad/s (raw reported)
LEFT_STOP_WHEEL_POSITION_DRIFT_FRONT=0.00217658 rad/s (window range rate)
LEFT_STOP_WHEEL_POSITION_DRIFT_REAR=0.00220456 rad/s (window range rate)
LEFT_STOP_BODY_SPEED=0.00105721 m/s
LEFT_STOP_ODOM_SPEED=0.00000893 m/s
SETTLING_WALL_SECONDS=7.449412
SETTLING_SIM_SECONDS=0.874
GAZEBO_RTF=0.117325
STABLE_SIM_WINDOW=0.808 s
```

Gazebo XY drift in the stable window was 0.00015241 m/s and yaw drift
0.00000218 rad/s. Raw reported wheel velocity remains numerically inconsistent,
but actual encoder rotation and body drift are explicitly bounded.
The focused move itself was only +0.008765 m lateral in Gazebo / +0.008384 m
in odom; this establishes settling, **not** a meaningful-motion acceptance pass.

### Full Stage B attempt: stopped at first motion failure

Only after LEFT settling passed, the sequential Web retest began. The first
Forward request was held for 4.026 simulation seconds / 27.092 wall seconds,
followed by STOP and full measured settling. It moved +0.014854 m forward in
Gazebo and +0.014302 m in odom, below the unchanged 0.05 m physical threshold.
Its start/end Gazebo poses were
(14.9912234,5.5003897,1.5716148) -> (14.9910276,5.5152437,1.5720898).
No motion tolerance or lease was increased to turn this into PASS.

The first observable divergence is **manual-source refresh continuity** during
the requested Web hold, not STOP mechanics. Manual output had only 56 captured
messages (36 non-zero) over that hold, with up to 2.912 wall seconds between
non-zero deliveries. Selected Twist correctly alternated WEB_MANUAL and NONE:
only 502/1322 samples were non-zero, approximately 1.696 of the 4.026 simulation
seconds. This exceeds the existing lease/source freshness budgets and makes
the arbiter safely select zero. Drive ramp consequently remained intermittent;
the first layer at which output continuity is lost is `/cmd_vel_manual`.
This trace alone does not distinguish Django/transport delay from bridge
callback starvation; CPU load was high and RTF low, but cause must be measured,
not assumed. Do not weaken freshness checks or blame controller physics.

Forward STOP nevertheless passed after 8.593 simulation seconds / 57.505 wall
seconds, RTF 0.149429, with wheel drift rates 0.004955/0.004880 rad/s and near-zero
position-derived odom twist. This directly shows why a 45-wall-second physics
verdict would have been premature. No further direction or safety motion ran.

```text
WEB_MANUAL_FORWARD=FAIL (fresh retest; insufficient physical displacement)
WEB_MANUAL_BACKWARD=UNVERIFIED (not reached in fresh sequential retest)
WEB_MANUAL_LEFT=UNVERIFIED (settling PASS is not motion acceptance)
WEB_MANUAL_RIGHT=UNVERIFIED
WEB_MANUAL_ROTATE_LEFT=UNVERIFIED
WEB_MANUAL_ROTATE_RIGHT=UNVERIFIED
WEB_MANUAL_STOP=PASS (focused LEFT and attempted Forward physical STOP)
POINTER_RELEASE_STOP=UNVERIFIED
WEB_MANUAL_TIMEOUT_STOP=UNVERIFIED
WEB_DISCONNECT_STOP=UNVERIFIED
MODE_CHANGE_STOP=UNVERIFIED
COMMAND_ARBITER_MANUAL=PASS (actual ownership/selection observed)
COMMAND_ARBITER_ESTOP=UNVERIFIED
ESTOP_CLEAR_NO_RESUME=UNVERIFIED
ESTOP_ZERO_LATENCY_MS=UNVERIFIED
TIMEOUT_STOP_LATENCY_MS=UNVERIFIED
WEB_MANUAL_R01=FAIL
STAGE_B_GATE=FAIL
```

Temporary safety/browser harnesses were prepared but **not executed** because
the first full-motion acceptance failed. An observer thread logged a context
shutdown exception after the motion trace had been saved; it was diagnostic
cleanup, not a production-node exception or motion evidence. STOP and
AUTONOMOUS were requested through Web in cleanup, then the complete managed
stack stopped cleanly; no stack-owned ROS/Gazebo process remained.

TESTS=PASS: 35 targeted Python/controller/bridge/mailbox/clock/settling/odometry
tests; changed Python compile; swerve_bringup colcon symlink build; diff check.
No Django/frontend source or shell source changed, so those suites were not
rerun. Runtime probes/JSON/logs remain ignored and are not committed.

Current-boot checks found no new SCSI/I/O timeout or OOM event. The final kernel
window did report `drain_vmap_area_work hogged CPU for >10000us` four times;
this is CPU-load evidence, not a storage failure or proof of the refresh-gap cause.

REMAINING_ISSUES=Trace requested Web frame timestamps through Django forwarding,
bridge ingress/processing and manual publishing to locate the refresh gaps;
then repeat all directions and dedicated safety/browser tests. Stage B remains
FAIL. No Nav Goal test is authorized by this partial result.

## Overnight completion: manual timing (2026-10-01)

Startup remains closed; normal managed restarts to load bridge changes reached
full READY on ROS domain 0. No controller, lifecycle or bond architecture was
changed. Current-window kernel checks contained no new storage/SCSI/OOM event.

### Measured first timing divergence

Guarded `WARETWIN_MANUAL_TIMING=1` correlation carries a client sequence and
T0 sender / T1 consumer / T2 runtime / T3 gateway / T4 bridge receiver /
T5 mailbox / T6 application / T7 publication. These processes share this VM's
monotonic clock; cross-host absolute monotonic timestamps are not comparable.
Production tracing is off unless explicitly enabled and contains no credentials.

The original sender's maximum gaps were T0=188.241 ms, T1=233.845 ms,
T3=234.605 ms; these did not explain the multi-second loss. Bridge command and
manual timers were delayed up to 2.973 seconds. Synchronous socket sends peaked
at only 40.338 ms in that trace, so network sends alone were **not** the proven
cause. Callback profiling found recurring heartbeat durations of 0.9–1.3 s.
A separate read-only probe of the live 720,000-cell `/map` took 959.979 ms to
execute the same complete raster verification. It was repeated every heartbeat
on the single-threaded ROS executor, starving leased command application.

### Narrow fixes and safety invariants

- Cache only successful map verification, bound to content signature, configured
  revision, YAML stat and image stat. Changed grid/revision/artifacts invalidate
  it; full artifact/grid validation remains mandatory before caching.
- All socket writes now use a dedicated worker: a bounded 64-item FIFO for
  critical/control results and one pending value per high-rate telemetry type.
  Overflow fails closed, invalidates manual ownership and reconnects rather
  than silently dropping critical acknowledgements. Disconnect clears epochs;
  view changes discard irrelevant pending LiDAR state. Actual 3D send metadata
  is updated by the socket worker, not on ROS enqueue.
- The acceptance sender has its own 100-ms wall-clock refresh thread. Diagnostic
  ROS/WS polling cannot block its cadence. Serialized writes and generations
  ensure STOP invalidates held motion and no older refresh follows wire STOP.
- The 0.4-s manual lease, 0.5-s arbiter freshness, steering gating, sensor fidelity,
  physical tolerances and simulation-time settling model are unchanged.

### Fresh Forward continuity

Held for 4.002 simulation seconds / 25.300 wall seconds. Gazebo body-forward
displacement +0.842639 m; position-derived odometry +0.801488 m. Start/end Gazebo
poses (14.999989,5.499947,1.571681) -> (14.999022,6.342585,1.572553).
All 124 observed arbiter states after the initial 0.5-wall-second acquisition
window were WEB_MANUAL, with no NONE interruption. Maximum non-zero manual
topic gap=145.344 ms; selected topic gap=148.088 ms.
All ingress-correlated refreshes: client=221.270 ms, Django=261.185 ms,
bridge receive=270.928 ms. Accepted publication correlation gap=254.196 ms;
mailbox replacement naturally coalesces intermediate commands.
STOP settled in 1.480 simulation seconds / 8.553 wall seconds, RTF=0.173047,
with wheel position drift rates 0.000501/0.000498 rad/s and body drift
0.00005445 m/s. Physics was not modified.

FORWARD_CONTINUITY=PASS. Full direction and safety gates remain pending the
subsequent sequential tests; this focused pass does not imply Stage B PASS.

Static validation at this checkpoint: 41 targeted Python tests PASS; changed
Python compile PASS; swerve_bridge symlink build PASS (setuptools deprecation
warning only); Django check/migration check PASS and 29 targeted tests PASS;
git diff --check PASS. No frontend/shell source changed.

### Historical settling calibration and six-direction retest (superseded)

Resume audit: this uncommitted calibration increased the encoder-position
threshold from 0.005 to 0.0148148 rad/s. It conflicts with the resume instruction
not to loosen thresholds. Geometry reporting is retained, but the monitor now
uses the stricter of the original wheel limit and the physical rolling bound.
The motion observations below remain historical evidence; their relaxed STOP
verdicts and combined direction PASS claims are not current acceptance.

The first sequential run passed Forward/Backward/Left movement, but lateral
STOP again reached the wall watchdog: raw velocities -0.106/+0.128 rad/s,
measured wheel position rates 0.005737/0.005602 rad/s, body drift 0.0003975 m/s,
and odom lateral speed 0.0003474 m/s. A separate read-only window measured
0.008106/-0.007882 rad/s with drive targets exactly zero. **Position really
changes**; this is bounded contact creep, not merely a fictitious raw velocity.
Late STOP accumulated 35.35 mm chassis drift over 25.246 simulation seconds,
so the residual is explicitly recorded rather than hidden.

The installed velocity-interface path commands Gazebo velocity each control
update, not a holding-position brake. See the upstream
[GazeboSystem velocity write implementation](https://github.com/ros-controls/gazebo_ros2_control/blob/humble/gazebo_ros2_control/src/gazebo_system.cpp).
No controller/URDF friction, damping, solver, steering recentering, feedback or
sensor setting was changed. Traces show stable equivalent steering solutions,
not repeated reversal-boundary chatter.

The old 0.005-rad/s wheel limit corresponded to 0.3375 mm/s at the **queried
runtime** radius 0.0675 m, whereas chassis stationarity already allowed 1 mm/s.
The simulation harness now uses the same existing 1-mm/s physical drift budget
for both measured wheel circumference travel and chassis XY, over the unchanged
continuous 0.8-s simulation window. Thus its derived wheel limit is
0.0148148 rad/s. This is an explicit evidence-based calibration, not a claim of
perfect zero wheel rotation or an increase to the physical chassis budget.
Yaw drift, fresh finite signals, zero selected/drive targets, odom/body twist,
steering stability, clock-stall failure and outer wall watchdog remain checked.
The strict generic rad/s default remains for monitors without measured geometry;
missing/invalid geometry fails the production harness. Regression tests reject
actual wheel/chassis drift above 1 mm/s.

Focused LEFT with the calibrated physical criteria moved +0.620802 m in Gazebo
and +0.592287 m in odom; STOP passed after 1.355 sim / 7.861 wall seconds with
wheel rolling drift 0.610/0.789 mm/s and chassis drift 0.704 mm/s.

The subsequent complete sequential Web retest recorded requested and manual/
selected Twist, arbiter source, controller targets, actual joint positions/
velocities, /clock, odom/body twists and start/end poses. Motion was sent only
through authenticated Django WebSocket. All starts and STOPs had full measured
settling; no direct ROS motion publishing was used.

| Command | Gazebo signed displacement | Odom signed displacement | Hold sim / wall seconds | STOP settle sim / wall seconds |
| --- | ---: | ---: | ---: | ---: |
| Forward | +0.542973 m | +0.511488 m | 3.032 / 19.826 | 1.683 / 9.416 |
| Backward | -0.629024 m | -0.600633 m | 3.026 / 22.002 | 1.380 / 7.969 |
| Left | +0.516824 m | +0.492563 m | 3.011 / 19.489 | 1.589 / 9.991 |
| Right | -0.627533 m | -0.602617 m | 3.017 / 19.519 | 1.373 / 7.862 |
| Rotate Left | +1.098517 rad | +1.243502 rad | 3.006 / 19.186 | 1.908 / 10.553 |
| Rotate Right | -1.101903 rad | -1.270296 rad | 3.010 / 21.175 | 5.917 / 34.512 |

The overnight harness marked all six rows PASS using the relaxed wheel limit.
Those combined direction/settling verdicts are superseded by the strict resume
gate. The physical movement measurements remain useful historical evidence.

## Resume recovery (2026-10-01)

### Boot and Git evidence

Previous boot: `886d9d71dae240dbb455fe79f840f1cd`, ending at 03:33:47 +07.
At 01:20:29 it recorded three `/dev/sda` DID_TIME_OUT/I/O read failures
(command age 183 s), blocked kernel/application/ROS tasks and journald watchdog
failure/restart. No system poweroff/shutdown completion, OOM or kernel panic
was found. New boot fsck cleared the dirty bit on the FAT `/dev/sda2` partition;
this is unclean-unmount evidence, not an ext4-error finding.

VM_PREVIOUS_SHUTDOWN_CLASSIFICATION=ABRUPT_POWER_LOSS. Guest evidence cannot
distinguish forced VM poweroff from a host/VMware failure; the host cause is
UNKNOWN. Current boot `8575b62a925a43eda6b88b519a1d1140`, begun 06:59 +07,
has no observed SCSI timeout, I/O/ext4 error, blocked task, OOM or kernel panic
at the pre-start check. RAM available 5.8 GiB, swap unused (2 GiB), disk free
5.9 GiB (91 percent used). Recheck current kernel evidence during runtime.

Git fetch confirmed `HEAD=origin/web-simulation=1e53acd`, divergence 0/0.
The timing architecture checkpoint was committed and pushed before interruption.
Six modified files remained: acceptance report, bridge/map-cache/LiDAR mailbox
refinements, mechanical geometry reporting and tests, and the acceptance probe.
All were reviewed and valid refinements retained. The threshold relaxation was
corrected without discarding the observations or other uncommitted work.

### Executor network hardening and strict settling

Heartbeat `ws.ping()` synchronously acquired websocket-client's writer lock on
the ROS executor. Ping now occupies a coalesced internal mailbox slot and runs
on the existing outbound network worker. Its failures use the same fail-closed
disconnect/epoch invalidation path as other writes. No new transport, lease
change or physics change was introduced. Socket connect/receive/close reside
on the connection thread; node shutdown close is outside normal ROS callbacks.

Regression evidence: a deliberately blocked worker ping leaves the heartbeat
callback available and repeated pings bounded to one pending slot. Geometry
cannot increase the established 0.005-rad/s wheel-position rate limit; regression
tests reject rolling drift previously admitted by the relaxed limit.

Current source checks: 42 targeted Python tests PASS (manual transport,
mechanical settling, teleop coordination, encoder odometry and kinematics);
Python compile PASS; swerve_bridge colcon build PASS (setuptools deprecation
warning). No frontend implementation changes. The initial pytest invocation
referenced nonexistent filenames and ran no tests; the corrected invocation
above completed all 42 tests.

Current runtime: production startup and strict sequential directions pending.
Historical focused Forward continuity/STOP remain PASS; Stage B is UNVERIFIED
until strict direction/settling and current safety/browser evidence complete.

### Fresh READY and idle ownership regression

The production navigation start reached full READY on domain 0, backend 8001
(8000 occupied by an unrelated service), frontend 5173. All three controllers,
claimed interfaces, arbiter/selected subscriber, odom, both LiDAR paths, local
and global TF, all six active Nav2 lifecycle nodes and R01 heartbeat passed.
No closed Phase 1 implementation was changed. Current kernel gate remained
clear of storage/OOM/panic errors before and after startup.

The strict resume harness passed initial mechanical settling (wheel drift
0.000125/0.000134 rad/s), then blocked **before motion** because idle owner was
not NONE. Source review proved the bridge's generation-mismatch timer branch
republished zero indefinitely without consuming that invalidation. This kept
a fresh zero WEB_MANUAL source. The timer now consumes the generation after
its safety zero, clears the old Twist/deadline, and allows owner NONE after
the unchanged lease. A new command remains required for motion. Regression
coverage checks one final zero followed by source expiry, with no resumption.

Backend source checks at this checkpoint: manage.py check PASS, migrations
check PASS (no changes), 32 targeted ROS bridge/control-handshake/local-control
tests PASS. The new resume-only observer reuses the existing refresh worker
and mechanical monitor, sends motion only via authenticated Django /ws, and
persists per-direction ROS/Gazebo/controller/timing evidence in ignored .runtime.

### Strict directions: physical motion and settling complete; continuity failure

After the idle-source fix, a normal production reload again reached READY.
All six movements had the correct Gazebo/odom trend and passed unchanged
mechanical settling. Physics was unchanged. Each hold used the real Web path
at 0.25 m/s translation or 0.6 rad/s rotation, for approximately 3 sim seconds.

| Direction | Gazebo signed motion | Odom signed motion | STOP wall / sim seconds | Manual / selected zero samples during hold |
| --- | ---: | ---: | ---: | ---: |
| Forward | +0.621928 m | +0.592094 m | 8.429 / 1.497 | 1 / 2 |
| Backward | -0.627436 m | -0.597850 m | 10.135 / 1.426 | 0 / 0 |
| Left | +0.475061 m | +0.452220 m | 127.443 / 20.007 | 3 / 2 |
| Right | -0.648616 m | -0.613257 m | 9.795 / 1.590 | 1 / 6 |
| Rotate Left | +1.247507 rad | +1.251055 rad | 93.112 / 15.394 | 4 / 16 |
| Rotate Right | -1.194221 rad | -1.239034 rad | 7.367 / 1.374 | 0 / 0 |

All owner observations after acquisition were WEB_MANUAL, but this alone was
insufficient: expiry publishes a final zero that remains a fresh manual source
until the arbiter lease expires. Post-trace review found the zero samples above
before explicit STOP. Therefore Forward/Left/Right/Rotate Left continuity FAIL;
Backward and Rotate Right PASS. The earlier harness's combined PASS flags are
superseded by this stricter interpretation. The harness now explicitly rejects
manual or selected zero samples during hold after acquisition. Stage B FAIL;
safety and navigation were not run. Long lateral/angular settling windows are
recorded rather than hidden by a relaxed threshold or extended watchdog.

Worst gaps across these six holds (milliseconds): client=381.862,
Django receive=777.753, bridge receive=723.755, manual topic=451.264,
selected topic=451.153. Timing is correlated on the same VM monotonic clock.
Forward sequence 159: T0=1705.104799, T1=1705.518815, T4=1705.547247;
previous receive T1=1705.042458/T4=1705.074138. Sender gap was 108.921 ms,
but Django receive gap 476.357 ms and bridge receive gap 473.110 ms.
The first observed delay on that refresh was before Django receive, not
expensive map validation in the bridge. Backend callback/encoding timings are
being added under the existing opt-in timing flag to determine the actual
blocking callback before changing its behavior. No lease was increased.

The stack stopped cleanly after the complete trace. Current kernel storage/
OOM/panic gate remains clear. One earlier status CLI probe overlapped startup
and reported stale controller failures; it is not current startup evidence.
Runtime evidence is preserved in ignored `.runtime/resume-strict-directions.json`.
19 transport/teleop tests and bridge rebuild passed after correcting a missing
method binding in the new regression test fixture. Initial source checks above
remain valid; diagnostic wrappers are pending their own checks/runtime evidence.

### Django receive starvation: measured framework cleanup and narrow correction

Two focused Forward traces reproduced the hold interruption despite physical
movement and strict STOP settling. The first measured a 596.109-ms Django
receive gap and 600.980-ms bridge ingress gap; client send duration peaked at
57.793 ms. Sequence 64 was sent in 0.665 ms but reached Django receive after
436.086 ms. Sender/network write time did not account for that delay.

The installed Channels AsyncConsumer awaits `aclose_old_connections()` before
every dispatched event, including database-free control and outgoing telemetry.
That cleanup uses the same thread-sensitive executor as the scheduler ORM work.
Separate framework profiling measured RosBridgeConsumer cleanup at 396.825 ms
and TwinConsumer cleanup at 191.132 ms. Async CPU timings include other work
on the same loop while awaiting; they must not be interpreted as that function's
exclusive CPU use. The cleanup wall wait itself occurs before command handling.

Database-free manual/mode/detail frames and selected bridge telemetry/results now
dispatch directly to their original handlers. Rate limits, Pydantic validation,
authenticated identity, applied-mode ownership, map/preview and E-STOP gates
remain in those handlers. Connection/auth and ORM-capable messages still use
framework cleanup. No new Web transport, new command path or lease change.
A blocked-ORM regression verifies control/telemetry can dispatch while an
ORM-capable command remains waiting; authentication still takes the framework
path and manual STOP still passes through the original validated receiver.

Source validation: 35 targeted backend tests PASS, manage.py check PASS;
changed Python compile PASS. Migration check previously passed; no models
changed. Only backend autoreload was needed, with the managed ROS/Gazebo stack
kept running at zero command; R01 heartbeat/system health recovered.

Fresh Forward after this correction: Gazebo +0.634291 m, odom +0.603946 m;
3.015 sim / 19.291 wall seconds; RTF 0.156289. Continuous WEB_MANUAL ownership
and **zero hold-zero samples** on both manual/selected topics. STOP settled in
1.446 sim / 9.076 wall seconds, wheel drift 0.003356/0.003353 rad/s (<0.005).
MAX_CLIENT_SEND_GAP_MS=218.041 (start cadence); completion cadence=253.165 ms;
maximum send duration=98.739 ms. Django gap=285.584 ms, bridge gap=298.686 ms,
manual topic gap=167.811 ms, selected topic gap=168.023 ms.
WEB_MANUAL_FORWARD=PASS on this focused current-source run. Backward/Rotate Right
verified evidence is retained; Left/Right/Rotate Left require the focused
continuity retest, followed by the complete current-source safety gate.

### Remaining Left retest: lease application race and strict settling failure

After the Django dispatch fix, focused Left moved +0.471200 m in Gazebo and
+0.447410 m in odom over 3 sim / 22.777 wall seconds. WEB_MANUAL ownership
remained continuous, but one manual-zero sample occurred (no selected-zero
sample). Gaps in ms: client starts 187.173 / completions 167.659, Django
358.483, bridge 392.220, manual 311.238, selected 311.638. Fresh ingress can
arrive within the unchanged 400-ms lease while the separate command timer
has not applied it before the old deadline expires. The existing manual timer
now consumes the latest fresh mailbox packet through the same application
validator before testing expiry. Pending STOP/mode/E-STOP/disconnect barriers
always prevent that consumption; generation and received-time checks remain.
This correction does not extend the lease or introduce another command path.

Left STOP did not meet the strict wheel drift gate within the unchanged
180-wall-second watchdog (24.830 sim seconds). Wheel-position-derived drift
was 0.006465 / 0.006326 rad/s, above 0.005; chassis drift was 0.459 mm/s.
Selected/drive targets were zero. This is a real acceptance failure, not
permission to relax the threshold. Left is FAIL until a fresh corrected
runtime retest proves both continuity and settling. Right/Rotate Left have
not yet been rerun in this focused attempt. Stage B remains FAIL; later
runtime phases remain gated.

Backend validation after malformed-message dispatch hardening: 36 targeted
tests PASS, manage.py check PASS, migrations check PASS (no changes).
Current boot storage/OOM/panic gate remains clear; managed stack was stopped
cleanly to rebuild/reload the manual-timer correction. Physics is unchanged.
Source validation: 49 targeted transport/teleop/settling/odometry/kinematics
tests PASS, Python compile PASS, swerve_bridge colcon build PASS (only the
existing setuptools deprecation warning). The first new test attempt failed
because its synthetic bridge fixture omitted a production state field; that
fixture was corrected before the passing rerun.

Fresh production reload of commit `0eeb2a7` reached navigation READY on managed
domain 0 (backend 8001, frontend 5173). The focused Left retest now PASS:
Gazebo lateral +0.528950 m, odom +0.494847 m; 3.016 sim / 22.238 wall seconds,
RTF 0.135622. WEB_MANUAL continuous, manual/selected hold-zero counts 0/0.
STOP settled in 1.544 sim / 11.806 wall seconds with wheel drift
0.002352/0.002776 rad/s under unchanged 0.005. This supersedes the preceding
Left failure for current corrected source. Physics remains unchanged.
Gaps (ms): client starts 202.409 / completions 195.849, Django 292.599,
bridge 292.024, manual 220.333, selected 240.014. Full raw controller/joint/
pose/sequence evidence is in ignored `.runtime/resume-pending-ingress-fixed.json`.
Right retest also PASS: Gazebo lateral -0.733821 m, odom -0.594841 m;
3.012 sim / 23.672 wall seconds, RTF 0.127242. Continuous WEB_MANUAL,
manual/selected hold-zero counts 0/0. STOP 2.692 sim / 19.650 wall seconds,
wheel drift 0.000552/0.000719 rad/s. Gaps (ms): client 197.531, Django
330.433, bridge 370.803, manual 206.271, selected 196.906. Opposite-direction
magnitudes are not required to be equal; both independent pose trends agree.

Rotate Left retest: command continuity PASS (manual/selected hold zeros 0/0),
Gazebo yaw +1.150056 rad / odom +1.186841 rad; hold 3.003 sim / 22.767 wall
seconds. Gaps ms: client 307.032, Django 316.448, bridge 348.073,
manual 276.712, selected 277.705. STOP **FAIL** at the unchanged 180.014-wall
watchdog / 27.711 sim seconds: wheel drift 0.015302/0.001005 rad/s and yaw
drift 0.001458 rad/s exceed the existing 0.005/0.001 gates. Post-STOP trace
contains 8630 selected samples all exactly zero, 990 drive targets all zero,
990 steering targets fixed at pi/2, with measured steering stable. First
remaining divergence is actual Gazebo wheel/chassis settling, not command I/O.
Front wheel net drift decreased only from -0.019396 to -0.015448 rad/s over
the observed STOP period; raw velocities are not used as a settling override.
Stage B remains FAIL; all safety/navigation phases are still gated.

Read-only `gz topic -w warehouse_generated -r physics_info` confirms the actual
canonical generated world uses ODE quick / 50 iterations, dt=0.001,
sor=1.3, cfm=0, erp=0.2. Its SDF contains no explicit physics element; the
checkout's fallback warehouse has 120 iterations but is not the running world.
This is a solver-convergence hypothesis, not a proven root cause. Gazebo's
[official physics documentation](https://get.gazebosim.org/tutorials?cat=physics&tut=physics_params)
describes quickstep's iteration-dependent accuracy; increasing iterations is
not guaranteed to fix a particular contact system. Canonical artifacts and
production physics remain unmodified pending an isolated reversible diagnosis.

The zero-command diagnostic with quick/120 also FAILed the unchanged watchdog:
180.014 wall / 22.263 sim seconds; wheel drift 0.011809/0.006786 rad/s.
Yaw drift improved to 0.000155 rad/s, but wheel stability did not pass.
The transport query verified restoration to quick/50 with all other reported
physics properties unchanged. Increasing iterations alone is **not** a proven
fix and is not being integrated. An initial diagnostic setup timed out on the
5-second bridge parameter query before any physics write; the next attempt
obtained both services without extending their deadline.

The remaining executor audit found two synchronous action-discovery waits
(GoToTag/NavigateToPose, configured 2 seconds). They now use the existing
ActionClient.server_is_ready check and fail closed immediately when offline;
no safety callback waits for Nav discovery. 27 focused transport/teleop tests
PASS, including unavailable-action-server regression for both goal kinds;
compile and swerve_bridge build PASS. This source fix is loaded after the
subsequent managed reload; live navigation remains UNVERIFIED and Stage B
remains FAIL.

Additional zero-command physics diagnoses (all restored to verified quick/50,
sor=1.3, cfm=0 afterward; no canonical artifacts edited): direct ODE world
solver met drift gate in 15.238 wall / 1.798 sim seconds but produced thousands
of LCP errors. World with cfm=1e-9 still produced 1395 new LCP errors despite
meeting drift gate. Both are rejected as production fixes. Quick/120/sor=1
met drift gate only after 169.019 wall / 16.001 sim seconds (wheel
0.004982/0.001840 rad/s, no new LCP errors); no fresh motion/STOP was proven
on that profile, and it is not integrated. None is Stage B acceptance evidence.

The installed gazebo_ros2_control package is 0.4.10. Its matching
[upstream source](https://raw.githubusercontent.com/ros-controls/gazebo_ros2_control/0.4.10/gazebo_ros2_control/src/gazebo_system.cpp)
uses instantaneous SetVelocity for unconfigured velocity interfaces, versus
torque feedback for its existing velocity PID mode. Its write runs each physics
step. Source-backed inference: contact integration after a velocity assignment
can leave wheel drift without a holding motor; this is not yet a proven fix.
A narrow drive-servo candidate supplies vel_kp=1, ki=kd=0, zero integral
limits on both existing velocity command interfaces. No steering/controller
topology, mass/inertia/friction/damping, map or world physics settings changed.
Its source checks passed but the focused production trial failed, as recorded
below; the candidate has been removed. Any retained change affecting drive
actuation requires fresh six-direction regression before Stage B.

Frontend current-source targeted Control route/workflow tests: 19 PASS.
TypeScript/Vite build subsequently PASSed. Current kernel storage/OOM/panic gate
remains clear. Stack was stopped cleanly for the candidate actuation reload.
Candidate source checks now PASS: 52 targeted Python tests, swerve_bringup
colcon build, git diff --check. Current frontend tsc --noEmit and Vite build
PASS (existing large-chunk warning; no frontend source change).

The PID candidate loaded both gains in actual Gazebo and startup remained READY
with original velocity resource claims and unchanged quick/50 physics. Rotate
Left moved +0.978718 rad Gazebo / +0.908535 rad odom with no manual/selected
hold zeros. STOP FAILed after 180.027 wall / 27.108 sim seconds: wheel drift
0.017756/0.001374 rad/s. Candidate gain additions and their candidate-only unit
fixture were removed; no PID tuning is being retained. Raw wheel velocity is
the previously diagnosed numerical feedback and should not drive blind gain
tuning. The focused trace remains in ignored `.runtime/resume-drive-servo-rotate-left.json`.

Next narrow candidate: an ODE effort-limited joint motor constraint on the
original velocity interface storage. The adapter delegates init/state/steering/
read/write to the ORIGINAL installed GazeboSystem via pluginlib, retaining a
read-only view of original command storage; no second writer, ROS topic, manual
path or feedback substitution. Only actively claimed drive velocity interfaces
receive SetParam(vel/fmax); bounds come from the existing URDF effort/velocity
limits, and inactive interfaces restore their original passive motor force.
Nonfinite/out-of-limit targets and parameter-setting failures report ERROR
with zero requested, rather than relaxing limits. Upstream's default class has
an incomplete private implementation in its public header, so delegation keeps
its ABI instead of cloning or replacing its whole implementation.
Gazebo's [official velocity tutorial](https://get.gazebosim.org/tutorials?tut=set_velocity)
describes instantaneous velocity assignment versus force-driven ODE joint
motors. This is an actuation correction candidate, not an acceptance PASS.
Native motor-bound/failure tests and actual production motion were exercised
below; source world profiles, canonical artifacts, steering and settling gates
remain unchanged. Active motor force is an explicit actuation change, not a
claim that every effective physics parameter is unchanged.
Initial adapter build found a missing exported transitive PID header dependency
when attempting concrete inheritance; using the public interface and delegation
avoids that private/PID implementation dependency. No system packages installed.
Adapter source checks: 53 targeted Python tests PASS, swerve_bringup ROS build
PASS, native motor constraint gtest target PASS (signed target/limits/failure
cases), git diff --check PASS. The managed production reload reached READY and
loaded the adapter. Gazebo joint info confirms fmax=200 on both actively claimed
drive joints (existing URDF effort bound); inactive restoration uses the saved
original passive fmax=0.05. The quick/50 profile was unchanged.

Focused adapter Rotate Left: Gazebo yaw +1.400853 rad / odom +1.404662 rad,
hold 3.034 sim / 22.974 wall seconds, RTF 0.132064, continuous WEB_MANUAL,
manual/selected hold zeros 0/0. STOP **FAIL** at 180.019 wall / 24.427 sim
seconds: wheel drift 0.005072524/0.004685038 rad/s. The first wheel is still
above 0.005; rounding must not convert this result to PASS. Body XY/yaw drift
0.0003473/0.0002542 satisfy their unchanged gates. Gaps ms: client 193.656,
Django 268.916, bridge 359.305, manual 202.415, selected 205.754. No LCP errors
observed in this trial. Candidate remains uncommitted and is not Stage B PASS.

Subsequent zero-command diagnostic measured quick/50 net wheel drift
-0.007702/-0.010342 rad/s over 1.596 sim seconds. With only iterations changed
to 120, net drift was -0.002673/-0.002177 over 1.586 sim seconds and the strict
settling monitor passed after 5.964 wall / 0.811 sim seconds (window wheel
drift 0.003121/0.003944). This is diagnostic evidence, not a fresh motion/STOP
acceptance. The finally block restored quick/50 and verified all reported
physics properties. A fresh Rotate Left/STOP diagnostic on motor+120 was
blocked by its pre-motion settling gate: after 180.014 wall / 22.922 sim
seconds, wheel drift 0.000149/0.000355 was within limits but chassis yaw drift
0.002051 exceeded 0.001 rad/s. No motion was commanded. Its finally block
restored and verified quick/50. This disproves treating the earlier short
diagnostic window as a reliable fix. No production profile change has been
integrated. Motor+quick/300 also failed pre-motion settling after 180.019 wall /
14.780 sim seconds: wheel drift 0.004779/0.001461 was within limits but chassis
XY/yaw drift 0.003002/0.007688 was not. No motion was commanded; quick/50 was
restored and verified. Iteration-only tuning is rejected as a reliable fix.

At selected/drive zero and owner NONE on quick/50, temporarily deactivating
steering reached the unchanged settling gate after 3.946 wall / 0.800 sim
seconds: wheel 0.001797/0.004155, chassis XY/yaw 0.00002145/0.00004157. Steering
was reactivated successfully in finally. A subsequent 3.179-sim observation
with steering active again also stayed stable (wheel net -0.001402/-0.002919,
yaw net -0.00001689 rad/s). This does NOT isolate teleporting as the sole
cause; mode switching can change the contact state. It does localize further
investigation to solver/actuation state, not any stale motion command.

The matching native write implementation SetPositions steering and assigns
velocity each step. Gazebo's
[joint implementation](https://raw.githubusercontent.com/gazebosim/gazebo-classic/gazebo11/gazebo/physics/Joint.cc)
kinematically moves connected child links for SetPosition. Source-backed
inference: this can interfere with contact constraints. Next adapter candidate
keeps native init, resource storage, state exports and measured read, but uses
ODE motors for ALL four existing resources instead of native write. Steering
tracks the shortest continuous angle by a one-physics-step desired velocity,
bounded by the existing 6-rad/s velocity and 100-Nm effort limits. Drives retain
their original velocity targets and 30-rad/s / 200-Nm bounds. No gain tuning,
feedback substitution, extra command path, world-profile or physical URDF
coefficient change. Active actuator motor limits are explicitly changed;
inactive interfaces restore saved passive fmax. Invalid input/setting failure
zeros all active motor requests and returns hardware ERROR. This remains an
uncommitted candidate pending native/source tests and fresh six-direction
runtime regression; previous direction PASSes cannot establish this actuation
candidate's gate.

All-resource motor candidate source checks: native 4 cases PASS, 54 targeted
Python tests PASS, bringup ROS build PASS, shell syntax and git diff check PASS.
An initial pytest command named a nonexistent teleop test; another lacked
ros_env/generated GoToTag and had four import failures. The corrected command
used scripts/ros_env.sh and /usr/bin/python3 and completed all 54 tests. Managed
production reload reached full READY on domain 0 and logged all four motor
resources with their original URDF bounds; read-only query confirmed quick/50.

Its focused Rotate Left moved +1.378039 rad Gazebo / +1.432018 rad odom,
hold 3.005 sim / 18.188 wall seconds, RTF 0.165220, continuous WEB_MANUAL with
manual/selected zeros 0/0. STOP **FAIL** at 180.000 wall / 30.518 sim seconds:
wheel drift 0.004786/0.008205 exceeds 0.005 at the rear wheel; chassis XY/yaw
0.000461/0.000392 is within bounds. Client/manual/selected gaps ms
190.132/139.273/135.333. Django/bridge ingress gaps are UNVERIFIED for this
focused run: its managed reload omitted the opt-in WARETWIN_MANUAL_TIMING
environment, so zero correlated rows must not be reported as zero latency.
Any six-direction gate run will restore instrumentation at production startup.
No candidate PASS or Stage B PASS is claimed. A reversible motor-all-resources
+ quick/120 diagnostic is now pending, distinct from the rejected iteration-
only and drive-only experiments.

All-resource motor+120 focused Rotate Left diagnostic met the unchanged drift
gate only after 161.228 wall / 21.653 sim seconds (wheel 0.001203/0.004990,
chassis XY/yaw 0.000172/0.000715). Gazebo/odom yaw +1.654319/+1.724744,
continuous WEB_MANUAL, zero hold interruptions. This is not a reliable Stage B
or production-profile proof; quick/50 was restored and verified.

### Rolling collision friction-frame defect (current investigation)

Read-only actual GetEntityState at zero command measured the configured local
X friction direction's world dot product with the floor normal: front
-0.923617, rear +0.409893. The wheel Y axle rolls this X vector into/out of the
floor normal. The installed Gazebo 11.10.2 matching
[contact assembly](https://raw.githubusercontent.com/gazebosim/gazebo-classic/gazebo11_11.10.2/gazebo/physics/ode/ODEPhysics.cc)
rotates the configured vector into world coordinates and passes it to ODE.
Matching [ODE contact code](https://raw.githubusercontent.com/gazebosim/gazebo-classic/gazebo11_11.10.2/deps/opende/src/joints/contact.cpp)
uses that vector directly in the tangential Jacobian (with a fallback only when
exactly parallel); it does not generally project it back onto the contact plane.
Inference backed by the measured frame defect: erroneous normal components in
friction constraints can drive contact/holding drift and ill-condition solving.
This is more specific than the rejected iteration/motor hypotheses; live
regression is still required before claiming root cause resolved.

The motor adapter candidate, its candidate-only tests/build declarations and
dependencies were removed. Only candidate-created installation/index symlinks
were moved into an ignored .runtime/rejected-motor-artifacts directory, so the
old loader export cannot pollute the original GazeboSystem loader. These
generated artifacts are recoverable; no overnight/user work was deleted.
Original native hardware, control interfaces and quick/50 profile are restored.
The new narrow candidate changes only two rolling-wheel fdir1 vectors from
1 0 0 to 0 0 0, selecting ODE's automatic contact-tangent basis for the existing
isotropic mu1=mu2=2 surfaces. No mass/inertia, geometry, friction coefficients,
kp/kd, damping, limits, solver profile or safety gate is retuned. Xacro-to-SDF
regression verifies the actual collision surface contract before runtime.

Current narrow-fix checks: 54 targeted Python tests PASS, including generated
Xacro-to-Gazebo SDF surface checks; Python compile PASS, shell syntax PASS,
swerve_bringup colcon build PASS, git diff --check PASS. CMake/package.xml are
back to the committed baseline; there is no retained motor adapter or added
Gazebo native build dependency. The current kernel storage/OOM/panic gate is
clear. Managed navigation reload now restores WARETWIN_MANUAL_TIMING=1 for
the required per-direction Django/bridge ingress measurements.

The first friction-frame reload failed before controller-manager creation:
gazebo_ros2_control's ROS CLI/YAML robot_description parser rejected colon-space
in the newly added XML comment. A native rclpy parameter-override regression
reproduced the same RCLInvalidROSArgsError before correction. The comment now
uses a semicolon; parsing succeeds. YAML folds presentation whitespace, so the
test compares canonical XML semantics instead of raw byte identity. No spawner,
Nav lifecycle or bond implementation was reopened. Failed reload was stopped,
current kernel gate rechecked clean, and the corrected managed reload reached
full navigation READY on domain 0. All three controllers, original resources,
arbiter, odom/LiDAR/TF, all six Nav2 lifecycle nodes and R01 fresh heartbeat
were confirmed. 55 current source tests PASS including the native parameter-
override regression. Fix checkpoint d52cb53 was pushed to origin/web-simulation.
Fresh six-direction regression is now running because the contact-basis fix
affects all directions; closed Phase 1 acceptance is not being rerun.

### Fresh native-hardware/contact-tangent directions

Ignored raw evidence: .runtime/resume-tangent-directions.json. All holds used
real authenticated Django /ws, original velocity/steering interfaces and
quick/50. Every observed hold had continuous WEB_MANUAL ownership, correct
manual/selected Twist and zero manual/selected interruption samples.

| Direction | Requested (vx,vy,wz) | Gazebo / odom signed movement | Hold wall / sim | RTF | STOP wall / sim | Result |
| --- | --- | --- | --- | --- | --- | --- |
| Forward | .25,0,0 | +.672699 / +.635541 m | 20.259 / 3.000 | .148082 | 10.048 / 1.584 | PASS |
| Backward | -.25,0,0 | -.642050 / -.606202 m | 20.005 / 3.013 | .150611 | 10.701 / 1.445 | PASS |
| Left | 0,.25,0 | +.501233 / +.472844 m | 18.582 / 3.014 | .162199 | 7.999 / 1.413 | PASS |
| Right | 0,-.25,0 | -.632898 / -.597070 m | 17.977 / 3.026 | .168326 | 8.343 / 1.459 | PASS |
| Rotate Left | 0,0,.6 | +1.352417 / +1.152341 rad | 19.366 / 3.027 | .156304 | 180.026 / 28.714 | FAIL STOP |
| Rotate Right | 0,0,-.6 | not run after failed prerequisite | - | - | - | UNVERIFIED |

Rotate Left STOP wheel drift 0.016432/0.018042 exceeded 0.005 rad/s and chassis
XY drift 0.001178 exceeded 0.001 m/s; yaw drift 0.000233 was within limit.
Thus correcting the invalid contact frame is not sufficient to close Stage B.
Selected/drive zero did not produce adequate physical holding. No Nav/Mapping
was run. The gate still FAILs and no relaxed threshold/lease is used.

| Direction | Client gap ms | Django gap ms | Bridge gap ms | Manual gap ms | Selected gap ms |
| --- | --- | --- | --- | --- | --- |
| Forward | 194.764 | 571.101 | 649.796 | 177.063 | 169.805 |
| Backward | 225.209 | 286.051 | 336.255 | 179.975 | 158.161 |
| Left | 216.111 | 333.360 | 363.459 | 205.598 | 212.025 |
| Right | 161.976 | 231.933 | 242.556 | 162.310 | 150.233 |
| Rotate Left | 174.155 | 446.734 | 484.059 | 163.687 | 162.989 |

The raw ingress calculation includes all frames SENT during a hold, even when
received afterward. Forward's largest gap is the last in-flight nonzero frame:
seq163 T4=9809.231096, seq164 T0=9809.378108/T1=9809.794837/T4=9809.880892,
hold ended at 9809.463920. STOP seq165 T0=9809.464110/T1=9809.892799/
T4=9810.023548. These endpoint delays are retained, not clipped from reported
maxima. Actual within-hold topic continuity passed; subsequent safety acceptance
must independently measure STOP/timeout behavior.

Next candidate restores ONLY the earlier drive motor adapter, now paired with
the corrected contact-tangent basis. Native steering/write/read, original
resource storage, URDF bounds and world solver remain unchanged. Actively
claimed drive interfaces get effort-bounded ODE vel/fmax constraints throughout
contact solving; this changes active actuation holding force explicitly (not
passive coefficients or safety thresholds). All active drives get zero on any
invalid target or motor-setting failure; inactive interfaces restore saved
passive fmax. Per-step targets are preallocated. No steering motor servo or
solver tuning is reintroduced. Runtime regression is pending; no candidate
Stage B PASS is claimed.

### Current drive constraint candidate regression (2026-10-01)

The drive-only ODE motor constraint was reintroduced with the corrected
isotropic contact tangent basis. Production navigation reached READY on
managed ROS domain 0, with all controllers active, Nav2 active, and R01 bridge
heartbeat fresh. ODE logged the existing drive URDF bounds (effort 200,
velocity 30); quick/50 remained selected. A focused Rotate Left run and all six
directions then passed through authenticated Django `/ws`, R01, the manual
bridge, arbiter, selected command, controllers, and Gazebo. Every hold had
continuous WEB_MANUAL ownership, no zero samples on manual or selected command,
correct signed Gazebo displacement, and matching signed odom trend. All STOP
settling checks passed the unchanged position-derived limits.

| Direction | Gazebo signed displacement | Odom signed displacement | Hold wall / sim s | RTF | STOP wall / sim s | Result |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Forward | +0.756689 m | +0.715041 m | 22.860 / 3.032 | .132635 | 9.610 / 1.236 | PASS |
| Backward | -0.757228 m | -0.713933 m | 22.538 / 3.033 | .134571 | 8.000 / 1.267 | PASS |
| Left | +0.621801 m | +0.588873 m | 21.676 / 3.017 | .139183 | 9.578 / 1.231 | PASS |
| Right | -0.751240 m | -0.708949 m | 22.419 / 3.036 | .135423 | 7.957 / 1.222 | PASS |
| Rotate Left | +2.066364 rad | +1.735106 rad | 20.351 / 3.021 | .148443 | 7.714 / 1.106 | PASS |
| Rotate Right | -2.060904 rad | -1.699095 rad | 21.695 / 3.013 | .138881 | 7.681 / 1.161 | PASS |

Position-derived wheel rates at STOP were all at most 0.000805 rad/s across the
focused Rotate Left and six-direction run (strict limit 0.005); chassis
position/yaw drift also passed the existing 0.001 m/s
and rad/s limits. Numeric joint velocity feedback remains classified
NUMERICAL_FEEDBACK and did not replace position-based acceptance. Per-direction
maximum client-send, Django receive, bridge receive, manual topic and selected
topic gaps are preserved in ignored raw trace
`.runtime/resume-tangent-drive-motor-six.json`; ingress maxima include delayed
frames correlated to sends during each hold. Largest observed values across
the six holds were 249.046, 304.288, 290.675, 195.511 and 197.034 ms,
respectively. Command continuity passed, including for ingress timing samples
received after a hold boundary.

Safety acceptance on the same source then passed explicit STOP, lease timeout,
WebSocket disconnect, MANUAL to AUTONOMOUS, E-STOP, clear-without-resume and
real browser pointer release. Selected zero latencies from each trigger were
171, 571, 98, 307, 427, 30 and 192 ms, respectively. E-STOP selected output
became zero 14.5 ms after ROS bridge receipt (the full Web API-to-zero time was
427 ms); arbiter owner was ESTOP until clear, and no old motion resumed.
PointerUp emitted the actual wire STOP and produced no browser errors. The
safety runner's current-source Stage B safety gate is PASS. Along with the six
fresh direction and per-motion STOP results above, STAGE_B_GATE=PASS. This is
Gazebo/ROS/Web validation only, not physical robot validation.

### Current Web Nav resume attempt (2026-10-01)

The production stack remained READY on managed domain 0 with a fresh R01
heartbeat. Map state was sampled at READY, before preview, after preview,
immediately before Send Goal, and at backend acceptance. Every sample was
canonical map `CANONICAL`, revision 21; ROS, Gazebo, Nav2, and tag-map
revisions were all 21, TF health was true, and sync was `SYNCED`. No map
identity/revision transition or TF-health error occurred. Thus the prior
`MAP_OUT_OF_SYNC` behavior did not recur in this run.

The real Control Detail browser produced a valid 30-point, 0.768130 m path on
revision 21 and sent its bound preview request through Django/R01. A negative
missing-preview request was rejected as `PATH_PREVIEW_REQUIRED`; selected
nonzero samples and NAV2 ownership were both zero, with only 0.000010 m
Gazebo displacement. PATH_PREVIEW and PATH_PREVIEW_ENFORCEMENT therefore
PASS.

Nav2 accepted the browser goal and returned SUCCEEDED. NAV2 command, selected
command, NAV2 ownership, drive output, Gazebo movement, and odom movement were
all observed. Goal yaw error was 0.00918 rad. However the latched Gazebo XY
error was 0.10297 m, over the unchanged 0.05 m acceptance limit. A separate
stationary post-settle observation measured map pose `(14.66157, 7.37550)` and
Gazebo pose `(14.63450, 7.51117)` for goal `(14.63418, 7.39909)`: the map-frame
pose was about 0.036 m from the goal while Gazebo's world pose remained about
0.112 m away. The same map-vs-Gazebo Y offset was already about 0.103 m before
Send Goal (map Y 6.81844, Gazebo Y 6.92100). Wheel-position settling passed;
this is not residual motion.

Source audit identifies `ekf_v30e` as the sole map->odom TF owner, using
`/tag_navigation/global_measurement` for absolute V30E corrections and wheel
velocity/IMU between them. No competing TF publisher or map revision mismatch
was found. The exact localization cause of the observed map/Gazebo divergence
is not yet proven; do not infer canonical map validity from map-sync metadata
alone. No thresholds were changed. `SEND_GOAL` and `COMMAND_ARBITER_NAV` are
observed PASS, but `WEB_NAV_GOAL_R01=FAIL` for final Gazebo XY tolerance, so
the Phase 3 gate is FAIL and Mapping/Map Save/Load/Init Pose/VDA5050 remain
NOT RUN. Raw browser screenshots and traces are retained under ignored
`.runtime/web-nav-resume-*` and `.runtime/resume-web-navigation.json`.

The ROS heartbeat callback now enqueues `_SOCKET_PING`; `ws.ping()` executes
on the existing outbound WebSocket worker. Regression coverage is in
`tests/test_manual_transport.py` and is included in the source-test results
recorded at the end of this resume run.

Current resume-run source checks: `tests/test_manual_transport.py` 7/7 PASS
after sourcing `scripts/ros_env.sh`; navigation acceptance Python
`py_compile` PASS; Playwright helper `node --check` PASS; `git diff --check`
PASS. A first test invocation without the ROS workspace environment failed
imports (`swerve_bringup` unavailable); it was rerun in the supported ROS
environment and passed. No localization/ROS source was modified in this
resume run, so no ROS rebuild was needed.

### Robot Control visualization performance checkpoint (2026-10-01)

Nav Goal, Mapping, Localization and VDA5050 work is paused for this requested
visualization fix. The existing production Gazebo session is retained; current
boot kernel checks contain no new storage/OOM/panic faults.

Instrumentation covers browser click/send, Django receive, bridge receive/apply,
frame preparation, network send, Django/browser receive, and useful canvas
render. The initial real browser baseline (two measured switches per direction,
after warmup) observed max useful content latency: GLOBAL->2D 1500 ms,
2D->3D 8780 ms, 3D->2D 4009 ms, 2D->GLOBAL 5467 ms. The default simulation-clock
detail callback ran at about 1.4-1.7 wall seconds between samples. Occupancy
compression reached 1582 ms; point-cloud preparation reached 460 ms; one
browser decode/raster build took 1328 ms. Raw evidence is ignored
`.runtime/view-performance-before.json`.

Historical intermediate traces showed cached render mostly 28-92 ms, but ACKs
still took 4-7 seconds. A 20-second Django CPU profile identified pure-Python
Autobahn UTF-8 validation (6.14 s) and WebSocket masking (5.78 s), not sensor
rendering, as the next transport bottleneck. Installing native `wsaccel` into
the worktree venv initially did not affect the live server: its copied activate
script selected the original checkout's venv. `backend/run.sh` now explicitly
uses this checkout's interpreter; setup/preflight verify the dependency. Native
UTF-8 validation remains enabled and has a rejection regression test.

Browser CPU profiling subsequently identified React development reconciliation
and property validation as substantial costs. Production startup now builds
the production frontend with the selected backend port before launching runtime
services, then serves those assets; `WARETWIN_FRONTEND_MODE=development` retains
the editing workflow. The production assets were exercised on the existing
frontend port. The complete stack was not restarted or Phase 1 reopened.

Current source implementation:

- Dedicated monotonic latest-only visualization worker: 5 wall-Hz 2D and the
  existing 3 wall-Hz 3D limit, immediate coalesced wake on view change, no cloud
  FIFO. ROS sensor callbacks store latest references. Cloud transforms/voxel
  sampling use bounded native arrays; the 4000-point output cap is unchanged.
- Lightweight correlated REQUESTED/APPLIED/FRESH acknowledgement with robot,
  request ID, bridge epoch and view epoch. Stale epochs cannot become fresh.
- Heartbeat socket ping, graph diagnostics and telemetry TF/map-status waits
  are off the control executor. Telemetry retains its previous cadence;
  safety/control timers and leases are unchanged.
- Bounded Django latest-only display outboxes and opt-in browser frame receipts
  prevent accumulating disposable frames in a slow browser's transport queue.
  Safety/control acknowledgements retain their separate delivery path.
- Last-good frames retained per robot, with a waiting indicator. Visualization
  caches confer no navigation authorization; preview/map checks are unchanged.
- Occupancy raster LRU limited to four entries / 8M cells; GLOBAL changes do not
  reset the bridge map signature. Reconnect can resend to a genuinely uncached
  backend, and late browsers receive Django's retained snapshot.
- Retained 3D Canvas uses demand rendering when fresh/visible and never renders
  while hidden. Its geometry buffer is reused. No stale-epoch GPU redraw before
  the view ACK. Static metric/panel reconciliation is memoized.
- The acceptance sender reuses `ManualRefreshWorker` in an isolated process so
  ROS probing cannot contend for its GIL. Non-rendering probe/control sessions
  use bounded display delivery; only the real browser consumes the full visual
  stream. All command-topic samples remain recorded, with no dead-man change.

Current source checks: frontend targeted tests **21/21 PASS**, frontend
TypeScript/Vite production build **PASS** (existing large-bundle warning);
backend targeted tests **21/21 PASS**, `manage.py check` and
`makemigrations --check --dry-run` **PASS**; bridge/transport/worker/process
tests **27/27 PASS**, shell/setup regression tests **10/10 PASS**; ROS bridge
`colcon build --base-paths swerve_bridge --packages-select swerve_bridge
--symlink-install` **PASS**, one package actually built. Python compile,
JavaScript syntax, shell syntax and `git diff --check` **PASS**.

Current real VM / Gazebo / Django / production-browser evidence: 10 measured
switches for each pair (40 total), after warmup, during authenticated Web
manual rotation. Values below are wall milliseconds, **median / max**.

| Transition | Cached useful canvas | Applied ACK at browser | Fresh frame at browser |
| --- | ---: | ---: | ---: |
| GLOBAL -> 2D | 11.65 / 20.70 | 227.65 / 399.00 | 322.20 / 549.70 |
| 2D -> 3D | 35.50 / 97.10 | 194.30 / 452.40 | 481.75 / 929.40 |
| 3D -> 2D | 12.55 / 34.30 | 303.25 / 885.00 | 474.55 / 1094.50 |
| 2D -> GLOBAL | 12.80 / 61.70 | 246.05 / 513.60 | 246.05 / 513.60 |

GLOBAL fresh means the retained map is useful and its requested view has been
acknowledged; no new map is required. Fresh LiDAR is the matching epoch/request
frame received, not a promise of a newer sensor acquisition or completed GPU
paint. T0-T8 instrumentation separates those events. The 3D render hook marks
frame submission, not GPU completion. Cached render marks retained canvas
visibility on requestAnimationFrame, not a hardware presentation timestamp.
Evidence: ignored `.runtime/view-performance-after.json` and
`.runtime/view-control-safety.json`; source helpers are reproducible.

All 40 cached switches meet 100 ms. No map messages, raster rebuilds or Canvas
3D remounts occurred during measured switches. Map compression count remained
constant at 2 (startup/reconnect only). Maximum cloud output was 2994 points
(cap 4000); maximum Django display pending count was 2. GC-normalized browser
heap rose 924920 bytes including bounded tracing over the run; this short test
does not prove absence of a long-duration leak. Measured browser frame rates
were 3.96 Hz 2D and 1.47 Hz 3D; source clouds remain simulation/source limited.
Measured motion RTF was **0.12337** (8.205 simulation seconds / 66.507 wall
seconds), not the noisier instantaneous diagnostic ratio.

Current safety actions while rapidly switching: explicit STOP and strict
mechanical settle **PASS** (7.226 wall / 1.107 simulation seconds), applied
AUTONOMOUS handoff and settle **PASS** (selected-zero latency 60.40 ms), E-STOP
and settle **PASS** (Web request-to-zero 9.64 ms, ROS receipt-to-zero 12.85 ms;
different subscriptions have independent receipt timestamps), E-STOP clear
without old motion resuming **PASS**. During hold there were 1802 observed
manual samples, zero zero-command samples, and all 328 arbiter samples reported
WEB_MANUAL. Nevertheless **MAX_CMD_VEL_MANUAL_GAP_MS=226.85**, above the retained
200 ms continuity gate: overall control timing acceptance remains **FAIL**.
Earlier intermediate runs had STOP settling watchdog failures; those are
historical, not the current outcome, and no settling limits were relaxed.

**VIEW_SWITCH_PERFORMANCE=FAIL**: cached switching is now immediate, but the
250 ms ACK, 500 ms fresh-2D and 200 ms manual-gap gates do not all pass under
concurrent probing/load. Current correlated ACK traces locate the largest
remaining tail between Django browser-send and browser receipt (up to 772 ms
for 2D); bridge receive-to-apply reached 213 ms. Independent client refresh
also still had scheduling outliers; a VM/host cause is not proven. The earlier
multi-second sim-clock/cache/Autobahn/React-development issues are corrected,
but these remaining tails must not be hidden with larger leases, weaker gates,
synthetic sensor frames or a success-only report. Nav Goal, Mapping,
Localization and VDA5050 remain paused. This validates Gazebo/ROS/Web only,
not a physical robot.

Handoff health: fresh direct ROS subscription confirms selected Twist zero,
owner NONE and applied AUTONOMOUS. A typed echo succeeded after an untyped CLI
graph query failed to discover the topic; that was not a missing publisher.
The bridge-only reload is registered in ignored `.runtime/ros_bridge.pid`;
scoped ownership checks and `stop_stack.sh` cover this optional separate group
so it cannot be left behind on the next stack stop. Normal ROS launch ownership
is unchanged. No full startup/stop acceptance was repeated for this utility
change; targeted ownership/shell regression tests passed.

## 2026-10-01: accumulated LiDAR SLAM mapping and local map save

The Mapping workflow now runs in the production `mapping` runtime with SLAM
Toolbox as the only `map -> odom` owner. Mapping mode no longer starts the
simulation's V30E localization owner or a canonical-map server. The bridge
publishes content-versioned `/map` snapshots from a bounded worker, transforms
current `/scan` into `map`, and reports TF-derived robot pose and a bounded
trajectory. React renders the accumulated occupancy raster, current scan,
robot, trajectory, and optional one-metre grid on a shared world transform;
scan points are not accumulated in the browser.

### Live mapping evidence

From the production Mapping UI and ROS observer, `/scan` was live in
`lidar_link`, `/odom` was live as `odom -> base_footprint`, and `map <- lidar`
TF was `OK`. The Mapping status panel reported SLAM Toolbox active, accumulated
`/map` live, and `MAP -> ODOM OWNER = SLAM_TOOLBOX`. Browser status during the
real teleop mapping run measured scan at 1.18-1.23 Hz and odometry at 4.66 Hz;
these are the VMware simulation's observed rates, not source-rate targets.

The same SLAM session produced these observed map snapshots at 0.05 m/cell:

| Snapshot | Dimensions | Known | Free | Occupied |
| --- | ---: | ---: | ---: | ---: |
| Initial ROS observation | 426 x 598 | 20,302 | 19,882 | 420 |
| Next ROS observation | 431 x 598 | 45,231 | 44,555 | 676 |
| Browser before movement, v3 | 431 x 598 | 45,231 | — | — |
| Browser after Web forward, v4 | 440 x 598 | 60,875 | 59,945 | 930 |
| Final paused UI, v5 | 446 x 598 | 71,741 | 70,636 | 1,105 |

On the first ROS comparison, 7,976 of 8,093 previously known coarse-grid cells
remained represented (98.6%). During the authenticated browser run, 73 Web
manual forward refreshes moved the TF pose from about `(0.60, 0.20)` m to
`(0.82, 0.32)` m; known cells increased from 45,231 to 60,875 and trajectory
samples increased from 1 to 7, then 9 after STOP. The same session delivered
45 current scan frames during the hold, each with 624 points in `map`, sourced
from `lidar_link`; the transformed sensor pose changed with the robot. The UI
showed the accumulated map and scan concurrently with ROBOT, SCAN, and
TRAJECTORY enabled. This proves map growth and persistence over the exercised
route, but not full-warehouse coverage or loop-closure quality.

`MAPPING_START=PASS`, `MAPPING_LIVE_MAP=PASS`, `MAP_ACCUMULATION=PASS`,
`WEB_MAPPING_DISPLAY=PASS`, and `MAPPING_STOP=PASS` for that production run.
STOP was acknowledged by SLAM Toolbox and the UI reported PAUSED before Save.

### Save and canonical-map evidence

The real UI Save action created navigation and resumable SLAM products under
the robot-scoped local map store. For `slam_accumulated_20261001_01`, the YAML
was 146 bytes, PGM 263,135 bytes, pose graph 18,123,813 bytes, and SLAM data
111,527 bytes. The PGM header was `440 x 598`; the YAML resolved its image and
contained resolution `0.05`, origin `[-5.46, -14.9, 0]`, negate `0`, occupied
threshold `0.65`, and free threshold `0.25`. The authenticated map API returned
an opaque map ID and metadata including `SAVED_LOCAL_MAP`, resolution, origin,
dimensions, cell counts, source session/version, and
`slam_session_state=AVAILABLE`; it did not expose filesystem paths. The final
UI listed three robot-local saves, each with an available SLAM Toolbox
session. No resume operation or Map Load acceptance was run in this phase.

The canonical revision-21 YAML and PGM hashes were recorded before Save and
matched after it:

```text
warehouse_1.yaml  47626f718be173195e33e26e1676ab5bc797526f7c1b6254d20065a506fc0230
warehouse_1.pgm   cc1050c62b6cf55edb4180329be803ca103ec9fcdb4845960a544805c6b7c897
```

`MAP_SAVE=PASS`, `MAP_FILES_VALID=PASS`, `MAP_REGISTRY=PASS`, and
`CANONICAL_MAP_UNCHANGED=PASS` for these saved local maps.

### Manual refresh worker and final runtime gate

An earlier real Mapping UI hold had a 750 ms main-thread refresh gap. The
unchanged 400 ms dead-man lease correctly released manual ownership for about
191 ms before a later refresh reacquired it. The refresh loop has since moved
to a dedicated control-only WebSocket worker; the lease was not lengthened.
Targeted worker/channel regressions passed: frontend workflow 19/19 and Django
dispatch/local-control/bridge tests 48/48. The ROS bridge/readiness mapping
tests had passed 30/30, and the relevant ROS packages had built successfully.
`manage.py check`, migration check, and Python compile checks also passed.

The worker-backed production retest and final frontend bundle did not complete.
TypeScript had completed, but Vite was still building when the session ended;
there is no successful bundle exit status. `TELEOP_DURING_MAPPING` therefore
remains `UNVERIFIED` after the worker change, despite earlier scan/map UI and
Save acceptance.

The previous boot ended without a clean shutdown record (`last -x` reports
`crash`); its cause cannot be isolated, and its journal already contains
virtual-disk I/O failures. During the current boot, at 2026-10-01 19:54 local,
the kernel reported repeated `/dev/sda` `DID_TIME_OUT` and `I/O error` events,
blocked `jbd2`/`kswapd` tasks, and journald watchdog failures. The project
stack was stopped successfully with `./scripts/stop_stack.sh`; no project
Gazebo, ROS launch, Django, or frontend process remains. No more build or live
acceptance work was started after the storage fault.

`VM_STORAGE_GATE=FAIL`; `TELEOP_DURING_MAPPING=UNVERIFIED`;
`FRONTEND_BUNDLE=UNVERIFIED`; `COMMIT_PUSH=UNVERIFIED` (the mapping changes
remain uncommitted in `web-simulation`). `MAPPING_GATE=FAIL` because required
worker-backed teleop and production-bundle checks remain unverified; the map
and save portions passed. Do not start Gazebo again until current kernel
storage health is clear. Map Load, SLAM resume, Nav Goal, Localization, and
VDA5050 remain outside this mapping checkpoint.
