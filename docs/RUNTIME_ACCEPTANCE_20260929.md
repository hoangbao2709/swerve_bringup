# Runtime Acceptance Report

This report records the September 29 recovery session. The most recent full base-runtime observation is from the earlier managed navigation run. No new Gazebo/Nav2 run was started at the current checkpoint because the current boot journal contains virtual-disk I/O errors.

## Environment

- Date/time checked: 2026-09-29 16:48 +07:00 (Asia/Ho_Chi_Minh)
- Git branch: `web-simulation`
- Git commit before this report change: `26d067a` (`fix: align Gazebo readiness and trace startup stages`; already on `origin/web-simulation`)
- ROS distro: Humble
- Current shell after `source scripts/ros_env.sh`: `ROS_DOMAIN_ID=12`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_LOCALHOST_ONLY=0`, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`
- Runtime `.runtime/stack.env`: absent at this checkpoint; no current runtime was launched. The earlier managed base run used `ROS_DOMAIN_ID=0`.
- Headless mode: enabled by default (`gzserver` only; no `gzclient` or `rviz2` observed in the earlier managed run)
- Map used in the earlier managed run: canonical warehouse map, revision 21
- Robot ID: R01

## Previous failure context

- The prior long SpawnEntity attempt took 986.765 seconds. Previous-boot kernel evidence showed SCSI `DID_TIME_OUT`, `/dev/sda` I/O errors, and blocked ext4 journal work. No OOM-killer or `gzserver` segfault evidence was found. The guest logs do not identify the underlying VMware host/VMDK cause.
- The later full production-profile insertion completed in 27.872 seconds. A minimal diagnostic configuration with LiDAR disabled inserted in 2.18 seconds; a full visual/control configuration with LiDAR disabled inserted in 1.34 seconds. These comparisons identify LiDAR as a substantial insertion/runtime cost, but production LiDAR settings were left unchanged.
- The diagnostic production LiDAR profile (720 x 16 x 10 Hz) reached about 1.43 GiB `gzserver` RSS and roughly 117-121% CPU. A later managed base run peaked at about 1,494,896 KiB `gzserver` RSS, approximately 5.1/7.7 GiB system memory used (about 2.2 GiB available), and 66 MiB of 2 GiB swap.
- At the current checkpoint, `free -h` showed 4.5 GiB available and swap at 66 MiB/2 GiB; `/` had 6.6 GiB free (89% used). Those figures do not rule out storage failure.

## Current storage health gate

- `dmesg -T` and its filtered form could not read the kernel ring buffer (`Operation not permitted`); `journalctl -k -b` was used instead.
- Current-boot journal evidence at 2026-09-29 15:59:35 includes repeated `hostbyte=DID_TIME_OUT` for `/dev/sda`, multiple `I/O error, dev sda` entries, and a systemd-journald watchdog failure/restart. Later current-boot workqueue warnings were recorded at 16:00:54, 16:25:41, and 16:47:47.
- No current OOM or swap-exhaustion evidence was observed. Disk capacity is not exhausted, but kernel-reported device I/O failures are present.
- Decision: no new stack startup or runtime acceptance was attempted. No project Gazebo, RViz, ROS launch, backend, frontend, or bridge process was running at inspection time.
- Ran `./scripts/stop_stack.sh`; it reported that ROS, frontend, and backend were not owned/running under this stack and that no unrelated processes were touched.

## Startup timeline

The entries below are from the earlier successful managed base-runtime run, not a new run at this checkpoint. They use the final readiness attempt's elapsed values where retained; earlier probes had already observed some stages before controller/service responses stalled.

| Stage | Elapsed from T0 | Observation |
|---|---:|---|
| T0 start_stack begins | 0.000 s | Observed |
| T1 gzserver process started | 62.940 s | Observed |
| T2 three advancing `/clock` samples | 669.101 s final probe; 105.320 s first probe | Advancing samples were observed in both probes |
| T3 `/spawn_entity` service ready | 669.149 s final probe; 105.340 s first probe | The request began at 76.138 s, so the service was available no later than T4; readiness polling observed it later |
| T4 SpawnEntity request begins | 76.138 s | ROS launch log timestamp |
| T5 SpawnEntity request returns | 104.010 s | ROS launch log timestamp; request duration 27.872 s |
| T6 robot entity confirmed | 669.226 s final probe; 105.427 s first probe | `/model_states`, entity `swerve_base` |
| T7 controller_manager available | 669.227 s | `/controller_manager/list_controllers` service ready in final probe |
| T8 joint_state_broadcaster ACTIVE | 669.794 s | Controller activation log |
| T9 steering_controller ACTIVE | 669.794 s | Controller activation log |
| T10 drive_controller ACTIVE | 669.794 s | Controller activation log |
| Joint states ready | 669.016 s | Valid required joints observed |
| T11 `/odom` ready | 671.317 s | Readiness log |
| Filtered odometry ready | 669.047 s | Valid `/odometry/filtered` sample observed |
| T12 raw LiDAR ready | 669.552 s | Valid `/lidar/points` sample observed |
| T13 filtered LiDAR ready | 669.151 s | Valid `/lidar/points_filtered` sample observed |
| T14 `/scan` ready | 669.121 s | Valid `/scan` sample observed |
| TF ready | UNVERIFIED elapsed | TF readiness passed, but this probe does not record a separate TF timeline event |
| T15 Nav2 lifecycle startup begins | 671.696 s | Readiness log |
| T16 `/navigate_to_pose` ready | 681.037 s | Readiness log |

The timeline's per-sample values are not strictly ordered: odometry and sensor topics were already live while the readiness probe was blocked waiting for controller service responses. The earlier startup eventually reached base-runtime readiness, but it coincided with the storage errors recorded above and took about 681 seconds overall. This does not establish reliable startup.

## Resource observations

- Peak `gzserver` RSS: 1,494,896 KiB (~1.43 GiB) in the earlier production run.
- Approximate peak system memory: ~5.1/7.7 GiB used; ~2.2 GiB available.
- Peak observed swap: 66 MiB/2 GiB.
- LiDAR diagnostic CPU: approximately 117-121% during insertion; no production sensor-rate change was made.
- Current kernel warnings: repeated `/dev/sda` SCSI timeouts and I/O errors at 15:59:35; journald watchdog restart at the same time; later workqueue delay warnings. Current `dmesg` access was denied, so journal records are the evidence source.

## Acceptance matrix

`PASS` entries below were observed in the earlier managed base-runtime run. Motion and Web tests were not reached. No new runtime checks were attempted after the current storage health gate failed.

The retained readiness log printed `GAZEBO_WORLD_READY=PASS` twice per probe. The readiness reporter was corrected to include the model count in its single `_stage` success report; the code change is covered by a focused regression test.

| Item | Result | Reason/evidence |
|---|---|---|
| CLOCK_READY | PASS | Earlier managed readiness observed multiple advancing `/clock` samples. |
| GAZEBO_FACTORY_READY | PASS | Earlier managed readiness observed `/spawn_entity` and successfully inserted the entity. |
| ROBOT_SPAWNED | PASS | Earlier `/model_states` sample confirmed `swerve_base`. |
| CONTROLLERS_READY | PASS | Earlier `ros2_control` readiness reported joint state broadcaster, steering, and drive controllers ACTIVE. |
| JOINT_STATES_READY | PASS | Earlier readiness received valid `/joint_states`. |
| ODOM_READY | PASS | Earlier readiness received valid `/odom`. |
| FILTERED_ODOM_READY | PASS | Earlier readiness received valid `/odometry/filtered`. |
| LIDAR_RAW_READY | PASS | Earlier readiness received valid `/lidar/points`. |
| LIDAR_FILTERED_READY | PASS | Earlier readiness received valid `/lidar/points_filtered`. |
| SCAN_READY | PASS | Earlier readiness received valid `/scan`. |
| TF_READY | PASS | Earlier readiness verified required TF. |
| NAV2_READY | PASS | Earlier lifecycle and `/navigate_to_pose` action server became ready. |
| DIRECT_FORWARD | UNVERIFIED | Direct ROS motion test was not run; no pose-change evidence. |
| DIRECT_STRAFE | UNVERIFIED | Direct ROS motion test was not run; no pose-change evidence. |
| DIRECT_ROTATE | UNVERIFIED | Direct ROS motion test was not run; no pose-change evidence. |
| DIRECT_NAV_GOAL | UNVERIFIED | Direct Nav2 goal was not run. |
| WEB_MANUAL_R01 | UNVERIFIED | WebSocket-to-Gazebo manual-control test was not run. |
| WEB_NAV_GOAL_R01 | UNVERIFIED | Web/Django R01 navigation goal test was not run. |
| HEADLESS_DEFAULT | PASS | Earlier managed run had `gzserver` and no `gzclient` or `rviz2`. |

## Static validation

- `bash -n` on `scripts/start_stack.sh`, `scripts/stop_stack.sh`, `scripts/status_stack.sh`, and `scripts/ros_env.sh`: PASS.
- `git diff --check`: PASS.
- `python3 -m compileall -q scripts waretwin/backend` in the sourced ROS environment: PASS.
- Targeted tests `tests/test_navigation_readiness.py` and `tests/test_setup_scripts.py`: 9 passed.

## FINAL STATUS

CRASH_ROOT_CAUSE=PASS
STARTUP_BOTTLENECK=PASS
CLOCK_READY=PASS
ROBOT_SPAWNED=PASS
CONTROLLERS_READY=PASS
JOINT_STATES_READY=PASS
ODOM_READY=PASS
FILTERED_ODOM_READY=PASS
LIDAR_RAW_READY=PASS
LIDAR_FILTERED_READY=PASS
SCAN_READY=PASS
TF_READY=PASS
NAV2_READY=PASS
DIRECT_FORWARD=UNVERIFIED
DIRECT_STRAFE=UNVERIFIED
DIRECT_ROTATE=UNVERIFIED
DIRECT_NAV_GOAL=UNVERIFIED
WEB_MANUAL_R01=UNVERIFIED
WEB_NAV_GOAL_R01=UNVERIFIED
HEADLESS_DEFAULT=PASS

SPAWN_ENTITY_DURATION=27.872s
PEAK_GZSERVER_RSS=1,494,896 KiB (~1.43 GiB)
PEAK_SYSTEM_MEMORY=~5.1/7.7 GiB used; ~2.2 GiB available
PEAK_SWAP=66 MiB/2 GiB

REMAINING_ISSUES:
- Current-boot VMware virtual-disk/SCSI I/O errors make another Gazebo/Nav2 run unsafe until guest storage health is restored.
- Direct forward/strafe/rotate, direct Nav2 goal, Web Manual R01, and Web NAV_GOAL R01 acceptance remain UNVERIFIED.
- The underlying host/VMDK cause of the guest storage errors is unknown from guest evidence.
- TF readiness passed, but the readiness logger does not currently record a separate TF event timestamp.

# Final Motion Recovery Session

- Timestamp: 2026-09-30 09:24 +07:00 (Asia/Ho_Chi_Minh; final motion session report)
- Git branch: `web-simulation`
- Commit at session start: `f84a6ba` (`fix: report Gazebo world readiness once`)
- ROS_DOMAIN_ID: runtime `.runtime/stack.env=0`; sourced diagnostic shell `0`
- RMW_IMPLEMENTATION: `rmw_fastrtps_cpp`
- FASTDDS_BUILTIN_TRANSPORTS: `UDPv4`
- ROS_LOCALHOST_ONLY: `0`
- Map/revision: current production launch verified the published canonical warehouse bundle at revision 21; selected `/home/yahboom/swerve_bringup/generated/maps/WH-TEST-01/21/{gazebo/warehouse.world,nav2/warehouse_1.yaml}`
- Robot ID: `R01`
- Runtime mode: `GAZEBO_ROS`; requested production profile `navigation`, headless
- Storage health gate: `free -h` reported 4.4 GiB available; swap 0.5 MiB / 2 GiB; `/` has 6.1 GiB free (90% used); uptime 17 minutes. Current boot scan has no `DID_TIME_OUT`, device `I/O error`, ext4 error, OOM, killed-process, or segfault entries. It does contain recent workqueue latency warnings (`blk_mq_run_work_fn` through 08:31:53 and PSI/e1000 warnings through 08:32:02); these are recorded as latency warnings, not observed storage I/O errors. The controlled runtime decision is pending ownership/port inspection.
- Startup result: the first production `./scripts/start_stack.sh navigation` run reached READY headless at 08:39. It reported `GAZEBO_PROCESS_READY`, advancing `/clock`, SpawnEntity, 56 models including `swerve_base`, active expected controllers and claimed command interfaces, data topics/TF, active Nav2 lifecycle, `/navigate_to_pose`, and R01 bridge connected. T1 was 115.590 s after T0, clock readiness 272.661 s, action server readiness 285.884 s; SpawnEntity call took 91.554 s. A newer same-checkout process set was present at 08:46; trigger not observed. That runtime later disappeared by 09:06 with `.runtime/stack.env` and PID files absent and no retained shutdown cause. Two subsequent production-path retries at 09:12 and 09:17 failed before fresh ROS launch logging/Gazebo startup, then cleaned their owned backend/frontend/ROS groups. Both selected domain 0, Fast DDS UDPv4, canonical revision 21, R01, GAZEBO_ROS, and headless. `system.launch.py --show-args` succeeds in isolation, but the current production stack is unavailable. No motion command was sent.
- DIRECT_FORWARD: UNVERIFIED; no pose-change measurement yet.
- DIRECT_STRAFE: UNVERIFIED; gated on DIRECT_FORWARD.
- DIRECT_ROTATE: UNVERIFIED; gated on DIRECT_FORWARD and DIRECT_STRAFE.
- DIRECT_NAV_GOAL: UNVERIFIED; gated on all direct motion checks.
- WEB_MANUAL_R01: UNVERIFIED; gated on direct motion checks.
- WEB_NAV_GOAL_R01: UNVERIFIED; gated on DIRECT_NAV_GOAL.
- Root causes: Direct movement was gated before sending because idle `/cmd_vel` had 11/11 nonzero samples; the source was not attributed before the stack disappeared. Prior Nav2 logs show off-costmap goals and stale `odom -> map` transforms ending in controller patience aborts, but the goal source and direct-goal behavior are unverified. Web manual and Web NAV_GOAL payloads were not sent because their required direct movement/Nav2 gates did not pass.
- Files changed: `docs/RUNTIME_ACCEPTANCE_20260929.md`, `scripts/end_to_end_acceptance.py`.

## Motion test log

| Timestamp | Test/gate | Result | Evidence / next action |
|---|---|---|---|
| 2026-09-30 08:32 +07:00 | Storage health gate | PASS WITH LATENCY WARNINGS | No active or historical current-boot storage I/O error signatures matched; recent `blk_mq_run_work_fn` latency warnings exist. Inspect stack ownership and backend port before startup. |
| 2026-09-30 08:33 +07:00 | Process ownership gate | PASS | Backend PIDs 8159/8324 have current-checkout cwd and match `.runtime/backend.pid`; no Gazebo/ROS process was listed. Safe to run the ownership-scoped `stop_stack.sh`. |
| 2026-09-30 08:34 +07:00 | Production stop | PASS | `./scripts/stop_stack.sh` stopped the recorded backend process group and removed the runtime snapshot; unrelated processes were not touched. `start_stack.sh navigation` began at monotonic 1206.97 s. |
| 2026-09-30 08:36 +07:00 | Production launch configuration | PASS; ROS startup pending | Runtime snapshot and sourced shell agree on domain 0 / Fast DDS UDPv4; backend health 200; frontend Vite ready; canonical revision 21 selected for R01 at spawn `(15.0, 5.5, 0.2, 1.57079632679)`. No current `gzserver` process/READY observation yet. Recent kernel scan still has no storage I/O error signature. |
| 2026-09-30 08:36 +07:00 | Gazebo process start | PASS; readiness pending | `T1_GAZEBO_PROCESS_STARTED=PASS`, PID 15330, monotonic 1322.56 s; spawn process PID 15338 is running with canonical R01 pose. Headless mode is confirmed by `gui:=false`, `start_rviz:=false`; no motion command sent yet. |
| 2026-09-30 08:38 +07:00 | Gazebo ros2_control plugin | PASS; readiness pending | Current launch log reports `GazeboSystem` successfully initialized, configured, and activated; `gazebo_ros2_control` loaded controller_manager. `gzserver` RSS 1,479,012 KiB, 2.4 GiB available, swap 13 MiB / 2 GiB. No motion probe or storage I/O error observed. |
| 2026-09-30 08:39 +07:00 | Production navigation readiness | PASS | `/runtime/stack.env` and sourced shell agree on ROS domain 0. Production readiness reported model `swerve_base` in a 56-model world, three expected controllers ACTIVE, command interfaces claimed, `/joint_states`, `/odom`, `/odometry/filtered`, raw/filtered LiDAR and `/scan`, required local/global TF, Nav2 lifecycle ACTIVE, `/navigate_to_pose`, and authenticated bridge R01 connected. T2=272.661 s, action server=285.884 s, SpawnEntity=91.554 s. This is startup evidence only; robot motion remains untested. |
| 2026-09-30 08:40 +07:00 | Base controller/graph preconditions | PASS; idle publisher activity under diagnosis | Diagnostic shell reported ROS domain 0. `joint_state_broadcaster`, `steering_controller`, and `drive_controller` are ACTIVE. Both steering position and wheel drive velocity command interfaces are available and claimed. Expected swerve controller, odometry, EKF, robot_state_publisher, controller manager, Nav2, and bridge nodes are present. `/cmd_vel` is `geometry_msgs/msg/Twist`, RELIABLE/VOLATILE endpoints; 7 publisher endpoints belong to `behavior_server` (4), `swerve_bridge`, `tag_route_planner`, and `controller_server`; the sole subscriber is `swerve_controller`. Idle publication frequency/source has not yet been proven.
| 2026-09-30 08:41 +07:00 | Base topic samples | PASS | `/joint_states --once` showed required steering and drive joint position/velocity values finite; effort fields were NaN. `/odom` pose was approximately `(-0.003751, -0.00000003, yaw≈0)` in the odom frame; `/odometry/filtered` approximately `(-0.003969, -0.00000004, yaw≈0)`. Both were valid and advancing. Current Gazebo/map pose will be separately measured before motion. |
| 2026-09-30 08:52 +07:00 | Motion harness update | PASS static compile | `scripts/end_to_end_acceptance.py` samples Gazebo `/model_states`, `/joint_states`, raw/filtered odometry and command/controller data; requires physical model movement and Nav2 goal tolerance; measures idle `/cmd_vel`; and gates the sequence after failures. `python3 -m py_compile scripts/end_to_end_acceptance.py` and `git diff --check` passed. Runtime watchdog parameter query returned `command_timeout=0.5` s on `/swerve_controller`. The frontend repeats manual messages every 0.10 s; bridge manual timeout is 0.40 s; static source maps FORWARD/BACKWARD to ±linear.x, LEFT/RIGHT to ±linear.y, and ROTATE_LEFT/RIGHT to ±angular.z. Runtime motion/cadence are still unverified. `GoalStatus` import is fixed for action result handling.
| 2026-09-30 08:55 +07:00 | Current process set recheck | READY evidence stale for current PID set | `.runtime/stack.env` remains navigation, domain 0, revision 21, R01, GAZEBO_ROS, headless. Recorded current PIDs are backend 24069/frontend 24484/ROS 24624; exactly one `gzserver` PID 25636. Process set started 08:45–08:46, replacing prior PID 15330; trigger not visible in this session. Resource scan: 2.3 GiB available, 360 MiB swap used, load average 13.96/17.35/11.90. No SCSI timeout, device I/O error, ext4 error, OOM or segfault signature; recent VMware/workqueue CPU latency warnings continue. Revalidate the live control layer before motion.
| 2026-09-30 08:58 +07:00 | Current controller preconditions | PASS | On current process set, ROS domain 0; all three controllers ACTIVE; steering/drive command interfaces available and claimed. `/cmd_vel` QoS is RELIABLE/VOLATILE, 7 publisher endpoints (`tag_route_planner`, Nav2 `controller_server`, `behavior_server` x4, `swerve_bridge`) and the only subscriber is `swerve_controller`. No motion command sent. Current idle traffic is to be measured by the acceptance harness before DIRECT_FORWARD.
| 2026-09-30 08:59 +07:00 | Motion harness static validation | PASS | Latest acceptance code passed `python3 -m py_compile scripts/end_to_end_acceptance.py` and `git diff --check`. It now measures watchdog expiry after DIRECT_FORWARD, captures actual Gazebo/model and joint deltas, and exercises the six frontend manual action labels in MANUAL mode with real WebSocket refresh/STOP behavior. No motion test has started.
| 2026-09-30 09:01 +07:00 | E2E protocol setup | FAIL before ROS probe; motion UNVERIFIED | Login at `/api/auth/login` returned 200 in 4.46 s; the Django log shows `/ws` handshaking then disconnect. `websocket.create_connection` in the acceptance script used a 0.02 s connection timeout and raised `WebSocketTimeoutException: Connection timed out`. No `/odom`/pose acceptance result, no `/cmd_vel` test command, and no physical motion was attempted. Harness connection timeout is the isolated cause; direct and product Web behavior remain untested.
| 2026-09-30 09:02 +07:00 | Storage/resource gate before retry | PASS WITH CPU PRESSURE | 2.4 GiB available, swap 463 MiB / 2 GiB, root has 6.0 GiB free; `gzserver` PID 25636 RSS 1,499,936 KiB. Load average 24.72/21.22/16.23 and CPU PSI `some avg10=33.47%`; recent `blk_mq`/VM workqueue warnings are latency warnings. No SCSI timeout, device I/O error, ext4 error, OOM or segfault signature. Retrying only the WebSocket setup plus sequential fail-fast acceptance; no parallel test suite/profiling.
| 2026-09-30 09:03 +07:00 | E2E idle `/cmd_vel` gate | BLOCKED before DIRECT_FORWARD; no motion sent | Corrected WebSocket handshake passed; ROS `/odom`/clock readiness passed. One-second idle observation saw 11 non-zero `/cmd_vel` samples, so DIRECT_FORWARD was marked UNVERIFIED and the mandatory gates skipped direct Nav2, Web Manual and Web NAV_GOAL. No acceptance `/cmd_vel` publisher command was sent. `artifacts/runtime_acceptance/latest.json` contains the observed idle samples. Exact active publisher/source remains to be identified.
| 2026-09-30 09:06 +07:00 | Production stack loss and retained-log diagnosis | FAIL; motion tests remain gated | `.runtime/stack.env` and runtime PID files are absent; `status_stack.sh` found no owned backend/frontend/ROS processes or live health endpoints. `ros.log` ends at 09:06:09 and `backend.log` records `/ws/ros` bridge disconnect at 09:06:31. No shutdown trigger or process exit code is retained. The same ROS log contains repeated Nav2 recovery/goal activity, goals reported off the global costmap, `controller_server` control-loop misses, stale `odom -> map` transforms (data 91.137 s vs TF 90.836 s at 09:05:13), then `Controller patience exceeded` and `follow_path` abort. A read-only `/navigate_to_pose/_action/status` snapshot contained two terminal status values `6` (`ABORTED`), with no active goal in that snapshot. This is Nav2/TF evidence, not proof of the source of the earlier idle `/cmd_vel` samples. A later publisher probe was invalid because the stack was already gone and its shell had fallen back to domain 12; it observed no graph and sent no command. Current health: 4.4 GiB available, 504 MiB swap used, 6.0 GiB free on `/`, no current-boot storage I/O/ext4/OOM/segfault signatures; CPU/workqueue latency warnings and RT throttling were observed. Do not infer a Gazebo crash cause from these logs.
| 2026-09-30 09:12 +07:00 | Repeat storage/resource gate | PASS WITH HISTORICAL CPU LATENCY WARNINGS | 4.3 GiB available, swap 504 MiB / 2 GiB, root has 6.0 GiB free (90% used), uptime 57 minutes, `gzserver` absent. Current-boot grep found no `DID_TIME_OUT`, device I/O, ext4, OOM, killed-process, or segfault errors. Kernel log retains earlier workqueue latency warnings and RT throttling at 09:02; current CPU PSI `some avg10=1.21%`, no full pressure. Production restart may proceed cautiously; no motion or alternate launch command was sent.
| 2026-09-30 09:12 +07:00 | Second production launch configuration | PASS; Gazebo startup pending | Re-ran the specified `stop_stack.sh` → `source scripts/ros_env.sh` → `start_stack.sh navigation` path. The runtime snapshot and launch both selected domain 0, `rmw_fastrtps_cpp`, `ROS_LOCALHOST_ONLY=0`, UDPv4, GAZEBO_ROS, headless, canonical map revision 21, and R01. Backend chose 8001 because port 8000 is occupied by an unrelated process; no unrelated process was changed. Frontend is starting; `gzserver` has not started yet. No motion command sent.
| 2026-09-30 09:13 +07:00 | Second production launch result | FAIL before `ros2 launch`; no motion sent | `start_stack.sh` timed out its 45 s Gazebo-marker gate and cleaned only its owned backend/frontend/ROS groups. Backend and Vite logs show they started; `logs/ros.log` and `~/.ros/log` were not refreshed, no new ROS child or `gzserver` remains, and no controller or motion test ran. This places failure before ROS 2 launch logging, but the specific child startup step is not yet known. No storage/OOM/segfault signature appeared in the last five minutes. Do not classify this as a Gazebo crash or modify LiDAR/physics; preserve logs and diagnose the pre-launch step before retrying.
| 2026-09-30 09:14 +07:00 | Isolated pre-launch shell check | PASS; production failure cause still unknown | A child-equivalent shell successfully sourced `scripts/ros_env.sh`, selected domain 0/8001, and ran `ros_stack_supervisor.py --help` in 7.95 s. That shows the basic sourced environment is usable but does not explain why the prior owned ROS child never refreshed `ros.log` or created a new `~/.ros/log` directory. Publisher-GID capture is now in the acceptance harness; it remains runtime-unverified. A monitored production-path retry was scheduled and its result is recorded below.
| 2026-09-30 09:18 +07:00 | `/cmd_vel` publisher attribution harness check | PASS isolated observer; production traffic UNVERIFIED | ROS 2 Humble's stock `SingleThreadedExecutor` discards the `MessageInfo` returned by `take_message`; the acceptance probe now uses a narrow executor override to preserve `publisher_gid` for `/cmd_vel` while leaving other callbacks unchanged. `py_compile`, CLI help, and a no-publish observer spin passed. On domain 0 after the failed production launch, `ros2 topic info /cmd_vel` reported no topic and the one-second observer saw 0 samples; its endpoint inventory contained only the acceptance node's own idle publisher. This does not attribute the earlier 11 production-stack samples.
| 2026-09-30 09:18 +07:00 | Third production launch result | FAIL before Gazebo marker; no motion sent | A second monitored retry through `source scripts/ros_env.sh` → `start_stack.sh navigation` again waited 45 s without a fresh ROS log or `~/.ros/log` directory, then cleaned its own started backend/frontend/ROS process groups. It did not start `gzserver`, and no motion test ran. Repeated failure is before the ROS launch CLI; further heavy retries are stopped pending an isolated launch-command diagnosis. No storage I/O or OOM error was observed.
| 2026-09-30 09:19 +07:00 | Isolated ROS launch argument expansion | PASS; production runtime still unavailable | The production `swerve_bringup system.launch.py --show-args` command resolved all launch arguments and package paths in 7.39 s under domain 0 without starting Gazebo or ROS nodes. This rules out a missing package/launch description; the two production retries remain unexplained before a fresh ROS launch log is created. No motion command sent.
| 2026-09-30 09:21 +07:00 | Requested static validation | PASS | `bash -n` passed for `start_stack.sh`, `stop_stack.sh`, `status_stack.sh`, and `ros_env.sh`; `python3 -m compileall -q scripts waretwin/backend` passed; `git diff --check` passed. Harness-specific `py_compile`, `--help`, and no-publish ROS executor smoke also passed. No frontend source changed, and no broad test suite was run.

## FINAL MOTION STATUS

DIRECT_FORWARD=UNVERIFIED
DIRECT_STRAFE=UNVERIFIED
DIRECT_ROTATE=UNVERIFIED
DIRECT_NAV_GOAL=UNVERIFIED
WEB_MANUAL_R01=UNVERIFIED
WEB_NAV_GOAL_R01=UNVERIFIED

ROOT_CAUSE_DIRECT_MOTION=UNVERIFIED: the domain 0 idle gate observed 11 non-zero `/cmd_vel` samples out of 11 and withheld all test commands; publisher identity could not be measured before the stack disappeared.
ROOT_CAUSE_NAV2=UNVERIFIED for the requested direct goal: retained logs show planner goals outside the global costmap and stale `odom -> map` data (91.137 s versus TF 90.836 s), followed by controller patience exceeded and `follow_path` abort; source of those earlier goals is unknown.
ROOT_CAUSE_WEB_MANUAL=UNVERIFIED: gated before the `ROBOT_MODE`/`ROBOT_MANUAL` motion payloads because direct movement did not pass; no manual command reached the bridge during this session.
ROOT_CAUSE_WEB_NAV=UNVERIFIED: gated because direct Nav2 did not pass; no Web `NAV_GOAL` payload was sent.

FILES_CHANGED=docs/RUNTIME_ACCEPTANCE_20260929.md,scripts/end_to_end_acceptance.py
REMAINING_ISSUES=No current production stack; two retries stopped before fresh ROS launch logging; idle `/cmd_vel` source and previous Nav2 goal source unresolved; no physical motion evidence for any of the six required tests.

## Command ownership follow-up (static only)

| Timestamp | Test/gate | Result | Evidence / next action |
|---|---|---|---|
| 2026-09-30 13:08 +07:00 | Command source ownership implementation | PASS static; motion UNVERIFIED | The prior graph had seven publishers sharing `/cmd_vel`; the source of the 11 non-zero idle samples is still unattributed. Added a production `command_arbiter` with distinct direct/manual, Web manual, Nav2, and tag-approach inputs; MANUAL/AUTONOMOUS mode and E-STOP gates select one fresh owner, and the swerve controller consumes `/cmd_vel_selected`. Web manual leases publish one final zero after expiry. Tag-route cancellation now releases tag ownership. The production readiness gate requires the arbiter node, selected-topic publisher/subscriber, and a live owner sample. The physical-motion acceptance harness verifies source topic, selected command, and `/command_owner`. Targeted ownership, readiness, map-sync, and renderer tests: 24 passed. `./scripts/build_ros.sh` built both ROS packages; launch `--show-args` passed. No Gazebo stack or motion probe was started, so DIRECT_FORWARD, DIRECT_STRAFE, DIRECT_ROTATE, DIRECT_NAV_GOAL, WEB_MANUAL_R01, and WEB_NAV_GOAL_R01 remain UNVERIFIED. Next: start through the production stack path and execute the ordered physical-motion gates. |

FOLLOWUP_FILES_CHANGED=CMakeLists.txt,README.md,config/command_arbiter.yaml,config/swerve_controller.yaml,config/tag_navigation.yaml,docs/ARCHITECTURE.md,docs/LOCAL_ROBOT_CONTROL_ARCHITECTURE.md,docs/ROS_TOPICS.md,launch/gazebo.launch.py,scripts/end_to_end_acceptance.py,scripts/navigation_readiness.py,scripts/test_command_ownership.py,swerve_bridge/config/bridge.yaml,swerve_bridge/swerve_bridge/bridge_node.py,swerve_controller/command_arbiter_node.py,swerve_controller/command_ownership.py,swerve_controller/swerve_controller_node.py,swerve_controller/tag_navigation_core.py,swerve_controller/tag_route_planner_node.py,swerve_navigation/launch/navigation.launch.py,tests/test_navigation_readiness.py
