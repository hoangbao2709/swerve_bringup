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
