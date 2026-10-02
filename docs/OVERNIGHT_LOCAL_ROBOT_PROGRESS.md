# Overnight Local Robot Completion

Current branch: web-simulation
Current HEAD: a0d35e4
Started: 2026-10-03 (Asia/Ho_Chi_Minh)
Last updated: 2026-10-03 (Asia/Ho_Chi_Minh)

| ID | Task | Status | Commit | Runtime evidence | Notes |
|---|---|---|---|---|---|
| T00 | Preflight/recovery | BLOCKED | 3f0b192 | Current-boot kernel journal, 2026-10-02 22:02 and 2026-10-03 00:25 local | Branch `web-simulation`; pre-ledger HEAD `d431326`; `origin/web-simulation...HEAD` = `0 0`; `git diff --check` clean before ledger update. Root filesystem 59% used, 39G available; 7.7 GiB RAM with 5.3 GiB available; 2 GiB swap with ~1.9 GiB free. Current boot has repeated `/dev/sda` `DID_TIME_OUT` and I/O errors; `jbd2/sda3-8` and workers blocked. Heavy Gazebo runtime is unsafe. |
| T01 | Global warehouse regression | PASS | d431326 | `docs/ROBOT_POSE_ALIGNMENT_20261002.md#runtime-result`; `docs/evidence/web_pose_alignment_20261002.json`; `.runtime/pose-alignment-AqoPO6/p1-2d.png`, `p2-3d.png`, `p3-2d.png` | Existing same-source runtime: canonical warehouse visible in Mapping at three poses; 2D/3D position errors max < 0.000001 m and yaw errors max < 0.000009 rad. Prior 30 s motion probe: marker visible in 316/316 100 ms samples; 9 unique Gazebo pose samples; source `CANONICAL_WAREHOUSE`, pose `GAZEBO_MODEL_STATES`, revision 21. No source changes since runtime. |
| T02 | 2D SLAM accumulated map | BLOCKED | a0d35e4 | `.runtime/mapping-browser-final.json`; `.runtime/mapping-browser-control-observer.json`; `.runtime/mapping-browser-control-verified.json` | Prior real SLAM Toolbox map snapshots grew v3→v4→v5: known cells 45,231→60,875→71,741; extent 431×598→440×598→446×598; one session `e71a0aa5f19e`; trajectory 1→8 and 624-point scan frames were observed. However, no synchronized Pose A/B/C plus per-map hashes/old-cell comparison was retained. Current-boot storage I/O errors block the requested controlled rerun. First failing gate: runtime evidence completeness, not evidence of a map-clearing defect. |
| T03 | 3D accumulated SLAM cloud | BLOCKED | d431326 | Prior mapping Web/ROS probe; `swerve_bridge/test/test_mapping_snapshot_handoff.py::test_3d_cloud_accumulates_in_slam_map_while_global_is_visible` | Prior probe: filtered cloud 11 messages/8 s; exact map←lidar TF available in 11/13 lookups; Web 3D remained at 0 points and no `LIDAR_MAP_3D` event was captured. First observed divergence is after filtered cloud/TF input and before a verified Web frame; exact sublayer is unresolved. Source uses exact-stamp TF with 2.5 s wait and bounded voxel accumulation; targeted regression now confirms a missing exact TF raises/drops the cloud without adding points. Test PASS (1/1); install/build and real Web/Gazebo retest blocked by T00. |
| T04 | Robot pose/frame stability | RUNNING | - | `docs/ROBOT_POSE_ALIGNMENT_20261002.md#runtime-result`; frontend frame-source audit in progress | Checking generic pose consumers and the cached-pose/TTL contract against existing three-position runtime measurements. |
| T05 | Resume Mapping | PENDING | - | - | - |
| T06 | Map Point navigation | PENDING | - | - | - |
| T07 | Tag registry/navigation target | PENDING | - | - | - |
| T08 | Tag dropdown UI | PENDING | - | - | - |
| T09 | Tag path preview | PENDING | - | - | - |
| T10 | Tag navigation runtime | PENDING | - | - | - |
| T11 | Navigation cancel/safety | PENDING | - | - | - |
| T12 | Navigation repeatability/accuracy | PENDING | - | - | - |
| T13 | Large-route mapping/loop closure | PENDING | - | - | - |
| T14 | Full integrated acceptance | PENDING | - | - | - |

## T00 preflight evidence

- `git fetch origin` succeeded; `origin/web-simulation...HEAD` is `0 0`.
- `df -h /`: `/dev/sda3` 98G total, 55G used, 39G available (59%).
- `free -h`: 7.7GiB total RAM, 5.3GiB available; swap 2.0GiB total, 1.9GiB available.
- Current-boot `journalctl -k -b` includes blocked `jbd2/sda3-8`, repeated SCSI `hostbyte=DID_TIME_OUT`, and `/dev/sda` read I/O errors at 2026-10-02 22:02 and 2026-10-03 00:25 local. This is a storage-health failure even though free capacity and memory are adequate.
- Decision: `T00=BLOCKED`; do not launch or repeatedly restart Gazebo/RViz until a healthy boot is verified. Static/source/test inspection may continue without claiming runtime acceptance.

## T01 runtime evidence

- The runtime report and browser evidence record three separated actual Gazebo poses and rendered canonical warehouse 2D/3D views while Mapping was active. The screenshots show the boundary, left/right storage, aisles, graph nodes and R01 marker. The reported 2D/3D pose errors are all below `3.9e-7 m` and `8.8e-6 rad`; the Mapping TF comparison is also below `4.7e-6 m` and `7.4e-5 rad`.
- A separate same-source 30 s motion probe from the immediately preceding execution observed 316/316 100 ms samples with the marker visible, minimum measured teal pixel area 6, and 9 unique Gazebo pose readings. The canonical source was `CANONICAL_WAREHOUSE`; marker pose source was `GAZEBO_MODEL_STATES`; warehouse revision was 21.
- The source commit under test remains `d431326`; the intervening change was only the overnight ledger. T01 passes without restarting the unhealthy VM.

## T02 prior map-growth evidence and blocker

- `mapping-browser-final.json`: SLAM Toolbox session `e71a0aa5f19e`, version 3 at 45,231 known cells / 431×598 and version 4 at 60,875 / 440×598.
- `mapping-browser-control-observer.json`: the same session advanced to version 5 at 71,741 known cells / 446×598. The Web mapping observer recorded trajectory growth from 1 to 8 and current `/scan` frames with 624 points.
- These artifacts support genuine SLAM map growth, but they do not preserve synchronized Pose A/B/C values, a map hash per pose, or a pixel/cell-level old-area persistence comparison. The new stricter gate is therefore not a pass. First blocker: a controlled runtime measurement was not safely repeatable after the current boot's `/dev/sda` timeouts and read I/O errors.

## T03 runtime and regression evidence

- In the prior real Mapping/Web probe, `/filtered_points` produced 11 messages in 8 s and exact timestamped `map <- lidar_link` TF succeeded for 11/13 lookups. The 3D UI remained at 0 points and no `LIDAR_MAP_3D` browser event was captured. This is not a runtime pass; the first observed divergence is in the bridge-to-Web cloud-frame path, with its precise internal layer unresolved.
- Source commit `d431326` waits up to 2.5 s for the cloud's exact timestamped TF, transforms into `map`, merges into a 20,000-point bounded voxel map at 0.04 m, and tags the frame as `SLAM_VISUALIZATION_VOXEL_MAP`. No latest-TF substitute is used; a failed lookup propagates to the render guard and emits no frame.
- Added a regression assertion that an unavailable exact-stamp transform does not add points or queue a Web frame; `source scripts/ros_env.sh && python3 -m pytest -q swerve_bridge/test/test_mapping_snapshot_handoff.py::test_3d_cloud_accumulates_in_slam_map_while_global_is_visible` passed (1 test). The built install and full real runtime retest were not attempted because T00 is blocked by current-boot storage errors.
