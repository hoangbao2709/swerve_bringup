# Robot pose alignment — 2026-10-02

Branch: `web-simulation`. Scope: robot overlay frame semantics during Mapping.

## Root cause and source trace

Before the fix, the main 3D `Robots` component used `twin.robots[id].position`
and `heading` directly. The main 2D view's earlier safety fix hid the marker
during Mapping. Those legacy coordinates came from Django
`TwinRuntime._apply_ros_robot_state`, which applied `ros_pose_to_waretwin` to
the authenticated bridge's top-level `ROBOT_STATE`. `SwerveBridge.telemetry_timer`
obtained that pose via `_lookup_robot_pose`: TF `map -> base_footprint` (with the
existing base-frame fallback). During Mapping, SLAM Toolbox owns `map -> odom`.
Thus the top-level pose was in the independent SLAM frame, not the canonical
warehouse frame used by both warehouse renderers. Equal frame names did not
establish equal origins or yaw.

## Corrected contract

- `slam_pose`: SLAM TF `map -> base_footprint`, with the same SLAM session/map ID
  as the authoritative `/map`. Used by the dedicated Mapping canvas.
- `canonical_pose` during simulation Mapping: exact `swerve_base` model pose
  from `/model_states`, with validated world-to-canonical identity. Used by
  both main 2D and 3D warehouse views through `robotPoseFrame.ts`.
- `GazeboCanonicalAlignment` checks the published world's path, SHA-256 hashes,
  frame/metres/revision, canonical floor boundary/holes/bounds, robot identity,
  and SDF boundary-wall poses against canonical geometry. It also checks at
  least two separated live static anchors and rejects any mismatching anchor.
  No spawn anchor, fixed offset, scale change or Y correction is inferred.
- Payloads identify x/y/yaw, frame, map ID/revision/source, pose source,
  timestamp and validity. Canonical Gazebo poses also identify source frame
  `world` and transform source `VALIDATED_CANONICAL_WORLD_BUNDLE`.
- Frontend rejects incompatible, missing, invalid, nonfinite, stale or offline
  poses. External 3D interpolation is disabled so 2D and 3D use the same current
  measured canonical pose. LOCAL_SIM interpolation remains unchanged.
- Legacy generic coordinates remain available for existing saved-local-map
  consumers; the canonical warehouse views never fall back to those coordinates.

The entity origin and base footprint share planar xy/yaw: the URDF's fixed
base_footprint-to-base_link joint is vertical only. This is not a geometric
correction of the robot marker.

## Automated checks

- Bridge alignment tests: 2 passed; rejects mismatching/missing live anchors
  and tampered artifact hashes.
- Django ROS telemetry tests: 5 passed; distinct SLAM and canonical poses are
  retained, and an old SLAM session is rejected.
- Frontend frame, control-route and robot-workflow tests: 33 passed.
- TypeScript check and production Vite build: passed (existing chunk-size
  warning only). Runtime bundle uses the same environment as start_stack,
  including the selected backend port from `.runtime/stack.env`.
- Bridge ROS build, Python/JS syntax checks and `git diff --check`: passed.

## Runtime method

Production startup: `./scripts/start_stack.sh mapping --gui --rviz`.
Canonical bundle: WH-TEST-01 revision 21. RViz fixed frame: `map`.

`scripts/pose_alignment_observer.py` independently records exact Gazebo entity
pose, physical stopped state, clock, selected command and SLAM TF.
`scripts/web_pose_alignment_acceptance.cjs` authenticates the real production
Web, reads the actual SVG translation/heading and the actual Three.js mesh
translation/yaw (published by useFrame to label attributes), then reads the
Mapping canvas's selected draw pose. No backend/API pose is substituted for a
renderer measurement.

Default test placement uses `/set_entity_state` to move the actual Gazebo model
to three nearby, distinct test poses while selected commands are zero. Placement
targets are test fixtures, not offsets in production frame conversion. The
observer verifies service success and actual physical placement. Sampling waits
for physical linear/angular speed <= 0.005, selected zero, fresh Gazebo/TF and
2.5 seconds of continuous settling. Limits stay <= 0.05 m and <= 0.05 rad.
Optional `POSITION_METHOD=web-manual` exercises actual pointer teleop instead;
the default placement run is not a teleop or navigation acceptance claim.

Run the observer after sourcing `scripts/ros_env.sh`, with `--output` pointing
to an existing runtime evidence directory's `observer.json`. Explicitly add
`--allow-placement` for the default Gazebo-placement probe; otherwise the
observer is read-only. Old placement requests are ignored. Run the browser
probe with `POSE_ALIGNMENT_DIR`, `BACKEND_URL`, `FRONTEND_URL` and the existing
admin credential environment. It writes raw measurements, browser errors and
screenshots without saving authentication credentials.

## Runtime result

PASS: three distinct actual Gazebo positions, with production Web 2D/3D and
RViz open in Mapping. Canonical frame/map: `map` / `CANONICAL`, revision `21`.
SLAM frame/map: `map` / `SLAM-dc3e46b37591`, session `dc3e46b37591`.

| Position | Gazebo x/y/yaw (m/m/rad) | 2D error m/rad | 3D error m/rad | Mapping vs TF error m/rad |
| --- | --- | --- | --- | --- |
| 1 | 15.299959 / 6.299841 / 1.223600 | 3.8755e-7 / 1.9388e-7 | 1.7743e-7 / 8.7966e-6 | 1.5771e-6 / 7.3695e-5 |
| 2 | 15.299959 / 6.699841 / 1.223601 | 2.3200e-7 / 2.4346e-7 | 5.5007e-8 / 2.3081e-6 | 4.6433e-6 / 1.4939e-5 |
| 3 | 15.599964 / 7.099848 / 0.873607 | 2.0097e-7 / 1.3412e-7 | 3.7190e-7 / 6.4597e-7 | 4.3160e-8 / 4.2884e-5 |

All values are below 0.05 m / 0.05 rad. Zero browser page errors, console
errors or failed requests in the passing run. Raw actual/expected poses,
individual sampling truth and test configuration are committed in
`evidence/web_pose_alignment_20261002.json`.

Runtime screenshots and pre-restart SLAM backup are retained under
`.runtime/pose-alignment-AqoPO6/`. The third-position 3D screenshot visibly
shows R01 in the central warehouse aisle, consistent with its canonical pose.
The Mapping UI identifies SLAM Toolbox as ACTIVE, TF as OK, and the matching
SLAM session. RViz renders the robot/TF in fixed frame `map`.

Earlier attempts are not PASS: a stale observer, a standalone bundle missing
the selected backend port, and software-rendered 3D initialization/capture
timeouts interrupted those attempts. The passing run uses a headed Chromium
OpenGL backend with default medium-quality scene and lights on; neither pose
data nor error/freshness/settling limits were changed. Snapshot captures may
show a temporarily masked robot when its pose has expired under VM load.
This is the required fail-closed behavior, not a fallback to SLAM coordinates.

These are Gazebo entity-placement alignment tests, not continuous manual
teleop, Nav2 completion, map quality, or physical-hardware acceptance.

After measurements, the observer's explicit-placement gate was smoke-tested:
a previous request was ignored, a fresh opt-in request to the current pose
succeeded, and the default read-only observer ignored a fresh request. SLAM
serialization to `post-alignment-slam` returned result 0 (18 MB pose graph plus
29 KB data); the pre-restart session backup remains preserved separately.
The final read-only observer stopped cleanly. `stop_stack.sh` then confirmed
the owned ROS, frontend and backend process groups stopped; unrelated services
were not touched.
