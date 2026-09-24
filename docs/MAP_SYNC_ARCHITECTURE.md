# Map synchronization architecture

## Audited root causes (before implementation)

1. The database `WarehouseMapVersion` and its generated artifact directory are
   already the publish authority, but the bundle contains canonical JSON,
   DataMatrix/tag graph YAML and a Gazebo world only. It does not generate the
   Nav2 occupancy image/YAML, so navigation can start with the independent
   package map `swerve_navigation/maps/warehouse.yaml`.
2. Canonical layout/export metadata and QR/DataMatrix YAML say
   `warehouse_map`; the ROS bridge defaults to `map`. No single global-frame
   migration/adapter contract connects those names.
3. `swerve_bridge.telemetry_timer()` sends the latest `/odometry/filtered`
   position directly as `ROBOT_STATE`, without reading its header frame or
   looking up `map -> base_footprint`. Backend then stores those coordinates as
   the warehouse pose. This is not valid when the odometry frame is `odom`.
4. Overview `MapView2D` draws warehouse and robot coordinates directly in an
   SVG viewBox whose Y axis increases downward. The robot detail canvas has a
   separate transform that inverts Y and uses occupancy-map bounds, while its
   warehouse overlays still come from the frontend layout with zero-origin
   bounds. Thus the views can disagree even for identical metric data.
5. LiDAR is transformed through TF to the configured `map` frame, but path
   payloads retain their source frame and the renderer treats their points as
   global. Map snapshots, layout geometry, path and tag layers therefore do
   not share a checked frame contract.
6. `start_stack.sh` warns and selects the package development world if a
   published bundle or the selected robot spawn record is absent. Navigation
   defaults to the package map YAML. This allows a runtime to combine a new Web
   layout with an old Gazebo world and/or Nav2 map.
7. Existing sync status compares only published, bridge-reported ROS and
   Gazebo revision integers. `MAP_PUBLISHED` reads tag/world files but does not
   reload Gazebo, Nav2 or the tag consumer; the bridge can report a matching
   world revision without proving those consumers loaded that revision. No
   `nav2_revision`, `tag_map_revision` or TF health participates in SYNCED.
8. Map publish creates the artifact bundle atomically, but does not produce a
   Nav2 map or coordinate a runtime reload/acknowledgement sequence.
9. Simulator encoder odometry starts in a local `odom` frame at `(0, 0, 0)`,
   while the entity is spawned at its canonical warehouse pose. A live mapping
   run showed SLAM scan matching could then shift `map -> odom`; the resulting
   TF was not a trustworthy absolute warehouse pose. The simulation contract
   now keeps SLAM's exploratory map on `/slam/map` and uses the canonical tag
   localization pipeline as the sole `map -> odom` publisher.
10. The V30E simulator and readiness gate depend on Gazebo's
    `/get_entity_state`, but generated worlds did not load the Gazebo ROS state
    plugin. Mapping startup therefore failed before it could prove a global
    pose. Generated worlds and runtime validation now require that plugin.
11. A live run produced the correct tag-derived absolute measurement near the
    canonical spawn, but `ekf_v30e` remained near `(0, 0, 0)`: its zero prior
    and small initial covariance rejected the first far-away map measurement.
    Simulation now seeds only the global filter from the selected revision's
    robot spawn; local encoder odometry remains in `odom` and is not promoted
    to a warehouse position.
12. The bridge subscribed to `/map` with the default volatile QoS. A late bridge
    could miss map_server's one-shot transient-local occupancy snapshot and
    report `nav2_revision: N/A` although the map server had loaded the correct
    artifact. The bridge now requests the reliable transient-local map QoS and
    can validate the latched snapshot after startup/reconnect.
13. Runtime navigation exposed a bridge crash while transforming a freshly
    stamped Nav2 path: its timestamp was 12 ms newer than the latest
    `map -> odom` TF. The bridge now permits a latest-TF fallback only for a
    bounded small future skew, rejects stale/past lookups, and catches path or
    goal transform failures so diagnostics report TF unavailable without
    terminating the telemetry process.

These findings describe the checked-in code path; no runtime component is
considered synchronized until it reports the loaded revision and required TF.

## Intended invariant

The active immutable `WarehouseMapVersion.layout` and its revision artifacts
are the only warehouse geometry source. Global objects and goals use `map`,
metres, ROS +X/+Y and CCW radians. Web view transforms are presentation-only
and reversible. Robot absolute pose comes from TF `map -> base_footprint`
(with `base_link` as an explicit fallback only when the former frame is absent),
never from raw odometry coordinates.

```text
                 CANONICAL MAP / revision N (database)
                         frame=map, metres
                              |
                 immutable generated artifact bundle
                    /          |          \
                 Gazebo       Nav2        Tags/graph
                    \          |          /
                   loaded-revision acknowledgements
                              |
                             ROS
                    TF map -> base_footprint
                              |
                 one reversible Web world transform
                              |
                             Web
```

`SYNCED` is a measured state: published, Gazebo, ROS map, Nav2 and tag-map
revisions agree, required map-frame TF is fresh, and required runtime consumers
acknowledged loading. Missions are blocked otherwise; Emergency Stop is not.

## Implemented data contract

- `WarehouseMapVersion.layout` remains the canonical persisted layout. Publish
  validates it and atomically builds an immutable
  `generated/maps/<warehouse>/<revision>/` bundle before activating that
  version. No pixel-space geometry or second layout database was introduced.
- `canonical_map.json` declares `frame_id: map`, `units: m`, yaw in radians,
  and world-space `origin`, `width`, and `height`. Existing object coordinates
  are preserved; normalization derives bounds but never shifts geometry.
- The bundle contains canonical JSON, Gazebo world and mesh files, DataMatrix
  YAML, tag graph YAML, and Nav2 PGM/YAML per floor plus a primary-map alias.
  The manifest records each artifact path, revision, frame, origin/bounds,
  spawn poses, Nav2 bounds and SHA-256 for all generated files.
- Nav2 occupancy cells are rasterized from the same floor/wall/hole/rack,
  station/obstacle, conveyor and column geometry. PGM row order is north-to-
  south; the YAML origin is the floor's minimum world X/Y and `frame_id` is
  `map`. Resolution is 0.05 m/cell (bounded to 16 million cells).
- Robot spawn is read from the revision's Gazebo manifest. Runtime validates
  that world, floor-specific Nav2 map and tag inputs resolve inside the same
  immutable revision and that their recorded hashes still match.
- Gazebo spawn and tag poses are both generated from the selected canonical
  revision. In simulation, encoder odometry remains local; V30E tag
  measurements correct the global estimator, whose initial map prior is the
  same revision's selected robot spawn. This avoids both zero-origin rejection
  and an assumption that raw odometry is already in `map`.
- In simulated mapping, the canonical Nav2 map server owns `/map`; SLAM's
  exploratory occupancy output is isolated on `/slam/map` with TF publishing
  disabled. The V30E/global EKF is the sole simulation `map -> odom` publisher.
  On a physical mapping setup, SLAM remains the owner of `/map` and
  `map -> odom`.
- The ROS bridge sends absolute robot position only after a fresh TF lookup of
  `map -> base_footprint`, falling back to `map -> base_link` only when the
  preferred frame cannot be looked up. `/odometry/filtered` contributes twist
  only. Robot state carries `robot_id`, `frame_id`, and `map_revision`;
  backend rejects any state in `odom` or from another published revision.
- Scan points and every path/goal pose are transformed using tf2 into `map`
  before sending. LiDAR is throttled to `lidar_ui_hz` (5 Hz by default) and
  bounded to `lidar_max_points` (720 by default). Occupancy snapshots are
  content-cached and resent when content or revision changes, not at telemetry
  frequency.
- `coordinates.ts` is the frontend's reversible world/view transform. It uses
  map bounds, viewport, zoom and center; screen Y is inverted once. The 2D
  warehouse renderer applies the same world-axis convention to its global
  layers. Robot detail converts map/scan/path/goal points with the shared
  `worldToScreen` and click goals with `screenToWorld` before sending
  `frame_id: map`.

## Publish and runtime reload

1. Validate the draft/canonical layout (floor polygons and holes, finite
   geometry, object/tag IDs and references, graph edges, tags/aisle bounds,
   and Gazebo spawn poses).
2. Export all artifacts into a temporary sibling directory, verify required
   files, hash them, then activate the immutable revision with the database
   transaction. A failed export does not activate a partial revision.
3. Broadcast the new published revision. The connected bridge verifies the
   manifest and hashes. If its selected Gazebo/Nav2/tag inputs do not match,
   it zeros manual velocity, cancels an active navigation goal, and writes an
   atomic `.runtime/map-sync-request.json` request.
4. `ros_stack_supervisor.py` independently re-verifies the revision bundle,
   gracefully stops the owned ROS launch, and relaunches Gazebo, Nav2, tag
   consumers and bridge with the new revision. It retains and reports an
   invalid request; it never chooses a package world/map as fallback.
5. The bridge reports the loaded Nav2 revision only after `/map` agrees with
   the selected PGM/YAML in frame, dimensions, resolution, origin and cells.
   Navigation tag revision is reported only after both tag-consuming ROS nodes
   (`v30e_sim_node` and `tag_route_planner`) are present; their launch
   parameters resolve to the verified revision files, and each node must have
   parsed its file to start. `map -> base_*` must be fresh. Until all required checks agree,
   runtime remains `SYNCING`/`OUT_OF_SYNC`/`ERROR`, and mission/navigation
   start or resume is rejected. Emergency Stop remains available.
6. Map acknowledgements carry `robot_id`. The backend retains each connected
   bridge's revision/TF report and requires every connected robot to report a
   fresh, agreeing revision; a synchronized robot cannot hide another robot
   that is stale or running a different map.

## Runtime and development startup

`./scripts/start_stack.sh mapping` and `navigation` read the active published
revision from Django, verify the complete bundle for the selected robot, and
log the warehouse/revision/frame/artifact paths/spawn/ROS domain. Navigation
selects the Nav2 map for the spawn pose's floor. A missing or invalid published
bundle fails before ROS starts. The packaged demonstration world/map can only
be selected explicitly with `--allow-dev-world` (or
`WARETWIN_ALLOW_DEV_WORLD=true`); explicit `--world`/`--map` are also treated
as development assets and require that opt-in.

Simulated mapping runs the canonical `map_server` to own `/map` and the tag
localization nodes that own `map -> odom`; it does not run the Nav2 planner /
navigation lifecycle. The occupancy map and tag inputs are still revision
checked in mapping mode. Navigation additionally requires the active planner,
controller, behavior-tree and waypoint lifecycle servers for navigation
readiness.

## Troubleshooting

- `No valid published map bundle`: publish a map to generate the current
  revision artifacts; do not copy an older world/map into the revision folder.
- `artifact hash mismatch`: an immutable artifact was changed after publish;
  retain the revision for diagnosis and publish a fresh revision.
- `OUT_OF_SYNC` with a component revision: compare the revision values exposed
  by `/api/map/sync-status`; the bridge asks the stack supervisor to reload
  when the running bundle differs.
- `OUT_OF_SYNC` with TF detail: verify the live `map -> base_footprint` (or
  `base_link`) transform and timestamps. Odometry is not a substitute for a
  global pose. For a path stamped just ahead of `map -> odom`, only the
  configured small future-TF tolerance may use the latest transform; older or
  larger gaps are surfaced as a TF error and are not rendered as global data.
- `nav2_revision: null`: verify the canonical map server is active, the bridge
  subscribes with reliable transient-local QoS, `/map` uses frame `map`, and
  occupancy dimensions/origin/resolution/cells match the selected revision's
  Nav2 YAML/image.
- `tag_map_revision: null`: verify the DataMatrix and graph input paths both
  point into the same revision directory and the tag consumer restarted.
- Development fallback is intentionally opt-in and is never a valid published
  revision; it must not be used to claim synchronized production mapping.

## Verification boundary

Unit and integration checks verify artifact generation, hashes, coordinate
round-trips, frame rejection, status gating, and supervisor selection. A live
Gazebo pose comparison, measured +1 m/+90-degree movement, obstacle-to-LiDAR
alignment, clicked Nav2 goal, and publish-during-runtime reload are reported
as verified only when those commands are actually exercised against the local
simulation. The status fields are not a substitute for those physical runtime
checks.

```text
                 CANONICAL MAP
                    revision N
                        |
          +-------------+-------------+
          |             |             |
        Gazebo          Nav2          Web
          |             |             |
          +-------------+-------------+
                        |
                       ROS
                        |
             TF map -> base_footprint
                        |
                        v
                       Web
```
