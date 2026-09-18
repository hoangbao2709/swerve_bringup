# WareTwin warehouse map pipeline

This document describes the authoritative map path from the editor to ROS 2 and
Gazebo.  It covers PART 1–9 and intentionally does not add a live-reload
protocol beyond the revision acknowledgement described below.

## Architecture

The editor owns a draft canonical Map V2 document.  Floors and holes constrain
aisles, tags and the polygon navgrid.  Navigation graph edges are generated from
aisle geometry, then a successful publish snapshots an immutable
`WarehouseMapVersion` and creates revision artifacts:

```text
editor draft → validate → publish revision
    → canonical_map.json / datamatrix_map.yaml / tag_graph.yaml
    → gazebo/warehouse.world
    → WebSocket map.published → ROS bridge ACK
```

The database is authoritative for the published version.  Draft edits never
change an older `WarehouseMapVersion` or its artifact directory.

## Canonical Map V2

Canonical coordinates are always warehouse metres:

* `X/Y` are the floor plane;
* `Z` is height/elevation;
* `yaw` is radians;
* frame is `warehouse_map`;
* units are `meter`.

The frontend may render the floor plane as `x/z` in Three.js, but that is only a
view conversion.  `tools/export_gazebo_world.py` is the single Gazebo coordinate
conversion point.  Floor IDs are `number | string`; comparisons use canonical
identity, so `F1`/`F2` work without numeric casts.

## Floor editor, aisles and tags

Floors contain a `boundary` polygon and optional `holes`.  Aisles contain a
meter-space `centerline`, width and direction (`bidirectional`, `forward`, or
`reverse`).  Navigation tags have immutable `uuid` identity and editable
physical `tag_id`.  Dragging or changing physical placement makes an automatic
tag a locked manual override.  Intersections are shared tags, not duplicates.

## Navigation graph and NavGrid

Adjacent tags on each aisle become graph edges.  Direction is represented by
actual directed edges; bidirectional edges are traversable both ways.  Edge
distance follows the aisle polyline.  The navgrid checks each cell centre
against the floor polygon and every hole, then inflates boundary and physical
objects by `robot_radius + safety_margin`.  Aisles are semantic corridors, not
obstacles.

## Publish revision and artifacts

The admin editor provides Validate, Save & Sync, and Publish.  Publish validates
the complete canonical document, builds artifacts in a temporary directory,
verifies them, then atomically commits the database version and renames the
directory:

```text
generated/maps/<warehouse-code>/<revision>/
  canonical_map.json
  datamatrix_map.yaml
  tag_graph.yaml
  manifest.json
  gazebo/warehouse.world
```

`manifest.json` contains the map revision, published version, relative artifact
paths and SHA-256 hashes.  A failed exporter or database transaction cleans the
temporary directory and leaves the previous published version untouched.

## Gazebo export

The exporter creates deterministic polygon meshes, boundary/hole walls,
physical object collisions, and navigation tag markers.  It is called by the
publish service; it is not duplicated in the ROS bridge.

## ROS config export and revision sync

The exported DataMatrix YAML retains the legacy `markers` list and adds rich
tag metadata.  `tag_graph.yaml` retains the ROS consumer's `tags`/`neighbors`
mapping and also carries canonical edge metadata.

After publish, Channels sends `map.published` to web clients and
`MAP_PUBLISHED` to the authenticated bridge.  The bridge reads the revision's
DataMatrix map, tag graph and Gazebo world, then sends `MAP_REVISION_ACK`.
The backend exposes the same state at `/api/map/sync-status`:

```json
{
  "published_revision": 12,
  "ros_revision": 12,
  "gazebo_revision": 12,
  "status": "SYNCED"
}
```

Possible external-runtime states are `SYNCED`, `OUT_OF_SYNC`, `ROS_OFFLINE`,
`ERROR`, and `NO_PUBLISHED_MAP`.  Reconnecting a stale bridge causes the
published artifact bundle to be sent again.  Navigation missions are rejected
while revisions differ; Emergency Stop remains available.

## How to run

Start the backend:

```bash
cd waretwin/backend
python manage.py runserver 0.0.0.0:8000
```

Start the frontend:

```bash
cd waretwin/frontend
npm run dev -- --host 0.0.0.0
```

Start the ROS bridge with the shared artifact root (the default is
`generated/maps` relative to the bridge process):

```bash
cd ~/swerve_bringup
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch swerve_bridge bridge.launch.py \
  django_ws_url:=ws://127.0.0.1:8000/ws/ros \
  artifact_root:=/home/yahboom/swerve_bringup/generated/maps
```

After publishing, launch the selected world (the path is the immutable revision
artifact, not the editor draft):

```bash
ros2 launch swerve_bringup gazebo.launch.py \
  world:=/home/yahboom/swerve_bringup/generated/maps/<warehouse-code>/<revision>/gazebo/warehouse.world
```

For the V30E/tag-route simulation, pass the same immutable revision artifacts
explicitly.  Package `config/` files remain development fallbacks:

```bash
ros2 launch swerve_bringup v30e_sim.launch.py \
  enable_v30e_sim:=true \
  datamatrix_map_file:=/home/yahboom/swerve_bringup/generated/maps/<warehouse-code>/<revision>/datamatrix_map.yaml \
  tag_graph_file:=/home/yahboom/swerve_bringup/generated/maps/<warehouse-code>/<revision>/tag_graph.yaml
```

## How to test

Backend and exporter:

```bash
cd waretwin/backend
python manage.py check
python manage.py test
cd ../..
python -m unittest tools.test_export_gazebo_world -v
python -m unittest tools.test_warehouse_pipeline -v
```

Frontend:

```bash
cd waretwin/frontend
./node_modules/.bin/tsc --noEmit
node node_modules/vitest/vitest.mjs run
./node_modules/.bin/vite build
```

ROS bridge and generated world:

```bash
cd ~/swerve_bringup
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m py_compile swerve_bridge/swerve_bridge/bridge_node.py
gz sdf -k generated/maps/<warehouse-code>/<revision>/gazebo/warehouse.world
```

## Troubleshooting

* `409 Conflict` on Save/Publish means `X-Layout-Revision` is stale; reload the
  draft and retry.
* `OUT_OF_SYNC` means the bridge/Gazebo revision differs from the published
  revision.  Keep the bridge connected and let it receive `MAP_PUBLISHED`, or
  reconnect it with the same artifact root.
* `ROS_OFFLINE` means the authenticated `/ws/ros` bridge is not connected.
* An exporter validation error is intentionally atomic: no new DB version is
  published and no incomplete revision directory is retained.
* Static files in `config/` and `worlds/` remain development/sample fallbacks;
  a published revision's generated files take precedence in production.
