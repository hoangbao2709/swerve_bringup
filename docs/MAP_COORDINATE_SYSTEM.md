# Map coordinate system

## Canonical convention

The warehouse map is a right-handed metric frame named `warehouse_map`:

```text
X/Y = floor plane in metres
Z   = elevation in metres
yaw = counter-clockwise radians about +Z
```

The default Gazebo world, ROS map frame and WareTwin canonical layout share the
same origin and orientation. A map YAML follows the Nav2 convention: `origin`
is `[x, y, yaw]` in metres/radians, and `resolution` is metres per pixel.

## Web conversion

The editor stores world coordinates, never SVG/canvas pixels. For a viewport
with world origin `(ox, oy)` and scale `s` pixels/metre:

```text
screen_x = (world_x - ox) * s
screen_y = (world_y - oy) * s
world_x  = screen_x / s + ox
world_y  = screen_y / s + oy
```

The renderer may invert screen Y for a particular canvas, but that inversion is
explicit and must not mutate the stored object. Zoom, pan and resize only change
`ox`, `oy` or `s`; they do not change object coordinates. Grid snapping is in
metres before rendering.

## Gazebo and ROS conversion

Gazebo export consumes the canonical document and emits the same X/Y pose,
orientation and object dimensions in an SDF world. There is no frontend robot
animation: the browser renders the `ROBOT_STATE` pose received from ROS.

The exporter is the single place for mesh/SDF details. `manifest.json` carries
revision and SHA-256 hashes so ROS, Gazebo and Web can verify they are using the
same published version.

## Objects

Every floor, wall, shelf, station, conveyor, zone, lane, waypoint and tag has a
stable ID and world pose. Tags additionally carry family, physical ID, size,
`z`, yaw, lane/zone linkage and metadata. Lane/QR/waypoint spacing is measured
along a metric centerline, so a 20 m lane with 2 m spacing produces positions
approximately 2, 4, …, 18 m from the chosen endpoint—not pixel-derived values.

## Validation checklist

1. Validate floor boundary and holes in world coordinates.
2. Check all object positions are finite metres and all yaw values are radians.
3. Validate lane widths, graph edges and tag spacing.
4. Publish one immutable revision.
5. Compare Web layout, generated `canonical_map.json`, Gazebo world and ROS
   `MAP_REVISION_STATUS` before sending a mission.

