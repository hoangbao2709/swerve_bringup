# Canonical Warehouse Map V2

Warehouse geometry is expressed in metres in the ROS/Gazebo `map` frame. X and
Y are the floor plane, Z is elevation, and yaw is CCW radians. A floor is an
arbitrary simple polygon with optional holes; it is not required to be a
rectangle. Aisles use a centreline and width, while navigation tags and edges
are explicit graph data.

Legacy layouts containing only `size.width` and `size.depth` are normalized to
the rectangle `(0,0) -> (width,0) -> (width,depth) -> (0,depth)` without
destructively changing their stored source document. Shared coordinate helpers
are in `frontend/src/layout/coordinates.ts` and `backend/twin/canonical_map.py`.

Published runtime artifacts are immutable revision bundles. Nav2 occupancy,
Gazebo geometry, DataMatrix/tag data and graph exports are generated from that
same normalized layout. A package Nav2 map or Gazebo world is development-only
and requires an explicit allow-development option.

The Three.js adapter maps warehouse `(x,y,z)` to Three `(x,z,y)`. ROS keeps the
canonical axis order `(x,y,z)`. Components must use these adapters instead of
swapping axes locally.
