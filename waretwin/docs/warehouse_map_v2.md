# Canonical Warehouse Map V2

Warehouse geometry is expressed in metres in the `warehouse_map` frame. X and
Y are the floor plane, Z is elevation, and yaw is radians. A floor is an
arbitrary simple polygon with optional holes; it is not required to be a
rectangle. Aisles use a centreline and width, while navigation tags and edges
are explicit graph data.

Legacy layouts containing only `size.width` and `size.depth` are normalized to
the rectangle `(0,0) -> (width,0) -> (width,depth) -> (0,depth)` without
destructively changing their stored source document. Shared coordinate helpers
are in `frontend/src/layout/coordinates.ts` and `backend/twin/canonical_map.py`.

The Three.js adapter maps warehouse `(x,y,z)` to Three `(x,z,y)`. ROS keeps the
canonical axis order `(x,y,z)`. Components must use these adapters instead of
swapping axes locally.
