# Shelf occupancy visualization

- Every physical shelf/rack has a fixed capacity of **8 orders**.
- **1 order = 12.5%** occupancy.
- The initial demo load is intentionally light and uses only **0, 1, 2, 3** orders per shelf.
- 2D map: every shelf renders its own horizontal occupancy bar and percentage.
- 3D map: every shelf renders exactly `current_load` boxes and a percentage label above the shelf.
- `Shelf.current_load` is the persisted source of truth. The `/api/layout` response projects it into each rack as `current_load`.

Initial starter pattern for shelves that do not yet have saved occupancy:

`0, 1, 0, 2, 0, 1, 0, 3, ...`

This keeps the first view sparse instead of filling the warehouse with boxes.
