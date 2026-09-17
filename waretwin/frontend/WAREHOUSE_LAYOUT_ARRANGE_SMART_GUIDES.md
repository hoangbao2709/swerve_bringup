# WareTwin Warehouse Editor — Arrange & Smart Guides

Implemented after Layers + Blocks:

- Align Left / Center X / Right
- Align Top / Center Z / Bottom
- Distribute Horizontal / Vertical (center based)
- Smart Guides toggle
- Smart guide snapping to nearby object centers and matching edges
- Visual dashed X/Z guide lines while dragging
- Locked/hidden layers excluded from guide targets
- Existing block, layer, overlap cycling and CAD mouse UX retained

## UX

- LMB: select/move
- MMB drag: pan
- Wheel: zoom at cursor
- Alt + LMB: cycle overlapped objects
- Arrange: align/distribute selected objects
- Guides ON: snap while dragging

The features are editor metadata/interaction behavior and do not alter the backend `WarehouseLayout` schema.
