# Live Map Shelf Selection

On `/`, the live warehouse map now supports physical shelf/rack inspection.

## 3D view

1. Select the arrow **Select robot / shelf** tool.
2. Click any shelf/rack.
3. The selected rack is highlighted in cyan.
4. A floating card shows:
   - Shelf/rack ID
   - Zone
   - Floor
   - X, Y, Z position in metres
5. Click the `×` button or empty 3D space to clear the shelf selection.

## 2D map

Click a shelf footprint in **MAP VIEW**, **TRAFFIC VIEW**, or **HEATMAP**. The same information card is shown and the selected footprint is highlighted.

## Coordinates

The displayed business coordinates use the same convention as Warehouse Data:

- `X = layout.position[0]`
- `Y = layout.position[2]` (warehouse map depth)
- `Z = layout.position[1]` (elevation)

The selection reads the current database-backed live layout, so it follows Warehouse/Editor map synchronization.
