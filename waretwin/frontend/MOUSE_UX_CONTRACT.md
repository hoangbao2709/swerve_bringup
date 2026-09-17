# WareTwin Warehouse Editor — CAD Mouse UX Contract

The Warehouse Editor follows a CAD-like mouse contract:

- Hover = preselection/highlight only; never changes layout data.
- Left click = select. Shift/Ctrl/Cmd + click = additive/toggle selection.
- Left drag = edit the selected object (move/resize/rotate depending on tool).
- Empty-canvas left drag = selection marquee in 2D.
- Middle mouse drag = pan in 2D and 3D.
- Mouse wheel = zoom; in 2D the zoom is centered on the cursor position.
- Right click = context menu in 2D.
- In 3D, right-drag orbits the camera while middle-drag pans.
- Esc cancels the current interaction/mode and clears transient state.
- Delete removes the current selection; Ctrl/Cmd+Z and Ctrl/Cmd+Y undo/redo; Ctrl/Cmd+D duplicates.
- Selection state and edit state are distinct from hover/preselection state.
- 2D and 3D share selection/edit semantics even though camera navigation differs.
