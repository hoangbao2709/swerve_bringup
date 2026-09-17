# 3D Label Layer Controls

The 3D viewport gear button now opens label-layer settings.

Layers can be enabled/disabled independently:
- Zone labels
- Dock / Station labels (INBOUND, OUTBOUND, PACKING)
- Lift labels
- Shelf load (%) labels
- Robot labels
- Human warning labels

Each layer has a user-facing Z value from 1 to 9:
- 1 = farther behind other labels
- 9 = in front of other scene labels

Scene labels are kept below normal UI overlays. Shelf Inventory, viewport controls, tabs and the bottom toolbar therefore remain clickable and visible even when labels overlap their screen area.

Settings for individual layers are saved in browser localStorage under `waretwin.label-layers.v1`.
The existing label button in the bottom 3D toolbar remains a master show/hide switch.
