# Warehouse map synchronization

## Single source of truth

The synchronized warehouse geometry is stored in Django database table `twin_warehousemap`.

Each `Warehouse` owns exactly one `WarehouseMap`:

- `layout`: current authoritative geometry used by `/` and the simulation runtime.
- `draft`: latest editor copy (kept together with current layout in this implementation).
- `revision`: increments on every synchronized change.
- `published_version`: increments only when the editor publishes.
- `is_active`: identifies which warehouse is displayed by the live console `/`.

Published history is stored in `WarehouseMapVersion`.

## Three synchronized pages

- `/admin/warehouse` — relational master data: Warehouse -> Zone -> Shelf.
- `/admin/warehouse-editor` — CAD-like map geometry editor.
- `/` — live 2D/3D map.

### Warehouse Data -> Editor / Live Map

Create/update/delete Warehouse, Zone or Shelf -> Django updates master tables -> rebuilds the same WarehouseMap -> increments revision -> emits `LAYOUT_UPDATED` -> all connected pages refresh.

### Editor -> Warehouse Data / Live Map

`Save & Sync` or `Publish` -> validates geometry -> saves WarehouseMap -> synchronizes Warehouse/Zone/Shelf -> increments revision -> emits `LAYOUT_UPDATED` -> all connected pages refresh.

`Publish` additionally creates a `WarehouseMapVersion` record.

## Multiple warehouses

Every warehouse has its own independent map. Only one can be `is_active=true`.

In `/admin/warehouse`, select a warehouse and click **Use on live map**. The `/` page and `/admin/warehouse-editor` then switch to that warehouse map.

## Concurrency protection

The editor sends `X-Layout-Revision` on Save/Publish. If another session already changed the map, Django returns HTTP 409 instead of overwriting a newer revision.

## Geometry consistency

The same map stores meters for `position`, `size`, and degrees for rack `rotation`.

- Editor renders these values.
- Live 2D map renders the exact rotated rack footprint.
- Live 3D rack renderer applies the same rotation.
- Frontend and backend navigation grids use the same rotated bounding-box rule for collision blocking.

## Safety behavior

A live map geometry change resets/pauses the mock simulation runtime. This prevents robots in the simulator from continuing on a navigation grid that no longer matches the warehouse geometry.
