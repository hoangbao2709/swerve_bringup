# Shelf item operations

Each physical shelf has 8 slots. `Shelf.current_load` is the fast 2D/3D occupancy counter; `ShelfInventoryItem` stores the identity of each individual order/item on that shelf.

## Dashboard flow

Click a shelf in 2D or 3D. The inventory panel lists the individual items currently on that shelf. Select one `READY` item and choose either:

- **OUTBOUND**: creates an OUTBOUND order from the current shelf to the nearest enabled outbound dock.
- **TRANSFER SHELF**: choose any enabled shelf with a free slot; creates a TRANSFER order from the current shelf to that shelf.

Both actions create a `WarehouseOrder`, reserve the exact inventory item, create a `RobotSchedule`, and automatically select a robot. A reserved item cannot be dispatched a second time.

The **Ưu tiên robot không chở hàng** toggle excludes robots that currently carry goods or are in `TRANSPORTING` / `DELIVERING`. Truly idle unloaded robots rank first. Robots still travelling to/picking another order are allowed when they have not picked up cargo yet; the new schedule is queued after their current planned work instead of interrupting it.

When the schedule completes, the backend atomically updates the exact item location and shelf counters. OUTBOUND removes the item from storage; TRANSFER decrements the source shelf and increments the destination shelf. A `LAYOUT_UPDATED` event then refreshes the 2D loading bar and 3D box count.

## API

- `GET /api/shelf-inventory/<rack_id>` — shelf details, exact inventory items, outbound docks, and candidate destination shelves.
- `POST /api/inventory-items/<item_id>/dispatch`

Example transfer body:

```json
{
  "action": "TRANSFER",
  "destination_shelf_code": "RACK-B204",
  "priority": "NORMAL",
  "prefer_unloaded_robot": true
}
```

Example outbound body:

```json
{
  "action": "OUTBOUND",
  "priority": "HIGH",
  "prefer_unloaded_robot": true
}
```
