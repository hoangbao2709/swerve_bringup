# Inbound / Outbound file import + automatic robot scheduling

Dashboard -> **Inbound / Outbound** accepts one file per import: either INBOUND or OUTBOUND.
Supported formats: `.xlsx` and `.json`.

## Columns

Use these columns in Excel (first row = headers), or keys in JSON:

- `order_no`: business order number. Re-uploading the same value is skipped as a duplicate.
- `external_ref`: PO / ASN / SO / customer reference.
- `item_code`, `item_name`: goods identity saved into order metadata.
- `quantity`: business quantity inside the order. One imported row still occupies exactly one shelf order slot.
- `priority`: `LOW`, `NORMAL`, `HIGH`, `CRITICAL`.
- `shelf_code`: rack/shelf location. May be empty for INBOUND; required for OUTBOUND.
- `dock_code`: inbound/outbound dock. If empty, the first configured dock of the matching flow is used.
- `payload_weight_kg`: payload weight.
- `due_at`: ISO datetime or Excel datetime cell.
- `notes`: handling instructions.
- `load_units`: optional robot load units, 1..4, default 1.

Aliases such as `sku`, `rack_id`, `location`, `qty`, `source_code`, and `destination_code` are also accepted.

## Flow rules

### INBOUND

`INBOUND dock -> SHELF`

When `shelf_code` is present, that shelf is used. When it is empty, the backend automatically chooses an enabled shelf with a free slot. Shelves have 8 order slots, so one completed inbound order adds `1/8 = 12.5%`.

### OUTBOUND

`SHELF -> OUTBOUND dock`

`shelf_code` is required because it tells the robot where the goods currently are. The import is rejected if the shelf has no unreserved goods. One completed outbound order removes one shelf slot (`-12.5%`).

## Automatic scheduling

Use the flows sequentially: **INBOUND first**, wait until the inbound schedules complete and the items are physically stored on their shelves, then import **OUTBOUND**. This is required so outbound can reserve the exact `ShelfInventoryItem`/SKU on the supplied shelf.

After valid rows are created, the backend schedules them by:

1. priority (`CRITICAL` -> `HIGH` -> `NORMAL` -> `LOW`),
2. earliest due time,
3. file row order.

The scheduler chooses enabled/online robots that satisfy battery and payload constraints. Future jobs are placed after each robot's existing planned work. Shared dock/resource conflicts are shifted forward automatically.

If a row is valid but no robot is currently eligible, the order remains in the database as `NEW` and is returned as `UNSCHEDULED` instead of being lost.

## JSON

Either an array:

```json
[
  {"order_no":"IN-001","item_code":"SKU-1","shelf_code":"RACK-A101"}
]
```

or:

```json
{
  "orders": [
    {"order_no":"IN-001","item_code":"SKU-1","shelf_code":"RACK-A101"}
  ]
}
```

See `samples/inbound_orders.json` and `samples/outbound_orders.json`.

## API

`POST /api/orders/import` as `multipart/form-data` with:

- `inbound_file` = `.xlsx` or `.json`
- `outbound_file` = `.xlsx` or `.json`

Exactly one file is required per request. Sending both together is rejected because outbound goods do not exist as stored inventory until the inbound schedules complete.
