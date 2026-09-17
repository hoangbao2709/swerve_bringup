# Inbound / Outbound Management

Dashboard now has an **Inbound / Outbound** modal backed by `WarehouseOrder`.

- INBOUND route: `INBOUND dock -> physical shelf`
- OUTBOUND route: `physical shelf -> OUTBOUND dock`
- One completed INBOUND order adds one shelf slot (`+1/8`).
- One completed OUTBOUND order removes one shelf slot (`-1/8`).
- Active flow orders reserve shelf slots so the UI/API cannot overbook inbound or outbound work.
- Shelf occupancy changes broadcast `LAYOUT_UPDATED`, so the 2D/3D map refreshes from `/api/layout`.
- Dock work-points preserve `INBOUND` / `OUTBOUND` kinds when scheduler master data is synchronized.

After updating an existing database, run once:

```bash
python manage.py migrate
python manage.py seed_demo
```

On Windows PowerShell, activate `.venv` first and run the same Django commands with `python`.
