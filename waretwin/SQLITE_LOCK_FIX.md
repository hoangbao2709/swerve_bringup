# SQLite / WebSocket concurrency fix

Root cause fixed:

- Scheduler GET endpoints previously called `ensure_scheduler_master_data()`.
- That function synchronized ~389 work-points and 20 robot profiles on **every GET**, turning parallel dashboard reads into concurrent SQLite writes.
- Under Daphne/ASGI this produced `sqlite3.OperationalError: database is locked`.

Changes:

1. `ensure_scheduler_master_data()` is now read-only.
2. Master data is synchronized only by `sync_master_data`, warehouse/layout publish hooks, or `POST /api/scheduler/sync`.
3. Explicit sync skips unchanged WorkPoint/RobotProfile rows.
4. SQLite uses a 30s busy timeout and `sync_master_data` enables WAL mode.
5. The local frontend opens `/ws` directly without a browser identity or token.
6. App owns one global WebSocket connection.

After replacing the project run:

```powershell
cd backend
python manage.py migrate
python manage.py sync_master_data
python manage.py runserver 127.0.0.1:8000
```

Expected sync output includes:

```text
SQLite configured: WAL + busy_timeout=30000ms
```

Then start the frontend normally. The local frontend opens `/` directly and uses
one unauthenticated browser WebSocket at `/ws`; scheduler GET endpoints should
return 200 without writing master data.
