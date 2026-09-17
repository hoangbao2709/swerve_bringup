# SQLite / WebSocket concurrency fix

Root cause fixed:

- Scheduler GET endpoints previously called `ensure_scheduler_master_data()`.
- That function synchronized ~389 work-points and 20 robot profiles on **every GET**, turning parallel dashboard reads into concurrent SQLite writes.
- Under Daphne/ASGI this produced `sqlite3.OperationalError: database is locked`.

Changes:

1. `ensure_scheduler_master_data()` is now read-only.
2. Master data is synchronized only by `seed_demo`, warehouse/layout publish hooks, or `POST /api/scheduler/sync`.
3. Explicit sync skips unchanged WorkPoint/RobotProfile rows.
4. SQLite uses a 30s busy timeout and `seed_demo` enables WAL mode.
5. Frontend does not open `/ws` without an auth token.
6. Admin no longer creates a second competing WebSocket; App owns one global connection.

After replacing the project run:

```powershell
cd backend
python manage.py migrate
python manage.py seed_demo
python manage.py runserver 0.0.0.0:8000
```

Expected seed output includes:

```text
SQLite configured: WAL + busy_timeout=30000ms
```

Then start frontend normally. A pre-login `/api/auth/me 401` is expected when there is no stored session. After login there should be a single authenticated `WebSocket CONNECT /ws` and scheduler GET endpoints should return 200 without writing master data.
