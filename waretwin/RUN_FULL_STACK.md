# WareTwin Django API-connected build

## 1. Start Django backend

```bash
cd backend
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python manage.py migrate
python manage.py seed_demo
python manage.py runserver 0.0.0.0:8000
```

Default development account:

- username: `admin`
- password: `admin12345`

## 2. Configure frontend

For frontend and Django on the same PC, keep:

```env
VITE_DEMO_MODE=false
VITE_BACKEND_MODE=true
VITE_BACKEND_PORT=8000
VITE_API_BASE_URL=
VITE_WS_BASE_URL=
```

The empty base URLs resolve the Django hostname from the browser automatically.
Set explicit `VITE_API_BASE_URL`/`VITE_WS_BASE_URL` for a reverse proxy or a
backend on another host/port.

Also add that frontend origin to backend `.env`:

```env
CORS_ALLOWED_ORIGINS=http://localhost:5173,http://127.0.0.1:5173,http://<FRONTEND_PC_IP>:5173
```

## 3. Start frontend

```bash
cd frontend
npm ci
npm run dev -- --host 0.0.0.0
```

Open `http://<FRONTEND_PC_IP>:5173`.

## Data flow

```text
React frontend
   | REST + WebSocket
   v
Django central backend
   | mock SimEngine for now
   v
TwinState
```

In backend mode, Django is authoritative. If WebSocket is lost, the UI displays OFFLINE and keeps the last confirmed state; it does not start a local simulation.

## Warehouse master data CRUD

After migration/login, open:

```text
http://127.0.0.1:5173/admin/warehouse
```

The page provides CRUD for Warehouse -> Zone -> Shelf, search/filter, protected delete, position/access-point fields and a preview. Use **Sync published layout** to import/update the current warehouse geometry into Django master data.

If upgrading an existing database, run `python manage.py migrate` before starting the backend.

## Synchronized warehouse maps (new)

After updating to this build, run the new database migration:

```bash
cd backend
source .venv/bin/activate
python manage.py migrate
python manage.py seed_demo
python manage.py runserver 0.0.0.0:8000
```

Then start the frontend as usual and use:

- `http://localhost:5173/` — live map
- `http://localhost:5173/admin/warehouse` — Warehouse/Zone/Shelf data
- `http://localhost:5173/admin/warehouse-editor` — map editor

All three pages use the same database-backed active WarehouseMap. Use **Use on live map** in Warehouse Data to switch the active warehouse.


## Robot Scheduler + Orders

After this build, run migration `0004_scheduler_orders_workpoints`:

```bash
cd backend
source .venv/bin/activate
python manage.py migrate
python manage.py seed_demo
python manage.py runserver 0.0.0.0:8000
```

Frontend pages:

- `http://localhost:5173/` — simplified Overview dashboard
- `http://localhost:5173/operations` — detailed legacy operations console
- **Robot Schedule** — persistent orders/schedules modal
- `/admin/warehouse` and `/admin/warehouse-editor` — map/master-data management

All order, route, assignment, reservation and schedule data is stored in Django DB. The mock robots execute scheduled route legs through the existing A* simulation engine.

### Scheduler verification on your machine

After installing dependencies and migrating, run:

```bash
python manage.py test twin.tests.test_scheduler
```

The scheduler stores orders, schedules, ordered stops and resource reservations in Django DB. `/` is the simplified overview; `/operations` keeps the full diagnostic console.
