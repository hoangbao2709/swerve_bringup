# WareTwin Django API-connected build

## 1. Start Django backend

```bash
cd backend
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python manage.py migrate
python manage.py sync_master_data
python manage.py runserver 127.0.0.1:8000
```

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
npm run dev -- --host 127.0.0.1
```

Open `http://127.0.0.1:5173/`. The local application has no login step.

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

The retained frontend routes are `/`, `/control`, and
`/robots/:robotId/control`. Legacy admin and product pages are unavailable and
unknown routes return to `/`.

## Synchronized warehouse maps (new)

After updating to this build, run the new database migration:

```bash
cd backend
source .venv/bin/activate
python manage.py migrate
python manage.py sync_master_data
python manage.py runserver 127.0.0.1:8000
```

Then start the frontend and open `http://127.0.0.1:5173/` for the live map.


## Robot Scheduler + Orders

After this build, run migration `0004_scheduler_orders_workpoints`:

```bash
cd backend
source .venv/bin/activate
python manage.py migrate
python manage.py sync_master_data
python manage.py runserver 127.0.0.1:8000
```

The Overview is available at `/`; Robot Control and robot details remain at
`/control` and `/robots/:robotId/control`.

All order, route, assignment, reservation and schedule data is stored in Django DB. The mock robots execute scheduled route legs through the existing A* simulation engine.

### Scheduler verification on your machine

After installing dependencies and migrating, run:

```bash
python manage.py test twin.tests.test_scheduler
```

The scheduler stores orders, schedules, ordered stops and resource reservations in Django DB.
