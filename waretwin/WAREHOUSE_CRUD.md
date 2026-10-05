# Warehouse / Zone / Shelf CRUD

## Data model

- Warehouse has many Zones.
- Zone belongs to one Warehouse and has many Shelves.
- Shelf belongs to one Zone.
- Parent deletion is protected while children exist.

A Shelf stores physical geometry (`position`, `size`, `rotation`, `levels`, capacity/load/status) and a robot navigation access point (`access_x`, `access_y`, `access_yaw`).

## Frontend

The legacy warehouse administration UI is not available in the retained local
application; active warehouse geometry remains visible on `/`.

Functions:

- create/edit/delete Warehouse
- create/edit/delete Zone
- create/edit/delete Shelf
- Warehouse -> Zone tree navigation
- Shelf search and status filtering
- capacity/load/status view
- position and access-point preview
- sync current published layout into Django master data

## Backend APIs

```text
GET    /api/warehouses
POST   /api/warehouses
GET    /api/warehouses/{id}
PATCH  /api/warehouses/{id}
DELETE /api/warehouses/{id}

GET    /api/zones?warehouse={id}
POST   /api/zones
GET    /api/zones/{id}
PATCH  /api/zones/{id}
DELETE /api/zones/{id}

GET    /api/shelves?warehouse={id}&zone={id}&search=...&status=...
POST   /api/shelves
GET    /api/shelves/{id}
PATCH  /api/shelves/{id}
DELETE /api/shelves/{id}

GET    /api/warehouse-tree
POST   /api/warehouse-sync/from-layout
```

The legacy administration APIs are not part of the retained local application
surface. Browser user accounts and role checks are not used by the runtime.

## Upgrade / run

```bash
cd backend
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python manage.py migrate
python manage.py sync_master_data
python manage.py runserver 127.0.0.1:8000
```

Then run the frontend and open `/` to view the active map.
