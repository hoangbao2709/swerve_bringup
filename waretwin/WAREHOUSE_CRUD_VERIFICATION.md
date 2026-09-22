# Warehouse CRUD verification

Checks completed in the build environment:

- Python source compile: PASS (`python -m compileall backend`)
- TypeScript/TSX syntax transpilation: PASS for all 42 `src/**/*.ts(x)` files
- Frontend warehouse API paths matched to Django URL routes: PASS
- Admin route `/admin/warehouse`: PASS
- Backend/frontend bundled warehouse layout SHA-256 match: PASS
- Bundled layout sync target: 1 warehouse / 5 zones / 184 physical shelf-rack records
- Django migration `0002_warehouse_zone_shelf.py`: included
- CRUD test source: `backend/twin/tests/test_warehouse_crud.py`

The sandbox cannot install Django/npm dependencies because external package resolution is unavailable, so the actual Django test runner and Vite production build could not be executed here. On the target machine, run:

```bash
cd backend
python manage.py migrate
python manage.py test twin.tests.test_warehouse_crud
python manage.py seed_demo
python manage.py sync_master_data

cd ../frontend
npm ci
npm run build
```
