# Django API connection

This frontend is configured for the Django backend in `django_backend_api_connected`.

## Contract

- REST base: `VITE_API_BASE_URL` (legacy `VITE_API_URL` is still accepted)
- WebSocket: `VITE_WS_BASE_URL` (legacy `VITE_WS_URL` is still accepted)
- Authentication: `Authorization: Bearer <token>` for REST; `?token=<token>` for WebSocket.
- Django is authoritative in backend mode. If WebSocket disconnects, the UI freezes at the last confirmed state and shows **OFFLINE**. It does not start the browser SimEngine.

## Development

Backend:

```bash
python manage.py migrate
python manage.py seed_demo
python manage.py sync_master_data
python manage.py runserver 0.0.0.0:8000
```

Frontend:

```bash
npm ci
npm run dev
```

By default the frontend derives the backend hostname from the browser and uses
`VITE_BACKEND_PORT=8000`. This works on the same PC and across a LAN when Django
listens on `0.0.0.0`. Set explicit base URLs only when using a reverse proxy,
HTTPS, or a backend on a different port/host.

## Connected functionality

Auth, admin users, health, AI status, events, simulation controls, scenario injection/clear, task create/assign, alert acknowledgement, FULL/PATCH/HEATMAP realtime state, Copilot, What-if, VLM observe, warehouse draft/load/save/publish.
