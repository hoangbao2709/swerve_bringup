# Django API connection

This frontend is configured for the Django backend in `django_backend_api_connected`.

## Contract

- REST base: `VITE_API_URL`
- WebSocket: `VITE_WS_URL`
- Authentication: `Authorization: Bearer <token>` for REST; `?token=<token>` for WebSocket.
- Django is authoritative in backend mode. If WebSocket disconnects, the UI freezes at the last confirmed state and shows **OFFLINE**. It does not start the browser SimEngine.

## Development

Backend:

```bash
python manage.py migrate
python manage.py seed_demo
python manage.py runserver 0.0.0.0:8000
```

Frontend:

```bash
npm ci
npm run dev
```

Default backend URLs are `http://127.0.0.1:8000` and `ws://127.0.0.1:8000/ws`. If the browser and Django are on different machines, change both values to the Django PC IP.

## Connected functionality

Auth, admin users, health, AI status, events, simulation controls, scenario injection/clear, task create/assign, alert acknowledgement, FULL/PATCH/HEATMAP realtime state, Copilot, What-if, VLM observe, warehouse draft/load/save/publish.
