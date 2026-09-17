# WareTwin Frontend-Only Showcase

This build is configured for frontend-only demo mode:

```env
VITE_DEMO_MODE=true
VITE_BACKEND_MODE=false
```

No FastAPI backend, login, JWT, or WebSocket is required to open the app.

The Warehouse Editor uses the bundled `src/layout/warehouse_layout.json` as its local fallback draft.
