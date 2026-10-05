# WareTwin Frontend-Only Showcase

This build is configured for frontend-only demo mode:

```env
VITE_DEMO_MODE=true
VITE_BACKEND_MODE=false
```

This frontend-only showcase is separate from the local Gazebo/ROS runtime
profile. It does not provide or require a user login.

The Warehouse Editor uses the bundled `src/layout/warehouse_layout.json` as its local fallback draft.
