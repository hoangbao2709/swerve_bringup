# WareTwin Frontend Demo Mode

The frontend defaults to `VITE_DEMO_MODE=true`. In this mode authentication and backend availability are not required to render the console or `/admin/*` routes, including `/admin/warehouse-editor`.

Run with:

```powershell
npm install
npm run dev
```

For authenticated backend deployment set:

```env
VITE_DEMO_MODE=false
```
