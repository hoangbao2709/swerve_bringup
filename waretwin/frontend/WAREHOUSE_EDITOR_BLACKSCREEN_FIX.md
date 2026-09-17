# Warehouse Editor black-screen fix

The initial render was crashing because `WarehouseLayout.locations` are point-like records using `access_point: [x, z]`, not `rect` or `position`. The 2D editor included locations in the render list, but `get2DBox("location", obj)` fell through to `obj.rect`, causing a runtime TypeError during render. The fix adds an explicit location case based on `access_point` and updates location movement to edit `access_point`.
