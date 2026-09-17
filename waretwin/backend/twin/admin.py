from django.contrib import admin
from .models import (
    EventLog, RobotEndpoint, Mission, Warehouse, Zone, Shelf, WarehouseMap, WarehouseMapVersion,
    WorkPoint, RobotProfile, WarehouseOrder, ShelfInventoryItem, RobotSchedule, ScheduleStop, ResourceReservation,
)

for model in (
    EventLog, RobotEndpoint, Mission, Warehouse, Zone, Shelf, WarehouseMap, WarehouseMapVersion,
    WorkPoint, RobotProfile, WarehouseOrder, ShelfInventoryItem, RobotSchedule, ScheduleStop, ResourceReservation,
):
    admin.site.register(model)
