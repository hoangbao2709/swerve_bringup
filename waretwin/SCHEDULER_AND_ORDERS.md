# Robot Scheduler + Order Management

## Database-backed planning

The scheduler uses Django DB as the durable source for planning/business data:

- `WorkPoint`: robot-accessible warehouse points — shelves, conveyor IN/OUT, inbound, outbound, packing, sorting, chargers, docks and custom stations.
- `RobotProfile`: persistent fleet metadata/capabilities and dispatch limits.
- `WarehouseOrder`: business transport/order request.
- `RobotSchedule`: assigned robot, mode, planned/actual time, score and execution status.
- `ScheduleStop`: ordered route (`source -> via... -> destination`).
- `ResourceReservation`: time-window reservation for shared conveyor/dock/station/charger interfaces.
- Existing `EventLog`: runtime/audit events.

Live pose, battery and FSM are telemetry and remain in the runtime state; they are intentionally not written to the order tables every simulation tick.

## Scheduling flow

```text
Warehouse map in DB
      -> WorkPoints
Order -> ordered route
      -> validate shared-resource conflicts
      -> score eligible robots
      -> AUTO / SEMI_AUTO / MANUAL selection
      -> persist RobotSchedule + ScheduleStops + ResourceReservations
      -> wait until planned_start
      -> dispatch route leg to SimEngine
      -> A* moves the exact assigned simulated robot
      -> leg complete -> next scheduled work-point
      -> all route legs complete -> schedule/order COMPLETED
```

Random demo task generation/assignment is disabled while this persistent scheduler is authoritative. The old A* movement engine is retained only as the simulated robot executor, so the same schedule contract can later dispatch to Robot Server -> ROS2/Nav2.

## Order management

The scheduler modal stores orders in DB and supports:

- Create order
- Read/search/list orders
- Edit priority, due time, ERP/external reference and notes
- Cancel order (also cancels an active schedule/task)
- Delete an order when it has no active schedule
- Plan an existing order into a robot schedule

Important route-changing order data cannot be modified while an active schedule exists; cancel/re-plan first.

## Scheduling UI

`/` is the simplified **Operations Overview**. The original dense dashboard remains at `/operations`.

Click **Robot Schedule** on Overview or Operations Console. The modal contains:

- **Create schedule** — choose/create order, source/destination, optional intermediate work-points, planned start, AUTO/SEMI/MANUAL dispatch, validate and preview robot scores.
- **Orders** — persistent DB order management.
- **Timeline** — active schedules grouped by robot.

Preview also checks shared-resource reservations. A schedule cannot be created while its planned conveyor/dock/station/charger time overlaps a reserved window.

## Example planned route

```text
SHELF-A01 -> CV01-IN -> CV01-OUT -> OUTBOUND-1
```

The route is saved as ordered `ScheduleStop` rows. During execution, the simulation dispatches those legs in the same order and the robot path is visible through the existing path overlay.
