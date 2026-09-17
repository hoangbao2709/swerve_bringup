import { apiFetch } from "./api";

export type WorkPointKind = "SHELF" | "CONVEYOR_IN" | "CONVEYOR_OUT" | "INBOUND" | "OUTBOUND" | "PACKING" | "SORTING" | "BUFFER" | "CHARGING" | "DOCK" | "STATION" | "PARKING" | "CUSTOM";
export type WorkPoint = {
  id: number; warehouse_id: number; zone_id: number | null; code: string; name: string;
  kind: WorkPointKind; floor: number; x: number; y: number; z: number; yaw: number;
  resource_type: string; resource_id: string; enabled: boolean; capacity: number; metadata: Record<string, unknown>;
};
export type ManagedRobot = {
  id: number; warehouse_id: number; robot_id: string; name: string; enabled: boolean;
  payload_capacity: number; min_dispatch_battery: number; capabilities: string[];
  telemetry: {
    status: string; fsm: string; battery: number; floor: number; position: number[]; zone: string | null; current_task_id: string | null;
    load?: { current: number; capacity: number }; carrying_load?: boolean;
  };
};
export type WarehouseOrder = {
  id: number; order_no: string; external_ref: string; warehouse_id: number; type: string; priority: string; status: string;
  source: WorkPoint; destination: WorkPoint; quantity: number; load_units: number; payload_weight_kg: number;
  due_at: string | null; notes: string; metadata: Record<string, unknown>; created_at: string; updated_at: string;
};
export type ScheduleStop = { id: number; sequence: number; action: string; service_seconds: number; status: string; workpoint: WorkPoint; arrived_at: string | null; completed_at: string | null };
export type RobotSchedule = {
  id: number; schedule_id: string; warehouse_id: number; order_id: number; order_no: string; priority: string; robot_id: string;
  mode: string; status: string; planned_start: string; planned_end: string | null; actual_start: string | null; actual_end: string | null;
  estimated_distance_m: number; estimated_duration_s: number; score: Record<string, unknown>; current_leg: number; engine_task_id: string; failure_reason: string;
  created_at: string; stops?: ScheduleStop[];
};
export type ScheduleCandidate = { robot_id: string; robot_profile_id: number; score: number; distance_m: number; duration_s: number; battery: number; fsm: string; eligible: boolean; rejected_reason: string | null; reasons: string[] };
export type ResourceConflict = { workpoint: string; resource_type: string; resource_id: string; starts_at: string; ends_at: string; conflicting_schedule: string };
export type SchedulePreview = { order: WarehouseOrder; planned_start: string; route: WorkPoint[]; candidates: ScheduleCandidate[]; recommended: ScheduleCandidate | null; resource_conflicts: ResourceConflict[] };

export type OrderImportRowResult = {
  flow: "INBOUND" | "OUTBOUND"; filename: string; row: number;
  status: "SCHEDULED" | "UNSCHEDULED" | "CREATED" | "DUPLICATE" | "FAILED";
  order_id?: number; order_no?: string; source?: string; destination?: string; shelf_code?: string; item_code?: string;
  schedule_id?: string; robot_id?: string; planned_start?: string; planned_end?: string | null; message?: string;
};
export type OrderImportResult = {
  batch_id: string;
  summary: { files: number; rows: number; created: number; scheduled: number; unscheduled: number; duplicates: number; failed: number };
  results: OrderImportRowResult[];
};

export type ShelfInventoryItem = {
  id: number; item_uid: string; warehouse_id: number; shelf_id: number | null; shelf_code: string | null;
  status: "STORED" | "RESERVED" | "OUTBOUND"; item_code: string; item_name: string; external_ref: string;
  quantity: number; load_units: number; payload_weight_kg: number;
  origin_order_id: number | null; origin_order_no: string | null;
  last_movement_order_id: number | null; last_movement_order_no: string | null;
  reserved_by_order_id: number | null; reserved_by_order_no: string | null;
  metadata: Record<string, unknown>; stored_at: string | null; updated_at: string | null;
};
export type ShelfDestination = {
  shelf_id: number; shelf_code: string; zone: string; floor: number; current_load: number; reserved_in: number; available_slots: number; enabled: boolean;
};
export type ShelfInventoryResponse = {
  shelf: { id: number; code: string; rack_id: string; zone: string; floor: number; current_load: number; capacity: number; percent: number };
  items: ShelfInventoryItem[]; outbound_docks: WorkPoint[]; destination_shelves: ShelfDestination[];
};
export type InventoryDispatchResult = {
  ok: boolean; action: "OUTBOUND" | "TRANSFER"; prefer_unloaded_robot: boolean; item: ShelfInventoryItem;
  order: WarehouseOrder; schedule: RobotSchedule; selected_robot: ManagedRobot;
};

export type SchedulerOverview = {
  warehouse_id: number; workpoints: number; robots: number;
  orders: { total: number; new: number; planned: number; running: number; completed: number; failed: number };
  flows: {
    inbound: { active: number; running: number; completed: number };
    outbound: { active: number; running: number; completed: number };
  };
  schedules: { active: number; running: number; next: RobotSchedule[] };
};
export type ConveyorCommand = "START" | "STOP" | "MAINTENANCE" | "EMERGENCY_STOP" | "CLEAR_FAULT" | "FAULT" | "SET_SPEED" | "ENQUEUE" | "RELEASE_EXIT";

async function json<T>(response: Response): Promise<T> {
  const data = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error((data as { detail?: string }).detail || `${response.status} ${response.statusText}`);
  return data as T;
}
export const schedulerApi = {
  conveyors: () => apiFetch("/api/conveyors").then(json<Array<Record<string, unknown>>>),
  conveyorCommand: (id: string, action: ConveyorCommand, body: Record<string, unknown> = {}) => apiFetch(`/api/conveyors/${encodeURIComponent(id)}/command`, { method: "POST", body: JSON.stringify({ ...body, action }) }).then(json<Record<string, unknown>>),
  conveyorHandshake: (id: string, body: { robot_id: string; item_id: string; phase: string }) => apiFetch(`/api/conveyors/${encodeURIComponent(id)}/handshake`, { method: "POST", body: JSON.stringify(body) }).then(json<Record<string, unknown>>),
  overview: () => apiFetch("/api/scheduler/overview").then(json<SchedulerOverview>),
  sync: () => apiFetch("/api/scheduler/sync", { method: "POST" }).then(json<Record<string, unknown>>),
  workpoints: () => apiFetch("/api/scheduler/workpoints").then(json<WorkPoint[]>),
  robots: () => apiFetch("/api/scheduler/robots").then(json<ManagedRobot[]>),
  orders: () => apiFetch("/api/orders?limit=500").then(json<WarehouseOrder[]>),
  createOrder: (body: Record<string, unknown>) => apiFetch("/api/orders", { method: "POST", body: JSON.stringify(body) }).then(json<WarehouseOrder>),
  importOrders: (body: FormData) => apiFetch("/api/orders/import", { method: "POST", body }).then(json<OrderImportResult>),
  shelfInventory: (rackId: string) => apiFetch(`/api/shelf-inventory/${encodeURIComponent(rackId)}`).then(json<ShelfInventoryResponse>),
  dispatchInventoryItem: (id: number, body: Record<string, unknown>) => apiFetch(`/api/inventory-items/${id}/dispatch`, { method: "POST", body: JSON.stringify(body) }).then(json<InventoryDispatchResult>),
  updateOrder: (id: number, body: Record<string, unknown>) => apiFetch(`/api/orders/${id}`, { method: "PATCH", body: JSON.stringify(body) }).then(json<WarehouseOrder>),
  deleteOrder: (id: number) => apiFetch(`/api/orders/${id}`, { method: "DELETE" }).then(json<{ ok: boolean }>),
  cancelOrder: (id: number) => apiFetch(`/api/orders/${id}/cancel`, { method: "POST" }).then(json<WarehouseOrder>),
  schedules: () => apiFetch("/api/schedules?limit=500").then(json<RobotSchedule[]>),
  preview: (body: Record<string, unknown>) => apiFetch("/api/schedules/preview", { method: "POST", body: JSON.stringify(body) }).then(json<SchedulePreview>),
  createSchedule: (body: Record<string, unknown>) => apiFetch("/api/schedules/create", { method: "POST", body: JSON.stringify(body) }).then(json<RobotSchedule>),
  cancelSchedule: (id: number) => apiFetch(`/api/schedules/${id}/cancel`, { method: "POST" }).then(json<RobotSchedule>),
};
