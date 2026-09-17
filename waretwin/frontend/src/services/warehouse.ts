import { apiFetch } from "./api";

export type WarehouseRecord = {
  id: number; code: string; name: string; description: string; status: "ACTIVE"|"INACTIVE";
  width: number; depth: number; height: number; units: string; layout_id: string;
  zone_count?: number; shelf_count?: number; created_at?: string|null; updated_at?: string|null;
};
export type ZoneRecord = {
  id: number; warehouse_id: number; code: string; name: string;
  type: "STORAGE"|"PICKING"|"BUFFER"|"CHARGING"|"RESTRICTED"|"OTHER";
  status: "ACTIVE"|"INACTIVE"|"BLOCKED"; floor: number; color: string; polygon: number[][];
  description: string; layout_zone_id: string; shelf_count?: number;
};
export type ShelfRecord = {
  id: number; zone_id: number; warehouse_id: number; code: string; name: string;
  type: "STORAGE"|"PICK_FACE"|"BUFFER"|"OTHER";
  status: "AVAILABLE"|"OCCUPIED"|"FULL"|"RESERVED"|"DISABLED"|"MAINTENANCE";
  floor: number;
  position: {x:number;y:number;z:number}; size: {width:number;depth:number;height:number};
  rotation_deg: number; levels: number; capacity: number; current_load: number;
  access_point: {x:number;y:number;yaw:number}; layout_rack_id: string; description: string;
  metadata: Record<string, unknown>;
};
export type WarehouseTreeRecord = WarehouseRecord & { zones: ZoneRecord[] };
export type WarehouseMapRecord = {
  warehouse_id: number; warehouse_code: string; warehouse_name: string;
  layout_id?: string; revision: number; published_version: number; is_active: boolean;
  updated_at?: string | null;
};

async function parse<T>(response: Response): Promise<T> {
  if (response.ok) return response.json() as Promise<T>;
  let message = `${response.status} ${response.statusText}`;
  try {
    const body = await response.json();
    if (body?.detail) message = String(body.detail);
  } catch {
    const text = await response.text().catch(() => "");
    if (text) message = text;
  }
  throw new Error(message);
}

export const warehouseApi = {
  tree: () => apiFetch("/api/warehouse-tree").then(parse<WarehouseTreeRecord[]>),
  maps: () => apiFetch("/api/warehouse-maps").then(parse<WarehouseMapRecord[]>),
  activateMap: (warehouseId:number) => apiFetch(`/api/warehouse-maps/${warehouseId}/activate`, {method:"POST", body:"{}"}).then(parse<{ok:boolean;warehouse_id:number;revision:number;is_active:boolean}>),
  warehouses: () => apiFetch("/api/warehouses").then(parse<WarehouseRecord[]>),
  createWarehouse: (body: Partial<WarehouseRecord>) => apiFetch("/api/warehouses", {method:"POST", body:JSON.stringify(body)}).then(parse<WarehouseRecord>),
  updateWarehouse: (id:number, body: Partial<WarehouseRecord>) => apiFetch(`/api/warehouses/${id}`, {method:"PATCH", body:JSON.stringify(body)}).then(parse<WarehouseRecord>),
  deleteWarehouse: (id:number) => apiFetch(`/api/warehouses/${id}`, {method:"DELETE"}).then(parse<{ok:boolean}>),
  zones: (warehouseId?:number) => apiFetch(`/api/zones${warehouseId ? `?warehouse=${warehouseId}` : ""}`).then(parse<ZoneRecord[]>),
  createZone: (body: Partial<ZoneRecord> & {warehouse_id:number}) => apiFetch("/api/zones", {method:"POST", body:JSON.stringify(body)}).then(parse<ZoneRecord>),
  updateZone: (id:number, body: Partial<ZoneRecord>) => apiFetch(`/api/zones/${id}`, {method:"PATCH", body:JSON.stringify(body)}).then(parse<ZoneRecord>),
  deleteZone: (id:number) => apiFetch(`/api/zones/${id}`, {method:"DELETE"}).then(parse<{ok:boolean}>),
  shelves: (params: {warehouse?:number;zone?:number;search?:string;status?:string} = {}) => {
    const q = new URLSearchParams();
    if (params.warehouse) q.set("warehouse", String(params.warehouse));
    if (params.zone) q.set("zone", String(params.zone));
    if (params.search) q.set("search", params.search);
    if (params.status) q.set("status", params.status);
    return apiFetch(`/api/shelves${q.size ? `?${q.toString()}` : ""}`).then(parse<ShelfRecord[]>);
  },
  createShelf: (body: Record<string, unknown> & {zone_id:number}) => apiFetch("/api/shelves", {method:"POST", body:JSON.stringify(body)}).then(parse<ShelfRecord>),
  updateShelf: (id:number, body: Record<string, unknown>) => apiFetch(`/api/shelves/${id}`, {method:"PATCH", body:JSON.stringify(body)}).then(parse<ShelfRecord>),
  deleteShelf: (id:number) => apiFetch(`/api/shelves/${id}`, {method:"DELETE"}).then(parse<{ok:boolean}>),
  syncFromLayout: () => apiFetch("/api/warehouse-sync/from-layout", {method:"POST", body:"{}"}).then(parse<{ok:boolean;warehouses:number;zones:number;shelves:number}>),
};
