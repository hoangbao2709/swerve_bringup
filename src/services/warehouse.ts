import { apiFetch } from "./api";

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
  shelves: (params: {warehouse?:number;zone?:number;search?:string;status?:string} = {}) => {
    const q = new URLSearchParams();
    if (params.warehouse) q.set("warehouse", String(params.warehouse));
    if (params.zone) q.set("zone", String(params.zone));
    if (params.search) q.set("search", params.search);
    if (params.status) q.set("status", params.status);
    return apiFetch(`/api/shelves${q.size ? `?${q.toString()}` : ""}`).then(parse<ShelfRecord[]>);
  },
};
