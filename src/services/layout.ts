import type { WarehouseLayout } from "../layout/types";
import { useStore } from "../state/store";
import { apiFetch } from "./api";

export type LayoutUpdateMeta = {
  warehouse_id?: number | null;
  layout_id?: string;
  revision?: number;
  published_version?: number;
  is_active?: boolean;
  updated_at?: string | null;
  source?: string;
};

async function parseJson<T>(response: Response): Promise<T> {
  if (response.ok) return response.json() as Promise<T>;
  let detail = `${response.status} ${response.statusText}`;
  try {
    const body = await response.json();
    if (body?.detail) detail = String(body.detail);
  } catch {
    // ignore
  }
  throw new Error(detail);
}

export async function fetchAuthoritativeLayout(meta: LayoutUpdateMeta = {}): Promise<WarehouseLayout> {
  const response = await apiFetch("/api/layout");
  const next = await parseJson<WarehouseLayout>(response);
  useStore.getState().setLayout(next, {
    revision: meta.revision ?? (Number(response.headers.get("x-layout-revision") || 0) || undefined),
    warehouse_id: meta.warehouse_id ?? (Number(response.headers.get("x-warehouse-id") || 0) || null),
    updated_at: meta.updated_at ?? response.headers.get("x-layout-updated-at"),
  });
  return next;
}

export async function bootstrapLayout(): Promise<void> {
  try {
    await fetchAuthoritativeLayout();
  } catch (error) {
    console.warn("[layout] failed to load authoritative layout", error);
  }
}
