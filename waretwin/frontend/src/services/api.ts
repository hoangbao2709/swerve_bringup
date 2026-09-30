import { API_URL } from "./ws";
import { useStore } from "../state/store";
import type { TagNavigationState } from "../schema/twin_state";

async function responseError(response: Response): Promise<string> {
  try {
    const body = await response.json() as {
      detail?: string;
      error?: { message?: string };
    };
    return body.detail || body.error?.message || response.statusText || `HTTP ${response.status}`;
  } catch {
    return response.statusText || `HTTP ${response.status}`;
  }
}

export async function navigationApi(path: string, init: RequestInit = {}) {
  const response = await apiFetch(`/api/navigation/${path}`, init);
  if (!response.ok) throw new Error(await responseError(response));
  return response.json();
}
export const startTagMission = (robot_id: string, target_tag_id: number) => {
  const state = useStore.getState();
  if (state.runtimeMode !== "LOCAL_SIM" && state.mapSync.status !== "SYNCED") {
    return Promise.reject(new Error(`Cannot start mission: map revision mismatch (${state.mapSync.status})`));
  }
  return navigationApi("missions/start", { method: "POST", body: JSON.stringify({ robot_id, target_tag_id }) }) as Promise<TagNavigationState>;
};
export const missionAction = (id: number, action: "pause" | "resume" | "cancel" | "replan") => navigationApi(`missions/${id}/${action}`, { method: "POST" });
export const emergencyStop = async (robotId: string) => {
  const response = await apiFetch(`/api/robots/${encodeURIComponent(robotId)}/emergency-stop`, { method: "POST" });
  if (!response.ok) throw new Error(await responseError(response));
  return response.json();
};
export const clearEmergencyStop = async (robotId: string) => {
  const response = await apiFetch(`/api/robots/${encodeURIComponent(robotId)}/clear-emergency-stop`, { method: "POST" });
  if (!response.ok) throw new Error(await responseError(response));
  return response.json();
};

export type LocalRobotMap = {
  id: string;
  name: string;
  robot_id: string;
  created_at: string;
  resolution: number;
  origin: number[];
  revision: string;
  frame_id: "map";
};

export type LocalRuntimeModeStatus = {
  robot_id: string;
  request_id?: string;
  mode: string | null;
  status: string;
  message?: string | null;
};

export type Vda5050Configuration = {
  robot_id: string;
  enabled: boolean;
  mqtt_host: string;
  mqtt_port: number;
  mqtt_username: string;
  password_configured: boolean;
  tls_enabled: boolean;
  topic_prefix: string;
  interface_name: string;
  manufacturer: string;
  serial_number: string;
  protocol_version: "2.0.0" | "2.1.0" | "3.0.0";
  mqtt_protocol_version: "3.1.1" | "5.0";
  allow_task: boolean;
  allow_instant_actions: boolean;
  auto_reconnect: boolean;
  reconnect_interval: number;
  connection_timeout: number;
  keepalive: number;
  client_id: string;
  connection_status: string;
  last_error: string | null;
  ignored_orders: number;
  updated_at: string | null;
};

async function localRobotApi<T>(robotId: string, path: string, init: RequestInit = {}): Promise<T> {
  const response = await apiFetch(`/api/robots/${encodeURIComponent(robotId)}/local/${path}`, init);
  if (!response.ok) throw new Error(await responseError(response));
  return response.json() as Promise<T>;
}

export const getLocalRobotMaps = (robotId: string) =>
  localRobotApi<{ robot_id: string; maps: LocalRobotMap[]; runtime_mode: string; mapping_state: string; mapping_duration_s: number; active_local_map_id: string | null; map_sync_status: string | null }>(robotId, "maps");

export const getLocalRuntimeMode = (robotId: string) =>
  localRobotApi<{ robot_id: string; current_mode: string; transition: LocalRuntimeModeStatus }>(robotId, "runtime-mode");

export const requestLocalRuntimeMode = (robotId: string, mode: "MAPPING" | "NAVIGATION") =>
  localRobotApi<{ ok: boolean; current_mode: string; requested_mode: string; request_id: string; status: string; message: string }>(robotId, "runtime-mode", { method: "POST", body: JSON.stringify({ mode }) });

export const setMappingState = (robotId: string, action: "start" | "stop") =>
  localRobotApi<{ ok: boolean; mapping_state: string }>(robotId, `mapping/${action}`, { method: "POST" });

export const saveLocalRobotMap = (robotId: string, name: string) =>
  localRobotApi<{ ok: boolean; map: LocalRobotMap }>(robotId, "maps/save", { method: "POST", body: JSON.stringify({ name }) });

export const loadLocalRobotMap = (robotId: string, map_id: string) =>
  localRobotApi<{ ok: boolean; active_map: LocalRobotMap; map_sync_status: string; message: string }>(robotId, "maps/load", { method: "POST", body: JSON.stringify({ map_id }) });

export const initializeLocalRobotPose = (robotId: string, pose: { x: number; y: number; yaw: number; frame_id: "map" }) =>
  localRobotApi<{ ok: boolean; pose: typeof pose; localization_owner: string }>(robotId, "initial-pose", { method: "POST", body: JSON.stringify(pose) });

export const getVda5050Configuration = (robotId: string) =>
  localRobotApi<Vda5050Configuration>(robotId, "vda5050");

export const applyVda5050Configuration = (robotId: string, config: Partial<Vda5050Configuration> & { mqtt_password?: string }) =>
  localRobotApi<{ ok: boolean; applied: boolean; configuration: Vda5050Configuration }>(robotId, "vda5050", { method: "PUT", body: JSON.stringify(config) });

export const testVda5050Connection = async (robotId: string, config: Partial<Vda5050Configuration> & { mqtt_password?: string }) => {
  const response = await apiFetch(`/api/robots/${encodeURIComponent(robotId)}/local/vda5050/test`, {
    method: "POST", body: JSON.stringify(config),
  });
  const result = await response.json() as { ok: boolean; latency_ms?: number; broker?: string; error_code?: string | null; message?: string | null };
  if (!response.ok && result.ok !== false) throw new Error(await responseError(response));
  return result;
};

export async function apiFetch(path: string, init: RequestInit = {}): Promise<Response> {
  const token = useStore.getState().authToken;
  const headers = new Headers(init.headers ?? {});
  if (token) headers.set("authorization", `Bearer ${token}`);
  if (init.body && !headers.has("content-type") && !(init.body instanceof FormData)) {
    headers.set("content-type", "application/json");
  }
  return fetch(`${API_URL}${path}`, { ...init, headers, credentials: "omit" });
}
