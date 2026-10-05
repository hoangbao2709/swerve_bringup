import { API_URL } from "./ws";
import { useStore } from "../state/store";

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

export type RobotNavigationTag = {
  id: number;
  tag_id: number;
  label: string;
  family: string;
  floor_id: string;
  lane_id: string;
  zone_id: number | null;
  x: number | null;
  y: number | null;
  z: number | null;
  yaw: number | null;
  enabled: boolean;
  navigable: boolean;
  reason: string | null;
  frame_id: "map";
  map_id: string;
  map_revision: string;
  navigation_pose: { x: number; y: number; yaw: number } | null;
  navigation_pose_source: string | null;
  tag_revision: string;
  metadata: Record<string, unknown>;
};

export type RobotNavigationTagRegistry = {
  robot_id: string;
  source: "WAREHOUSE_NAVIGATION_TAG_REGISTRY";
  warehouse_id: number | null;
  warehouse_code?: string;
  map_id: string | null;
  map_revision: string | null;
  active_map_id?: string | null;
  active_map_revision?: string | null;
  frame_id: "map";
  compatible: boolean;
  reason: string | null;
  registration_required?: boolean;
  transform_source?: string | null;
  registration_revision?: string | null;
  registry_revision: string | null;
  tags: RobotNavigationTag[];
};

export async function getRobotNavigationTags(robotId: string): Promise<RobotNavigationTagRegistry> {
  const response = await apiFetch(`/api/robots/${encodeURIComponent(robotId)}/navigation-tags`);
  if (!response.ok) throw new Error(await responseError(response));
  return response.json() as Promise<RobotNavigationTagRegistry>;
}

export type EmergencyStopResult = { ok: true; mission?: unknown };
export const emergencyStop = async (robotId: string): Promise<EmergencyStopResult> => {
  const response = await apiFetch(`/api/robots/${encodeURIComponent(robotId)}/emergency-stop`, { method: "POST" });
  if (!response.ok) throw new Error(await responseError(response));
  const body = await response.json() as Partial<EmergencyStopResult>;
  if (body.ok !== true) throw new Error("EMERGENCY_STOP_UNCONFIRMED: ROS bridge did not acknowledge the stop request");
  return body as EmergencyStopResult;
};
export type ClearEmergencyStopResult = {
  ok: true;
  code: "CLEAR_ESTOP_APPLIED";
  robot_id: string;
  emergency_stop_active: false;
  pre_stop_navigation_terminal: true;
  message?: string;
};
export const clearEmergencyStop = async (robotId: string): Promise<ClearEmergencyStopResult> => {
  const response = await apiFetch(`/api/robots/${encodeURIComponent(robotId)}/clear-emergency-stop`, { method: "POST" });
  const body = await response.json().catch(() => ({})) as Partial<ClearEmergencyStopResult> & { code?: string; error?: string };
  if (!response.ok) throw new Error(`${body.code ?? "CLEAR_ESTOP_REJECTED"}: ${body.error ?? `HTTP ${response.status}`}`);
  if (body.ok !== true || body.code !== "CLEAR_ESTOP_APPLIED" || body.emergency_stop_active !== false
      || body.pre_stop_navigation_terminal !== true) {
    throw new Error("CLEAR_ESTOP_UNCONFIRMED: backend did not confirm that the E-STOP latch was cleared");
  }
  return body as ClearEmergencyStopResult;
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
  width?: number;
  height?: number;
  image_sha256?: string;
  map_id?: string;
  map_kind?: "SAVED_LOCAL_MAP" | string;
  canonical_map_promoted?: boolean;
  known_cells?: number | null;
  unknown_cells?: number | null;
  free_cells?: number | null;
  occupied_cells?: number | null;
  explored_area_m2?: number | null;
  slam_session_state?: { status: string; engine?: string; artifact_id?: string | null };
  navigation_artifacts?: { yaml: boolean; image: boolean; image_format?: string };
};

export type LocalRuntimeModeStatus = {
  robot_id: string;
  request_id?: string;
  mode: string | null;
  status: string;
  message?: string | null;
};

export type LocalMapLoadResult = {
  ok: boolean;
  status?: "TRANSITIONING" | "LOADED" | string;
  active_map?: LocalRobotMap;
  active_map_id?: string;
  active_map_revision?: string;
  local_active_map_id?: string;
  local_active_map_revision?: string;
  canonical_map_revision?: string | number | null;
  map_source?: "SAVED_LOCAL" | string;
  map_sync_status?: string;
  request_id?: string | null;
  transition?: LocalRuntimeModeStatus;
  mapping_state?: string;
  message: string;
};

export type LocalSlamResumeResult = {
  ok: boolean;
  status?: "TRANSITIONING" | "RESUMED" | string;
  map?: LocalRobotMap;
  map_id?: string;
  request_id?: string | null;
  mapping_state?: string;
  transition?: LocalRuntimeModeStatus;
  restore_evidence?: {
    passed?: boolean;
    saved_dimensions?: number[];
    live_dimensions?: number[];
    saved_known_cells?: number;
    live_known_cells?: number;
    saved_coverage_ratio?: number;
    known_overlap_ratio?: number;
    cell_class_agreement_ratio?: number;
    saved_image_sha256?: string;
    live_grid_sha256?: string;
    reason?: string;
  } | null;
  message: string;
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
  instant_actions_supported: boolean;
  instant_actions_status: string;
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
  localRobotApi<{ robot_id: string; maps: LocalRobotMap[]; runtime_mode: string; mapping_state: string; mapping_duration_s: number; robot_control_mode: string; robot_stopped: boolean; active_local_map_id: string | null; local_active_map_id: string | null; local_active_map_revision: string | null; active_map_id: string | null; active_map_revision: string | null; canonical_map_revision: string | number | null; map_sync_status: string | null }>(robotId, "maps");

export const getLocalRuntimeMode = (robotId: string) =>
  localRobotApi<{ robot_id: string; current_mode: string; transition: LocalRuntimeModeStatus }>(robotId, "runtime-mode");

export const requestLocalRuntimeMode = (robotId: string, mode: "MAPPING" | "NAVIGATION") =>
  localRobotApi<{ ok: boolean; current_mode: string; requested_mode: string; request_id: string; status: string; message: string }>(robotId, "runtime-mode", { method: "POST", body: JSON.stringify({ mode }) });

export const setMappingState = (robotId: string, action: "start" | "stop") =>
  localRobotApi<{ ok: boolean; mapping_state: string }>(robotId, `mapping/${action}`, { method: "POST" });

export const saveLocalRobotMap = (robotId: string, name: string) =>
  localRobotApi<{ ok: boolean; map: LocalRobotMap }>(robotId, "maps/save", { method: "POST", body: JSON.stringify({ name }) });

export const loadLocalRobotMap = (robotId: string, map_id: string) =>
  localRobotApi<LocalMapLoadResult>(robotId, "maps/load", { method: "POST", body: JSON.stringify({ map_id }) });

export const resumeLocalRobotSlamSession = (robotId: string, map_id: string) =>
  localRobotApi<LocalSlamResumeResult>(robotId, "maps/resume-session", { method: "POST", body: JSON.stringify({ map_id }) });

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
