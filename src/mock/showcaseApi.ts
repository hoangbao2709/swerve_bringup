import type { LocalMapLoadResult, LocalRobotMap, Vda5050Configuration } from "../services/api";
import { SHOWCASE_LOCAL_MAP, SHOWCASE_MAP_ID, SHOWCASE_MAP_REVISION, STATIC_ROBOT_ID } from "./fixtures";

const MAPS_KEY = "ptagv.web-local.showcase-maps.v1";
const VDA_KEY = "ptagv.web-local.showcase-vda5050.v1";

function jsonResponse(body: unknown, status = 200): Response {
  return new Response(JSON.stringify(body), { status, headers: { "content-type": "application/json" } });
}

function readJson<T>(key: string, fallback: T): T {
  try {
    const value = JSON.parse(window.localStorage.getItem(key) ?? "null") as T | null;
    return value ?? fallback;
  } catch {
    return fallback;
  }
}

function writeJson<T>(key: string, value: T): boolean {
  try {
    window.localStorage.setItem(key, JSON.stringify(value));
    return true;
  } catch {
    return false;
  }
}

function maps(): LocalRobotMap[] {
  const stored = readJson<LocalRobotMap[]>(MAPS_KEY, [SHOWCASE_LOCAL_MAP]);
  return Array.isArray(stored) && stored.some((map) => map.id === SHOWCASE_MAP_ID)
    ? stored
    : [SHOWCASE_LOCAL_MAP, ...(Array.isArray(stored) ? stored : [])];
}

const DEFAULT_VDA: Vda5050Configuration = {
  robot_id: STATIC_ROBOT_ID,
  enabled: false,
  mqtt_host: "broker.example.invalid",
  mqtt_port: 1883,
  mqtt_username: "",
  password_configured: false,
  tls_enabled: true,
  topic_prefix: "uagv/v2/ptagv",
  interface_name: "PTAGV Web Local",
  manufacturer: "PTAGV",
  serial_number: "PTAGV-001",
  protocol_version: "2.1.0",
  mqtt_protocol_version: "5.0",
  allow_task: false,
  allow_instant_actions: false,
  instant_actions_supported: false,
  instant_actions_status: "NOT_IMPLEMENTED",
  auto_reconnect: true,
  reconnect_interval: 5,
  connection_timeout: 10,
  keepalive: 30,
  client_id: "ptagv-web-local-showcase",
  connection_status: "DISCONNECTED",
  last_error: null,
  ignored_orders: 0,
  updated_at: null,
};

function vdaConfig(): Vda5050Configuration {
  const value = readJson<Partial<Vda5050Configuration>>(VDA_KEY, {});
  return { ...DEFAULT_VDA, ...value, password_configured: false, connection_status: "DISCONNECTED", last_error: null };
}

function requestBody(init: RequestInit): Record<string, unknown> {
  if (typeof init.body !== "string") return {};
  try {
    const value = JSON.parse(init.body) as unknown;
    return value && typeof value === "object" ? value as Record<string, unknown> : {};
  } catch {
    return {};
  }
}

function unavailable(operation: string): Response {
  return jsonResponse({ ok: false, code: "STANDALONE_ACTION_UNAVAILABLE", error: `${operation} requires the robot runtime.` }, 409);
}

/** Local, deterministic transport for the public build. It never calls fetch or opens a socket. */
export async function showcaseApiFetch(path: string, init: RequestInit = {}): Promise<Response> {
  const method = (init.method ?? "GET").toUpperCase();
  const route = path.split("?", 1)[0].replace(/\/+$/, "") || "/";
  const localRoot = `/api/robots/${STATIC_ROBOT_ID}/local`;

  if (route === "/api/system/status") {
    return jsonResponse({
      status: "SAMPLE_DATA",
      ok: true,
      backend_status: "STANDALONE",
      backend_ready: false,
      system_status: "SAMPLE DATA",
      system_ready: false,
      database: null,
      timestamp: null,
      version: "Web Local",
      mode: "SAMPLE",
      runtime_mode: "STANDALONE",
      system: {
        cpu_load_1m: 0.42,
        cpu_count: 4,
        memory: { total_bytes: 4_294_967_296, used_bytes: 2_039_576_576, used_percent: 47.5 },
        disk: { total_bytes: 64_000_000_000, free_bytes: 39_680_000_000, used_bytes: 24_320_000_000, used_percent: 38 },
        uptime_s: 18 * 3600 + 24 * 60,
      },
      runtime: { bridge_state: "DISCONNECTED", ros_connected: false, online_robot_ids: [], nav2_state: "UNAVAILABLE", last_telemetry_at: null },
      robots: { [STATIC_ROBOT_ID]: { status: "OFFLINE", fsm: "IDLE", last_telemetry_at: null } },
    });
  }

  if (route === "/api/local/identity") return unavailable("Robot identity lookup");

  if (route === "/api/map/sync-status") {
    return jsonResponse({ published_revision: null, published_version: 0, ros_revision: null,
      gazebo_revision: null, nav2_revision: null, tag_map_revision: null, tf_status: false,
      status: "UNAVAILABLE", error: null, robot_map_sync: {} });
  }

  if (route === `${localRoot}/maps` && method === "GET") {
    return jsonResponse({ robot_id: STATIC_ROBOT_ID, maps: maps(), runtime_mode: "IDLE",
      mapping_state: "UNAVAILABLE", mapping_duration_s: 0, robot_control_mode: "AUTONOMOUS",
      robot_stopped: null, robot_navigation_state: "UNAVAILABLE", estop_active: null,
      active_local_map_id: SHOWCASE_MAP_ID, local_active_map_id: SHOWCASE_MAP_ID,
      local_active_map_revision: SHOWCASE_MAP_REVISION, active_map_id: null, active_map_revision: null,
      canonical_map_revision: null, map_sync_status: "UNAVAILABLE" });
  }

  if (route === `${localRoot}/maps/save` && method === "POST") {
    const requested = String(requestBody(init).name ?? "").trim();
    if (!requested || requested.length > 64) return jsonResponse({ ok: false, code: "INVALID_MAP_NAME", error: "Enter a map name between 1 and 64 characters." }, 400);
    const current = maps();
    const nextId = `local-map-${String(current.length).padStart(2, "0")}`;
    const next: LocalRobotMap = { ...SHOWCASE_LOCAL_MAP, id: nextId, map_id: nextId, name: requested,
      revision: `local-${current.length}`, created_at: new Date().toISOString(),
      slam_session_state: { status: "UNAVAILABLE", engine: "none", artifact_id: null },
      navigation_artifacts: { yaml: false, image: false } };
    if (!writeJson(MAPS_KEY, [next, ...current])) return jsonResponse({ ok: false, code: "BROWSER_STORAGE_UNAVAILABLE", error: "Map copy could not be saved in this browser." }, 507);
    return jsonResponse({ ok: true, map: next, scope: "BROWSER_LOCAL_COPY" });
  }

  if (route === `${localRoot}/maps/load` && method === "POST") {
    const id = String(requestBody(init).map_id ?? "");
    const selected = maps().find((map) => map.id === id);
    if (!selected) return jsonResponse({ ok: false, code: "MAP_NOT_FOUND", error: "Select an available saved map." }, 404);
    const result: LocalMapLoadResult & { scope: string } = { ok: true, status: "PREVIEW_SELECTED", active_map: selected,
      active_map_id: selected.id, active_map_revision: selected.revision, local_active_map_id: selected.id,
      local_active_map_revision: selected.revision, canonical_map_revision: null, map_source: "LOCAL_MAP",
      map_sync_status: "UNAVAILABLE", request_id: null, mapping_state: "UNAVAILABLE",
      message: `Map preview selected: ${selected.name}. Robot map was not changed.`, scope: "BROWSER_LOCAL_PREVIEW" };
    return jsonResponse(result);
  }

  if (route === `${localRoot}/vda5050` && method === "GET") return jsonResponse(vdaConfig());
  if (route === `${localRoot}/vda5050` && method === "PUT") {
    const body = requestBody(init);
    const editable = { ...body };
    delete editable.mqtt_password;
    const next = { ...vdaConfig(), ...editable, password_configured: false,
      connection_status: "DISCONNECTED", last_error: null, updated_at: new Date().toISOString() } as Vda5050Configuration;
    if (!writeJson(VDA_KEY, next)) return jsonResponse({ ok: false, code: "BROWSER_STORAGE_UNAVAILABLE", error: "Configuration could not be saved in this browser." }, 507);
    return jsonResponse({ ok: true, applied: false, scope: "BROWSER_LOCAL_CONFIGURATION", configuration: next });
  }
  if (route === `${localRoot}/vda5050/test` && method === "POST") {
    return jsonResponse({ ok: false, error_code: "STANDALONE_NO_BROKER", message: "Broker connection was not attempted in this standalone view." });
  }

  if (route === `${localRoot}/runtime-mode` || route === `${localRoot}/initial-pose` || route.includes("/mapping/")) {
    return unavailable("Robot runtime operation");
  }

  if (route.startsWith("/api/robots/")) return unavailable("Robot operation");
  if (route.startsWith("/api/system/settings/")) return jsonResponse({ available: false, code: "HOST_SETTINGS_UNAVAILABLE", message: "Host settings are unavailable in this browser." }, 503);
  return jsonResponse({ ok: false, code: "SHOWCASE_ENDPOINT_NOT_FOUND", error: "This operation is not available in the standalone view." }, 404);
}
