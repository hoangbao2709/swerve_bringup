/**
 * WebSocket client（Phase 3）
 *
 *  Backend → FULL / PATCH / HEATMAP / ERROR   →  store.twin / store.heat
 *  UI      → SIM_CONTROL / INJECT / CREATE_TASK / ACK_ALERT / RESYNC
 *
 * PATCH 合併規則（與 backend/app/main.py make_patch 對應）：
 *  - sim / kpi / subsystems / recent_decisions：整段取代
 *  - robots：每台 {...prev, ...patch}（path 只在變更時出現）
 *  - tasks / zones / conveyors / cameras / sensors / people / alerts：以 id 合併；值為 null 代表刪除
 *  - events：prepend 到 recent_events（ring 500）
 * 若 patch.base_tick 與本地 tick 不符 → 送 RESYNC 要 FULL。
 */
import type { ServerMessage, ClientMessage, TwinState, HeatmapLayer, RobotState } from "../schema/twin_state";
import { THRESHOLDS } from "../schema/twin_state";
import { useStore } from "../state/store";

export type ConnState = "connecting" | "online" | "reconnecting" | "offline" | "error" | "unauthorized";

type RuntimeLocation = Pick<Location, "protocol" | "hostname">;
type RuntimeUrls = { apiUrl: string; wsUrl: string };

const env = (import.meta as unknown as { env?: Record<string, string | undefined> }).env ?? {};

function trimTrailingSlash(value: string): string {
  return value.replace(/\/$/, "");
}

function websocketUrlFromApi(apiUrl: string): string {
  return `${trimTrailingSlash(apiUrl).replace(/^https:/, "wss:").replace(/^http:/, "ws:")}/ws`;
}

function websocketEndpoint(value: string): string {
  const url = trimTrailingSlash(value);
  return /\/ws(?:\?|$)/.test(url) ? url : `${url}/ws`;
}

/**
 * REST is the canonical backend configuration.  Unless an operator explicitly
 * overrides the WebSocket endpoint, derive it from that same host and port so
 * a REST backend on (for example) :8001 never leaves Channels on :8000.
 */
export function resolveBackendUrls(
  runtimeEnv: Record<string, string | undefined>,
  runtimeLocation: RuntimeLocation,
): RuntimeUrls {
  const protocol = runtimeLocation.protocol === "https:" ? "https" : "http";
  const host = runtimeLocation.hostname || "127.0.0.1";
  const port = runtimeEnv.VITE_BACKEND_PORT?.trim();
  const defaultApiUrl = `${protocol}://${host}${port ? `:${port}` : ""}`;
  const apiUrl = trimTrailingSlash(runtimeEnv.VITE_API_BASE_URL || runtimeEnv.VITE_API_URL || defaultApiUrl);
  const explicitWsUrl = runtimeEnv.VITE_WS_BASE_URL || runtimeEnv.VITE_WS_URL;
  return {
    apiUrl,
    wsUrl: explicitWsUrl ? websocketEndpoint(explicitWsUrl) : websocketUrlFromApi(apiUrl),
  };
}

const runtimeLocation: RuntimeLocation = typeof window === "undefined"
  ? { protocol: "http:", hostname: "127.0.0.1" }
  : window.location;
const runtimeUrls = resolveBackendUrls(env, runtimeLocation);
/** Django Channels endpoint. Its default always follows the REST backend URL. */
export const WS_URL = runtimeUrls.wsUrl;
/** Django REST base. Prefer explicit base URL, then legacy alias, then browser host/port. */
export const API_URL = runtimeUrls.apiUrl;
const COLLECTIONS = ["tasks", "lifts", "zones", "conveyors", "cameras", "sensors", "people", "alerts"] as const;

let socket: WebSocket | null = null;
let reconnectTimer = 0;
let reconnectAttempt = 0;
let stopped = false;
let localTick = -1;
let onStateChange: ((s: ConnState) => void) | null = null;
type CopilotReply = { request_id: string; text: string; citations: Array<{ robot_id?: string; task_id?: string; event_id?: string }>; model?: string };
const copilotListeners = new Set<(r: CopilotReply) => void>();
/** 訂閱 COPILOT_REPLY；回傳取消函式 */
export function onCopilotReply(fn: (r: CopilotReply) => void): () => void { copilotListeners.add(fn); return () => copilotListeners.delete(fn); }
const layoutListeners = new Set<(meta: Extract<ServerMessage, { type: "LAYOUT_UPDATED" }>) => void>();
export function onLayoutUpdated(fn: (meta: Extract<ServerMessage, { type: "LAYOUT_UPDATED" }>) => void): () => void {
  layoutListeners.add(fn);
  return () => layoutListeners.delete(fn);
}

async function refreshLayout(meta: Extract<ServerMessage, { type: "LAYOUT_UPDATED" }>) {
  try {
    if (meta.is_active === false) {
      layoutListeners.forEach((fn) => fn(meta));
      return;
    }
    const token = useStore.getState().authToken;
    const headers: Record<string, string> = {};
    if (token) headers.authorization = `Bearer ${token}`;
    const response = await fetch(`${API_URL}/api/layout`, { headers, credentials: "omit" });
    if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
    const next = await response.json();
    useStore.getState().setLayout(next, {
      revision: meta.revision,
      warehouse_id: meta.warehouse_id,
      updated_at: meta.updated_at ?? null,
    });
    layoutListeners.forEach((fn) => fn(meta));
  } catch (error) {
    console.warn("[layout] realtime refresh failed", error);
  }
}

async function refreshMapSyncStatus() {
  try {
    const token = useStore.getState().authToken;
    const headers: Record<string, string> = {};
    if (token) headers.authorization = `Bearer ${token}`;
    const response = await fetch(`${API_URL}/api/map/sync-status`, { headers, credentials: "omit" });
    if (!response.ok) return;
    const data = await response.json() as Record<string, unknown>;
    useStore.getState().setMapSync({
      publishedRevision: typeof data.published_revision === "number" ? data.published_revision : null,
      publishedVersion: Number(data.published_version ?? 0),
      rosRevision: typeof data.ros_revision === "number" ? data.ros_revision : null,
      gazeboRevision: typeof data.gazebo_revision === "number" ? data.gazebo_revision : null,
      nav2Revision: typeof data.nav2_revision === "number" ? data.nav2_revision : null,
      tagMapRevision: typeof data.tag_map_revision === "number" ? data.tag_map_revision : null,
      tfStatus: Boolean(data.tf_status),
      status: String(data.status ?? "ERROR"),
      error: data.error ? String(data.error) : null,
      robots: data.robot_map_sync && typeof data.robot_map_sync === "object"
        ? Object.fromEntries(Object.entries(data.robot_map_sync as Record<string, Record<string, unknown>>).map(([id, row]) => [id, {
            rosRevision: typeof row.ros_revision === "number" ? row.ros_revision : null,
            gazeboRevision: typeof row.gazebo_revision === "number" ? row.gazebo_revision : null,
            nav2Revision: typeof row.nav2_revision === "number" ? row.nav2_revision : null,
            tagMapRevision: typeof row.tag_map_revision === "number" ? row.tag_map_revision : null,
            tfStatus: Boolean(row.tf_status), error: row.error ? String(row.error) : null,
            status: String(row.status ?? "OUT_OF_SYNC"),
          }])) : {},
    });
  } catch (error) {
    console.warn("[map-sync] status refresh failed", error);
  }
}

const scheduleListeners = new Set<(source: string) => void>();
export function onScheduleUpdated(fn: (source: string) => void): () => void { scheduleListeners.add(fn); return () => scheduleListeners.delete(fn); }

const whatifListeners = new Set<(r: unknown) => void>();
export function onWhatIfResult(fn: (r: unknown) => void): () => void { whatifListeners.add(fn); return () => whatifListeners.delete(fn); }
const whatifErrorListeners = new Set<(message: string, request_id: string | null) => void>();
/** 後端對 WHATIF_RUN 回 ERROR（RATE_LIMITED / BAD_MESSAGE…）時通知，讓抽屜解除 Simulating 狀態 */
export function onWhatIfError(fn: (message: string, request_id: string | null) => void): () => void { whatifErrorListeners.add(fn); return () => whatifErrorListeners.delete(fn); }
/** 正在等待結果的 What-if request_id（null = 沒有）；ERROR / WHATIF_RESULT 都要帶同一個 id 才會被當成它的回應 */
let whatifPending: string | null = null;
export function markWhatIfPending(id: string | null) { whatifPending = id; }

export function wsSend(msg: ClientMessage): boolean {
  if (socket && socket.readyState === WebSocket.OPEN) { socket.send(JSON.stringify(msg)); return true; }
  return false;
}

export type ManualAction = "FORWARD" | "BACKWARD" | "LEFT" | "RIGHT" | "ROTATE_LEFT" | "ROTATE_RIGHT" | "STOP";

/** Change the authoritative ROS control mode for one robot. */
export function wsSetRobotMode(robot_id: string, mode: "MANUAL" | "AUTONOMOUS"): boolean {
  return wsSend({ type: "ROBOT_MODE", robot_id, mode });
}

/** Send one dead-man manual command. The bridge stops when commands expire. */
export function wsManualCommand(robot_id: string, action: ManualAction): boolean {
  return wsSend({ type: "ROBOT_MANUAL", robot_id, action });
}

// 分頁從背景回到前景：累積的 PATCH 可能被瀏覽器節流，直接要一份 FULL 最省事（具名 listener 才能在 disconnect 時移除）
const onVisible = () => { if (document.visibilityState === "visible") { localTick = -1; wsSend({ type: "RESYNC" }); } };

export function wsConnect(onChange: (s: ConnState) => void) {
  wsDisconnect();                       // 重複呼叫（StrictMode 雙掛載）時先把前一條連線收掉
  stopped = false; onStateChange = onChange;
  open();
  document.addEventListener("visibilitychange", onVisible);
}
export function wsDisconnect() {
  stopped = true; clearTimeout(reconnectTimer);
  reconnectAttempt = 0;
  document.removeEventListener("visibilitychange", onVisible);
  if (socket) { const s = socket; socket = null; s.onclose = null; s.onmessage = null; s.close(); }
  onStateChange = null; localTick = -1;
}

function open() {
  if (stopped) return;
  onStateChange?.("connecting");
  let ws: WebSocket;
  try {
    const token = useStore.getState().authToken;
    // Django Channels requires the auth token. Do not hammer /ws with requests
    // that are guaranteed to be rejected before login/session restore completes.
    if (!token) {
      onStateChange?.("unauthorized");
      return;
    }
    const url = `${WS_URL}${WS_URL.includes("?") ? "&" : "?"}token=${encodeURIComponent(token)}`;
    // https 頁面開 ws:// 會同步丟 SecurityError（沒設 VITE_WS_URL 時），要接住，否則整個 App 掛掉
    ws = new WebSocket(url);
  } catch (e) {
    console.warn("[ws] cannot open", WS_URL, e, "— backend unavailable");
    onStateChange?.("error");
    return;
  }
  socket = ws;
  const timeout = window.setTimeout(() => { if (ws.readyState !== WebSocket.OPEN) ws.close(); }, 2500);
  ws.onopen = () => {
    clearTimeout(timeout); reconnectAttempt = 0; onStateChange?.("online");
    void refreshMapSyncStatus();
    // Pull the database-backed map once on connect so / always matches admin pages.
    const st = useStore.getState();
    const token = st.authToken;
    const headers: Record<string, string> = {};
    if (token) headers.authorization = `Bearer ${token}`;
    fetch(`${API_URL}/api/layout`, { headers, credentials: "omit" })
      .then(async (r) => {
        if (!r.ok) throw new Error(`${r.status}`);
        const next = await r.json();
        st.setLayout(next, {
          revision: Number(r.headers.get("x-layout-revision") || 0) || undefined,
          warehouse_id: Number(r.headers.get("x-warehouse-id") || 0) || null,
          updated_at: r.headers.get("x-layout-updated-at"),
        });
      })
      .catch((e) => console.warn("[layout] initial refresh failed", e));
  };
  ws.onmessage = (ev) => {
    try { handle(JSON.parse(ev.data) as ServerMessage); }
    catch (e) { console.warn("[ws] invalid server message", e); }
  };
  ws.onerror = () => { onStateChange?.("error"); };
  ws.onclose = (ev) => {
    clearTimeout(timeout);
    if (socket === ws) socket = null;
    const authFailed = ev.code === 4401 || ev.code === 4403;
    if (authFailed) {
      if (whatifPending) { const id = whatifPending; whatifPending = null; whatifErrorListeners.forEach((fn) => fn("authentication failed — please log in again", id)); }
      onStateChange?.("unauthorized");
      return;
    }
    if (whatifPending) { const id = whatifPending; whatifPending = null; whatifErrorListeners.forEach((fn) => fn("connection lost — please run again", id)); }
    if (!stopped) {
      onStateChange?.("reconnecting");
      // A backend restart or Wi-Fi handover should not create a reconnect
      // storm.  Back off exponentially, add a small jitter so multiple tabs
      // do not reconnect on the same millisecond, and cap recovery at 30s.
      const delay = Math.min(30000, 1000 * (2 ** Math.min(reconnectAttempt, 5))) + Math.floor(Math.random() * 250);
      reconnectAttempt += 1;
      reconnectTimer = window.setTimeout(open, delay);
    } else {
      onStateChange?.("offline");
    }
  };
}

function handle(msg: ServerMessage) {
  const st = useStore.getState();
  switch (msg.type) {
    case "FULL": {
      localTick = msg.state.sim.tick;
      st.setTwin(msg.state); syncControls(msg.state);
      break;
    }
    case "PATCH": {
      if (localTick >= 0 && msg.base_tick !== localTick && msg.base_tick !== msg.tick) {
        // 漏掉了 tick（例如分頁休眠），要求全量重送
        localTick = -1; wsSend({ type: "RESYNC" }); return;
      }
      localTick = msg.tick;
      const next = applyPatch(st.twin, msg);
      st.setTwin(next); if (msg.patch.sim) syncControls(next);
      break;
    }
    case "HEATMAP": st.setHeat(msg.layer); break;
    case "LAYOUT_UPDATED": {
      void refreshLayout(msg);
      break;
    }
    case "map.published": {
      st.setMapSync({ publishedRevision: msg.map_revision, publishedVersion: msg.published_version, status: "PENDING_SYNC", error: `Revision ${msg.map_revision} is waiting for runtime reload and acknowledgement` });
      break;
    }
    case "SCHEDULE_UPDATED": scheduleListeners.forEach((fn) => fn(msg.source)); break;
    case "RUNTIME_STATUS":
      st.setRuntimeMode(msg.runtime_mode);
      if (msg.runtime_state) st.setRuntimeState(msg.runtime_state);
      if (msg.bridge_state) st.setBridgeState(msg.bridge_state);
      st.setRosConnected(msg.ros_connected);
      st.setConnectedRobotIds(Array.isArray(msg.connected_robot_ids) ? msg.connected_robot_ids : []);
      st.setNav2State(msg.nav2_state);
      st.setLastTelemetryAt(msg.last_telemetry_at);
      if (msg.diagnostics) st.setRosDiagnostics(msg.diagnostics);
      st.setMapSync({
        publishedRevision: msg.published_revision ?? st.mapSync.publishedRevision,
        publishedVersion: msg.published_version ?? st.mapSync.publishedVersion,
        rosRevision: msg.ros_revision ?? st.mapSync.rosRevision,
        gazeboRevision: msg.gazebo_revision ?? st.mapSync.gazeboRevision,
        nav2Revision: msg.nav2_revision ?? st.mapSync.nav2Revision,
        tagMapRevision: msg.tag_map_revision ?? st.mapSync.tagMapRevision,
        tfStatus: msg.tf_status ?? st.mapSync.tfStatus,
        status: msg.map_sync_status ?? st.mapSync.status,
        error: msg.map_sync_error ?? null,
        robots: msg.robot_map_sync
          ? Object.fromEntries(Object.entries(msg.robot_map_sync).map(([id, row]) => [id, {
              rosRevision: row.ros_revision, gazeboRevision: row.gazebo_revision,
              nav2Revision: row.nav2_revision, tagMapRevision: row.tag_map_revision,
              tfStatus: row.tf_status, error: row.error, status: row.status,
            }]))
          : st.mapSync.robots,
      });
      for (const [robotId, state] of Object.entries(msg.local_active_maps ?? {})) {
        st.setRobotDetail(robotId, {
          activeLocalMapId: state.local_active_map_id ?? null,
          activeLocalMapRevision: state.local_active_map_revision ?? null,
          localMapSyncStatus: state.map_sync_status ?? null,
        });
      }
      break;
    case "ROBOT_CONTROL_STATUS":
      if (msg.mode_transition_state === "APPLIED") {
        const current = st.robotDetail[msg.robot_id];
        if (current?.modeTransitionState === "REQUESTED" && current.requestedMode !== msg.applied_mode) break;
        if (current?.modeRequestId && msg.request_id !== current.modeRequestId) break;
      }
      st.setRobotDetail(msg.robot_id, {
        modeRequestId: msg.request_id ?? null,
        requestedMode: msg.requested_mode ?? null,
        ...(msg.applied_mode ? { appliedMode: msg.applied_mode } : {}),
        modeTransitionState: msg.mode_transition_state ?? null,
      });
      if (!msg.accepted) st.setNotice(`Robot control rejected: ${msg.reason || "command was rejected"}`);
      else if (msg.mode_transition_state === "APPLIED" && st.twin.robots[msg.robot_id]) st.setTwin({
        ...st.twin,
        robots: {
          ...st.twin.robots,
          [msg.robot_id]: { ...st.twin.robots[msg.robot_id], control_mode: msg.applied_mode ?? msg.mode },
        },
      });
      break;
    case "ROBOT_STATE":
      // External bridges normally arrive as PATCH/ROBOT_STATE on the same
      // socket. Keep a detail heartbeat as well for bridge versions that
      // publish the richer state packet directly.
      st.setRobotDetail(msg.robot_id, {
        navigationStatus: msg.navigation_state ?? null,
        activeLocalMapId: msg.active_map_id && msg.active_map_id !== "CANONICAL" ? msg.active_map_id : null,
        activeLocalMapRevision: msg.active_map_id && msg.active_map_id !== "CANONICAL" ? msg.active_map_revision ?? null : null,
        localMapSyncStatus: msg.active_map_id && msg.active_map_id !== "CANONICAL" ? "LOCAL_ONLY" : null,
      });
      break;
    case "LIDAR_SCAN":
      st.setRobotDetail(msg.scan.robot_id, { scan: msg.scan });
      break;
    case "LIDAR_MAP_2D":
      st.setRobotDetail(msg.robot_id, { lidar2d: msg });
      break;
    case "LIDAR_MAP_3D":
      st.setRobotDetail(msg.robot_id, { lidar3d: msg });
      break;
    case "MAP_SNAPSHOT":
      st.setRobotDetail(msg.map.robot_id, { map: msg.map });
      break;
    case "NAV_GLOBAL_PATH":
      st.setRobotDetail(msg.path.robot_id, { globalPath: msg.path });
      break;
    case "NAV_LOCAL_PATH":
      st.setRobotDetail(msg.path.robot_id, { localPath: msg.path });
      break;
    case "NAV_GOAL":
      st.setRobotDetail(msg.goal.robot_id, { goal: msg.goal });
      break;
    case "PATH_PREVIEW_RESULT":
      st.setRobotDetail(msg.robot_id, { pathPreview: msg });
      break;
    case "LOCAL_MAP_STATUS":
      st.setRobotDetail(msg.robot_id, {
        activeLocalMapId: msg.loaded ? msg.map_id ?? null : null,
        activeLocalMapRevision: msg.loaded ? msg.active_map_revision ?? msg.map_revision ?? null : null,
        localMapSyncStatus: msg.loaded ? msg.map_sync_status ?? "LOCAL_ONLY" : null,
      });
      break;
    case "COMMAND_DIAGNOSTICS": {
      const existing = st.robotDetail[msg.robot_id]?.diagnostics;
      st.setRobotDetail(msg.robot_id, {
        diagnostics: { ...(existing ?? {
          ros: false, gazebo: false, controller_manager: false, slam: false,
          nav2: false, tf: false, lidar: false, nodes: [], topics: [],
          controllers: [], simulation_time: null, last_update_at: null,
        }), command_ownership: {
          active_command_source: msg.active_command_source,
          active_control_mode: msg.active_control_mode,
          last_command_age: msg.last_command_age,
          manual_source_active: msg.manual_source_active,
          nav_source_active: msg.nav_source_active,
          tag_source_active: msg.tag_source_active,
          estop_active: msg.estop_active,
        } },
      });
      break;
    }
    case "LIDAR_STREAM_DIAGNOSTICS":
      st.setRobotDetail(msg.robot_id, { lidarStreamDiagnostics: msg });
      break;
    case "VDA5050_RUNTIME_STATUS":
      break;
    case "CONTROLLER_STATE":
      st.setRobotDetail(msg.controller.robot_id, { controller: msg.controller });
      break;
    case "SYSTEM_DIAGNOSTICS":
      st.setRosDiagnostics(msg.diagnostics);
      st.setRobotDetail(msg.robot_id, {
        diagnostics: msg.diagnostics,
        errors: msg.diagnostics.errors ?? [],
      });
      break;
    case "NAV_STATUS":
      {
        const existingErrors = st.robotDetail[msg.robot_id]?.errors ?? [];
        const errors = msg.status === "FAILED"
          ? [...existingErrors.filter((item) => item.code !== "NAV_GOAL_FAILED"), {
            severity: "ERROR" as const,
            code: "NAV_GOAL_FAILED",
            message: msg.reason ?? "Nav2 goal failed",
            timestamp: msg.timestamp ?? null,
          }]
          : existingErrors;
      st.setRobotDetail(msg.robot_id, {
        navigationStatus: msg.status,
        errors,
        goal: msg.x == null || msg.y == null || msg.yaw == null
          ? st.robotDetail[msg.robot_id]?.goal ?? null
          : {
            robot_id: msg.robot_id,
            frame_id: "map",
            x: msg.x,
            y: msg.y,
            yaw: msg.yaw,
            status: msg.status,
            timestamp: msg.timestamp ?? null,
          },
      });
      }
      break;
    case "TAG_NAV_STATUS":
      if (msg.mission) st.setTagNavigation(msg.mission);
      else if (st.tagNavigation) st.setTagNavigation({ ...st.tagNavigation, mission_id: msg.mission_id ?? st.tagNavigation.mission_id, id: msg.mission_id ?? st.tagNavigation.id, robot_id: msg.robot_id, status: msg.state ?? msg.status ?? st.tagNavigation.status, current_tag_id: msg.current_tag_id ?? st.tagNavigation.current_tag_id, next_tag_id: msg.next_tag_id ?? st.tagNavigation.next_tag_id, target_tag_id: msg.target_tag_id ?? st.tagNavigation.target_tag_id, route: msg.route ?? st.tagNavigation.route, route_index: msg.route_index ?? st.tagNavigation.route_index, progress_percent: msg.progress_percent ?? st.tagNavigation.progress_percent });
      break;
    case "TAG_DETECTION": st.setTagDetection({ visible: msg.visible, tagId: msg.tag_id ?? null, offsetX: msg.offset_x ?? null, offsetY: msg.offset_y ?? null, yaw: msg.yaw ?? null, timestamp: msg.timestamp ?? null }); break;
    case "LOCALIZATION_STATUS": st.setLocalization({ state: msg.state, lastTagId: msg.last_tag_id ?? null, expectedTagId: msg.expected_tag_id ?? null, tagVisible: msg.tag_visible ?? false, lastTagSeenAt: msg.last_tag_seen_at ?? null }); break;
    case "TAG_NAV_ROUTE": if (st.tagNavigation) st.setTagNavigation({ ...st.tagNavigation, route: msg.route }); break;
    case "TAG_NAV_EVENT": break;
    case "COPILOT_REPLY": copilotListeners.forEach((fn) => fn(msg as unknown as CopilotReply)); break;
    case "WHATIF_RESULT": if (!msg.request_id || msg.request_id === whatifPending) whatifPending = null; whatifListeners.forEach((fn) => fn(msg.result)); break;
    case "ERROR": {
      console.warn("[ws] server error", msg.code, msg.message);
      if (msg.code === "RATE_LIMITED" || msg.code === "TOO_LARGE" || msg.code === "BAD_TASK" || msg.code === "BAD_ASSIGN" || msg.code === "BAD_MESSAGE" || msg.code === "FORBIDDEN") {
        st.setNotice(`${msg.code === "RATE_LIMITED" ? "Rate limit" : msg.code === "TOO_LARGE" ? "Request too large" : msg.code === "BAD_TASK" ? "Task rejected" : msg.code === "BAD_ASSIGN" ? "Assignment rejected" : msg.code === "FORBIDDEN" ? "Forbidden" : "Rejected"}: ${msg.message}`);
        // Copilot 等待中的泡泡也要收掉
        const rid = (msg as unknown as { request_id?: string }).request_id;
        if (rid) copilotListeners.forEach((fn) => fn({ request_id: rid, text: `⏳ ${msg.message}`, citations: [] }));
        // 只有 request_id 等於正在等待的 What-if 才算它的錯誤（其他錯誤例如 BAD_TASK 不會誤關 Simulating）
        if (whatifPending && rid === whatifPending) { const id = whatifPending; whatifPending = null; whatifErrorListeners.forEach((fn) => fn(msg.message, id)); }
      }
      break;
    }
    default: break;
  }
}

/** 後端是權威：播放/暫停/倍速以 sim 欄位為準（例如另一個分頁按了暫停） */
function syncControls(t: TwinState) {
  const st = useStore.getState();
  if (st.speed !== t.sim.speed) st.setSpeed(t.sim.speed);
  const paused = t.sim.mode === "PAUSED";
  if (st.paused !== paused) st.setPaused(paused);
}

type Patch = Extract<ServerMessage, { type: "PATCH" }>;

function applyPatch(prev: TwinState, msg: Patch): TwinState {
  const p = msg.patch as Record<string, unknown>;
  const next: TwinState = { ...prev };
  if (p.sim) next.sim = { ...prev.sim, ...(p.sim as TwinState["sim"]) };
  if (p.kpi) next.kpi = p.kpi as TwinState["kpi"];
  if (p.subsystems) next.subsystems = p.subsystems as TwinState["subsystems"];
  if (p.recent_decisions) next.recent_decisions = p.recent_decisions as TwinState["recent_decisions"];
  if (p.robots) {
    const robots = { ...prev.robots };
    for (const [id, d] of Object.entries(p.robots as Record<string, Partial<RobotState>>)) robots[id] = { ...robots[id], ...d } as RobotState;
    next.robots = robots;
  }
  for (const key of COLLECTIONS) {
    const d = p[key] as Record<string, unknown> | undefined;
    if (!d) continue;
    const col = { ...(prev[key] as Record<string, unknown>) };
    for (const [id, v] of Object.entries(d)) { if (v === null) delete col[id]; else col[id] = v; }
    (next as unknown as Record<string, unknown>)[key] = col;
  }
  if (msg.events.length) next.recent_events = [...msg.events.slice().reverse(), ...prev.recent_events].slice(0, THRESHOLDS.EVENT_RING_SIZE);
  return next;
}

export type { HeatmapLayer };
