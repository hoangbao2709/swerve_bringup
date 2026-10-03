import { create } from "zustand";
import layoutJson from "../layout/warehouse_layout.json";
import type { WarehouseLayout, LayoutLocation } from "../layout/types";
import type { TwinState, RobotId, HeatmapLayer, TagNavigationState, RosDiagnostics, RuntimeState, RobotRuntimeCapabilities, RobotDetailState } from "../schema/twin_state";
import { RUNTIME_MODE, type RuntimeMode } from "../config";

export type ViewTab = "3D" | "MAP" | "TRAFFIC" | "HEATMAP";
export type Quality = "low" | "medium" | "high";
export type SceneTool = "select" | "pan" | "paths" | "labels" | "measure";
export type LabelLayer = "zones" | "workpoints" | "lifts" | "lift1" | "lift2" | "rackLoads" | "robots" | "people";
export type LabelLayerSetting = { visible: boolean; zIndex: number };
export type LabelLayers = Record<LabelLayer, LabelLayerSetting>;

export const DEFAULT_LABEL_LAYERS: LabelLayers = {
  zones: { visible: true, zIndex: 5 },
  workpoints: { visible: true, zIndex: 6 },
  lifts: { visible: true, zIndex: 7 },
  lift1: { visible: true, zIndex: 7 },
  lift2: { visible: true, zIndex: 7 },
  rackLoads: { visible: true, zIndex: 2 },
  robots: { visible: true, zIndex: 8 },
  people: { visible: true, zIndex: 9 },
};

const LABEL_SETTINGS_KEY = "waretwin.label-layers.v1";
function loadLabelLayers(): LabelLayers {
  if (typeof window === "undefined") return DEFAULT_LABEL_LAYERS;
  try {
    const raw = window.localStorage.getItem(LABEL_SETTINGS_KEY);
    if (!raw) return DEFAULT_LABEL_LAYERS;
    const parsed = JSON.parse(raw) as Partial<Record<LabelLayer, Partial<LabelLayerSetting>>>;
    return Object.fromEntries(
      (Object.keys(DEFAULT_LABEL_LAYERS) as LabelLayer[]).map((key) => {
        const z = Number(parsed[key]?.zIndex ?? DEFAULT_LABEL_LAYERS[key].zIndex);
        return [key, {
          visible: parsed[key]?.visible ?? DEFAULT_LABEL_LAYERS[key].visible,
          zIndex: Number.isFinite(z) ? Math.max(1, Math.min(9, Math.round(z))) : DEFAULT_LABEL_LAYERS[key].zIndex,
        }];
      }),
    ) as LabelLayers;
  } catch {
    return DEFAULT_LABEL_LAYERS;
  }
}
function persistLabelLayers(next: LabelLayers) {
  if (typeof window === "undefined") return;
  try { window.localStorage.setItem(LABEL_SETTINGS_KEY, JSON.stringify(next)); } catch { /* storage may be unavailable */ }
}

/** Convert user-facing label priority (1..9) to a non-overlapping drei Html CSS z-index range. */
export function labelZIndexRange(priority: number): [number, number] {
  const p = Math.max(1, Math.min(9, Math.round(priority)));
  return [p * 10 + 9, p * 10];
}
export type AuthStatus = "loading" | "guest" | "authenticated";
export type WebSocketState = "CONNECTING" | "CONNECTED" | "RECONNECTING" | "DISCONNECTED" | "ERROR";
export type AuthUser = { id: number; username: string; email: string; role: "admin" | "user"; is_active?: boolean };
export type ModalKind = "audit" | "tasks" | "robot" | "fleet" | "scheduler" | "flows" | "shelf" | "conveyor";
export type TagGraph = {
  warehouse_id: number | null;
  frame_id?: string;
  units?: string;
  tags: Array<{ id: number; tag_id: number; family?: string; size?: number; floor_id?: string; x: number; y: number; z?: number; yaw: number; lane_id?: string; zone_id?: number | null; metadata?: Record<string, unknown>; label?: string }>;
  edges: Array<{ from_tag_id: number; to_tag_id: number; cost?: number; bidirectional?: boolean }>;
};

export interface WindowInstance {
  id: string;
  kind: ModalKind;
  title: string;
  entityId?: string;
}

export const EMPTY_ROBOT_DETAIL: RobotDetailState = {
  scan: null,
  lidar2dSensorFrame: null,
  slam3dAccumulatedCloud: null,
  slam2dMap: null,
  runtimeMapSnapshot: null,
  globalPath: null,
  localPath: null,
  goal: null,
  controller: null,
  diagnostics: null,
  errors: [],
  navigationStatus: null,
  remainingDistanceM: null,
  pathPreview: null,
  mappingState: null,
  activeLocalMapId: null,
  activeLocalMapRevision: null,
  localMapSyncStatus: null,
  lidarStreamDiagnostics: null,
};

export let layout = layoutJson as unknown as WarehouseLayout;

interface Store {
  tagNavigation: TagNavigationState | null;
  tagDetection: { visible: boolean; tagId: number | null; offsetX: number | null; offsetY: number | null; yaw: number | null; timestamp: string | null };
  localization: { state: string; lastTagId: number | null; expectedTagId: number | null; tagVisible: boolean; lastTagSeenAt: string | null };
  tagGraph: TagGraph | null;
  targetTagId: number | null;
  setTagNavigation: (mission: TagNavigationState | null) => void;
  setTargetTagId: (id: number | null) => void;
  setTagGraph: (graph: Store["tagGraph"]) => void;
  setTagDetection: (data: Store["tagDetection"]) => void;
  setLocalization: (data: Store["localization"]) => void;
  runtimeMode: RuntimeMode;
  setRuntimeMode: (mode: RuntimeMode) => void;
  runtimeState: RuntimeState;
  setRuntimeState: (state: RuntimeState) => void;
  robotCapabilities: Record<RobotId, RobotRuntimeCapabilities>;
  setRobotCapabilities: (capabilities: Record<RobotId, RobotRuntimeCapabilities>) => void;
  bridgeState: string;
  setBridgeState: (state: string) => void;
  connectedRobotIds: string[];
  setConnectedRobotIds: (robotIds: string[]) => void;
  websocketState: WebSocketState;
  setWebsocketState: (state: WebSocketState) => void;
  rosDiagnostics: RosDiagnostics | null;
  setRosDiagnostics: (value: RosDiagnostics | null) => void;
  rosConnected: boolean;
  setRosConnected: (connected: boolean) => void;
  nav2State: string;
  setNav2State: (state: string) => void;
  lastTelemetryAt: string | null;
  setLastTelemetryAt: (value: string | null) => void;
  twin: TwinState;
  /** layout.locations 以 id 索引 */
  locations: Record<string, LayoutLocation>;
  /** Authoritative database-backed warehouse map revision. */
  layoutRevision: number;
  activeWarehouseId: number | null;
  layoutUpdatedAt: string | null;
  mapSync: { publishedRevision: number | null; publishedVersion: number; rosRevision: number | null; gazeboRevision: number | null; nav2Revision: number | null; tagMapRevision: number | null; tfStatus: boolean; status: string; error: string | null; robots: Record<string, { rosRevision: number | null; gazeboRevision: number | null; nav2Revision: number | null; tagMapRevision: number | null; tfStatus: boolean; error: string | null; status: string }> };
  setMapSync: (next: Partial<Store["mapSync"]>) => void;
  setLayout: (next: WarehouseLayout, meta?: { revision?: number; warehouse_id?: number | null; updated_at?: string | null }) => void;
  selectedRobot: RobotId | null;
  quickDetailRobotId: RobotId | null;
  openRobotQuickDetail: (id: RobotId) => void;
  closeRobotQuickDetail: () => void;
  robotDetail: Record<RobotId, RobotDetailState>;
  setRobotDetail: (id: RobotId, patch: Partial<RobotDetailState>) => void;
  /** Shelf/rack selected from the live warehouse map. */
  selectedShelf: string | null;
  viewTab: ViewTab;
  quality: Quality;
  showPaths: boolean;
  showLabels: boolean;
  showLights: boolean;
  showCameras: boolean;
  /** Fine-grained 3D label visibility and stacking priority. */
  labelLayers: LabelLayers;
  tool: SceneTool;
  focusTarget: [number, number, number] | null;
  activeCamera: string;
  select: (id: RobotId | null) => void;
  selectShelf: (id: string | null) => void;
  setViewTab: (t: ViewTab) => void;
  setQuality: (q: Quality) => void;
  setTool: (t: SceneTool) => void;
  togglePaths: () => void;
  toggleLabels: () => void;
  toggleLights: () => void;
  toggleCameras: () => void;
  setLabelLayerVisible: (layer: LabelLayer, visible: boolean) => void;
  setLabelLayerZIndex: (layer: LabelLayer, zIndex: number) => void;
  resetLabelLayers: () => void;
  focus: (p: [number, number, number] | null) => void;
  setActiveCamera: (id: string) => void;
  /** 模擬控制（Phase 2 本地；Phase 3 改送 SIM_CONTROL） */
  speed: 0 | 1 | 2 | 5 | 10;
  paused: boolean;
  seed: number;
  setSpeed: (v: 0 | 1 | 2 | 5 | 10) => void;
  setPaused: (p: boolean) => void;
  /** 每 tick 由 runner (本地) 或 WebSocket (online) 呼叫 */
  setTwin: (t: TwinState) => void;
  /** Data source. In backend mode, offline never falls back to the local engine. */
  source: "connecting" | "online" | "offline" | "local" | "unauthorized";
  setSource: (s: "connecting" | "online" | "offline" | "local" | "unauthorized") => void;
  /** 後端送來的熱圖層（online 時）；local 時從本地引擎讀 */
  /** 遠端熱圖層，key = `${kind}:${floor}`（每樓獨立） */
  heat: Record<string, HeatmapLayer> | null;
  setHeat: (l: HeatmapLayer | null) => void;
  /** Phase 4 UI：Modal / 抽屜 */
  modal: ModalKind | null;
  setModal: (m: ModalKind | null) => void;
  windows: WindowInstance[];
  activeWindowId: string | null;
  windowOrder: string[];
  minimizedWindows: string[];
  openWindow: (win: WindowInstance) => void;
  focusWindow: (windowId: string) => void;
  minimizeWindow: (windowId: string) => void;
  restoreWindow: (windowId: string) => void;
  closeWindow: (windowId: string) => void;
  /** 短暫提示（例如後端回 RATE_LIMITED）；null = 不顯示 */
  notice: { text: string; kind: "warn" | "info"; until: number } | null;
  setNotice: (text: string | null, kind?: "warn" | "info") => void;
  /** 3D / Map 顯示的樓層："all" = 疊起來全顯示 */
  /** "exploded" = 二樓視覺上抬高 5 m（僅 render transform，不動模擬座標） */
  activeFloor: "all" | "exploded" | number;
  setActiveFloor: (f: "all" | "exploded" | number) => void;
  /** 點選的電梯（右欄顯示 Lift 面板；與 selectedRobot 互斥） */
  selectedLift: string | null;
  selectLift: (id: string | null) => void;
  drawer: null | "scenarios" | "ops" | "whatif";
  setDrawer: (d: null | "scenarios" | "ops" | "whatif") => void;
  /** 最近一次 What-if 結果（後端回傳，含 schema 外的 window 對照資料） */
  whatif: unknown | null;
  setWhatIf: (r: unknown | null) => void;
  authStatus: AuthStatus;
  authToken: string | null;
  authUser: AuthUser | null;
  setAuth: (next: { status?: AuthStatus; token?: string | null; user?: AuthUser | null }) => void;
  clearAuth: () => void;
}

/** 空的初始 TwinState（runner 掛載後立刻被引擎快照取代） */
const EMPTY: TwinState = {
  schema_version: "1.0", layout_id: layout.id,
  sim: { tick: 0, tick_ms: 100, speed: 1, mode: "PAUSED", seed: 42, baseline_snapshot_id: null },
  robots: {}, tasks: {}, lifts: {}, zones: {}, conveyors: {}, cameras: {}, sensors: {}, people: {}, alerts: {}, recent_events: [], recent_decisions: [],
  kpi: { tick: 0, fleet: { total: 0, active: 0, charging: 0, idle: 0, warning: 0, error: 0, offline: 0 }, operation: { throughput_per_min: 0, completed_today: 0, completed_target: 150, pending: 0, ongoing: 0, avg_task_time_s: 0, on_time_rate: 1, avg_utilization: 0 }, efficiency: { avg_travel_distance_m: 0, avg_wait_time_s: 0, congestion_index: 0, energy_kwh: 0 }, throughput_series: [], lifts: { trips: 0, utilization: 0, avg_wait_s: 0, faults: 0 } },
  subsystems: { WAREHOUSE: "NORMAL", CONVEYORS: "NORMAL", CHARGING: "NORMAL", CCTV: "NORMAL", NETWORK: "NORMAL" },
};

const layoutCollectionKeys = [
  "floors", "aisles", "navigation_tags", "navigation_edges", "lifts", "zones",
  "docks", "racks", "conveyors", "stations", "charging_stations", "parking",
  "restricted_areas", "walkways", "cameras", "sensors", "locations", "obstacles",
] as const;

/**
 * A backend layout refresh can briefly contain null optional collections. Keep
 * the last valid map until the shape is usable, and canonicalize collections
 * to empty arrays rather than letting renderers dereference null.
 */
function canonicalLayout(next: unknown): WarehouseLayout | null {
  if (!next || typeof next !== "object") return null;
  const candidate = next as Partial<WarehouseLayout>;
  if (!candidate.size || !candidate.grid
    || !Number.isFinite(candidate.size.width) || !Number.isFinite(candidate.size.depth)
    || !Number.isFinite(candidate.grid.cell_size) || !Number.isFinite(candidate.grid.cols) || !Number.isFinite(candidate.grid.rows)) return null;
  const normalized = { ...candidate } as Record<string, unknown>;
  for (const key of layoutCollectionKeys) normalized[key] = Array.isArray(candidate[key]) ? candidate[key] : [];
  normalized.spawn = candidate.spawn && Array.isArray(candidate.spawn.robots) ? candidate.spawn : { robots: [] };
  return normalized as unknown as WarehouseLayout;
}

export const useStore = create<Store>((set) => ({
  tagNavigation: null,
  tagDetection: { visible: false, tagId: null, offsetX: null, offsetY: null, yaw: null, timestamp: null },
  localization: { state: "UNANCHORED", lastTagId: null, expectedTagId: null, tagVisible: false, lastTagSeenAt: null },
  tagGraph: null,
  targetTagId: null,
  setTagNavigation: (tagNavigation) => set({ tagNavigation }),
  setTargetTagId: (targetTagId) => set({ targetTagId }),
  setTagGraph: (tagGraph) => set({ tagGraph }),
  setTagDetection: (tagDetection) => set({ tagDetection }),
  setLocalization: (localization) => set({ localization }),
  twin: EMPTY,
  runtimeMode: RUNTIME_MODE,
  setRuntimeMode: (runtimeMode) => set({ runtimeMode }),
  runtimeState: RUNTIME_MODE === "LOCAL_SIM" ? "SIMULATION" : "IDLE",
  setRuntimeState: (runtimeState) => set({ runtimeState }),
  robotCapabilities: {},
  setRobotCapabilities: (robotCapabilities) => set({ robotCapabilities }),
  bridgeState: RUNTIME_MODE === "LOCAL_SIM" ? "LOCAL" : "DISCONNECTED",
  setBridgeState: (bridgeState) => set({ bridgeState }),
  connectedRobotIds: [],
  setConnectedRobotIds: (connectedRobotIds) => set({ connectedRobotIds: [...new Set(connectedRobotIds)] }),
  websocketState: "DISCONNECTED",
  setWebsocketState: (websocketState) => set({
    websocketState,
    ...(websocketState === "CONNECTED" ? {} : { connectedRobotIds: [], robotCapabilities: {} }),
  }),
  rosDiagnostics: null,
  setRosDiagnostics: (rosDiagnostics) => set({ rosDiagnostics }),
  rosConnected: false,
  setRosConnected: (rosConnected) => set({ rosConnected }),
  nav2State: RUNTIME_MODE === "LOCAL_SIM" ? "LOCAL" : "OFFLINE",
  setNav2State: (nav2State) => set({ nav2State }),
  lastTelemetryAt: null,
  setLastTelemetryAt: (lastTelemetryAt) => set({ lastTelemetryAt }),
  speed: 1, paused: false, seed: 42,
  source: "connecting",
  setSource: (source) => set({ source }),
  authStatus: "loading",
  authToken: null,
  authUser: null,
  setAuth: (next) => set((st) => ({
    authStatus: next.status ?? st.authStatus,
    authToken: next.token === undefined ? st.authToken : next.token,
    authUser: next.user === undefined ? st.authUser : next.user,
  })),
  clearAuth: () => set({ authStatus: "guest", authToken: null, authUser: null, source: "connecting" }),
  modal: null,
  activeWindowId: null,
  windows: [], minimizedWindows: [], windowOrder: [],
  openWindow: (win) => set((st) => {
    const exists = st.windows.some((w) => w.id === win.id);
    return {
      windows: exists ? st.windows : [...st.windows, win],
      activeWindowId: win.id,
      modal: win.kind,
      minimizedWindows: st.minimizedWindows.filter((id) => id !== win.id),
      windowOrder: [...st.windowOrder.filter((id) => id !== win.id), win.id],
    };
  }),
  focusWindow: (windowId) => set((st) => ({
    activeWindowId: windowId,
    modal: st.windows.find((w) => w.id === windowId)?.kind ?? st.modal,
    windowOrder: [...st.windowOrder.filter((id) => id !== windowId), windowId],
  })),
  minimizeWindow: (windowId) => set((st) => ({
    minimizedWindows: st.minimizedWindows.includes(windowId) ? st.minimizedWindows : [...st.minimizedWindows, windowId],
    activeWindowId: st.activeWindowId === windowId ? null : st.activeWindowId,
    modal: st.activeWindowId === windowId ? null : st.modal,
  })),
  restoreWindow: (windowId) => set((st) => ({
    activeWindowId: windowId,
    modal: st.windows.find((w) => w.id === windowId)?.kind ?? st.modal,
    minimizedWindows: st.minimizedWindows.filter((id) => id !== windowId),
    windowOrder: [...st.windowOrder.filter((id) => id !== windowId), windowId],
  })),
  closeWindow: (windowId) => set((st) => ({
    windows: st.windows.filter((w) => w.id !== windowId),
    minimizedWindows: st.minimizedWindows.filter((id) => id !== windowId),
    windowOrder: st.windowOrder.filter((id) => id !== windowId),
    activeWindowId: st.activeWindowId === windowId ? null : st.activeWindowId,
    modal: st.activeWindowId === windowId ? null : st.modal,
  })),
  setModal: (kind) => set((st) => {
    if (!kind) {
      const active = st.activeWindowId;
      return active ? {
        windows: st.windows.filter((w) => w.id !== active),
        minimizedWindows: st.minimizedWindows.filter((id) => id !== active),
        windowOrder: st.windowOrder.filter((id) => id !== active),
        activeWindowId: null,
        modal: null,
      } : { modal: null };
    }
    const titleByKind: Record<ModalKind, string> = {
      audit: "Audit / Event Log",
      tasks: "Tasks",
      robot: "Robot Detail",
      fleet: "Robot Fleet",
      scheduler: "Robot Scheduler",
      flows: "Inbound / Outbound",
      shelf: "Shelf Inventory",
      conveyor: "Conveyor Control",
    };
    const win: WindowInstance = { id: kind, kind, title: titleByKind[kind] };
    const exists = st.windows.some((w) => w.id === win.id);
    const isMinimized = st.minimizedWindows.includes(win.id);

    // Menu buttons behave like toggles for the singleton windows:
    // open/focus on the first click, close when the active window is clicked again.
    if (exists && st.activeWindowId === win.id && !isMinimized) {
      return {
        windows: st.windows.filter((item) => item.id !== win.id),
        minimizedWindows: st.minimizedWindows.filter((id) => id !== win.id),
        windowOrder: st.windowOrder.filter((id) => id !== win.id),
        activeWindowId: null,
        modal: null,
      };
    }
    return {
      modal: kind,
      activeWindowId: win.id,
      windows: exists ? st.windows : [...st.windows, win],
      minimizedWindows: st.minimizedWindows.filter((id) => id !== win.id),
      windowOrder: [...st.windowOrder.filter((id) => id !== win.id), win.id],
    };
  }),
  notice: null,
  setNotice: (text, kind = "warn") => set({ notice: text ? { text, kind, until: Date.now() + 4000 } : null }),
  activeFloor: "all",
  setActiveFloor: (activeFloor) => set({ activeFloor }),
  selectedLift: null,
  selectLift: (selectedLift) => set(selectedLift ? { selectedLift, selectedRobot: null, selectedShelf: null } : { selectedLift }),
  whatif: null,
  setWhatIf: (whatif) => set({ whatif }),
  drawer: null,
  setDrawer: (drawer) => set((st) => ({ drawer: st.drawer === drawer ? null : drawer })),
  heat: null,
  setHeat: (l) => set((st) => (l === null ? { heat: null } : { heat: { ...(st.heat ?? {}), [`${l.kind}:${l.floor ?? 1}`]: l } })),
  setSpeed: (speed) => set({ speed, paused: speed === 0 }),
  setPaused: (paused) => set({ paused }),
  layoutRevision: 0,
  activeWarehouseId: null,
  layoutUpdatedAt: null,
  mapSync: { publishedRevision: null, publishedVersion: 0, rosRevision: null, gazeboRevision: null, nav2Revision: null, tagMapRevision: null, tfStatus: false, status: "ROS_OFFLINE", error: null, robots: {} },
  setMapSync: (next) => set((st) => ({ mapSync: { ...st.mapSync, ...next } })),
  setLayout: (next: WarehouseLayout, meta: { revision?: number; warehouse_id?: number | null; updated_at?: string | null } = {}) => {
    const nextLayout = canonicalLayout(next);
    if (nextLayout) layout = nextLayout;
    ZONE_COLOR = Object.fromEntries((layout.zones ?? []).map((z) => [z.id, z.color]));
    set((st) => ({
      locations: Object.fromEntries((layout.locations ?? []).map((l) => [l.id, l])),
      layoutRevision: meta.revision ?? st.layoutRevision + 1,
      activeWarehouseId: meta.warehouse_id === undefined ? st.activeWarehouseId : meta.warehouse_id,
      layoutUpdatedAt: meta.updated_at === undefined ? st.layoutUpdatedAt : meta.updated_at,
    }));
  },
  locations: Object.fromEntries(layout.locations.map((l) => [l.id, l])),
  selectedRobot: null,
  quickDetailRobotId: null,
  openRobotQuickDetail: (id) => set({ selectedRobot: id, quickDetailRobotId: id, selectedLift: null, selectedShelf: null }),
  closeRobotQuickDetail: () => set({ quickDetailRobotId: null }),
  robotDetail: {},
  setRobotDetail: (id, patch) => set((st) => ({
    robotDetail: {
      ...st.robotDetail,
      [id]: { ...EMPTY_ROBOT_DETAIL, ...(st.robotDetail[id] ?? {}), ...patch },
    },
  })),
  selectedShelf: null,
  viewTab: "3D",
  quality: "medium",
  showPaths: true,
  showLabels: true,
  showLights: true,
  showCameras: true,
  labelLayers: loadLabelLayers(),
  tool: "select",
  focusTarget: null,
  activeCamera: "CAM-B01",
  select: (id) => set(id ? { selectedRobot: id, selectedLift: null, selectedShelf: null } : { selectedRobot: id }),
  selectShelf: (id) => set(id ? { selectedShelf: id, selectedRobot: null, selectedLift: null } : { selectedShelf: null }),
  setViewTab: (viewTab) => set({ viewTab }),
  setQuality: (quality) => set({ quality }),
  setTool: (tool) => set({ tool }),
  togglePaths: () => set((s) => ({ showPaths: !s.showPaths })),
  toggleLabels: () => set((s) => ({ showLabels: !s.showLabels })),
  toggleLights: () => set((s) => ({ showLights: !s.showLights })),
  toggleCameras: () => set((s) => ({ showCameras: !s.showCameras })),
  setLabelLayerVisible: (layer, visible) => set((s) => {
    const labelLayers = { ...s.labelLayers, [layer]: { ...s.labelLayers[layer], visible } };
    persistLabelLayers(labelLayers);
    return { labelLayers };
  }),
  setLabelLayerZIndex: (layer, zIndex) => set((s) => {
    const safe = Math.max(1, Math.min(9, Math.round(zIndex)));
    const labelLayers = { ...s.labelLayers, [layer]: { ...s.labelLayers[layer], zIndex: safe } };
    persistLabelLayers(labelLayers);
    return { labelLayers };
  }),
  resetLabelLayers: () => set(() => {
    const labelLayers = Object.fromEntries(
      (Object.keys(DEFAULT_LABEL_LAYERS) as LabelLayer[]).map((key) => [key, { ...DEFAULT_LABEL_LAYERS[key] }]),
    ) as LabelLayers;
    persistLabelLayers(labelLayers);
    return { labelLayers, showLabels: true };
  }),
  focus: (focusTarget) => set({ focusTarget }),
  setActiveCamera: (activeCamera) => set({ activeCamera }),
  setTwin: (twin) => set({ twin }),
}));

/** 狀態 → 顏色，全 App 共用 */
export const STATUS_COLOR: Record<string, string> = {
  ACTIVE: "#22c55e", CHARGING: "#3b82f6", IDLE: "#eab308", WARNING: "#f97316", ERROR: "#ef4444", OFFLINE: "#6b7280",
};
export const SEVERITY_COLOR: Record<string, string> = {
  INFO: "#3b82f6", LOW: "#3b82f6", MEDIUM: "#3b82f6", HIGH: "#eab308", CRITICAL: "#ef4444",
};
export let ZONE_COLOR = Object.fromEntries(layout.zones.map((z) => [z.id, z.color]));

/** 模擬時鐘：tick 0 = 08:00:00 */
export const SIM_START_S = 8 * 3600;
export function tickToClock(tick: number, tickMs = 100, withSeconds = false): string {
  const s = Math.max(0, Math.floor(SIM_START_S + (tick * tickMs) / 1000));
  const hh = Math.floor(s / 3600) % 24, mm = Math.floor((s % 3600) / 60), ss = s % 60;
  const p = (n: number) => String(n).padStart(2, "0");
  return withSeconds ? `${p(hh)}:${p(mm)}:${p(ss)}` : `${p(hh)}:${p(mm)}`;
}
