import { create } from "zustand";
import type { WarehouseLayout } from "../layout/types";
import type {
  RobotDetailState, RobotId, RobotRuntimeCapabilities, RosDiagnostics,
  RuntimeState, TagNavigationState, TwinState,
} from "../schema/twin_state";

export type RuntimeMode = "UNKNOWN" | "GAZEBO_ROS" | "REAL_ROBOT";
export type AuthStatus = "loading" | "guest" | "authenticated";
export type WebSocketState = "CONNECTING" | "CONNECTED" | "RECONNECTING" | "DISCONNECTED" | "ERROR";
export type AuthUser = { id: number; username: string; email: string; role: "admin" | "user"; is_active?: boolean };

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

type RobotMapSync = {
  rosRevision: number | null;
  gazeboRevision: number | null;
  nav2Revision: number | null;
  tagMapRevision: number | null;
  tfStatus: boolean;
  error: string | null;
  status: string;
};
type MapSync = {
  publishedRevision: number | null;
  publishedVersion: number;
  rosRevision: number | null;
  gazeboRevision: number | null;
  nav2Revision: number | null;
  tagMapRevision: number | null;
  tfStatus: boolean;
  status: string;
  error: string | null;
  robots: Record<string, RobotMapSync>;
};

function emptyMapSync(): MapSync {
  return {
    publishedRevision: null, publishedVersion: 0, rosRevision: null,
    gazeboRevision: null, nav2Revision: null, tagMapRevision: null,
    tfStatus: false, status: "UNKNOWN", error: null, robots: {},
  };
}

const layoutCollectionKeys = [
  "floors", "aisles", "navigation_tags", "navigation_edges", "lifts", "zones",
  "docks", "racks", "conveyors", "stations", "charging_stations", "parking",
  "restricted_areas", "walkways", "cameras", "sensors", "locations", "obstacles",
] as const;

function canonicalLayout(value: unknown): WarehouseLayout | null {
  if (!value || typeof value !== "object") return null;
  const candidate = value as Partial<WarehouseLayout>;
  if (!candidate.size || !candidate.grid
    || !Number.isFinite(candidate.size.width) || !Number.isFinite(candidate.size.depth)
    || !Number.isFinite(candidate.grid.cell_size) || !Number.isFinite(candidate.grid.cols)
    || !Number.isFinite(candidate.grid.rows)) return null;
  const normalized = { ...candidate } as Record<string, unknown>;
  for (const key of layoutCollectionKeys) normalized[key] = Array.isArray(candidate[key]) ? candidate[key] : [];
  normalized.spawn = candidate.spawn && Array.isArray(candidate.spawn.robots) ? candidate.spawn : { robots: [] };
  return normalized as unknown as WarehouseLayout;
}

type Notice = { text: string; kind: "warn" | "info"; until: number } | null;
type LocalizationState = {
  state: string;
  lastTagId: number | null;
  expectedTagId: number | null;
  tagVisible: boolean;
  lastTagSeenAt: string | null;
} | null;
type Store = {
  authStatus: AuthStatus;
  authToken: string | null;
  authUser: AuthUser | null;
  setAuth: (next: { status?: AuthStatus; token?: string | null; user?: AuthUser | null }) => void;
  clearAuth: () => void;

  twin: TwinState | null;
  setTwin: (value: TwinState | null) => void;
  robotDetail: Record<RobotId, RobotDetailState>;
  setRobotDetail: (id: RobotId, patch: Partial<RobotDetailState>) => void;
  tagNavigation: TagNavigationState | null;
  setTagNavigation: (value: TagNavigationState | null) => void;
  localization: LocalizationState;
  setLocalization: (value: LocalizationState) => void;

  runtimeMode: RuntimeMode;
  setRuntimeMode: (value: RuntimeMode) => void;
  runtimeState: RuntimeState;
  setRuntimeState: (value: RuntimeState) => void;
  nav2State: string;
  setNav2State: (value: string) => void;
  bridgeState: string;
  setBridgeState: (value: string) => void;
  connectedRobotIds: string[];
  setConnectedRobotIds: (value: string[]) => void;
  websocketState: WebSocketState;
  setWebsocketState: (value: WebSocketState) => void;
  rosDiagnostics: RosDiagnostics | null;
  setRosDiagnostics: (value: RosDiagnostics | null) => void;
  rosConnected: boolean;
  setRosConnected: (value: boolean) => void;
  robotCapabilities: Record<RobotId, RobotRuntimeCapabilities>;
  setRobotCapabilities: (value: Record<RobotId, RobotRuntimeCapabilities>) => void;
  lastTelemetryAt: string | null;
  setLastTelemetryAt: (value: string | null) => void;

  layout: WarehouseLayout | null;
  layoutRevision: number;
  activeWarehouseId: number | null;
  layoutUpdatedAt: string | null;
  setLayout: (value: WarehouseLayout, meta?: { revision?: number; warehouse_id?: number | null; updated_at?: string | null }) => void;
  mapSync: MapSync;
  setMapSync: (next: Partial<MapSync>) => void;

  notice: Notice;
  setNotice: (text: string | null, kind?: "warn" | "info") => void;
};

export const useStore = create<Store>((set) => ({
  authStatus: "loading",
  authToken: null,
  authUser: null,
  setAuth: (next) => set((state) => ({
    authStatus: next.status ?? state.authStatus,
    authToken: next.token === undefined ? state.authToken : next.token,
    authUser: next.user === undefined ? state.authUser : next.user,
  })),
  clearAuth: () => set({
    authStatus: "guest", authToken: null, authUser: null,
    twin: null, robotDetail: {}, tagNavigation: null, localization: null,
    runtimeMode: "UNKNOWN", runtimeState: "UNKNOWN", nav2State: "UNKNOWN", bridgeState: "UNKNOWN",
    connectedRobotIds: [], websocketState: "DISCONNECTED", rosDiagnostics: null,
    rosConnected: false, robotCapabilities: {}, lastTelemetryAt: null,
    layout: null, layoutRevision: 0, activeWarehouseId: null, layoutUpdatedAt: null,
    mapSync: emptyMapSync(), notice: null,
  }),

  twin: null,
  setTwin: (twin) => set({ twin }),
  robotDetail: {},
  setRobotDetail: (id, patch) => set((state) => ({
    robotDetail: {
      ...state.robotDetail,
      [id]: { ...EMPTY_ROBOT_DETAIL, ...(state.robotDetail[id] ?? {}), ...patch },
    },
  })),
  tagNavigation: null,
  setTagNavigation: (tagNavigation) => set({ tagNavigation }),
  localization: null,
  setLocalization: (localization) => set({ localization }),

  runtimeMode: "UNKNOWN",
  setRuntimeMode: (runtimeMode) => set({ runtimeMode }),
  runtimeState: "UNKNOWN",
  setRuntimeState: (runtimeState) => set({ runtimeState }),
  nav2State: "UNKNOWN",
  setNav2State: (nav2State) => set({ nav2State }),
  bridgeState: "UNKNOWN",
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
  robotCapabilities: {},
  setRobotCapabilities: (robotCapabilities) => set({ robotCapabilities }),
  lastTelemetryAt: null,
  setLastTelemetryAt: (lastTelemetryAt) => set({ lastTelemetryAt }),

  layout: null,
  layoutRevision: 0,
  activeWarehouseId: null,
  layoutUpdatedAt: null,
  setLayout: (value, meta = {}) => {
    const layout = canonicalLayout(value);
    if (!layout) return;
    set((state) => ({
      layout,
      layoutRevision: meta.revision ?? state.layoutRevision + 1,
      activeWarehouseId: meta.warehouse_id === undefined ? state.activeWarehouseId : meta.warehouse_id,
      layoutUpdatedAt: meta.updated_at === undefined ? state.layoutUpdatedAt : meta.updated_at,
    }));
  },
  mapSync: emptyMapSync(),
  setMapSync: (next) => set((state) => ({ mapSync: { ...state.mapSync, ...next } })),

  notice: null,
  setNotice: (text, kind = "warn") => set({
    notice: text ? { text, kind, until: Date.now() + 4000 } : null,
  }),
}));
