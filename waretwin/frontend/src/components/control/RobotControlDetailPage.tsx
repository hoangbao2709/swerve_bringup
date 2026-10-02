import { Component, memo, useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState, type ErrorInfo, type ReactNode, type MouseEvent as ReactMouseEvent } from "react";
import { apiFetch, clearEmergencyStop, emergencyStop, getRobotNavigationTags, type RobotNavigationTag, type RobotNavigationTagRegistry } from "../../services/api";
import { WS_URL, wsManualCommand, wsSetRobotMode, wsSend, type ManualAction } from "../../services/ws";
import { useSimulationRunner } from "../../simulation/runner";
import { layout, useStore } from "../../state/store";
import type { FramePose, RobotDetailError, RobotDetailGoal, RobotDetailMapSnapshot, RobotDetailPath, RobotDetailPathPreview, RobotState, RobotSystemDiagnostics } from "../../schema/twin_state";
import type { WarehouseLayout } from "../../layout/types";
import { createWorldTransform, floorBoundary, screenToWorld, worldToScreen, type WorldBounds, type WorldTransform } from "../../layout/coordinates";
import { useStableDisplayedFramePose, type MapPoseIdentity } from "../../layout/robotPoseFrame";
import { AccumulatedSlamMap2DView, LocalRobotSection } from "./LocalRobotSections";
import { RobotLidar3DView } from "./RobotLidarViews";
import { occupancyRasters } from "./occupancyRaster";
import { detailPerformance } from "./detailPerformance";

type WorldGoal = { x: number; y: number; yaw: number };
type HostStatus = { system?: { cpu_load_1m?: number | null; memory?: { used_percent?: number | null } } };
type LocalTab = "CONTROL" | "MAPPING" | "LOCALIZATION" | "VDA5050" | "DIAGNOSTICS";
type MapSource = "GLOBAL" | "LIDAR";
type LidarDimension = "2D" | "3D";
type NavigationTargetMethod = "MAP_POINT" | "TAG";
const LOCAL_TABS: LocalTab[] = ["CONTROL", "MAPPING", "LOCALIZATION", "VDA5050", "DIAGNOSTICS"];

const MANUAL_ACTIONS: Array<{ action: ManualAction; label: string; title: string }> = [
  { action: "FORWARD", label: "▲", title: "Forward (W / ↑)" },
  { action: "LEFT", label: "◀", title: "Strafe left (A / ←)" },
  { action: "STOP", label: "■", title: "Stop" },
  { action: "RIGHT", label: "▶", title: "Strafe right (D / →)" },
  { action: "BACKWARD", label: "▼", title: "Backward (S / ↓)" },
  { action: "ROTATE_LEFT", label: "↺", title: "Rotate left (Q)" },
  { action: "ROTATE_RIGHT", label: "↻", title: "Rotate right (E)" },
];

const EMPTY_ERRORS: RobotDetailError[] = [];

function pushRoute(path: string) {
  window.history.pushState({}, "", path);
  window.dispatchEvent(new PopStateEvent("popstate"));
}

function safeNumber(value: unknown, digits = 2, suffix = "") {
  const n = Number(value);
  return Number.isFinite(n) ? `${n.toFixed(digits)}${suffix}` : "N/A";
}

function safeText(value: unknown, fallback = "N/A") {
  return value === null || value === undefined || value === "" ? fallback : String(value);
}

function statusClass(value: unknown) {
  const text = String(value ?? "").toUpperCase();
  if (["OK", "ONLINE", "CONNECTED", "ACTIVE", "LOCALIZED", "AVAILABLE", "SUCCEEDED", "READY", "RUNNING"].includes(text)) return "ok";
  if (["WARNING", "DEGRADED", "WAITING", "STALE", "INITIALIZING", "PENDING", "PAUSED"].includes(text)) return "warning";
  if (["ERROR", "OFFLINE", "DISCONNECTED", "LOST", "FAILED", "CRITICAL"].includes(text)) return "error";
  return "neutral";
}

function StatusValue({ value }: { value: unknown }) {
  return <span className={`robot-detail-status ${statusClass(value)}`}>{safeText(value)}</span>;
}

class RobotDetailErrorBoundary extends Component<{ children: ReactNode }, { error: Error | null }> {
  state = { error: null as Error | null };
  static getDerivedStateFromError(error: Error) { return { error }; }
  componentDidCatch(error: Error, info: ErrorInfo) { console.error("[RobotControlDetail] render error", error, info); }
  render() {
    if (!this.state.error) return this.props.children;
    return (
      <main className="robot-detail-error-boundary" role="alert">
        <span className="robot-console-kicker">ROBOT CONTROL DETAIL</span>
        <h1>Robot Control could not be rendered</h1>
        <pre>{this.state.error.message}</pre>
        <button type="button" onClick={() => window.location.reload()}>RELOAD VIEW</button>
      </main>
    );
  }
}

export function RobotControlDetailPage({ robotId }: { robotId: string }) {
  return <RobotDetailErrorBoundary><RobotControlDetailContent robotId={robotId} /></RobotDetailErrorBoundary>;
}

function RobotControlDetailContent({ robotId }: { robotId: string }) {
  useSimulationRunner();

  const robot = useStore((state) => state.twin?.robots?.[robotId]);
  const robotIdsKey = useStore((state) => Object.keys(state.twin?.robots ?? {}).join("\u0000"));
  const robotIds = useMemo(() => robotIdsKey ? robotIdsKey.split("\u0000") : [], [robotIdsKey]);
  const currentMission = useStore((state) => {
    const taskId = state.twin?.robots?.[robotId]?.current_task_id;
    return taskId ? state.twin?.tasks?.[taskId] ?? null : null;
  });
  const select = useStore((state) => state.select);
  const rawDiagnostics = useStore((state) => state.rosDiagnostics);
  const diagnostics = rawDiagnostics as RobotSystemDiagnostics | null;
  const rawLocalization = useStore((state) => state.localization);
  const tagMission = useStore((state) => state.tagNavigation);
  const runtimeMode = useStore((state) => state.runtimeMode);
  const runtimeState = useStore((state) => state.runtimeState);
  const rosConnected = useStore((state) => state.rosConnected);
  const connectedRobotIds = useStore((state) => state.connectedRobotIds);
  const websocketState = useStore((state) => state.websocketState);
  const slam2dMap = useStore((state) => state.robotDetail[robotId]?.slam2dMap ?? null);
  const runtimeMapSnapshot = useStore((state) => state.robotDetail[robotId]?.runtimeMapSnapshot ?? null);
  const controller = useStore((state) => state.robotDetail[robotId]?.controller ?? null);
  const detailDiagnostics = useStore((state) => state.robotDetail[robotId]?.diagnostics ?? null);
  const detailErrors = useStore((state) => state.robotDetail[robotId]?.errors ?? EMPTY_ERRORS);
  const remainingDistanceM = useStore((state) => state.robotDetail[robotId]?.remainingDistanceM ?? null);
  const globalPath = useStore((state) => state.robotDetail[robotId]?.globalPath ?? null);
  const localPath = useStore((state) => state.robotDetail[robotId]?.localPath ?? null);
  const goal = useStore((state) => state.robotDetail[robotId]?.goal ?? null);
  const mappingScan = useStore((state) => state.robotDetail[robotId]?.scan ?? null);
  const mappingSessionId = useStore((state) => state.robotDetail[robotId]?.mappingSessionId ?? null);
  const navigationStatus = useStore((state) => state.robotDetail[robotId]?.navigationStatus ?? null);
  const slam3dAccumulatedCloud = useStore((state) => state.robotDetail[robotId]?.slam3dAccumulatedCloud ?? null);
  const viewStatus = useStore((state) => state.robotDetail[robotId]?.viewStatus ?? null);
  const lidarStreamDiagnostics = useStore((state) => state.robotDetail[robotId]?.lidarStreamDiagnostics ?? null);
  const pathPreview = useStore((state) => state.robotDetail[robotId]?.pathPreview ?? null);
  const activeLocalMapId = useStore((state) => state.robotDetail[robotId]?.activeLocalMapId ?? null);
  const activeLocalMapRevision = useStore((state) => state.robotDetail[robotId]?.activeLocalMapRevision ?? null);
  const localMapSyncStatus = useStore((state) => state.robotDetail[robotId]?.localMapSyncStatus ?? null);
  const activeMapSnapshot = runtimeState === "MAPPING" ? slam2dMap : runtimeMapSnapshot;
  const setRobotDetail = useStore((state) => state.setRobotDetail);
  const authToken = useStore((state) => state.authToken);
  const appliedMode = useStore((state) => state.robotDetail[robotId]?.appliedMode);
  const requestedMode = useStore((state) => state.robotDetail[robotId]?.requestedMode);
  const modeTransitionState = useStore((state) => state.robotDetail[robotId]?.modeTransitionState);
  const mapSync = useStore((state) => state.mapSync);
  const layoutRevision = useStore((state) => state.layoutRevision);
  const robotMapSync = mapSync.robots[robotId];
  const [controlMode, setControlMode] = useState<"MANUAL" | "AUTONOMOUS">(robot?.control_mode ?? "AUTONOMOUS");
  const [activeTab, setActiveTab] = useState<LocalTab>("CONTROL");
  const [mapSource, setMapSource] = useState<MapSource>("GLOBAL");
  const [lidarDimension, setLidarDimension] = useState<LidarDimension>("2D");
  const [targetMethod, setTargetMethod] = useState<NavigationTargetMethod>("MAP_POINT");
  const [tagRegistry, setTagRegistry] = useState<RobotNavigationTagRegistry | null>(null);
  const [tagRegistryState, setTagRegistryState] = useState<"IDLE" | "LOADING" | "READY" | "ERROR">("IDLE");
  const [tagRegistryError, setTagRegistryError] = useState("");
  const [selectedTagId, setSelectedTagId] = useState<number | null>(null);
  const [goalPreview, setGoalPreview] = useState<WorldGoal | null>(null);
  const [pathRequestState, setPathRequestState] = useState<"IDLE" | "PLANNING">("IDLE");
  const latestPathRequest = useRef("");
  const [error, setError] = useState("");
  const [busy, setBusy] = useState(false);
  const [host, setHost] = useState<HostStatus | null>(null);
  const [clockNow, setClockNow] = useState(() => Date.now());
  const manualTimer = useRef<number | null>(null);
  const manualActive = useRef(false);
  const manualWorker = useRef<Worker | null>(null);

  const robotBridgeOnline = connectedRobotIds.includes(robotId);
  const robotOnline = Boolean(robot && robot.status !== "OFFLINE" && (runtimeMode === "LOCAL_SIM" || (robotBridgeOnline && rosConnected && websocketState === "CONNECTED")));
  const controlOnline = Boolean(robotOnline && runtimeMode !== "LOCAL_SIM" && robotBridgeOnline && rosConnected && websocketState === "CONNECTED");
  const localization = diagnostics?.localization ?? rawLocalization?.state ?? null;
  const canonicalMapReady = !activeLocalMapId && (robotMapSync?.status ?? mapSync.status) === "SYNCED";
  const activeMappingSnapshot = runtimeState === "MAPPING" && slam2dMap?.map_source === "SLAM_TOOLBOX"
    && slam2dMap.frame_id === "map" && Boolean(slam2dMap.mapping_session_id)
    && slam2dMap.active_map_id === `SLAM-${slam2dMap.mapping_session_id}`
    && (!mappingSessionId || slam2dMap.mapping_session_id === mappingSessionId) ? slam2dMap : null;
  const activeMapId = runtimeState === "MAPPING" ? activeMappingSnapshot?.active_map_id ?? null
    : activeLocalMapId ?? (canonicalMapReady ? "CANONICAL" : activeMapSnapshot?.active_map_id ?? null);
  const activeMapRevision = runtimeState === "MAPPING" ? activeMappingSnapshot?.active_map_revision ?? null
    : activeLocalMapRevision ?? activeMapSnapshot?.active_map_revision
      ?? (robotMapSync?.rosRevision ?? mapSync.rosRevision)?.toString() ?? null;
  const activeMapStatus = runtimeState === "MAPPING" ? activeMappingSnapshot ? "SLAM · GOALS DISABLED" : "WAITING FOR SLAM MAP"
    : activeLocalMapId
      ? localMapSyncStatus ?? (activeLocalMapRevision ? "LOCAL_ONLY" : "LOADING")
      : canonicalMapReady ? "CANONICAL" : robotMapSync?.status ?? mapSync.status;
  const activeMapReady = runtimeState !== "MAPPING" && Boolean(activeMapId && activeMapRevision
    && ["CANONICAL", "LOCAL_ONLY"].includes(activeMapStatus ?? ""));
  const currentSlam2dMap = activeMappingSnapshot;
  const currentSlam3dCloud = runtimeState === "MAPPING" && slam3dAccumulatedCloud?.accumulated
    && slam3dAccumulatedCloud.accumulation_mode === "SLAM_VISUALIZATION_VOXEL_MAP"
    && slam3dAccumulatedCloud.frame_id === "map"
    && slam3dAccumulatedCloud.slam_pose?.valid
    && slam3dAccumulatedCloud.slam_pose.pose_source === "TF"
    && slam3dAccumulatedCloud.slam_pose.map_source === "SLAM_TOOLBOX"
    && slam3dAccumulatedCloud.slam_pose.frame_id === slam3dAccumulatedCloud.frame_id
    && Boolean(slam3dAccumulatedCloud.slam_pose.mapping_session_id)
    && slam3dAccumulatedCloud.slam_pose.map_id === `SLAM-${slam3dAccumulatedCloud.slam_pose.mapping_session_id}`
    && (!mappingSessionId || slam3dAccumulatedCloud.slam_pose.mapping_session_id === mappingSessionId)
    ? slam3dAccumulatedCloud : null;
  const statePose = robot?.active_map_pose ?? (runtimeMode === "LOCAL_SIM" && robot ? {
    x: robot.position[0], y: robot.position[2], yaw: robot.heading,
    frame_id: "LOCAL_SIM", map_id: "LOCAL_SIM",
  } : null);
  const selectedTag = tagRegistry?.tags.find((tag) => tag.tag_id === selectedTagId) ?? null;
  const compatibleTags = tagRegistry?.compatible ? tagRegistry.tags : [];
  const navigableTags = compatibleTags.filter((tag) => tag.navigable);
  const selectedMapTag = selectedTag && selectedTag.map_id === "CANONICAL"
    && selectedTag.map_revision === String(mapSync.publishedRevision ?? mapSync.rosRevision ?? layoutRevision)
    ? selectedTag : null;
  const candidatePreview = pathPreview?.request_id === latestPathRequest.current ? pathPreview : null;
  const previewAgeMs = candidatePreview?.timestamp ? clockNow - Date.parse(candidatePreview.timestamp) : Infinity;
  const previewTargetMatches = Boolean(candidatePreview?.goal && goalPreview
    && Math.abs(candidatePreview.goal.x - goalPreview.x) <= 1e-4
    && Math.abs(candidatePreview.goal.y - goalPreview.y) <= 1e-4
    && Math.abs(candidatePreview.goal.yaw - goalPreview.yaw) <= 1e-4);
  const previewSourceMatches = targetMethod === "TAG"
    ? Boolean(candidatePreview?.source_type === "TAG" && selectedTag
      && candidatePreview.tag_id === selectedTag.tag_id
      && candidatePreview.tag_revision === selectedTag.tag_revision
      && candidatePreview.registry_revision === tagRegistry?.registry_revision)
    : (candidatePreview?.source_type ?? "MAP_POINT") === "MAP_POINT";
  const approvedPreview: RobotDetailPathPreview | null = candidatePreview
    && candidatePreview.status === "VALID" && candidatePreview.path.length > 0
    && candidatePreview.active_map_id === activeMapId
    && candidatePreview.active_map_revision === activeMapRevision
    && previewTargetMatches && previewSourceMatches && Number.isFinite(previewAgeMs)
    && previewAgeMs >= -5_000 && previewAgeMs <= 120_000 ? candidatePreview : null;

  useEffect(() => {
    select(robotId);
    setGoalPreview(null);
    latestPathRequest.current = "";
    setPathRequestState("IDLE");
    setRobotDetail(robotId, { pathPreview: null });
  }, [robotId, select, setRobotDetail]);

  useEffect(() => {
    if (targetMethod !== "TAG") return;
    let current = true;
    setTagRegistry(null);
    setTagRegistryError("");
    setTagRegistryState("LOADING");
    setSelectedTagId(null);
    getRobotNavigationTags(robotId).then((registry) => {
      if (!current) return;
      if (registry.robot_id !== robotId) throw new Error("Tag registry response belongs to another robot");
      setTagRegistry(registry);
      setTagRegistryState("READY");
    }).catch((cause: unknown) => {
      if (!current) return;
      setTagRegistry(null);
      setTagRegistryError(cause instanceof Error ? cause.message : "Tag registry request failed");
      setTagRegistryState("ERROR");
    });
    return () => { current = false; };
  }, [activeMapId, activeMapRevision, robotId, targetMethod]);

  useEffect(() => {
    if (runtimeState === "MAPPING") setGoalPreview(null);
  }, [runtimeState]);

  useEffect(() => {
    setControlMode(appliedMode ?? robot?.control_mode ?? "AUTONOMOUS");
  }, [appliedMode, robot?.control_mode, robotId]);

  useEffect(() => {
    const timer = window.setInterval(() => setClockNow(Date.now()), 1000);
    return () => window.clearInterval(timer);
  }, []);

  const detailView: "GLOBAL" | "LIDAR_2D" | "LIDAR_3D" = mapSource === "GLOBAL" ? "GLOBAL" : `LIDAR_${lidarDimension}`;
  const visitedViews = useRef(new Set<string>());
  visitedViews.current.add(detailView);
  const viewFresh = detailView === "GLOBAL" || (detailView === "LIDAR_2D" && Boolean(currentSlam2dMap))
    || (detailView === "LIDAR_3D" && Boolean(currentSlam3dCloud))
    || (viewStatus?.requested_view === detailView && viewStatus.state === "FRESH");
  useEffect(() => {
    if (!robotBridgeOnline || websocketState !== "CONNECTED") return;
    const request_id = `${robotId}-${Date.now()}-${Math.random().toString(36).slice(2, 8)}`;
    setRobotDetail(robotId, { viewStatus: { robot_id: robotId, requested_view: detailView, request_id, state: "REQUESTED" } });
    wsSend({ type: "ROBOT_DETAIL_VIEW", robot_id: robotId, view: detailView, request_id, delivery_ack: true });
  }, [detailView, robotBridgeOnline, robotId, websocketState, setRobotDetail]);
  useLayoutEffect(() => {
    const cached = detailView === "GLOBAL" ? layout.floors.length > 0
      : detailView === "LIDAR_2D" ? occupancyRasters.peek(slam2dMap) : slam3dAccumulatedCloud;
    if (!cached) return;
    const paint = requestAnimationFrame(() => detailPerformance("view_render", { view: detailView, robot_id: robotId, useful: true, cached: true }));
    return () => cancelAnimationFrame(paint);
    // This measures a cached canvas becoming visible, not a newly received frame.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [detailView, robotId]);

  useEffect(() => {
    let cancelled = false;
    const loadHostStatus = async () => {
      try {
        const response = await apiFetch("/api/system/status/");
        if (!response.ok) throw new Error(String(response.status));
        const body = await response.json() as HostStatus;
        if (!cancelled) setHost(body);
      } catch {
        if (!cancelled) setHost(null);
      }
    };
    void loadHostStatus();
    const timer = window.setInterval(() => void loadHostStatus(), 5000);
    return () => { cancelled = true; window.clearInterval(timer); };
  }, []);

  const stopManual = useCallback(() => {
    if (manualTimer.current !== null) {
      window.clearInterval(manualTimer.current);
      manualTimer.current = null;
    }
    if (manualActive.current && controlOnline) {
      if (manualWorker.current) manualWorker.current.postMessage({ type: "STOP", robot_id: robotId });
      else wsManualCommand(robotId, "STOP");
    }
    manualActive.current = false;
  }, [controlOnline, robotId]);

  useEffect(() => () => stopManual(), [stopManual]);

  useEffect(() => {
    if (!controlOnline || !authToken || typeof Worker === "undefined") return;
    let worker: Worker;
    try {
      worker = new Worker(new URL("../../services/manualCommand.worker.ts", import.meta.url), { type: "module" });
    } catch {
      // Keep the existing WebSocket path as the compatibility fallback.
      return;
    }
    manualWorker.current = worker;
    worker.onmessage = (event: MessageEvent<{ type?: string; message?: string }>) => {
      if (event.data?.type !== "ERROR") return;
      if (manualTimer.current !== null) {
        window.clearInterval(manualTimer.current);
        manualTimer.current = null;
      }
      manualActive.current = false;
      setError(event.data.message || "Manual command refresh stopped safely");
    };
    worker.onerror = () => {
      if (manualWorker.current === worker) manualWorker.current = null;
      if (manualActive.current) wsManualCommand(robotId, "STOP");
      manualActive.current = false;
      setError("Manual refresh worker failed; the dead-man stop is active");
    };
    worker.postMessage({ type: "CONNECT", url: WS_URL, token: authToken });
    return () => {
      if (manualWorker.current === worker) manualWorker.current = null;
      worker.postMessage({ type: "DISCONNECT", robot_id: robotId });
      window.setTimeout(() => worker.terminate(), 150);
    };
  }, [authToken, controlOnline, robotId]);

  const estopActive = Boolean(detailDiagnostics?.command_ownership?.estop_active
    ?? diagnostics?.command_ownership?.estop_active);
  useEffect(() => {
    if (estopActive) stopManual();
  }, [estopActive, stopManual]);

  const runAction = useCallback(async (fn: () => Promise<unknown>) => {
    setBusy(true);
    setError("");
    try { await fn(); }
    catch (caught) { setError(caught instanceof Error ? caught.message : "Request failed"); }
    finally { setBusy(false); }
  }, []);

  const setMode = useCallback((next: "MANUAL" | "AUTONOMOUS") => {
    stopManual();
    latestPathRequest.current = "";
    setGoalPreview(null);
    setPathRequestState("IDLE");
    setRobotDetail(robotId, { pathPreview: null });
    if (!controlOnline) { setError("MANUAL/AUTONOMOUS requires an online ROS bridge"); return; }
    if (!wsSetRobotMode(robotId, next)) { setError("Robot control channel is disconnected"); return; }
    setRobotDetail(robotId, { requestedMode: next, modeTransitionState: "REQUESTED" });
    setError("");
  }, [controlOnline, robotId, setRobotDetail, stopManual]);

  const holdManual = useCallback((action: ManualAction) => {
    if (action === "STOP") {
      const wasActive = manualActive.current;
      stopManual();
      if (!wasActive && controlOnline) {
        if (manualWorker.current) manualWorker.current.postMessage({ type: "STOP", robot_id: robotId });
        else wsManualCommand(robotId, "STOP");
      }
      return;
    }
    if (controlMode !== "MANUAL" || modeTransitionState === "REQUESTED" || modeTransitionState === "FAILED") { setError("Wait for applied MANUAL mode before driving"); return; }
    if (!controlOnline) { setError("Manual control is disabled while ROS bridge is disconnected"); return; }
    if (manualWorker.current) {
      manualWorker.current.postMessage({ type: "HOLD", robot_id: robotId, action });
      manualActive.current = true;
      return;
    }
    if (!wsManualCommand(robotId, action)) { setError("Manual command was not sent"); return; }
    if (manualTimer.current !== null) window.clearInterval(manualTimer.current);
    manualActive.current = true;
    manualTimer.current = window.setInterval(() => {
      if (!wsManualCommand(robotId, action)) stopManual();
    }, 100);
  }, [controlMode, controlOnline, modeTransitionState, robotId, stopManual]);

  useEffect(() => {
    const keyActions: Record<string, ManualAction> = {
      w: "FORWARD", W: "FORWARD", ArrowUp: "FORWARD", s: "BACKWARD", S: "BACKWARD", ArrowDown: "BACKWARD",
      a: "LEFT", A: "LEFT", ArrowLeft: "LEFT", d: "RIGHT", D: "RIGHT", ArrowRight: "RIGHT", q: "ROTATE_LEFT", Q: "ROTATE_LEFT", e: "ROTATE_RIGHT", E: "ROTATE_RIGHT",
    };
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.code === "Space") {
        if (event.repeat || (event.target instanceof HTMLElement && ["INPUT", "SELECT", "TEXTAREA"].includes(event.target.tagName))) return;
        event.preventDefault();
        holdManual("STOP");
        return;
      }
      const action = keyActions[event.key];
      if (!action || event.repeat || (event.target instanceof HTMLElement && ["INPUT", "SELECT", "TEXTAREA"].includes(event.target.tagName))) return;
      event.preventDefault();
      holdManual(action);
    };
    const onKeyUp = (event: KeyboardEvent) => {
      if (event.code === "Space") { event.preventDefault(); return; }
      if (!keyActions[event.key]) return;
      event.preventDefault();
      stopManual();
    };
    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    return () => { window.removeEventListener("keydown", onKeyDown); window.removeEventListener("keyup", onKeyUp); stopManual(); };
  }, [holdManual, stopManual]);

  const sendGoal = () => {
    const approvedGoal = approvedPreview?.goal;
    if (!goalPreview || !approvedPreview || approvedPreview.status !== "VALID" || !approvedGoal) return;
    if (controlMode !== "AUTONOMOUS") { setError("Switch to AUTONOMOUS before sending a goal"); return; }
    if (!controlOnline) { setError("Navigation goal requires an online ROS bridge"); return; }
    if (!activeMapReady) { setError(`The selected robot active map is not confirmed (${activeMapStatus})`); return; }
    if (!wsSend({ type: "NAV_GOAL", robot_id: robotId, x: approvedGoal.x, y: approvedGoal.y, yaw: approvedGoal.yaw,
      frame_id: "map", preview_request_id: approvedPreview.request_id, active_map_id: activeMapId!,
      active_map_revision: activeMapRevision!, source_type: approvedPreview.source_type ?? "MAP_POINT",
      source_id: approvedPreview.source_id ?? undefined })) {
      setError("Navigation goal was not sent");
      return;
    }
    setGoalPreview(null);
    latestPathRequest.current = "";
    setPathRequestState("IDLE");
    setRobotDetail(robotId, { pathPreview: null });
    setError("");
  };

  const selectMapPoint = useCallback((target: WorldGoal) => {
    setSelectedTagId(null);
    wsSend({ type: "PATH_PREVIEW_INVALIDATE", robot_id: robotId });
    setGoalPreview(target);
    latestPathRequest.current = "";
    setPathRequestState("IDLE");
    setRobotDetail(robotId, { pathPreview: null });
    setError("");
  }, [robotId, setRobotDetail]);

  const selectTargetMethod = (method: NavigationTargetMethod) => {
    if (method === targetMethod) return;
    wsSend({ type: "PATH_PREVIEW_INVALIDATE", robot_id: robotId });
    latestPathRequest.current = "";
    setPathRequestState("IDLE");
    setGoalPreview(null);
    setSelectedTagId(null);
    setRobotDetail(robotId, { pathPreview: null });
    setTargetMethod(method);
    setTagRegistryError("");
    setError("");
  };

  const selectTag = (rawTagId: string) => {
    const tagId = rawTagId ? Number(rawTagId) : null;
    const tag = tagId === null ? null : compatibleTags.find((candidate) => candidate.tag_id === tagId);
    if (tagId !== null && (!tag || !tag.navigable)) return;
    wsSend({ type: "PATH_PREVIEW_INVALIDATE", robot_id: robotId });
    latestPathRequest.current = "";
    setPathRequestState("IDLE");
    setGoalPreview(tag?.navigation_pose ? { ...tag.navigation_pose } : null);
    setSelectedTagId(tagId);
    setRobotDetail(robotId, { pathPreview: null });
    setError("");
  };

  const requestPathPreview = useCallback((target: WorldGoal) => {
    setGoalPreview(target);
    setError("");
    if (targetMethod === "TAG" && (!selectedTag?.navigable || !tagRegistry?.registry_revision)) {
      latestPathRequest.current = "";
      setPathRequestState("IDLE");
      setRobotDetail(robotId, { pathPreview: null });
      setError("Select a valid Tag from the current compatible registry before previewing");
      return;
    }
    if (!activeMapReady) {
      latestPathRequest.current = "";
      setPathRequestState("IDLE");
      setRobotDetail(robotId, { pathPreview: null });
      setError(`Path planning is blocked: the selected robot active map is not confirmed (${activeMapStatus})`);
      return;
    }
    if (!controlOnline || controlMode !== "AUTONOMOUS") {
      latestPathRequest.current = "";
      setPathRequestState("IDLE");
      setRobotDetail(robotId, { pathPreview: null });
      setError("Switch to online AUTONOMOUS mode to request a Nav2 path preview");
      return;
    }
    const requestId = typeof crypto !== "undefined" && "randomUUID" in crypto
      ? crypto.randomUUID()
      : `${Date.now()}-${Math.random().toString(16).slice(2)}`;
    latestPathRequest.current = requestId;
    setPathRequestState("PLANNING");
    setRobotDetail(robotId, { pathPreview: null });
    const common = { type: "PATH_PREVIEW_REQUEST" as const, robot_id: robotId, request_id: requestId,
      frame_id: "map" as const, active_map_id: activeMapId!, active_map_revision: activeMapRevision! };
    const sent = targetMethod === "TAG" && selectedTag && tagRegistry?.registry_revision
      ? wsSend({ ...common, source_type: "TAG", tag_id: selectedTag.tag_id,
        tag_revision: selectedTag.tag_revision, registry_revision: tagRegistry.registry_revision })
      : wsSend({ ...common, source_type: "MAP_POINT", x: target.x, y: target.y, yaw: target.yaw });
    if (!sent) {
      latestPathRequest.current = "";
      setPathRequestState("IDLE");
      setError("Path preview request was not sent because the WebSocket is disconnected");
    }
  }, [activeMapId, activeMapReady, activeMapRevision, activeMapStatus, controlMode, controlOnline, robotId, selectedTag, setRobotDetail, tagRegistry?.registry_revision, targetMethod]);

  useEffect(() => {
    if (pathPreview?.request_id && pathPreview.request_id === latestPathRequest.current) setPathRequestState("IDLE");
  }, [pathPreview]);

  const cancelPathPreview = () => {
    wsSend({ type: "PATH_PREVIEW_INVALIDATE", robot_id: robotId });
    latestPathRequest.current = "";
    setGoalPreview(null);
    setSelectedTagId(null);
    setPathRequestState("IDLE");
    setRobotDetail(robotId, { pathPreview: null });
  };

  const navCommand = (type: "NAV_CANCEL" | "NAV_PAUSE" | "NAV_RESUME") => {
    if (!controlOnline) { setError("Navigation control requires an online ROS bridge"); return; }
    if (!wsSend({ type, robot_id: robotId })) setError("Navigation command was not sent");
  };

  const changeRobot = (next: string) => {
    if (next) pushRoute(`/robots/${encodeURIComponent(next)}/control`);
  };

  const moveButtonEvents = useCallback((action: ManualAction) => ({
    onPointerDown: (event: React.PointerEvent<HTMLButtonElement>) => {
      event.currentTarget.setPointerCapture?.(event.pointerId);
      holdManual(action);
    },
    onPointerUp: stopManual,
    onPointerLeave: stopManual,
    onPointerCancel: stopManual,
    onLostPointerCapture: stopManual,
  }), [holdManual, stopManual]);

  return (
    <div className="robot-detail-shell">
      <header className="robot-detail-header">
        <div className="robot-detail-identity">
          <button type="button" className="robot-detail-back" onClick={() => pushRoute("/")}>← BACK</button>
          <div><span className="robot-console-kicker">ROBOT CONTROL CONSOLE</span><h1>{robotId}</h1></div>
          <StatusValue value={robotOnline ? "ONLINE" : "OFFLINE"} />
          <span className="robot-detail-mode">{controlMode}{modeTransitionState === "REQUESTED" ? ` → ${requestedMode} REQUESTED` : modeTransitionState === "FAILED" ? " TRANSITION FAILED" : " APPLIED"}</span>
          <span className="robot-detail-runtime">{runtimeState} / {safeText(robot?.navigation_state, "N/A")}</span>
          <span className="robot-detail-mission">{currentMission ? `${currentMission.id} · ${currentMission.status}` : "mission N/A"}</span>
          <span className="robot-detail-latency">connection latency {safeNumber(diagnostics?.websocket_latency_ms, 0, " ms")}</span>
        </div>
        <div className="robot-detail-header-actions">
          <label className="robot-detail-robot-select"><span>ROBOT</span><select aria-label="Select robot" value={robotId} onChange={(event) => changeRobot(event.target.value)}><option value="">Select robot</option>{robotIds.map((id) => <option key={id} value={id}>{id}</option>)}</select></label>
          <button type="button" className={controlMode === "MANUAL" ? "is-active" : ""} disabled={!controlOnline || busy} onClick={() => setMode("MANUAL")}>MANUAL</button>
          <button type="button" className={controlMode === "AUTONOMOUS" ? "is-active" : ""} disabled={!controlOnline || busy} onClick={() => setMode("AUTONOMOUS")}>AUTONOMOUS</button>
          <button type="button" className="robot-console-danger" disabled={busy || !robotId} onClick={() => { stopManual(); void runAction(() => emergencyStop(robotId)); }}>EMERGENCY STOP</button>
        </div>
      </header>

      <nav className="robot-local-tabs" role="tablist" aria-label="Local robot control sections">
        {LOCAL_TABS.map((tab) => <button key={tab} type="button" role="tab" aria-selected={activeTab === tab} className={activeTab === tab ? "is-active" : ""} onClick={() => setActiveTab(tab)}>{tab}</button>)}
      </nav>

      {activeTab !== "CONTROL" ? <main className="robot-detail-section-main">
        <LocalRobotSection section={activeTab} robotId={robotId} robot={robot} slam2dMap={slam2dMap} runtimeMapSnapshot={runtimeMapSnapshot} localizationMap={activeLocalMapId ? runtimeMapSnapshot : runtimeState === "MAPPING" ? slam2dMap : runtimeMapSnapshot} scan={mappingScan} diagnostics={detailDiagnostics ?? diagnostics} errors={detailErrors.length ? detailErrors : detailDiagnostics?.errors ?? diagnostics?.errors ?? EMPTY_ERRORS} controlOnline={controlOnline} controlMode={controlMode} runtimeMode={runtimeMode} runtimeState={runtimeState} localization={localization} websocketState={websocketState} mapRevision={mapSync.publishedRevision} activeLocalMapId={activeLocalMapId} activeLocalMapRevision={activeLocalMapRevision} localMapSyncStatus={localMapSyncStatus} lidarStreamDiagnostics={lidarStreamDiagnostics} mappingSessionId={mappingSessionId} />
      </main> : <main className="robot-detail-main">
        <aside className="robot-detail-column robot-detail-left">
          <SystemInputsPanel robotId={robotId} robot={robot} controlMode={controlMode} runtimeMode={runtimeMode} goal={goalPreview ?? goal} mission={tagMission?.robot_id === robotId ? tagMission : null} />
          <StatePanel pose={statePose} robot={robot} localization={localization} diagnostics={detailDiagnostics ?? diagnostics} controller={controller} navigationStatus={navigationStatus} />
        </aside>

        <section className="robot-detail-map-panel">
          <div className="robot-map-source-bar">
            <div role="tablist" aria-label="Robot map view">
              <button type="button" role="tab" aria-selected={mapSource === "GLOBAL"} className={mapSource === "GLOBAL" ? "is-active" : ""} onClick={() => setMapSource("GLOBAL")}>GLOBAL MAP</button>
              <button type="button" role="tab" aria-selected={mapSource === "LIDAR" && lidarDimension === "2D"} className={mapSource === "LIDAR" && lidarDimension === "2D" ? "is-active" : ""} onClick={() => { setMapSource("LIDAR"); setLidarDimension("2D"); }}>MAP VIEW 2D</button>
              <button type="button" role="tab" aria-selected={mapSource === "LIDAR" && lidarDimension === "3D"} className={mapSource === "LIDAR" && lidarDimension === "3D" ? "is-active" : ""} onClick={() => { setMapSource("LIDAR"); setLidarDimension("3D"); }}>MAP VIEW 3D</button>
            </div>
            <span className="map-sync-warning" data-view-state={viewFresh ? "FRESH" : viewStatus?.state ?? "REQUESTED"}>{viewFresh ? "LIVE" : "WAITING FOR FRESH FRAME"}</span>
            <span className={`map-sync-warning ${activeMapReady ? "map-sync-warning-ready" : ""}`}>{activeMapId ? `${activeMapId} · r${activeMapRevision ?? "—"} · ${activeMapStatus}` : `MAP ${activeMapStatus} · LOCAL GOALS BLOCKED`}</span>
            {activeLocalMapId && runtimeState !== "MAPPING" && <span className="map-sync-warning">LOCAL MAP DIFFERS FROM CANONICAL r{mapSync.publishedRevision ?? "—"} · LOCAL NAV ONLY</span>}
          </div>
          <div className="robot-detail-target-toolbar">
            <span>NAVIGATION TARGET</span>
            <div role="group" aria-label="Navigation target method">
              <button type="button" className={targetMethod === "MAP_POINT" ? "is-active" : ""} aria-pressed={targetMethod === "MAP_POINT"} onClick={() => selectTargetMethod("MAP_POINT")}>MAP POINT</button>
              <button type="button" className={targetMethod === "TAG" ? "is-active" : ""} aria-pressed={targetMethod === "TAG"} onClick={() => selectTargetMethod("TAG")}>TAG</button>
            </div>
            {targetMethod === "TAG" && <label className="robot-detail-tag-select">
              <span>DESTINATION TAG</span>
              <select aria-label="Destination Tag" value={selectedTagId ?? ""}
                disabled={tagRegistryState !== "READY" || !tagRegistry?.compatible || compatibleTags.length === 0}
                onChange={(event) => selectTag(event.currentTarget.value)}>
                <option value="">{tagRegistryState === "LOADING" ? "Loading Tags…" : "Select a Tag…"}</option>
                {compatibleTags.map((tag) => <option key={tag.tag_id} value={tag.tag_id} disabled={!tag.navigable}>
                  {tag.tag_id} — {tag.label}{tag.navigable ? "" : " · DISABLED"}
                </option>)}
              </select>
            </label>}
          </div>
          {targetMethod === "TAG" && <div className="robot-detail-tag-status" role="status" aria-live="polite">
            {tagRegistryState === "LOADING" && <span>Loading authoritative Tags for {activeMapId ?? "the active map"}…</span>}
            {tagRegistryState === "ERROR" && <span className="is-error">Tag registry unavailable: {tagRegistryError}</span>}
            {tagRegistryState === "READY" && !tagRegistry?.compatible && <span className="is-error">Tags unavailable: {tagRegistry?.reason ?? "active map is incompatible"}</span>}
            {tagRegistryState === "READY" && tagRegistry?.compatible && compatibleTags.length === 0 && <span>No Tags are registered for this active map.</span>}
            {tagRegistryState === "READY" && tagRegistry?.compatible && compatibleTags.length > 0 && navigableTags.length === 0 && <span className="is-error">No enabled, valid Tags are navigable on this map.</span>}
            {selectedTag && selectedTag.navigation_pose && <div className="robot-detail-tag-summary" data-testid="selected-navigation-tag" data-tag-id={selectedTag.tag_id}>
              <span>TAG ID <b>{selectedTag.tag_id}</b></span>
              <span>LABEL <b>{selectedTag.label}</b></span>
              <span>TYPE <b>{selectedTag.family}</b></span>
              <span>MAP <b>{selectedTag.map_id}</b></span>
              <span>REVISION <b>{selectedTag.map_revision}</b></span>
              <span>X <b>{safeNumber(selectedTag.navigation_pose.x, 3)}</b></span>
              <span>Y <b>{safeNumber(selectedTag.navigation_pose.y, 3)}</b></span>
              <span>YAW <b>{safeNumber(selectedTag.navigation_pose.yaw, 3)} rad</b></span>
            </div>}
          </div>}
          <div className="robot-detail-view-stack" key={robotId}>
            <div className={"robot-detail-view-layer " + (detailView === "GLOBAL" ? "is-active" : "")} data-view="GLOBAL" aria-hidden={detailView !== "GLOBAL"}><DetailMapCanvas active={detailView === "GLOBAL"} robotId={robotId} robot={robot} canonicalRevision={mapSync.publishedRevision ?? mapSync.rosRevision ?? layoutRevision} globalPath={globalPath} localPath={localPath} goal={goal} goalPreview={goalPreview} pathPreview={approvedPreview} selectedTag={selectedMapTag} onGoalSelect={selectMapPoint} canPick={targetMethod === "MAP_POINT" && runtimeState !== "MAPPING" && activeMapId === "CANONICAL" && !activeLocalMapId} /></div>
            {visitedViews.current.has("LIDAR_2D") && <div className={"robot-detail-view-layer " + (detailView === "LIDAR_2D" ? "is-active" : "")} data-view="LIDAR_2D" aria-hidden={detailView !== "LIDAR_2D"}><AccumulatedSlamMap2DView map={currentSlam2dMap} robot={robot} scan={mappingScan} /></div>}
            {visitedViews.current.has("LIDAR_3D") && <div className={"robot-detail-view-layer " + (detailView === "LIDAR_3D" ? "is-active" : "")} data-view="LIDAR_3D" aria-hidden={detailView !== "LIDAR_3D"}><RobotLidar3DView active={detailView === "LIDAR_3D"} frame={currentSlam3dCloud} robot={robot} slamMap={currentSlam2dMap} /></div>}
          </div>
          <div className="robot-detail-goal-toolbar">
            <span>{targetMethod === "TAG" ? selectedTag ? `TAG ${selectedTag.tag_id} SELECTED · ${pathRequestState === "PLANNING" ? "PLANNING" : approvedPreview?.status === "VALID" ? `PREVIEW VALID · ${safeNumber(approvedPreview.path_length_m, 2, " m")}` : candidatePreview?.status === "INVALID" || candidatePreview?.status === "NO_PATH" ? candidatePreview.reason ?? candidatePreview.status : "PATH PREVIEW REQUIRED"}` : "Select a compatible destination Tag" : goalPreview ? `TARGET ${safeNumber(goalPreview.x, 2)} / ${safeNumber(goalPreview.y, 2)} / ${safeNumber(goalPreview.yaw, 2)} rad · ${pathRequestState === "PLANNING" ? "PLANNING" : approvedPreview?.status ?? "NO PREVIEW"}${approvedPreview?.status === "VALID" ? ` · ${safeNumber(approvedPreview.path_length_m, 2, " m")}` : approvedPreview?.reason ? ` · ${approvedPreview.reason}` : ""}` : mapSource === "LIDAR" && lidarDimension === "3D" ? "Use GLOBAL MAP or LIDAR 2D to select a target" : "Select destination, then preview the Nav2 path"}</span>
            <button type="button" disabled={targetMethod === "TAG" || !goalPreview || pathRequestState === "PLANNING"} onClick={() => goalPreview && selectMapPoint({ ...goalPreview, yaw: goalPreview.yaw - Math.PI / 12 })}>YAW −</button>
            <button type="button" disabled={targetMethod === "TAG" || !goalPreview || pathRequestState === "PLANNING"} onClick={() => goalPreview && selectMapPoint({ ...goalPreview, yaw: goalPreview.yaw + Math.PI / 12 })}>YAW +</button>
            <button type="button" disabled={(!goalPreview || targetMethod === "TAG" && !selectedTag?.navigable) || pathRequestState === "PLANNING"} onClick={() => goalPreview && requestPathPreview(goalPreview)}>PREVIEW PATH</button>
            <button type="button" disabled={!goalPreview || !approvedPreview || !controlOnline || controlMode !== "AUTONOMOUS" || !activeMapReady || runtimeState !== "NAVIGATION"} className="robot-console-primary" onClick={sendGoal}>SEND GOAL</button>
            <button type="button" disabled={!goalPreview && selectedTagId === null} onClick={cancelPathPreview}>CANCEL</button>
          </div>
        </section>

        <aside className="robot-detail-column robot-detail-right">
          <SystemPanel diagnostics={detailDiagnostics ?? diagnostics} rosConnected={rosConnected} websocketState={websocketState} host={host} runtimeState={runtimeState} />
          <LidarPanel robotId={robotId} diagnostics={detailDiagnostics ?? diagnostics} />
          <ErrorMessagesPanel errors={detailErrors.length ? detailErrors : detailDiagnostics?.errors ?? diagnostics?.errors ?? EMPTY_ERRORS} />
        </aside>
      </main>
      }

      <footer className="robot-detail-bottom">
        <ManualBar controlMode={controlMode} controlOnline={controlOnline} holdManual={holdManual} moveButtonEvents={moveButtonEvents} setMode={setMode} />
        <div className="robot-detail-nav-bar">
          <div><span>NAV STATUS</span><b>{safeText(navigationStatus ?? robot?.navigation_state, "N/A")}</b></div>
          <div><span>GOAL</span><b>{goal ? `${safeNumber(goal.x, 2)} / ${safeNumber(goal.y, 2)}` : "N/A"}</b></div>
          <div><span>REMAINING</span><b>{safeNumber(remainingDistanceM, 2, " m")}</b></div>
          <button type="button" disabled={!controlOnline || controlMode !== "AUTONOMOUS"} onClick={() => navCommand("NAV_PAUSE")}>PAUSE</button>
          <button type="button" disabled={!controlOnline || controlMode !== "AUTONOMOUS"} onClick={() => navCommand("NAV_RESUME")}>RESUME</button>
          <button type="button" className="robot-console-danger-outline" disabled={!controlOnline} onClick={() => navCommand("NAV_CANCEL")}>CANCEL NAV</button>
          <button type="button" disabled={busy || !robotId} onClick={() => void runAction(() => clearEmergencyStop(robotId))}>CLEAR STOP</button>
        </div>
        {error && <div className="robot-detail-command-error" role="alert">{error}</div>}
      </footer>
    </div>
  );
}

function SystemInputsPanel({ robotId, robot, controlMode, runtimeMode, goal, mission }: { robotId: string; robot?: RobotState; controlMode: string; runtimeMode: string; goal: RobotDetailGoal | WorldGoal | null; mission: { target_tag_id?: number | null } | null }) {
  return <Panel title="SYSTEM INPUTS">
    <Metric label="Robot" value={robotId} mono />
    <Metric label="Control mode" value={controlMode} status />
    <Metric label="Runtime mode" value={runtimeMode} />
    <Metric label="Navigation mode" value={safeText(robot?.navigation_state)} />
    <Metric label="Target goal" value={goal ? `${safeNumber(goal.x, 2)} / ${safeNumber(goal.y, 2)} / ${safeNumber(goal.yaw, 2)}` : "N/A"} mono />
    <Metric label="Target tag" value={mission?.target_tag_id ?? "N/A"} mono />
    <Metric label="Current command" value="N/A" />
    <div className="robot-detail-subtitle">COMMAND VELOCITY · measured</div>
    <Metric label="linear x" value={safeNumber(robot?.vx, 3, " m/s")} mono />
    <Metric label="linear y" value={safeNumber(robot?.vy, 3, " m/s")} mono />
    <Metric label="angular z" value={safeNumber(robot?.wz, 3, " rad/s")} mono />
    <div className="robot-detail-subtitle">GOAL</div>
    <Metric label="x" value={goal ? safeNumber(goal.x, 3, " m") : "N/A"} mono />
    <Metric label="y" value={goal ? safeNumber(goal.y, 3, " m") : "N/A"} mono />
    <Metric label="yaw" value={goal ? safeNumber(goal.yaw, 3, " rad") : "N/A"} mono />
  </Panel>;
}

function StatePanel({ pose, robot, localization, diagnostics, controller, navigationStatus }: { pose: { x: number; y: number; yaw: number; frame_id: string; map_id: string } | null; robot?: RobotState; localization: unknown; diagnostics: RobotSystemDiagnostics | null; controller: { controllers: Array<{ name: string; state: string }> } | null; navigationStatus: string | null }) {
  const controllerValue = controller?.controllers.length ? (controller.controllers.every((item) => item.state === "active") ? "ACTIVE" : "ERROR") : diagnostics?.controller_manager ? "ACTIVE" : "N/A";
  return <Panel title="STATE">
    <div className="robot-detail-subtitle">ACTIVE MAP POSE · {pose ? `${pose.frame_id} / ${pose.map_id}` : "WAITING"}</div>
    <Metric label="x" value={safeNumber(pose?.x, 3, " m")} mono />
    <Metric label="y" value={safeNumber(pose?.y, 3, " m")} mono />
    <div className="robot-detail-subtitle">ORIENTATION · active map</div>
    <Metric label="yaw" value={safeNumber(pose?.yaw, 3, " rad")} mono />
    <div className="robot-detail-subtitle">VELOCITY</div>
    <Metric label="vx" value={safeNumber(robot?.vx, 3, " m/s")} mono />
    <Metric label="vy" value={safeNumber(robot?.vy, 3, " m/s")} mono />
    <Metric label="wz" value={safeNumber(robot?.wz, 3, " rad/s")} mono />
    <div className="robot-detail-subtitle">LOCALIZATION / TF</div>
    <Metric label="Localization" value={safeText(localization)} status />
    <Metric label="map → odom" value={diagnostics?.tf ? "AVAILABLE" : "N/A"} status />
    <Metric label="odom → base_link" value={diagnostics?.tf ? "AVAILABLE" : "N/A"} status />
    <Metric label="Controller" value={controllerValue} status />
    <Metric label="Navigation" value={safeText(navigationStatus)} status />
  </Panel>;
}

const SystemPanel = memo(SystemPanelContent);

function SystemPanelContent({ diagnostics, rosConnected, websocketState, host, runtimeState }: { diagnostics: RobotSystemDiagnostics | null; rosConnected: boolean; websocketState: string; host: HostStatus | null; runtimeState: string }) {
  const metrics = diagnostics?.metrics ?? {};
  const cpu = host?.system?.cpu_load_1m;
  const ram = host?.system?.memory?.used_percent;
  return <Panel title="SYSTEM">
    <Metric label="ROS" value={diagnostics ? (diagnostics.ros ? "OK" : "OFFLINE") : "N/A"} status />
    <Metric label="Gazebo" value={diagnostics ? (diagnostics.gazebo ? "OK" : "OFFLINE") : "N/A"} status />
    <Metric label="ROS bridge" value={rosConnected ? "CONNECTED" : "DISCONNECTED"} status />
    <Metric label="WebSocket" value={websocketState} status />
    <Metric label="controller_manager" value={diagnostics ? (diagnostics.controller_manager ? "ACTIVE" : "N/A") : "N/A"} status />
    <Metric label="SLAM / Nav2" value={diagnostics ? `${diagnostics.slam ? "SLAM" : "N/A"} / ${diagnostics.nav2 ? "Nav2" : "N/A"}` : "N/A"} />
    <Metric label="CPU" value={cpu == null ? "N/A" : safeNumber(cpu, 2)} mono />
    <Metric label="RAM" value={ram == null ? "N/A" : safeNumber(ram, 1, "%")} mono />
    <Metric label="Simulation time" value={diagnostics?.simulation_time == null ? "N/A" : safeNumber(diagnostics.simulation_time, 3, " s")} mono />
    <Metric label="Gazebo RTF" value={diagnostics?.gazebo_rtf == null && metrics.gazebo_rtf == null ? "N/A" : safeNumber(diagnostics?.gazebo_rtf ?? metrics.gazebo_rtf, 3)} mono />
    <Metric label="WS latency" value={diagnostics?.websocket_latency_ms == null ? "N/A" : safeNumber(diagnostics.websocket_latency_ms, 0, " ms")} mono />
    <Metric label="Runtime" value={runtimeState} />
  </Panel>;
}

const LidarPanel = memo(LidarPanelContent);

function LidarPanelContent({ robotId, diagnostics }: { robotId: string; diagnostics: RobotSystemDiagnostics | null }) {
  const scan = useStore((state) => state.robotDetail[robotId]?.scan ?? null);
  const age = scan?.timestamp ? (Date.now() - Date.parse(scan.timestamp)) / 1000 : null;
  const lidarStatus = !scan
    ? diagnostics ? (diagnostics.lidar ? "WAITING" : "ERROR") : "N/A"
    : diagnostics?.lidar === false ? "ERROR" : age !== null && age > 3 ? "STALE" : "OK";
  return <Panel title="LIDAR OUTPUTS">
    <Metric label="topic" value={scan?.topic ?? "/scan"} mono />
    <Metric label="frame" value={scan?.source_frame_id ?? scan?.frame_id ?? "N/A"} mono />
    <Metric label="scan Hz sim" value={scan?.scan_hz_sim == null ? "N/A" : safeNumber(scan.scan_hz_sim, 2, " Hz")} mono />
    <Metric label="scan Hz wall" value={scan?.scan_hz_wall == null ? "N/A" : safeNumber(scan.scan_hz_wall, 2, " Hz")} mono />
    <Metric label="point count" value={scan?.point_count ?? "N/A"} mono />
    <Metric label="minimum range" value={scan?.minimum_range == null ? "N/A" : safeNumber(scan.minimum_range, 2, " m")} mono />
    <Metric label="maximum range" value={scan?.maximum_range == null ? "N/A" : safeNumber(scan.maximum_range, 2, " m")} mono />
    <Metric label="last update" value={scan?.timestamp ?? "N/A"} mono />
    <Metric label="age" value={age == null || !Number.isFinite(age) ? "N/A" : safeNumber(Math.max(0, age), 2, " s")} mono />
    <Metric label="status" value={lidarStatus} status />
  </Panel>;
}

const ErrorMessagesPanel = memo(ErrorMessagesPanelContent);

function ErrorMessagesPanelContent({ errors }: { errors: RobotDetailError[] }) {
  return <Panel title="ERROR MESSAGES" className="robot-detail-errors">
    {errors.length === 0 && <div className="robot-detail-no-errors">No errors reported</div>}
    {errors.map((item, index) => <div className={`robot-detail-error-row ${statusClass(item.severity)}`} key={`${item.code ?? item.message}-${index}`}><div><b>{item.severity}</b><span>{item.message}</span></div><small>{item.timestamp ?? "N/A"}</small></div>)}
  </Panel>;
}

const ManualBar = memo(ManualBarContent);

function ManualBarContent({ controlMode, controlOnline, holdManual, moveButtonEvents, setMode }: { controlMode: string; controlOnline: boolean; holdManual: (action: ManualAction) => void; moveButtonEvents: (action: ManualAction) => Record<string, (event: React.PointerEvent<HTMLButtonElement>) => void>; setMode: (mode: "MANUAL" | "AUTONOMOUS") => void }) {
  return <section className="robot-detail-manual">
    <div className="robot-detail-manual-head"><div><span className="robot-console-kicker">MANUAL CONTROL</span><b>DEAD-MAN ENABLED</b><small>Release key/button → STOP · W/S/A/D · Q/E · arrows · Space STOP</small></div><div className="robot-detail-manual-mode"><button type="button" className={controlMode === "MANUAL" ? "is-active" : ""} disabled={!controlOnline} onClick={() => setMode("MANUAL")}>MANUAL</button><button type="button" className={controlMode === "AUTONOMOUS" ? "is-active" : ""} disabled={!controlOnline} onClick={() => setMode("AUTONOMOUS")}>AUTONOMOUS</button></div></div>
    <div className="robot-detail-manual-pad">{MANUAL_ACTIONS.map((item) => <button type="button" key={item.action} className={`manual-key manual-key-${item.action.toLowerCase()}`} title={item.title} aria-label={item.title} disabled={!controlOnline || controlMode !== "MANUAL"} {...moveButtonEvents(item.action)} onClick={item.action === "STOP" ? () => holdManual("STOP") : undefined}>{item.label}<small>{item.action === "FORWARD" ? "W / ↑" : item.action === "BACKWARD" ? "S / ↓" : item.action === "LEFT" ? "A / ←" : item.action === "RIGHT" ? "D / →" : item.action === "ROTATE_LEFT" ? "Q" : item.action === "ROTATE_RIGHT" ? "E" : "STOP"}</small></button>)}</div>
  </section>;
}

function Panel({ title, children, className = "" }: { title: string; children: ReactNode; className?: string }) {
  return <section className={`robot-detail-panel ${className}`}><header>{title}</header><div className="robot-detail-panel-body">{children}</div></section>;
}

const Metric = memo(MetricContent);

function MetricContent({ label, value, mono = false, status = false }: { label: string; value: unknown; mono?: boolean; status?: boolean }) {
  return <div className="robot-detail-metric"><span>{label}</span>{status ? <StatusValue value={value} /> : <b className={mono ? "mono" : ""}>{safeText(value)}</b>}</div>;
}

type MapCanvasProps = { active?: boolean; robotId: string; robot?: RobotState; canonicalRevision: string | number; globalPath: RobotDetailPath | null; localPath: RobotDetailPath | null; goal: RobotDetailGoal | null; goalPreview: WorldGoal | null; pathPreview: RobotDetailPathPreview | null; selectedTag: RobotNavigationTag | null; onGoalSelect: (goal: WorldGoal) => void; canPick: boolean };

function DetailMapCanvas({ active = true, robotId, robot, canonicalRevision, globalPath, localPath, goal, goalPreview, pathPreview, selectedTag, onGoalSelect, canPick }: MapCanvasProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const hostRef = useRef<HTMLDivElement>(null);
  const [size, setSize] = useState({ width: 0, height: 0 });
  const [zoom, setZoom] = useState(1);
  const [center, setCenter] = useState<{ x: number; y: number } | null>(null);
  const [follow, setFollow] = useState(true);
  const [showGrid, setShowGrid] = useState(true);
  const [showPaths, setShowPaths] = useState(true);
  const layoutRevision = useStore((state) => state.layoutRevision);
  const runtimeState = useStore((state) => state.runtimeState);

  useEffect(() => {
    setFollow(true);
    setCenter(null);
    setZoom(1);
  }, [robotId]);

  useEffect(() => {
    const host = hostRef.current;
    if (!host) return;
    const resize = () => setSize({ width: Math.round(host.clientWidth), height: Math.round(host.clientHeight) });
    resize();
    const observer = typeof ResizeObserver === "undefined" ? null : new ResizeObserver(resize);
    observer?.observe(host);
    window.addEventListener("resize", resize);
    return () => { observer?.disconnect(); window.removeEventListener("resize", resize); };
  }, []);

  const bounds = useMemo(() => worldBounds(null, layout), [layoutRevision]);
  const displayedMapIdentity = useMemo<MapPoseIdentity>(() => ({
    frame_id: layout.coordinate_system?.frame ?? "map",
    active_map_id: "CANONICAL",
    active_map_revision: canonicalRevision,
    map_source: "CANONICAL",
  }), [canonicalRevision, layoutRevision]);
  const displayedPose = useStableDisplayedFramePose(robot, displayedMapIdentity);
  const transform = useMemo(() => makeTransform(size.width, size.height, bounds, zoom, center, follow, displayedPose), [bounds, center, follow, displayedPose, size.height, size.width, zoom]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!active || !canvas || size.width <= 0 || size.height <= 0) return;
    const dpr = Math.max(1, Math.min(2, window.devicePixelRatio || 1));
    if (canvas.width !== Math.round(size.width * dpr)) canvas.width = Math.round(size.width * dpr);
    if (canvas.height !== Math.round(size.height * dpr)) canvas.height = Math.round(size.height * dpr);
    canvas.style.width = `${size.width}px`;
    canvas.style.height = `${size.height}px`;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const canonicalPaths = runtimeState === "MAPPING" ? false : showPaths;
    drawDetailMap(ctx, size.width, size.height, transform, bounds, null, null, displayedPose, robot,
      canonicalPaths ? globalPath : null, canonicalPaths ? localPath : null, canonicalPaths ? goal : null,
      canonicalPaths ? goalPreview : null, canonicalPaths ? pathPreview : null,
      { showGrid, showLidar: false, showPaths: canonicalPaths, showWarehouse: true });
    if (selectedTag?.navigation_pose) drawSelectedTag(ctx, selectedTag, transform.toCanvas);
    detailPerformance("view_render", { view: "GLOBAL", useful: true, robot_id: robotId });
  }, [active, bounds, goal, goalPreview, globalPath, localPath, pathPreview, selectedTag, displayedPose, robot, runtimeState, showGrid, showPaths, size.height, size.width, transform]);

  const handleMapClick = (event: ReactMouseEvent<HTMLCanvasElement>) => {
    if (!canPick) return;
    const rect = event.currentTarget.getBoundingClientRect();
    const point = transform.toWorld(event.clientX - rect.left, event.clientY - rect.top);
    if (!Number.isFinite(point.x) || !Number.isFinite(point.y)) return;
    onGoalSelect({ x: point.x, y: point.y, yaw: displayedPose?.yaw ?? 0 });
    setFollow(false);
  };

  const fit = () => { setZoom(1); setCenter(null); setFollow(false); };
  const recenter = () => { setFollow(true); setCenter(null); };
  return <div className="robot-detail-map-host" ref={hostRef}>
    <canvas ref={canvasRef} className="robot-detail-map-canvas" data-testid="global-warehouse-map"
      data-map-source="CANONICAL_WAREHOUSE" data-pose-source={displayedPose?.pose_source}
      data-render-x={displayedPose?.x} data-render-y={displayedPose?.y}
      data-render-yaw={displayedPose?.yaw}
      data-selected-tag-id={selectedTag?.tag_id}
      onClick={handleMapClick} aria-label="Canonical warehouse and robot pose map" />
    <div className="robot-detail-map-toolbar" role="toolbar" aria-label="Map controls">
      <button type="button" onClick={() => setZoom((value) => Math.min(8, value * 1.25))} aria-label="Zoom in">+</button>
      <button type="button" onClick={() => setZoom((value) => Math.max(0.25, value / 1.25))} aria-label="Zoom out">−</button>
      <button type="button" onClick={fit}>FIT</button>
      <button type="button" onClick={recenter}>RECENTER</button>
      <button type="button" className={follow ? "is-active" : ""} onClick={() => setFollow((value) => !value)}>FOLLOW</button>
      <button type="button" className={showGrid ? "is-active" : ""} onClick={() => setShowGrid((value) => !value)}>GRID</button>
      <button type="button" className={showPaths ? "is-active" : ""} onClick={() => setShowPaths((value) => !value)}>PATH</button>
    </div>
    <div className="robot-detail-map-readout"><span>FRAME {layout.coordinate_system?.frame ?? "map"}</span><span>CANONICAL WAREHOUSE · r{canonicalRevision}</span><span>{displayedPose ? "CANONICAL_POSE" : "ROBOT POSE WAITING"}</span></div>
  </div>;
}

type MapBounds = WorldBounds;
type MapTransform = WorldTransform & { toCanvas: (x: number, y: number) => { x: number; y: number }; toWorld: (x: number, y: number) => { x: number; y: number } };

function worldBounds(mapSnapshot: RobotDetailMapSnapshot | null, mapLayout: WarehouseLayout): MapBounds {
  if (mapSnapshot?.frame_id === "map" && mapSnapshot.width > 0 && mapSnapshot.height > 0 && mapSnapshot.resolution > 0) {
    const mapWidth = mapSnapshot.width * mapSnapshot.resolution;
    const mapHeight = mapSnapshot.height * mapSnapshot.resolution;
    const yaw = mapSnapshot.origin.yaw;
    const corners = [[0, 0], [mapWidth, 0], [mapWidth, mapHeight], [0, mapHeight]].map(([x, y]) => ({
      x: mapSnapshot.origin.x + x * Math.cos(yaw) - y * Math.sin(yaw),
      y: mapSnapshot.origin.y + x * Math.sin(yaw) + y * Math.cos(yaw),
    }));
    return { minX: Math.min(...corners.map((point) => point.x)), maxX: Math.max(...corners.map((point) => point.x)), minY: Math.min(...corners.map((point) => point.y)), maxY: Math.max(...corners.map((point) => point.y)) };
  }
  const floor = mapLayout.floors?.[0];
  const boundary = floor ? floorBoundary(floor, mapLayout.size.width, mapLayout.size.depth) : [];
  const xs = boundary.map((point) => point.x), ys = boundary.map((point) => point.y);
  return boundary.length ? { minX: Math.min(...xs), maxX: Math.max(...xs), minY: Math.min(...ys), maxY: Math.max(...ys) }
    : { minX: 0, maxX: mapLayout.size.width, minY: 0, maxY: mapLayout.size.depth };
}

function makeTransform(width: number, height: number, bounds: MapBounds, zoom: number, center: { x: number; y: number } | null, follow: boolean, pose?: FramePose): MapTransform {
  const viewCenter = follow && pose ? { x: pose.x, y: pose.y } : center;
  const world = createWorldTransform({ width, height }, bounds, zoom, viewCenter, 28);
  return { ...world, toCanvas: (x, y) => worldToScreen({ x, y }, world), toWorld: (x, y) => screenToWorld({ x, y }, world) };
}

function drawDetailMap(ctx: CanvasRenderingContext2D, width: number, height: number, transform: MapTransform, bounds: MapBounds, mapSnapshot: RobotDetailMapSnapshot | null, occupancyRaster: HTMLCanvasElement | null, pose: FramePose | undefined, robot: RobotState | undefined, globalPath: RobotDetailPath | null, localPath: RobotDetailPath | null, goal: RobotDetailGoal | null, goalPreview: WorldGoal | null, pathPreview: RobotDetailPathPreview | null, layers: { showGrid: boolean; showLidar: boolean; showPaths: boolean; showWarehouse: boolean }) {
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#07101c";
  ctx.fillRect(0, 0, width, height);
  const worldToCanvas = transform.toCanvas;
  const drawPolygon = (points: Array<[number, number]>, fill: string, stroke?: string, lineWidth = 1) => {
    if (!points.length) return;
    ctx.beginPath();
    points.forEach(([x, y], index) => { const p = worldToCanvas(x, y); if (index === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y); });
    ctx.closePath();
    if (fill) { ctx.fillStyle = fill; ctx.fill(); }
    if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = lineWidth; ctx.stroke(); }
  };
  drawPolygon([[bounds.minX, bounds.minY], [bounds.maxX, bounds.minY], [bounds.maxX, bounds.maxY], [bounds.minX, bounds.maxY]], "#0b1725", "#29445f", 1);

  if (mapSnapshot?.frame_id === "map" && occupancyRaster) drawOccupancy(ctx, mapSnapshot, occupancyRaster, worldToCanvas, transform.scale);
  if (layers.showGrid) drawWorldGrid(ctx, width, height, transform, bounds);
  if (layers.showWarehouse) drawWarehouseLayer(ctx, mapLayoutForCanvas(), worldToCanvas);

  if (layers.showPaths) {
    drawPath(ctx, globalPath?.frame_id === "map" ? globalPath.points : [], worldToCanvas, "#9b87ff", 2.6, false);
    drawPath(ctx, localPath?.frame_id === "map" ? localPath.points : [], worldToCanvas, "#33c7ff", 1.8, true);
    drawPath(ctx, pathPreview?.status === "VALID" ? pathPreview.path : [], worldToCanvas, "#f4cf52", 2.8, false);
  }
  const actualGoal = goalPreview ?? (goal?.frame_id === "map" ? goal : null);
  if (actualGoal) drawGoal(ctx, actualGoal, worldToCanvas, goalPreview ? "#facc15" : "#b08cff");
  if (robot && pose) drawRobot(ctx, robot.id, robot.status, pose, worldToCanvas, transform.scale);
  ctx.fillStyle = "#8aa4bf";
  ctx.font = "10px JetBrains Mono, monospace";
  ctx.fillText(`scale ${transform.scale.toFixed(1)} px/m`, 12, height - 12);

  function mapLayoutForCanvas() { return layout; }
}

function drawOccupancy(ctx: CanvasRenderingContext2D, mapSnapshot: RobotDetailMapSnapshot, raster: HTMLCanvasElement, worldToCanvas: MapTransform["toCanvas"], scale: number) {
  const origin = worldToCanvas(mapSnapshot.origin.x, mapSnapshot.origin.y);
  ctx.save();
  ctx.translate(origin.x, origin.y);
  ctx.rotate(-mapSnapshot.origin.yaw);
  ctx.scale(scale * mapSnapshot.resolution, -scale * mapSnapshot.resolution);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(raster, 0, -mapSnapshot.height);
  ctx.restore();
}

function drawWorldGrid(ctx: CanvasRenderingContext2D, width: number, height: number, transform: MapTransform, bounds: MapBounds) {
  const step = Math.max(1, Math.ceil(Math.max(bounds.maxX - bounds.minX, bounds.maxY - bounds.minY) / 20 / 5) * 5);
  ctx.strokeStyle = "rgba(83, 125, 158, .22)";
  ctx.lineWidth = 1;
  for (let x = Math.floor(bounds.minX / step) * step; x <= bounds.maxX; x += step) { const p1 = transform.toCanvas(x, bounds.minY); const p2 = transform.toCanvas(x, bounds.maxY); ctx.beginPath(); ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y); ctx.stroke(); }
  for (let y = Math.floor(bounds.minY / step) * step; y <= bounds.maxY; y += step) { const p1 = transform.toCanvas(bounds.minX, y); const p2 = transform.toCanvas(bounds.maxX, y); ctx.beginPath(); ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y); ctx.stroke(); }
  ctx.fillStyle = "#6b89a5";
  ctx.font = "9px JetBrains Mono, monospace";
  ctx.fillText("MAP / WORLD", 12, 18);
  void width; void height;
}

function drawWarehouseLayer(ctx: CanvasRenderingContext2D, mapLayout: WarehouseLayout, worldToCanvas: MapTransform["toCanvas"]) {
  const floor = mapLayout.floors?.[0];
  const floorMatches = (value: number | string | undefined) => value === undefined || !floor || String(value) === String(floor.id);
  const boundary = floor ? floorBoundary(floor, mapLayout.size.width, mapLayout.size.depth) : [];
  if (boundary.length) {
    drawPolygon(ctx, boundary.map((point) => [point.x, point.y] as [number, number]), worldToCanvas, "rgba(20, 39, 57, .9)", "#7693ad");
    for (const hole of floor?.holes ?? []) drawPolygon(ctx, hole as Array<[number, number]>, worldToCanvas, "#07101c", "#526d86");
  }
  for (const zone of mapLayout.zones ?? []) if (floorMatches(zone.floor)) drawPolygon(ctx, zone.polygon, worldToCanvas, `${zone.color || "#48617b"}16`, zone.color || "#48617b");
  for (const restricted of mapLayout.restricted_areas ?? []) if (floorMatches(restricted.floor)) drawRect(ctx, restricted.rect, worldToCanvas, restricted.robots_allowed ? "rgba(245, 179, 66, .12)" : "rgba(222, 76, 76, .18)", restricted.robots_allowed ? "#d8a84d" : "#df6262");
  for (const aisle of mapLayout.aisles ?? []) if (floorMatches(aisle.floor_id)) drawPolyline(ctx, aisle.centerline.map((point) => [point.x, point.y] as [number, number]), worldToCanvas, "rgba(69, 194, 216, .52)", Math.max(1, aisle.width * 0.35));
  const tags = (mapLayout.navigation_tags ?? []).filter((tag) => floorMatches(tag.floor_id));
  const tagByKey = new Map(tags.flatMap((tag) => [[String(tag.uuid), tag], [String(tag.tag_id), tag]] as const));
  for (const edge of mapLayout.navigation_edges ?? []) {
    if (edge.enabled === false || !floorMatches(edge.floor_id)) continue;
    const from = tagByKey.get(String(edge.from_tag_uuid)) ?? tagByKey.get(String(edge.from_tag_id));
    const to = tagByKey.get(String(edge.to_tag_uuid)) ?? tagByKey.get(String(edge.to_tag_id));
    if (from && to) drawPolyline(ctx, [[from.x, from.y], [to.x, to.y]], worldToCanvas, "rgba(123, 164, 199, .58)", 1.5);
  }
  for (const rack of mapLayout.racks ?? []) if (floorMatches(rack.floor)) drawRect(ctx, [rack.position[0], rack.position[2], rack.position[0] + rack.size[0], rack.position[2] + rack.size[2]], worldToCanvas, "rgba(192, 126, 49, .25)", "#b9813f");
  for (const tag of tags) { const p = worldToCanvas(tag.x, tag.y); ctx.fillStyle = "#f4b942"; ctx.fillRect(p.x - 2, p.y - 2, 4, 4); }
}

function drawPolygon(ctx: CanvasRenderingContext2D, points: Array<{ x: number; y: number } | [number, number]>, worldToCanvas: MapTransform["toCanvas"], fill: string, stroke: string) {
  if (!points.length) return;
  ctx.beginPath();
  points.forEach((point, index) => { const x = Array.isArray(point) ? point[0] : point.x; const y = Array.isArray(point) ? point[1] : point.y; const p = worldToCanvas(x, y); if (index === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y); });
  ctx.closePath(); ctx.fillStyle = fill; ctx.fill(); ctx.strokeStyle = stroke; ctx.lineWidth = 1; ctx.stroke();
}

function drawRect(ctx: CanvasRenderingContext2D, rect: [number, number, number, number], worldToCanvas: MapTransform["toCanvas"], fill: string, stroke: string) { drawPolygon(ctx, [[rect[0], rect[1]], [rect[2], rect[1]], [rect[2], rect[3]], [rect[0], rect[3]]], worldToCanvas, fill, stroke); }
function drawPolyline(ctx: CanvasRenderingContext2D, points: Array<[number, number]>, worldToCanvas: MapTransform["toCanvas"], color: string, width: number) { if (!points.length) return; ctx.beginPath(); points.forEach(([x, y], index) => { const p = worldToCanvas(x, y); if (index === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y); }); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.stroke(); }
function drawPath(ctx: CanvasRenderingContext2D, points: Array<[number, number]>, worldToCanvas: MapTransform["toCanvas"], color: string, width: number, dashed: boolean) { if (!points.length) return; ctx.save(); ctx.setLineDash(dashed ? [6, 5] : []); drawPolyline(ctx, points, worldToCanvas, color, width); ctx.restore(); }
function drawGoal(ctx: CanvasRenderingContext2D, goal: WorldGoal | RobotDetailGoal, worldToCanvas: MapTransform["toCanvas"], color: string) { const p = worldToCanvas(goal.x, goal.y); ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(p.x, p.y, 8, 0, Math.PI * 2); ctx.stroke(); ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(p.x + Math.cos(goal.yaw) * 16, p.y - Math.sin(goal.yaw) * 16); ctx.stroke(); ctx.fillRect(p.x - 2, p.y - 2, 4, 4); }
function drawSelectedTag(ctx: CanvasRenderingContext2D, tag: RobotNavigationTag, worldToCanvas: MapTransform["toCanvas"]) { const pose = tag.navigation_pose; if (!pose) return; const p = worldToCanvas(pose.x, pose.y); ctx.save(); ctx.strokeStyle = "#e19aff"; ctx.fillStyle = "#f2c7ff"; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(p.x, p.y, 11, 0, Math.PI * 2); ctx.stroke(); ctx.beginPath(); ctx.moveTo(p.x - 15, p.y); ctx.lineTo(p.x + 15, p.y); ctx.moveTo(p.x, p.y - 15); ctx.lineTo(p.x, p.y + 15); ctx.stroke(); ctx.font = "bold 10px JetBrains Mono, monospace"; ctx.fillText(`${tag.tag_id} · ${tag.label}`, p.x + 15, p.y - 12); ctx.restore(); }
function drawRobot(ctx: CanvasRenderingContext2D, robotId: string, status: string, pose: FramePose, worldToCanvas: MapTransform["toCanvas"], scale: number) { const p = worldToCanvas(pose.x, pose.y); const color = status === "ERROR" ? "#ef6262" : status === "WARNING" ? "#f3c64e" : "#37d6c1"; ctx.save(); ctx.translate(p.x, p.y); ctx.rotate(-pose.yaw); ctx.fillStyle = "rgba(14, 29, 45, .95)"; ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath(); ctx.rect(-0.60 * scale, -0.30 * scale, 1.20 * scale, 0.60 * scale); ctx.fill(); ctx.stroke(); ctx.fillStyle = color; ctx.beginPath(); ctx.moveTo(0.60 * scale, 0); ctx.lineTo(0.36 * scale, -0.12 * scale); ctx.lineTo(0.36 * scale, 0.12 * scale); ctx.closePath(); ctx.fill(); ctx.restore(); ctx.fillStyle = "#e8f3ff"; ctx.font = "bold 10px JetBrains Mono, monospace"; ctx.fillText(robotId, p.x + 0.64 * scale, p.y - 0.38 * scale); }
