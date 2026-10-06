import { Component, memo, useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState, type ErrorInfo, type ReactNode, type MouseEvent as ReactMouseEvent } from "react";
import { apiFetch, clearEmergencyStop, emergencyStop, getRobotNavigationTags, type RobotNavigationTag, type RobotNavigationTagRegistry } from "../../services/api";
import { WS_URL, wsManualCommand, wsSetRobotMode, wsSend, type ManualAction } from "../../services/ws";
import { MANUAL_COMMAND_REFRESH_MS, nextManualCommand, type ActiveManualCommand } from "../../services/manualCommand";
import { layout, useStore } from "../../state/store";
import type { FramePose, RobotDetailError, RobotDetailGoal, RobotDetailMapSnapshot, RobotDetailPath, RobotDetailPathPreview, RobotState, RobotSystemDiagnostics } from "../../schema/twin_state";
import type { WarehouseLayout } from "../../layout/types";
import { createWorldTransform, floorBoundary, screenToWorld, worldToScreen, type WorldBounds, type WorldTransform } from "../../layout/coordinates";
import { useStableDisplayedFramePose, type MapPoseIdentity } from "../../layout/robotPoseFrame";
import { ActiveNavigationMap2DView, LocalRobotSection } from "./LocalRobotSections";
import { RobotLidar3DView } from "./RobotLidarViews";
import { occupancyRasters } from "./occupancyRaster";
import { detailPerformance } from "./detailPerformance";
import { displayedNavigationMapIdentity as getDisplayedNavigationMapIdentity, mapPointPreviewPayload, mapPointTarget, sameNavigationMapIdentity, type MapPointNavigationTarget, type NavigationMapIdentity } from "./navigationMapIdentity";
import { evaluatePreviewApproval } from "./navigationPreviewApproval";

type WorldGoal = { x: number; y: number; yaw: number };
type GoalSelection = WorldGoal & Partial<NavigationMapIdentity> & Partial<Pick<MapPointNavigationTarget, "source_map_id" | "source_map_revision">>;
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

function isMapPointTarget(target: GoalSelection | null): target is MapPointNavigationTarget {
  return Boolean(target && target.frame_id === "map" && target.map_id && target.map_revision
    && target.source_type && target.source_map_id && target.source_map_revision);
}

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
  const runtimeCapabilities = useStore((state) => state.robotCapabilities[robotId] ?? null);
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
  const slamRuntimeActive = runtimeState === "MAPPING" || runtimeState === "UNIFIED";
  const useLiveSlamMap = slamRuntimeActive && !activeLocalMapId;
  const activeMapSnapshot = useLiveSlamMap ? slam2dMap : runtimeMapSnapshot;
  const setRobotDetail = useStore((state) => state.setRobotDetail);
  const appliedMode = useStore((state) => state.robotDetail[robotId]?.appliedMode);
  const requestedMode = useStore((state) => state.robotDetail[robotId]?.requestedMode);
  const modeTransitionState = useStore((state) => state.robotDetail[robotId]?.modeTransitionState);
  const mapSync = useStore((state) => state.mapSync);
  const layoutRevision = useStore((state) => state.layoutRevision);
  const robotMapSync = mapSync.robots[robotId];
  const [controlMode, setControlMode] = useState<"MANUAL" | "AUTONOMOUS">(robot?.control_mode ?? "AUTONOMOUS");
  const [activeManualCommand, setActiveManualCommand] = useState<ActiveManualCommand | null>(null);
  const [activeTab, setActiveTab] = useState<LocalTab>("CONTROL");
  const [mapSource, setMapSource] = useState<MapSource>("GLOBAL");
  const [lidarDimension, setLidarDimension] = useState<LidarDimension>("2D");
  const [targetMethod, setTargetMethod] = useState<NavigationTargetMethod>("MAP_POINT");
  const [tagRegistry, setTagRegistry] = useState<RobotNavigationTagRegistry | null>(null);
  const [tagRegistryState, setTagRegistryState] = useState<"IDLE" | "LOADING" | "READY" | "ERROR">("IDLE");
  const [tagRegistryError, setTagRegistryError] = useState("");
  const [selectedTagId, setSelectedTagId] = useState<number | null>(null);
  const [goalPreview, setGoalPreview] = useState<GoalSelection | null>(null);
  const [pathRequestState, setPathRequestState] = useState<"IDLE" | "PLANNING">("IDLE");
  const latestPathRequest = useRef("");
  const tagRegistryRequestVersion = useRef(0);
  const [error, setError] = useState("");
  const [safetyNotice, setSafetyNotice] = useState("");
  const [busy, setBusy] = useState(false);
  const [host, setHost] = useState<HostStatus | null>(null);
  const [clockNow, setClockNow] = useState(() => Date.now());
  const manualTimer = useRef<number | null>(null);
  const activeManualCommandRef = useRef<ActiveManualCommand | null>(null);
  const activeManualRobotId = useRef<string | null>(null);
  const manualWorker = useRef<Worker | null>(null);
  const manualKeyboardHandlers = useRef<{
    activate: (action: ManualAction) => void;
    stop: (force?: boolean) => void;
  }>({ activate: () => undefined, stop: () => undefined });

  const robotBridgeOnline = connectedRobotIds.includes(robotId);
  const robotOnline = Boolean(robot && robot.status !== "OFFLINE" && robotBridgeOnline && rosConnected && websocketState === "CONNECTED");
  const controlOnline = robotOnline;
  const localization = diagnostics?.localization ?? rawLocalization?.state ?? null;
  const canonicalMapReady = !activeLocalMapId && (robotMapSync?.status ?? mapSync.status) === "SYNCED";
  const activeMappingSnapshot = useLiveSlamMap && slam2dMap?.map_source === "SLAM_TOOLBOX"
    && slam2dMap.frame_id === "map" && Boolean(slam2dMap.mapping_session_id)
    && slam2dMap.active_map_id === `SLAM-${slam2dMap.mapping_session_id}`
    && slam2dMap.active_map_revision === `session-${slam2dMap.mapping_session_id}`
    && (!mappingSessionId || slam2dMap.mapping_session_id === mappingSessionId) ? slam2dMap : null;
  const activeMapId = useLiveSlamMap ? activeMappingSnapshot?.active_map_id ?? null
    : activeLocalMapId ?? (canonicalMapReady ? "CANONICAL" : activeMapSnapshot?.active_map_id ?? null);
  const activeMapRevision = useLiveSlamMap ? activeMappingSnapshot?.active_map_revision ?? null
    : activeLocalMapRevision ?? activeMapSnapshot?.active_map_revision
      ?? (robotMapSync?.rosRevision ?? mapSync.rosRevision)?.toString() ?? null;
  const activeMapStatus = useLiveSlamMap ? activeMappingSnapshot ? "SLAM · LIVE · LOCAL_ONLY" : "WAITING FOR SLAM MAP"
    : activeLocalMapId
      ? localMapSyncStatus ?? (activeLocalMapRevision ? "LOCAL_ONLY" : "LOADING")
      : canonicalMapReady ? "CANONICAL" : robotMapSync?.status ?? mapSync.status;
  const activeMapReady = Boolean(activeMapId && activeMapRevision && (
    useLiveSlamMap ? Boolean(activeMappingSnapshot)
      : ["CANONICAL", "LOCAL_ONLY"].includes(activeMapStatus ?? "")
  ));
  const navigationUiAvailable = Boolean(runtimeCapabilities?.goal_available
    && controlMode === "AUTONOMOUS" && controlOnline);
  const canonicalMapRevision = mapSync.publishedRevision ?? mapSync.rosRevision ?? layoutRevision;
  const currentSlam2dMap = activeMappingSnapshot;
  const currentSlam3dCloud = useLiveSlamMap && slam3dAccumulatedCloud?.accumulated
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
  const statePose = robot?.active_map_pose ?? null;
  const informationalTagRegistry = tagRegistry?.reason === "TAG_MAP_REGISTRATION_REQUIRED";
  const availableTags = tagRegistry?.tags ?? [];
  const compatibleTags = tagRegistry?.compatible || informationalTagRegistry ? tagRegistry.tags : [];
  const selectedTag = availableTags.find((tag) => tag.tag_id === selectedTagId) ?? null;
  const isTagNavigable = (tag: RobotNavigationTag | null | undefined) => Boolean(tag?.navigable
    && tagRegistry?.compatible && tagRegistry.registry_revision
    && tag.map_id === activeMapId && tag.map_revision === activeMapRevision);
  const navigableTags = compatibleTags.filter(isTagNavigable);
  const previewApproval = evaluatePreviewApproval({
    pathPreview, requestId: latestPathRequest.current, targetMethod, target: goalPreview,
    activeMapId, activeMapRevision, canonicalMapRevision, selectedTag,
    orientationPolicy: typeof selectedTag?.metadata.orientation_policy === "string"
      ? selectedTag.metadata.orientation_policy : null,
    registryRevision: tagRegistry?.registry_revision ?? null, now: clockNow,
  });
  const candidatePreview = previewApproval.candidatePreview;
  const approvedPreview: RobotDetailPathPreview | null = previewApproval.approvedPreview;

  const activeMap2dSnapshot = useLiveSlamMap ? currentSlam2dMap
    : runtimeMapSnapshot?.active_map_id === activeMapId
      && String(runtimeMapSnapshot.active_map_revision ?? "") === String(activeMapRevision ?? "")
      ? runtimeMapSnapshot : null;
  const activeNavigationMapKey = JSON.stringify([robotId, activeMapId, activeMapRevision, activeMapReady]);
  const previousActiveNavigationMapKey = useRef(activeNavigationMapKey);
  const registrationRevision = runtimeCapabilities?.registration_revision ?? null;
  const previousRegistrationRevision = useRef<number | null>(registrationRevision);

  useEffect(() => {
    select(robotId);
    setGoalPreview(null);
    latestPathRequest.current = "";
    setPathRequestState("IDLE");
    setRobotDetail(robotId, { pathPreview: null });
  }, [robotId, select, setRobotDetail]);

  useEffect(() => {
    if (previousActiveNavigationMapKey.current === activeNavigationMapKey) return;
    previousActiveNavigationMapKey.current = activeNavigationMapKey;
    wsSend({ type: "PATH_PREVIEW_INVALIDATE", robot_id: robotId });
    latestPathRequest.current = "";
    setPathRequestState("IDLE");
    setGoalPreview(null);
    setSelectedTagId(null);
    setRobotDetail(robotId, { pathPreview: null });
  }, [activeNavigationMapKey, robotId, setRobotDetail]);

  useEffect(() => {
    if (previousRegistrationRevision.current === registrationRevision) return;
    previousRegistrationRevision.current = registrationRevision;
    // Only GLOBAL/canonical points depend on this transform. Direct active-map
    // points remain valid when canonical registration is recalibrated.
    if (targetMethod !== "MAP_POINT" || goalPreview?.source_type !== "CANONICAL_MAP_POINT") return;
    latestPathRequest.current = "";
    setPathRequestState("IDLE");
    setRobotDetail(robotId, { pathPreview: null });
  }, [goalPreview?.source_type, registrationRevision, robotId, setRobotDetail, targetMethod]);

  const loadTagRegistry = useCallback(() => {
    if (!activeMapId || !activeMapRevision) return;
    const requestVersion = ++tagRegistryRequestVersion.current;
    setTagRegistry(null);
    setTagRegistryError("");
    setTagRegistryState("LOADING");
    getRobotNavigationTags(robotId).then((registry) => {
      if (requestVersion !== tagRegistryRequestVersion.current) return;
      if (registry.robot_id !== robotId) throw new Error("Tag registry response belongs to another robot");
      setTagRegistry(registry);
      setTagRegistryState("READY");
    }).catch((cause: unknown) => {
      if (requestVersion !== tagRegistryRequestVersion.current) return;
      setTagRegistry(null);
      setTagRegistryError(cause instanceof Error ? cause.message : "Tag registry request failed");
      setTagRegistryState("ERROR");
    });
  }, [activeMapId, activeMapRevision, robotId]);

  useEffect(() => {
    loadTagRegistry();
    return () => { tagRegistryRequestVersion.current += 1; };
  }, [loadTagRegistry, runtimeCapabilities?.registration_revision]);

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
  const displayedNavigationMapIdentity = getDisplayedNavigationMapIdentity({
    view: detailView,
    active_map_id: activeMapId, active_map_revision: activeMapRevision,
    canonical_revision: canonicalMapRevision, map_snapshot: activeMap2dSnapshot,
  });
  const displayedMapPointPickIdentity = targetMethod === "MAP_POINT" && activeMapReady
    && navigationUiAvailable && controlMode === "AUTONOMOUS" ? displayedNavigationMapIdentity : null;
  const visitedViews = useRef(new Set<string>());
  visitedViews.current.add(detailView);
  const viewFresh = detailView === "GLOBAL" || (detailView === "LIDAR_2D" && Boolean(activeMap2dSnapshot))
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
      : detailView === "LIDAR_2D" ? occupancyRasters.peek(activeMap2dSnapshot) : slam3dAccumulatedCloud;
    if (!cached) return;
    const paint = requestAnimationFrame(() => detailPerformance("view_render", { view: detailView, robot_id: robotId, useful: true, cached: true }));
    return () => cancelAnimationFrame(paint);
    // This measures a cached canvas becoming visible, not a newly received frame.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [activeMap2dSnapshot, detailView, robotId, slam3dAccumulatedCloud]);

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

  const setManualCommand = useCallback((command: ActiveManualCommand | null) => {
    activeManualCommandRef.current = command;
    setActiveManualCommand(command);
    if (command) activeManualRobotId.current = robotId;
    else activeManualRobotId.current = null;
  }, [robotId]);

  const stopManual = useCallback((force = false) => {
    const hadActiveCommand = activeManualCommandRef.current !== null;
    const commandRobotId = activeManualRobotId.current ?? robotId;
    if (manualTimer.current !== null) {
      window.clearInterval(manualTimer.current);
      manualTimer.current = null;
    }
    activeManualCommandRef.current = null;
    activeManualRobotId.current = null;
    setActiveManualCommand(null);
    if ((hadActiveCommand || force) && controlOnline && commandRobotId) {
      if (manualWorker.current) manualWorker.current.postMessage({ type: "STOP", robot_id: commandRobotId });
      else wsManualCommand(commandRobotId, "STOP");
    }
  }, [controlOnline, robotId]);

  useEffect(() => () => stopManual(), [stopManual]);

  const previousRuntime = useRef({ runtimeMode, runtimeState });
  useEffect(() => {
    const previous = previousRuntime.current;
    previousRuntime.current = { runtimeMode, runtimeState };
    if (previous.runtimeMode !== runtimeMode || previous.runtimeState !== runtimeState) stopManual();
  }, [runtimeMode, runtimeState, stopManual]);

  useEffect(() => {
    if (controlMode !== "MANUAL") stopManual();
  }, [controlMode, stopManual]);

  useEffect(() => {
    const stopBeforePageExit = () => stopManual();
    window.addEventListener("pagehide", stopBeforePageExit);
    window.addEventListener("beforeunload", stopBeforePageExit);
    return () => {
      window.removeEventListener("pagehide", stopBeforePageExit);
      window.removeEventListener("beforeunload", stopBeforePageExit);
    };
  }, [stopManual]);

  useEffect(() => {
    if (!controlOnline || typeof Worker === "undefined") return;
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
      stopManual(true);
      setError(event.data.message || "Manual command refresh stopped safely");
    };
    worker.onerror = () => {
      if (manualWorker.current === worker) manualWorker.current = null;
      stopManual(true);
      setError("Manual refresh worker failed; the backend timeout stop is active");
    };
    worker.postMessage({ type: "CONNECT", url: WS_URL });
    return () => {
      if (manualWorker.current === worker) manualWorker.current = null;
      worker.postMessage({ type: "DISCONNECT", robot_id: robotId });
      window.setTimeout(() => worker.terminate(), 150);
    };
  }, [controlOnline, robotId, stopManual]);

  const estopActive = Boolean(detailDiagnostics?.command_ownership?.estop_active
    ?? diagnostics?.command_ownership?.estop_active);
  useEffect(() => {
    if (estopActive) stopManual();
  }, [estopActive, stopManual]);

  const runAction = useCallback(async (fn: () => Promise<unknown>) => {
    setBusy(true);
    setError("");
    setSafetyNotice("");
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

  const toggleManual = useCallback((action: ManualAction) => {
    if (action === "STOP") {
      stopManual(true);
      return;
    }
    if (controlMode !== "MANUAL" || modeTransitionState === "REQUESTED" || modeTransitionState === "FAILED") { setError("Wait for applied MANUAL mode before driving"); return; }
    if (estopActive) { setError("Manual control is blocked while E-STOP is active"); return; }
    if (!controlOnline) { setError("Manual control is disabled while ROS bridge is disconnected"); return; }
    const nextCommand = nextManualCommand(activeManualCommandRef.current, action);
    if (!nextCommand) {
      stopManual(true);
      return;
    }
    if (manualWorker.current) {
      manualWorker.current.postMessage({ type: "HOLD", robot_id: robotId, action: nextCommand });
      setManualCommand(nextCommand);
      return;
    }
    if (manualTimer.current !== null) window.clearInterval(manualTimer.current);
    if (!wsManualCommand(robotId, nextCommand)) {
      setError("Manual command was not sent");
      stopManual(true);
      return;
    }
    setManualCommand(nextCommand);
    manualTimer.current = window.setInterval(() => {
      if (!wsManualCommand(robotId, nextCommand)) stopManual(true);
    }, MANUAL_COMMAND_REFRESH_MS);
  }, [controlMode, controlOnline, estopActive, modeTransitionState, robotId, setManualCommand, stopManual]);

  manualKeyboardHandlers.current = { activate: toggleManual, stop: stopManual };

  useEffect(() => {
    const keyActions: Record<string, ManualAction> = {
      w: "FORWARD", W: "FORWARD", ArrowUp: "FORWARD", s: "BACKWARD", S: "BACKWARD", ArrowDown: "BACKWARD",
      a: "LEFT", A: "LEFT", ArrowLeft: "LEFT", d: "RIGHT", D: "RIGHT", ArrowRight: "RIGHT", q: "ROTATE_LEFT", Q: "ROTATE_LEFT", e: "ROTATE_RIGHT", E: "ROTATE_RIGHT",
    };
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.code === "Space") {
        event.preventDefault();
        manualKeyboardHandlers.current.stop(true);
        return;
      }
      const action = keyActions[event.key];
      if (!action || event.repeat || (event.target instanceof HTMLElement && ["INPUT", "SELECT", "TEXTAREA"].includes(event.target.tagName))) return;
      event.preventDefault();
      manualKeyboardHandlers.current.activate(action);
    };
    window.addEventListener("keydown", onKeyDown);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      manualKeyboardHandlers.current.stop();
    };
  }, []);

  const sendGoal = () => {
    const approvedGoal = approvedPreview?.goal;
    if (!goalPreview || !approvedPreview || approvedPreview.status !== "VALID" || !approvedGoal) return;
    if (!navigationUiAvailable) { setError("Nav2 is not ready for this active map and control mode"); return; }
    if (controlMode !== "AUTONOMOUS") { setError("Switch to AUTONOMOUS before sending a goal"); return; }
    if (!controlOnline) { setError("Navigation goal requires an online ROS bridge"); return; }
    if (!activeMapReady) { setError(`The selected robot active map is not confirmed (${activeMapStatus})`); return; }
    if (!wsSend({ type: "NAV_GOAL", robot_id: robotId, x: approvedGoal.x, y: approvedGoal.y, yaw: approvedGoal.yaw,
      frame_id: "map", preview_request_id: approvedPreview.request_id, active_map_id: activeMapId!,
      active_map_revision: activeMapRevision!, map_id: approvedPreview.active_map_id!,
      map_revision: approvedPreview.active_map_revision!, source_type: approvedPreview.source_type ?? "MAP_POINT",
      source_id: approvedPreview.source_id ?? undefined,
      source_map_id: approvedPreview.source_map_id ?? undefined,
      source_map_revision: approvedPreview.source_map_revision ?? undefined,
      route_revision: approvedPreview.source_type === "TAG" ? approvedPreview.route_revision ?? undefined : undefined })) {
      setError("Navigation goal was not sent");
      return;
    }
    setGoalPreview(null);
    latestPathRequest.current = "";
    setPathRequestState("IDLE");
    setRobotDetail(robotId, { pathPreview: null });
    setError("");
  };

  const selectMapPoint = useCallback((target: MapPointNavigationTarget) => {
    const sourceMatches = target.source_type === "CANONICAL_MAP_POINT"
      ? target.source_map_id === "CANONICAL"
        && target.source_map_revision === String(canonicalMapRevision)
      : target.source_type === "ACTIVE_MAP_POINT"
        && target.source_map_id === activeMapId
        && target.source_map_revision === activeMapRevision;
    if (target.frame_id !== "map" || !sourceMatches) {
      setError("Map point selection belongs to a map identity or revision that is no longer current");
      return;
    }
    setSelectedTagId(null);
    wsSend({ type: "PATH_PREVIEW_INVALIDATE", robot_id: robotId });
    setGoalPreview(target);
    latestPathRequest.current = "";
    setPathRequestState("IDLE");
    setRobotDetail(robotId, { pathPreview: null });
    setError("");
  }, [activeMapId, activeMapRevision, canonicalMapRevision, robotId, setRobotDetail]);

  const selectTargetMethod = (method: NavigationTargetMethod) => {
    if (method === targetMethod) {
      if (method === "TAG" && tagRegistryState === "ERROR") loadTagRegistry();
      return;
    }
    wsSend({ type: "PATH_PREVIEW_INVALIDATE", robot_id: robotId });
    latestPathRequest.current = "";
    setPathRequestState("IDLE");
    setGoalPreview(null);
    setSelectedTagId(null);
    setRobotDetail(robotId, { pathPreview: null });
    setTargetMethod(method);
    setTagRegistryError("");
    setError("");
    if (method === "TAG" && (tagRegistryState === "IDLE" || tagRegistryState === "ERROR")) loadTagRegistry();
  };

  const selectTagById = (tagId: number | null) => {
    const tag = tagId === null ? null : availableTags.find((candidate) => candidate.tag_id === tagId);
    if (tagId !== null && !tag) return;
    setTargetMethod("TAG");
    wsSend({ type: "PATH_PREVIEW_INVALIDATE", robot_id: robotId });
    latestPathRequest.current = "";
    setPathRequestState("IDLE");
    setGoalPreview(isTagNavigable(tag) && tag?.navigation_pose ? { ...tag.navigation_pose } : null);
    setSelectedTagId(tagId);
    setRobotDetail(robotId, { pathPreview: null });
    setError("");
  };

  const requestPathPreview = useCallback((target: GoalSelection) => {
    setGoalPreview(target);
    setError("");
    if (targetMethod === "TAG" && (!isTagNavigable(selectedTag) || !tagRegistry?.registry_revision)) {
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
    const pointTarget = targetMethod === "MAP_POINT" && isMapPointTarget(target) ? target : null;
    if (targetMethod === "MAP_POINT" && (!pointTarget || pointTarget.frame_id !== "map"
        || pointTarget.source_type === "ACTIVE_MAP_POINT"
          && (pointTarget.source_map_id !== activeMapId || pointTarget.source_map_revision !== activeMapRevision)
        || pointTarget.source_type === "CANONICAL_MAP_POINT"
          && (pointTarget.source_map_id !== "CANONICAL"
            || pointTarget.source_map_revision !== String(canonicalMapRevision)))) {
      latestPathRequest.current = "";
      setPathRequestState("IDLE");
      setRobotDetail(robotId, { pathPreview: null });
      setError("Map point preview is blocked because the selected point belongs to a different map or revision");
      return;
    }
    if (!controlOnline || controlMode !== "AUTONOMOUS") {
      latestPathRequest.current = "";
      setPathRequestState("IDLE");
      setRobotDetail(robotId, { pathPreview: null });
      setError("Switch to online AUTONOMOUS mode to request a Nav2 path preview");
      return;
    }
    if (!navigationUiAvailable) {
      latestPathRequest.current = "";
      setPathRequestState("IDLE");
      setRobotDetail(robotId, { pathPreview: null });
      setError("Nav2 is not ready for this active map and control mode");
      return;
    }
    const requestId = typeof crypto !== "undefined" && "randomUUID" in crypto
      ? crypto.randomUUID()
      : `${Date.now()}-${Math.random().toString(16).slice(2)}`;
    latestPathRequest.current = requestId;
    setPathRequestState("PLANNING");
    setRobotDetail(robotId, { pathPreview: null });
    const common = { type: "PATH_PREVIEW_REQUEST" as const, robot_id: robotId, request_id: requestId,
      frame_id: "map" as const, active_map_id: activeMapId!, active_map_revision: activeMapRevision!,
      map_id: activeMapId!, map_revision: activeMapRevision! };
    const pointPayload = pointTarget ? mapPointPreviewPayload(pointTarget,
      { map_id: activeMapId!, map_revision: activeMapRevision! }, activeMap2dSnapshot?.map_content_revision) : null;
    const sent = targetMethod === "TAG" && selectedTag && tagRegistry?.registry_revision
      ? wsSend({ ...common, source_type: "TAG", tag_id: selectedTag.tag_id,
        tag_revision: selectedTag.tag_revision, registry_revision: tagRegistry.registry_revision })
      : pointPayload ? wsSend({ ...common, ...pointPayload }) : false;
    if (!sent) {
      latestPathRequest.current = "";
      setPathRequestState("IDLE");
      setError("Path preview request was not sent because the WebSocket is disconnected");
    }
  }, [activeMap2dSnapshot?.map_content_revision, activeMapId, activeMapReady, activeMapRevision, activeMapStatus, canonicalMapRevision, controlMode, controlOnline, navigationUiAvailable, robotId, selectedTag, setRobotDetail, tagRegistry?.registry_revision, targetMethod]);

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

  const adjustSelectedMapPointYaw = (delta: number) => {
    if (!isMapPointTarget(goalPreview)) return;
    selectMapPoint({ ...goalPreview, yaw: goalPreview.yaw + delta });
  };

  const changeRobot = (next: string) => {
    if (next) pushRoute(`/robots/${encodeURIComponent(next)}/control`);
  };

  const moveButtonEvents = useCallback((action: ManualAction) => ({
    onClick: () => toggleManual(action),
  }), [toggleManual]);

  return (
    <div className="robot-detail-shell">
      <header className="robot-detail-header">
        <div className="robot-detail-identity">
          <button type="button" className="robot-detail-back" onClick={() => pushRoute("/")}>← BACK</button>
          <div><span className="robot-console-kicker">ROBOT CONTROL CONSOLE</span><h1>{robotId}</h1></div>
          <StatusValue value={robotOnline ? "ONLINE" : "OFFLINE"} />
          <span className="robot-detail-mode">{controlMode}{modeTransitionState === "REQUESTED" ? ` → ${requestedMode} REQUESTED` : modeTransitionState === "FAILED" ? " TRANSITION FAILED" : " APPLIED"}</span>
          <span className="robot-detail-runtime">{runtimeState} / {safeText(robot?.navigation_state, "N/A")}</span>
          {runtimeState === "UNIFIED" && <>
            <span className="robot-detail-runtime" data-testid="slam-runtime-state">SLAM {diagnostics?.mapping?.slam_state === "ACTIVE" && diagnostics?.mapping?.map_live ? "LIVE" : safeText(diagnostics?.mapping?.slam_state, "STARTING")}</span>
            <span className="robot-detail-runtime" data-testid="nav2-runtime-state">NAV2 {runtimeCapabilities?.nav2_ready ? "READY" : diagnostics?.nav2 ? "STARTING" : "UNAVAILABLE"}</span>
          </>}
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
        <LocalRobotSection section={activeTab} robotId={robotId} robot={robot} slam2dMap={slam2dMap} runtimeMapSnapshot={runtimeMapSnapshot} localizationMap={activeLocalMapId ? runtimeMapSnapshot : slamRuntimeActive ? slam2dMap : runtimeMapSnapshot} scan={mappingScan} diagnostics={detailDiagnostics ?? diagnostics} errors={detailErrors.length ? detailErrors : detailDiagnostics?.errors ?? diagnostics?.errors ?? EMPTY_ERRORS} controlOnline={controlOnline} controlMode={controlMode} runtimeMode={runtimeMode} runtimeState={runtimeState} runtimeCapabilities={runtimeCapabilities} localization={localization} websocketState={websocketState} mapRevision={mapSync.publishedRevision} activeLocalMapId={activeLocalMapId} activeLocalMapRevision={activeLocalMapRevision} localMapSyncStatus={localMapSyncStatus} lidarStreamDiagnostics={lidarStreamDiagnostics} mappingSessionId={mappingSessionId} />
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
            <span className={`map-sync-warning ${activeMapReady ? "map-sync-warning-ready" : ""}`}>{activeMapId ? `${activeMapId} · r${activeMapRevision ?? "—"} · ${activeMapStatus}` : `MAP ${activeMapStatus}`}</span>
            {activeLocalMapId && <span className="map-sync-warning">LOCAL MAP DIFFERS FROM CANONICAL r{mapSync.publishedRevision ?? "—"} · LOCAL NAV ONLY</span>}
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
                disabled={tagRegistryState !== "READY" || (!tagRegistry?.compatible && !informationalTagRegistry) || compatibleTags.length === 0}
                onChange={(event) => selectTagById(event.currentTarget.value ? Number(event.currentTarget.value) : null)}>
                <option value="">{tagRegistryState === "LOADING" ? "Loading Tags…" : "Select a Tag…"}</option>
                {compatibleTags.map((tag) => <option key={tag.tag_id} value={tag.tag_id} disabled={!isTagNavigable(tag)}>
                  {tag.tag_id} — {tag.label}{tag.navigable ? "" : ` · ${tag.reason ?? "DISABLED"}`}
                </option>)}
              </select>
            </label>}
          </div>
          {targetMethod === "TAG" && <div className="robot-detail-tag-status" role="status" aria-live="polite">
            {tagRegistryState === "LOADING" && <span>Loading authoritative Tags for {activeMapId ?? "the active map"}…</span>}
            {tagRegistryState === "ERROR" && <span className="is-error">Tag registry unavailable: {tagRegistryError}</span>}
            {tagRegistryState === "READY" && informationalTagRegistry && <span className="is-error">Canonical Tags are informational only on this local map. Navigation is disabled: TAG_MAP_REGISTRATION_REQUIRED.</span>}
            {tagRegistryState === "READY" && !tagRegistry?.compatible && !informationalTagRegistry && <span className="is-error">Tags unavailable: {tagRegistry?.reason ?? "active map is incompatible"}</span>}
            {tagRegistryState === "READY" && tagRegistry?.compatible && compatibleTags.length === 0 && <span>No Tags are registered for this active map.</span>}
            {tagRegistryState === "READY" && tagRegistry?.compatible && compatibleTags.length > 0 && navigableTags.length === 0 && <span className="is-error">No enabled, valid Tags are navigable on this map.</span>}
            {selectedTag && <div className="robot-detail-tag-summary" data-testid="selected-navigation-tag"
              data-tag-id={selectedTag.tag_id} data-navigable={isTagNavigable(selectedTag)} data-reason={selectedTag.reason ?? undefined}>
              <span>TAG ID <b>{selectedTag.tag_id}</b></span>
              <span>LABEL <b>{selectedTag.label}</b></span>
              <span>TYPE <b>{selectedTag.family}</b></span>
              <span>MAP <b>{selectedTag.map_id}</b></span>
              <span>REVISION <b>{selectedTag.map_revision}</b></span>
              <span>X <b>{safeNumber(selectedTag.navigation_pose?.x ?? selectedTag.canonical_navigation_pose?.x ?? selectedTag.x ?? Number.NaN, 3)}</b></span>
              <span>Y <b>{safeNumber(selectedTag.navigation_pose?.y ?? selectedTag.canonical_navigation_pose?.y ?? selectedTag.y ?? Number.NaN, 3)}</b></span>
              <span>YAW <b>{safeNumber(selectedTag.navigation_pose?.yaw ?? selectedTag.canonical_navigation_pose?.yaw ?? selectedTag.yaw ?? Number.NaN, 3)} rad</b></span>
              {selectedTag.metadata?.semantic_role != null && <span>ROLE <b>{safeText(selectedTag.metadata.semantic_role)}</b></span>}
              {selectedTag.metadata?.orientation_policy != null && <span>ORIENTATION <b>{safeText(selectedTag.metadata.orientation_policy)}</b></span>}
              {selectedTag.metadata?.rack_id != null && <span>RACK <b>{safeText(selectedTag.metadata.rack_id)}</b></span>}
              <span>NAVIGABLE <b>{isTagNavigable(selectedTag) ? "YES" : "NO"}</b></span>
              {!isTagNavigable(selectedTag) && <span data-testid="selected-tag-blocker">REASON <b>{selectedTag.reason ?? tagRegistry?.reason ?? "TAG_DISABLED"}</b></span>}
            </div>}
          </div>}
          <div className="robot-detail-view-stack" key={robotId}>
            <div className={"robot-detail-view-layer " + (detailView === "GLOBAL" ? "is-active" : "")} data-view="GLOBAL" aria-hidden={detailView !== "GLOBAL"}><DetailMapCanvas active={detailView === "GLOBAL"} robotId={robotId} robot={robot} canonicalRevision={canonicalMapRevision} globalPath={globalPath} localPath={localPath} goal={goal} goalPreview={goalPreview} pathPreview={approvedPreview} selectedTag={selectedTag} navigationTags={tagRegistryState === "READY" ? availableTags.map((tag) => ({ ...tag, navigable: isTagNavigable(tag) })) : []} onGoalSelect={selectMapPoint} onTagSelect={(tagId) => selectTagById(tagId)} pickMapIdentity={detailView === "GLOBAL" ? displayedMapPointPickIdentity : null} /></div>
            {visitedViews.current.has("LIDAR_2D") && <div className={"robot-detail-view-layer " + (detailView === "LIDAR_2D" ? "is-active" : "")} data-view="LIDAR_2D" aria-hidden={detailView !== "LIDAR_2D"}><ActiveNavigationMap2DView map={activeMap2dSnapshot} robot={robot} scan={mappingScan} target={isMapPointTarget(goalPreview) ? goalPreview : null} navigationPath={approvedPreview?.active_path ?? approvedPreview?.path ?? []} routeNodes={approvedPreview?.active_route_points ?? []} canPick={detailView === "LIDAR_2D" && Boolean(displayedMapPointPickIdentity)} onPick={selectMapPoint} /></div>}
            {visitedViews.current.has("LIDAR_3D") && <div className={"robot-detail-view-layer " + (detailView === "LIDAR_3D" ? "is-active" : "")} data-view="LIDAR_3D" aria-hidden={detailView !== "LIDAR_3D"}><RobotLidar3DView active={detailView === "LIDAR_3D"} frame={currentSlam3dCloud} robot={robot} slamMap={currentSlam2dMap} /></div>}
          </div>
          <div className="robot-detail-goal-toolbar">
            <span>{targetMethod === "TAG" ? selectedTag ? !isTagNavigable(selectedTag) ? selectedTag.reason ?? tagRegistry?.reason ?? "TAG_DISABLED" : `TAG ${selectedTag.tag_id} SELECTED · ${pathRequestState === "PLANNING" ? "PLANNING" : approvedPreview?.status === "VALID" ? `PREVIEW VALID · ${safeNumber(approvedPreview.path_length_m, 2, " m")}` : candidatePreview?.status === "INVALID" || candidatePreview?.status === "NO_PATH" ? candidatePreview.reason ?? candidatePreview.status : "PATH PREVIEW REQUIRED"}` : !navigationUiAvailable ? `${runtimeCapabilities?.goal_blocker_code ?? "NAVIGATION_UNAVAILABLE"} · ${runtimeCapabilities?.goal_blocker_reason ?? "runtime has not confirmed Nav2 readiness"}` : "Select a compatible destination Tag" : goalPreview ? `TARGET ${safeNumber(goalPreview.x, 2)} / ${safeNumber(goalPreview.y, 2)} / ${safeNumber(goalPreview.yaw, 2)} rad · ${pathRequestState === "PLANNING" ? "PLANNING" : approvedPreview?.status ?? "NO PREVIEW"}${approvedPreview?.status === "VALID" ? ` · ${safeNumber(approvedPreview.path_length_m, 2, " m")}` : approvedPreview?.reason ? ` · ${approvedPreview.reason}` : ""}` : !navigationUiAvailable ? `${runtimeCapabilities?.goal_blocker_code ?? "NAVIGATION_UNAVAILABLE"} · ${runtimeCapabilities?.goal_blocker_reason ?? "runtime has not confirmed Nav2 readiness"}` : mapSource === "LIDAR" && lidarDimension === "3D" ? "Use GLOBAL MAP or LIDAR 2D to select a target" : "Select destination, then preview the Nav2 path"}</span>
            <button type="button" disabled={targetMethod === "TAG" || !isMapPointTarget(goalPreview) || pathRequestState === "PLANNING"} onClick={() => adjustSelectedMapPointYaw(-Math.PI / 12)}>YAW −</button>
            <button type="button" disabled={targetMethod === "TAG" || !isMapPointTarget(goalPreview) || pathRequestState === "PLANNING"} onClick={() => adjustSelectedMapPointYaw(Math.PI / 12)}>YAW +</button>
            <button type="button" disabled={!navigationUiAvailable || (!goalPreview || targetMethod === "TAG" && !isTagNavigable(selectedTag)) || pathRequestState === "PLANNING"}
              data-preview-status={pathPreview?.status ?? "NONE"}
              data-preview-request-id={pathPreview?.request_id ?? ""}
              data-preview-approved={previewApproval.approved ? "true" : "false"}
              data-preview-reject-reason={previewApproval.rejectReason ?? undefined}
              data-preview-gates={JSON.stringify(previewApproval.gates)}
              onClick={() => goalPreview && requestPathPreview(goalPreview)}>PREVIEW PATH</button>
            <button type="button" disabled={!navigationUiAvailable || !goalPreview || !approvedPreview || !controlOnline || controlMode !== "AUTONOMOUS" || !activeMapReady} className="robot-console-primary" onClick={sendGoal}>SEND GOAL</button>
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
        <ManualBar controlMode={controlMode} controlOnline={controlOnline} activeManualCommand={activeManualCommand} moveButtonEvents={moveButtonEvents} setMode={setMode} />
        <div className="robot-detail-nav-bar">
          <div><span>NAV STATUS</span><b>{safeText(navigationStatus ?? robot?.navigation_state, "N/A")}</b></div>
          <div><span>GOAL</span><b>{goal ? `${safeNumber(goal.x, 2)} / ${safeNumber(goal.y, 2)}` : "N/A"}</b></div>
          <div><span>REMAINING</span><b>{safeNumber(remainingDistanceM, 2, " m")}</b></div>
          <button type="button" disabled={!controlOnline || controlMode !== "AUTONOMOUS"} onClick={() => navCommand("NAV_PAUSE")}>PAUSE</button>
          <button type="button" disabled={!controlOnline || controlMode !== "AUTONOMOUS"} onClick={() => navCommand("NAV_RESUME")}>RESUME</button>
          <button type="button" className="robot-console-danger-outline" disabled={!controlOnline} onClick={() => navCommand("NAV_CANCEL")}>CANCEL NAV</button>
          <button type="button" disabled={busy || !robotId} onClick={() => void runAction(async () => {
            const result = await clearEmergencyStop(robotId);
            if (result.code !== "CLEAR_ESTOP_APPLIED" || result.emergency_stop_active
                || !result.pre_stop_navigation_terminal) throw new Error("CLEAR_ESTOP_UNCONFIRMED: runtime did not confirm the stop latch is clear");
            setSafetyNotice("CLEAR_ESTOP_APPLIED · bridge confirmed latch clear and prior navigation terminal");
          })}>CLEAR STOP</button>
        </div>
        {safetyNotice && <div className="robot-detail-command-notice" role="status">{safetyNotice}</div>}
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

function ManualBarContent({ controlMode, controlOnline, activeManualCommand, moveButtonEvents, setMode }: { controlMode: string; controlOnline: boolean; activeManualCommand: ActiveManualCommand | null; moveButtonEvents: (action: ManualAction) => { onClick: () => void }; setMode: (mode: "MANUAL" | "AUTONOMOUS") => void }) {
  return <section className="robot-detail-manual">
    <div className="robot-detail-manual-head"><div><span className="robot-console-kicker">MANUAL CONTROL</span><b>{activeManualCommand ? `${activeManualCommand} LATCHED` : "STOPPED"}</b><small>Click or key press toggles; release does not stop · Space / STOP to stop</small></div><div className="robot-detail-manual-mode"><button type="button" className={controlMode === "MANUAL" ? "is-active" : ""} disabled={!controlOnline} onClick={() => setMode("MANUAL")}>MANUAL</button><button type="button" className={controlMode === "AUTONOMOUS" ? "is-active" : ""} disabled={!controlOnline} onClick={() => setMode("AUTONOMOUS")}>AUTONOMOUS</button></div></div>
    <div className="robot-detail-manual-pad">{MANUAL_ACTIONS.map((item) => <button type="button" key={item.action} className={`manual-key manual-key-${item.action.toLowerCase()}${activeManualCommand === item.action ? " is-active" : ""}`} title={item.title} aria-label={item.title} aria-pressed={item.action !== "STOP" && activeManualCommand === item.action} disabled={!controlOnline || (item.action !== "STOP" && controlMode !== "MANUAL")} {...moveButtonEvents(item.action)}>{item.label}<small>{item.action === "FORWARD" ? "W / ↑" : item.action === "BACKWARD" ? "S / ↓" : item.action === "LEFT" ? "A / ←" : item.action === "RIGHT" ? "D / →" : item.action === "ROTATE_LEFT" ? "Q" : item.action === "ROTATE_RIGHT" ? "E" : "STOP"}</small></button>)}</div>
  </section>;
}

function Panel({ title, children, className = "" }: { title: string; children: ReactNode; className?: string }) {
  return <section className={`robot-detail-panel ${className}`}><header>{title}</header><div className="robot-detail-panel-body">{children}</div></section>;
}

const Metric = memo(MetricContent);

function MetricContent({ label, value, mono = false, status = false }: { label: string; value: unknown; mono?: boolean; status?: boolean }) {
  return <div className="robot-detail-metric"><span>{label}</span>{status ? <StatusValue value={value} /> : <b className={mono ? "mono" : ""}>{safeText(value)}</b>}</div>;
}

type MapCanvasProps = { active?: boolean; robotId: string; robot?: RobotState; canonicalRevision: string | number; globalPath: RobotDetailPath | null; localPath: RobotDetailPath | null; goal: RobotDetailGoal | null; goalPreview: WorldGoal | null; pathPreview: RobotDetailPathPreview | null; selectedTag: RobotNavigationTag | null; navigationTags: RobotNavigationTag[]; onGoalSelect: (goal: MapPointNavigationTarget) => void; onTagSelect: (tagId: number) => void; pickMapIdentity: NavigationMapIdentity | null };

function DetailMapCanvas({ active = true, robotId, robot, canonicalRevision, globalPath, localPath, goal, goalPreview, pathPreview, selectedTag, navigationTags, onGoalSelect, onTagSelect, pickMapIdentity }: MapCanvasProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const hostRef = useRef<HTMLDivElement>(null);
  const [size, setSize] = useState({ width: 0, height: 0 });
  const [zoom, setZoom] = useState(1);
  const [center, setCenter] = useState<{ x: number; y: number } | null>(null);
  const [follow, setFollow] = useState(true);
  const [showGrid, setShowGrid] = useState(true);
  const [showPaths, setShowPaths] = useState(true);
  const [hoveredTagId, setHoveredTagId] = useState<number | null>(null);
  const layoutRevision = useStore((state) => state.layoutRevision);
  const runtimeState = useStore((state) => state.runtimeState);
  const canonicalTagPreview = showPaths && pathPreview?.source_type === "TAG"
    && pathPreview.status === "VALID" && pathPreview.canonical_path?.length
    ? { ...pathPreview, path: pathPreview.canonical_path } : null;

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
  const clickMapIdentity: NavigationMapIdentity = {
    frame_id: displayedMapIdentity.frame_id ?? "map",
    map_id: displayedMapIdentity.active_map_id ?? "CANONICAL",
    map_revision: String(displayedMapIdentity.active_map_revision ?? ""),
    source_type: "CANONICAL_MAP_POINT",
  };
  const canPick = sameNavigationMapIdentity(pickMapIdentity, clickMapIdentity);
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
    // GLOBAL is always the canonical warehouse frame. Unified SLAM/Nav2 paths
    // use the active SLAM map frame and are rendered only in the SLAM views.
    const canonicalPaths = runtimeState === "MAPPING" || runtimeState === "UNIFIED" ? false : showPaths;
    const showCanonicalPaths = canonicalPaths || Boolean(canonicalTagPreview);
    drawDetailMap(ctx, size.width, size.height, transform, bounds, null, null, displayedPose, robot,
      canonicalPaths ? globalPath : null, canonicalPaths ? localPath : null, canonicalPaths ? goal : null,
      canonicalPaths ? goalPreview : null, canonicalTagPreview ?? (canonicalPaths ? pathPreview : null),
      { showGrid, showLidar: false, showPaths: showCanonicalPaths, showWarehouse: true });
    drawNavigationTagMarkers(ctx, navigationTags, selectedTag?.tag_id ?? null, hoveredTagId, transform.toCanvas);
    detailPerformance("view_render", { view: "GLOBAL", useful: true, robot_id: robotId });
  }, [active, bounds, goal, goalPreview, globalPath, localPath, navigationTags, pathPreview, selectedTag, hoveredTagId, displayedPose, robot, runtimeState, showGrid, showPaths, size.height, size.width, transform]);

  const handleMapClick = (event: ReactMouseEvent<HTMLCanvasElement>) => {
    const rect = event.currentTarget.getBoundingClientRect();
    const x = event.clientX - rect.left, y = event.clientY - rect.top;
    const hitTag = findNavigationTagAtScreenPoint(navigationTags, x, y, transform.toCanvas);
    if (hitTag) {
      onTagSelect(hitTag.tag_id);
      return;
    }
    if (!canPick) return;
    const point = transform.toWorld(x, y);
    if (!Number.isFinite(point.x) || !Number.isFinite(point.y)) return;
    onGoalSelect(mapPointTarget(clickMapIdentity, { x: point.x, y: point.y, yaw: displayedPose?.yaw ?? 0 }));
    setFollow(false);
  };

  const handleMapPointerMove = (event: ReactMouseEvent<HTMLCanvasElement>) => {
    const rect = event.currentTarget.getBoundingClientRect();
    const hitTag = findNavigationTagAtScreenPoint(
      navigationTags, event.clientX - rect.left, event.clientY - rect.top, transform.toCanvas);
    setHoveredTagId((current) => current === (hitTag?.tag_id ?? null) ? current : hitTag?.tag_id ?? null);
    event.currentTarget.style.cursor = hitTag ? "pointer" : canPick ? "crosshair" : "default";
  };

  const clearMapPointer = (event: ReactMouseEvent<HTMLCanvasElement>) => {
    setHoveredTagId(null);
    event.currentTarget.style.cursor = canPick ? "crosshair" : "default";
  };

  const fit = () => { setZoom(1); setCenter(null); setFollow(false); };
  const recenter = () => { setFollow(true); setCenter(null); };
  const hoveredTag = navigationTags.find((tag) => tag.tag_id === hoveredTagId) ?? null;
  return <div className="robot-detail-map-host" ref={hostRef}>
    <canvas ref={canvasRef} className="robot-detail-map-canvas" data-testid="global-warehouse-map"
      data-map-source="CANONICAL_WAREHOUSE" data-pose-source={displayedPose?.pose_source}
      data-preview-path-frame={canonicalTagPreview ? "CANONICAL" : undefined}
      data-preview-path-point-count={canonicalTagPreview?.path.length ?? 0}
      data-preview-route-node-count={canonicalTagPreview?.canonical_route_points?.filter((point) => point.tag_id !== null).length ?? 0}
      data-render-x={displayedPose?.x} data-render-y={displayedPose?.y}
      data-render-yaw={displayedPose?.yaw}
      data-navigation-tag-count={navigationTags.length}
      data-hovered-tag-id={hoveredTagId ?? undefined}
      data-selected-tag-id={selectedTag?.tag_id}
      onClick={handleMapClick} onMouseMove={handleMapPointerMove} onMouseLeave={clearMapPointer}
      title={hoveredTag ? `${hoveredTag.tag_id} · ${hoveredTag.label}` : undefined}
      aria-label="Canonical warehouse and robot pose map" />
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
    if (pathPreview?.status === "VALID" && pathPreview.source_type === "TAG") {
      drawRouteNodes(ctx, pathPreview.canonical_route_points ?? [], worldToCanvas);
    }
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
function canonicalTagPose(tag: RobotNavigationTag): { x: number; y: number; yaw: number } | null {
  const pose = tag.canonical_navigation_pose
    ?? (tag.map_id === "CANONICAL" ? tag.navigation_pose : null)
    ?? (Number.isFinite(tag.x) && Number.isFinite(tag.y)
      ? { x: tag.x as number, y: tag.y as number, yaw: tag.yaw ?? 0 } : null);
  return pose && Number.isFinite(pose.x) && Number.isFinite(pose.y) && Number.isFinite(pose.yaw)
    ? pose : null;
}

function findNavigationTagAtScreenPoint(
  tags: RobotNavigationTag[], screenX: number, screenY: number,
  worldToCanvas: MapTransform["toCanvas"], hitRadiusPx = 12,
): RobotNavigationTag | null {
  let nearest: RobotNavigationTag | null = null;
  let nearestDistanceSquared = hitRadiusPx * hitRadiusPx;
  for (const tag of tags) {
    const pose = canonicalTagPose(tag);
    if (!pose) continue;
    const point = worldToCanvas(pose.x, pose.y);
    const dx = point.x - screenX, dy = point.y - screenY;
    const distanceSquared = dx * dx + dy * dy;
    if (distanceSquared <= nearestDistanceSquared) {
      nearest = tag;
      nearestDistanceSquared = distanceSquared;
    }
  }
  return nearest;
}

function drawNavigationTagMarkers(
  ctx: CanvasRenderingContext2D, tags: RobotNavigationTag[], selectedTagId: number | null,
  hoveredTagId: number | null, worldToCanvas: MapTransform["toCanvas"],
) {
  ctx.save();
  for (const tag of tags) {
    const pose = canonicalTagPose(tag);
    if (!pose) continue;
    const point = worldToCanvas(pose.x, pose.y);
    const selected = tag.tag_id === selectedTagId;
    const hovered = tag.tag_id === hoveredTagId;
    const navigable = tag.navigable && tag.map_id && tag.map_revision;
    const radius = selected ? 8 : hovered ? 7 : 5;
    ctx.globalAlpha = navigable ? 1 : 0.48;
    ctx.beginPath();
    ctx.arc(point.x, point.y, radius, 0, Math.PI * 2);
    ctx.fillStyle = selected ? "#f2c7ff" : hovered ? "#8de8dc" : "#f4b942";
    ctx.strokeStyle = selected ? "#e19aff" : hovered ? "#37d6c1" : "#08111d";
    ctx.lineWidth = selected || hovered ? 2.5 : 1.5;
    ctx.fill();
    ctx.stroke();
    if (selected || hovered) {
      ctx.globalAlpha = 1;
      ctx.fillStyle = "#f2e8ff";
      ctx.font = "bold 10px JetBrains Mono, monospace";
      ctx.fillText(`${tag.tag_id} · ${tag.label}`, point.x + radius + 4, point.y - radius - 3);
    }
  }
  ctx.restore();
}

function drawRouteNodes(ctx: CanvasRenderingContext2D, points: NonNullable<RobotDetailPathPreview["canonical_route_points"]>, worldToCanvas: MapTransform["toCanvas"]) {
  ctx.save();
  for (const point of points) {
    if (point.tag_id === null || point.kind === "START") continue;
    const p = worldToCanvas(point.x, point.y);
    ctx.beginPath(); ctx.arc(p.x, p.y, point.kind === "TAG_SERVICE" ? 7 : 5, 0, Math.PI * 2);
    ctx.fillStyle = point.kind === "TAG_SERVICE" ? "#ffd166" : "#e8b8ff";
    ctx.strokeStyle = "#08111d"; ctx.lineWidth = 1.5; ctx.fill(); ctx.stroke();
    ctx.fillStyle = "#f2e8ff"; ctx.font = "bold 9px JetBrains Mono, monospace";
    ctx.fillText(String(point.tag_id), p.x + 7, p.y - 6);
  }
  ctx.restore();
}
function drawRobot(ctx: CanvasRenderingContext2D, robotId: string, status: string, pose: FramePose, worldToCanvas: MapTransform["toCanvas"], scale: number) { const p = worldToCanvas(pose.x, pose.y); const color = status === "ERROR" ? "#ef6262" : status === "WARNING" ? "#f3c64e" : "#37d6c1"; ctx.save(); ctx.translate(p.x, p.y); ctx.rotate(-pose.yaw); ctx.fillStyle = "rgba(14, 29, 45, .95)"; ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath(); ctx.rect(-0.60 * scale, -0.30 * scale, 1.20 * scale, 0.60 * scale); ctx.fill(); ctx.stroke(); ctx.fillStyle = color; ctx.beginPath(); ctx.moveTo(0.60 * scale, 0); ctx.lineTo(0.36 * scale, -0.12 * scale); ctx.lineTo(0.36 * scale, 0.12 * scale); ctx.closePath(); ctx.fill(); ctx.restore(); ctx.fillStyle = "#e8f3ff"; ctx.font = "bold 10px JetBrains Mono, monospace"; ctx.fillText(robotId, p.x + 0.64 * scale, p.y - 0.38 * scale); }
