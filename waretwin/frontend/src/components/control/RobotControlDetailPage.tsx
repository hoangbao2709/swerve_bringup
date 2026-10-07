import { Component, memo, useCallback, useEffect, useId, useLayoutEffect, useMemo, useRef, useState, type ErrorInfo, type ReactNode } from "react";
import { apiFetch, clearEmergencyStop, emergencyStop } from "../../services/api";
import { WS_URL, wsManualCommand, wsSetRobotMode, wsSend, type ManualAction } from "../../services/ws";
import { MANUAL_COMMAND_REFRESH_MS, nextManualCommand, type ActiveManualCommand } from "../../services/manualCommand";
import { useStore } from "../../state/store";
import type { RobotDetailError, RobotDetailPathPreview, RobotSystemDiagnostics } from "../../schema/twin_state";
import { ActiveNavigationMap2DView, LocalRobotSection, type ActiveMapLayers } from "./LocalRobotSections";
import type { ControlSection } from "../shell/Sidebar";
import { occupancyRasters } from "./occupancyRaster";
import { detailPerformance } from "./detailPerformance";
import { displayedNavigationMapIdentity as getDisplayedNavigationMapIdentity, mapPointPreviewPayload, mapPointTarget, type MapPointNavigationTarget } from "./navigationMapIdentity";
import { evaluatePreviewApproval } from "./navigationPreviewApproval";
import { resolveDemoRoute } from "./demoRoute";

type HostStatus = { system?: { cpu_load_1m?: number | null; memory?: { used_percent?: number | null } } };
const MANUAL_ACTIONS: Array<{
  action: ManualAction;
  label: string;
  title: string;
}> = [
  { action: "FORWARD_LEFT", label: "↖", title: "Forward left" },
  { action: "FORWARD", label: "▲", title: "Forward (W / ↑)" },
  { action: "FORWARD_RIGHT", label: "↗", title: "Forward right" },

  { action: "LEFT", label: "◀", title: "Strafe left (A / ←)" },
  { action: "STOP", label: "■", title: "Stop" },
  { action: "RIGHT", label: "▶", title: "Strafe right (D / →)" },

  { action: "BACKWARD_LEFT", label: "↙", title: "Backward left" },
  { action: "BACKWARD", label: "▼", title: "Backward (S / ↓)" },
  { action: "BACKWARD_RIGHT", label: "↘", title: "Backward right" },

  { action: "ROTATE_LEFT", label: "↺", title: "Rotate left (Q)" },
  { action: "ROTATE_RIGHT", label: "↻", title: "Rotate right (E)" },
];
const EMPTY_ERRORS: RobotDetailError[] = [];

function isMapPointTarget(target: MapPointNavigationTarget | null): target is MapPointNavigationTarget {
  return Boolean(target && target.frame_id === "map" && target.source_type === "ACTIVE_MAP_POINT"
    && target.map_id && target.map_revision && target.source_map_id && target.source_map_revision);
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

export function RobotControlDetailPage({ robotId, activeSection = "CONTROL", onSectionChange = () => undefined }: {
  robotId: string;
  activeSection?: ControlSection;
  onSectionChange?: (section: ControlSection) => void;
}) {
  return <RobotDetailErrorBoundary><RobotControlDetailContent robotId={robotId} activeSection={activeSection} onSectionChange={onSectionChange} /></RobotDetailErrorBoundary>;
}

function RobotControlDetailContent({ robotId, activeSection, onSectionChange }: {
  robotId: string;
  activeSection: ControlSection;
  onSectionChange: (section: ControlSection) => void;
}) {

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
  const runtimeMode = useStore((state) => state.runtimeMode);
  const runtimeState = useStore((state) => state.runtimeState);
  const runtimeCapabilities = useStore((state) => state.robotCapabilities[robotId] ?? null);
  const rosConnected = useStore((state) => state.rosConnected);
  const connectedRobotIds = useStore((state) => state.connectedRobotIds);
  const websocketState = useStore((state) => state.websocketState);
  const slam2dMap = useStore((state) => state.robotDetail[robotId]?.slam2dMap ?? null);
  const runtimeMapSnapshot = useStore((state) => state.robotDetail[robotId]?.runtimeMapSnapshot ?? null);
  const detailDiagnostics = useStore((state) => state.robotDetail[robotId]?.diagnostics ?? null);
  const detailErrors = useStore((state) => state.robotDetail[robotId]?.errors ?? EMPTY_ERRORS);
  const controlErrors = detailErrors.length ? detailErrors : detailDiagnostics?.errors ?? diagnostics?.errors ?? EMPTY_ERRORS;
  const errorLogTone = controlErrors.some((item) => item.severity === "CRITICAL" || item.severity === "ERROR") ? "error"
    : controlErrors.some((item) => item.severity === "WARNING") ? "warning"
      : controlErrors.length ? "info" : "neutral";
  const remainingDistanceM = useStore((state) => state.robotDetail[robotId]?.remainingDistanceM ?? null);
  const mappingScan = useStore((state) => state.robotDetail[robotId]?.scan ?? null);
  const mappingSessionId = useStore((state) => state.robotDetail[robotId]?.mappingSessionId ?? null);
  const navigationStatus = useStore((state) => state.robotDetail[robotId]?.navigationStatus ?? null);
  const globalPath = useStore((state) => state.robotDetail[robotId]?.globalPath ?? null);
  const navigationGoal = useStore((state) => state.robotDetail[robotId]?.goal ?? null);
  const lidarStreamDiagnostics = useStore((state) => state.robotDetail[robotId]?.lidarStreamDiagnostics ?? null);
  const pathPreview = useStore((state) => state.robotDetail[robotId]?.pathPreview ?? null);
  const activeLocalMapId = useStore((state) => state.robotDetail[robotId]?.activeLocalMapId ?? null);
  const activeLocalMapRevision = useStore((state) => state.robotDetail[robotId]?.activeLocalMapRevision ?? null);
  const localMapSyncStatus = useStore((state) => state.robotDetail[robotId]?.localMapSyncStatus ?? null);
  const slamRuntimeActive = runtimeState === "MAPPING" || runtimeState === "UNIFIED";
  const useLiveSlamMap = slamRuntimeActive && !activeLocalMapId;
  const setRobotDetail = useStore((state) => state.setRobotDetail);
  const appliedMode = useStore((state) => state.robotDetail[robotId]?.appliedMode);
  const requestedMode = useStore((state) => state.robotDetail[robotId]?.requestedMode);
  const modeTransitionState = useStore((state) => state.robotDetail[robotId]?.modeTransitionState);
  const mapSync = useStore((state) => state.mapSync);
  const [controlMode, setControlMode] = useState<"MANUAL" | "AUTONOMOUS">(robot?.control_mode ?? "AUTONOMOUS");
  const [activeManualCommand, setActiveManualCommand] = useState<ActiveManualCommand | null>(null);
  const [goalPreview, setGoalPreview] = useState<MapPointNavigationTarget | null>(null);
  const [pointYawPending, setPointYawPending] = useState(false);
  const [pointSelectionResetVersion, setPointSelectionResetVersion] = useState(0);
  const [mapLayers, setMapLayers] = useState<ActiveMapLayers>({ robot: true, scan: true, path: true, trajectory: false, grid: false });
  const [systemInputsExpanded, setSystemInputsExpanded] = useState(true);
  const [statePanelExpanded, setStatePanelExpanded] = useState(true);
  const [operationPanelExpanded, setOperationPanelExpanded] = useState(true);
  const [manualPanelExpanded, setManualPanelExpanded] = useState(true);
  const [errorLogOpen, setErrorLogOpen] = useState(false);
  const closeErrorLog = useCallback(() => setErrorLogOpen(false), []);
  const panelId = useId();
  const [pathRequestState, setPathRequestState] = useState<"IDLE" | "PLANNING" | "WAITING_FOR_NAV2">("IDLE");
  const latestPathRequest = useRef("");
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
  const activeMappingSnapshot = useLiveSlamMap && slam2dMap?.map_source === "SLAM_TOOLBOX"
    && slam2dMap.frame_id === "map" && Boolean(slam2dMap.mapping_session_id)
    && slam2dMap.active_map_id === `SLAM-${slam2dMap.mapping_session_id}`
    && slam2dMap.active_map_revision === `session-${slam2dMap.mapping_session_id}`
    && (!mappingSessionId || slam2dMap.mapping_session_id === mappingSessionId) ? slam2dMap : null;
  const activeMapSnapshot = useLiveSlamMap ? activeMappingSnapshot : runtimeMapSnapshot;
  const activeMapId = useLiveSlamMap ? activeMappingSnapshot?.active_map_id ?? null
    : activeLocalMapId ?? activeMapSnapshot?.active_map_id ?? null;
  const activeMapRevision = useLiveSlamMap ? activeMappingSnapshot?.active_map_revision ?? null
    : activeLocalMapRevision ?? (activeMapSnapshot?.active_map_id === activeMapId
      ? activeMapSnapshot.active_map_revision ?? null : null);
  const activeMap2dSnapshot = activeMapSnapshot?.active_map_id === activeMapId
      && String(activeMapSnapshot.active_map_revision ?? "") === String(activeMapRevision ?? "")
    ? activeMapSnapshot : null;
  const localizationMapSnapshot = activeLocalMapId
    ? runtimeMapSnapshot?.map_source === "LOCAL_MAP"
      && runtimeMapSnapshot.active_map_id === activeLocalMapId
      && String(runtimeMapSnapshot.active_map_revision ?? "") === String(activeLocalMapRevision ?? "")
      ? runtimeMapSnapshot : null
    : slamRuntimeActive ? slam2dMap : runtimeMapSnapshot;
  useEffect(() => {
    if (!runtimeMapSnapshot) return;
    const matchesActiveLocalMap = Boolean(activeLocalMapId
      && runtimeMapSnapshot.map_source === "LOCAL_MAP"
      && runtimeMapSnapshot.active_map_id === activeLocalMapId
      && String(runtimeMapSnapshot.active_map_revision ?? "") === String(activeLocalMapRevision ?? ""));
    if ((runtimeMapSnapshot.map_source === "LOCAL_MAP" || activeLocalMapId) && !matchesActiveLocalMap) {
      setRobotDetail(robotId, { runtimeMapSnapshot: null });
    }
  }, [activeLocalMapId, activeLocalMapRevision, robotId, runtimeMapSnapshot, setRobotDetail]);
  const activeMapStatus = useLiveSlamMap ? activeMappingSnapshot ? "SLAM · LIVE · LOCAL_ONLY" : "WAITING FOR SLAM MAP"
    : activeLocalMapId
      ? localMapSyncStatus ?? (activeMap2dSnapshot ? "LOCAL_ONLY" : "WAITING FOR LOCAL MAP")
      : activeMap2dSnapshot ? "ACTIVE MAP" : "WAITING FOR ACTIVE MAP";
  const activeMapReady = Boolean(activeMap2dSnapshot && activeMapId && activeMapRevision
    && activeMap2dSnapshot.frame_id === "map"
    && ["SLAM_TOOLBOX", "LOCAL_MAP", "NAV2_MAP"].includes(activeMap2dSnapshot.map_source ?? ""));
  const nav2Ready = runtimeCapabilities?.nav2_ready === true && diagnostics?.nav2_ready === true;
  const navigationUiAvailable = Boolean(runtimeCapabilities?.goal_available
    && nav2Ready && controlMode === "AUTONOMOUS" && controlOnline);
  const previewApproval = evaluatePreviewApproval({
    pathPreview, requestId: latestPathRequest.current, target: goalPreview,
    activeMapId, activeMapRevision, now: clockNow,
  });
  const approvedPreview: RobotDetailPathPreview | null = previewApproval.approvedPreview;
  const displayedNavigationMapIdentity = getDisplayedNavigationMapIdentity({
    active_map_id: activeMapId, active_map_revision: activeMapRevision,
    map_snapshot: activeMap2dSnapshot,
  });
  const displayRoute = resolveDemoRoute({
    navigationStatus: navigationStatus ?? null,
    activeMapId,
    activeMapRevision,
    mapSource: activeMap2dSnapshot?.map_source ?? null,
    mapContentRevision: activeMap2dSnapshot?.map_content_revision ?? null,
    registrationRevision: runtimeCapabilities?.registration_revision,
    approvedPreview,
    globalPath,
  });
  const navigationIsActive = ["ACTIVE", "NAVIGATING", "PAUSED"].includes(String(navigationStatus ?? "").toUpperCase());
  const activeGoalTarget = navigationIsActive && displayedNavigationMapIdentity && navigationGoal
    && navigationGoal.frame_id === "map"
    && navigationGoal.active_map_id === activeMapId
    && String(navigationGoal.active_map_revision ?? "") === String(activeMapRevision ?? "")
    ? mapPointTarget(displayedNavigationMapIdentity, navigationGoal) : null;

  const activeNavigationMapKey = JSON.stringify([robotId, activeMapId, activeMapRevision, activeMapReady]);
  const previousActiveNavigationMapKey = useRef(activeNavigationMapKey);

  useEffect(() => {
    select(robotId);
    setGoalPreview(null);
    setPointYawPending(false);
    latestPathRequest.current = "";
    setPathRequestState("IDLE");
    setRobotDetail(robotId, { pathPreview: null, globalPath: null, localPath: null });
  }, [robotId, select, setRobotDetail]);

  useEffect(() => {
    if (previousActiveNavigationMapKey.current === activeNavigationMapKey) return;
    previousActiveNavigationMapKey.current = activeNavigationMapKey;
    wsSend({ type: "PATH_PREVIEW_INVALIDATE", robot_id: robotId });
    latestPathRequest.current = "";
    setPathRequestState("IDLE");
    setGoalPreview(null);
    setPointYawPending(false);
    setRobotDetail(robotId, { pathPreview: null, globalPath: null, localPath: null, goal: null });
  }, [activeNavigationMapKey, robotId, setRobotDetail]);

  useEffect(() => {
    if (runtimeState === "MAPPING") {
      setGoalPreview(null);
      setPointYawPending(false);
    }
  }, [runtimeState]);

  useEffect(() => {
    setControlMode(appliedMode ?? robot?.control_mode ?? "AUTONOMOUS");
  }, [appliedMode, robot?.control_mode, robotId]);

  useEffect(() => {
    if (controlMode === "MANUAL") setManualPanelExpanded(true);
    else setOperationPanelExpanded(true);
  }, [controlMode]);

  useEffect(() => {
    const timer = window.setInterval(() => setClockNow(Date.now()), 1000);
    return () => window.clearInterval(timer);
  }, []);

  const detailView = "LIDAR_2D" as const;
  const displayedMapPointPickIdentity = activeMapReady
    && controlOnline && controlMode === "AUTONOMOUS" ? displayedNavigationMapIdentity : null;
  useEffect(() => {
    if (!robotBridgeOnline || websocketState !== "CONNECTED") return;
    const request_id = `${robotId}-${Date.now()}-${Math.random().toString(36).slice(2, 8)}`;
    setRobotDetail(robotId, { viewStatus: { robot_id: robotId, requested_view: detailView, request_id, state: "REQUESTED" } });
    wsSend({ type: "ROBOT_DETAIL_VIEW", robot_id: robotId, view: detailView, request_id, delivery_ack: true });
  }, [activeLocalMapId, activeLocalMapRevision, detailView, robotBridgeOnline, robotId, websocketState, setRobotDetail]);
  useLayoutEffect(() => {
    const cached = occupancyRasters.peek(activeMap2dSnapshot);
    if (!cached) return;
    const paint = requestAnimationFrame(() => detailPerformance("view_render", { view: detailView, robot_id: robotId, useful: true, cached: true }));
    return () => cancelAnimationFrame(paint);
    // This measures a cached canvas becoming visible, not a newly received frame.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [activeMap2dSnapshot, detailView, robotId]);

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
    setRobotDetail(robotId, { pathPreview: null, globalPath: null, localPath: null });
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
    })) {
      setError("Navigation goal was not sent");
      return;
    }
    setGoalPreview(null);
    setPointYawPending(false);
    latestPathRequest.current = "";
    setPathRequestState("IDLE");
    setRobotDetail(robotId, { pathPreview: null, globalPath: null, localPath: null });
    setError("");
  };

  const selectMapPoint = useCallback((target: MapPointNavigationTarget) => {
    const sourceMatches = target.source_type === "ACTIVE_MAP_POINT"
      && target.source_map_id === activeMapId
      && target.source_map_revision === activeMapRevision;
    if (target.frame_id !== "map" || !sourceMatches || !activeMapReady) {
      setError("Map point selection belongs to a map identity or revision that is no longer current");
      return;
    }
    wsSend({ type: "PATH_PREVIEW_INVALIDATE", robot_id: robotId });
    setGoalPreview(target);
    setPointYawPending(true);
    latestPathRequest.current = "";
    setPathRequestState("IDLE");
    setRobotDetail(robotId, { pathPreview: null });
    setError("");
  }, [activeMapId, activeMapReady, activeMapRevision, robotId, setRobotDetail]);

  const requestPathPreview = useCallback((target: MapPointNavigationTarget) => {
    setError("");
    if (!activeMapReady) {
      latestPathRequest.current = "";
      setPathRequestState("IDLE");
      setRobotDetail(robotId, { pathPreview: null });
      setError(`Path planning is blocked: the selected robot active map is not confirmed (${activeMapStatus})`);
      return;
    }
    const pointTarget = isMapPointTarget(target) ? target : null;
    if (!pointTarget || pointTarget.source_map_id !== activeMapId
        || pointTarget.source_map_revision !== activeMapRevision) {
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
    if (!nav2Ready || !runtimeCapabilities?.goal_available) {
      latestPathRequest.current = "";
      setPathRequestState("WAITING_FOR_NAV2");
      setRobotDetail(robotId, { pathPreview: null });
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
    const sent = pointPayload ? wsSend({ ...common, ...pointPayload }) : false;
    if (!sent) {
      latestPathRequest.current = "";
      setPathRequestState("IDLE");
      setError("Path preview request was not sent because the WebSocket is disconnected");
      return;
    }
    setGoalPreview(target);
  }, [activeMap2dSnapshot?.map_content_revision, activeMapId, activeMapReady, activeMapRevision, activeMapStatus, controlMode, controlOnline, nav2Ready, robotId, runtimeCapabilities?.goal_available, setRobotDetail]);

  const confirmMapPoint = useCallback((target: MapPointNavigationTarget) => {
    selectMapPoint(target);
    setPointYawPending(false);
    requestPathPreview(target);
  }, [requestPathPreview, selectMapPoint]);

  useEffect(() => {
    if (pathRequestState !== "WAITING_FOR_NAV2" || pointYawPending || !goalPreview || !navigationUiAvailable) return;
    requestPathPreview(goalPreview);
  }, [goalPreview, navigationUiAvailable, pathRequestState, pointYawPending, requestPathPreview]);

  useEffect(() => {
    if (pathPreview?.request_id && pathPreview.request_id === latestPathRequest.current) setPathRequestState("IDLE");
  }, [pathPreview]);

  const cancelPathPreview = () => {
    wsSend({ type: "PATH_PREVIEW_INVALIDATE", robot_id: robotId });
    latestPathRequest.current = "";
    setGoalPreview(null);
    setPointYawPending(false);
    setPathRequestState("IDLE");
    setRobotDetail(robotId, { pathPreview: null });
    setPointSelectionResetVersion((version) => version + 1);
  };

  const navCommand = (type: "NAV_CANCEL" | "NAV_PAUSE" | "NAV_RESUME") => {
    if (!controlOnline) { setError("Navigation control requires an online ROS bridge"); return; }
    if (!wsSend({ type, robot_id: robotId })) setError("Navigation command was not sent");
  };

  const changeRobot = (next: string) => {
    if (next) select(next);
  };

  const moveButtonEvents = useCallback((action: ManualAction) => ({
    onClick: () => toggleManual(action),
  }), [toggleManual]);

  const slamLive = diagnostics?.mapping?.slam_state === "ACTIVE" && diagnostics.mapping.map_live;
  const rawNavigationState = String(navigationStatus ?? robot?.navigation_state ?? "").toUpperCase();
  const navigationLabel = ["", "IDLE"].includes(rawNavigationState)
    ? nav2Ready ? "READY" : "UNAVAILABLE"
    : ["ACTIVE", "NAVIGATING"].includes(rawNavigationState) ? "RUNNING" : rawNavigationState;
  const selectedPoint = goalPreview;
  const nav2BlockerReason = runtimeCapabilities?.goal_blocker_reason
    ?? diagnostics?.nav2_lifecycle_blocker_reason ?? "";
  const localSection = activeSection === "DIAGNOSIS" ? "DIAGNOSTICS"
    : activeSection === "VDA5050" ? "VDA5050"
      : activeSection === "MAPS" ? "MAPS"
        : activeSection === "MAPPING" ? "MAPPING" : "LOCALIZATION";
  const dashboardStateRows: Array<[string, string]> = [
    ["CURRENT STATE", runtimeState], ["CONTROL MODE", controlMode],
    ["MAPPING", diagnostics?.mapping?.slam_state ?? "UNKNOWN"],
    ["LOCALIZATION", activeLocalMapId ? runtimeCapabilities?.localization_ready ? "LOCALIZED" : "INITIAL POSE REQUIRED" : safeText(localization, "UNKNOWN")],
    ["NAVIGATION", navigationLabel], ["E-STOP", estopActive ? "ACTIVE" : "CLEAR"],
  ];
  const systemInputRows: Array<[string, string]> = [
    ["ACTIVE MAP", activeMapId ?? "WAITING"],
    ["MAP SOURCE", activeMap2dSnapshot?.map_source?.replace(/_/g, " ") ?? "WAITING"],
    ["LIDAR", diagnostics?.mapping?.scan_live ? "LIVE" : diagnostics?.lidar ? "READY" : "UNAVAILABLE"],
    ["ODOMETRY", diagnostics?.mapping?.odom_live ? "RECEIVING" : "WAITING"],
    ["ROS BRIDGE", controlOnline ? "ONLINE" : "OFFLINE"],
    ["SLAM", slamLive ? "LIVE" : diagnostics?.slam ? "READY" : "INACTIVE"],
    ["NAV2", nav2Ready ? "READY" : nav2BlockerReason ? "BLOCKED" : diagnostics?.nav2 ? "STARTING" : "INACTIVE"],
  ];

  return (
    <div className={`robot-detail-shell industrial-hmi${activeSection === "CONTROL" ? " is-control-view" : ""}${activeSection === "CONTROL" && import.meta.env.VITE_DEMO_COMPACT_VIEW === "true" ? " demo-compact-map" : ""}`}>
      <header className="robot-detail-header">
        <div className="robot-detail-identity">
          {activeSection === "CONTROL" && <span className="hmi-app-identity">WARETWIN</span>}
          <label className="robot-detail-robot-select"><span>ROBOT</span><select aria-label="Select robot" value={robotId} onChange={(event) => changeRobot(event.target.value)}><option value="">Select robot</option>{robotIds.map((id) => <option key={id} value={id}>{id}</option>)}</select></label>
          <span className={`hmi-state-pill ${robotOnline ? "is-ready" : "is-fault"}`} data-testid="robot-connection-state">{robotOnline ? "ONLINE" : "OFFLINE"}</span>
          <span className="robot-detail-mode">{modeTransitionState === "REQUESTED" ? `${controlMode} → ${requestedMode} REQUESTED` : modeTransitionState === "FAILED" ? `${controlMode} TRANSITION FAILED` : controlMode}</span>
          <span className={`hmi-state-pill ${slamLive ? "is-ready" : "is-warning"}`} data-testid="slam-runtime-state">SLAM {slamLive ? "LIVE" : safeText(diagnostics?.mapping?.slam_state, "STARTING")}</span>
          <span className={`hmi-state-pill ${nav2Ready ? "is-ready" : diagnostics?.nav2 ? "is-warning" : "is-fault"}`} data-testid="nav2-runtime-state">NAV2 {nav2Ready ? "READY" : nav2BlockerReason ? "BLOCKED" : diagnostics?.nav2 ? "STARTING" : "UNAVAILABLE"}</span>
          <span className={`hmi-state-pill ${estopActive ? "is-fault" : "is-ready"}`} data-testid="estop-state">E-STOP {estopActive ? "ACTIVE" : "CLEAR"}</span>
        </div>
        <div className="robot-detail-header-actions">
          <button type="button" className={controlMode === "MANUAL" ? "is-active" : ""} disabled={!controlOnline || busy} onClick={() => setMode("MANUAL")}>MANUAL</button>
          <button type="button" className={controlMode === "AUTONOMOUS" ? "is-active" : ""} disabled={!controlOnline || busy} onClick={() => setMode("AUTONOMOUS")}>AUTONOMOUS</button>
          <button type="button" className="robot-console-danger hmi-estop" disabled={busy || !robotId} onClick={() => { stopManual(); void runAction(() => emergencyStop(robotId)); }}>EMERGENCY STOP</button>
          <button type="button" className="hmi-clear-stop" disabled={busy || !robotId} onClick={() => void runAction(async () => {
            const result = await clearEmergencyStop(robotId);
            if (result.code !== "CLEAR_ESTOP_APPLIED" || result.emergency_stop_active
                || !result.pre_stop_navigation_terminal) throw new Error("CLEAR_ESTOP_UNCONFIRMED: runtime did not confirm the stop latch is clear");
            setSafetyNotice("CLEAR_ESTOP_APPLIED · bridge confirmed latch clear and prior navigation terminal");
          })}>CLEAR STOP</button>
        </div>
      </header>

      {(safetyNotice || error) && <div className={`hmi-global-feedback ${error ? "is-error" : "is-success"}`} role={error ? "alert" : "status"}>{error || safetyNotice}</div>}

      {activeSection !== "CONTROL" ? <main className={`robot-detail-section-main hmi-section-main hmi-section-${activeSection.toLowerCase()}`}>
        <header className="hmi-section-heading"><div><span>ROBOT {robotId}</span><h2>{activeSection === "VDA5050" ? "VDA5050 CONFIGURATION" : activeSection === "DIAGNOSIS" ? "DIAGNOSIS / SYSTEM" : activeSection}</h2></div>
          {activeSection === "VDA5050" && <button type="button" className="hmi-secondary-action" onClick={() => onSectionChange("DIAGNOSIS")}>BACK TO DIAGNOSIS</button>}
        </header>
        <LocalRobotSection section={localSection} robotId={robotId} robot={robot} slam2dMap={slam2dMap} runtimeMapSnapshot={runtimeMapSnapshot} localizationMap={localizationMapSnapshot} scan={mappingScan} diagnostics={detailDiagnostics ?? diagnostics} errors={detailErrors.length ? detailErrors : detailDiagnostics?.errors ?? diagnostics?.errors ?? EMPTY_ERRORS} controlOnline={controlOnline} controlMode={controlMode} runtimeMode={runtimeMode} runtimeState={runtimeState} runtimeCapabilities={runtimeCapabilities} localization={localization} websocketState={websocketState} mapRevision={mapSync.publishedRevision} activeLocalMapId={activeLocalMapId} activeLocalMapRevision={activeLocalMapRevision} localMapSyncStatus={localMapSyncStatus} lidarStreamDiagnostics={lidarStreamDiagnostics} mappingSessionId={mappingSessionId} hostStatus={host} ensureManualMode={() => setMode("MANUAL")} />
      </main> : <main className="robot-detail-main hmi-control-main">
        <section className="robot-detail-map-panel">
          <div className="robot-detail-view-stack" key={robotId}>
            <div className="robot-detail-view-layer is-active" data-view="LIDAR_2D">
              <ActiveNavigationMap2DView map={activeMap2dSnapshot} robot={robot} scan={mappingScan}
                compactWorkspace
                target={isMapPointTarget(goalPreview) ? goalPreview : activeGoalTarget}
                navigationPath={displayRoute?.points ?? []} navigationPathKind={displayRoute?.kind ?? null}
                layers={mapLayers} onLayerToggle={(layer) => setMapLayers((current) => ({ ...current, [layer]: !current[layer] }))}
                errorLogCount={controlErrors.length} errorLogTone={errorLogTone} onOpenErrorLog={() => setErrorLogOpen(true)}
                canPick={Boolean(displayedMapPointPickIdentity)} onPick={selectMapPoint} onConfirmPoint={confirmMapPoint}
                pointSelectionResetVersion={pointSelectionResetVersion}
                onCancelPointSelection={cancelPathPreview}
                onInteractionModeChange={(mode) => { if (mode === "PAN" && pointYawPending) cancelPathPreview(); }} />
            </div>
          </div>
          <div className="hmi-dashboard-panels" aria-label="Map system status">
            <DashboardStatusPanel title="SYSTEM INPUTS" rows={systemInputRows} expanded={systemInputsExpanded} controls={`${panelId}-inputs`} onToggle={() => setSystemInputsExpanded((value) => !value)} />
            <DashboardStatusPanel title="STATE" rows={dashboardStateRows} expanded={statePanelExpanded} controls={`${panelId}-state`} onToggle={() => setStatePanelExpanded((value) => !value)} />
          </div>

        <div className="hmi-right-floating-group" aria-label="Robot control cards" data-testid="control-operation-panels" data-control-mode={controlMode}>
        {controlMode === "AUTONOMOUS" && <>
        <aside className="hmi-operation-panel hmi-floating-panel" aria-label="Robot operation panel" data-expanded={operationPanelExpanded}>
          <header><div><span>ROBOT / MISSION</span><h2>{robotId}</h2></div><span className={`hmi-state-pill ${robotOnline ? "is-ready" : "is-fault"}`}>{robotOnline ? "ONLINE" : "OFFLINE"}</span><PanelToggle title="Robot / Mission" expanded={operationPanelExpanded} controls={`${panelId}-operation`} onToggle={() => setOperationPanelExpanded((value) => !value)} /></header>
          <div className="hmi-floating-content" id={`${panelId}-operation`} hidden={!operationPanelExpanded}>
          <div className="hmi-operation-status">
            <div><span>NAV</span><b className={`hmi-state-pill ${navigationLabel === "READY" || navigationLabel === "SUCCEEDED" ? "is-ready" : navigationLabel === "RUNNING" ? "is-running" : statusClass(navigationLabel) === "error" ? "is-fault" : "is-warning"}`}>{navigationLabel}</b></div>
            <div></div>
            <div><span>REMAINING</span><b>{safeNumber(remainingDistanceM, 2, " m")}</b></div>
            {currentMission && <div><span>MISSION</span><b>{`${currentMission.id} · ${currentMission.status}`}</b></div>}
          </div>
          <section className="hmi-destination">
            <h3>DESTINATION</h3>
            {selectedPoint ? <><p className="hmi-point-selected" data-testid="selected-point-status">{pointYawPending ? "SELECT YAW" : "POINT SELECTED"}</p><div className="hmi-coordinate-grid">
              <span>X</span><b>{safeNumber(selectedPoint.x, 3)} m</b>
              <span>Y</span><b>{safeNumber(selectedPoint.y, 3)} m</b>
              <span>YAW</span><b>{safeNumber(selectedPoint.yaw, 2)} rad</b>
            </div></> : <p>NO TARGET</p>}
          </section>
          <div className="hmi-navigation-actions">
            <button type="button" aria-label="SEND GOAL" disabled={!navigationUiAvailable || !goalPreview || !approvedPreview || !controlOnline || !activeMapReady} className="robot-console-primary hmi-send-goal" onClick={sendGoal}>SEND</button>
            <button type="button" className="hmi-cancel-point" disabled={!goalPreview} onClick={cancelPathPreview}>CANCEL</button>
          </div>
          {!nav2Ready && nav2BlockerReason && <div className="hmi-nav-blocker" role="status"><b>GOALS UNAVAILABLE</b>
            {(runtimeCapabilities?.goal_blocker_code ?? diagnostics?.nav2_lifecycle_blocker_code) && <code>{runtimeCapabilities?.goal_blocker_code ?? diagnostics?.nav2_lifecycle_blocker_code}</code>}
            <span>{nav2BlockerReason}</span></div>}
          <div className="hmi-nav-management" aria-label="Navigation actions">
            <button type="button" disabled={!controlOnline} onClick={() => navCommand("NAV_PAUSE")}>PAUSE</button>
            <button type="button" disabled={!controlOnline} onClick={() => navCommand("NAV_RESUME")}>RESUME</button>
            <button type="button" disabled={!controlOnline} onClick={() => navCommand("NAV_CANCEL")}>CANCEL NAV</button>
          </div>
          <div className="hmi-preview-state" role="status" data-testid="point-navigation-state">
            {!goalPreview ? "WAITING FOR DESTINATION" : pointYawPending ? "SELECT YAW" : pathRequestState === "PLANNING" ? "PLANNING" : pathRequestState === "WAITING_FOR_NAV2" ? "WAITING FOR NAV2" : approvedPreview?.status === "VALID" ? `PREVIEW VALID · ${safeNumber(approvedPreview.path_length_m, 2, " m")}` : approvedPreview?.reason ?? "POINT SELECTED"}
          </div>
          </div>
        </aside>
        </>}
        {controlMode === "MANUAL" && <ManualBar controlMode={controlMode} controlOnline={controlOnline} activeManualCommand={activeManualCommand} moveButtonEvents={moveButtonEvents} expanded={manualPanelExpanded} controls={`${panelId}-manual`} onToggle={() => setManualPanelExpanded((value) => !value)} />}
        </div>
          {errorLogOpen && <ErrorLog errors={controlErrors} onClose={closeErrorLog} />}
        </section>
      </main>
      }
    </div>
  );
}

const ManualBar = memo(ManualBarContent);

function PanelToggle({ title, expanded, controls, onToggle }: { title: string; expanded: boolean; controls: string; onToggle: () => void }) {
  return <button type="button" className="hmi-panel-toggle" aria-label={`${expanded ? "Collapse" : "Expand"} ${title}`} aria-expanded={expanded} aria-controls={controls} onClick={onToggle}><span aria-hidden="true">{expanded ? "▴" : "▾"}</span></button>;
}

function ManualBarContent({
  controlMode,
  controlOnline,
  activeManualCommand,
  moveButtonEvents,
  expanded,
  controls,
  onToggle,
}: {
  controlMode: string;
  controlOnline: boolean;
  activeManualCommand: ActiveManualCommand | null;
  moveButtonEvents: (action: ManualAction) => { onClick: () => void };
  expanded: boolean;
  controls: string;
  onToggle: () => void;
}) {
  const button = (action: ManualAction, key = "") => {
    const item = MANUAL_ACTIONS.find(
      (candidate) => candidate.action === action,
    )!;

    return (
      <button
        type="button"
        key={item.action}
        className={`manual-key manual-key-${item.action.toLowerCase()}${
          activeManualCommand === item.action ? " is-active" : ""
        }`}
        title={item.title}
        aria-label={item.title}
        aria-pressed={
          item.action !== "STOP" &&
          activeManualCommand === item.action
        }
        disabled={
          !controlOnline ||
          (item.action !== "STOP" && controlMode !== "MANUAL")
        }
        {...moveButtonEvents(item.action)}
      >
        {item.label}
        {key && <small>{key}</small>}
      </button>
    );
  };

  return (
    <section
      className="robot-detail-manual hmi-manual-jog-panel hmi-floating-panel"
      aria-label="Manual jog panel"
      data-testid="manual-jog-panel"
      data-expanded={expanded}
    >
      <header className="robot-detail-manual-head hmi-floating-header">
        <div>
          <span className="hmi-manual-kicker">MANUAL JOG</span>
          <b>
            {activeManualCommand
              ? `${activeManualCommand.replace(/_/g, " ")} LATCHED`
              : "STOPPED"}
          </b>
        </div>

        <span
          className={`hmi-state-pill ${
            controlMode === "MANUAL"
              ? "is-running"
              : "is-neutral"
          }`}
        >
          {controlMode === "AUTONOMOUS" ? "AUTO" : "MANUAL"}
        </span>
        <PanelToggle title="Manual jog" expanded={expanded} controls={controls} onToggle={onToggle} />
      </header>

      <div className="hmi-floating-content" id={controls} hidden={!expanded}>
      <div className="hmi-jog-layout">
        <div
          className="hmi-jog-pad hmi-jog-pad-8way"
          role="group"
          aria-label="Manual movement controls"
        >
          {button("FORWARD_LEFT")}
          {button("FORWARD", "W")}
          {button("FORWARD_RIGHT")}

          {button("LEFT", "A")}
          {button("STOP", "SPACE")}
          {button("RIGHT", "D")}

          {button("BACKWARD_LEFT")}
          {button("BACKWARD", "S")}
          {button("BACKWARD_RIGHT")}
        </div>

        <div
          className="hmi-rotate-pad"
          role="group"
          aria-label="Manual rotation controls"
        >
          {button("ROTATE_LEFT", "Q")}
          {button("ROTATE_RIGHT", "E")}
        </div>
      </div>
      </div>
    </section>
  );
}

function DashboardStatusPanel({ title, rows, expanded, controls, onToggle }: { title: string; rows: Array<[string, string]>; expanded: boolean; controls: string; onToggle: () => void }) {
  return <section className="hmi-dashboard-status-panel hmi-floating-panel" aria-label={title} data-expanded={expanded}>
    <header className="hmi-floating-header"><b>{title}</b><PanelToggle title={title} expanded={expanded} controls={controls} onToggle={onToggle} /></header>
    <div className="hmi-floating-content" id={controls} hidden={!expanded}><dl>{rows.map(([label, value]) => {
      const state = value.toUpperCase();
      const tone = ["READY", "LIVE", "ONLINE", "RECEIVING", "LOCALIZED", "CLEAR", "ACTIVE", "RUNNING"].includes(state)
        ? "is-good" : ["BLOCKED", "INITIAL POSE REQUIRED", "WAITING", "STARTING", "UNKNOWN"].includes(state)
          ? "is-warning" : ["OFFLINE", "UNAVAILABLE", "INACTIVE", "ERROR", "FAILED"].includes(state) ? "is-fault" : "is-neutral";
      return <div key={label}><dt>{label}</dt><dd className={tone}>{value}</dd></div>;
    })}</dl></div>
  </section>;
}

function ErrorLog({ errors, onClose }: { errors: RobotDetailError[]; onClose: () => void }) {
  const titleId = useId();
  useEffect(() => {
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key === "Escape") onClose();
    };
    document.addEventListener("keydown", onKeyDown);
    return () => document.removeEventListener("keydown", onKeyDown);
  }, [onClose]);
  const newestFirst = [...errors].sort((left, right) => {
    const leftTime = left.timestamp ? Date.parse(left.timestamp) : Number.NaN;
    const rightTime = right.timestamp ? Date.parse(right.timestamp) : Number.NaN;
    if (Number.isFinite(leftTime) && Number.isFinite(rightTime)) return rightTime - leftTime;
    if (Number.isFinite(leftTime)) return -1;
    if (Number.isFinite(rightTime)) return 1;
    return 0;
  }).slice(0, 40);

  return <div className="hmi-error-modal-backdrop" data-testid="control-error-log-backdrop"
    onClick={(event) => { if (event.target === event.currentTarget) onClose(); }}>
    <section className="hmi-error-modal" role="dialog" aria-modal="true" aria-labelledby={titleId}
      aria-label="Error log" data-testid="control-error-log" data-error-count={errors.length}>
    <header><h2 id={titleId}>ERROR LOG</h2><span>{errors.length} {errors.length === 1 ? "ENTRY" : "ENTRIES"}</span>
      <button type="button" className="hmi-error-modal-close" aria-label="Close error log" onClick={onClose}>×</button></header>
    <div className="hmi-error-modal-body">
    {newestFirst.length === 0 ? <p className="hmi-error-log-empty">No active errors.</p> : <div className="hmi-error-log-table-wrap">
      <table className="hmi-error-log-table">
        <thead><tr><th scope="col">TIME</th><th scope="col">LEVEL</th><th scope="col">SOURCE</th><th scope="col">MESSAGE</th></tr></thead>
        <tbody>{newestFirst.map((item, index) => {
          const level = String(item.severity ?? "ERROR").toUpperCase();
          const source = (item as RobotDetailError & { source?: string | null }).source;
          const parsedTime = item.timestamp ? Date.parse(item.timestamp) : Number.NaN;
          const displayTime = Number.isFinite(parsedTime) ? new Date(parsedTime).toLocaleTimeString() : "—";
          return <tr key={`${item.timestamp ?? "untimed"}-${item.code ?? item.message}-${index}`}>
            <td>{displayTime}</td>
            <td><span className={`hmi-error-level is-${level.toLowerCase()}`}>{level}</span></td>
            <td>{source || "—"}</td>
            <td>{item.message}</td>
          </tr>;
        })}</tbody>
      </table>
    </div>}
    </div>
    </section>
  </div>;
}
