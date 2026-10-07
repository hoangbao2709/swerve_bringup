import { Component, memo, useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState, type ErrorInfo, type ReactNode } from "react";
import { apiFetch, clearEmergencyStop, emergencyStop } from "../../services/api";
import { WS_URL, wsManualCommand, wsSetRobotMode, wsSend, type ManualAction } from "../../services/ws";
import { MANUAL_COMMAND_REFRESH_MS, nextManualCommand, type ActiveManualCommand } from "../../services/manualCommand";
import { useStore } from "../../state/store";
import type { RobotDetailError, RobotDetailGoal, RobotDetailPathPreview, RobotState, RobotSystemDiagnostics } from "../../schema/twin_state";
import { ActiveNavigationMap2DView, LocalRobotSection } from "./LocalRobotSections";
import { occupancyRasters } from "./occupancyRaster";
import { detailPerformance } from "./detailPerformance";
import { displayedNavigationMapIdentity as getDisplayedNavigationMapIdentity, mapPointPreviewPayload, type MapPointNavigationTarget } from "./navigationMapIdentity";
import { evaluatePreviewApproval } from "./navigationPreviewApproval";

type HostStatus = { system?: { cpu_load_1m?: number | null; memory?: { used_percent?: number | null } } };
type LocalTab = "CONTROL" | "MAPPING" | "LOCALIZATION" | "VDA5050" | "DIAGNOSTICS";
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
  const goal = useStore((state) => state.robotDetail[robotId]?.goal ?? null);
  const mappingScan = useStore((state) => state.robotDetail[robotId]?.scan ?? null);
  const mappingSessionId = useStore((state) => state.robotDetail[robotId]?.mappingSessionId ?? null);
  const navigationStatus = useStore((state) => state.robotDetail[robotId]?.navigationStatus ?? null);
  const viewStatus = useStore((state) => state.robotDetail[robotId]?.viewStatus ?? null);
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
  const [activeTab, setActiveTab] = useState<LocalTab>("CONTROL");
  const [goalPreview, setGoalPreview] = useState<MapPointNavigationTarget | null>(null);
  const [pathRequestState, setPathRequestState] = useState<"IDLE" | "PLANNING">("IDLE");
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
  const activeMapStatus = useLiveSlamMap ? activeMappingSnapshot ? "SLAM · LIVE · LOCAL_ONLY" : "WAITING FOR SLAM MAP"
    : activeLocalMapId
      ? localMapSyncStatus ?? (activeMap2dSnapshot ? "LOCAL_ONLY" : "WAITING FOR LOCAL MAP")
      : activeMap2dSnapshot ? "ACTIVE MAP" : "WAITING FOR ACTIVE MAP";
  const activeMapReady = Boolean(activeMap2dSnapshot && activeMapId && activeMapRevision
    && activeMap2dSnapshot.frame_id === "map"
    && ["SLAM_TOOLBOX", "LOCAL_MAP", "NAV2_MAP"].includes(activeMap2dSnapshot.map_source ?? ""));
  const navigationUiAvailable = Boolean(runtimeCapabilities?.goal_available
    && controlMode === "AUTONOMOUS" && controlOnline);
  const statePose = robot?.active_map_pose ?? null;
  const previewApproval = evaluatePreviewApproval({
    pathPreview, requestId: latestPathRequest.current, target: goalPreview,
    activeMapId, activeMapRevision, now: clockNow,
  });
  const approvedPreview: RobotDetailPathPreview | null = previewApproval.approvedPreview;

  const activeNavigationMapKey = JSON.stringify([robotId, activeMapId, activeMapRevision, activeMapReady]);
  const previousActiveNavigationMapKey = useRef(activeNavigationMapKey);

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
    setRobotDetail(robotId, { pathPreview: null });
  }, [activeNavigationMapKey, robotId, setRobotDetail]);

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

  const detailView = "LIDAR_2D" as const;
  const displayedNavigationMapIdentity = getDisplayedNavigationMapIdentity({
    active_map_id: activeMapId, active_map_revision: activeMapRevision,
    map_snapshot: activeMap2dSnapshot,
  });
  const displayedMapPointPickIdentity = activeMapReady
    && navigationUiAvailable && controlMode === "AUTONOMOUS" ? displayedNavigationMapIdentity : null;
  const viewFresh = Boolean(activeMap2dSnapshot)
    || (viewStatus?.requested_view === detailView && viewStatus.state === "FRESH");
  useEffect(() => {
    if (!robotBridgeOnline || websocketState !== "CONNECTED") return;
    const request_id = `${robotId}-${Date.now()}-${Math.random().toString(36).slice(2, 8)}`;
    setRobotDetail(robotId, { viewStatus: { robot_id: robotId, requested_view: detailView, request_id, state: "REQUESTED" } });
    wsSend({ type: "ROBOT_DETAIL_VIEW", robot_id: robotId, view: detailView, request_id, delivery_ack: true });
  }, [detailView, robotBridgeOnline, robotId, websocketState, setRobotDetail]);
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
    })) {
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
    const sourceMatches = target.source_type === "ACTIVE_MAP_POINT"
      && target.source_map_id === activeMapId
      && target.source_map_revision === activeMapRevision;
    if (target.frame_id !== "map" || !sourceMatches || !activeMapReady) {
      setError("Map point selection belongs to a map identity or revision that is no longer current");
      return;
    }
    wsSend({ type: "PATH_PREVIEW_INVALIDATE", robot_id: robotId });
    setGoalPreview(target);
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
    const sent = pointPayload ? wsSend({ ...common, ...pointPayload }) : false;
    if (!sent) {
      latestPathRequest.current = "";
      setPathRequestState("IDLE");
      setError("Path preview request was not sent because the WebSocket is disconnected");
      return;
    }
    setGoalPreview(target);
  }, [activeMap2dSnapshot?.map_content_revision, activeMapId, activeMapReady, activeMapRevision, activeMapStatus, controlMode, controlOnline, navigationUiAvailable, robotId, setRobotDetail]);

  useEffect(() => {
    if (pathPreview?.request_id && pathPreview.request_id === latestPathRequest.current) setPathRequestState("IDLE");
  }, [pathPreview]);

  const cancelPathPreview = () => {
    wsSend({ type: "PATH_PREVIEW_INVALIDATE", robot_id: robotId });
    latestPathRequest.current = "";
    setGoalPreview(null);
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
    if (next) select(next);
  };

  const moveButtonEvents = useCallback((action: ManualAction) => ({
    onClick: () => toggleManual(action),
  }), [toggleManual]);

  return (
    <div className="robot-detail-shell">
      <header className="robot-detail-header">
        <div className="robot-detail-identity">
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
          <SystemInputsPanel robotId={robotId} robot={robot} controlMode={controlMode} runtimeMode={runtimeMode} goal={goalPreview ?? goal} />
          <StatePanel pose={statePose} robot={robot} localization={localization} diagnostics={detailDiagnostics ?? diagnostics} controller={controller} navigationStatus={navigationStatus} />
        </aside>

        <section className="robot-detail-map-panel">
          <div className="robot-map-source-bar">
            <span className="robot-map-view-label">MAP VIEW 2D · ACTIVE MAP / SLAM · FRAME map</span>
            <span className="map-sync-warning" data-view-state={viewFresh ? "FRESH" : viewStatus?.state ?? "REQUESTED"}>{viewFresh ? "LIVE" : "WAITING FOR FRESH FRAME"}</span>
            <span className={`map-sync-warning ${activeMapReady ? "map-sync-warning-ready" : ""}`}>{activeMapId ? `${activeMapId} · r${activeMapRevision ?? "—"} · ${activeMapStatus}` : `MAP ${activeMapStatus}`}</span>
          </div>
          <div className="robot-detail-target-toolbar">
            <span>POINT NAVIGATION · SELECT A FREE POINT ON THE ACTIVE MAP</span>
            <span>{goalPreview ? `TARGET ${safeNumber(goalPreview.x, 2)} / ${safeNumber(goalPreview.y, 2)}` : "NO POINT SELECTED"}</span>
          </div>
          <div className="robot-detail-view-stack" key={robotId}>
            <div className="robot-detail-view-layer is-active" data-view="LIDAR_2D">
              <ActiveNavigationMap2DView map={activeMap2dSnapshot} robot={robot} scan={mappingScan}
                target={isMapPointTarget(goalPreview) ? goalPreview : null}
                navigationPath={approvedPreview?.active_path ?? approvedPreview?.path ?? []}
                canPick={Boolean(displayedMapPointPickIdentity)} onPick={selectMapPoint} />
            </div>
          </div>
          <div className="robot-detail-goal-toolbar">
            <span>{goalPreview
              ? `TARGET ${safeNumber(goalPreview.x, 2)} / ${safeNumber(goalPreview.y, 2)} / ${safeNumber(goalPreview.yaw, 2)} rad · ${pathRequestState === "PLANNING" ? "PLANNING" : approvedPreview?.status ?? "NO PREVIEW"}${approvedPreview?.status === "VALID" ? ` · ${safeNumber(approvedPreview.path_length_m, 2, " m")}` : approvedPreview?.reason ? ` · ${approvedPreview.reason}` : ""}`
              : !navigationUiAvailable
                ? `${runtimeCapabilities?.goal_blocker_code ?? "NAVIGATION_UNAVAILABLE"} · ${runtimeCapabilities?.goal_blocker_reason ?? "runtime has not confirmed Nav2 readiness"}`
                : "Select a point, adjust yaw if needed, then preview the Nav2 path"}</span>
            <button type="button" disabled={!isMapPointTarget(goalPreview) || pathRequestState === "PLANNING"} onClick={() => adjustSelectedMapPointYaw(-Math.PI / 12)}>YAW −</button>
            <button type="button" disabled={!isMapPointTarget(goalPreview) || pathRequestState === "PLANNING"} onClick={() => adjustSelectedMapPointYaw(Math.PI / 12)}>YAW +</button>
            <button type="button" disabled={!navigationUiAvailable || !goalPreview || pathRequestState === "PLANNING"}
              data-preview-status={pathPreview?.status ?? "NONE"}
              data-preview-request-id={pathPreview?.request_id ?? ""}
              data-preview-approved={previewApproval.approved ? "true" : "false"}
              data-preview-reject-reason={previewApproval.rejectReason ?? undefined}
              data-preview-gates={JSON.stringify(previewApproval.gates)}
              onClick={() => goalPreview && requestPathPreview(goalPreview)}>PREVIEW PATH</button>
            <button type="button" disabled={!navigationUiAvailable || !goalPreview || !approvedPreview || !controlOnline || controlMode !== "AUTONOMOUS" || !activeMapReady} className="robot-console-primary" onClick={sendGoal}>SEND GOAL</button>
            <button type="button" disabled={!goalPreview} onClick={cancelPathPreview}>CANCEL</button>
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

function SystemInputsPanel({ robotId, robot, controlMode, runtimeMode, goal }: { robotId: string; robot?: RobotState; controlMode: string; runtimeMode: string; goal: RobotDetailGoal | MapPointNavigationTarget | null }) {
  return <Panel title="SYSTEM INPUTS">
    <Metric label="Robot" value={robotId} mono />
    <Metric label="Control mode" value={controlMode} status />
    <Metric label="Runtime mode" value={runtimeMode} />
    <Metric label="Navigation mode" value={safeText(robot?.navigation_state)} />
    <Metric label="Target goal" value={goal ? `${safeNumber(goal.x, 2)} / ${safeNumber(goal.y, 2)} / ${safeNumber(goal.yaw, 2)}` : "N/A"} mono />
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
