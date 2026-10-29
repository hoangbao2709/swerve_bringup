import { lazy, memo, Suspense, useCallback, useEffect, useId, useLayoutEffect, useRef, useState, type MouseEvent as ReactMouseEvent, type PointerEvent as ReactPointerEvent } from "react";
import { WS_URL, wsAcquireManual, wsManualCommand, wsSend, type ManualAction } from "../../services/ws";
import { MANUAL_COMMAND_REFRESH_MS, type ActiveManualCommand } from "../../services/manualCommand";
import { useStore } from "../../state/store";
import type { RobotDetailError, RobotDetailPathPreview, RobotSystemDiagnostics } from "../../schema/twin_state";
import { ActiveNavigationMap2DView, ControlMapViewSwitcher, type ActiveMapLayers } from "../../components/shared/maps/LocalMapViews";
import { occupancyRasters } from "../../components/shared/maps/occupancyRaster";
import { detailPerformance } from "../../services/detailPerformance";
import { displayedNavigationMapIdentity as getDisplayedNavigationMapIdentity, mapPointPreviewPayload, mapPointTarget, type MapPointNavigationTarget } from "../../components/shared/maps/navigationMapIdentity";
import { evaluatePreviewApproval } from "./navigationPreviewApproval";
import { resolveDemoRoute } from "./demoRoute";
import { IS_SHOWCASE, IS_STATIC_UI } from "../../config";
import { displayRuntimeRows, displayRuntimeValue } from "../../mock/staticPresentation";
import { useLocalAppRuntime } from "../../app/AppLayout";
import { Alert, Badge, Button } from "../../components/shared/ui/HmiPrimitives";

const RobotLidar3DView = lazy(() => import("./components/RobotLidarViews").then((module) => ({ default: module.RobotLidar3DView })));

const MANUAL_ACTIONS: Array<{ action: ManualAction; label: string; title: string }> = [
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
const manualGridPlacement: Record<ManualAction, string> = {
  FORWARD_LEFT: "col-start-1 row-start-1", FORWARD: "col-start-2 row-start-1", FORWARD_RIGHT: "col-start-3 row-start-1",
  LEFT: "col-start-1 row-start-2", STOP: "col-start-2 row-start-2", RIGHT: "col-start-3 row-start-2",
  BACKWARD_LEFT: "col-start-1 row-start-3", BACKWARD: "col-start-2 row-start-3", BACKWARD_RIGHT: "col-start-3 row-start-3",
  ROTATE_LEFT: "", ROTATE_RIGHT: "",
};
const dashboardToneClass: Record<string, string> = {
  "is-good": "text-ptagv-success", "is-warning": "text-ptagv-warning", "is-fault": "text-ptagv-error", "is-neutral": "text-ptagv-muted",
};
const EMPTY_ERRORS: RobotDetailError[] = [];
type ManualButtonHandlers = {
  onPointerDown: (event: ReactPointerEvent<HTMLButtonElement>) => void;
  onPointerUp: () => void;
  onPointerCancel: () => void;
  onLostPointerCapture: () => void;
  onPointerLeave: () => void;
  onClick: () => void;
  onContextMenu: (event: ReactMouseEvent<HTMLButtonElement>) => void;
};

function isMapPointTarget(target: MapPointNavigationTarget | null): target is MapPointNavigationTarget {
  return Boolean(target && target.frame_id === "map" && target.source_type === "ACTIVE_MAP_POINT"
    && target.map_id && target.map_revision && target.source_map_id && target.source_map_revision);
}

function safeNumber(value: unknown, digits = 2, suffix = ""): string {
  const number = Number(value);
  return Number.isFinite(number) ? `${number.toFixed(digits)}${suffix}` : "N/A";
}

function safeText(value: unknown, fallback = "N/A"): string {
  return value === null || value === undefined || value === "" ? fallback : String(value);
}

function browserSupportsWebGl(): boolean {
  if (typeof document === "undefined"
      || (typeof WebGLRenderingContext === "undefined" && typeof WebGL2RenderingContext === "undefined")) return false;
  try {
    const context = document.createElement("canvas").getContext("webgl2")
      ?? document.createElement("canvas").getContext("webgl");
    context?.getExtension("WEBGL_lose_context")?.loseContext();
    return Boolean(context);
  } catch {
    return false;
  }
}

function statusClass(value: unknown): string {
  const text = String(value ?? "").toUpperCase();
  if (["OK", "ONLINE", "CONNECTED", "ACTIVE", "LOCALIZED", "AVAILABLE", "SUCCEEDED", "READY", "RUNNING"].includes(text)) return "ok";
  if (["WARNING", "DEGRADED", "WAITING", "STALE", "INITIALIZING", "PENDING", "PAUSED"].includes(text)) return "warning";
  if (["ERROR", "OFFLINE", "DISCONNECTED", "LOST", "FAILED", "CRITICAL"].includes(text)) return "error";
  return "neutral";
}

export function ControlPage() {
  const { robotId } = useLocalAppRuntime();
  return robotId ? <ControlPageBody robotId={robotId} /> : null;
}
function ControlPageBody({ robotId }: { robotId: string }) {
  const { controlMode, controlOnline, robotOnline, estopState, setError, registerControlPageActions } = useLocalAppRuntime();

  const robot = useStore((state) => state.twin?.robots?.[robotId]);
  const currentMission = useStore((state) => {
    const taskId = state.twin?.robots?.[robotId]?.current_task_id;
    return taskId ? state.twin?.tasks?.[taskId] ?? null : null;
  });
  const rawDiagnostics = useStore((state) => state.rosDiagnostics);
  const diagnostics = rawDiagnostics as RobotSystemDiagnostics | null;
  const rawLocalization = useStore((state) => state.localization);
  const runtimeMode = useStore((state) => state.runtimeMode);
  const runtimeState = useStore((state) => state.runtimeState);
  const runtimeCapabilities = useStore((state) => state.robotCapabilities[robotId] ?? null);
  const slam2dMap = useStore((state) => state.robotDetail[robotId]?.slam2dMap ?? null);
  const runtimeMapSnapshot = useStore((state) => state.robotDetail[robotId]?.runtimeMapSnapshot ?? null);
  const lidar3dAccumulatedCloud = useStore((state) => state.robotDetail[robotId]?.slam3dAccumulatedCloud ?? null);
  const detailDiagnostics = useStore((state) => state.robotDetail[robotId]?.diagnostics ?? null);
  const modeTransitionState = useStore((state) => state.robotDetail[robotId]?.modeTransitionState);
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
  const pathPreview = useStore((state) => state.robotDetail[robotId]?.pathPreview ?? null);
  const activeLocalMapId = useStore((state) => state.robotDetail[robotId]?.activeLocalMapId ?? null);
  const activeLocalMapRevision = useStore((state) => state.robotDetail[robotId]?.activeLocalMapRevision ?? null);
  const localMapSyncStatus = useStore((state) => state.robotDetail[robotId]?.localMapSyncStatus ?? null);
  const slamRuntimeActive = runtimeState === "MAPPING" || runtimeState === "UNIFIED";
  const useLiveSlamMap = slamRuntimeActive && !activeLocalMapId;
  const setRobotDetail = useStore((state) => state.setRobotDetail);
  const [activeManualCommand, setActiveManualCommand] = useState<ActiveManualCommand | null>(null);
  const [goalPreview, setGoalPreview] = useState<MapPointNavigationTarget | null>(null);
  const [pointYawPending, setPointYawPending] = useState(false);
  const [pointSelectionResetVersion, setPointSelectionResetVersion] = useState(0);
  const [mapLayers, setMapLayers] = useState<ActiveMapLayers>({ robot: true, scan: true, path: true, trajectory: false, grid: false });
  const [mapView, setMapView] = useState<"2D" | "3D">("2D");
  const [webglSupported, setWebglSupported] = useState(false);
  const [systemInputsExpanded, setSystemInputsExpanded] = useState(true);
  const [statePanelExpanded, setStatePanelExpanded] = useState(true);
  const [operationPanelExpanded, setOperationPanelExpanded] = useState(true);
  const [manualPanelExpanded, setManualPanelExpanded] = useState(true);
  const [errorLogOpen, setErrorLogOpen] = useState(false);
  const closeErrorLog = useCallback(() => setErrorLogOpen(false), []);
  const panelId = useId();
  const [pathRequestState, setPathRequestState] = useState<"IDLE" | "PLANNING" | "WAITING_FOR_NAV2">("IDLE");
  const latestPathRequest = useRef("");
  const [clockNow, setClockNow] = useState(() => Date.now());
  const manualTimer = useRef<number | null>(null);
  const activeManualCommandRef = useRef<ActiveManualCommand | null>(null);
  const activeManualRobotId = useRef<string | null>(null);
  const manualLeaseId = useRef<string | null>(null);
  const manualWorker = useRef<Worker | null>(null);
  const manualKeyboardHandlers = useRef<{
    activate: (action: ManualAction) => void;
    stop: (force?: boolean) => void;
  }>({ activate: () => undefined, stop: () => undefined });

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
    setGoalPreview(null);
    setPointYawPending(false);
    latestPathRequest.current = "";
    setPathRequestState("IDLE");
    setRobotDetail(robotId, { pathPreview: null });
  }, [robotId, setRobotDetail]);

  useEffect(() => {
    if (previousActiveNavigationMapKey.current === activeNavigationMapKey) return;
    previousActiveNavigationMapKey.current = activeNavigationMapKey;
    const hasUnacceptedPreview = Boolean(latestPathRequest.current || goalPreview || pathPreview
      || pointYawPending || pathRequestState !== "IDLE");
    if (hasUnacceptedPreview) wsSend({ type: "PATH_PREVIEW_INVALIDATE", robot_id: robotId });
    latestPathRequest.current = "";
    setPathRequestState("IDLE");
    setGoalPreview(null);
    setPointYawPending(false);
    setRobotDetail(robotId, { pathPreview: null });
  }, [activeNavigationMapKey, goalPreview, pathPreview, pointYawPending, pathRequestState, robotId, setRobotDetail]);

  useEffect(() => {
    if (runtimeState === "MAPPING") {
      setGoalPreview(null);
      setPointYawPending(false);
    }
  }, [runtimeState]);

  useEffect(() => {
    if (controlMode === "MANUAL") setManualPanelExpanded(true);
    else setOperationPanelExpanded(true);
  }, [controlMode]);

  useEffect(() => {
    const timer = window.setInterval(() => setClockNow(Date.now()), 1000);
    return () => window.clearInterval(timer);
  }, []);

  const detailView = mapView === "3D" ? "LIDAR_3D" as const : "LIDAR_2D" as const;
  useEffect(() => setWebglSupported(browserSupportsWebGl()), []);
  const displayedMapPointPickIdentity = activeMapReady
    && (controlOnline || IS_STATIC_UI || IS_SHOWCASE) && controlMode === "AUTONOMOUS" ? displayedNavigationMapIdentity : null;
  useLayoutEffect(() => {
    const cached = occupancyRasters.peek(activeMap2dSnapshot);
    if (!cached) return;
    const paint = requestAnimationFrame(() => detailPerformance("view_render", { view: detailView, robot_id: robotId, useful: true, cached: true }));
    return () => cancelAnimationFrame(paint);
    // This measures a cached canvas becoming visible, not a newly received frame.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [activeMap2dSnapshot, detailView, robotId]);

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
      else wsManualCommand(commandRobotId, "STOP", manualLeaseId.current ?? undefined);
    }
    manualLeaseId.current = null;
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
    if (IS_STATIC_UI || !controlOnline || typeof Worker === "undefined") return;
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

  const estopLabel = estopState;
  useEffect(() => {
    if (estopState !== "CLEAR") stopManual();
  }, [estopState, stopManual]);

  const holdManual = useCallback((action: Exclude<ManualAction, "STOP">) => {
    if (activeManualCommandRef.current === action) return;
    if (controlMode !== "MANUAL" || modeTransitionState === "REQUESTED" || modeTransitionState === "FAILED") { setError("Wait for applied MANUAL mode before driving"); return; }
    if (estopState !== "CLEAR") { setError(`Manual control requires fresh E-STOP CLEAR telemetry (current: ${estopState})`); return; }
    if (!controlOnline) { setError("Manual control is disabled while ROS bridge is disconnected"); return; }
    if (manualWorker.current) {
      manualWorker.current.postMessage({ type: "HOLD", robot_id: robotId, action });
      setManualCommand(action);
      return;
    }
    if (!manualLeaseId.current) {
      const leaseId = typeof crypto !== "undefined" && typeof crypto.randomUUID === "function"
        ? crypto.randomUUID()
        : `${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}-${Math.random().toString(36).slice(2)}`;
      if (!wsAcquireManual(robotId, leaseId)) { setError("Manual control channel is disconnected"); return; }
      manualLeaseId.current = leaseId;
    }
    if (manualTimer.current !== null) window.clearInterval(manualTimer.current);
    if (!wsManualCommand(robotId, action, manualLeaseId.current)) {
      setError("Manual command was not sent");
      stopManual(true);
      return;
    }
    setManualCommand(action);
    const leaseId = manualLeaseId.current;
    manualTimer.current = window.setInterval(() => {
      if (!wsManualCommand(robotId, action, leaseId ?? undefined)) stopManual(true);
    }, MANUAL_COMMAND_REFRESH_MS);
  }, [controlMode, controlOnline, estopState, modeTransitionState, robotId, setManualCommand, stopManual]);

  manualKeyboardHandlers.current = { activate: (action) => { if (action !== "STOP") holdManual(action); }, stop: stopManual };

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
    const onKeyUp = (event: KeyboardEvent) => {
      if (!keyActions[event.key]) return;
      event.preventDefault();
      manualKeyboardHandlers.current.stop();
    };
    const onBlur = () => manualKeyboardHandlers.current.stop();
    const onPointerUp = (event: PointerEvent) => {
      if (event.target instanceof Element && event.target.closest(".manual-key-stop")) return;
      manualKeyboardHandlers.current.stop();
    };
    const onVisibilityChange = () => {
      if (document.visibilityState !== "visible") manualKeyboardHandlers.current.stop();
    };
    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    window.addEventListener("blur", onBlur);
    window.addEventListener("pointerup", onPointerUp);
    window.addEventListener("pointercancel", onBlur);
    window.addEventListener("pagehide", onBlur);
    document.addEventListener("visibilitychange", onVisibilityChange);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
      window.removeEventListener("blur", onBlur);
      window.removeEventListener("pointerup", onPointerUp);
      window.removeEventListener("pointercancel", onBlur);
      window.removeEventListener("pagehide", onBlur);
      document.removeEventListener("visibilitychange", onVisibilityChange);
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
    if (IS_STATIC_UI) {
      latestPathRequest.current = "";
      setPathRequestState("IDLE");
      setGoalPreview(target);
      setRobotDetail(robotId, { pathPreview: null });
      return;
    }
    if (IS_SHOWCASE) {
      const requestId = `sample-${Date.now()}`;
      const start = robot?.active_map_pose;
      const startPoint: [number, number] = [start?.x ?? 30, start?.y ?? 17];
      const goalPoint: [number, number] = [target.x, target.y];
      const distance = Math.hypot(goalPoint[0] - startPoint[0], goalPoint[1] - startPoint[1]);
      latestPathRequest.current = requestId;
      setPathRequestState("IDLE");
      setGoalPreview(target);
      setRobotDetail(robotId, { pathPreview: {
        robot_id: robotId,
        request_id: requestId,
        status: "VALID",
        reason: "Sample visualization only; no Nav2 planning or robot command was performed.",
        frame_id: "map",
        path: [startPoint, goalPoint],
        path_length_m: distance,
        goal: { x: target.x, y: target.y, yaw: target.yaw, frame_id: "map" },
        source_type: "ACTIVE_MAP_POINT",
        source_map_id: activeMapId,
        source_map_revision: activeMapRevision,
        active_map_id: activeMapId,
        active_map_revision: activeMapRevision,
        map_content_revision: activeMap2dSnapshot?.map_content_revision ?? null,
        timestamp: new Date().toISOString(),
      } });
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
  }, [activeMap2dSnapshot?.map_content_revision, activeMapId, activeMapReady, activeMapRevision, activeMapStatus, controlMode, controlOnline, nav2Ready, robot, robotId, runtimeCapabilities?.goal_available, setRobotDetail]);

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

  const cancelPathPreview = useCallback(() => {
    wsSend({ type: "PATH_PREVIEW_INVALIDATE", robot_id: robotId });
    latestPathRequest.current = "";
    setGoalPreview(null);
    setPointYawPending(false);
    setPathRequestState("IDLE");
    setRobotDetail(robotId, { pathPreview: null });
    setPointSelectionResetVersion((version) => version + 1);
  }, [robotId, setRobotDetail]);

  const clearNavigationPreviewForModeChange = useCallback(() => {
    const hasUnacceptedPreview = Boolean(latestPathRequest.current || goalPreview || pathPreview
      || pointYawPending || pathRequestState !== "IDLE");
    if (hasUnacceptedPreview) wsSend({ type: "PATH_PREVIEW_INVALIDATE", robot_id: robotId });
    latestPathRequest.current = "";
    setGoalPreview(null);
    setPointYawPending(false);
    setPathRequestState("IDLE");
    setRobotDetail(robotId, { pathPreview: null });
  }, [goalPreview, pathPreview, pointYawPending, pathRequestState, robotId, setRobotDetail]);
  useEffect(() => registerControlPageActions({ stopManual, clearNavigationPreview: clearNavigationPreviewForModeChange }),
    [clearNavigationPreviewForModeChange, registerControlPageActions, stopManual]);
  const routeCleanupRef = useRef({ stopManual, cancelPathPreview, hasPendingPreview: false });
  routeCleanupRef.current = { stopManual, cancelPathPreview,
    hasPendingPreview: Boolean(goalPreview || pathPreview || pointYawPending || pathRequestState !== "IDLE") };
  useEffect(() => () => {
    const cleanup = routeCleanupRef.current;
    cleanup.stopManual();
    if (cleanup.hasPendingPreview) cleanup.cancelPathPreview();
  }, [robotId]);
  const navCommand = (type: "NAV_CANCEL" | "NAV_PAUSE" | "NAV_RESUME") => {
    if (!controlOnline) { setError("Navigation control requires an online ROS bridge"); return; }
    if (!wsSend({ type, robot_id: robotId })) setError("Navigation command was not sent");
  };

  const moveButtonEvents = useCallback((action: ManualAction) => ({
    onPointerDown: (event: ReactPointerEvent<HTMLButtonElement>) => {
      if (action === "STOP") { event.preventDefault(); return; }
      if (event.button !== 0) return;
      event.preventDefault();
      holdManual(action);
    },
    onPointerUp: () => { if (action !== "STOP") stopManual(); },
    onPointerCancel: () => { if (action !== "STOP") stopManual(); },
    onLostPointerCapture: () => { if (action !== "STOP") stopManual(); },
    onPointerLeave: () => { if (action !== "STOP") stopManual(); },
    onClick: () => { if (action === "STOP") stopManual(true); },
    onContextMenu: (event: ReactMouseEvent<HTMLButtonElement>) => event.preventDefault(),
  }), [holdManual, stopManual]);

  const slamLive = diagnostics?.mapping?.slam_state === "ACTIVE" && diagnostics.mapping.map_live;
  const rawNavigationState = String(navigationStatus ?? robot?.navigation_state ?? "").toUpperCase();
  const navigationLabel = ["", "IDLE"].includes(rawNavigationState)
    ? nav2Ready ? "READY" : "UNAVAILABLE"
    : ["ACTIVE", "NAVIGATING"].includes(rawNavigationState) ? "RUNNING" : rawNavigationState;
  const selectedPoint = goalPreview;
  const nav2BlockerReason = runtimeCapabilities?.goal_blocker_reason
    ?? diagnostics?.nav2_lifecycle_blocker_reason ?? "";
  const dashboardStateRows = displayRuntimeRows(IS_STATIC_UI, [
    ["CURRENT STATE", runtimeState], ["CONTROL MODE", controlMode],
    ["MAPPING", diagnostics?.mapping?.slam_state ?? "UNKNOWN"],
    ["LOCALIZATION", activeLocalMapId ? runtimeCapabilities?.localization_ready ? "LOCALIZED" : "INITIAL POSE REQUIRED" : safeText(localization, "UNKNOWN")],
    ["NAVIGATION", navigationLabel], ["E-STOP", estopLabel],
  ]);
  const systemInputRows = displayRuntimeRows(IS_STATIC_UI, [
    ["ACTIVE MAP", activeMapId ?? "WAITING"],
    ["MAP SOURCE", activeMap2dSnapshot?.map_source?.replace(/_/g, " ") ?? "WAITING"],
    ["LIDAR", diagnostics?.mapping?.scan_live ? "LIVE" : diagnostics?.lidar ? "READY" : "UNAVAILABLE"],
    ["ODOMETRY", diagnostics?.mapping?.odom_live ? "RECEIVING" : "WAITING"],
    ["ROS BRIDGE", controlOnline ? "ONLINE" : "OFFLINE"],
    ["SLAM", slamLive ? "LIVE" : diagnostics?.slam ? "READY" : "INACTIVE"],
    ["NAV2", nav2Ready ? "READY" : nav2BlockerReason ? "BLOCKED" : diagnostics?.nav2 ? "STARTING" : "INACTIVE"],
  ]);

  return (
    <main className="robot-detail-main hmi-control-main relative flex min-h-0 min-w-0 flex-1 overflow-hidden bg-ptagv-page p-hmi-2">
      <section className="robot-detail-map-panel relative min-h-0 min-w-0 flex-1 overflow-hidden border border-ptagv-border bg-slate-500">
          <div className="robot-detail-view-stack absolute inset-0 min-h-0 w-full" key={robotId}>
            {mapView === "2D" ? <div className="robot-detail-view-layer is-active absolute inset-0 flex visible pointer-events-auto" data-view="LIDAR_2D">
              <ActiveNavigationMap2DView map={activeMap2dSnapshot} robot={IS_STATIC_UI ? undefined : robot}
                fixtureRobotPose={IS_SHOWCASE ? robot?.active_map_pose ?? undefined : undefined} scan={IS_STATIC_UI ? null : mappingScan}
                compactWorkspace controlViewMode={IS_STATIC_UI ? undefined : "2D"}
                onControlViewModeChange={IS_STATIC_UI ? undefined : setMapView}
                webglSupported={webglSupported} threeDimensionalAvailable={IS_SHOWCASE || Boolean(lidar3dAccumulatedCloud)}
                target={isMapPointTarget(goalPreview) ? goalPreview : activeGoalTarget}
                navigationPath={displayRoute?.points ?? []} navigationPathKind={displayRoute?.kind ?? null}
                layers={mapLayers} onLayerToggle={(layer) => setMapLayers((current) => ({ ...current, [layer]: !current[layer] }))}
                errorLogCount={controlErrors.length} errorLogTone={errorLogTone} onOpenErrorLog={() => setErrorLogOpen(true)}
                canPick={Boolean(displayedMapPointPickIdentity)} onPick={selectMapPoint} onConfirmPoint={confirmMapPoint}
                pointSelectionResetVersion={pointSelectionResetVersion}
                onCancelPointSelection={cancelPathPreview}
                onInteractionModeChange={(mode) => { if (mode === "PAN" && pointYawPending) cancelPathPreview(); }} />
            </div> : <div className="absolute inset-0 flex min-h-0 min-w-0" data-view="LIDAR_3D">
              <Suspense fallback={<div className="grid flex-1 place-items-center bg-slate-950 text-xs font-semibold text-slate-200">LOADING 3D VIEW</div>}>
                <RobotLidar3DView frame={IS_STATIC_UI ? null : lidar3dAccumulatedCloud} robot={IS_STATIC_UI ? undefined : robot}
                  slamMap={activeMap2dSnapshot} fixtureRobotPose={IS_SHOWCASE ? robot?.active_map_pose ?? undefined : undefined}
                  sample={IS_SHOWCASE} active={mapView === "3D"} />
              </Suspense>
            </div>}
            {mapView === "3D" && <div className="pointer-events-none absolute inset-0 z-30">
              <ControlMapViewSwitcher view="3D" onChange={setMapView} webglSupported={webglSupported}
                available={IS_SHOWCASE || Boolean(lidar3dAccumulatedCloud)} />
            </div>}
          </div>
          <div className="hmi-dashboard-panels pointer-events-none absolute left-hmi-2 top-hmi-2 z-10 flex max-h-[calc(100%-9.25rem)] w-44 flex-col gap-hmi-2 overflow-auto [scrollbar-width:thin] lg:w-hmi-status-panel 2xl:w-56" aria-label="Map system status">
            <DashboardStatusPanel title="SYSTEM INPUTS" rows={systemInputRows} expanded={systemInputsExpanded} controls={`${panelId}-inputs`} onToggle={() => setSystemInputsExpanded((value) => !value)} />
            <DashboardStatusPanel title="STATE" rows={dashboardStateRows} expanded={statePanelExpanded} controls={`${panelId}-state`} onToggle={() => setStatePanelExpanded((value) => !value)} />
          </div>
        <div className="hmi-right-floating-group pointer-events-none absolute right-hmi-2 top-hmi-2 z-10 flex max-h-[calc(100%-1rem)] w-hmi-operation-panel flex-col gap-hmi-2 overflow-auto [scrollbar-width:thin]" aria-label="Robot control cards" data-testid="control-operation-panels" data-control-mode={controlMode}>
        {controlMode === "AUTONOMOUS" && <>
        <aside className="hmi-operation-panel hmi-floating-panel pointer-events-auto flex w-full shrink-0 flex-col overflow-hidden border border-ptagv-border bg-white text-ptagv-text" aria-label="Robot operation panel" data-expanded={operationPanelExpanded}>
          <header className="flex min-h-8 shrink-0 items-center justify-between gap-hmi-1 border-b border-ptagv-border-soft bg-ptagv-surface-muted px-hmi-2 py-1">
            <span className="text-hmi-kicker font-bold tracking-wide text-ptagv-muted">MISSION</span>
            {!IS_STATIC_UI && <Badge tone={robotOnline ? "success" : "error"}>{robotOnline ? "ONLINE" : "OFFLINE"}</Badge>}
            <PanelToggle title="Mission" expanded={operationPanelExpanded} controls={`${panelId}-operation`} onToggle={() => setOperationPanelExpanded((value) => !value)} />
          </header>
          <div ref={(element) => setPanelContentInert(element, !operationPanelExpanded)} className="hmi-floating-content min-h-0 flex-1" id={`${panelId}-operation`} data-expanded={operationPanelExpanded} aria-hidden={!operationPanelExpanded}>
          <div className="hmi-floating-content-inner min-h-0 overflow-auto">
          <div className="hmi-operation-status grid grid-cols-3 gap-x-hmi-2 border-b border-ptagv-border-soft px-hmi-2 py-hmi-1">
            <div className="grid min-h-8 min-w-0 content-center gap-0.5 border-b border-ptagv-border-soft"><span className="text-[0.55rem] font-bold text-ptagv-muted">NAV</span>{IS_STATIC_UI ? <span className="text-xs text-ptagv-muted">{displayRuntimeValue(true, navigationLabel)}</span> : <Badge className="w-fit" tone={navigationLabel === "READY" || navigationLabel === "SUCCEEDED" ? "success" : navigationLabel === "RUNNING" ? "info" : statusClass(navigationLabel) === "error" ? "error" : "warning"}>{navigationLabel}</Badge>}</div>
            <div></div>
            <div className="grid min-h-8 min-w-0 content-center gap-0.5 border-b border-ptagv-border-soft"><span className="text-[0.55rem] font-bold text-ptagv-muted">REMAINING</span><b className="text-xs text-ptagv-text">{displayRuntimeValue(IS_STATIC_UI, safeNumber(remainingDistanceM, 2, " m"))}</b></div>
            {!IS_STATIC_UI && currentMission && <div className="col-span-full grid min-h-8 min-w-0 content-center gap-0.5 border-b border-ptagv-border-soft"><span className="text-[0.55rem] font-bold text-ptagv-muted">MISSION</span><b className="break-words text-xs text-ptagv-text">{`${currentMission.id} · ${currentMission.status}`}</b></div>}
          </div>
          <section className="hmi-destination border-b border-ptagv-border-soft px-hmi-2 py-hmi-2">
            <h3 className="m-0 mb-hmi-2 text-xs font-bold text-ptagv-text">DESTINATION</h3>
            {selectedPoint ? <><p className="hmi-point-selected m-0 mb-hmi-1 text-hmi-kicker font-bold tracking-wide text-ptagv-warning" data-testid="selected-point-status">{pointYawPending ? "SELECT YAW" : "POINT SELECTED"}</p><div className="hmi-coordinate-grid grid grid-cols-[auto_minmax(0,1fr)_auto_minmax(0,1fr)_auto_minmax(0,1fr)] items-center gap-0.5">
              <span className="text-[0.55rem] font-bold text-ptagv-muted">X</span><b className="text-[0.6rem] font-mono text-ptagv-text">{safeNumber(selectedPoint.x, 3)} m</b>
              <span className="text-[0.55rem] font-bold text-ptagv-muted">Y</span><b className="text-[0.6rem] font-mono text-ptagv-text">{safeNumber(selectedPoint.y, 3)} m</b>
              <span className="text-[0.55rem] font-bold text-ptagv-muted">YAW</span><b className="text-[0.6rem] font-mono text-ptagv-text">{safeNumber(selectedPoint.yaw, 2)} rad</b>
            </div></> : <p className="m-0 text-xs text-ptagv-muted">NO TARGET</p>}
          </section>
          <div className="hmi-navigation-actions grid grid-cols-2 gap-hmi-1 px-hmi-2 py-hmi-1">
            <Button aria-label="SEND GOAL" disabled={!navigationUiAvailable || !goalPreview || !approvedPreview || !controlOnline || !activeMapReady} className="hmi-send-goal min-h-7" tone="primary" onClick={sendGoal}>SEND</Button>
            <Button className="hmi-cancel-point min-h-7" size="compact" disabled={!goalPreview} onClick={cancelPathPreview}>CANCEL</Button>
          </div>
          {!IS_STATIC_UI && !nav2Ready && nav2BlockerReason && <Alert className="hmi-nav-blocker mx-hmi-2 mb-hmi-2 grid gap-1" tone="warning" role="status"><b className="text-hmi-kicker">GOALS UNAVAILABLE</b>
            {(runtimeCapabilities?.goal_blocker_code ?? diagnostics?.nav2_lifecycle_blocker_code) && <code className="font-mono text-hmi-kicker">{runtimeCapabilities?.goal_blocker_code ?? diagnostics?.nav2_lifecycle_blocker_code}</code>}
            <span>{nav2BlockerReason}</span></Alert>}
          <div className="hmi-nav-management grid grid-cols-3 gap-1 px-hmi-2 pb-hmi-2" aria-label="Navigation actions">
            <Button size="compact" disabled={!controlOnline} onClick={() => navCommand("NAV_PAUSE")}>PAUSE</Button>
            <Button size="compact" disabled={!controlOnline} onClick={() => navCommand("NAV_RESUME")}>RESUME</Button>
            <Button size="compact" disabled={!controlOnline} onClick={() => navCommand("NAV_CANCEL")}>CANCEL NAV</Button>
          </div>
          <div className="hmi-preview-state mx-hmi-2 mb-hmi-2 border border-ptagv-border-soft bg-slate-50 px-hmi-2 py-hmi-1 text-[0.6rem] leading-snug text-ptagv-muted [overflow-wrap:anywhere]" role="status" data-testid="point-navigation-state">
            {!goalPreview ? "WAITING FOR DESTINATION" : pointYawPending ? "SELECT YAW" : IS_STATIC_UI ? "POINT SELECTED" : IS_SHOWCASE ? approvedPreview ? `SAMPLE ROUTE · ${safeNumber(approvedPreview.path_length_m, 2, " m")} · NOT SENT` : "LOCAL SAMPLE PATH PREVIEW" : pathRequestState === "PLANNING" ? "PLANNING" : pathRequestState === "WAITING_FOR_NAV2" ? "WAITING FOR NAV2" : approvedPreview?.status === "VALID" ? `PREVIEW VALID · ${safeNumber(approvedPreview.path_length_m, 2, " m")}` : approvedPreview?.reason ?? "POINT SELECTED"}
          </div>
          </div>
          </div>
        </aside>
        </>}
        {controlMode === "MANUAL" && <ManualBar controlMode={controlMode} controlOnline={controlOnline} activeManualCommand={activeManualCommand} moveButtonEvents={moveButtonEvents} expanded={manualPanelExpanded} controls={`${panelId}-manual`} onToggle={() => setManualPanelExpanded((value) => !value)} />}
        </div>
          {errorLogOpen && <ErrorLog errors={controlErrors} onClose={closeErrorLog} />}
      </section>
    </main>
  );
}


const ManualBar = memo(ManualBarContent);

function PanelToggle({ title, expanded, controls, onToggle }: { title: string; expanded: boolean; controls: string; onToggle: () => void }) {
  return <button type="button" className="hmi-panel-toggle inline-grid size-7 shrink-0 place-items-center border border-ptagv-border bg-white text-sm font-bold text-ptagv-text hover:bg-slate-100 focus-visible:outline-2 focus-visible:outline-ptagv-blue" aria-label={`${expanded ? "Collapse" : "Expand"} ${title}`} aria-expanded={expanded} aria-controls={controls} onClick={onToggle}><span className="hmi-panel-chevron" aria-hidden="true">▾</span></button>;
}

function setPanelContentInert(element: HTMLDivElement | null, inert: boolean) {
  if (element) element.toggleAttribute("inert", inert);
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
  moveButtonEvents: (action: ManualAction) => ManualButtonHandlers;
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
        className={`manual-key manual-key-${item.action.toLowerCase()} ${manualGridPlacement[item.action]} inline-flex min-h-8 w-10 flex-col items-center justify-center gap-0.5 border text-base font-bold leading-none touch-none select-none disabled:cursor-not-allowed disabled:border-ptagv-border disabled:bg-slate-100 disabled:text-slate-400 ${item.action === "STOP" ? "border-slate-600 bg-slate-600 text-white" : activeManualCommand === item.action ? "is-active border-blue-500 bg-blue-50 text-ptagv-blue" : "border-ptagv-border bg-slate-50 text-ptagv-text hover:bg-slate-100"}`}
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
          {key && <small className="text-[0.5rem] font-normal leading-none">{key}</small>}
      </button>
    );
  };

  return (
    <section
      className="robot-detail-manual hmi-manual-jog-panel hmi-floating-panel pointer-events-auto relative w-full shrink-0 overflow-hidden border border-ptagv-border bg-white text-ptagv-text"
      aria-label="Manual jog panel"
      data-testid="manual-jog-panel"
      data-expanded={expanded}
    >
      <header className="robot-detail-manual-head hmi-floating-header flex min-h-8 items-center justify-between gap-hmi-1 border-b border-ptagv-border-soft bg-ptagv-surface-muted px-hmi-2 py-1">
        <div className="flex min-w-0 items-center gap-hmi-1">
          <span className="hmi-manual-kicker text-hmi-kicker font-bold text-ptagv-muted">MANUAL JOG</span>
          <b className="truncate text-xs text-ptagv-text">
            {activeManualCommand
              ? `${activeManualCommand.replace(/_/g, " ")} · HOLDING`
              : "STOPPED"}
          </b>
        </div>

        <Badge tone={controlMode === "MANUAL" ? "info" : "neutral"}>{controlMode === "AUTONOMOUS" ? "AUTO" : "MANUAL"}</Badge>
        <PanelToggle title="Manual jog" expanded={expanded} controls={controls} onToggle={onToggle} />
      </header>

      <div ref={(element) => setPanelContentInert(element, !expanded)} className="hmi-floating-content min-h-0" id={controls} data-expanded={expanded} aria-hidden={!expanded}>
      <div className="hmi-floating-content-inner min-h-0 overflow-auto">
      <div className="hmi-jog-layout flex items-center justify-center gap-hmi-1 p-hmi-1">
        <div
          className="hmi-jog-pad hmi-jog-pad-8way grid grid-cols-[repeat(3,var(--spacing-hmi-jog-cell))] grid-rows-[repeat(3,var(--spacing-hmi-jog-row))] gap-hmi-1"
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
          className="hmi-rotate-pad grid w-[5.125rem] grid-cols-2 gap-hmi-1"
          role="group"
          aria-label="Manual rotation controls"
        >
          {button("ROTATE_LEFT", "Q")}
          {button("ROTATE_RIGHT", "E")}
        </div>
      </div>
      </div>
      </div>
    </section>
  );
}

function DashboardStatusPanel({ title, rows, expanded, controls, onToggle }: { title: string; rows: Array<[string, string]>; expanded: boolean; controls: string; onToggle: () => void }) {
  return <section className="hmi-dashboard-status-panel hmi-floating-panel pointer-events-auto relative w-full shrink-0 overflow-hidden border border-ptagv-border bg-white" aria-label={title} data-expanded={expanded}>
    <header className="hmi-floating-header flex min-h-8 items-center justify-between gap-1 border-b border-ptagv-border-soft bg-ptagv-surface-muted px-hmi-2 py-1"><b className="text-[0.6rem] font-bold tracking-wide text-ptagv-navy">{title}</b><PanelToggle title={title} expanded={expanded} controls={controls} onToggle={onToggle} /></header>
    <div ref={(element) => setPanelContentInert(element, !expanded)} className="hmi-floating-content min-h-0" id={controls} data-expanded={expanded} aria-hidden={!expanded}>
    <div className="hmi-floating-content-inner min-h-0 overflow-auto"><dl className="m-0 grid gap-x-hmi-2 px-hmi-2 py-1">{rows.map(([label, value]) => {
      const state = value.toUpperCase();
      const tone = ["READY", "LIVE", "ONLINE", "RECEIVING", "LOCALIZED", "CLEAR", "ACTIVE", "RUNNING"].includes(state)
        ? "is-good" : ["BLOCKED", "INITIAL POSE REQUIRED", "WAITING", "STARTING", "UNKNOWN"].includes(state)
          ? "is-warning" : ["OFFLINE", "UNAVAILABLE", "INACTIVE", "ERROR", "FAILED"].includes(state) ? "is-fault" : "is-neutral";
      return <div className="grid min-h-6 grid-cols-[minmax(0,1fr)_minmax(0,1.1fr)] items-center gap-hmi-1 border-b border-ptagv-border-soft last:border-b-0" key={label}><dt className="min-w-0 text-[0.55rem] font-bold text-ptagv-muted">{label}</dt><dd className={`m-0 min-w-0 text-right text-[0.55rem] font-bold [overflow-wrap:anywhere] ${tone} ${dashboardToneClass[tone]}`}>{value}</dd></div>;
    })}</dl></div></div>
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

  return <div className="hmi-error-modal-backdrop fixed inset-0 z-[10000] grid place-items-center bg-slate-950/65 p-hmi-4" data-testid="control-error-log-backdrop"
    onClick={(event) => { if (event.target === event.currentTarget) onClose(); }}>
    <section className="hmi-error-modal flex max-h-[70vh] w-[min(820px,calc(100vw-2rem))] flex-col overflow-hidden border border-ptagv-border bg-white text-ptagv-text shadow-lg" role="dialog" aria-modal="true" aria-labelledby={titleId}
      aria-label="Error log" data-testid="control-error-log" data-error-count={errors.length}>
    <header className="flex min-h-10 shrink-0 items-center gap-hmi-2 border-b border-ptagv-border-soft bg-ptagv-surface-muted px-hmi-3 py-1"><h2 className="m-0 text-sm font-bold tracking-wide text-ptagv-navy" id={titleId}>ERROR LOG</h2><span className="ml-auto text-xs text-ptagv-muted">{errors.length} {errors.length === 1 ? "ENTRY" : "ENTRIES"}</span>
      <button type="button" className="hmi-error-modal-close grid size-7 place-items-center border border-ptagv-border bg-white text-lg text-ptagv-text hover:bg-slate-100" aria-label="Close error log" onClick={onClose}>×</button></header>
    <div className="hmi-error-modal-body flex min-h-24 flex-1 flex-col overflow-hidden">
    {newestFirst.length === 0 ? <p className="hmi-error-log-empty grid min-h-24 place-items-center m-0 text-sm text-ptagv-muted">No active errors.</p> : <div className="hmi-error-log-table-wrap min-h-0 flex-1 overflow-auto [scrollbar-width:thin]">
      <table className="hmi-error-log-table w-full table-fixed border-collapse text-xs leading-snug text-ptagv-text">
        <thead className="sticky top-0 z-10 bg-white"><tr><th className="w-28 border-b border-ptagv-border-soft p-hmi-1 text-left text-hmi-kicker text-ptagv-muted" scope="col">TIME</th><th className="w-28 border-b border-ptagv-border-soft p-hmi-1 text-left text-hmi-kicker text-ptagv-muted" scope="col">LEVEL</th><th className="w-36 border-b border-ptagv-border-soft p-hmi-1 text-left text-hmi-kicker text-ptagv-muted" scope="col">SOURCE</th><th className="border-b border-ptagv-border-soft p-hmi-1 text-left text-hmi-kicker text-ptagv-muted" scope="col">MESSAGE</th></tr></thead>
        <tbody>{newestFirst.map((item, index) => {
          const level = String(item.severity ?? "ERROR").toUpperCase();
          const source = (item as RobotDetailError & { source?: string | null }).source;
          const parsedTime = item.timestamp ? Date.parse(item.timestamp) : Number.NaN;
          const displayTime = Number.isFinite(parsedTime) ? new Date(parsedTime).toLocaleTimeString() : "—";
          return <tr key={`${item.timestamp ?? "untimed"}-${item.code ?? item.message}-${index}`}>
            <td className="border-b border-ptagv-border-soft p-hmi-1 align-middle whitespace-nowrap">{displayTime}</td>
            <td className="border-b border-ptagv-border-soft p-hmi-1 align-middle"><span className={`hmi-error-level inline-flex min-w-16 justify-center border px-2 py-0.5 text-hmi-kicker font-bold ${level.includes("CRITICAL") || level.includes("ERROR") ? "is-error border-red-300 bg-red-50 text-ptagv-error" : level.includes("WARN") ? "is-warning border-amber-300 bg-amber-50 text-ptagv-warning" : "border-ptagv-border bg-slate-50 text-ptagv-muted"}`}>{level}</span></td>
            <td className="border-b border-ptagv-border-soft p-hmi-1 align-middle [overflow-wrap:anywhere]">{source || "—"}</td>
            <td className="border-b border-ptagv-border-soft p-hmi-1 align-middle [overflow-wrap:anywhere]">{item.message}</td>
          </tr>;
        })}</tbody>
      </table>
    </div>}
    </div>
    </section>
  </div>;
}
