import {
  Component,
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
  type ErrorInfo,
  type ReactNode,
} from "react";
import { clearEmergencyStop, emergencyStop } from "../services/api";
import { wsSend, wsSetRobotMode } from "../services/ws";
import { useStore } from "../state/store";
import type { RobotState, RobotSystemDiagnostics } from "../schema/twin_state";
import { IS_SHOWCASE, IS_STATIC_UI } from "../config";
import { runtimeEnv } from "../runtimeEnv";
import { LocalAppHeader } from "./LocalAppHeader";
import { Sidebar } from "./Sidebar";
import { PreviewRobotContext } from "./PreviewRobotContext";
import { Alert } from "../components/shared/ui/HmiPrimitives";
import { resolveEStopState, type EStopState } from "../services/estopState";
import { controlModeBlockReason } from "../services/controlModeSafety";

type RobotControlMode = "MANUAL" | "AUTONOMOUS";

export type ControlPageActions = {
  stopManual: () => void;
  clearNavigationPreview: () => void;
};

export type LocalAppRuntime = {
  robotId: string | null;
  robot: RobotState | undefined;
  diagnostics: RobotSystemDiagnostics | null;
  controlMode: RobotControlMode;
  setControlMode: (mode: RobotControlMode, forceRequest?: boolean) => void;
  appliedControlMode: RobotControlMode | null;
  requestedControlMode: RobotControlMode | null;
  controlModeTransitionState: "REQUESTED" | "APPLIED" | "FAILED" | null;
  previewRequestedControlMode: RobotControlMode | null;
  confirmPreviewControlMode: () => void;
  controlModeBlockReason: string | null;
  robotOnline: boolean;
  controlOnline: boolean;
  estopState: EStopState;
  identityError: string | null;
  error: string;
  setError: (error: string) => void;
  safetyNotice: string;
  setSafetyNotice: (notice: string) => void;
  busy: boolean;
  registerControlPageActions: (actions: ControlPageActions) => () => void;
};

const LocalAppRuntimeContext = createContext<LocalAppRuntime | null>(null);

export function useLocalAppRuntime(): LocalAppRuntime {
  const runtime = useContext(LocalAppRuntimeContext);
  if (!runtime) throw new Error("useLocalAppRuntime must be used inside AppLayout");
  return runtime;
}

class AppErrorBoundary extends Component<{ children: ReactNode }, { error: Error | null }> {
  state = { error: null as Error | null };
  static getDerivedStateFromError(error: Error) { return { error }; }
  componentDidCatch(error: Error, info: ErrorInfo) { console.error("[PTAGV AppLayout] render error", error, info); }
  render() {
    if (!this.state.error) return this.props.children;
    return <main className="grid min-h-screen place-items-center bg-ptagv-page p-hmi-4 text-ptagv-text" role="alert">
      <section className="w-full max-w-2xl border border-ptagv-border bg-white p-hmi-4">
        <span className="text-hmi-kicker font-bold tracking-widest text-ptagv-blue">PTAGV WEB LOCAL</span>
        <h1 className="mb-2 mt-2 text-xl font-bold">Page could not be rendered</h1>
        <pre className="max-h-[40vh] overflow-auto whitespace-pre-wrap break-words border border-red-200 bg-red-50 p-hmi-3 text-sm text-ptagv-error">{this.state.error.message}</pre>
        <button type="button" className="mt-hmi-3 min-h-10 border border-ptagv-border bg-white px-hmi-3 text-hmi-kicker font-bold hover:bg-slate-50 focus-visible:outline-2 focus-visible:outline-ptagv-blue" onClick={() => window.location.reload()}>RELOAD VIEW</button>
      </section>
    </main>;
  }
}

type AppLayoutProps = {
  pageTitle: string;
  isControlPage: boolean;
  children: ReactNode;
};

export function AppLayout({ pageTitle, isControlPage, children }: AppLayoutProps) {
  const configuredRobotId = useStore((state) => state.localRobotId);
  const previewRobotId = useContext(PreviewRobotContext);
  const isStandaloneUi = IS_STATIC_UI || IS_SHOWCASE;
  const robotId = isStandaloneUi ? previewRobotId : configuredRobotId;
  const robot = useStore((state) => robotId ? state.twin?.robots?.[robotId] : undefined);
  const estopTelemetryAt = useStore((state) => robotId ? state.robotDetail[robotId]?.estopTelemetryAt : null);
  const lastTelemetryAt = useStore((state) => state.lastTelemetryAt);
  const detailDiagnostics = useStore((state) => robotId ? state.robotDetail[robotId]?.diagnostics : null);
  const rosDiagnostics = useStore((state) => state.rosDiagnostics);
  const diagnostics = (detailDiagnostics ?? rosDiagnostics) as RobotSystemDiagnostics | null;
  const connectedRobotIds = useStore((state) => state.connectedRobotIds);
  const rosConnected = useStore((state) => state.rosConnected);
  const websocketState = useStore((state) => state.websocketState);
  const activeLocalMapId = useStore((state) => robotId ? state.robotDetail[robotId]?.activeLocalMapId ?? null : null);
  const activeLocalMapRevision = useStore((state) => robotId ? state.robotDetail[robotId]?.activeLocalMapRevision ?? null : null);
  const setRobotDetail = useStore((state) => state.setRobotDetail);
  const identityVerified = useStore((state) => state.localIdentityVerified);
  const identityError = useStore((state) => state.localIdentityError);
  const runtimeCapabilities = useStore((state) => robotId ? state.robotCapabilities[robotId] ?? null : null);
  const appliedMode = useStore((state) => robotId ? state.robotDetail[robotId]?.appliedMode : undefined);
  const requestedMode = useStore((state) => robotId ? state.robotDetail[robotId]?.requestedMode : undefined);
  const modeTransitionState = useStore((state) => robotId ? state.robotDetail[robotId]?.modeTransitionState : undefined);
  const showcaseControlMode = useStore((state) => state.showcaseControlMode);
  const setShowcaseControlMode = useStore((state) => state.setShowcaseControlMode);
  const [runtimeControlMode, setRuntimeControlMode] = useState<RobotControlMode>(robot?.control_mode ?? "AUTONOMOUS");
  const controlMode = IS_SHOWCASE ? showcaseControlMode : runtimeControlMode;
  const [previewRequestedControlMode, setPreviewRequestedControlMode] = useState<RobotControlMode | null>(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");
  const [safetyNotice, setSafetyNotice] = useState("");
  const [emergencyStopBusy, setEmergencyStopBusy] = useState(false);
  const emergencyStopPending = useRef(false);
  const [clockNow, setClockNow] = useState(() => Date.now());
  const controlPageActions = useRef<ControlPageActions | null>(null);

  const robotBridgeOnline = robotId !== null && connectedRobotIds.includes(robotId);
  const robotOnline = Boolean(robot && robot.status !== "OFFLINE" && robotBridgeOnline && rosConnected && websocketState === "CONNECTED");
  const controlOnline = robotOnline && identityVerified;
  const estopActive = detailDiagnostics?.command_ownership?.estop_active
    ?? diagnostics?.command_ownership?.estop_active;
  const estopObservedAt = estopTelemetryAt ?? diagnostics?.last_update_at ?? robot?.last_telemetry_at ?? lastTelemetryAt;
  const estopState = resolveEStopState(estopActive, robotOnline, estopObservedAt, clockNow);
  const nav2Ready = runtimeCapabilities?.nav2_ready === true && diagnostics?.nav2_ready === true;
  const slamLive = diagnostics?.mapping?.slam_state === "ACTIVE" && diagnostics.mapping.map_live;
  const nav2BlockerReason = runtimeCapabilities?.goal_blocker_reason
    ?? diagnostics?.nav2_lifecycle_blocker_reason ?? "";
  const modeBlockReason = isStandaloneUi ? null : controlModeBlockReason({
    online: controlOnline,
    robot,
    estopClear: estopState === "CLEAR",
    pending: modeTransitionState === "REQUESTED",
    busy,
  });

  useEffect(() => {
    if (!robotOnline || estopState === "UNKNOWN" || !estopObservedAt) return;
    const timer = window.setInterval(() => setClockNow(Date.now()), 1000);
    return () => window.clearInterval(timer);
  }, [estopObservedAt, estopState, robotOnline]);

  useEffect(() => {
    if (IS_SHOWCASE) return;
    setRuntimeControlMode(appliedMode ?? robot?.control_mode ?? "AUTONOMOUS");
  }, [appliedMode, robot?.control_mode, robotId]);

  // Request the robot detail stream from the shared shell so map-dependent pages
  // keep receiving the same data while the CONTROL page is unmounted.
  useEffect(() => {
    if (isStandaloneUi || !robotBridgeOnline || websocketState !== "CONNECTED" || !robotId) return;
    const request_id = `${robotId}-${Date.now()}-${Math.random().toString(36).slice(2, 8)}`;
    setRobotDetail(robotId, { viewStatus: { robot_id: robotId, requested_view: "LIDAR_2D", request_id, state: "REQUESTED" } });
    wsSend({ type: "ROBOT_DETAIL_VIEW", robot_id: robotId, view: "LIDAR_2D", request_id, delivery_ack: true });
  }, [activeLocalMapId, activeLocalMapRevision, robotBridgeOnline, robotId, setRobotDetail, websocketState]);

  const registerControlPageActions = useCallback((actions: ControlPageActions) => {
    controlPageActions.current = actions;
    return () => {
      if (controlPageActions.current === actions) controlPageActions.current = null;
    };
  }, []);

  const setControlMode = useCallback((next: RobotControlMode, forceRequest = false) => {
    if (IS_STATIC_UI) {
      if (next !== controlMode) setPreviewRequestedControlMode(next);
      setError("");
      return;
    }
    if (IS_SHOWCASE) {
      setShowcaseControlMode(next);
      setPreviewRequestedControlMode(null);
      setError("");
      return;
    }
    controlPageActions.current?.stopManual();
    controlPageActions.current?.clearNavigationPreview();
    if (!forceRequest && next === (appliedMode ?? robot?.control_mode ?? controlMode)) return;
    const reason = controlModeBlockReason({
      online: controlOnline,
      robot,
      estopClear: estopState === "CLEAR",
      pending: modeTransitionState === "REQUESTED",
      busy,
    });
    if (reason) { setError(reason); return; }
    if (!robotId) { setError("MANUAL/AUTONOMOUS requires a verified local robot identity"); return; }
    if (!wsSetRobotMode(robotId, next)) { setError("Robot control channel is disconnected"); return; }
    useStore.getState().setRobotDetail(robotId, { requestedMode: next, modeTransitionState: "REQUESTED" });
    setError("");
  }, [appliedMode, busy, controlMode, controlOnline, estopState, modeTransitionState, robot, robotId, setShowcaseControlMode]);

  const confirmPreviewControlMode = useCallback(() => {
    if (!IS_STATIC_UI || !previewRequestedControlMode) return;
    setRuntimeControlMode(previewRequestedControlMode);
    setPreviewRequestedControlMode(null);
  }, [previewRequestedControlMode]);

  const runAction = useCallback(async (action: () => Promise<unknown>) => {
    setBusy(true);
    setError("");
    setSafetyNotice("");
    try { await action(); }
    catch (caught) { setError(caught instanceof Error ? caught.message : "Request failed"); }
    finally { setBusy(false); }
  }, []);

  const requestEmergencyStop = useCallback(async () => {
    controlPageActions.current?.stopManual();
    if (IS_STATIC_UI || IS_SHOWCASE) {
      return;
    }
    if (!robotId) {
      setError("Emergency stop request failed: local robot identity is unavailable");
      return;
    }
    if (emergencyStopPending.current) return;
    emergencyStopPending.current = true;
    setEmergencyStopBusy(true);
    setError("");
    setSafetyNotice("");
    try {
      await emergencyStop(robotId);
      setSafetyNotice("EMERGENCY STOP REQUEST ACCEPTED · waiting for fresh E-STOP telemetry");
    } catch (caught) {
      setError(caught instanceof Error ? caught.message : "Emergency stop request failed");
    } finally {
      emergencyStopPending.current = false;
      setEmergencyStopBusy(false);
    }
  }, [robotId]);

  const runtime = useMemo<LocalAppRuntime>(() => ({
    robotId,
    robot,
    diagnostics,
    controlMode,
    setControlMode,
    appliedControlMode: isStandaloneUi ? null : appliedMode ?? robot?.control_mode ?? null,
    requestedControlMode: isStandaloneUi ? null : requestedMode ?? null,
    controlModeTransitionState: isStandaloneUi ? null : modeTransitionState ?? null,
    previewRequestedControlMode: IS_STATIC_UI ? previewRequestedControlMode : null,
    confirmPreviewControlMode,
    controlModeBlockReason: modeBlockReason,
    robotOnline,
    controlOnline,
    estopState,
    identityError,
    error,
    setError,
    safetyNotice,
    setSafetyNotice,
    busy,
    registerControlPageActions,
  }), [robotId, robot, diagnostics, controlMode, setControlMode, appliedMode, requestedMode,
    modeTransitionState, previewRequestedControlMode, confirmPreviewControlMode, modeBlockReason,
    robotOnline, controlOnline, estopState, identityError, error, safetyNotice, busy, registerControlPageActions]);

  const estopLabel = estopState;
  const nav2Label = IS_STATIC_UI ? "UNAVAILABLE" : nav2Ready ? "READY" : nav2BlockerReason ? "BLOCKED" : diagnostics?.nav2 ? "STARTING" : "UNAVAILABLE";
  const controlModeLabel = modeTransitionState === "REQUESTED" ? `${controlMode} → ${requestedMode} REQUESTED`
    : modeTransitionState === "FAILED" ? `${controlMode} TRANSITION FAILED` : controlMode;
  const slamLabel = slamLive ? "LIVE" : String(diagnostics?.mapping?.slam_state ?? "STARTING");

  return <AppErrorBoundary>
    <LocalAppRuntimeContext.Provider value={runtime}>
      <div className="robot-control-workspace flex h-screen min-h-screen w-full min-w-0 overflow-hidden bg-ptagv-page text-ptagv-text max-[900px]:h-auto max-[900px]:min-h-screen max-[900px]:flex-col max-[900px]:overflow-auto">
        <Sidebar />
        <div className={`robot-detail-shell relative flex h-screen min-h-0 min-w-0 flex-1 flex-col overflow-hidden bg-ptagv-page font-sans text-ptagv-text max-[900px]:h-auto max-[900px]:min-h-[calc(100vh-3.5rem)] max-[900px]:overflow-auto industrial-hmi${isControlPage ? " is-control-view" : ""}${isControlPage && runtimeEnv.VITE_DEMO_COMPACT_VIEW === "true" ? " demo-compact-map" : ""}`}>
          <LocalAppHeader
            pageTitle={pageTitle}
            robotOnline={robotOnline}
            controlMode={controlMode}
            controlModeLabel={IS_STATIC_UI ? "UNAVAILABLE" : controlModeLabel}
            slamLabel={slamLabel}
            slamReady={Boolean(slamLive)}
            nav2Label={nav2Label}
            nav2Tone={IS_STATIC_UI ? "is-warning" : nav2Ready ? "is-ready" : diagnostics?.nav2 ? "is-warning" : "is-fault"}
            estopLabel={estopLabel}
            estopTone={estopState === "ACTIVE" ? "is-fault" : estopState === "CLEAR" ? "is-ready" : "is-warning"}
            controlOnline={controlOnline}
            showcaseMode={IS_SHOWCASE}
            canChangeMode={IS_SHOWCASE || Boolean(controlOnline && !modeBlockReason)}
            modeChangeBlockReason={IS_SHOWCASE ? "Sample selection only; no robot command is sent." : modeBlockReason ?? undefined}
            busy={busy}
            emergencyStopBusy={emergencyStopBusy}
            canEmergencyStop={!isStandaloneUi && Boolean(robotId)}
            onModeChange={setControlMode}
            onEmergencyStop={() => { void requestEmergencyStop(); }}
            onClearStop={() => {
              if (isStandaloneUi || !robotId) return;
              void runAction(async () => {
                const result = await clearEmergencyStop(robotId);
                if (result.code !== "CLEAR_ESTOP_APPLIED" || result.emergency_stop_active
                    || !result.pre_stop_navigation_terminal) throw new Error("CLEAR_ESTOP_UNCONFIRMED: runtime did not confirm the stop latch is clear");
                setSafetyNotice("CLEAR_ESTOP_APPLIED · bridge confirmed latch clear and prior navigation terminal");
              });
            }}
          />
          {(safetyNotice || error || identityError) && <div className="global-notification-region flex min-w-0 shrink-0 flex-wrap items-center justify-center gap-hmi-2 px-hmi-3 py-hmi-1" data-testid="global-notification-region">
            {(safetyNotice || error || identityError) && <Alert className="w-fit max-w-[min(48rem,100%)] py-hmi-1 text-xs leading-snug shadow-sm" tone={error || identityError ? "error" : "success"} role={error || identityError ? "alert" : "status"}>{error || identityError || safetyNotice}</Alert>}
          </div>}
          {robotId ? children : <div className="grid min-h-0 flex-1 place-items-center bg-ptagv-page px-hmi-4 text-center font-mono text-hmi-body font-bold tracking-widest text-ptagv-muted" role="status">{identityError ?? "WAITING FOR LOCAL ROBOT CONFIGURATION"}</div>}
        </div>
      </div>
    </LocalAppRuntimeContext.Provider>
  </AppErrorBoundary>;
}
