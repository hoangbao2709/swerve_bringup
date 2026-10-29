import { useEffect, useState } from "react";
import { useLocalAppRuntime } from "../../app/AppLayout";
import { useFocusTrap } from "../../components/ui/useFocusTrap";
import { Alert, Button, MetricRow, StatusIndicator } from "../../components/shared/ui/HmiPrimitives";
import { IS_SHOWCASE, IS_STATIC_UI } from "../../config";
import { LOCAL_APP_PATHS, handleLocalLinkClick } from "../../routing/localRoutes";
import { getLocalRuntimeMode, type LocalRuntimeModeStatus } from "../../services/api";
import { controlModeBlockReason, isRobotStopped } from "../../services/controlModeSafety";
import { useStore } from "../../state/store";
import { SettingsCard, SettingsPageFrame } from "./SettingsPrimitives";

type ControlMode = "MANUAL" | "AUTONOMOUS";
type RuntimeMode = "MAPPING" | "NAVIGATION" | "UNIFIED" | "IDLE" | "SIMULATION" | "ERROR";
const localSelectionOnly = IS_STATIC_UI || IS_SHOWCASE;

function safeRuntimeMode(value: string | null | undefined): RuntimeMode | null {
  const normalized = String(value ?? "").toUpperCase();
  return ["MAPPING", "NAVIGATION", "UNIFIED", "IDLE", "SIMULATION", "ERROR"].includes(normalized)
    ? normalized as RuntimeMode
    : null;
}

function RuntimeStatusLink({ href, children }: { href: string; children: string }) {
  return <a href={href} onClick={(event) => handleLocalLinkClick(event, href)}
    className="inline-flex min-h-9 items-center border border-ptagv-border bg-white px-3 text-xs font-bold text-ptagv-blue no-underline hover:bg-slate-50 focus-visible:outline-2 focus-visible:outline-ptagv-blue">{children}</a>;
}

export function OperationModeSettingsPage({ embedded = false, active = true }: { embedded?: boolean; active?: boolean } = {}) {
  const runtime = useLocalAppRuntime();
  const runtimeState = useStore((state) => state.runtimeState);
  const runtimeProfile = useStore((state) => state.runtimeMode);
  const setRuntimeState = useStore((state) => state.setRuntimeState);
  const capabilities = useStore((state) => runtime.robotId ? state.robotCapabilities[runtime.robotId] ?? null : null);
  const activeMapId = useStore((state) => runtime.robotId ? state.robotDetail[runtime.robotId]?.activeLocalMapId ?? null : null);
  const [serverRuntimeMode, setServerRuntimeMode] = useState<string | null>(null);
  const [transition, setTransition] = useState<LocalRuntimeModeStatus | null>(null);
  const [pendingMapId, setPendingMapId] = useState<string | null>(null);
  const [refreshError, setRefreshError] = useState("");
  const [modeToConfirm, setModeToConfirm] = useState<ControlMode | null>(null);
  const dialogRef = useFocusTrap<HTMLDivElement>(modeToConfirm !== null && active);

  useEffect(() => {
    if (!active || localSelectionOnly || !runtime.robotId) return;
    let alive = true;
    let timer: number | undefined;
    let busy = false;
    let pollTransitionSoon = false;
    const refresh = async () => {
      if (busy) return;
      busy = true;
      try {
        const result = await getLocalRuntimeMode(runtime.robotId!);
        if (!alive) return;
        setServerRuntimeMode(result.current_mode);
        setTransition(result.transition);
        setPendingMapId(result.pending_local_map_id ?? null);
        setRefreshError("");
        pollTransitionSoon = ["REQUESTED", "STARTING", "RESTARTING", "ROLLING_BACK"].includes(String(result.transition.status).toUpperCase());
        const next = safeRuntimeMode(result.current_mode);
        if (next) setRuntimeState(next);
      } catch (caught) {
        if (alive) setRefreshError(caught instanceof Error ? caught.message : "Runtime status could not be loaded.");
      } finally {
        busy = false;
        if (alive) {
          timer = window.setTimeout(() => void refresh(), pollTransitionSoon ? 1000 : 5000);
        }
      }
    };
    void refresh();
    return () => { alive = false; if (timer !== undefined) window.clearTimeout(timer); };
  }, [active, runtime.robotId, setRuntimeState]);

  const currentRuntimeMode = localSelectionOnly
    ? safeRuntimeMode(runtimeState) ?? "IDLE"
    : safeRuntimeMode(serverRuntimeMode) ?? safeRuntimeMode(runtimeState) ?? "IDLE";
  const runtimeFixed = currentRuntimeMode === "UNIFIED";
  const previewRequested = runtime.previewRequestedControlMode;
  const backendRequested = runtime.requestedControlMode;
  const requested = localSelectionOnly ? previewRequested : backendRequested;
  const transitionState = localSelectionOnly
    ? previewRequested ? "REQUESTED" : ""
    : runtime.controlModeTransitionState ?? "";
  const controlModeReason = localSelectionOnly ? null : controlModeBlockReason({
    online: runtime.controlOnline,
    robot: runtime.robot,
    estopClear: runtime.estopState === "CLEAR",
    pending: runtime.controlModeTransitionState === "REQUESTED",
    busy: runtime.busy,
  });
  const canRequestMode = (mode: ControlMode) => localSelectionOnly
    ? IS_SHOWCASE || mode !== runtime.controlMode
    : Boolean(!controlModeReason && runtime.appliedControlMode !== mode);
  const runtimeTransitionPending = ["REQUESTED", "STARTING", "RESTARTING", "ROLLING_BACK"].includes(String(transition?.status ?? "").toUpperCase());
  const stopped = isRobotStopped(runtime.robot);

  const confirmModeRequest = () => {
    if (!modeToConfirm) return;
    runtime.setControlMode(modeToConfirm);
    setModeToConfirm(null);
  };

  const runtimeStatus = localSelectionOnly
    ? IS_SHOWCASE ? "Local mode selection only; robot runtime unchanged." : ""
    : transition?.message || (runtimeTransitionPending ? "A runtime transition is in progress." : "Use supervised Mapping or Saved Maps to change runtime.");

  return <SettingsPageFrame title="Operation Mode" embedded={embedded} active={active}
    sectionId={embedded ? "settings-operation-mode" : undefined}>
    {refreshError && <Alert tone="error" role="alert">Runtime status unavailable. {refreshError}</Alert>}
    {runtime.error && <Alert tone="error" role="alert">{runtime.error}</Alert>}

    <div className="grid min-w-0 grid-cols-1 gap-hmi-3 xl:grid-cols-2 xl:items-start">
      <SettingsCard title="ROBOT CONTROL MODE">
        {!localSelectionOnly && <p className="m-0 text-xs leading-relaxed text-ptagv-muted">Applied mode changes only after runtime confirmation.</p>}
        {localSelectionOnly
          ? <MetricRow label="ROBOT-REPORTED MODE" value="—" />
          : <MetricRow label="APPLIED ROBOT MODE" value={runtime.appliedControlMode ?? "—"} />}
        {!localSelectionOnly && <MetricRow label="REQUESTED MODE" value={requested ?? "—"} />}
        {!localSelectionOnly && <MetricRow label="TRANSITION STATE" value={transitionState || "—"} />}
        {localSelectionOnly && <MetricRow label={IS_SHOWCASE ? "LOCAL SELECTION" : "BROWSER SELECTION"} value={runtime.controlMode} />}
        <div className="grid min-w-0 grid-cols-1 gap-2 sm:grid-cols-2">
          {(["MANUAL", "AUTONOMOUS"] as const).map((mode) => <button key={mode} type="button"
            aria-pressed={localSelectionOnly ? runtime.controlMode === mode : runtime.appliedControlMode === mode}
            disabled={!canRequestMode(mode)} onClick={() => {
              if (IS_SHOWCASE) runtime.setControlMode(mode);
              else setModeToConfirm(mode);
            }}
            className={`min-h-14 border px-3 py-2 text-left focus-visible:outline-2 focus-visible:outline-ptagv-blue disabled:cursor-not-allowed disabled:opacity-55 ${(localSelectionOnly ? runtime.controlMode === mode : runtime.appliedControlMode === mode) ? "border-blue-400 bg-blue-50" : "border-ptagv-border bg-white hover:bg-slate-50"}`}>
            <b className="block text-sm text-ptagv-text">{mode}</b>
            <small className="text-xs text-ptagv-muted">{mode === "MANUAL" ? "Operator motion" : "Autonomy"}</small>
          </button>)}
        </div>
        {!localSelectionOnly && controlModeReason && <p className="m-0 text-xs text-ptagv-warning" role="note">{controlModeReason}</p>}
        {IS_STATIC_UI && previewRequested && <div className="grid gap-2 border-t border-ptagv-border-soft pt-hmi-2">
          <StatusIndicator tone="info">REQUESTED · {previewRequested}</StatusIndicator>
          <Button tone="primary" onClick={runtime.confirmPreviewControlMode}>CONFIRM BROWSER SELECTION</Button>
        </div>}
        {!localSelectionOnly && requested && transitionState === "REQUESTED" && <Alert tone="info" role="status">Waiting for runtime confirmation of {requested}; not applied yet.</Alert>}
        {!localSelectionOnly && transitionState === "FAILED" && <Alert tone="error" role="alert">Runtime rejected the request. Applied mode remains {runtime.appliedControlMode ?? "unreported"}.</Alert>}
      </SettingsCard>

      <SettingsCard title="MAPPING / NAVIGATION RUNTIME">
        {!localSelectionOnly && <p className="m-0 text-xs leading-relaxed text-ptagv-muted">Change runtime through supervised Mapping or Saved Maps.</p>}
        <MetricRow label="RUNTIME PROFILE" value={localSelectionOnly ? IS_SHOWCASE ? "STANDALONE SAMPLE" : "—" : runtimeProfile} />
        <MetricRow label="CURRENT RUNTIME MODE" value={currentRuntimeMode} />
        <MetricRow label="RUNTIME TRANSITION" value={localSelectionOnly ? "NOT CONNECTED" : transition?.status ?? "—"} />
        {!localSelectionOnly && <MetricRow label="ACTIVE LOCAL MAP" value={activeMapId ?? "—"} mono />}
        {!localSelectionOnly && pendingMapId && <Alert tone="warning" role="status">Saved map {pendingMapId} is still loading. Wait for that supervised transition to finish.</Alert>}
        <div className="grid gap-2 border-t border-ptagv-border-soft pt-hmi-2">
          <div className="flex items-center justify-between gap-2"><span className="text-sm text-ptagv-text">Stopped robot state</span><StatusIndicator tone={localSelectionOnly ? "neutral" : stopped ? "success" : "warning"}>{localSelectionOnly ? "NOT VERIFIED" : stopped ? "CONFIRMED" : "REQUIRED"}</StatusIndicator></div>
          <div className="flex items-center justify-between gap-2"><span className="text-sm text-ptagv-text">E-STOP clear</span><StatusIndicator tone={localSelectionOnly ? "neutral" : runtime.estopState === "CLEAR" ? "success" : "warning"}>{localSelectionOnly ? "NOT REPORTED" : runtime.estopState === "CLEAR" ? "CONFIRMED" : "REQUIRED"}</StatusIndicator></div>
          <div className="flex items-center justify-between gap-2"><span className="text-sm text-ptagv-text">SLAM ready</span><StatusIndicator tone={localSelectionOnly ? "neutral" : capabilities?.mapping_available ? "success" : "warning"}>{localSelectionOnly ? "NOT CONNECTED" : capabilities?.mapping_available ? "READY" : "REQUIRED"}</StatusIndicator></div>
          <div className="flex items-center justify-between gap-2"><span className="text-sm text-ptagv-text">Navigation map / Nav2</span><StatusIndicator tone={localSelectionOnly ? "neutral" : activeMapId && capabilities?.nav2_ready ? "success" : "warning"}>{localSelectionOnly ? "NOT CONNECTED" : activeMapId && capabilities?.nav2_ready ? "READY" : "REQUIRED"}</StatusIndicator></div>
        </div>
        {runtimeFixed && <Alert tone="neutral" role="note">UNIFIED runtime is fixed. Change MANUAL/AUTONOMOUS separately.</Alert>}
        {runtimeStatus && <p className="m-0 text-xs leading-relaxed text-ptagv-muted">{runtimeStatus}</p>}
        {IS_STATIC_UI ? <div className="flex flex-wrap gap-2 border-t border-ptagv-border-soft pt-hmi-2">
          <Button disabled={currentRuntimeMode === "MAPPING"} onClick={() => setRuntimeState("MAPPING")}>SELECT MAPPING</Button>
          <Button disabled={currentRuntimeMode === "NAVIGATION"} onClick={() => setRuntimeState("NAVIGATION")}>SELECT NAVIGATION</Button>
        </div> : <div className="flex flex-wrap gap-2 border-t border-ptagv-border-soft pt-hmi-2">
          <RuntimeStatusLink href={LOCAL_APP_PATHS.MAPPING}>OPEN MAPPING WORKFLOW</RuntimeStatusLink>
          <RuntimeStatusLink href={LOCAL_APP_PATHS.SETTINGS_MAP}>SELECT & APPLY SAVED MAP</RuntimeStatusLink>
        </div>}
      </SettingsCard>
    </div>

    {modeToConfirm && <div className="fixed inset-0 z-50 grid place-items-center bg-slate-950/45 p-hmi-3" role="presentation">
      <div ref={dialogRef} role="dialog" aria-modal="true" aria-labelledby="confirm-control-mode-title" tabIndex={-1}
        className="grid w-full max-w-lg gap-hmi-3 border border-ptagv-border bg-white p-hmi-4 shadow-xl">
        <h3 id="confirm-control-mode-title" className="m-0 text-base font-bold text-ptagv-text">Confirm {modeToConfirm} mode</h3>
        <p className="m-0 text-sm leading-relaxed text-ptagv-muted">{localSelectionOnly
          ? "Browser selection only. No robot command is sent."
          : "This sends a request; applied mode changes only after runtime acknowledgement."}</p>
        {!localSelectionOnly && controlModeReason && <Alert tone="warning" role="note">{controlModeReason}</Alert>}
        <div className="flex flex-wrap justify-end gap-2">
          <Button onClick={() => setModeToConfirm(null)}>CANCEL</Button>
          <Button tone="primary" disabled={!localSelectionOnly && Boolean(controlModeReason)} onClick={confirmModeRequest}>CONFIRM REQUEST</Button>
        </div>
      </div>
    </div>}
  </SettingsPageFrame>;
}
