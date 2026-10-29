import { Button } from "../components/shared/ui/HmiPrimitives";

type LocalAppHeaderProps = {
  pageTitle: string;
  robotOnline: boolean;
  controlMode: "MANUAL" | "AUTONOMOUS";
  controlModeLabel: string;
  slamLabel: string;
  slamReady: boolean;
  nav2Label: string;
  nav2Tone: "is-ready" | "is-warning" | "is-fault";
  estopLabel: string;
  estopTone: "is-ready" | "is-warning" | "is-fault";
  controlOnline: boolean;
  showcaseMode?: boolean;
  canChangeMode?: boolean;
  modeChangeBlockReason?: string;
  busy: boolean;
  emergencyStopBusy: boolean;
  canEmergencyStop: boolean;
  onModeChange: (mode: "MANUAL" | "AUTONOMOUS") => void;
  onEmergencyStop: () => void;
  onClearStop: () => void;
};

/**
 * Presentation-only header. Runtime diagnostics and E-STOP state are still
 * evaluated by AppLayout; this component intentionally does not render them.
 * Keep the original props contract for existing callers.
 */
export function LocalAppHeader({
  pageTitle,
  controlMode,
  controlOnline,
  showcaseMode = false,
  canChangeMode = controlOnline,
  modeChangeBlockReason,
  busy,
  emergencyStopBusy,
  canEmergencyStop,
  onModeChange,
  onEmergencyStop,
  onClearStop,
}: LocalAppHeaderProps) {
  return (
    <header
      className="local-app-header app-shell-header-height flex shrink-0 flex-wrap items-center justify-between gap-x-4 gap-y-2 border-b border-blue-950/40 bg-ptagv-navy px-3 py-2 text-white"
      aria-label={`PTAGV ${pageTitle}`}
    >
      <div className="flex min-w-0 flex-wrap items-center gap-x-3 gap-y-1">
        <h1 className="local-app-header-title m-0 truncate text-xs font-bold tracking-wider">
          {pageTitle}
        </h1>
        {showcaseMode && <span className="inline-flex min-h-6 items-center border border-amber-200/40 bg-amber-100/10 px-2 text-[0.62rem] font-bold tracking-wide text-amber-100" role="status">
          ROBOT NOT CONNECTED
        </span>}
      </div>

      <div
        className="flex shrink-0 flex-wrap items-center justify-end gap-1.5"
        role="group"
        aria-label="Robot control actions"
      >
        <div className="flex items-center gap-1 rounded-sm border border-white/20 bg-white/10 p-0.5">
          <Button
            size="compact"
            className={`min-h-9 px-3 text-[0.7rem] font-bold ${
              (controlOnline || showcaseMode) && controlMode === "MANUAL"
              ? "border-blue-200 !bg-blue-100 !text-ptagv-navy"
                : "border-transparent !bg-transparent !text-white hover:!bg-white/15"
            }`}
            aria-pressed={controlMode === "MANUAL"}
            disabled={!showcaseMode && (!canChangeMode || busy)}
            title={modeChangeBlockReason}
            onClick={() => onModeChange("MANUAL")}
          >
            MANUAL
          </Button>
          <Button
            size="compact"
            className={`min-h-9 px-3 text-[0.7rem] font-bold ${
              (controlOnline || showcaseMode) && controlMode === "AUTONOMOUS"
              ? "border-blue-200 !bg-blue-100 !text-ptagv-navy"
                : "border-transparent !bg-transparent !text-white hover:!bg-white/15"
            }`}
            aria-pressed={controlMode === "AUTONOMOUS"}
            disabled={!showcaseMode && (!canChangeMode || busy)}
            title={modeChangeBlockReason}
            onClick={() => onModeChange("AUTONOMOUS")}
          >
            AUTONOMOUS
          </Button>
        </div>

        <Button
          size="compact"
          tone="danger"
          className="hmi-estop min-h-9 px-3 text-[0.7rem] font-extrabold"
            disabled={emergencyStopBusy || !canEmergencyStop}
            title={canEmergencyStop ? undefined : "Requires a connected robot."}
          onClick={onEmergencyStop}
        >
          EMERGENCY STOP
        </Button>
        <Button
          size="compact"
          className="hmi-clear-stop min-h-9 border-green-800 bg-green-800 px-3 text-[0.7rem] font-bold text-white hover:bg-green-900 disabled:bg-slate-400"
          disabled={busy || !controlOnline}
          title={controlOnline ? undefined : "Requires a connected robot."}
          onClick={onClearStop}
        >
          CLEAR STOP
        </Button>
      </div>
    </header>
  );
}
