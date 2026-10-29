import { useState } from "react";
import type { LocalRobotMap } from "../../../services/api";
import { Alert, Badge, Button, Card, CardHeader, Input } from "../../../components/shared/ui/HmiPrimitives";
import { displayFixtureMapName, STATIC_UI_EMPTY_VALUE } from "../../../mock/staticPresentation";
import { useFocusTrap } from "../../../components/ui/useFocusTrap";

type Props = {
  maps: LocalRobotMap[];
  selected: string;
  activeMapId: string | null;
  isStatic: boolean;
  isShowcase?: boolean;
  resumeDisabledReason: string;
  resumeBusy: boolean;
  resumeState: string;
  resumeError: string;
  resumeNotice: string;
  onSelect: (id: string) => void;
  onResume: () => void;
};

export function SavedMapsPanel({ maps, selected, activeMapId, isStatic, isShowcase = false, resumeDisabledReason, resumeBusy, resumeState, resumeError, resumeNotice, onSelect, onResume }: Props) {
  const [filter, setFilter] = useState("");
  const [confirmingResume, setConfirmingResume] = useState(false);
  const resumeDialogRef = useFocusTrap<HTMLDivElement>(confirmingResume);
  const selectedMap = maps.find((map) => map.id === selected) ?? null;
  const sessionAvailable = selectedMap?.slam_session_state?.status?.toUpperCase() === "AVAILABLE"
    && selectedMap.slam_session_state.engine?.toUpperCase() === "SLAM_TOOLBOX";
  const resumeStatus = resumeBusy ? resumeState : resumeError || resumeNotice || resumeDisabledReason
    || (sessionAvailable ? "POSE GRAPH AVAILABLE" : "NO SAVED POSE GRAPH");
  const resumeStatusTone = resumeError ? "error" : resumeNotice ? "success" : resumeDisabledReason ? "warning" : "neutral";
  const normalizedFilter = filter.trim().toLocaleLowerCase();
  const filteredMaps = normalizedFilter ? maps.filter((map) => [map.name, map.id, map.map_id, map.revision, map.frame_id]
    .some((value) => String(value ?? "").toLocaleLowerCase().includes(normalizedFilter))) : maps;
  return <Card className="shrink-0 overflow-hidden" aria-labelledby="mapping-saved-maps-title">
    <CardHeader>
      <div>
        <h2 className="m-0 flex items-center gap-2 text-hmi-title font-bold text-ptagv-text" id="mapping-saved-maps-title">Saved Maps <Badge>{maps.length}</Badge></h2>
      </div>
      {!isStatic && <Badge tone="neutral">{isShowcase ? "BROWSER LIBRARY" : "THIS ROBOT"}</Badge>}
    </CardHeader>
    <div className="flex min-h-0 min-w-0 flex-col">
    {maps.length > 0 && <label className="grid gap-1 px-hmi-3 pt-hmi-2 text-xs font-bold text-ptagv-muted">
      FILTER SAVED MAPS
      <Input type="search" value={filter} onChange={(event) => setFilter(event.target.value)} placeholder="Name, map ID, or revision"
        aria-label="Filter saved maps" />
    </label>}
    <div className="grid max-h-40 min-h-15 gap-1.5 overflow-y-auto p-hmi-3 [scrollbar-width:thin]" role="listbox" aria-label="Saved maps">
      {maps.length === 0 ? <div className="p-hmi-3 text-sm text-ptagv-muted">No saved maps available.</div>
        : filteredMaps.length === 0 ? <div className="p-hmi-3 text-sm text-ptagv-muted" role="status">No saved maps match this filter.</div>
        : filteredMaps.map((map) => {
        const active = !isStatic && activeMapId === map.id;
        const isSelected = selected === map.id;
        return <button type="button" role="option" aria-selected={isSelected} aria-pressed={isSelected}
          className={`mapping-map-option flex min-h-12 min-w-0 items-center justify-between gap-2 border border-l-4 px-2 py-2 text-left focus-visible:outline-2 focus-visible:outline-ptagv-blue ${isSelected ? "is-selected border-blue-300 border-l-ptagv-blue bg-blue-50" : "border-ptagv-border border-l-transparent bg-slate-50 hover:border-blue-200"}`} key={map.id} onClick={() => { setConfirmingResume(false); onSelect(map.id); }}>
          <span className="grid min-w-0 gap-1">
            <b className="truncate text-sm text-ptagv-text">{displayFixtureMapName(isStatic, map.name)}</b>
            <small className="text-xs text-ptagv-muted">{isStatic ? STATIC_UI_EMPTY_VALUE : <>r{map.revision} <span aria-hidden="true">·</span> {map.resolution.toFixed(3)} m/cell</>}</small>
          </span>
          {!isStatic && <Badge className={`mapping-map-status shrink-0${active ? " is-active" : ""}`} tone={active ? "success" : "neutral"}>{isShowcase ? "SAMPLE" : active ? "ACTIVE" : "STORED"}</Badge>}
        </button>;
      })}
    </div>
    <div className="flex flex-wrap items-center justify-between gap-2 border-t border-ptagv-border-soft px-hmi-3 py-hmi-2">
      <Button size="compact" disabled={isStatic || !sessionAvailable || Boolean(resumeDisabledReason) || resumeBusy}
        aria-describedby={isStatic ? undefined : "saved-slam-resume-state"} onClick={() => setConfirmingResume(true)}>
        {resumeBusy ? "RESTORING SLAM SESSION..." : "RESUME SAVED SLAM SESSION"}
      </Button>
      {!isStatic && <Alert id="saved-slam-resume-state" className="min-w-0 flex-1 text-xs" tone={resumeStatusTone}
        role={resumeError ? "alert" : "status"} aria-live="polite">{resumeStatus}</Alert>}
    </div>
    </div>
    {confirmingResume && selectedMap && !isStatic && <div className="fixed inset-0 z-50 grid place-items-center bg-slate-950/45 p-hmi-3" role="presentation">
      <div ref={resumeDialogRef} role="dialog" aria-modal="true" aria-labelledby="confirm-slam-resume-title" tabIndex={-1}
        className="grid w-full max-w-lg gap-hmi-3 border border-ptagv-border bg-white p-hmi-4 shadow-xl">
        <h3 id="confirm-slam-resume-title" className="m-0 text-base font-bold text-ptagv-text">Restore saved SLAM session?</h3>
        <p className="m-0 text-sm leading-relaxed text-ptagv-muted">Restart the supervised Mapping runtime with <b className="text-ptagv-text">{selectedMap.name}</b> pose graph.</p>
        <p className="m-0 text-sm leading-relaxed text-ptagv-muted">The active map identity clears during restart. The session is restored only after the backend verifies the live map.</p>
        {resumeDisabledReason && <Alert tone="warning" role="note">{resumeDisabledReason}</Alert>}
        <div className="flex flex-wrap justify-end gap-2">
          <Button onClick={() => setConfirmingResume(false)}>CANCEL</Button>
          <Button tone="primary" disabled={Boolean(resumeDisabledReason) || resumeBusy}
            onClick={() => { setConfirmingResume(false); onResume(); }}>RESTORE SLAM SESSION</Button>
        </div>
      </div>
    </div>}
  </Card>;
}
