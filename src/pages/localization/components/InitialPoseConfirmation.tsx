import { useEffect, useRef } from "react";
import { Button, MetricRow } from "../../../components/shared/ui/HmiPrimitives";
import type { InitialPose } from "../localizationModel";

type Props = {
  open: boolean;
  pose: InitialPose;
  mapId: string;
  mapRevision: string;
  pending: boolean;
  error: string;
  showcase?: boolean;
  onCancel: () => void;
  onConfirm: () => void;
};

export function InitialPoseConfirmation({ open, pose, mapId, mapRevision, pending, error, showcase = false, onCancel, onConfirm }: Props) {
  const dialogRef = useRef<HTMLDivElement>(null);
  useEffect(() => {
    if (!open) return;
    dialogRef.current?.focus();
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key === "Escape" && !pending) onCancel();
    };
    window.addEventListener("keydown", onKeyDown);
    return () => window.removeEventListener("keydown", onKeyDown);
  }, [open, onCancel, pending]);
  if (!open) return null;
  return <div className="fixed inset-0 z-[100] grid place-items-center bg-slate-950/45 p-hmi-3" data-testid="initial-pose-confirmation-backdrop"
    onMouseDown={(event) => { if (event.target === event.currentTarget && !pending) onCancel(); }}>
    <div ref={dialogRef} tabIndex={-1} className="w-full max-w-lg border border-ptagv-border bg-white p-hmi-4 shadow-xl focus:outline-none" role="dialog" aria-modal="true" aria-labelledby="initial-pose-confirmation-title" data-testid="initial-pose-confirmation"
      onKeyDown={(event) => {
        if (event.key === "Escape" && !pending) { event.preventDefault(); onCancel(); return; }
        if (event.key !== "Tab") return;
        const focusable = Array.from(event.currentTarget.querySelectorAll<HTMLElement>('button:not(:disabled), [href], input:not(:disabled), select:not(:disabled), textarea:not(:disabled), [tabindex]:not([tabindex="-1"])'));
        if (!focusable.length) return;
        const first = focusable[0], last = focusable[focusable.length - 1];
        if (event.shiftKey && document.activeElement === first) { event.preventDefault(); last.focus(); }
        else if (!event.shiftKey && document.activeElement === last) { event.preventDefault(); first.focus(); }
      }}>
      <h2 id="initial-pose-confirmation-title" className="m-0 text-base font-bold tracking-wide text-ptagv-text">CONFIRM INITIAL POSE REQUEST</h2>
      <p className="mb-hmi-2 mt-hmi-2 text-sm leading-relaxed text-ptagv-muted">{showcase
        ? "This records the selected pose only in the current view. No localization request is sent."
        : "This submits a map-frame pose through the configured localization interface. It does not move the robot. The backend validates robot mode, stopped state and current map identity."}</p>
      <div className="border border-ptagv-border-soft bg-slate-50 px-hmi-3">
        <MetricRow label="X · MAP" value={`${pose.x.toFixed(3)} m`} mono />
        <MetricRow label="Y · MAP" value={`${pose.y.toFixed(3)} m`} mono />
        <MetricRow label="YAW" value={`${pose.yaw.toFixed(4)} rad`} mono />
        <MetricRow label="FRAME" value="map" mono />
        <MetricRow label="MAP ID" value={mapId} mono />
        <MetricRow label="MAP REVISION" value={mapRevision} mono />
      </div>
      {error && <div className="mt-hmi-2 border border-red-300 bg-red-50 px-hmi-2 py-hmi-2 text-sm leading-snug text-ptagv-error" role="alert">{error}</div>}
      <div className="mt-hmi-3 flex flex-wrap justify-end gap-hmi-2">
        <Button disabled={pending} onClick={onCancel}>CANCEL</Button>
        <Button tone="primary" disabled={pending} onClick={onConfirm}>{pending ? "SENDING REQUEST…" : showcase ? "CONFIRM LOCAL SELECTION" : "CONFIRM SET POSE"}</Button>
      </div>
    </div>
  </div>;
}
