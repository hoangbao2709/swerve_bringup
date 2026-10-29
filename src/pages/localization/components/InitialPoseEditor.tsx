import { Badge, Button, Card, CardBody, CardHeader, Input, SectionHeading } from "../../../components/shared/ui/HmiPrimitives";
import type { AngleUnit, InitialPose } from "../localizationModel";
import { displayYawToRadians, radiansToDisplayYaw } from "../localizationModel";

type Props = {
  pose: InitialPose;
  hasDraft: boolean;
  mapId: string;
  mapRevision: string;
  angleUnit: AngleUnit;
  validationMessage: string;
  blocker: string;
  requestState: string;
  requestPending: boolean;
  applyDisabled: boolean;
  canResetToCurrentPose: boolean;
  resetPoseLabel: string;
  onAngleUnitChange: (unit: AngleUnit) => void;
  onChange: (key: keyof InitialPose, value: number) => void;
  onRotate: (deltaRadians: number) => void;
  onResetToCurrentPose: () => void;
  onClear: () => void;
  onPickAgain: () => void;
  onOpenConfirmation: () => void;
};

const fieldLabel = "grid min-w-0 gap-1 text-xs font-bold tracking-wide text-ptagv-muted";
function fieldValue(value: number, unit?: AngleUnit): string {
  if (!Number.isFinite(value)) return "";
  const displayed = unit ? radiansToDisplayYaw(value, unit) : value;
  return (Math.abs(displayed) < 0.0005 ? 0 : displayed).toFixed(3);
}

export function InitialPoseEditor({ pose, hasDraft, mapId, mapRevision, angleUnit, validationMessage, blocker,
  requestState, requestPending, applyDisabled, canResetToCurrentPose, resetPoseLabel, onAngleUnitChange, onChange, onRotate,
  onResetToCurrentPose, onClear, onPickAgain, onOpenConfirmation }: Props) {
  const disabled = Boolean(validationMessage || blocker || requestPending || applyDisabled);
  return <Card aria-labelledby="localization-initial-pose-heading">
    <CardHeader className="min-h-10"><SectionHeading id="localization-initial-pose-heading" className="text-hmi-kicker uppercase tracking-wider">INITIAL POSE EDITOR</SectionHeading>
      <Badge tone={requestPending ? "info" : hasDraft ? "neutral" : "warning"}>{requestPending ? "REQUEST PENDING" : hasDraft ? "DRAFT" : "NO DRAFT"}</Badge>
    </CardHeader>
    <CardBody className="gap-hmi-2">
      <div className="grid min-w-0 grid-cols-2 gap-hmi-2">
        <label className={fieldLabel}><span>X · MAP (m)</span>
          <Input aria-label="Initial pose X in meters" type="number" step="0.01" value={fieldValue(pose.x)} disabled={requestPending}
            onChange={(event) => onChange("x", event.target.value === "" ? Number.NaN : Number(event.target.value))} />
        </label>
        <label className={fieldLabel}><span>Y · MAP (m)</span>
          <Input aria-label="Initial pose Y in meters" type="number" step="0.01" value={fieldValue(pose.y)} disabled={requestPending}
            onChange={(event) => onChange("y", event.target.value === "" ? Number.NaN : Number(event.target.value))} />
        </label>
        <label className={fieldLabel}><span>YAW ({angleUnit === "RAD" ? "rad" : "deg"})</span>
          <Input aria-label={`Initial pose yaw in ${angleUnit === "RAD" ? "radians" : "degrees"}`} type="number"
            min={angleUnit === "RAD" ? -Math.PI : -180} max={angleUnit === "RAD" ? Math.PI : 180}
            step={angleUnit === "RAD" ? 0.01 : 1} value={fieldValue(pose.yaw, angleUnit)} disabled={requestPending}
            onChange={(event) => onChange("yaw", event.target.value === "" ? Number.NaN
              : displayYawToRadians(Number(event.target.value), angleUnit))} />
        </label>
        <div className="grid grid-cols-2 gap-hmi-1 self-end" role="group" aria-label="Yaw display units">
          <Button size="compact" aria-pressed={angleUnit === "RAD"} className={angleUnit === "RAD" ? "border-blue-300 bg-blue-50 text-ptagv-blue" : ""}
            disabled={requestPending} onClick={() => onAngleUnitChange("RAD")}>RAD</Button>
          <Button size="compact" aria-pressed={angleUnit === "DEG"} className={angleUnit === "DEG" ? "border-blue-300 bg-blue-50 text-ptagv-blue" : ""}
            disabled={requestPending} onClick={() => onAngleUnitChange("DEG")}>DEG</Button>
        </div>
      </div>
      <div className="grid grid-cols-4 gap-hmi-1" role="group" aria-label="Rotate initial pose">
        {([[-90, -Math.PI / 2], [-15, -Math.PI / 12], [15, Math.PI / 12], [90, Math.PI / 2]] as const).map(([degrees, radians]) =>
          <Button key={degrees} size="compact" disabled={!Number.isFinite(pose.yaw) || requestPending}
            onClick={() => onRotate(radians)}>{degrees > 0 ? `+${degrees}°` : `${degrees}°`}</Button>)}
      </div>
      <div className="grid min-w-0 grid-cols-2 gap-x-hmi-3 border-y border-ptagv-border-soft px-hmi-2">
        <div className="py-hmi-1 text-hmi-kicker text-ptagv-muted">FRAME ID <b className="ml-2 font-mono text-ptagv-text">map</b></div>
        <div className="min-w-0 break-all py-hmi-1 text-right text-hmi-kicker text-ptagv-muted">MAP ID <b className="ml-2 font-mono text-ptagv-text">{mapId}</b></div>
        <div className="col-span-2 min-w-0 break-all border-t border-ptagv-border-soft py-hmi-1 text-hmi-kicker text-ptagv-muted">MAP REVISION <b className="ml-2 font-mono text-ptagv-text">{mapRevision}</b></div>
      </div>
      <div className="flex flex-wrap gap-hmi-1">
        <Button size="compact" disabled={!canResetToCurrentPose || requestPending} onClick={onResetToCurrentPose}>{resetPoseLabel}</Button>
        <Button size="compact" disabled={!hasDraft || requestPending} onClick={onClear}>CLEAR DRAFT</Button>
        <Button size="compact" disabled={requestPending} onClick={onPickAgain}>SELECT ON MAP</Button>
      </div>
      {validationMessage && <div className="border border-amber-200 bg-amber-50 px-hmi-2 py-hmi-1 text-xs leading-snug text-ptagv-warning" role="status" data-testid="initial-pose-validation">{validationMessage}</div>}
      {blocker && <div className="border border-ptagv-border-soft bg-slate-50 px-hmi-2 py-hmi-1 text-xs leading-snug text-ptagv-muted" role="status" data-testid="initial-pose-blocker">{blocker}</div>}
      <div className="flex flex-wrap items-center justify-between gap-hmi-2 border-t border-ptagv-border-soft pt-hmi-2">
        <span className="text-hmi-kicker font-semibold text-ptagv-muted" data-testid="initial-pose-request-state">{requestState}</span>
        <Button tone="primary" disabled={disabled} onClick={onOpenConfirmation} data-testid="set-initial-pose">
          {requestPending ? "SENDING…" : "SET INITIAL POSE"}
        </Button>
      </div>
    </CardBody>
  </Card>;
}
