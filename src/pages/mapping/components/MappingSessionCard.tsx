type Props = {
  status: string;
  statusTone: "ready" | "warning" | "error" | "preview";
  isStatic: boolean;
  primaryLabel: string;
  primaryDisabled: boolean;
  pauseDisabled: boolean;
  primaryReason: string;
  pauseReason: string;
  operationState: string;
  busy: boolean;
  error: string;
  onStartOrResume: () => void;
  onPause: () => void;
};

export function MappingSessionCard({ status, statusTone, isStatic, primaryLabel, primaryDisabled, pauseDisabled, primaryReason, pauseReason, operationState, busy, error, onStartOrResume, onPause }: Props) {
  return <Card className="shrink-0 overflow-hidden" aria-labelledby="mapping-session-title">
    <CardHeader>
      <div>
        <h2 className="m-0 text-hmi-title font-bold text-ptagv-text" id="mapping-session-title">Mapping Session</h2>
      </div>
      {!isStatic && <Badge tone={statusTone === "ready" ? "success" : statusTone === "error" ? "error" : "warning"} role="status">{status}</Badge>}
    </CardHeader>
    <CardBody>
      {busy && <LoadingState>{operationState}</LoadingState>}
      <div className="grid grid-cols-1 gap-hmi-2 sm:grid-cols-[1.12fr_.88fr]">
        <div className="min-w-0">
          <Button className="w-full" tone="primary" disabled={primaryDisabled} onClick={onStartOrResume}>{primaryLabel}</Button>
          {primaryDisabled && primaryReason && !isStatic && <small className="mt-1 block text-xs leading-snug text-ptagv-warning">{primaryReason}</small>}
        </div>
        <div className="min-w-0">
          <Button className="w-full" disabled={pauseDisabled} onClick={onPause}>PAUSE MAPPING</Button>
          {pauseDisabled && pauseReason && !isStatic && <small className="mt-1 block text-xs leading-snug text-ptagv-warning">{pauseReason}</small>}
        </div>
      </div>
      {error && <Alert tone="error" role="alert">{error}</Alert>}
      {!isStatic && <p className="m-0 text-xs leading-relaxed text-ptagv-muted">Mapping requires an online robot in Manual mode with SLAM ready. Pause the session before saving.</p>}
    </CardBody>
  </Card>;
}
import { Alert, Badge, Button, Card, CardBody, CardHeader, LoadingState } from "../../../components/shared/ui/HmiPrimitives";
