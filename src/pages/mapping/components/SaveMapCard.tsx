type Props = {
  name: string;
  onNameChange: (name: string) => void;
  disabled: boolean;
  disabledReason: string;
  busy: boolean;
  operationState: string;
  error: string;
  notice: string;
  isShowcase?: boolean;
  onSave: () => void;
};

export function SaveMapCard({ name, onNameChange, disabled, disabledReason, busy, operationState, error, notice, isShowcase = false, onSave }: Props) {
  return <Card className="shrink-0 overflow-hidden" aria-labelledby="mapping-save-title">
    <CardHeader>
      <div>
        <h2 className="m-0 text-hmi-title font-bold text-ptagv-text" id="mapping-save-title">Save Map</h2>
      </div>
      {busy && operationState === "SAVING" && <Badge tone="info" role="status">SAVING</Badge>}
    </CardHeader>
    <CardBody>
      <label className="grid min-w-0 gap-1.5 text-xs font-bold text-ptagv-muted">
        <span>MAP NAME</span>
        <Input value={name} maxLength={64} onChange={(event) => onNameChange(event.target.value)} placeholder="warehouse_floor_1" />
      </label>
      {isShowcase && <p className="m-0 text-xs text-ptagv-muted">Saves a map copy in this browser. SLAM is not running.</p>}
      <Button className="mapping-save-action w-full sm:self-end sm:px-hmi-6" tone="primary" disabled={disabled} onClick={onSave}>SAVE MAP</Button>
      {disabled && disabledReason && <p className="m-0 text-xs leading-snug text-ptagv-warning" role="status">{disabledReason}</p>}
      {error && <Alert tone="error" role="alert">{error}</Alert>}
      {notice && <Alert tone="success" role="status">{notice}</Alert>}
    </CardBody>
  </Card>;
}
import { Alert, Badge, Button, Card, CardBody, CardHeader, Input } from "../../../components/shared/ui/HmiPrimitives";
