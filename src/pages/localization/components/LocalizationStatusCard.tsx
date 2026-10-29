import { Badge, Card, CardBody, CardHeader, MetricRow, SectionHeading } from "../../../components/shared/ui/HmiPrimitives";
import type { EStopState } from "../../../services/estopState";

export type LocalizationStateLabel = "LOCALIZED" | "INITIAL POSE REQUIRED" | "LOCALIZATION DEGRADED" | "UNAVAILABLE" | "UNKNOWN";

type Props = {
  state: LocalizationStateLabel;
  detail: string;
  sample: boolean;
  sampleData?: boolean;
  runtimeMode: string;
  mapId: string;
  mapRevision: string;
  requestState: string;
  estopState: EStopState;
};

const toneByState = {
  LOCALIZED: "success",
  "INITIAL POSE REQUIRED": "warning",
  "LOCALIZATION DEGRADED": "warning",
  UNAVAILABLE: "neutral",
  UNKNOWN: "neutral",
} as const;

export function LocalizationStatusCard({ state, detail, sample, sampleData = false, runtimeMode, mapId, mapRevision, requestState, estopState }: Props) {
  return <Card aria-labelledby="localization-status-heading">
    <CardHeader className="min-h-10"><SectionHeading id="localization-status-heading" className="text-hmi-kicker uppercase tracking-wider">LOCALIZATION STATUS</SectionHeading></CardHeader>
    <CardBody className="gap-hmi-1">
      <MetricRow density="compact" label="STATE" value={<Badge tone={sample ? "neutral" : toneByState[state]} data-testid="localization-state-badge">{sample ? "—" : state}</Badge>} />
      {!sample && <>
        <p className="m-0 text-sm leading-snug text-ptagv-text" data-testid="localization-state-detail">{detail}</p>
        {sampleData && <MetricRow density="compact" label="DATA SOURCE" value="LOCAL SAMPLE" />}
        <MetricRow density="compact" label="RUNTIME" value={runtimeMode} />
        <MetricRow density="compact" label="ACTIVE MAP" value={mapId} mono />
        <MetricRow density="compact" label="MAP REVISION" value={mapRevision} mono />
        <MetricRow density="compact" label="E-STOP" value={<Badge data-testid="localization-estop-state"
          tone={estopState === "ACTIVE" ? "error" : estopState === "CLEAR" ? "success" : "warning"}>{estopState}</Badge>} />
        <MetricRow density="compact" label="POSE REQUEST" value={requestState} />
      </>}
    </CardBody>
  </Card>;
}
