import type { RobotMappingDiagnostics, RobotSystemDiagnostics } from "../../../schema/twin_state";
import { Card, CardBody, CardHeader, MetricRow, SectionHeading } from "../../../components/shared/ui/HmiPrimitives";
import { LocalizationEventHistory } from "./LocalizationEventHistory";
import type { LocalizationEvent } from "../hooks/useLocalizationEventHistory";
import { displayRuntimeValue } from "../../../mock/staticPresentation";

function boolStatus(value: boolean | undefined, staticUi: boolean): string {
  if (staticUi) return displayRuntimeValue(true, "UNKNOWN");
  if (value === true) return "AVAILABLE";
  if (value === false) return "NOT LIVE";
  return displayRuntimeValue(staticUi, "UNKNOWN");
}

function validity(value: boolean | undefined, staticUi: boolean): string {
  if (staticUi) return displayRuntimeValue(true, "UNKNOWN");
  if (value === true) return "VALID";
  if (value === false) return "INVALID";
  return displayRuntimeValue(staticUi, "UNKNOWN");
}

function rate(value: number | null | undefined): string {
  return Number.isFinite(value) ? `${Number(value).toFixed(1)} Hz` : "NOT AVAILABLE";
}

function age(value: number | null | undefined): string {
  return Number.isFinite(value) && Number(value) >= 0 ? `${Number(value).toFixed(2)} s` : "NOT AVAILABLE";
}

type Props = {
  diagnostics: RobotSystemDiagnostics | null;
  mapping: RobotMappingDiagnostics | undefined;
  staticUi: boolean;
  nav2Ready: boolean | undefined;
  localizationOwner: string;
  poseSource: string;
  poseAge: number | null;
  events: LocalizationEvent[];
  onClearHistory: () => void;
};

export function TfDiagnosticsPanel({ diagnostics, mapping, staticUi, nav2Ready, localizationOwner,
  poseSource, poseAge, events, onClearHistory }: Props) {
  return <Card aria-labelledby="localization-diagnostics-heading">
    <CardHeader className="min-h-10"><SectionHeading id="localization-diagnostics-heading" className="text-hmi-kicker uppercase tracking-wider">TF & SENSOR HEALTH</SectionHeading></CardHeader>
    <CardBody className="gap-hmi-2">
      <MetricRow density="compact" label="TF TREE" value={validity(diagnostics?.tf, staticUi)} />
      <MetricRow density="compact" label="LIDAR" value={boolStatus(mapping?.scan_live ?? diagnostics?.lidar, staticUi)} />
      <MetricRow density="compact" label="ODOMETRY" value={boolStatus(mapping?.odom_live, staticUi)} />
      <MetricRow density="compact" label="NAV2 LOCALIZATION" value={displayRuntimeValue(staticUi, nav2Ready === true ? "READY" : nav2Ready === false ? "NOT READY" : "UNKNOWN")} />
      <details className="border border-ptagv-border-soft bg-white" data-testid="localization-advanced-diagnostics">
        <summary className="cursor-pointer px-hmi-3 py-hmi-2 text-hmi-kicker font-bold tracking-wide text-ptagv-text">E · ADVANCED DIAGNOSTICS</summary>
        <div className="space-y-hmi-3 border-t border-ptagv-border-soft p-hmi-3">
        <div>
          <h3 className="mb-hmi-1 mt-0 text-hmi-kicker font-bold text-ptagv-muted">TRANSFORMS & OWNERSHIP</h3>
          <MetricRow density="compact" label="MAP → ODOM OWNER" value={displayRuntimeValue(staticUi, localizationOwner)} />
          <MetricRow density="compact" label="MAP → ODOM EDGE" value={displayRuntimeValue(staticUi, "NOT REPORTED")} />
          <MetricRow density="compact" label="ODOM FRAME" value={displayRuntimeValue(staticUi, mapping?.odom_frame || "UNKNOWN")} mono />
          <MetricRow density="compact" label="BASE FRAME" value={displayRuntimeValue(staticUi, mapping?.base_frame || "UNKNOWN")} mono />
          <MetricRow density="compact" label="ODOM → BASE EDGE" value={displayRuntimeValue(staticUi, "FRAME IDS REPORTED; EDGE VALIDITY NOT REPORTED")} />
          <MetricRow density="compact" label="LIDAR → MAP TF" value={validity(mapping?.tf_lidar_to_map_valid, staticUi)} />
          {!staticUi && mapping?.tf_error && <p className="m-0 break-words py-hmi-1 text-xs text-ptagv-warning" role="status">TF DIAGNOSTIC: {mapping.tf_error}</p>}
        </div>
        <div>
          <h3 className="mb-hmi-1 mt-0 text-hmi-kicker font-bold text-ptagv-muted">SENSOR & NAVIGATION READINESS</h3>
          <MetricRow density="compact" label="LIDAR RATE / AGE" value={displayRuntimeValue(staticUi, `${rate(mapping?.scan_hz)} / ${age(mapping?.scan_age_s)}`)} />
          <MetricRow density="compact" label="ODOM RATE" value={displayRuntimeValue(staticUi, rate(mapping?.odom_hz))} />
          {!staticUi && <MetricRow density="compact" label="POSE SOURCE / AGE" value={`${poseSource} / ${poseAge === null ? "NOT AVAILABLE" : `${poseAge.toFixed(2)} s`}`} />}
          <MetricRow density="compact" label="POSE COVARIANCE / QUALITY" value={displayRuntimeValue(staticUi, "NOT AVAILABLE FROM CURRENT CONTRACT")} />
        </div>
          <LocalizationEventHistory events={events} onClear={onClearHistory} />
      </div>
      </details>
    </CardBody>
  </Card>;
}
