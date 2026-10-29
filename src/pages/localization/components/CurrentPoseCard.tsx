import { Card, CardBody, CardHeader, EmptyState, MetricRow, SectionHeading } from "../../../components/shared/ui/HmiPrimitives";
import type { InitialPose } from "../localizationModel";
import { STATIC_UI_EMPTY_VALUE } from "../../../mock/staticPresentation";

export type CurrentPoseData = InitialPose & {
  frameId: string;
  mapId: string;
  mapRevision: string;
  source: string;
  timestamp: string;
  ageSeconds: number;
};

function coordinate(value: number, suffix: string) {
  return Number.isFinite(value) ? `${(Math.abs(value) < 0.0005 ? 0 : value).toFixed(3)}${suffix}` : "UNKNOWN";
}

export function CurrentPoseCard({ pose, sample, sampleData = false }: { pose: CurrentPoseData | null; sample: boolean; sampleData?: boolean }) {
  return <Card aria-labelledby="localization-current-pose-heading">
    <CardHeader className="min-h-10"><SectionHeading id="localization-current-pose-heading" className="text-hmi-kicker uppercase tracking-wider">CURRENT ROBOT POSE</SectionHeading></CardHeader>
    <CardBody className="gap-hmi-1">
      {sample ? <>
        <MetricRow density="compact" label="X · MAP" value={STATIC_UI_EMPTY_VALUE} mono />
        <MetricRow density="compact" label="Y · MAP" value={STATIC_UI_EMPTY_VALUE} mono />
        <MetricRow density="compact" label="YAW" value={STATIC_UI_EMPTY_VALUE} mono />
      </> : pose ? <>
        <MetricRow density="compact" label="X · MAP" value={coordinate(pose.x, " m")} mono />
        <MetricRow density="compact" label="Y · MAP" value={coordinate(pose.y, " m")} mono />
        <MetricRow density="compact" label="YAW" value={coordinate(pose.yaw, " rad")} mono />
        <MetricRow density="compact" label="FRAME" value={pose.frameId} mono />
        <MetricRow density="compact" label="POSE SOURCE" value={pose.source} />
        <MetricRow density="compact" label="LAST UPDATE" value={new Date(pose.timestamp).toLocaleTimeString()} mono />
        <MetricRow density="compact" label="POSE AGE" value={`${pose.ageSeconds.toFixed(1)} s`} mono />
        <MetricRow density="compact" label="MAP REVISION" value={pose.mapRevision} mono />
        {sampleData && <MetricRow density="compact" label="DATA SOURCE" value="LOCAL SAMPLE" />}
      </> : <EmptyState className="min-h-16" data-testid="localization-current-pose-unavailable">FRESH MAP-MATCHED ROBOT POSE UNAVAILABLE</EmptyState>}
    </CardBody>
  </Card>;
}
