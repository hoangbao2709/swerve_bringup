import type { RobotDetailMapSnapshot, RobotRuntimeCapabilities, RobotSystemDiagnostics, RobotWorldPoint } from "../../../schema/twin_state";
import { MetricRow } from "../../../components/shared/ui/HmiPrimitives";
import { valueNumber, valueText } from "../../../components/shared/ui/RobotPagePrimitives";
import { displayRuntimeValue } from "../../../mock/staticPresentation";
import type { LocalRobotMap } from "../../../services/api";

type Props = {
  isStatic: boolean;
  runtimeState: string;
  controlMode: "MANUAL" | "AUTONOMOUS";
  capabilities: RobotRuntimeCapabilities | null;
  diagnostics: RobotSystemDiagnostics | null;
  slamMap: RobotDetailMapSnapshot | null;
  runtimeMap: RobotDetailMapSnapshot | null;
  localMapSyncStatus: string | null;
  mappingState: string;
  trajectory: RobotWorldPoint[];
  maps: LocalRobotMap[];
  operationState: string;
  canonicalRevision: string | number | null;
  robotPoseSummary: string;
  mapOrigin: string;
  activeMapValue: string;
};

export function MappingAdvancedDetails({ isStatic, runtimeState, controlMode, capabilities, diagnostics, slamMap, runtimeMap, localMapSyncStatus, mappingState, trajectory, maps, operationState, canonicalRevision, robotPoseSummary, mapOrigin, activeMapValue }: Props) {
  const mapping = diagnostics?.mapping;
  const nav2State = capabilities?.nav2_ready === true && diagnostics?.nav2_ready === true ? "READY"
    : diagnostics?.nav2 ? "STARTING" : runtimeState === "UNIFIED" ? "UNAVAILABLE" : "INACTIVE";
  return <details className="mapping-advanced-details hmi-advanced-details min-w-0 border border-ptagv-border bg-white text-ptagv-text">
    <summary className="cursor-pointer px-hmi-3 py-hmi-2 text-xs font-bold tracking-wide text-ptagv-muted">ADVANCED DIAGNOSTICS</summary>
    <div className="grid max-h-[34vh] min-w-0 grid-cols-1 gap-x-hmi-4 overflow-auto border-t border-ptagv-border-soft px-hmi-3 py-hmi-2 md:grid-cols-2 [scrollbar-width:thin]">
      <MetricRow label="ROBOT SOURCE" value={displayRuntimeValue(isStatic, "LOCAL ROBOT")} />
      <MetricRow label="RUNTIME MODE" value={displayRuntimeValue(isStatic, runtimeState)} />
      <MetricRow label="MAPPING STATE" value={displayRuntimeValue(isStatic, mappingState)} />
      <MetricRow label="SLAM TOOLBOX STATE" value={displayRuntimeValue(isStatic, mapping?.slam_state ?? "UNKNOWN")} />
      <MetricRow label="CONTROL MODE" value={displayRuntimeValue(isStatic, controlMode)} />
      <MetricRow label="NAV2 READINESS" value={displayRuntimeValue(isStatic, nav2State)} />
      <MetricRow label="TF VALIDITY" value={displayRuntimeValue(isStatic, mapping?.tf_lidar_to_map_valid ? "VALID" : mapping?.tf_error ?? "WAITING / INVALID")} />
      <MetricRow label="LIDAR SCAN RATE" value={displayRuntimeValue(isStatic, mapping?.scan_live ? `${valueNumber(mapping.scan_hz, 2, " Hz")} · ${mapping.scan_frame ?? "frame unknown"}` : "WAITING")} />
      <MetricRow label="ODOMETRY" value={displayRuntimeValue(isStatic, mapping?.odom_live ? `${valueNumber(mapping.odom_hz, 2, " Hz")} · ${mapping.odom_frame ?? "?"} → ${mapping.base_frame ?? "?"}` : "WAITING")} />
      <MetricRow label="ACCUMULATED /map" value={displayRuntimeValue(isStatic, runtimeMap?.map_source === "LOCAL_MAP" ? "ACTIVE · SAVED LOCAL MAP" : mapping?.map_live && slamMap ? "LIVE · SLAM TOOLBOX" : "WAITING FOR SLAM MAP")} />
      <MetricRow label="MAP UPDATE RATE" value={displayRuntimeValue(isStatic, mapping?.map_live ? `${valueNumber(mapping.map_hz, 2, " Hz")} · v${slamMap?.map_version ?? mapping.map_version ?? "—"}` : "WAITING")} />
      <MetricRow label="MAP / ODOM OWNER" value={displayRuntimeValue(isStatic, localMapSyncStatus ?? mapping?.map_odom_owner ?? "UNVERIFIED")} />
      <MetricRow label="MAP SYNCHRONIZATION" value={displayRuntimeValue(isStatic, localMapSyncStatus ?? "UNVERIFIED")} />
      <MetricRow label="WORKFLOW STATE" value={displayRuntimeValue(isStatic, operationState)} mono />
      <MetricRow label="MAP ORIGIN" value={displayRuntimeValue(isStatic, mapOrigin)} mono />
      <MetricRow label="CANONICAL REVISION" value={displayRuntimeValue(isStatic, valueText(canonicalRevision))} mono />
      <MetricRow label="UNKNOWN / FREE / OCCUPIED" value={displayRuntimeValue(isStatic, `${slamMap?.unknown_cells ?? "—"} / ${slamMap?.free_cells ?? "—"} / ${slamMap?.occupied_cells ?? "—"}`)} mono />
      <MetricRow label="KNOWN CELLS" value={displayRuntimeValue(isStatic, valueText(slamMap?.known_cells))} mono />
      <MetricRow label="ROBOT POSE · map → base" value={displayRuntimeValue(isStatic, robotPoseSummary)} mono />
      <MetricRow label="ACTIVE MAP ID / REVISION" value={displayRuntimeValue(isStatic, activeMapValue)} mono />
      <MetricRow label="ACTIVE LOCAL MAP SYNC" value={displayRuntimeValue(isStatic, localMapSyncStatus ?? "UNVERIFIED")} />
      <MetricRow label="TRAJECTORY SAMPLES" value={displayRuntimeValue(isStatic, trajectory.length)} mono />
      <MetricRow label="AVAILABLE MAPS" value={displayRuntimeValue(isStatic, maps.length)} mono />
    </div>
    <p className="m-0 px-hmi-3 pb-hmi-3 text-xs leading-relaxed text-ptagv-muted">Loading a saved map uses the supervised SLAM-to-Navigation handoff. Saved SLAM pose-graph restoration remains a separate operation.</p>
  </details>;
}
