import type { FramePose, RobotDetailMapSnapshot, RobotDetailScan, RobotState, RobotWorldPoint } from "../../../schema/twin_state";
import { PosePickerMap } from "../../../components/shared/maps/LocalMapViews";
import { Badge, Card } from "../../../components/shared/ui/HmiPrimitives";
import type { MappingLayers } from "../mappingTypes";

type Props = {
  map: RobotDetailMapSnapshot | null;
  robot?: RobotState;
  fixtureRobotPose?: FramePose;
  scan: RobotDetailScan | null;
  pose?: FramePose;
  layers: MappingLayers;
  trajectory: RobotWorldPoint[];
  isSlamMap: boolean;
  isStatic: boolean;
  isShowcase?: boolean;
  loadedLocalMap: boolean;
  emptyMessage: string;
  mapSource: string;
  onToggleLayer: (layer: keyof MappingLayers) => void;
};

const layerNames: Array<keyof MappingLayers> = ["robot", "scan", "trajectory", "grid"];

export function MappingMapWorkspace({ map, robot, fixtureRobotPose, scan, pose, layers, trajectory, isSlamMap, isStatic, isShowcase = false, loadedLocalMap, emptyMessage, mapSource, onToggleLayer }: Props) {
  return <Card className="mapping-map-card min-h-0 flex-1 overflow-hidden" aria-label="Mapping map workspace">
    <div className="mapping-layer-toolbar flex min-h-9 shrink-0 flex-wrap items-center gap-1.5 border-b border-ptagv-border-soft bg-white px-hmi-2 py-1" role="group" aria-label="Mapping map layers">
      {layerNames.map((layer) => <button key={layer} type="button" aria-pressed={layers[layer]}
        className={`inline-flex min-h-7 items-center gap-1.5 border px-2 text-hmi-kicker font-bold focus-visible:outline-2 focus-visible:outline-ptagv-blue ${layers[layer] ? "border-blue-300 bg-blue-50 text-ptagv-blue" : "border-ptagv-border bg-slate-50 text-ptagv-muted hover:bg-slate-100"}`} onClick={() => onToggleLayer(layer)}>
        <span className={`size-2 border border-current ${layers[layer] ? "bg-current" : "bg-transparent"}`} aria-hidden="true" />{layer.toUpperCase()}
      </button>)}
      {!isStatic && <Badge tone={isShowcase ? "neutral" : map ? "info" : "neutral"} className="ml-auto max-w-[55%] truncate" title={mapSource}>{isShowcase ? "SAMPLE MAP" : loadedLocalMap ? "ACTIVE SAVED MAP" : "LIVE SLAM MAP"}</Badge>}
    </div>
    <div className="mapping-map-viewport relative flex min-h-0 min-w-0 flex-1 overflow-hidden bg-slate-600">
      {map ? <PosePickerMap map={map} robot={robot} fixtureRobotPose={fixtureRobotPose} scan={isSlamMap ? scan : null}
        showRobot={!isStatic && layers.robot} showScan={Boolean(!isStatic && isSlamMap && layers.scan && scan)} showGrid={layers.grid} showPose={false}
        trajectory={layers.trajectory && (isSlamMap || isStatic) ? trajectory : []}
        pose={{ x: pose?.x ?? 0, y: pose?.y ?? 0, yaw: pose?.yaw ?? 0 }} active={false} onPick={() => undefined}
        ariaLabel="Mapping occupancy map" />
        : <div className="grid flex-1 place-items-center bg-slate-600 p-hmi-6 text-center text-sm text-white" role="status">{emptyMessage}</div>}
    </div>
  </Card>;
}
