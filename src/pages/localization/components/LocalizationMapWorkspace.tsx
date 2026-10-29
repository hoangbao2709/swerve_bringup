import { useCallback, useState } from "react";
import type { RobotDetailMapSnapshot, RobotDetailScan, RobotState } from "../../../schema/twin_state";
import { PosePickerMap, type ActiveMapLayers, type MapViewportReadout } from "../../../components/shared/maps/LocalMapViews";
import { Card, CardBody, CardHeader, EmptyState } from "../../../components/shared/ui/HmiPrimitives";
import type { InitialPose } from "../localizationModel";

type Props = {
  robot?: RobotState;
  map: RobotDetailMapSnapshot | null;
  sample: boolean;
  waitingForSavedMap: boolean;
  scan: RobotDetailScan | null;
  pose: InitialPose;
  showPose: boolean;
  pickMode: boolean;
  selectionResetVersion: number;
  pointModeRequestVersion: number;
  onPickModeChange: (mode: "PAN" | "POINT") => void;
  onPickPosition: (position: Pick<InitialPose, "x" | "y">, defaultYaw?: number) => void;
  onConfirmHeading: (position: Pick<InitialPose, "x" | "y">, yaw: number) => void;
  onCancelSelection: () => void;
};

const initialLayers: ActiveMapLayers = { robot: true, scan: false, path: false, trajectory: false, grid: false };

function display(value: number, digits = 2): string {
  return Number.isFinite(value) ? value.toFixed(digits) : "—";
}

export function LocalizationMapWorkspace({ robot, map, sample, waitingForSavedMap, scan, pose, showPose, pickMode,
  selectionResetVersion, pointModeRequestVersion, onPickModeChange, onPickPosition, onConfirmHeading, onCancelSelection }: Props) {
  const [layers, setLayers] = useState<ActiveMapLayers>(initialLayers);
  const [viewport, setViewport] = useState<MapViewportReadout | null>(null);
  const updateViewport = useCallback((info: MapViewportReadout) => {
    setViewport((previous) => previous
      && previous.centerX === info.centerX && previous.centerY === info.centerY
      && previous.scalePxPerMeter === info.scalePxPerMeter && previous.width === info.width
      && previous.height === info.height ? previous : info);
  }, []);
  const mapRevision = map?.active_map_revision ?? map?.map_content_revision ?? map?.map_version ?? "UNKNOWN";
  const mapSource = map?.map_source ?? "UNKNOWN";
  const mapIdentity = sample ? "—" : map?.active_map_id ? `${map.active_map_id} · r${mapRevision}` : "UNKNOWN";
  const scaleRatio = viewport && viewport.fitScalePxPerMeter > 0
    ? viewport.scalePxPerMeter / viewport.fitScalePxPerMeter : Number.NaN;
  const setLayer = (key: keyof ActiveMapLayers) => setLayers((previous) => ({ ...previous, [key]: !previous[key] }));

  return <Card className="localization-map-card flex h-full min-h-[46vh] min-w-0 flex-col overflow-hidden" aria-label="Localization map workspace">
    <CardHeader className="min-h-9">
      {!sample && <span className="min-w-0 break-all font-mono text-hmi-kicker text-ptagv-muted" data-testid="localization-map-identity">{mapIdentity}</span>}
      <span className="shrink-0 font-mono text-hmi-kicker font-bold text-ptagv-muted">{map?.frame_id ?? "FRAME UNKNOWN"}</span>
    </CardHeader>
    <CardBody className="localization-map-canvas flex min-h-0 min-w-0 flex-1 gap-0 p-0">
      {map ? <>
        <div className="relative flex min-h-0 min-w-0 flex-1 overflow-hidden">
          <PosePickerMap map={map} robot={sample ? undefined : robot} fixtureRobotPose={sample ? undefined : robot?.active_map_pose ?? undefined}
            scan={sample ? null : scan} trajectory={!sample && layers.trajectory ? scan?.trajectory ?? [] : []}
            showRobot={!sample && layers.robot} showScan={!sample && layers.scan} showGrid={layers.grid}
            showPose={showPose} layerState={layers} onLayerToggle={setLayer}
            pose={pose} defaultPickYaw={Number.isFinite(pose.yaw) ? pose.yaw : 0}
            active={pickMode} onPick={onPickPosition} onConfirmPoint={onConfirmHeading}
            onInteractionModeChange={onPickModeChange}
            pointSelectionResetVersion={selectionResetVersion} pointModeRequestVersion={pointModeRequestVersion}
            onCancelPointSelection={onCancelSelection}
            compactToolbar toolbarPlacement="workspace" pointModeLabel="SET POSE"
            toolbarLabel="Localization map tools" initialInteractionMode="PAN"
            onViewportInfo={updateViewport}
            pickInstruction={pickMode ? "CLICK POSITION · MOVE TO HEADING · CLICK TO CONFIRM" : "PAN MAP · SET POSE TO SELECT"}
            ariaLabel="Localization map and initial pose picker" />
        </div>
        <div className="grid shrink-0 grid-cols-2 gap-x-hmi-3 gap-y-1 border-t border-ptagv-border-soft bg-white px-hmi-3 py-hmi-2 text-hmi-kicker text-ptagv-muted sm:grid-cols-3 xl:grid-cols-5" data-testid="localization-map-metrics">
          <span><b className="text-ptagv-text">MAP</b> {map.width} × {map.height}</span>
          <span><b className="text-ptagv-text">RES</b> {display(map.resolution, 3)} m/cell</span>
          <span><b className="text-ptagv-text">SCALE</b> {Number.isFinite(scaleRatio) ? `${Math.round(scaleRatio * 100)}%` : "FIT"}</span>
          {!sample && <span><b className="text-ptagv-text">SOURCE</b> {mapSource}</span>}
          {!sample && <span className="min-w-0 break-all" title={mapIdentity}><b className="text-ptagv-text">REV</b> {mapRevision}</span>}
        </div>
      </> : <EmptyState className="m-hmi-3 flex-1" data-testid="localization-map-waiting">
        {sample ? "—" : waitingForSavedMap ? "WAITING FOR ACTIVE SAVED MAP SNAPSHOT..."
          : mapSource === "SLAM_TOOLBOX" ? "WAITING FOR A CURRENT MAP SNAPSHOT" : "WAITING FOR THE ACTIVE ROBOT MAP SNAPSHOT"}
      </EmptyState>}
    </CardBody>
  </Card>;
}
