import { useEffect, useId, useLayoutEffect, useMemo, useRef, useState, type MouseEvent, type PointerEvent as ReactPointerEvent } from "react";
import type { FramePose, RobotDetailMapSnapshot, RobotDetailScan, RobotState, RobotWorldPoint } from "../../../schema/twin_state";
import { createWorldTransform, worldToScreen, screenToWorld, type WorldBounds } from "../../../layout/coordinates";
import { centerMapViewportCamera, fitMapViewportCamera, fixedWorldTransform, MAP_PAN_THRESHOLD_PX, mapViewportSessionKey, occupancyMapWorldBounds, panMapViewportCamera, resolveMapViewport, zoomMapViewportCamera, zoomMapViewportCameraAt, type MapViewportState } from "../../../layout/mapViewport";
import { occupancyRasterKey, occupancyRasters } from "./occupancyRaster";
import { classifyOccupancyPoint, decodeOccupancyGrid, isFreeOccupancyPoint } from "./occupancyGrid";
import { displayedFramePose, useStableDisplayedFramePose, type MapPoseIdentity } from "../../../layout/robotPoseFrame";
import { mapPointTarget, type MapPointNavigationTarget, type NavigationMapIdentity } from "./navigationMapIdentity";
import { valueNumber } from "../ui/RobotPagePrimitives";
import { Button } from "../ui/HmiPrimitives";

type Pose = { x: number; y: number; yaw: number };

export function AccumulatedSlamMap2DView({ map, robot, scan }: {
  map: RobotDetailMapSnapshot | null;
  robot?: RobotState;
  scan: RobotDetailScan | null;
}) {
  if (!map || map.map_source !== "SLAM_TOOLBOX" || map.frame_id !== "map"
      || !map.mapping_session_id || map.active_map_id !== `SLAM-${map.mapping_session_id}`) {
    return <div className="grid min-h-24 place-items-center p-hmi-3 text-center text-sm text-ptagv-muted" data-testid="slam-map-2d-empty">Waiting for a current SLAM Toolbox /map.</div>;
  }
  const pose = robot?.slam_pose;
  const currentScan = scan?.mapping_session_id === map.mapping_session_id
    && scan.frame_id === map.frame_id ? scan : null;
  return <div className="flex h-full min-h-0 min-w-0 w-full flex-1 flex-col overflow-hidden" data-testid="slam-map-2d">
    <div data-testid="slam-map-2d-metrics" data-map-source="SLAM_TOOLBOX"
      data-map-frame={map.frame_id} data-mapping-session-id={map.mapping_session_id}
      data-map-version={map.map_version} data-known-cells={map.known_cells}
      data-explored-area-m2={map.explored_area_m2}
      data-scan-frame={currentScan?.frame_id} data-scan-points={currentScan?.point_count ?? 0}
      data-trajectory-points={currentScan?.trajectory?.length ?? 0} />
    <div className="robot-detail-view-readout flex min-h-8 shrink-0 flex-wrap items-center gap-x-hmi-3 gap-y-1 border-b border-ptagv-border-soft bg-white px-hmi-3 py-hmi-2 text-hmi-kicker tracking-normal text-ptagv-muted"><span className="font-bold text-ptagv-text">ACCUMULATED /map</span><span>{map.width} × {map.height} · {valueNumber(map.resolution, 3)} m/cell</span><span>{map.explored_area_m2?.toFixed(2) ?? "—"} m² explored</span></div>
    <PosePickerMap map={map} robot={robot}
      scan={currentScan}
      trajectory={currentScan?.trajectory ?? []}
      showRobot showScan showGrid={false} showPose={false}
      pose={{ x: pose?.x ?? 0, y: pose?.y ?? 0, yaw: pose?.yaw ?? 0 }}
      active={false} onPick={() => undefined} ariaLabel="Accumulated SLAM /map 2D view" />
  </div>;
}

export type ActiveMapLayers = { robot: boolean; scan: boolean; path: boolean; trajectory: boolean; grid: boolean };

export function ActiveNavigationMap2DView({ map, robot, fixtureRobotPose, scan, target, navigationPath = [], canPick, onPick, onConfirmPoint,
  navigationPathKind = null, layers, onLayerToggle, compactWorkspace = false, errorLogCount, errorLogTone, onOpenErrorLog, onInteractionModeChange,
  pointSelectionResetVersion, onCancelPointSelection, controlViewMode, onControlViewModeChange, webglSupported, threeDimensionalAvailable }: {
  map: RobotDetailMapSnapshot | null;
  robot?: RobotState;
  fixtureRobotPose?: FramePose;
  scan: RobotDetailScan | null;
  target: MapPointNavigationTarget | null;
  navigationPath?: RobotWorldPoint[];
  navigationPathKind?: "PREVIEW" | "ACTIVE_NAV" | null;
  canPick: boolean;
  onPick: (target: MapPointNavigationTarget) => void;
  onConfirmPoint?: (target: MapPointNavigationTarget) => void;
  layers?: ActiveMapLayers;
  onLayerToggle?: (layer: keyof ActiveMapLayers) => void;
  compactWorkspace?: boolean;
  errorLogCount?: number;
  errorLogTone?: "neutral" | "info" | "warning" | "error";
  onOpenErrorLog?: () => void;
  onInteractionModeChange?: (mode: "PAN" | "POINT") => void;
  pointSelectionResetVersion?: number;
  onCancelPointSelection?: () => void;
  controlViewMode?: "2D" | "3D";
  onControlViewModeChange?: (mode: "2D" | "3D") => void;
  webglSupported?: boolean;
  threeDimensionalAvailable?: boolean;
}) {
  if (!map || map.frame_id !== "map" || !map.active_map_id
      || (!map.active_map_revision && map.map_source !== "SLAM_TOOLBOX")
      || !["SLAM_TOOLBOX", "LOCAL_MAP", "NAV2_MAP"].includes(map.map_source ?? "")
      || (map.map_source === "SLAM_TOOLBOX"
        && (!map.mapping_session_id || map.active_map_id !== `SLAM-${map.mapping_session_id}`))) {
    return <div className="grid min-h-24 place-items-center p-hmi-3 text-center text-sm text-ptagv-muted" data-testid="slam-map-2d-empty">Waiting for the robot's active map snapshot.</div>;
  }
  const identity: MapPoseIdentity = {
    frame_id: map.frame_id, active_map_id: map.active_map_id,
    active_map_revision: map.active_map_revision, map_source: map.map_source,
    mapping_session_id: map.mapping_session_id,
  };
  const poseIdentity: MapPoseIdentity = map.active_map_id === "CANONICAL"
    ? { ...identity, map_source: "CANONICAL" } : identity;
  const displayedPose = robot?.id === map.robot_id ? displayedFramePose(robot, poseIdentity) : undefined;
  const slamMap = map.map_source === "SLAM_TOOLBOX";
  const currentScan = slamMap && scan && scan.mapping_session_id === map.mapping_session_id
    && scan.frame_id === map.frame_id ? scan : null;
  const mapIdentity: NavigationMapIdentity = {
    frame_id: map.frame_id, map_id: map.active_map_id, map_revision: String(map.active_map_revision),
    source_type: "ACTIVE_MAP_POINT",
  };
  const selectedTarget = target?.frame_id === mapIdentity.frame_id
    && target.map_id === mapIdentity.map_id && target.map_revision === mapIdentity.map_revision ? target : null;
  const path = layers?.path !== false && (selectedTarget || navigationPath.length) ? navigationPath : [];
  const title = slamMap ? "ACCUMULATED SLAM /map" : "ACTIVE NAVIGATION MAP";
  return <div className="flex h-full min-h-0 min-w-0 w-full flex-1 flex-col overflow-hidden" data-testid="active-navigation-map-2d"
    data-map-id={map.active_map_id} data-map-revision={map.active_map_revision}>
    <div data-testid="slam-map-2d-metrics" data-map-source={map.map_source}
      data-map-frame={map.frame_id} data-map-id={map.active_map_id}
      data-map-revision={map.active_map_revision} data-mapping-session-id={map.mapping_session_id ?? ""}
      data-map-version={map.map_version} data-known-cells={map.known_cells}
      data-explored-area-m2={map.explored_area_m2?.toFixed?.(3) ?? map.explored_area_m2}
      data-scan-frame={currentScan?.frame_id} data-scan-points={currentScan?.point_count ?? 0}
      data-trajectory-points={currentScan?.trajectory?.length ?? 0} />
    {!compactWorkspace && <div className="robot-detail-view-readout flex min-h-8 shrink-0 flex-wrap items-center gap-x-hmi-3 gap-y-1 border-b border-ptagv-border-soft bg-white px-hmi-3 py-hmi-2 text-hmi-kicker tracking-normal text-ptagv-muted"><span className="font-bold text-ptagv-text">{title}</span>
      <span>{map.width} × {map.height} · {valueNumber(map.resolution, 3)} m/cell</span>
      {slamMap && <span>{map.explored_area_m2?.toFixed(2) ?? "—"} m² explored</span>}
      {!slamMap && <span>ACTIVE MAP</span>}
    </div>}
    <PosePickerMap map={map} poseMapIdentity={poseIdentity} robot={robot?.id === map.robot_id ? robot : undefined}
      fixtureRobotPose={fixtureRobotPose}
      scan={currentScan} trajectory={layers?.trajectory ? currentScan?.trajectory ?? [] : []}
      controlViewMode={controlViewMode} onControlViewModeChange={onControlViewModeChange}
      webglSupported={webglSupported} threeDimensionalAvailable={threeDimensionalAvailable}
      showRobot={layers?.robot ?? true} showScan={slamMap && (layers?.scan ?? true)} showGrid={layers?.grid ?? false} showPose={Boolean(selectedTarget)}
      layerState={layers} onLayerToggle={onLayerToggle}
      errorLogCount={errorLogCount} errorLogTone={errorLogTone} onOpenErrorLog={onOpenErrorLog}
      pointSelectionResetVersion={pointSelectionResetVersion} onCancelPointSelection={onCancelPointSelection}
      pose={selectedTarget ?? { x: displayedPose?.x ?? 0, y: displayedPose?.y ?? 0, yaw: displayedPose?.yaw ?? 0 }}
      defaultPickYaw={displayedPose?.yaw ?? 0}
      onInteractionModeChange={onInteractionModeChange}
      navigationPath={path} navigationPathKind={navigationPathKind}
      compactToolbar={compactWorkspace}
      pickInstruction={canPick ? "FIRST CLICK: DESTINATION · SECOND CLICK: CONFIRM HEADING" : undefined}
      active={canPick} onPick={(point, yaw) => onPick(mapPointTarget(mapIdentity,
        { ...point, yaw: yaw ?? displayedPose?.yaw ?? 0 }))}
      onConfirmPoint={(point, yaw) => onConfirmPoint?.(mapPointTarget(mapIdentity, { ...point, yaw }))}
      ariaLabel={slamMap ? "Accumulated SLAM /map 2D view" : "Active navigation map 2D view"} />
  </div>;
}

export function ControlMapViewSwitcher({ view, onChange, webglSupported, available }: {
  view: "2D" | "3D";
  onChange: (mode: "2D" | "3D") => void;
  webglSupported: boolean;
  available: boolean;
}) {
  return <div className="control-map-toolbar-dock pointer-events-auto absolute left-1/2 top-hmi-2 z-30 flex w-fit max-w-[calc(100%-1rem)] -translate-x-1/2 items-center gap-1 rounded-md border border-ptagv-border bg-white p-1 shadow-sm"
    role="group" aria-label="Control map view">
    <MapViewButtons view={view} onChange={onChange} webglSupported={webglSupported} available={available} />
  </div>;
}

function MapViewButtons({ view, onChange, webglSupported, available }: {
  view: "2D" | "3D";
  onChange: (mode: "2D" | "3D") => void;
  webglSupported: boolean;
  available: boolean;
}) {
  const unavailableReason = !webglSupported ? "WebGL is unavailable in this browser." : "No 3D LiDAR frame is available.";
  return <>
    <Button tone={view === "2D" ? "primary" : "quiet"} size="compact" className="whitespace-nowrap"
      aria-label="2D map view" aria-pressed={view === "2D"} onClick={() => onChange("2D")}>2D MAP</Button>
    <Button tone={view === "3D" ? "primary" : "quiet"} size="compact" className="whitespace-nowrap"
      aria-label="3D LiDAR view" aria-pressed={view === "3D"} disabled={!webglSupported || !available}
      title={!webglSupported || !available ? unavailableReason : "Open 3D LiDAR view"}
      onClick={() => onChange("3D")}>3D VIEW</Button>
  </>;
}

type MapInteractionMode = "PAN" | "POINT";
type MapLayerKey = keyof ActiveMapLayers;
type MapToolIconName = "pan" | "point" | "layers";

function MapToolIcon({ name }: { name: MapToolIconName }) {
  return (
    <svg aria-hidden="true" viewBox="0 0 24 24" fill="none" stroke="currentColor"
      strokeWidth="1.7" strokeLinecap="round" strokeLinejoin="round" className="control-map-toolbar-icon">
      {name === "pan" && <path d="M7.5 10V5.2a1.5 1.5 0 0 1 3 0V10V4a1.5 1.5 0 0 1 3 0v6V5.2a1.5 1.5 0 0 1 3 0v6V7.1a1.5 1.5 0 0 1 3 0v5.1a5.8 5.8 0 0 1-5.8 5.8h-2.2a5 5 0 0 1-3.7-1.7l-3-3.3a1.5 1.5 0 0 1 2.2-2.1l1.5 1.3" />}
      {name === "point" && <>
        <circle cx="12" cy="12" r="4" />
        <path d="M12 2.5v3M12 18.5v3M2.5 12h3M18.5 12h3" />
        <circle cx="12" cy="12" r="1" fill="currentColor" stroke="none" />
      </>}
      {name === "layers" && <>
        <path d="m12 3 9 5-9 5-9-5 9-5Z" />
        <path d="m3 12 9 5 9-5M3 16l9 5 9-5" />
      </>}
    </svg>
  );
}

export type MapViewportReadout = {
  centerX: number;
  centerY: number;
  scalePxPerMeter: number;
  fitScalePxPerMeter: number;
  width: number;
  height: number;
};

function CompactControlMapToolbar({ interactionMode, onInteractionModeChange, layerState, onLayerToggle,
  placement = "control", pointModeLabel = "POINT", toolbarLabel = "Map interaction modes and layers",
  controlViewMode, onControlViewModeChange, webglSupported = true, threeDimensionalAvailable = true }: {
  interactionMode: MapInteractionMode;
  onInteractionModeChange: (mode: MapInteractionMode) => void;
  layerState: ActiveMapLayers;
  onLayerToggle: (layer: MapLayerKey) => void;
  placement?: "control" | "workspace";
  pointModeLabel?: string;
  toolbarLabel?: string;
  controlViewMode?: "2D" | "3D";
  onControlViewModeChange?: (mode: "2D" | "3D") => void;
  webglSupported?: boolean;
  threeDimensionalAvailable?: boolean;
}) {
  const [layersOpen, setLayersOpen] = useState(false);
  const rootRef = useRef<HTMLDivElement>(null);
  const id = useId();

  useEffect(() => {
    if (!layersOpen) return;
    const onPointerDown = (event: PointerEvent) => {
      if (event.target instanceof Node && !rootRef.current?.contains(event.target)) setLayersOpen(false);
    };
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key !== "Escape") return;
      setLayersOpen(false);
      rootRef.current?.querySelector<HTMLButtonElement>('[data-testid="map-layer-toggle"]')?.focus();
    };
    document.addEventListener("pointerdown", onPointerDown);
    document.addEventListener("keydown", onKeyDown);
    return () => {
      document.removeEventListener("pointerdown", onPointerDown);
      document.removeEventListener("keydown", onKeyDown);
    };
  }, [layersOpen]);

  const layers: Array<{ key: MapLayerKey; label: string }> = [
    { key: "robot", label: "ROBOT" },
    { key: "scan", label: "SCAN" },
    { key: "path", label: "PATH" },
    { key: "trajectory", label: "TRAJECTORY" },
    { key: "grid", label: "GRID" },
  ];

  const layerDockLeft = placement === "control"
    ? "calc(50% + (var(--spacing-hmi-status-panel) - var(--spacing-hmi-operation-panel)) / 2)" : "0px";
  const layerDockWidth = "min(22.25rem, calc(100vw - var(--spacing-hmi-status-panel) - var(--spacing-hmi-operation-panel) - 8rem))";
  return <div ref={rootRef} className="pointer-events-none absolute inset-0 z-30">
    <div className={`control-map-toolbar-dock pointer-events-auto absolute top-hmi-2 z-30 flex w-fit max-w-[calc(100%-1rem)] items-center gap-1 border border-ptagv-border bg-white p-1 rounded-md shadow-sm ${placement === "control" ? "-translate-x-1/2" : ""}`}
      role="group" aria-label={toolbarLabel} data-testid="robot-map-layers"
      style={{ left: layerDockLeft }}
      onPointerDown={(event) => event.stopPropagation()} onClick={(event) => event.stopPropagation()}>
      {placement === "control" && controlViewMode && onControlViewModeChange && threeDimensionalAvailable && <>
        <MapViewButtons view={controlViewMode} onChange={onControlViewModeChange}
          webglSupported={webglSupported} available={threeDimensionalAvailable} />
        <span className="control-map-toolbar-divider" aria-hidden="true" />
      </>}
      <Button tone="quiet" size="compact" className="control-map-toolbar-button whitespace-nowrap"
        aria-label="PAN" title="Pan map" aria-pressed={interactionMode === "PAN"} data-testid="map-mode-pan"
        onClick={() => onInteractionModeChange("PAN")}><MapToolIcon name="pan" /><span>PAN</span></Button>
      <Button tone="quiet" size="compact" className="control-map-toolbar-button whitespace-nowrap"
        aria-label={pointModeLabel === "POINT" ? "POINT" : pointModeLabel} title="Select a position and heading" aria-pressed={interactionMode === "POINT"} data-testid="map-mode-point"
        onClick={() => onInteractionModeChange("POINT")}><MapToolIcon name="point" /><span>{pointModeLabel}</span></Button>
      <span className="control-map-toolbar-divider" aria-hidden="true" />
      <Button tone="quiet" size="compact" className="control-map-toolbar-button whitespace-nowrap" aria-label="LAYERS" title="Show or hide map layers" aria-expanded={layersOpen} aria-haspopup="true"
        aria-controls={id + "-layers"} data-testid="map-layer-toggle"
        onClick={() => setLayersOpen((open) => !open)}><MapToolIcon name="layers" /><span>LAYERS</span></Button>
    </div>
    {layersOpen && <div id={id + "-layers"} className={`control-map-layer-popover pointer-events-auto absolute z-40 flex flex-wrap gap-x-hmi-3 gap-y-hmi-2 border border-ptagv-border bg-white px-hmi-3 py-hmi-2 rounded-md shadow-sm ${placement === "control" ? "top-24 -translate-x-1/2" : "left-0 top-11 w-max max-w-[calc(100vw-2rem)]"}`}
      style={placement === "control" ? { left: layerDockLeft, width: layerDockWidth } : undefined}
      role="group" aria-label="Map layers" data-testid="map-layer-options">
      {layers.map(({ key, label }) => <label className="inline-flex min-h-7 items-center gap-hmi-1 whitespace-nowrap text-hmi-kicker font-semibold text-ptagv-text"
        htmlFor={id + "-" + key} key={key}>
        <input id={id + "-" + key} className="size-4 accent-blue-700 focus-visible:outline-2 focus-visible:outline-offset-1 focus-visible:outline-ptagv-blue"
          type="checkbox" checked={layerState[key]} aria-label={label} data-testid={"map-layer-" + key}
          onChange={() => onLayerToggle(key)} />
        {label}
      </label>)}
    </div>}
  </div>;
}

type MapProps = { map: RobotDetailMapSnapshot; poseMapIdentity?: MapPoseIdentity; robot?: RobotState; fixtureRobotPose?: FramePose; scan?: RobotDetailScan | null; trajectory?: RobotWorldPoint[]; navigationPath?: RobotWorldPoint[]; navigationPathKind?: "PREVIEW" | "ACTIVE_NAV" | null; pickInstruction?: string; showRobot?: boolean; showScan?: boolean; showGrid?: boolean; showPose?: boolean; layerState?: ActiveMapLayers; onLayerToggle?: (layer: keyof ActiveMapLayers) => void; pose: Pose; defaultPickYaw?: number; active: boolean; onPick: (point: Pick<Pose, "x" | "y">, yaw?: number) => void; onConfirmPoint?: (point: Pick<Pose, "x" | "y">, yaw: number) => void; onInteractionModeChange?: (mode: "PAN" | "POINT") => void; pointSelectionResetVersion?: number; pointModeRequestVersion?: number; onCancelPointSelection?: () => void; errorLogCount?: number; errorLogTone?: "neutral" | "info" | "warning" | "error"; onOpenErrorLog?: () => void; compactToolbar?: boolean; toolbarPlacement?: "control" | "workspace"; pointModeLabel?: string; toolbarLabel?: string; initialInteractionMode?: "PAN" | "POINT"; onViewportInfo?: (info: MapViewportReadout) => void; controlViewMode?: "2D" | "3D"; onControlViewModeChange?: (mode: "2D" | "3D") => void; webglSupported?: boolean; threeDimensionalAvailable?: boolean; ariaLabel?: string };
type MapDrag = { pointerId: number; clientX: number; clientY: number; ratioX: number; ratioY: number; start: MapViewportState; moved: boolean; mode: "PAN" | "POINT" | "LEGACY" };
type PointDirectionPreview = { start: { x: number; y: number }; end: { x: number; y: number } };
type PointYawAnchor = { world: { x: number; y: number }; screen: { x: number; y: number } };
type InteractiveMapSnapshot = { map: RobotDetailMapSnapshot; occupancyGrid: Int8Array | null; raster: HTMLCanvasElement; contentKey: string; identityKey: string };
export function PosePickerMap({ map, poseMapIdentity = map, robot, fixtureRobotPose, scan = null, trajectory = [], navigationPath = [], navigationPathKind = null, pickInstruction, showRobot = true, showScan = true, showGrid = false, showPose = true, layerState, onLayerToggle, pose, defaultPickYaw, active, onPick, onConfirmPoint, onInteractionModeChange, pointSelectionResetVersion, pointModeRequestVersion, onCancelPointSelection, errorLogCount, errorLogTone = "neutral", onOpenErrorLog, compactToolbar = false, toolbarPlacement = "control", pointModeLabel = "POINT", toolbarLabel, initialInteractionMode = "POINT", onViewportInfo, controlViewMode, onControlViewModeChange, webglSupported = true, threeDimensionalAvailable = true, ariaLabel = "Select map frame initial robot position" }: MapProps) {
  const hostRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [size, setSize] = useState({ width: 0, height: 0 });
  const [viewportState, setViewportState] = useState<MapViewportState | null>(null);
  const [isPanning, setIsPanning] = useState(false);
  const [interactionMode, setInteractionMode] = useState<MapInteractionMode>(initialInteractionMode);
  const [directionPreview, setDirectionPreview] = useState<PointDirectionPreview | null>(null);
  const [pointYawAnchor, setPointYawAnchor] = useState<PointYawAnchor | null>(null);
  const [interactiveMapState, setInteractiveMapState] = useState<InteractiveMapSnapshot | null>(null);
  const [pickFeedback, setPickFeedback] = useState("");
  const drag = useRef<MapDrag | null>(null);
  const seenPointModeRequestVersion = useRef(pointModeRequestVersion);
  const suppressClick = useRef(false);
  const suppressClickTimer = useRef<number | null>(null);
  const pickFeedbackTimer = useRef<number | null>(null);
  const panFrame = useRef<number | null>(null);
  const pendingPan = useRef<MapViewportState | null>(null);
  const displayedPose = useStableDisplayedFramePose(robot?.id === map.robot_id ? robot : undefined, poseMapIdentity);
  const fixturePoseIsStaticUi = import.meta.env.DEV
    && fixtureRobotPose?.pose_source === "STATIC_UI_FIXTURE"
    && fixtureRobotPose.map_source === "STATIC_UI_FIXTURE";
  const fixturePoseIsShowcase = fixtureRobotPose?.pose_source === "LOCAL_SAMPLE"
    && fixtureRobotPose.map_source === "LOCAL_MAP";
  const fixturePoseMatchesMap = map.map_source === "LOCAL_MAP"
    && fixtureRobotPose?.valid === true && (fixturePoseIsStaticUi || fixturePoseIsShowcase)
    && fixtureRobotPose.frame_id === map.frame_id
    && Boolean(map.active_map_id) && fixtureRobotPose.map_id === map.active_map_id
    && String(fixtureRobotPose.map_revision) === String(map.active_map_revision ?? "")
    && [fixtureRobotPose.x, fixtureRobotPose.y, fixtureRobotPose.yaw].every(Number.isFinite);
  const renderedRobotPose = displayedPose ?? (fixturePoseMatchesMap ? fixtureRobotPose : undefined);
  const hasInteractionModeToolbar = Boolean(layerState && onLayerToggle);
  const mapContentKey = occupancyRasterKey(map);
  const mapIdentityKey = JSON.stringify([map.robot_id, map.frame_id, map.map_source, map.active_map_id,
    map.active_map_revision, map.mapping_session_id]);
  useEffect(() => {
    let current = true;
    // Publish geometry, clickable cells and raster atomically. During same-map
    // content updates the last complete snapshot remains usable until this one
    // has fully decoded and rasterized.
    void Promise.all([decodeOccupancyGrid(map), occupancyRasters.get(map)]).then(([occupancyGrid, raster]) => {
      if (current && raster) {
        setInteractiveMapState((previous) => {
          if (!occupancyGrid && previous?.identityKey === mapIdentityKey && previous.occupancyGrid) return previous;
          return { map, occupancyGrid, raster, contentKey: mapContentKey, identityKey: mapIdentityKey };
        });
      }
    });
    return () => { current = false; };
  }, [map, mapContentKey, mapIdentityKey]);
  const interactiveMapSnapshot = interactiveMapState?.identityKey === mapIdentityKey ? interactiveMapState : null;
  const occupancyGrid = interactiveMapSnapshot?.occupancyGrid ?? null;
  const raster = interactiveMapSnapshot?.raster ?? null;
  const renderMap = interactiveMapSnapshot?.map ?? map;
  useEffect(() => {
    if (!interactiveMapSnapshot || pickFeedback !== "MAP UPDATING") return;
    setPickFeedback("");
    if (pickFeedbackTimer.current !== null) window.clearTimeout(pickFeedbackTimer.current);
    pickFeedbackTimer.current = null;
  }, [interactiveMapSnapshot, pickFeedback]);
  const bounds = useMemo<WorldBounds>(() => occupancyMapWorldBounds(renderMap), [renderMap]);
  const viewportSessionKey = mapViewportSessionKey(map);
  const resolvedViewport = resolveMapViewport(viewportState, map, size);
  const camera = resolvedViewport?.camera ?? { centerX: 0, centerY: 0, scalePxPerMeter: 1, fitScalePxPerMeter: 1 };
  const transform = useMemo(() => fixedWorldTransform(size, camera), [camera, size]);
  useEffect(() => {
    onViewportInfo?.({ centerX: camera.centerX, centerY: camera.centerY,
      scalePxPerMeter: camera.scalePxPerMeter, fitScalePxPerMeter: camera.fitScalePxPerMeter,
      width: size.width, height: size.height });
  }, [camera.centerX, camera.centerY, camera.scalePxPerMeter, camera.fitScalePxPerMeter,
    onViewportInfo, size.height, size.width]);
  useLayoutEffect(() => {
    setViewportState((current) => resolveMapViewport(current, map, size));
  }, [viewportSessionKey, size.width, size.height]);
  useEffect(() => {
    drag.current = null;
    pendingPan.current = null;
    setDirectionPreview(null);
    setPointYawAnchor(null);
    setIsPanning(false);
    return () => {
      if (panFrame.current !== null) cancelAnimationFrame(panFrame.current);
      if (suppressClickTimer.current !== null) window.clearTimeout(suppressClickTimer.current);
      if (pickFeedbackTimer.current !== null) window.clearTimeout(pickFeedbackTimer.current);
      suppressClickTimer.current = null;
      pickFeedbackTimer.current = null;
      panFrame.current = null;
      pendingPan.current = null;
      drag.current = null;
    };
  }, [viewportSessionKey]);
  useEffect(() => {
    if (!showPose) {
      setPointYawAnchor(null);
      setDirectionPreview(null);
    }
  }, [showPose]);
  useEffect(() => {
    setPointYawAnchor(null);
    setDirectionPreview(null);
    setPickFeedback("");
  }, [pointSelectionResetVersion]);
  useEffect(() => {
    if (pointModeRequestVersion === undefined) return;
    if (seenPointModeRequestVersion.current === pointModeRequestVersion) return;
    seenPointModeRequestVersion.current = pointModeRequestVersion;
    setInteractionMode("POINT");
    setPointYawAnchor(null);
    setDirectionPreview(null);
  }, [pointModeRequestVersion]);
  const showPickFeedback = (message: string) => {
    setPickFeedback(message);
    if (pickFeedbackTimer.current !== null) window.clearTimeout(pickFeedbackTimer.current);
    pickFeedbackTimer.current = window.setTimeout(() => {
      setPickFeedback("");
      pickFeedbackTimer.current = null;
    }, 1800);
  };
  useEffect(() => {
    setPickFeedback("");
    if (pickFeedbackTimer.current !== null) window.clearTimeout(pickFeedbackTimer.current);
  }, [mapIdentityKey]);
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const wheel = (event: WheelEvent) => {
      if (!event.ctrlKey || event.deltaY === 0) return;
      event.preventDefault();
      if (drag.current) return;
      const rect = canvas.getBoundingClientRect();
      if (rect.width <= 0 || rect.height <= 0) return;
      const cursor = { x: (event.clientX - rect.left) * size.width / rect.width,
        y: (event.clientY - rect.top) * size.height / rect.height };
      setViewportState((current) => {
        const resolved = resolveMapViewport(current, map, size);
        return resolved ? { ...resolved, camera: zoomMapViewportCameraAt(resolved.camera,
          event.deltaY < 0 ? 1.12 : 1 / 1.12, size, cursor) } : current;
      });
    };
    canvas.addEventListener("wheel", wheel, { passive: false });
    return () => canvas.removeEventListener("wheel", wheel);
  }, [map, size]);
  useEffect(() => {
    const host = hostRef.current;
    if (!host) return;
    const resize = () => setSize({ width: host.clientWidth, height: host.clientHeight });
    resize();
    const observer = typeof ResizeObserver === "undefined" ? null : new ResizeObserver(resize);
    observer?.observe(host);
    return () => observer?.disconnect();
  }, []);
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || size.width <= 0 || size.height <= 0) return;
    const dpr = Math.max(1, Math.min(2, window.devicePixelRatio || 1));
    canvas.width = size.width * dpr; canvas.height = size.height * dpr;
    canvas.style.width = `${size.width}px`; canvas.style.height = `${size.height}px`;
    const context = canvas.getContext("2d"); if (!context) return;
    context.setTransform(dpr, 0, 0, dpr, 0, 0);
    context.fillStyle = "#919191"; context.fillRect(0, 0, size.width, size.height);
    if (raster) {
      const origin = worldToScreen({ x: renderMap.origin.x, y: renderMap.origin.y }, transform);
      context.save(); context.translate(origin.x, origin.y); context.rotate(-renderMap.origin.yaw);
      context.scale(transform.scale * renderMap.resolution, -transform.scale * renderMap.resolution);
      context.imageSmoothingEnabled = false; context.drawImage(raster, 0, 0); context.restore();
    }
    if (showGrid) drawMappingGrid(context, size.width, size.height, transform, bounds);
    if (showScan && scan?.frame_id === renderMap.frame_id) {
      const origin = scan.sensor_pose;
      // Accumulated occupancy is the base layer; the current LaserScan is a
      // restrained point overlay, not a fan/ray rendering that can dominate it.
      context.save(); context.globalAlpha = 0.18; context.fillStyle = "#00a99a";
      for (const [x, y] of scan.points) { const p = worldToScreen({ x, y }, transform); context.fillRect(p.x - 1, p.y - 1, 2, 2); }
      if (origin) { const center = worldToScreen({ x: origin.x, y: origin.y }, transform); context.globalAlpha = 0.75; context.fillRect(center.x - 2, center.y - 2, 4, 4); }
      context.restore();
    }
    if (trajectory.length > 1) {
      context.beginPath();
      trajectory.forEach(([x, y], index) => {
        const point = worldToScreen({ x, y }, transform);
        if (index === 0) context.moveTo(point.x, point.y); else context.lineTo(point.x, point.y);
      });
      context.strokeStyle = "#e18a2b"; context.lineWidth = 2; context.globalAlpha = 0.88; context.stroke(); context.globalAlpha = 1;
    }
    if (navigationPath.length > 1) {
      context.beginPath();
      navigationPath.forEach(([x, y], index) => {
        const point = worldToScreen({ x, y }, transform);
        if (index === 0) context.moveTo(point.x, point.y); else context.lineTo(point.x, point.y);
      });
      context.strokeStyle = navigationPathKind === "ACTIVE_NAV" ? "#168346" : "#74c78e";
      context.lineWidth = navigationPathKind === "ACTIVE_NAV" ? 3 : 2;
      context.lineCap = "round"; context.lineJoin = "round";
      context.setLineDash(navigationPathKind === "PREVIEW" ? [8, 5] : []);
      context.globalAlpha = navigationPathKind === "ACTIVE_NAV" ? 1 : 0.92;
      context.stroke(); context.setLineDash([]); context.globalAlpha = 1;
    }
    if (showPose) drawPose(context, transform, pose.x, pose.y, pose.yaw, "#f6cf4f");
    if (showRobot && renderedRobotPose) drawPose(context, transform, renderedRobotPose.x, renderedRobotPose.y, renderedRobotPose.yaw, "#1686d9");
    if (directionPreview) drawDirectionArrow(context, transform, directionPreview);
    context.fillStyle = "#8aa4bf"; context.font = "10px JetBrains Mono, monospace";
    context.fillText(active ? pickInstruction ?? (pointYawAnchor ? "SELECT YAW · CLICK TO CONFIRM" : "CLICK FREE CELL TO SET DESTINATION") : "MAP FRAME · METRES", 10, size.height - 10);
  }, [active, bounds, directionPreview, displayedPose, renderedRobotPose, map, navigationPath, navigationPathKind, pickInstruction, pointYawAnchor, pose, raster, renderMap, scan, showGrid, showPose, showRobot, showScan, size, trajectory, transform]);
  const pointAtClient = (clientX: number, clientY: number, canvas: HTMLCanvasElement, currentTransform = transform) => {
    const rect = canvas.getBoundingClientRect();
    if (rect.width <= 0 || rect.height <= 0) return null;
    return screenToWorld({ x: (clientX - rect.left) * size.width / rect.width,
      y: (clientY - rect.top) * size.height / rect.height }, currentTransform);
  };
  const isPointPickingMode = hasInteractionModeToolbar ? interactionMode === "POINT" : active;
  const canvasPointAtClient = (clientX: number, clientY: number, canvas: HTMLCanvasElement) => {
    const rect = canvas.getBoundingClientRect();
    if (rect.width <= 0 || rect.height <= 0) return null;
    return { x: (clientX - rect.left) * size.width / rect.width,
      y: (clientY - rect.top) * size.height / rect.height };
  };
  const click = (event: MouseEvent<HTMLCanvasElement>) => {
    if (suppressClick.current) {
      suppressClick.current = false;
      if (suppressClickTimer.current !== null) window.clearTimeout(suppressClickTimer.current);
      suppressClickTimer.current = null;
      return;
    }
    if (!active || !isPointPickingMode) return;
    if (!interactiveMapSnapshot || !raster || !occupancyGrid) {
      showPickFeedback("MAP UPDATING");
      return;
    }
    const screen = canvasPointAtClient(event.clientX, event.clientY, event.currentTarget);
    const cursorWorld = screen ? screenToWorld(screen, transform) : null;
    if (!cursorWorld) return;
    if (!hasInteractionModeToolbar) {
      if (isFreeOccupancyPoint(interactiveMapSnapshot.map, occupancyGrid, cursorWorld)) onPick(cursorWorld, defaultPickYaw ?? pose.yaw);
      return;
    }
    if (pointYawAnchor) {
      const distancePx = screen ? Math.hypot(screen.x - pointYawAnchor.screen.x, screen.y - pointYawAnchor.screen.y) : 0;
      const yaw = distancePx > MAP_PAN_THRESHOLD_PX
        ? Math.atan2(cursorWorld.y - pointYawAnchor.world.y, cursorWorld.x - pointYawAnchor.world.x)
        : defaultPickYaw ?? pose.yaw;
      onConfirmPoint?.(pointYawAnchor.world, yaw);
      setPointYawAnchor(null);
      setDirectionPreview(null);
      return;
    }
    const classification = classifyOccupancyPoint(interactiveMapSnapshot.map, occupancyGrid, cursorWorld);
    if (classification !== "FREE") {
      showPickFeedback(classification === "OCCUPIED" ? "OCCUPIED CELL"
        : classification === "UNKNOWN" ? "UNKNOWN / UNMAPPED CELL" : "OUTSIDE MAP");
      return;
    }
    const anchor = { world: cursorWorld, screen: screen! };
    setPointYawAnchor(anchor);
    setDirectionPreview(null);
    setPickFeedback("");
    onPick(cursorWorld, defaultPickYaw ?? pose.yaw);
  };
  const flushPan = () => {
    const next = pendingPan.current;
    pendingPan.current = null;
    panFrame.current = null;
    if (next) setViewportState((current) => current?.sessionKey === next.sessionKey ? next : current);
  };
  const movePointer = (event: ReactPointerEvent<HTMLCanvasElement>) => {
    const current = drag.current;
    if (!current) {
      if (hasInteractionModeToolbar && active && isPointPickingMode && pointYawAnchor) {
        const end = pointAtClient(event.clientX, event.clientY, event.currentTarget);
        const screen = canvasPointAtClient(event.clientX, event.clientY, event.currentTarget);
        if (end && screen && Math.hypot(screen.x - pointYawAnchor.screen.x, screen.y - pointYawAnchor.screen.y) > MAP_PAN_THRESHOLD_PX) {
          setDirectionPreview({ start: pointYawAnchor.world, end });
        } else setDirectionPreview(null);
      }
      return;
    }
    if (current.pointerId !== event.pointerId) return;
    const dx = event.clientX - current.clientX, dy = event.clientY - current.clientY;
    if (!current.moved && Math.hypot(dx, dy) <= MAP_PAN_THRESHOLD_PX) return;
    if (!current.moved) {
      current.moved = true;
      if (current.mode !== "POINT") setIsPanning(true);
    }
    suppressClick.current = true;
    event.preventDefault();
    if (current.mode === "POINT") return;
    pendingPan.current = { ...current.start, camera: panMapViewportCamera(current.start.camera,
      { x: dx * current.ratioX, y: dy * current.ratioY }) };
    if (panFrame.current === null) panFrame.current = requestAnimationFrame(flushPan);
  };
  const finishPointer = (event: ReactPointerEvent<HTMLCanvasElement>, cancelled = false) => {
    const current = drag.current;
    if (current?.pointerId !== event.pointerId) return;
    if (!cancelled) movePointer(event);
    if (current.mode === "POINT") {
      suppressClick.current = cancelled || current.moved;
    } else {
      suppressClick.current = cancelled || current.moved;
    }
    if (suppressClick.current) {
      if (suppressClickTimer.current !== null) window.clearTimeout(suppressClickTimer.current);
      suppressClickTimer.current = window.setTimeout(() => {
        suppressClick.current = false;
        suppressClickTimer.current = null;
      }, 0);
    }
    if (panFrame.current !== null) cancelAnimationFrame(panFrame.current);
    flushPan();
    drag.current = null;
    if (current.mode !== "POINT" || current.moved || cancelled) setDirectionPreview(null);
    setIsPanning(false);
    if (event.currentTarget.hasPointerCapture?.(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId);
  };
  const updateCamera = (update: (current: MapViewportState["camera"]) => MapViewportState["camera"]) => {
    setViewportState((current) => {
      const resolved = resolveMapViewport(current, map, size);
      return resolved ? { ...resolved, camera: update(resolved.camera) } : current;
    });
  };
  const cursorModeClass = hasInteractionModeToolbar
    ? interactionMode === "PAN" ? "is-pan-mode" : "is-point-mode"
    : active ? "is-picking" : "";
  const pickFeedbackMessage = pickFeedback || (!interactiveMapSnapshot ? "MAP UPDATING" : "");
  const errorToneClass: Record<NonNullable<MapProps["errorLogTone"]>, string> = {
    neutral: "is-neutral border-ptagv-border bg-slate-50 text-ptagv-muted",
    info: "is-info border-blue-200 bg-blue-50 text-blue-800",
    warning: "is-warning border-amber-300 bg-amber-50 text-ptagv-warning",
    error: "is-error border-red-300 bg-red-50 text-ptagv-error",
  };
  return <div ref={hostRef} className={`local-pose-map relative flex min-h-0 min-w-0 w-full flex-1 overflow-hidden bg-slate-500 ${cursorModeClass}${isPanning ? " is-panning" : ""}`}>
    <div className="absolute bottom-hmi-2 left-hmi-2 z-20 flex max-w-[calc(100%-1rem)] flex-wrap gap-1 border border-ptagv-border bg-white/95 p-1 shadow-sm" role="group" aria-label="Map controls" data-testid="robot-map-toolbar">
      <Button size="compact" className="min-h-8" aria-label="Zoom in" title="Zoom in" onClick={() => updateCamera((current) => zoomMapViewportCamera(current, 1.25))}>+</Button>
      <Button size="compact" className="min-h-8" aria-label="Zoom out" title="Zoom out" onClick={() => updateCamera((current) => zoomMapViewportCamera(current, 1 / 1.25))}>−</Button>
      <Button size="compact" className="min-h-8" onClick={() => updateCamera(() => fitMapViewportCamera(size, bounds))}>FIT</Button>
      <Button size="compact" className="min-h-8" disabled={!renderedRobotPose} onClick={() => {
        if (!renderedRobotPose) return;
        updateCamera((current) => centerMapViewportCamera(current, renderedRobotPose));
      }}>CENTER ROBOT</Button>
      {onOpenErrorLog && <Button size="compact" className={`hmi-error-log-trigger min-h-8 ${errorToneClass[errorLogTone]}`} aria-haspopup="dialog"
        data-testid="control-error-log-trigger" onClick={onOpenErrorLog}>
        ERROR LOG{errorLogCount ? ` (${errorLogCount})` : ""}
      </Button>}
    </div>
    {layerState && onLayerToggle && compactToolbar && <CompactControlMapToolbar
      interactionMode={interactionMode}
      onInteractionModeChange={(mode) => {
        setInteractionMode(mode);
        if (mode === "PAN") {
          setPointYawAnchor(null);
          setDirectionPreview(null);
        }
        setPickFeedback("");
        onInteractionModeChange?.(mode);
      }}
      layerState={layerState}
      onLayerToggle={onLayerToggle}
      placement={toolbarPlacement}
      pointModeLabel={pointModeLabel}
      toolbarLabel={toolbarLabel}
      controlViewMode={controlViewMode}
      onControlViewModeChange={onControlViewModeChange}
      webglSupported={webglSupported}
      threeDimensionalAvailable={threeDimensionalAvailable}
    />}
    {layerState && onLayerToggle && !compactToolbar && <div className="absolute right-hmi-2 top-hmi-2 z-20 flex max-w-[min(27.5rem,calc(100%-1rem))] flex-wrap justify-end gap-1 border border-ptagv-border bg-white/95 p-1" role="group" aria-label="Map interaction modes and layers" data-testid="robot-map-layers">
      <Button size="compact" className={`min-h-8 whitespace-nowrap ${interactionMode === "PAN" ? "is-active border-blue-300 bg-blue-50 text-ptagv-blue" : ""}`} aria-label="PAN" title="Pan map" aria-pressed={interactionMode === "PAN"} data-testid="map-mode-pan"
        onClick={() => { setInteractionMode("PAN"); setPointYawAnchor(null); setDirectionPreview(null); setPickFeedback(""); onInteractionModeChange?.("PAN"); }}>✋ PAN</Button>
      <Button size="compact" className={`min-h-8 whitespace-nowrap ${interactionMode === "POINT" ? "is-active border-blue-300 bg-blue-50 text-ptagv-blue" : ""}`} aria-label="POINT" title="Select a destination and yaw" aria-pressed={interactionMode === "POINT"} data-testid="map-mode-point"
        onClick={() => { setInteractionMode("POINT"); setPickFeedback(""); onInteractionModeChange?.("POINT"); }}>⌖ POINT</Button>
      <span className="my-0.5 w-px self-stretch bg-ptagv-border" aria-hidden="true" />
      {(["robot", "scan", "path", "trajectory", "grid"] as const).map((layer) => <Button
        key={layer} size="compact" className={`min-h-8 whitespace-nowrap ${layerState[layer] ? "is-active border-blue-300 bg-blue-50 text-ptagv-blue" : "bg-slate-100 text-slate-600"}`}
        aria-pressed={layerState[layer]} onClick={() => onLayerToggle(layer)}>{layerState[layer] ? "✓ " : "□ "}{layer.toUpperCase()}</Button>)}
    </div>}
    {active && isPointPickingMode && pickFeedbackMessage && <div className={`pointer-events-none absolute left-1/2 ${compactToolbar ? "top-[4.25rem]" : "top-14"} z-30 min-h-5 max-w-[calc(100%-1rem)] -translate-x-1/2 border border-slate-400 bg-white/95 px-2 py-1 text-center text-xs font-semibold leading-snug text-slate-700`} role="status" aria-live="polite" data-testid="map-pick-feedback">
      {pickFeedbackMessage}
    </div>}
    <canvas ref={canvasRef}
    data-testid={map.map_source === "SLAM_TOOLBOX" ? "slam-map-2d-canvas" : undefined}
    data-occupancy-layer="accumulated-map-snapshot"
    data-scan-layer={showScan && scan ? "low-opacity-current-scan-overlay" : "disabled"}
    data-scan-render-mode="points-only"
    data-navigation-path-point-count={navigationPath.length}
    data-navigation-path-kind={navigationPathKind ?? "none"}
    data-robot-layer={showRobot && renderedRobotPose ? "visible" : "disabled"}
    data-grid-layer={showGrid ? "visible" : "disabled"}
    data-path-layer={navigationPath.length > 1 ? "visible" : "disabled"}
    data-trajectory-layer={trajectory.length > 1 ? "visible" : "disabled"}
    data-map-source={renderMap.map_source} data-map-id={renderMap.active_map_id}
    data-interactive-map-content-key={interactiveMapSnapshot?.contentKey ?? ""}
    data-raster-map-content-key={interactiveMapSnapshot?.contentKey ?? ""}
    data-interactive-map-content-revision={renderMap.map_content_revision ?? renderMap.map_version ?? ""}
    data-interaction-mode={hasInteractionModeToolbar ? interactionMode : undefined}
    data-direction-preview={directionPreview ? "visible" : "hidden"}
    data-direction-preview-yaw={directionPreview ? Math.atan2(directionPreview.end.y - directionPreview.start.y,
      directionPreview.end.x - directionPreview.start.x) : undefined}
    data-point-selection-state={active ? pointYawAnchor ? "SELECT_YAW" : "WAITING_FOR_DESTINATION" : undefined}
    data-viewport-center-x={camera.centerX} data-viewport-center-y={camera.centerY}
    data-viewport-scale-px-per-meter={camera.scalePxPerMeter}
    data-viewport-width={size.width} data-viewport-height={size.height}
    data-pose-source={renderedRobotPose?.pose_source}
    data-render-x={renderedRobotPose?.x} data-render-y={renderedRobotPose?.y} data-render-yaw={renderedRobotPose?.yaw}
    onContextMenu={(event) => {
      event.preventDefault();
      // A context click only cancels a point/yaw/preview transaction. Once
      // SEND GOAL clears the selection, right-click must not affect Nav2.
      if (showPose || pointYawAnchor || directionPreview) onCancelPointSelection?.();
    }}
    onPointerDown={(event) => {
      if (event.button !== 0 || drag.current || !resolvedViewport) return;
      const rect = event.currentTarget.getBoundingClientRect();
      if (rect.width <= 0 || rect.height <= 0) return;
      const mode: MapDrag["mode"] = hasInteractionModeToolbar
        ? interactionMode : active ? "LEGACY" : "PAN";
      if (!pointAtClient(event.clientX, event.clientY, event.currentTarget)) return;
      suppressClick.current = false;
      drag.current = { pointerId: event.pointerId, clientX: event.clientX, clientY: event.clientY,
        ratioX: size.width / rect.width, ratioY: size.height / rect.height, start: resolvedViewport,
        moved: false, mode };
      if (mode !== "POINT") setDirectionPreview(null);
      event.currentTarget.setPointerCapture?.(event.pointerId);
    }}
    onPointerMove={movePointer} onPointerUp={(event) => finishPointer(event)}
    onPointerCancel={(event) => finishPointer(event, true)}
    onLostPointerCapture={(event) => { if (drag.current) finishPointer(event, true); }}
    onClick={click} aria-label={ariaLabel} /></div>;
}

function drawMappingGrid(context: CanvasRenderingContext2D, width: number, height: number, transform: ReturnType<typeof createWorldTransform>, bounds: WorldBounds) {
  context.save(); context.strokeStyle = "rgba(83,125,158,.24)"; context.lineWidth = 1;
  const step = 1;
  for (let x = Math.ceil(bounds.minX / step) * step; x <= bounds.maxX; x += step) {
    const a = worldToScreen({ x, y: bounds.minY }, transform), b = worldToScreen({ x, y: bounds.maxY }, transform);
    context.beginPath(); context.moveTo(a.x, a.y); context.lineTo(b.x, b.y); context.stroke();
  }
  for (let y = Math.ceil(bounds.minY / step) * step; y <= bounds.maxY; y += step) {
    const a = worldToScreen({ x: bounds.minX, y }, transform), b = worldToScreen({ x: bounds.maxX, y }, transform);
    context.beginPath(); context.moveTo(a.x, a.y); context.lineTo(b.x, b.y); context.stroke();
  }
  context.fillStyle = "#6b89a5"; context.font = "9px JetBrains Mono, monospace";
  context.fillText("GRID · 1 m", 10, height - 10); context.restore(); void width;
}

function drawPose(context: CanvasRenderingContext2D, transform: ReturnType<typeof createWorldTransform>, x: number, y: number, yaw: number, color: string) {
  const point = worldToScreen({ x, y }, transform);
  context.save(); context.translate(point.x, point.y); context.rotate(-yaw);
  context.fillStyle = color; context.strokeStyle = "#07101c"; context.lineWidth = 1.5;
  context.beginPath(); context.moveTo(11, 0); context.lineTo(-7, -6); context.lineTo(-4, 0); context.lineTo(-7, 6); context.closePath(); context.fill(); context.stroke(); context.restore();
}

function drawDirectionArrow(context: CanvasRenderingContext2D, transform: ReturnType<typeof createWorldTransform>, preview: PointDirectionPreview) {
  const start = worldToScreen(preview.start, transform), end = worldToScreen(preview.end, transform);
  const angle = Math.atan2(end.y - start.y, end.x - start.x);
  const headLength = 10;
  context.save();
  context.strokeStyle = "#d98212"; context.fillStyle = "#f2a62b"; context.lineWidth = 3;
  context.lineCap = "round"; context.beginPath(); context.moveTo(start.x, start.y); context.lineTo(end.x, end.y); context.stroke();
  context.beginPath();
  context.moveTo(end.x, end.y);
  context.lineTo(end.x - headLength * Math.cos(angle - Math.PI / 6), end.y - headLength * Math.sin(angle - Math.PI / 6));
  context.lineTo(end.x - headLength * Math.cos(angle + Math.PI / 6), end.y - headLength * Math.sin(angle + Math.PI / 6));
  context.closePath(); context.fill(); context.restore();
  drawPose(context, transform, preview.start.x, preview.start.y,
    Math.atan2(preview.end.y - preview.start.y, preview.end.x - preview.start.x), "#f6cf4f");
}
