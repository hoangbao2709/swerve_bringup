import { useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState, type MouseEvent, type PointerEvent as ReactPointerEvent, type ReactNode } from "react";
import {
  applyVda5050Configuration,
  getLocalRobotMaps,
  getLocalRuntimeMode,
  getVda5050Configuration,
  initializeLocalRobotPose,
  loadLocalRobotMap,
  saveLocalRobotMap,
  setMappingState,
  testVda5050Connection,
  type LocalRobotMap,
  type LocalMapLoadResult,
  type Vda5050Configuration,
} from "../../services/api";
import type { RobotDetailError, RobotDetailMapSnapshot, RobotDetailScan, RobotLidarStreamDiagnostics, RobotRuntimeCapabilities, RobotSystemDiagnostics, RobotState, RobotWorldPoint } from "../../schema/twin_state";
import { createWorldTransform, worldToScreen, screenToWorld, type WorldBounds } from "../../layout/coordinates";
import { centerMapViewportCamera, fitMapViewportCamera, fixedWorldTransform, MAP_PAN_THRESHOLD_PX, mapViewportSessionKey, occupancyMapWorldBounds, panMapViewportCamera, resolveMapViewport, zoomMapViewportCamera, zoomMapViewportCameraAt, type MapViewportState } from "../../layout/mapViewport";
import { occupancyRasterKey, occupancyRasters } from "./occupancyRaster";
import { classifyOccupancyPoint, decodeOccupancyGrid, isFreeOccupancyPoint } from "./occupancyGrid";
import { displayedFramePose, useStableDisplayedFramePose, type MapPoseIdentity } from "../../layout/robotPoseFrame";
import { mapPointTarget, type MapPointNavigationTarget, type NavigationMapIdentity } from "./navigationMapIdentity";
import { useStore } from "../../state/store";

type SectionName = "MAPPING" | "LOCALIZATION" | "VDA5050" | "DIAGNOSTICS";
type Pose = { x: number; y: number; yaw: number };
type Props = {
  section: SectionName;
  robotId: string;
  robot?: RobotState;
  slam2dMap: RobotDetailMapSnapshot | null;
  runtimeMapSnapshot: RobotDetailMapSnapshot | null;
  localizationMap: RobotDetailMapSnapshot | null;
  scan: RobotDetailScan | null;
  diagnostics: RobotSystemDiagnostics | null;
  errors: RobotDetailError[];
  controlOnline: boolean;
  controlMode: "MANUAL" | "AUTONOMOUS";
  runtimeMode: string;
  runtimeState: string;
  runtimeCapabilities: RobotRuntimeCapabilities | null;
  localization: unknown;
  websocketState: string;
  mapRevision: number | null;
  activeLocalMapId: string | null;
  activeLocalMapRevision: string | null;
  localMapSyncStatus: string | null;
  mappingSessionId?: string | null;
  lidarStreamDiagnostics: RobotLidarStreamDiagnostics | null;
  hostStatus?: { system?: { cpu_load_1m?: number | null; memory?: { used_percent?: number | null } } } | null;
  ensureManualMode?: () => void;
};

function valueText(value: unknown, fallback = "N/A") {
  return value === null || value === undefined || value === "" ? fallback : String(value);
}

function valueNumber(value: unknown, digits = 2, suffix = "") {
  const number = Number(value);
  return Number.isFinite(number) ? `${number.toFixed(digits)}${suffix}` : "N/A";
}

function formatMapCoordinate(value: number) {
  return (Math.abs(value) < 0.0005 ? 0 : value).toFixed(3);
}

function mapCoordinateText(value: unknown, suffix = "") {
  const number = Number(value);
  return Number.isFinite(number) ? `${formatMapCoordinate(number)}${suffix}` : "N/A";
}

function loadOperationLabel(phase?: string, status?: string) {
  const labels: Record<string, string> = {
    PREPARING: "PREPARING...",
    PAUSING_MAPPING: "PAUSING MAPPING...",
    SWITCHING_TO_NAVIGATION: "SWITCHING TO NAVIGATION...",
    STARTING_NAV2: "STARTING NAV2...",
    WAITING_FOR_MANUAL: "RESTORING MANUAL MODE...",
    LOADING_MAP: "LOADING MAP...",
    VERIFYING_MAP: "VERIFYING MAP...",
    ROLLING_BACK: "ROLLING BACK...",
  };
  if (phase && labels[phase]) return labels[phase];
  if (status === "REQUESTED") return labels.PREPARING;
  if (status === "RESTARTING" || status === "STARTING") return labels.SWITCHING_TO_NAVIGATION;
  if (status === "ROLLING_BACK") return labels.ROLLING_BACK;
  return labels.STARTING_NAV2;
}

const waitForLoadPoll = () => new Promise<void>((resolve) => window.setTimeout(resolve, 1000));

function classForStatus(value: unknown) {
  const status = String(value ?? "").toUpperCase();
  if (["OK", "ONLINE", "CONNECTED", "ACTIVE", "READY", "MAPPING", "LOCALIZED"].includes(status)) return "ok";
  if (["WAITING", "PAUSED", "DEGRADED", "STALE", "CONNECTING", "SAVED"].includes(status)) return "warning";
  if (["ERROR", "OFFLINE", "DISCONNECTED", "FAILED", "NO_PATH"].includes(status)) return "error";
  return "neutral";
}

function Status({ value }: { value: unknown }) {
  return <span className={`robot-detail-status ${classForStatus(value)}`}>{valueText(value)}</span>;
}

function SectionFrame({ children, className = "" }: { children: ReactNode; className?: string }) {
  return <div className={`local-robot-section ${className}`}>{children}</div>;
}

function SectionPanel({ title, children, className = "" }: { title: string; children: ReactNode; className?: string }) {
  return <section className={`robot-detail-panel local-section-panel ${className}`}><header>{title}</header><div className="robot-detail-panel-body">{children}</div></section>;
}

function Metric({ label, value, mono = false }: { label: string; value: unknown; mono?: boolean }) {
  return <div className="robot-detail-metric"><span>{label}</span><b className={mono ? "mono" : ""}>{valueText(value)}</b></div>;
}

export function LocalRobotSection(props: Props) {
  if (props.section === "MAPPING") return <MappingPanel {...props} />;
  if (props.section === "LOCALIZATION") return <LocalizationPanel {...props} />;
  if (props.section === "VDA5050") return <Vda5050Panel robotId={props.robotId} />;
  return <DiagnosticsPanel {...props} />;
}

function MappingPanel({ robotId, robot: rawRobot, slam2dMap, runtimeMapSnapshot, scan, diagnostics, controlOnline, controlMode, runtimeState, runtimeCapabilities, mappingSessionId, activeLocalMapRevision: selectedLocalMapRevision, localMapSyncStatus, mapRevision, ensureManualMode }: Props) {
  const setRobotDetail = useStore((state) => state.setRobotDetail);
  const [maps, setMaps] = useState<LocalRobotMap[]>([]);
  const [selected, setSelected] = useState("");
  const [name, setName] = useState("");
  const [mappingState, setMappingStateValue] = useState("UNKNOWN");
  const [mappingDuration, setMappingDuration] = useState(0);
  const [activeLocalMapId, setActiveLocalMapId] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");
  const [notice, setNotice] = useState("");
  const [loadError, setLoadError] = useState("");
  const [loadNotice, setLoadNotice] = useState("");
  const [operationState, setOperationState] = useState("READY");
  const [loadBlockReason, setLoadBlockReason] = useState("SELECT A SAVED MAP");
  const [trajectory, setTrajectory] = useState<RobotWorldPoint[]>([]);
  const [layers, setLayers] = useState({ robot: true, scan: true, trajectory: true, grid: false });
  const trajectoryRef = useRef<{ mapId: string; points: RobotWorldPoint[]; lastAt: number }>({ mapId: "", points: [], lastAt: 0 });
  const manualRestoreRequest = useRef<string | null>(null);

  const slamRuntimeActive = runtimeState === "MAPPING" || runtimeState === "UNIFIED";
  const slamMap = slamRuntimeActive && !activeLocalMapId && slam2dMap?.robot_id === robotId
    && slam2dMap?.map_source === "SLAM_TOOLBOX"
    && (!mappingSessionId || slam2dMap.mapping_session_id === mappingSessionId) ? slam2dMap : null;
  const slamPose = slamMap && rawRobot ? displayedFramePose(rawRobot, slamMap) : undefined;
  const expectedLocalMapRevision = selectedLocalMapRevision
    ?? maps.find((map) => map.id === activeLocalMapId)?.revision ?? null;
  const selectedMap = maps.find((map) => map.id === selected) ?? null;
  const loadedLocalMap = activeLocalMapId && runtimeMapSnapshot?.map_source === "LOCAL_MAP"
    && runtimeMapSnapshot.active_map_id === activeLocalMapId
    && String(runtimeMapSnapshot.active_map_revision ?? "") === String(expectedLocalMapRevision ?? "")
    ? runtimeMapSnapshot : null;
  const selectedMapIsActive = Boolean(selectedMap && selectedMap.id === activeLocalMapId);
  const selectedMapPreview = selectedMapIsActive ? loadedLocalMap : null;
  const selectedMapExtent = selectedMap?.width && selectedMap?.height
    ? `${valueNumber(selectedMap.width * selectedMap.resolution, 2)} × ${valueNumber(selectedMap.height * selectedMap.resolution, 2)} m`
    : "N/A";
  const displayMap = loadedLocalMap ?? slamMap;
  const displayPose = displayMap && rawRobot ? displayedFramePose(rawRobot, displayMap) : undefined;
  const robotPoseMatchesDisplayMap = Boolean(displayMap && displayPose);
  const mapping = diagnostics?.mapping;

  const refresh = useCallback(async () => {
    const result = await getLocalRobotMaps(robotId);
    setMaps(result.maps);
    setMappingStateValue(result.mapping_state);
    setMappingDuration(Number(result.mapping_duration_s) || 0);
    setActiveLocalMapId(result.active_local_map_id);
    const activeLocalMap = result.maps.find((map) => map.id === result.active_local_map_id);
    setRobotDetail(robotId, {
      activeLocalMapId: result.active_local_map_id,
      activeLocalMapRevision: result.local_active_map_revision ?? activeLocalMap?.revision ?? null,
      localMapSyncStatus: result.map_sync_status,
    });
    if (!selected && result.maps.length) setSelected(result.maps[0].id);
  }, [robotId, selected, setRobotDetail]);

  useEffect(() => {
    if (!controlOnline) return;
    let active = true;
    const poll = () => refresh().catch((caught: unknown) => {
      if (active) setError(caught instanceof Error ? caught.message : "Map registry is unavailable");
    });
    void poll();
    const timer = window.setInterval(() => { void poll(); }, 3000);
    return () => { active = false; window.clearInterval(timer); };
  }, [controlOnline, refresh]);

  const run = async (action: () => Promise<unknown>, success: (result: unknown) => string, pendingState: string) => {
    setOperationState(pendingState);
    setBusy(true); setError(""); setNotice("");
    try { const result = await action(); setNotice(success(result)); await refresh(); setOperationState("READY"); }
    catch (caught) { setOperationState("ERROR"); setError(caught instanceof Error ? caught.message : "Robot map operation failed"); }
    finally { setBusy(false); }
  };

  const save = () => void run(() => saveLocalRobotMap(robotId, name), (raw) => {
    const result = raw as { map: LocalRobotMap };
    setName(""); setSelected(result.map.id);
    return `SAVE SUCCESS · ${result.map.name} · revision ${result.map.revision}`;
  }, "SAVING");
  const load = () => {
    if (loadBlockReason) { setLoadError(loadBlockReason); return; }
    const mapId = selected;
    void (async () => {
      setBusy(true); setLoadError(""); setLoadNotice(""); setOperationState("PREPARING...");
      try {
        let result: LocalMapLoadResult = await loadLocalRobotMap(robotId, mapId);
        const deadline = Date.now() + 10 * 60 * 1000;
        while (result.status === "TRANSITIONING") {
          if (Date.now() >= deadline) throw new Error("Saved-map load timed out while the supervised runtime was transitioning");
          setOperationState(loadOperationLabel(result.phase, result.transition?.status));
          await waitForLoadPoll();
          const runtimeResult = await getLocalRuntimeMode(robotId);
          const transition = runtimeResult.transition;
          if (["ERROR", "ROLLED_BACK"].includes(String(transition.status).toUpperCase())) {
            throw new Error(transition.message || result.message || "The supervised navigation transition failed");
          }
          if (result.phase === "ROLLING_BACK"
              && transition.status === "READY"
              && String(transition.mode || "").toUpperCase() !== "NAVIGATION") {
            throw new Error(result.message || "Saved-map load failed; previous runtime was restored");
          }
          if (result.phase === "WAITING_FOR_MANUAL") {
            setOperationState("RESTORING MANUAL MODE...");
            const requestKey = result.request_id || `${robotId}:${mapId}`;
            if (manualRestoreRequest.current !== requestKey) {
              manualRestoreRequest.current = requestKey;
              ensureManualMode?.();
            }
          }
          if (transition.status === "READY" && String(transition.mode || "").toUpperCase() === "NAVIGATION") {
            setOperationState(loadOperationLabel(result.phase, transition.status));
            result = await loadLocalRobotMap(robotId, mapId);
          } else if (!result.request_id || transition.request_id === result.request_id) {
            result = await loadLocalRobotMap(robotId, mapId);
          }
        }
        if (result.status !== "LOADED" || !result.active_map
            || result.active_map.id !== mapId || result.active_map_id !== mapId
            || result.map_sync_status !== "LOCAL_ONLY" || result.map_source !== "LOCAL_MAP") {
          throw new Error(result.message || "ROS did not confirm the selected saved map as the active /map");
        }
        setActiveLocalMapId(result.active_map.id);
        setRobotDetail(robotId, {
          activeLocalMapId: result.active_map.id,
          activeLocalMapRevision: result.active_map.revision,
          localMapSyncStatus: "LOCAL_ONLY",
        });
        setLoadNotice(`MAP LOADED · ${result.active_map.name} · r${result.active_map.revision} · ${result.active_map.resolution.toFixed(3)} m/cell`);
        setOperationState("MAP LOADED");
        manualRestoreRequest.current = null;
        await refresh();
      } catch (caught) {
        setOperationState("LOAD FAILED");
        setLoadError(caught instanceof Error ? caught.message : "Saved-map load failed");
      } finally { setBusy(false); }
    })();
  };
  const changeMapping = (action: "start" | "stop") => void run(
    () => setMappingState(robotId, action),
    (raw) => {
      const result = raw as { mapping_state: string };
      setMappingStateValue(result.mapping_state);
      return `MAPPING ${result.mapping_state}`;
    }, action === "start" ? "STARTING" : "STOPPING",
  );

  const isUnified = runtimeState === "UNIFIED";
  const isMapping = runtimeState === "MAPPING"
    || (isUnified && (runtimeCapabilities?.mapping_available ?? Boolean(diagnostics?.slam)));
  const paused = mappingState === "PAUSED";
  useEffect(() => {
    if (busy) { setLoadBlockReason("MAP TRANSITION IN PROGRESS"); return; }
    if (!controlOnline) { setLoadBlockReason("ROBOT OFFLINE"); return; }
    if (!selected) { setLoadBlockReason("SELECT A SAVED MAP"); return; }
    if (controlMode !== "MANUAL" || (rawRobot?.control_mode && rawRobot.control_mode !== "MANUAL")) {
      setLoadBlockReason("SWITCH TO MANUAL"); return;
    }
    if (diagnostics?.command_ownership?.estop_active === true
        || String(rawRobot?.navigation_state || "").toUpperCase() === "EMERGENCY_STOPPED") {
      setLoadBlockReason("CLEAR E-STOP BEFORE LOADING MAP"); return;
    }
    const navigationState = String(rawRobot?.navigation_state || "").toUpperCase();
    if (!navigationState) { setLoadBlockReason("WAIT FOR ROBOT SAFETY STATUS"); return; }
    if (!["IDLE", "MANUAL", "SUCCEEDED", "COMPLETED", "FAILED", "ERROR", "CANCELED", "CANCELLED", "REJECTED"].includes(navigationState)) {
      setLoadBlockReason("NAVIGATION GOAL MUST BE CANCELLED"); return;
    }
    const velocities = [rawRobot?.vx, rawRobot?.vy, rawRobot?.wz].map(Number);
    if (!velocities.every((velocity) => Number.isFinite(velocity) && Math.abs(velocity) <= 0.05)) {
      setLoadBlockReason("STOP ROBOT BEFORE LOADING MAP"); return;
    }
    setLoadBlockReason("");
  }, [busy, controlMode, controlOnline, diagnostics?.command_ownership?.estop_active,
    rawRobot?.control_mode, rawRobot?.navigation_state, rawRobot?.vx, rawRobot?.vy, rawRobot?.wz, selected]);
  const startMapping = async () => {
    if (isMapping) { changeMapping("start"); return; }
    setError(isUnified ? "SLAM is not currently available in this Unified runtime. Wait for SLAM readiness before mapping." : "SLAM mapping requires the Unified runtime. Start one stack with ./scripts/start_stack.sh unified --gui --rviz.");
  };
  useEffect(() => {
    const mapId = `${robotId}:${slamMap?.active_map_id ?? "waiting"}`;
    if (trajectoryRef.current.mapId === mapId) return;
    trajectoryRef.current = { mapId, points: [], lastAt: 0 };
    setTrajectory([]);
  }, [robotId, slamMap?.active_map_id]);
  useEffect(() => {
    if (!slamMap || !slamPose || !isMapping || paused || !mapping?.tf_valid) return;
    const x = Number(slamPose.x);
    const y = Number(slamPose.y);
    if (!Number.isFinite(x) || !Number.isFinite(y)) return;
    const current = trajectoryRef.current;
    const previous = current.points[current.points.length - 1];
    const now = Date.now();
    if (previous && Math.hypot(x - previous[0], y - previous[1]) < 0.10 && now - current.lastAt < 2000) return;
    const points = [...current.points, [x, y] as RobotWorldPoint].slice(-500);
    trajectoryRef.current = { ...current, points, lastAt: now };
    setTrajectory(points);
  }, [isMapping, mapping?.tf_valid, paused, slamPose?.x, slamPose?.y, slamPose?.timestamp, slamMap]);
  return <SectionFrame className="hmi-mapping-workflow-layout">
    <SectionPanel title={loadedLocalMap ? "ACTIVE SAVED LOCAL MAP · /map" : "ACCUMULATED SLAM MAP · /map + CURRENT /scan"} className="hmi-mapping-map">
      <div className="local-map-toggles" role="group" aria-label="Mapping map layers">
        {(["robot", "scan", "trajectory", "grid"] as const).map((layer) => <button
          key={layer} type="button" aria-pressed={layers[layer]} className={layers[layer] ? "is-active" : ""}
          onClick={() => setLayers((current) => ({ ...current, [layer]: !current[layer] }))}>
          {layers[layer] ? "✓ " : "□ "}{layer.toUpperCase()}
        </button>)}
      </div>
      {displayMap ? <PosePickerMap map={displayMap} robot={robotPoseMatchesDisplayMap ? rawRobot : undefined}
        scan={slamMap && scan?.robot_id === robotId && scan.mapping_session_id === slamMap.mapping_session_id ? scan : null}
        showRobot={layers.robot} showScan={Boolean(slamMap && layers.scan)} showGrid={layers.grid} showPose={false}
        trajectory={layers.trajectory && slamMap && robotPoseMatchesDisplayMap ? trajectory : []} pose={{
        x: displayPose?.x ?? 0, y: displayPose?.y ?? 0, yaw: displayPose?.yaw ?? 0,
      }} active={false} onPick={() => undefined} /> : <div className="local-empty">{activeLocalMapId ? "Waiting for the confirmed active saved /map snapshot." : "Waiting for a fresh accumulated SLAM Toolbox /map. Live LiDAR frames are sensor views and are not the warehouse map."}</div>}
    </SectionPanel>
    <SectionPanel title="MAPPING STATUS" className="hmi-mapping-status">
      <div className="local-status-grid hmi-mapping-primary-metrics">
        <Metric label="STATE" value={isMapping ? mappingState : isUnified ? "WAITING FOR SLAM" : "INACTIVE"} />
        <Metric label="MAP SIZE" value={displayMap ? `${displayMap.width} × ${displayMap.height} cells` : "WAITING FOR MAP"} mono />
        <Metric label="RESOLUTION" value={valueNumber(displayMap?.resolution, 3, " m/cell")} mono />
        <Metric label="KNOWN CELLS" value={displayMap?.known_cells ?? "—"} mono />
        <Metric label="EXPLORED AREA" value={valueNumber(displayMap?.explored_area_m2, 2, " m²")} mono />
        <Metric label="SESSION TIME" value={`${valueNumber(mappingDuration, 1, " s")}${isMapping && !paused ? " · LIVE" : ""}`} mono />
      </div>
      <details className="hmi-advanced-details"><summary>ADVANCED MAPPING DETAILS</summary>
        <div className="local-status-grid">
          <Metric label="ROBOT" value={robotId} mono /><Metric label="RUNTIME MODE" value={runtimeState} />
          <Metric label="SLAM TOOLBOX" value={mapping?.slam_state ?? "UNKNOWN"} />
          <Metric label="NAV2" value={runtimeCapabilities?.nav2_ready === true && diagnostics?.nav2_ready === true ? "READY" : diagnostics?.nav2 ? "STARTING" : runtimeState === "UNIFIED" ? "UNAVAILABLE" : "INACTIVE"} />
          <Metric label="CONTROL MODE" value={controlMode} />
          <Metric label="LIVE LIDAR · /scan" value={mapping?.scan_live ? `LIVE · ${valueNumber(mapping.scan_hz, 2, " Hz")} · ${mapping.scan_frame ?? "frame unknown"}` : "WAITING"} />
          <Metric label="ODOMETRY" value={mapping?.odom_live ? `LIVE · ${valueNumber(mapping.odom_hz, 2, " Hz")} · ${mapping.odom_frame ?? "?"} → ${mapping.base_frame ?? "?"}` : "WAITING"} />
          <Metric label="TF · map ← lidar" value={mapping?.tf_lidar_to_map_valid ? "OK" : mapping?.tf_error ?? "WAITING / INVALID"} />
          <Metric label="ACCUMULATED /map" value={loadedLocalMap ? "ACTIVE · SAVED LOCAL MAP" : mapping?.map_live && slamMap ? "LIVE · SLAM TOOLBOX" : "WAITING FOR SLAM MAP"} />
          <Metric label="MAP UPDATE RATE" value={mapping?.map_live ? `${valueNumber(mapping.map_hz, 2, " Hz")} · v${slamMap?.map_version ?? mapping.map_version ?? "—"}` : "WAITING"} />
          <Metric label="MAP → ODOM OWNER" value={mapping?.map_odom_owner ?? "UNVERIFIED"} />
          <Metric label="WORKFLOW STATE" value={operationState} mono />
          <Metric label="MAP ORIGIN" value={slamMap ? `${valueNumber(slamMap.origin.x, 2)}, ${valueNumber(slamMap.origin.y, 2)} m` : "—"} mono />
          <Metric label="UNKNOWN CELLS" value={slamMap?.unknown_cells ?? "—"} mono />
          <Metric label="OCCUPIED CELLS" value={slamMap?.occupied_cells ?? "—"} mono />
          <Metric label="FREE CELLS" value={slamMap?.free_cells ?? "—"} mono />
          <Metric label="ROBOT POSE · TF map → base" value={robotPoseMatchesDisplayMap && displayPose ? `${valueNumber(displayPose.x, 2)}, ${valueNumber(displayPose.y, 2)} m · ${valueNumber(displayPose.yaw, 2)} rad` : "WAITING FOR MATCHING MAP POSE"} mono />
          <Metric label="ACTIVE MAP" value={activeLocalMapId ? `${activeLocalMapId} · r${runtimeMapSnapshot?.active_map_revision ?? "—"}` : isMapping ? `${slamMap?.active_map_id ?? "SLAM SESSION WAITING"} · LOCAL ONLY` : "CANONICAL"} mono />
          <Metric label="TRAJECTORY SAMPLES" value={trajectory.length} mono /><Metric label="AVAILABLE MAPS" value={maps.length} mono />
        </div>
        <p className="local-help">Loading a saved map uses the supervised SLAM-to-Navigation handoff. Saved SLAM pose-graph restoration remains a separate operation.</p>
      </details>
    </SectionPanel>
    <SectionPanel title="MAPPING CONTROL" className="hmi-mapping-control">
      <div className="local-action-row hmi-mapping-actions">
        <button type="button" className="robot-console-primary" disabled={!controlOnline || controlMode !== "MANUAL" || busy || !isMapping || mappingState === "MAPPING"} onClick={() => void startMapping()}>{isMapping && mappingState === "MAPPING" ? "MAPPING ACTIVE" : paused ? "RESUME MAPPING" : isMapping ? "START MAPPING" : isUnified ? "SLAM UNAVAILABLE" : "UNIFIED RUNTIME REQUIRED"}</button>
        <button type="button" disabled={!controlOnline || !isMapping || busy || paused} onClick={() => changeMapping("stop")}>PAUSE MAPPING</button>
      </div>
      <p className="local-help">Start and resume require an online robot in Manual mode with SLAM ready. Pause before saving the accumulated map.</p>
    </SectionPanel>
    <SectionPanel title="SAVE MAP" className="hmi-mapping-save">
      <div className="local-form-row">
        <label className="local-field local-field-grow"><span>MAP NAME</span><input value={name} maxLength={64} onChange={(event) => setName(event.target.value)} placeholder="warehouse_floor_1" /></label>
        <button type="button" className="robot-console-primary" disabled={!controlOnline || !isMapping || !paused || !slamMap || busy || !name.trim()} onClick={save}>SAVE MAP</button>
      </div>
      <p className="local-help">Pause mapping before saving the accumulated map and SLAM session. Canonical maps are not overwritten.</p>
      {error && <div className="local-feedback error" role="alert">{error}</div>}
      {notice && <div className="local-feedback ok" role="status">{notice}</div>}
    </SectionPanel>
    <SectionPanel title={`SAVED MAPS · THIS ROBOT (${maps.length})`} className="hmi-mapping-list">
      <div className="local-map-list hmi-mapping-library-list">
        {maps.length === 0 ? <div className="local-empty">No saved maps for {robotId}.</div> : maps.map((map) => <button type="button" className={`local-map-row ${selected === map.id ? "is-selected" : ""}`} key={map.id} onClick={() => setSelected(map.id)}>
          <span><b>{map.name}</b>{activeLocalMapId === map.id && <strong className="local-map-active-badge">ACTIVE</strong>}<small>{map.created_at} · {map.resolution.toFixed(3)} m/cell · SLAM {map.slam_session_state?.status ?? "NOT_SAVED"}</small></span>
          <small>r{map.revision}</small>
        </button>)}
      </div>
      <div className="local-action-row hmi-mapping-session-actions">
        <button type="button" disabled title="In-process SLAM pose-graph restore is not implemented yet">RESUME SAVED SLAM SESSION</button>
      </div>
    </SectionPanel>
    <SectionPanel title="MAP PREVIEW" className="hmi-mapping-preview">
      {selectedMapPreview ? <PosePickerMap map={selectedMapPreview} robot={rawRobot} pose={{ x: 0, y: 0, yaw: 0 }} active={false} onPick={() => undefined} ariaLabel={`Active saved map preview ${selectedMapPreview.active_map_id}`} />
        : <div className="local-empty hmi-map-preview-empty">{selectedMap ? "This saved map is not active. Load it to receive its confirmed navigation-map preview." : "Select a stored map to inspect its details."}</div>}
    </SectionPanel>
    <SectionPanel title="MAP DETAILS" className="hmi-mapping-details">
      {selectedMap ? <>
        <div className="hmi-map-detail-state"><b>{selectedMap.name}</b>{selectedMapIsActive && <strong className="local-map-active-badge">ACTIVE MAP</strong>}</div>
        <div className="local-status-grid hmi-map-details-grid">
          <Metric label="STATUS" value={selectedMapIsActive ? "ACTIVE · LOCAL MAP" : "STORED · NOT ACTIVE"} />
          <Metric label="MAP ID" value={selectedMap.map_id ?? selectedMap.id} mono />
          <Metric label="MAP REVISION" value={selectedMap.revision} mono />
          <Metric label="FRAME" value={selectedMap.frame_id} mono />
          <Metric label="MAP SYNC STATUS" value={selectedMapIsActive ? localMapSyncStatus ?? "UNVERIFIED" : "NOT ACTIVE"} />
          <Metric label="CANONICAL REVISION" value={mapRevision ?? "N/A"} mono />
          <Metric label="ACTIVE LOCAL MAP" value={activeLocalMapId ?? "NONE"} mono />
          <Metric label="ACTIVE LOCAL REVISION" value={expectedLocalMapRevision ?? "N/A"} mono />
          <Metric label="RESOLUTION" value={valueNumber(selectedMap.resolution, 3, " m/cell")} mono />
          <Metric label="WIDTH × HEIGHT" value={selectedMap.width && selectedMap.height ? `${selectedMap.width} × ${selectedMap.height}` : "N/A"} mono />
          <Metric label="EXTENT" value={selectedMapExtent} mono />
          <Metric label="ORIGIN" value={selectedMap.origin?.slice(0, 3).map((part) => valueNumber(part, 3)).join(", ") ?? "N/A"} mono />
          <Metric label="CREATED" value={selectedMap.created_at} />
          <Metric label="KNOWN CELLS" value={selectedMap.known_cells ?? "N/A"} mono />
          <Metric label="FREE CELLS" value={selectedMap.free_cells ?? "N/A"} mono />
          <Metric label="OCCUPIED CELLS" value={selectedMap.occupied_cells ?? "N/A"} mono />
          <Metric label="EXPLORED AREA" value={valueNumber(selectedMap.explored_area_m2, 2, " m²")} mono />
          <Metric label="SLAM SESSION" value={selectedMap.slam_session_state?.status ?? "NOT AVAILABLE"} />
        </div>
      </> : <div className="local-empty">Select a stored map to view its identity, synchronization, occupancy metadata, and statistics.</div>}
    </SectionPanel>
    <SectionPanel title="LOAD / RELOAD MAP" className="hmi-mapping-load">
      <button type="button" className="robot-console-primary hmi-map-load-button" aria-describedby="saved-map-load-state" disabled={!selectedMap || Boolean(loadBlockReason) || busy} onClick={load}>{busy ? operationState : selectedMapIsActive ? "RELOAD ACTIVE MAP" : "LOAD SAVED MAP"}</button>
      <p id="saved-map-load-state" className={loadBlockReason ? "local-help warning" : "local-help"} role="status">
        {busy ? operationState : loadError || loadNotice || loadBlockReason || (selectedMapIsActive ? "SELECTED MAP IS ACTIVE" : "READY · MANUAL MODE AND STOP REQUIRED")}
      </p>
    </SectionPanel>
  </SectionFrame>;
}

export function AccumulatedSlamMap2DView({ map, robot, scan }: {
  map: RobotDetailMapSnapshot | null;
  robot?: RobotState;
  scan: RobotDetailScan | null;
}) {
  if (!map || map.map_source !== "SLAM_TOOLBOX" || map.frame_id !== "map"
      || !map.mapping_session_id || map.active_map_id !== `SLAM-${map.mapping_session_id}`) {
    return <div className="local-empty" data-testid="slam-map-2d-empty">Waiting for a current SLAM Toolbox /map.</div>;
  }
  const pose = robot?.slam_pose;
  const currentScan = scan?.mapping_session_id === map.mapping_session_id
    && scan.frame_id === map.frame_id ? scan : null;
  return <div className="robot-slam-map-2d" data-testid="slam-map-2d">
    <div data-testid="slam-map-2d-metrics" data-map-source="SLAM_TOOLBOX"
      data-map-frame={map.frame_id} data-mapping-session-id={map.mapping_session_id}
      data-map-version={map.map_version} data-known-cells={map.known_cells}
      data-explored-area-m2={map.explored_area_m2}
      data-scan-frame={currentScan?.frame_id} data-scan-points={currentScan?.point_count ?? 0}
      data-trajectory-points={currentScan?.trajectory?.length ?? 0} />
    <div className="robot-detail-view-readout"><span>ACCUMULATED /map</span><span>{map.width} × {map.height} · {valueNumber(map.resolution, 3)} m/cell</span><span>{map.explored_area_m2?.toFixed(2) ?? "—"} m² explored</span></div>
    <PosePickerMap map={map} robot={robot}
      scan={currentScan}
      trajectory={currentScan?.trajectory ?? []}
      showRobot showScan showGrid={false} showPose={false}
      pose={{ x: pose?.x ?? 0, y: pose?.y ?? 0, yaw: pose?.yaw ?? 0 }}
      active={false} onPick={() => undefined} ariaLabel="Accumulated SLAM /map 2D view" />
  </div>;
}

export type ActiveMapLayers = { robot: boolean; scan: boolean; path: boolean; trajectory: boolean; grid: boolean };

export function ActiveNavigationMap2DView({ map, robot, scan, target, navigationPath = [], canPick, onPick, onConfirmPoint,
  navigationPathKind = null, layers, onLayerToggle, compactWorkspace = false, errorLogCount, errorLogTone, onOpenErrorLog, onInteractionModeChange,
  pointSelectionResetVersion, onCancelPointSelection }: {
  map: RobotDetailMapSnapshot | null;
  robot?: RobotState;
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
}) {
  if (!map || map.frame_id !== "map" || !map.active_map_id
      || (!map.active_map_revision && map.map_source !== "SLAM_TOOLBOX")
      || !["SLAM_TOOLBOX", "LOCAL_MAP", "NAV2_MAP"].includes(map.map_source ?? "")
      || (map.map_source === "SLAM_TOOLBOX"
        && (!map.mapping_session_id || map.active_map_id !== `SLAM-${map.mapping_session_id}`))) {
    return <div className="local-empty" data-testid="slam-map-2d-empty">Waiting for the robot's active map snapshot.</div>;
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
  return <div className="robot-slam-map-2d" data-testid="active-navigation-map-2d"
    data-map-id={map.active_map_id} data-map-revision={map.active_map_revision}>
    <div data-testid="slam-map-2d-metrics" data-map-source={map.map_source}
      data-map-frame={map.frame_id} data-map-id={map.active_map_id}
      data-map-revision={map.active_map_revision} data-mapping-session-id={map.mapping_session_id ?? ""}
      data-map-version={map.map_version} data-known-cells={map.known_cells}
      data-explored-area-m2={map.explored_area_m2?.toFixed?.(3) ?? map.explored_area_m2}
      data-scan-frame={currentScan?.frame_id} data-scan-points={currentScan?.point_count ?? 0}
      data-trajectory-points={currentScan?.trajectory?.length ?? 0} />
    {!compactWorkspace && <div className="robot-detail-view-readout"><span>{title}</span>
      <span>{map.width} × {map.height} · {valueNumber(map.resolution, 3)} m/cell</span>
      {slamMap && <span>{map.explored_area_m2?.toFixed(2) ?? "—"} m² explored</span>}
      {!slamMap && <span>ACTIVE MAP</span>}
    </div>}
    <PosePickerMap map={map} poseMapIdentity={poseIdentity} robot={robot?.id === map.robot_id ? robot : undefined}
      scan={currentScan} trajectory={layers?.trajectory ? currentScan?.trajectory ?? [] : []}
      showRobot={layers?.robot ?? true} showScan={slamMap && (layers?.scan ?? true)} showGrid={layers?.grid ?? false} showPose={Boolean(selectedTarget)}
      layerState={layers} onLayerToggle={onLayerToggle}
      errorLogCount={errorLogCount} errorLogTone={errorLogTone} onOpenErrorLog={onOpenErrorLog}
      pointSelectionResetVersion={pointSelectionResetVersion} onCancelPointSelection={onCancelPointSelection}
      pose={selectedTarget ?? { x: displayedPose?.x ?? 0, y: displayedPose?.y ?? 0, yaw: displayedPose?.yaw ?? 0 }}
      defaultPickYaw={displayedPose?.yaw ?? 0}
      onInteractionModeChange={onInteractionModeChange}
      navigationPath={path} navigationPathKind={navigationPathKind}
      pickInstruction={canPick ? "FIRST CLICK: DESTINATION · SECOND CLICK: CONFIRM HEADING" : undefined}
      active={canPick} onPick={(point, yaw) => onPick(mapPointTarget(mapIdentity,
        { ...point, yaw: yaw ?? displayedPose?.yaw ?? 0 }))}
      onConfirmPoint={(point, yaw) => onConfirmPoint?.(mapPointTarget(mapIdentity, { ...point, yaw }))}
      ariaLabel={slamMap ? "Accumulated SLAM /map 2D view" : "Active navigation map 2D view"} />
  </div>;
}

function LocalizationPanel({ robotId, robot, localizationMap, controlOnline, localization, runtimeCapabilities, activeLocalMapId, activeLocalMapRevision }: Props) {
  const current = useMemo<Pose | null>(() => {
    const reported = robot?.active_map_pose;
    const identityMatches = !activeLocalMapId || Boolean(localizationMap && reported?.map_id === localizationMap.active_map_id
      && String(reported?.map_revision ?? "") === String(localizationMap.active_map_revision ?? ""));
    return reported?.valid && identityMatches
      && (!activeLocalMapId || runtimeCapabilities?.localization_ready === true)
      && [reported.x, reported.y, reported.yaw].every(Number.isFinite)
      ? { x: reported.x, y: reported.y, yaw: reported.yaw } : null;
  }, [activeLocalMapId, localizationMap, robot?.active_map_pose, runtimeCapabilities?.localization_ready]);
  const [pose, setPose] = useState<Pose>({ x: Number.NaN, y: Number.NaN, yaw: Number.NaN });
  const [poseEdited, setPoseEdited] = useState(false);
  const [pickMode, setPickMode] = useState(false);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");
  const [notice, setNotice] = useState("");
  const selectedRobot = useRef(robotId);
  useEffect(() => {
    const robotChanged = selectedRobot.current !== robotId;
    selectedRobot.current = robotId;
    if (robotChanged || !poseEdited) setPose(current ?? { x: Number.NaN, y: Number.NaN, yaw: Number.NaN });
    if (robotChanged) setPoseEdited(false);
  }, [current, poseEdited, robotId]);
  const update = (key: keyof Pose, value: number) => {
    setPose((previous) => ({ ...previous, [key]: value }));
    setPoseEdited(true);
  };
  const poseReady = [pose.x, pose.y, pose.yaw].every(Number.isFinite);
  const apply = async () => {
    if (!poseReady) return;
    if (!window.confirm(`Initialize ${robotId} at x=${pose.x.toFixed(2)}, y=${pose.y.toFixed(2)}, yaw=${pose.yaw.toFixed(2)} rad?`)) return;
    setBusy(true); setError(""); setNotice("");
    try {
      await initializeLocalRobotPose(robotId, { ...pose, frame_id: "map" });
      setNotice("INITIAL POSE ACCEPTED BY ekf_v30e · waiting for the next localization update");
    } catch (caught) { setError(caught instanceof Error ? caught.message : "Initial pose was not accepted"); }
    finally { setBusy(false); }
  };

  return <SectionFrame className="hmi-localization-layout">
    <SectionPanel title="LOCALIZATION STATE" className="hmi-localization-status">
      <div className="local-status-grid">
        <Metric label="X · MAP" value={current ? mapCoordinateText(current.x, " m") : "UNKNOWN"} mono />
        <Metric label="Y · MAP" value={current ? mapCoordinateText(current.y, " m") : "UNKNOWN"} mono />
        <Metric label="YAW" value={current ? mapCoordinateText(current.yaw, " rad") : "UNKNOWN"} mono />
        <Metric label="FRAME" value="map" mono />
        <Metric label="LOCALIZATION" value={activeLocalMapId
          ? runtimeCapabilities?.localization_ready ? "LOCALIZED" : "INITIAL POSE REQUIRED"
          : localization} />
        {activeLocalMapId && <Metric label="ACTIVE SAVED MAP" value={`${activeLocalMapId} · r${activeLocalMapRevision ?? "—"}`} mono />}
        <Metric label="OWNER" value="ekf_v30e · map → odom" mono />
      </div>
      {activeLocalMapId && <p className={runtimeCapabilities?.localization_ready ? "local-feedback ok" : "local-feedback warning"} role="status">
        {runtimeCapabilities?.localization_ready ? "LOCALIZED · confirmed TF matches the active saved map." : "INITIAL POSE REQUIRED · point navigation remains unavailable until the requested map-frame pose is confirmed."}
      </p>}
    </SectionPanel>
    <SectionPanel title={localizationMap?.map_source === "LOCAL_MAP" ? "ACTIVE SAVED MAP · LOCALIZATION" : "ACTIVE ROS MAP · LOCALIZATION"} className="hmi-localization-map">
      {localizationMap ? <PosePickerMap map={localizationMap} robot={robot} pose={pose} showPose={poseReady} active={pickMode} onPick={(point) => { setPose((old) => ({ ...old, ...point })); setPoseEdited(true); }} ariaLabel="Localization map and initial pose picker" />
        : <div className="local-empty" data-testid="localization-map-waiting">{activeLocalMapId ? "WAITING FOR ACTIVE SAVED MAP SNAPSHOT..." : "Waiting for the robot scoped ROS map snapshot."}</div>}
    </SectionPanel>
    <SectionPanel title="INITIALIZE ROBOT POSE" className="local-pose-editor hmi-localization-editor">
      <div className="local-pose-fields">
        <label className="local-field"><span>X · MAP (m)</span><input type="number" step="0.01" value={Number.isFinite(pose.x) ? formatMapCoordinate(pose.x) : ""} onChange={(event) => update("x", event.target.value === "" ? Number.NaN : Number(event.target.value))} /></label>
        <label className="local-field"><span>Y · MAP (m)</span><input type="number" step="0.01" value={Number.isFinite(pose.y) ? formatMapCoordinate(pose.y) : ""} onChange={(event) => update("y", event.target.value === "" ? Number.NaN : Number(event.target.value))} /></label>
        <label className="local-field"><span>YAW (rad)</span><input type="number" step="0.01" value={Number.isFinite(pose.yaw) ? formatMapCoordinate(pose.yaw) : ""} onChange={(event) => update("yaw", event.target.value === "" ? Number.NaN : Number(event.target.value))} /></label>
        <div className="local-pose-yaw"><button type="button" disabled={!Number.isFinite(pose.yaw)} onClick={() => update("yaw", pose.yaw - Math.PI / 12)}>YAW −</button><button type="button" disabled={!Number.isFinite(pose.yaw)} onClick={() => update("yaw", pose.yaw + Math.PI / 12)}>YAW +</button></div>
        <button type="button" className={pickMode ? "is-active" : ""} onClick={() => setPickMode((value) => !value)} disabled={!localizationMap}>PICK ON MAP</button>
        <button type="button" className="robot-console-primary" disabled={!controlOnline || busy || !poseReady} onClick={() => void apply()}>SET INITIAL POSE</button>
      </div>
      <p className="local-help">The pose is applied through the current authoritative robot_localization EKF service. It changes localization and does not teleport the robot.</p>
    </SectionPanel>
    {error && <div className="local-feedback error" role="alert">{error}</div>}
    {notice && <div className="local-feedback ok" role="status">{notice}</div>}
  </SectionFrame>;
}

type MapProps = { map: RobotDetailMapSnapshot; poseMapIdentity?: MapPoseIdentity; robot?: RobotState; scan?: RobotDetailScan | null; trajectory?: RobotWorldPoint[]; navigationPath?: RobotWorldPoint[]; navigationPathKind?: "PREVIEW" | "ACTIVE_NAV" | null; pickInstruction?: string; showRobot?: boolean; showScan?: boolean; showGrid?: boolean; showPose?: boolean; layerState?: ActiveMapLayers; onLayerToggle?: (layer: keyof ActiveMapLayers) => void; pose: Pose; defaultPickYaw?: number; active: boolean; onPick: (point: Pick<Pose, "x" | "y">, yaw?: number) => void; onConfirmPoint?: (point: Pick<Pose, "x" | "y">, yaw: number) => void; onInteractionModeChange?: (mode: "PAN" | "POINT") => void; pointSelectionResetVersion?: number; onCancelPointSelection?: () => void; errorLogCount?: number; errorLogTone?: "neutral" | "info" | "warning" | "error"; onOpenErrorLog?: () => void; ariaLabel?: string };
type MapInteractionMode = "PAN" | "POINT";
type MapDrag = { pointerId: number; clientX: number; clientY: number; ratioX: number; ratioY: number; start: MapViewportState; moved: boolean; mode: "PAN" | "POINT" | "LEGACY" };
type PointDirectionPreview = { start: { x: number; y: number }; end: { x: number; y: number } };
type PointYawAnchor = { world: { x: number; y: number }; screen: { x: number; y: number } };
type InteractiveMapSnapshot = { map: RobotDetailMapSnapshot; occupancyGrid: Int8Array | null; raster: HTMLCanvasElement; contentKey: string; identityKey: string };
function PosePickerMap({ map, poseMapIdentity = map, robot, scan = null, trajectory = [], navigationPath = [], navigationPathKind = null, pickInstruction, showRobot = true, showScan = true, showGrid = false, showPose = true, layerState, onLayerToggle, pose, defaultPickYaw, active, onPick, onConfirmPoint, onInteractionModeChange, pointSelectionResetVersion, onCancelPointSelection, errorLogCount, errorLogTone = "neutral", onOpenErrorLog, ariaLabel = "Select map frame initial robot position" }: MapProps) {
  const hostRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [size, setSize] = useState({ width: 0, height: 0 });
  const [viewportState, setViewportState] = useState<MapViewportState | null>(null);
  const [isPanning, setIsPanning] = useState(false);
  const [interactionMode, setInteractionMode] = useState<MapInteractionMode>("POINT");
  const [directionPreview, setDirectionPreview] = useState<PointDirectionPreview | null>(null);
  const [pointYawAnchor, setPointYawAnchor] = useState<PointYawAnchor | null>(null);
  const [interactiveMapState, setInteractiveMapState] = useState<InteractiveMapSnapshot | null>(null);
  const [pickFeedback, setPickFeedback] = useState("");
  const drag = useRef<MapDrag | null>(null);
  const suppressClick = useRef(false);
  const suppressClickTimer = useRef<number | null>(null);
  const pickFeedbackTimer = useRef<number | null>(null);
  const panFrame = useRef<number | null>(null);
  const pendingPan = useRef<MapViewportState | null>(null);
  const displayedPose = useStableDisplayedFramePose(robot?.id === map.robot_id ? robot : undefined, poseMapIdentity);
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
    if (showRobot && displayedPose) drawPose(context, transform, displayedPose.x, displayedPose.y, displayedPose.yaw, "#1686d9");
    if (directionPreview) drawDirectionArrow(context, transform, directionPreview);
    context.fillStyle = "#8aa4bf"; context.font = "10px JetBrains Mono, monospace";
    context.fillText(active ? pickInstruction ?? (pointYawAnchor ? "SELECT YAW · CLICK TO CONFIRM" : "CLICK FREE CELL TO SET DESTINATION") : "MAP FRAME · METRES", 10, size.height - 10);
  }, [active, bounds, directionPreview, displayedPose, map, navigationPath, navigationPathKind, pickInstruction, pointYawAnchor, pose, raster, renderMap, scan, showGrid, showPose, showRobot, showScan, size, trajectory, transform]);
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
  return <div ref={hostRef} className={`local-pose-map ${cursorModeClass}${isPanning ? " is-panning" : ""}`}>
    <div className="hmi-map-toolbar" role="group" aria-label="Map controls" data-testid="robot-map-toolbar">
      <button type="button" aria-label="Zoom in" title="Zoom in" onClick={() => updateCamera((current) => zoomMapViewportCamera(current, 1.25))}>+</button>
      <button type="button" aria-label="Zoom out" title="Zoom out" onClick={() => updateCamera((current) => zoomMapViewportCamera(current, 1 / 1.25))}>−</button>
      <button type="button" onClick={() => updateCamera(() => fitMapViewportCamera(size, bounds))}>FIT</button>
      <button type="button" disabled={!displayedPose} onClick={() => {
        if (!displayedPose) return;
        updateCamera((current) => centerMapViewportCamera(current, displayedPose));
      }}>CENTER ROBOT</button>
      {onOpenErrorLog && <button type="button" className={`hmi-error-log-trigger is-${errorLogTone}`} aria-haspopup="dialog"
        data-testid="control-error-log-trigger" onClick={onOpenErrorLog}>
        ERROR LOG{errorLogCount ? ` (${errorLogCount})` : ""}
      </button>}
    </div>
    {layerState && onLayerToggle && <div className="hmi-map-layer-toolbar" role="group" aria-label="Map interaction modes and layers" data-testid="robot-map-layers">
      <button type="button" aria-label="PAN" title="Pan map" aria-pressed={interactionMode === "PAN"}
        className={interactionMode === "PAN" ? "is-active" : ""} data-testid="map-mode-pan"
        onClick={() => { setInteractionMode("PAN"); setPointYawAnchor(null); setDirectionPreview(null); setPickFeedback(""); onInteractionModeChange?.("PAN"); }}>✋ PAN</button>
      <button type="button" aria-label="POINT" title="Select a destination and yaw" aria-pressed={interactionMode === "POINT"}
        className={interactionMode === "POINT" ? "is-active" : ""} data-testid="map-mode-point"
        onClick={() => { setInteractionMode("POINT"); setPickFeedback(""); onInteractionModeChange?.("POINT"); }}>⌖ POINT</button>
      <span className="hmi-map-toolbar-separator" aria-hidden="true" />
      {(["robot", "scan", "path", "trajectory", "grid"] as const).map((layer) => <button
        key={layer} type="button" aria-pressed={layerState[layer]} className={layerState[layer] ? "is-active" : ""}
        onClick={() => onLayerToggle(layer)}>{layerState[layer] ? "✓ " : "□ "}{layer.toUpperCase()}</button>)}
    </div>}
    {active && isPointPickingMode && <div className="hmi-map-pick-feedback" role="status" aria-live="polite" data-testid="map-pick-feedback">
      {pickFeedback || (!interactiveMapSnapshot ? "MAP UPDATING" : "")}
    </div>}
    <canvas ref={canvasRef}
    data-testid={map.map_source === "SLAM_TOOLBOX" ? "slam-map-2d-canvas" : undefined}
    data-occupancy-layer="accumulated-map-snapshot"
    data-scan-layer={showScan && scan ? "low-opacity-current-scan-overlay" : "disabled"}
    data-scan-render-mode="points-only"
    data-navigation-path-point-count={navigationPath.length}
    data-navigation-path-kind={navigationPathKind ?? "none"}
    data-robot-layer={showRobot && displayedPose ? "visible" : "disabled"}
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
    data-pose-source={displayedPose?.pose_source}
    data-render-x={displayedPose?.x} data-render-y={displayedPose?.y} data-render-yaw={displayedPose?.yaw}
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

type VdaDraft = Omit<Vda5050Configuration, "connection_status" | "last_error" | "ignored_orders" | "updated_at"> & { mqtt_password: string };
const editableVdaFields = [
  "enabled", "mqtt_host", "mqtt_port", "mqtt_username", "tls_enabled", "topic_prefix",
  "interface_name", "manufacturer", "serial_number", "protocol_version",
  "mqtt_protocol_version", "allow_task", "auto_reconnect",
  "reconnect_interval", "connection_timeout", "keepalive", "client_id",
] as const;
function vdaSnapshot(draft: VdaDraft) {
  return JSON.stringify(Object.fromEntries(editableVdaFields.map((key) => [key, draft[key]])));
}

function Vda5050Panel({ robotId }: { robotId: string }) {
  const [draft, setDraft] = useState<VdaDraft | null>(null);
  const [baseline, setBaseline] = useState("");
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");
  const [notice, setNotice] = useState("");
  const [connectionStatus, setConnectionStatus] = useState("UNKNOWN");
  const [runtimeError, setRuntimeError] = useState("");
  const [testResult, setTestResult] = useState("");
  useEffect(() => {
    let active = true;
    void getVda5050Configuration(robotId).then((config) => {
      if (!active) return;
      const next = { ...config, mqtt_password: "" } as VdaDraft;
      setDraft(next); setBaseline(vdaSnapshot(next)); setConnectionStatus(config.connection_status); setRuntimeError(config.last_error ?? "");
    }).catch((caught: unknown) => { if (active) setError(caught instanceof Error ? caught.message : "VDA5050 configuration is unavailable"); });
    return () => { active = false; };
  }, [robotId]);
  useEffect(() => {
    let active = true;
    const refreshConnection = () => getVda5050Configuration(robotId).then((config) => {
      if (active) { setConnectionStatus(config.connection_status); setRuntimeError(config.last_error ?? ""); }
    }).catch(() => { if (active) setConnectionStatus("UNAVAILABLE"); });
    const timer = window.setInterval(() => { void refreshConnection(); }, 3000);
    return () => { active = false; window.clearInterval(timer); };
  }, [robotId]);
  const dirty = Boolean(draft && (vdaSnapshot(draft) !== baseline || draft.mqtt_password.length > 0));
  const update = <K extends keyof VdaDraft>(key: K, value: VdaDraft[K]) => setDraft((previous) => previous ? ({ ...previous, [key]: value }) : previous);
  const save = async () => {
    if (!draft) return;
    setBusy(true); setError(""); setNotice(""); setTestResult("");
    try {
      const body = { ...Object.fromEntries(editableVdaFields.map((key) => [key, draft[key]])), mqtt_password: draft.mqtt_password };
      const result = await applyVda5050Configuration(robotId, body);
      const next = { ...result.configuration, mqtt_password: "" } as VdaDraft;
      setDraft(next); setBaseline(vdaSnapshot(next)); setConnectionStatus(result.configuration.connection_status); setRuntimeError(result.configuration.last_error ?? "");
      setNotice(`CONFIGURATION APPLIED · ${result.configuration.connection_status}`);
    } catch (caught) { setError(caught instanceof Error ? caught.message : "VDA5050 configuration was not applied"); }
    finally { setBusy(false); }
  };
  const test = async () => {
    if (!draft) return;
    setBusy(true); setError(""); setNotice(""); setTestResult("");
    try {
      const body = { ...Object.fromEntries(editableVdaFields.map((key) => [key, draft[key]])), mqtt_password: draft.mqtt_password };
      const result = await testVda5050Connection(robotId, body);
      if (result.ok) setTestResult(`CONNECTED · ${result.latency_ms ?? "N/A"} ms · ${result.broker ?? "broker"}`);
      else setTestResult(`FAILED · ${result.error_code ?? "MQTT_ERROR"} · ${result.message ?? "broker connection failed"}`);
    } catch (caught) { setError(caught instanceof Error ? caught.message : "MQTT connection test failed"); }
    finally { setBusy(false); }
  };
  if (!draft) return <SectionFrame><SectionPanel title="VDA5050 CONFIGURATION">{error ? <div className="local-feedback error">{error}</div> : <div className="local-empty">Loading robot scoped configuration…</div>}</SectionPanel></SectionFrame>;

  return <SectionFrame>
    <SectionPanel title="VDA5050 CONFIGURATION" className="local-vda-panel">
      <div className="local-form-grid">
        <div className="local-vda-group">CONNECTION</div>
        <label className="local-field local-check-field"><span>ENABLED</span><input type="checkbox" checked={draft.enabled} onChange={(event) => update("enabled", event.target.checked)} /></label>
        <label className="local-field"><span>MQTT HOST</span><input value={draft.mqtt_host} onChange={(event) => update("mqtt_host", event.target.value)} placeholder="broker.local" /></label>
        <label className="local-field"><span>MQTT PORT</span><input type="number" min={1} max={65535} value={draft.mqtt_port} onChange={(event) => update("mqtt_port", Number(event.target.value))} /></label>
        <label className="local-field"><span>USERNAME</span><input autoComplete="username" value={draft.mqtt_username} onChange={(event) => update("mqtt_username", event.target.value)} /></label>
        <label className="local-field"><span>PASSWORD · SECRET</span><input type="password" autoComplete="new-password" value={draft.mqtt_password} onChange={(event) => update("mqtt_password", event.target.value)} placeholder={draft.password_configured ? "Stored · blank keeps current" : "Not configured"} /></label>
        <label className="local-field local-check-field"><span>TLS</span><input type="checkbox" checked={draft.tls_enabled} onChange={(event) => update("tls_enabled", event.target.checked)} /></label>
        <label className="local-field"><span>MQTT VERSION</span><select value={draft.mqtt_protocol_version} onChange={(event) => update("mqtt_protocol_version", event.target.value as VdaDraft["mqtt_protocol_version"])}><option value="3.1.1">3.1.1</option><option value="5.0">5.0</option></select></label>
        <div className="local-vda-group">PROTOCOL IDENTITY</div>
        <label className="local-field"><span>VDA5050 VERSION</span><select value={draft.protocol_version} onChange={(event) => update("protocol_version", event.target.value as VdaDraft["protocol_version"])}><option value="2.0.0">2.0.0</option><option value="2.1.0">2.1.0</option><option value="3.0.0">3.0.0</option></select></label>
        <label className="local-field"><span>TOPIC PREFIX</span><input value={draft.topic_prefix} onChange={(event) => update("topic_prefix", event.target.value)} /></label>
        <label className="local-field"><span>INTERFACE NAME</span><input value={draft.interface_name} onChange={(event) => update("interface_name", event.target.value)} /></label>
        <label className="local-field"><span>MANUFACTURER</span><input value={draft.manufacturer} onChange={(event) => update("manufacturer", event.target.value)} /></label>
        <label className="local-field"><span>SERIAL NUMBER</span><input value={draft.serial_number} onChange={(event) => update("serial_number", event.target.value)} /></label>
        <label className="local-field"><span>CLIENT ID</span><input value={draft.client_id} onChange={(event) => update("client_id", event.target.value)} placeholder={`waretwin-${robotId}`} /></label>
        <div className="local-vda-group">TASK POLICY</div>
        <label className="local-field local-check-field"><span>ALLOW TASK</span><input type="checkbox" checked={draft.allow_task} onChange={(event) => update("allow_task", event.target.checked)} /></label>
        <div className="local-vda-capability"><span>INSTANT ACTION EXECUTION</span><Status value="NOT IMPLEMENTED" /><small>Subscription and execution are disabled.<br />MQTT CONNECTED does not imply this capability.</small></div>
        <div className="local-vda-group">RUNTIME</div>
        <label className="local-field local-check-field"><span>AUTO RECONNECT</span><input type="checkbox" checked={draft.auto_reconnect} onChange={(event) => update("auto_reconnect", event.target.checked)} /></label>
        <label className="local-field"><span>RECONNECT INTERVAL · s</span><input type="number" min={1} max={300} value={draft.reconnect_interval} onChange={(event) => update("reconnect_interval", Number(event.target.value))} /></label>
        <label className="local-field"><span>CONNECTION TIMEOUT · s</span><input type="number" min={1} max={60} value={draft.connection_timeout} onChange={(event) => update("connection_timeout", Number(event.target.value))} /></label>
        <label className="local-field"><span>KEEPALIVE · s</span><input type="number" min={5} max={3600} value={draft.keepalive} onChange={(event) => update("keepalive", Number(event.target.value))} /></label>
      </div>
      <div className="local-vda-footer">
        <div><span>MQTT STATUS</span><Status value={connectionStatus} />{draft.password_configured && <small>secret configured</small>}</div>
        <div className="local-action-row"><button type="button" disabled={busy} onClick={() => void test()}>TEST CONNECTION</button><button type="button" className="robot-console-primary" disabled={busy || !dirty} onClick={() => void save()}>{busy ? "APPLYING…" : "SAVE & APPLY"}</button></div>
      </div>
      {draft.allow_task ? <p className="local-help">Incoming order messages are subscribed and forwarded to this robot’s authenticated ROS bridge. Disabling Allow Task removes the order subscription and rejects any in-flight order.</p> : <p className="local-help">New VDA5050 orders are blocked for this robot. Telemetry, bridge health, and local manual control remain available.</p>}
      {error && <div className="local-feedback error" role="alert">{error}</div>}
      {notice && <div className="local-feedback ok" role="status">{notice}</div>}
      {testResult && <div className={`local-feedback ${testResult.startsWith("CONNECTED") ? "ok" : "error"}`} role="status">{testResult}</div>}
      {runtimeError && <div className="local-feedback error">{runtimeError}</div>}
    </SectionPanel>
  </SectionFrame>;
}

function DiagnosticsPanel({ robotId, diagnostics, errors, controlOnline, runtimeState, localization, websocketState, mapRevision, lidarStreamDiagnostics, activeLocalMapId, activeLocalMapRevision, localMapSyncStatus, hostStatus }: Props) {
  const [vdaStatus, setVdaStatus] = useState("UNKNOWN");
  const [vdaError, setVdaError] = useState("");
  const [vdaRuntime, setVdaRuntime] = useState<{ enabled: boolean; host: string; port: number; allowTask: boolean; instantActionsSupported: boolean } | null>(null);
  useEffect(() => {
    let active = true;
    const refreshConnection = () => getVda5050Configuration(robotId).then((config) => {
      if (active) {
        setVdaStatus(config.connection_status); setVdaError(config.last_error ?? "");
        setVdaRuntime({ enabled: config.enabled, host: config.mqtt_host, port: config.mqtt_port, allowTask: config.allow_task, instantActionsSupported: config.instant_actions_supported });
      }
    }).catch(() => { if (active) setVdaStatus("UNAVAILABLE"); });
    void refreshConnection();
    const timer = window.setInterval(() => { void refreshConnection(); }, 3000);
    return () => { active = false; window.clearInterval(timer); };
  }, [robotId]);
  const metrics = diagnostics?.metrics ?? {};
  const command = diagnostics?.command_ownership;
  const stream = lidarStreamDiagnostics;
  const slamState = diagnostics?.mapping?.slam_state === "ACTIVE" && diagnostics.mapping.map_live
    ? "LIVE" : diagnostics?.mapping?.slam_state ?? (diagnostics?.slam ? "STARTING" : "INACTIVE");
  const nav2State = diagnostics?.nav2_ready ? "READY" : diagnostics?.nav2 ? "STARTING" : "INACTIVE";
  const cpuLoad = hostStatus?.system?.cpu_load_1m;
  const ramUsed = hostStatus?.system?.memory?.used_percent;
  const systemRows: Array<[string, unknown]> = [
    ["ROS", diagnostics?.ros ? "READY" : "OFFLINE"],
    ["Gazebo", diagnostics?.gazebo ? "READY" : "OFFLINE"],
    ["ROS Bridge", controlOnline ? "CONNECTED" : "DISCONNECTED"],
    ["WebSocket", websocketState],
    ["Controllers", diagnostics?.controller_manager ? "ACTIVE" : "UNKNOWN"],
    ["SLAM", slamState], ["Nav2", nav2State],
    ["LiDAR", diagnostics?.lidar ? "ACTIVE" : "UNAVAILABLE"],
  ];
  const detailRows: Array<[string, unknown]> = [
    ["LOCALIZATION", localization ?? "UNKNOWN"],
    ["ACTIVE LOCAL MAP", activeLocalMapId ?? "NONE REPORTED"],
    ["MAP SYNC", localMapSyncStatus ?? diagnostics?.map_state?.map_sync_status ?? "UNKNOWN"],
    ["CONTROL MODE", command?.active_control_mode ?? "UNKNOWN"],
    ["COMMAND SOURCE", command?.active_command_source ?? "NONE"],
    ["LIDAR STREAM", stream?.source_fps == null ? (diagnostics?.lidar ? "ACTIVE" : "UNAVAILABLE") : valueNumber(stream.source_fps, 1, " Hz")],
    ["ODOMETRY", diagnostics?.mapping?.odom_live ? "RECEIVING" : "WAITING"],
    ["TF", diagnostics?.tf ? "AVAILABLE" : "UNAVAILABLE"],
    ["VDA5050 MQTT", vdaStatus],
    ["E-STOP", command?.estop_active ? "ACTIVE" : "CLEAR"],
  ];
  return <SectionFrame className="hmi-diagnosis-layout">
    <SectionPanel title="SUBSYSTEM STATUS" className="hmi-system-status-panel">
      <div className="hmi-subsystem-table">
        {systemRows.map(([name, value]) => <div key={name}><span>{name}</span><Status value={value} /></div>)}
      </div>
      <div className="hmi-performance-grid">
        <Metric label="CPU LOAD" value={cpuLoad == null ? "N/A" : valueNumber(cpuLoad, 2)} mono />
        <Metric label="RAM USED" value={ramUsed == null ? "N/A" : valueNumber(ramUsed, 1, "%")} mono />
        <Metric label="GAZEBO RTF" value={valueNumber(diagnostics?.gazebo_rtf ?? metrics.gazebo_rtf, 3)} mono />
      </div>
      <details className="hmi-advanced-details"><summary>ADVANCED RUNTIME DETAILS</summary>
        <div className="local-status-grid">
          <Metric label="ROBOT" value={robotId} mono /><Metric label="RUNTIME MODE" value={runtimeState} />
          <Metric label="LOCALIZATION" value={localization} /><Metric label="TF MAP → BASE" value={diagnostics?.tf ? "AVAILABLE" : "UNAVAILABLE"} />
          <Metric label="VDA5050 MQTT" value={vdaStatus} /><Metric label="VDA5050 ENABLED" value={vdaRuntime?.enabled ? "YES" : "NO"} />
          <Metric label="MQTT BROKER" value={vdaRuntime ? `${vdaRuntime.host}:${vdaRuntime.port}` : "N/A"} mono />
          <Metric label="ALLOW TASK" value={vdaRuntime?.allowTask ? "ENABLED" : "BLOCKED"} />
          <Metric label="INSTANT ACTIONS" value={vdaRuntime?.instantActionsSupported ? "SUPPORTED" : "NOT IMPLEMENTED"} />
          <Metric label="MAP REVISION" value={mapRevision ?? "N/A"} mono /><Metric label="LOCAL ACTIVE MAP" value={activeLocalMapId ?? "CANONICAL"} mono />
          <Metric label="LOCAL MAP REVISION" value={activeLocalMapRevision ?? "N/A"} mono /><Metric label="CANONICAL REVISION" value={mapRevision ?? "N/A"} mono />
          <Metric label="MAP SYNC STATUS" value={localMapSyncStatus ?? diagnostics?.map_state?.map_sync_status ?? "UNKNOWN"} />
          <Metric label="CONTROL MODE" value={command?.active_control_mode ?? "UNKNOWN"} />
          <Metric label="ACTIVE COMMAND SOURCE" value={command?.active_command_source ?? "UNKNOWN"} mono />
          <Metric label="LAST COMMAND AGE" value={command?.last_command_age == null ? "N/A" : valueNumber(command.last_command_age, 2, " s")} mono />
          <Metric label="MANUAL SOURCE" value={command?.manual_source_active ? "ACTIVE" : "IDLE"} />
          <Metric label="NAV SOURCE" value={command?.nav_source_active ? "ACTIVE" : "IDLE"} />
          <Metric label="E-STOP" value={command?.estop_active ? "ACTIVE" : "CLEAR"} />
          <Metric label="LIDAR SOURCE FPS" value={stream?.source_fps == null ? "N/A" : valueNumber(stream.source_fps, 2, " Hz")} mono />
          <Metric label="LIDAR WEB OUTPUT FPS" value={stream?.web_output_fps == null ? "N/A" : valueNumber(stream.web_output_fps, 2, " Hz")} mono />
          <Metric label="LIDAR POINT COUNT" value={stream?.point_count ?? "N/A"} mono />
          <Metric label="LIDAR DROPPED FRAMES" value={stream?.dropped_frames ?? "N/A"} mono />
          <Metric label="SIMULATION TIME" value={valueNumber(diagnostics?.simulation_time, 3, " s")} mono />
          <Metric label="WEBSOCKET LATENCY" value={valueNumber(diagnostics?.websocket_latency_ms, 0, " ms")} mono />
          <Metric label="MQTT LAST ERROR" value={vdaError || "NONE"} />
        </div>
      </details>
    </SectionPanel>
    <SectionPanel title="RUNTIME / SENSOR STATUS" className="hmi-diagnosis-ros">
      <div className="hmi-diagnosis-detail-table">
        {detailRows.map(([name, value]) => <div key={name}><span>{name}</span><Status value={value} /></div>)}
      </div>
      <details className="hmi-advanced-details">
        <summary>ADVANCED ROS GRAPH</summary>
        <div className="local-status-grid">
          <Metric label="ACTIVE NODES" value={diagnostics?.nodes?.length ?? 0} mono />
          <Metric label="TOPICS" value={diagnostics?.topics?.length ?? 0} mono />
          <Metric label="CONTROLLERS" value={(diagnostics?.controllers ?? []).map((row) => `${row.name}:${row.state}`).join(" · ") || "N/A"} />
        </div>
        <div className="local-topic-list"><code>/map · /scan · /lidar/points · /lidar/points_filtered</code><code>/tf · /tf_static · /odom · /odometry/filtered</code><code>/navigate_to_pose · /compute_path_to_pose · /cmd_vel_selected</code></div>
      </details>
    </SectionPanel>
    <SectionPanel title="LIVE ERROR LOG" className="hmi-diagnosis-errors">
      {errors.length === 0 ? <div className="local-empty">No runtime errors reported.</div> : errors.map((item, index) => <div className={`robot-detail-error-row ${classForStatus(item.severity)}`} key={`${item.code ?? item.message}-${index}`}><div><b>{item.severity}</b><span>{item.message}</span></div><small>{item.timestamp ?? "N/A"}</small></div>)}
    </SectionPanel>
  </SectionFrame>;
}
