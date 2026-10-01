import { useCallback, useEffect, useMemo, useRef, useState, type MouseEvent, type ReactNode } from "react";
import {
  applyVda5050Configuration,
  getLocalRobotMaps,
  getVda5050Configuration,
  initializeLocalRobotPose,
  loadLocalRobotMap,
  resumeLocalRobotSlamSession,
  saveLocalRobotMap,
  getLocalRuntimeMode,
  requestLocalRuntimeMode,
  setMappingState,
  testVda5050Connection,
  type LocalRobotMap,
  type LocalRuntimeModeStatus,
  type Vda5050Configuration,
} from "../../services/api";
import { wsSetRobotMode } from "../../services/ws";
import type { RobotDetailError, RobotDetailMapSnapshot, RobotDetailScan, RobotLidarStreamDiagnostics, RobotSystemDiagnostics, RobotState, RobotWorldPoint } from "../../schema/twin_state";
import { createWorldTransform, worldToScreen, screenToWorld, type WorldBounds } from "../../layout/coordinates";
import { occupancyRasters } from "./occupancyRaster";

type SectionName = "MAPPING" | "LOCALIZATION" | "VDA5050" | "DIAGNOSTICS";
type Pose = { x: number; y: number; yaw: number };
type Props = {
  section: SectionName;
  robotId: string;
  robot?: RobotState;
  mapSnapshot: RobotDetailMapSnapshot | null;
  scan: RobotDetailScan | null;
  diagnostics: RobotSystemDiagnostics | null;
  errors: RobotDetailError[];
  controlOnline: boolean;
  controlMode: "MANUAL" | "AUTONOMOUS";
  runtimeState: string;
  localization: unknown;
  websocketState: string;
  mapRevision: number | null;
  activeLocalMapId: string | null;
  activeLocalMapRevision: string | null;
  localMapSyncStatus: string | null;
  mappingSessionId?: string | null;
  lidarStreamDiagnostics: RobotLidarStreamDiagnostics | null;
};

function valueText(value: unknown, fallback = "N/A") {
  return value === null || value === undefined || value === "" ? fallback : String(value);
}

function valueNumber(value: unknown, digits = 2, suffix = "") {
  const number = Number(value);
  return Number.isFinite(number) ? `${number.toFixed(digits)}${suffix}` : "N/A";
}

function wait(milliseconds: number) {
  return new Promise<void>((resolve) => window.setTimeout(resolve, milliseconds));
}

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

function SectionFrame({ children }: { children: ReactNode }) {
  return <div className="local-robot-section">{children}</div>;
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

function MappingPanel({ robotId, robot, mapSnapshot, scan, diagnostics, controlOnline, controlMode, runtimeState, mappingSessionId }: Props) {
  const [maps, setMaps] = useState<LocalRobotMap[]>([]);
  const [selected, setSelected] = useState("");
  const [name, setName] = useState("");
  const [mappingState, setMappingStateValue] = useState("UNKNOWN");
  const [mappingDuration, setMappingDuration] = useState(0);
  const [activeLocalMapId, setActiveLocalMapId] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");
  const [notice, setNotice] = useState("");
  const [operationState, setOperationState] = useState("READY");
  const [modeTransition, setModeTransition] = useState<LocalRuntimeModeStatus | null>(null);
  const [trajectory, setTrajectory] = useState<RobotWorldPoint[]>([]);
  const [layers, setLayers] = useState({ robot: true, scan: true, trajectory: true, grid: false });
  const trajectoryRef = useRef<{ mapId: string; points: RobotWorldPoint[]; lastAt: number }>({ mapId: "", points: [], lastAt: 0 });

  const slamMap = runtimeState === "MAPPING" && mapSnapshot?.robot_id === robotId
    && mapSnapshot?.map_source === "SLAM_TOOLBOX"
    && (!mappingSessionId || mapSnapshot.mapping_session_id === mappingSessionId) ? mapSnapshot : null;
  const mapping = diagnostics?.mapping;

  const refresh = useCallback(async () => {
    const result = await getLocalRobotMaps(robotId);
    setMaps(result.maps);
    setMappingStateValue(result.mapping_state);
    setMappingDuration(Number(result.mapping_duration_s) || 0);
    setActiveLocalMapId(result.active_local_map_id);
    if (!selected && result.maps.length) setSelected(result.maps[0].id);
  }, [robotId, selected]);

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

  useEffect(() => {
    let active = true;
    const refreshMode = () => getLocalRuntimeMode(robotId).then((result) => {
      if (active) setModeTransition(result.transition);
    }).catch(() => undefined);
    void refreshMode();
    const timer = window.setInterval(() => { void refreshMode(); }, 2000);
    return () => { active = false; window.clearInterval(timer); };
  }, [robotId]);

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
  const selectedMap = maps.find((map) => map.id === selected) ?? null;
  const resumeSession = () => {
    if (!selectedMap || selectedMap.slam_session_state?.status !== "AVAILABLE") return;
    if (!window.confirm(`Restart ${robotId} in Mapping mode and resume the saved SLAM Toolbox session for ${selectedMap.name}? The simulator will respawn at its configured dock.`)) return;
    setOperationState("RESUMING SLAM SESSION"); setBusy(true); setError(""); setNotice("");
    void (async () => {
      const deadline = Date.now() + 20 * 60 * 1000;
      try {
        let result = await resumeLocalRobotSlamSession(robotId, selectedMap.id);
        while (result.status === "TRANSITIONING" && Date.now() < deadline) {
          if (result.transition) {
            setModeTransition(result.transition);
            if (["ERROR", "ROLLED_BACK"].includes(result.transition.status)) {
              throw new Error(result.transition.message || "Saved SLAM session restart failed");
            }
          }
          if (result.mapping_state) setMappingStateValue(result.mapping_state);
          await wait(1500);
          result = await resumeLocalRobotSlamSession(robotId, selectedMap.id);
        }
        if (result.status !== "RESUMED" || !result.restore_evidence?.passed) {
          throw new Error(Date.now() >= deadline
            ? "Saved SLAM session did not restore before the runtime timeout"
            : result.message || "Live SLAM map did not verify the saved session");
        }
        setMappingStateValue("MAPPING"); setActiveLocalMapId(null);
        const evidence = result.restore_evidence;
        setNotice(`SLAM SESSION RESTORED · ${result.map?.name ?? selectedMap.name} · ${evidence.live_known_cells ?? "—"} live known cells · saved-map overlap ${((evidence.known_overlap_ratio ?? 0) * 100).toFixed(1)}%`);
        await refresh();
        setOperationState("READY");
      } catch (caught) {
        setOperationState("ERROR");
        setError(caught instanceof Error ? caught.message : "Saved SLAM session resume failed");
      } finally { setBusy(false); }
    })();
  };
  const load = () => {
    if (!selected) return;
    setOperationState("LOADING"); setBusy(true); setError(""); setNotice("");
    void (async () => {
      const waitForNavigation = async (requestId: string) => {
        const deadline = Date.now() + 20 * 60 * 1000;
        while (Date.now() < deadline) {
          const status = await getLocalRuntimeMode(robotId);
          setModeTransition(status.transition);
          if (status.transition.request_id && status.transition.request_id !== requestId) {
            throw new Error("Runtime mode transition was replaced by another request");
          }
          if (["ERROR", "ROLLED_BACK"].includes(status.transition.status)) {
            throw new Error(status.transition.message || "Navigation runtime failed its readiness gate");
          }
          if (status.current_mode === "NAVIGATION"
              && status.transition.status === "READY"
              && status.transition.mode?.toLowerCase() === "navigation") return;
          await wait(2000);
        }
        throw new Error("Navigation runtime did not become ready within 20 minutes");
      };

      const waitForManualStop = async () => {
        if (!wsSetRobotMode(robotId, "MANUAL")) {
          throw new Error("Robot control channel is disconnected; Navigation started but map loading is waiting for MANUAL mode");
        }
        const deadline = Date.now() + 30_000;
        while (Date.now() < deadline) {
          const status = await getLocalRobotMaps(robotId);
          if (status.robot_control_mode === "MANUAL" && status.robot_stopped) return;
          await wait(500);
        }
        throw new Error("Navigation started, but the robot did not confirm stopped MANUAL mode for map loading");
      };

      try {
        let result = await loadLocalRobotMap(robotId, selected);
        let manualRequested = false;
        while (result.status === "TRANSITIONING") {
          if (!result.request_id) throw new Error("Map load transition did not include its supervisor request ID");
          setModeTransition(result.transition ?? {
            robot_id: robotId, request_id: result.request_id,
            mode: "navigation", status: "REQUESTED", message: result.message,
          });
          if (result.mapping_state) setMappingStateValue(result.mapping_state);
          await waitForNavigation(result.request_id);
          if (!manualRequested) {
            await waitForManualStop();
            manualRequested = true;
          }
          result = await loadLocalRobotMap(robotId, selected);
          if (result.status === "TRANSITIONING") await wait(1000);
        }
        if (!result.active_map || result.map_sync_status !== "LOCAL_ONLY") {
          throw new Error("ROS did not confirm the selected saved map as the active local navigation map");
        }
        setActiveLocalMapId(result.active_map.id);
        setNotice(`MAP LOADED · ${result.active_map.name} · ${result.message}`);
        await refresh();
        setOperationState("READY");
      } catch (caught) {
        setOperationState("ERROR");
        setError(caught instanceof Error ? caught.message : "Saved map load failed");
      } finally {
        setBusy(false);
      }
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

  const switchRuntimeMode = async () => {
    const target = "NAVIGATION";
    if (!window.confirm(`Switch ${robotId} to ${target}? The selected runtime adapter will transition SLAM/Nav2 after a stopped MANUAL handoff.`)) return;
    setBusy(true); setError(""); setNotice("");
    try {
      const result = await requestLocalRuntimeMode(robotId, target);
      setModeTransition({ robot_id: robotId, request_id: result.request_id, mode: target, status: result.status, message: result.message });
      setNotice(`MODE CHANGE REQUESTED · ${target} · waiting for stack readiness`);
    } catch (caught) { setError(caught instanceof Error ? caught.message : "Runtime mode change was not accepted"); }
    finally { setBusy(false); }
  };

  const isMapping = runtimeState === "MAPPING";
  const paused = mappingState === "PAUSED";
  const startMapping = async () => {
    if (isMapping) {
      changeMapping("start");
      return;
    }
    if (!window.confirm(`Start a fresh SLAM Toolbox mapping session for ${robotId}? The runtime supervisor will switch modes after the MANUAL handoff.`)) return;
    setBusy(true); setError(""); setNotice("");
    try {
      const result = await requestLocalRuntimeMode(robotId, "MAPPING");
      setModeTransition({ robot_id: robotId, request_id: result.request_id, mode: "MAPPING", status: result.status, message: result.message });
      setNotice("MAPPING START REQUESTED · waiting for SLAM Toolbox and mapping readiness");
    } catch (caught) { setError(caught instanceof Error ? caught.message : "Mapping runtime was not accepted"); }
    finally { setBusy(false); }
  };
  useEffect(() => {
    const mapId = `${robotId}:${slamMap?.active_map_id ?? "waiting"}`;
    if (trajectoryRef.current.mapId === mapId) return;
    trajectoryRef.current = { mapId, points: [], lastAt: 0 };
    setTrajectory([]);
  }, [robotId, slamMap?.active_map_id]);
  useEffect(() => {
    if (!slamMap || !robot || !isMapping || paused || !mapping?.tf_valid) return;
    const x = Number(robot.position?.[0]);
    const y = Number(robot.position?.[2]);
    if (!Number.isFinite(x) || !Number.isFinite(y)) return;
    const current = trajectoryRef.current;
    const previous = current.points[current.points.length - 1];
    const now = Date.now();
    if (previous && Math.hypot(x - previous[0], y - previous[1]) < 0.10 && now - current.lastAt < 2000) return;
    const points = [...current.points, [x, y] as RobotWorldPoint].slice(-500);
    trajectoryRef.current = { ...current, points, lastAt: now };
    setTrajectory(points);
  }, [isMapping, mapping?.tf_valid, paused, robot?.position?.[0], robot?.position?.[2], slamMap?.active_map_id]);
  return <SectionFrame>
    <SectionPanel title="MAPPING SESSION" className="local-mapping-state">
      <div className="local-status-grid">
        <Metric label="ROBOT" value={robotId} mono />
        <Metric label="RUNTIME MODE" value={runtimeState} />
        <Metric label="MAPPING STATE" value={isMapping ? mappingState : "INACTIVE · NAVIGATION MODE"} />
        <Metric label="SLAM TOOLBOX" value={mapping?.slam_state ?? "UNKNOWN"} />
        <Metric label="LIVE LIDAR · /scan" value={mapping?.scan_live ? `LIVE · ${valueNumber(mapping.scan_hz, 2, " Hz")} · ${mapping.scan_frame ?? "frame unknown"}` : "WAITING"} />
        <Metric label="ODOMETRY" value={mapping?.odom_live ? `LIVE · ${valueNumber(mapping.odom_hz, 2, " Hz")} · ${mapping.odom_frame ?? "?"} → ${mapping.base_frame ?? "?"}` : "WAITING"} />
        <Metric label="TF · map ← lidar" value={mapping?.tf_lidar_to_map_valid ? "OK" : mapping?.tf_error ?? "WAITING / INVALID"} />
        <Metric label="ACCUMULATED /map" value={mapping?.map_live && slamMap ? "LIVE · SLAM TOOLBOX" : "WAITING FOR SLAM MAP"} />
        <Metric label="MAP UPDATE RATE" value={mapping?.map_live ? `${valueNumber(mapping.map_hz, 2, " Hz")} · v${slamMap?.map_version ?? mapping.map_version ?? "—"}` : "WAITING"} />
        <Metric label="MAP → ODOM OWNER" value={mapping?.map_odom_owner ?? "UNVERIFIED"} />
        <Metric label="WORKFLOW STATE" value={operationState} mono />
        <Metric label="SESSION DURATION" value={`${valueNumber(mappingDuration, 1, " s")}${isMapping && !paused ? " · LIVE" : ""}`} mono />
        <Metric label="MAP SIZE" value={slamMap ? `${slamMap.width} × ${slamMap.height} cells` : "WAITING FOR SLAM MAP"} mono />
        <Metric label="RESOLUTION" value={valueNumber(slamMap?.resolution, 3, " m/cell")} mono />
        <Metric label="MAP ORIGIN" value={slamMap ? `${valueNumber(slamMap.origin.x, 2)}, ${valueNumber(slamMap.origin.y, 2)} m` : "—"} mono />
        <Metric label="KNOWN / EXPLORED CELLS" value={slamMap?.known_cells ?? "—"} mono />
        <Metric label="UNKNOWN CELLS" value={slamMap?.unknown_cells ?? "—"} mono />
        <Metric label="OCCUPIED CELLS" value={slamMap?.occupied_cells ?? "—"} mono />
        <Metric label="FREE CELLS" value={slamMap?.free_cells ?? "—"} mono />
        <Metric label="EXPLORED AREA" value={valueNumber(slamMap?.explored_area_m2, 2, " m²")} mono />
        <Metric label="ROBOT POSE · TF map → base" value={robot ? `${valueNumber(robot.position[0], 2)}, ${valueNumber(robot.position[2], 2)} m · ${valueNumber(robot.heading, 2)} rad` : "WAITING"} mono />
        <Metric label="ACTIVE MAP" value={activeLocalMapId ? `${activeLocalMapId} · r${slamMap?.active_map_revision ?? "—"}` : isMapping ? `${slamMap?.active_map_id ?? "SLAM SESSION WAITING"} · LOCAL ONLY` : "CANONICAL"} mono />
        <Metric label="TRAJECTORY SAMPLES" value={trajectory.length} mono />
        <Metric label="AVAILABLE MAPS" value={maps.length} mono />
      </div>
      <div className="local-action-row">
        {isMapping && <button type="button" disabled={!controlOnline || controlMode !== "MANUAL" || busy || !["READY", "ROLLED_BACK", "ERROR"].includes(modeTransition?.status ?? "")} onClick={() => void switchRuntimeMode()}>SWITCH TO NAVIGATION</button>}
        <button type="button" className="robot-console-primary" disabled={!controlOnline || controlMode !== "MANUAL" || busy || (isMapping && mappingState === "MAPPING")} onClick={() => void startMapping()}>{isMapping && mappingState === "MAPPING" ? "MAPPING ACTIVE" : paused ? "RESUME MAPPING" : "START MAPPING"}</button>
        <button type="button" disabled={!controlOnline || !isMapping || busy || paused} onClick={() => changeMapping("stop")}>STOP MAPPING</button>
        <span className="local-help">The runtime adapter owns SLAM/Nav2 transitions. Confirmed runtime state is shown only after backend/ROS acknowledgement.</span>
      </div>
      {modeTransition && modeTransition.status !== "READY" && <div className={`local-feedback ${["ERROR", "ROLLED_BACK"].includes(modeTransition.status) ? "error" : "warning"}`} role="status">MODE TRANSITION · {modeTransition.status} · {modeTransition.message ?? "waiting for runtime readiness"}</div>}
    </SectionPanel>
    <SectionPanel title="SAVE NAVIGATION MAP + SLAM SESSION">
      <div className="local-form-row">
        <label className="local-field local-field-grow"><span>MAP NAME</span><input value={name} maxLength={64} onChange={(event) => setName(event.target.value)} placeholder="warehouse_floor_1" /></label>
        <button type="button" className="robot-console-primary" disabled={!controlOnline || !isMapping || !paused || !slamMap || busy || !name.trim()} onClick={save}>SAVE MAP</button>
      </div>
      <p className="local-help">Stop Mapping first. Save creates Nav2 YAML + PGM and SLAM Toolbox pose-graph + sensor-data files as separate products. The registry exposes opaque IDs, not disk paths. This remains a local saved map and never promotes or overwrites the canonical Fleet map.</p>
    </SectionPanel>
    <SectionPanel title="AVAILABLE MAPS · THIS ROBOT">
      {maps.length === 0 ? <div className="local-empty">No saved maps for {robotId}.</div> : <div className="local-map-list">
        {maps.map((map) => <button type="button" className={`local-map-row ${selected === map.id ? "is-selected" : ""}`} key={map.id} onClick={() => setSelected(map.id)}>
          <span><b>{map.name}</b><small>{map.created_at} · {map.resolution.toFixed(3)} m/cell · SLAM {map.slam_session_state?.status ?? "NOT_SAVED"}</small></span>
          <small>r{map.revision}</small>
        </button>)}
      </div>}
      <div className="local-action-row">
        <button type="button" disabled={!controlOnline || controlMode !== "MANUAL" || !["MAPPING", "NAVIGATION"].includes(runtimeState) || busy || !selected} onClick={load}>LOAD SAVED MAP FOR NAVIGATION</button>
        <button type="button" disabled={!controlOnline || controlMode !== "MANUAL" || !["MAPPING", "NAVIGATION"].includes(runtimeState) || busy || selectedMap?.slam_session_state?.status !== "AVAILABLE"} onClick={resumeSession}>RESUME SAVED SLAM SESSION</button>
        {activeLocalMapId && <Status value="LOCAL_ONLY · LOCAL NAVIGATION ENABLED" />}
      </div>
      <p className="local-help">LOAD FOR NAVIGATION switches to map_server and loads only YAML + image. RESUME SAVED SLAM SESSION is a separate supervised Mapping restart that restores the pose graph at the configured simulation dock, verifies the old cells on live /map, and then continues mapping. Neither action promotes the map to canonical.</p>
    </SectionPanel>
    <SectionPanel title="ACCUMULATED SLAM MAP · /map + CURRENT /scan">
      <div className="local-map-toggles" role="group" aria-label="Mapping map layers">
        {(["robot", "scan", "trajectory", "grid"] as const).map((layer) => <button
          key={layer} type="button" aria-pressed={layers[layer]} className={layers[layer] ? "is-active" : ""}
          onClick={() => setLayers((current) => ({ ...current, [layer]: !current[layer] }))}>
          {layers[layer] ? "✓ " : "□ "}{layer.toUpperCase()}
        </button>)}
      </div>
      {slamMap ? <PosePickerMap map={slamMap} robot={robot}
        scan={scan?.robot_id === robotId && scan.mapping_session_id === slamMap.mapping_session_id ? scan : null}
        showRobot={layers.robot} showScan={layers.scan} showGrid={layers.grid} showPose={false}
        trajectory={layers.trajectory ? trajectory : []} pose={{
        x: Number(robot?.position?.[0] ?? 0), y: Number(robot?.position?.[2] ?? 0),
        yaw: Number(robot?.heading ?? 0),
      }} active={false} onPick={() => undefined} /> : <div className="local-empty">Waiting for a fresh accumulated SLAM Toolbox /map. Live LiDAR frames are sensor views and are not the warehouse map.</div>}
    </SectionPanel>
    {error && <div className="local-feedback error" role="alert">{error}</div>}
    {notice && <div className="local-feedback ok" role="status">{notice}</div>}
  </SectionFrame>;
}

function LocalizationPanel({ robotId, robot, mapSnapshot, controlOnline, localization }: Props) {
  const current = useMemo<Pose>(() => ({
    x: Number(robot?.position?.[0] ?? 0),
    y: Number(robot?.position?.[2] ?? 0),
    yaw: Number(robot?.heading ?? 0),
  }), [robot?.heading, robot?.position]);
  const [pose, setPose] = useState<Pose>(current);
  const [pickMode, setPickMode] = useState(false);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");
  const [notice, setNotice] = useState("");
  const selectedRobot = useRef(robotId);
  useEffect(() => {
    if (selectedRobot.current === robotId) return;
    selectedRobot.current = robotId;
    setPose(current);
  }, [current, robotId]);
  const update = (key: keyof Pose, value: number) => setPose((previous) => ({ ...previous, [key]: value }));
  const apply = async () => {
    if (!window.confirm(`Initialize ${robotId} at x=${pose.x.toFixed(2)}, y=${pose.y.toFixed(2)}, yaw=${pose.yaw.toFixed(2)} rad?`)) return;
    setBusy(true); setError(""); setNotice("");
    try {
      await initializeLocalRobotPose(robotId, { ...pose, frame_id: "map" });
      setNotice("INITIAL POSE ACCEPTED BY ekf_v30e · waiting for the next localization update");
    } catch (caught) { setError(caught instanceof Error ? caught.message : "Initial pose was not accepted"); }
    finally { setBusy(false); }
  };

  return <SectionFrame>
    <SectionPanel title="LOCALIZATION STATE">
      <div className="local-status-grid">
        <Metric label="X · MAP" value={valueNumber(current.x, 3, " m")} mono />
        <Metric label="Y · MAP" value={valueNumber(current.y, 3, " m")} mono />
        <Metric label="YAW" value={valueNumber(current.yaw, 3, " rad")} mono />
        <Metric label="FRAME" value="map" mono />
        <Metric label="LOCALIZATION" value={localization} />
        <Metric label="OWNER" value="ekf_v30e · map → odom" mono />
      </div>
    </SectionPanel>
    <SectionPanel title="INITIALIZE ROBOT POSE" className="local-pose-editor">
      <div className="local-pose-fields">
        <label className="local-field"><span>X · MAP (m)</span><input type="number" step="0.01" value={pose.x} onChange={(event) => update("x", Number(event.target.value))} /></label>
        <label className="local-field"><span>Y · MAP (m)</span><input type="number" step="0.01" value={pose.y} onChange={(event) => update("y", Number(event.target.value))} /></label>
        <label className="local-field"><span>YAW (rad)</span><input type="number" step="0.01" value={pose.yaw} onChange={(event) => update("yaw", Number(event.target.value))} /></label>
        <div className="local-pose-yaw"><button type="button" onClick={() => update("yaw", pose.yaw - Math.PI / 12)}>YAW −</button><button type="button" onClick={() => update("yaw", pose.yaw + Math.PI / 12)}>YAW +</button></div>
        <button type="button" className={pickMode ? "is-active" : ""} onClick={() => setPickMode((value) => !value)} disabled={!mapSnapshot}>PICK ON MAP</button>
        <button type="button" className="robot-console-primary" disabled={!controlOnline || busy || !Number.isFinite(pose.x + pose.y + pose.yaw)} onClick={() => void apply()}>SET INITIAL POSE</button>
      </div>
      {mapSnapshot ? <PosePickerMap map={mapSnapshot} robot={robot} pose={pose} active={pickMode} onPick={(point) => setPose((old) => ({ ...old, ...point }))} /> : <div className="local-empty">Waiting for the robot scoped ROS map snapshot.</div>}
      <p className="local-help">The pose is applied through the current authoritative robot_localization EKF service. It changes localization and does not teleport the robot.</p>
    </SectionPanel>
    {error && <div className="local-feedback error" role="alert">{error}</div>}
    {notice && <div className="local-feedback ok" role="status">{notice}</div>}
  </SectionFrame>;
}

type MapProps = { map: RobotDetailMapSnapshot; robot?: RobotState; scan?: RobotDetailScan | null; trajectory?: RobotWorldPoint[]; showRobot?: boolean; showScan?: boolean; showGrid?: boolean; showPose?: boolean; pose: Pose; active: boolean; onPick: (point: Pick<Pose, "x" | "y">) => void };
function PosePickerMap({ map, robot, scan = null, trajectory = [], showRobot = true, showScan = true, showGrid = false, showPose = true, pose, active, onPick }: MapProps) {
  const hostRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [size, setSize] = useState({ width: 0, height: 0 });
  const [rasterState, setRasterState] = useState<{ map: RobotDetailMapSnapshot; raster: HTMLCanvasElement } | null>(() => {
    const raster = occupancyRasters.peek(map);
    return raster ? { map, raster } : null;
  });
  useEffect(() => {
    let cancelled = false;
    const cached = occupancyRasters.peek(map);
    if (cached) setRasterState({ map, raster: cached });
    else setRasterState((previous) => previous?.map.robot_id === map.robot_id
      && previous.map.active_map_id === map.active_map_id ? previous : null);
    void occupancyRasters.get(map).then((raster) => {
      if (!cancelled && raster) setRasterState({ map, raster });
    });
    return () => { cancelled = true; };
  }, [map]);
  const raster = rasterState?.raster ?? null;
  const rasterMap = rasterState?.map ?? map;
  const bounds = useMemo<WorldBounds>(() => {
    const yaw = map.origin.yaw;
    const width = map.width * map.resolution, height = map.height * map.resolution;
    const corners = [[0, 0], [width, 0], [width, height], [0, height]].map(([x, y]) => ({
      x: map.origin.x + x * Math.cos(yaw) - y * Math.sin(yaw),
      y: map.origin.y + x * Math.sin(yaw) + y * Math.cos(yaw),
    }));
    return { minX: Math.min(...corners.map((p) => p.x)), maxX: Math.max(...corners.map((p) => p.x)), minY: Math.min(...corners.map((p) => p.y)), maxY: Math.max(...corners.map((p) => p.y)) };
  }, [map]);
  const transform = useMemo(() => createWorldTransform(size, bounds, 1, null, 20), [bounds, size]);
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
    context.fillStyle = "#07101c"; context.fillRect(0, 0, size.width, size.height);
    if (showGrid) drawMappingGrid(context, size.width, size.height, transform, bounds);
    if (raster) {
      const origin = worldToScreen({ x: rasterMap.origin.x, y: rasterMap.origin.y }, transform);
      context.save(); context.translate(origin.x, origin.y); context.rotate(-rasterMap.origin.yaw);
      context.scale(transform.scale * rasterMap.resolution, -transform.scale * rasterMap.resolution);
      context.imageSmoothingEnabled = false; context.drawImage(raster, 0, -rasterMap.height); context.restore();
    }
    if (trajectory.length > 1) {
      context.beginPath();
      trajectory.forEach(([x, y], index) => {
        const point = worldToScreen({ x, y }, transform);
        if (index === 0) context.moveTo(point.x, point.y); else context.lineTo(point.x, point.y);
      });
      context.strokeStyle = "#f5a64a"; context.lineWidth = 2; context.globalAlpha = 0.85; context.stroke(); context.globalAlpha = 1;
    }
    const currentRobot = robot?.id === map.robot_id ? robot : undefined;
    if (showScan && scan?.frame_id === map.frame_id) {
      const origin = scan.sensor_pose ?? (currentRobot ? { x: currentRobot.position[0], y: currentRobot.position[2], yaw: currentRobot.heading } : null);
      context.fillStyle = "#27e0d0"; context.strokeStyle = "rgba(39,224,208,.16)"; context.lineWidth = 1;
      if (origin) {
        const center = worldToScreen({ x: origin.x, y: origin.y }, transform);
        context.beginPath();
        for (const [x, y] of scan.points) { const p = worldToScreen({ x, y }, transform); context.moveTo(center.x, center.y); context.lineTo(p.x, p.y); }
        context.stroke();
      }
      for (const [x, y] of scan.points) { const p = worldToScreen({ x, y }, transform); context.fillRect(p.x - 1.5, p.y - 1.5, 3, 3); }
    }
    if (showRobot && currentRobot) drawPose(context, transform, currentRobot.position[0], currentRobot.position[2], currentRobot.heading, "#42dfd2");
    if (showPose) drawPose(context, transform, pose.x, pose.y, pose.yaw, "#f6cf4f");
    context.fillStyle = "#8aa4bf"; context.font = "10px JetBrains Mono, monospace";
    context.fillText(active ? "CLICK TO SET XY · YAW CONTROLS BELOW" : "MAP FRAME · METRES", 10, size.height - 10);
  }, [active, bounds, map, pose, raster, rasterMap, robot, scan, showGrid, showPose, showRobot, showScan, size, trajectory, transform]);
  const click = (event: MouseEvent<HTMLCanvasElement>) => {
    if (!active) return;
    const rect = event.currentTarget.getBoundingClientRect();
    onPick(screenToWorld({ x: event.clientX - rect.left, y: event.clientY - rect.top }, transform));
  };
  return <div ref={hostRef} className={`local-pose-map ${active ? "is-picking" : ""}`}><canvas ref={canvasRef} onClick={click} aria-label="Select map frame initial robot position" /></div>;
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
        <div className="local-vda-capability"><span>INSTANT ACTION EXECUTION</span><Status value="NOT IMPLEMENTED" /><small>Subscription and execution are disabled; MQTT CONNECTED does not imply this capability.</small></div>
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
      {draft.allow_task ? <p className="local-help">Incoming order messages are subscribed and forwarded to this robot’s authenticated ROS bridge. Disabling Allow Task removes the order subscription and rejects any in-flight order.</p> : <p className="local-help">New VDA5050 orders are blocked for this robot. Telemetry, bridge health, and authorized local manual control remain available.</p>}
      {error && <div className="local-feedback error" role="alert">{error}</div>}
      {notice && <div className="local-feedback ok" role="status">{notice}</div>}
      {testResult && <div className={`local-feedback ${testResult.startsWith("CONNECTED") ? "ok" : "error"}`} role="status">{testResult}</div>}
      {runtimeError && <div className="local-feedback error">{runtimeError}</div>}
    </SectionPanel>
  </SectionFrame>;
}

function DiagnosticsPanel({ robotId, diagnostics, errors, controlOnline, runtimeState, localization, websocketState, mapRevision, lidarStreamDiagnostics, activeLocalMapId, activeLocalMapRevision, localMapSyncStatus }: Props) {
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
  return <SectionFrame>
    <SectionPanel title="ROBOT RUNTIME DIAGNOSTICS" className="local-diagnostics-grid">
      <div className="local-status-grid">
        <Metric label="ROS" value={diagnostics?.ros ? "ONLINE" : "OFFLINE"} />
        <Metric label="GAZEBO" value={diagnostics?.gazebo ? "ONLINE" : "OFFLINE"} />
        <Metric label="ROS BRIDGE" value={controlOnline ? "CONNECTED · R01" : "DISCONNECTED"} />
        <Metric label="WEBSOCKET" value={websocketState} />
        <Metric label="CONTROLLER MANAGER" value={diagnostics?.controller_manager ? "ACTIVE" : "UNKNOWN"} />
        <Metric label="NAV2" value={diagnostics?.nav2 ? runtimeState === "NAVIGATION" ? "ACTIVE" : "AVAILABLE" : "INACTIVE"} />
        <Metric label="SLAM TOOLBOX" value={diagnostics?.slam ? runtimeState === "MAPPING" ? "ACTIVE" : "AVAILABLE" : "INACTIVE"} />
        <Metric label="LOCALIZATION" value={localization} />
        <Metric label="TF MAP → BASE" value={diagnostics?.tf ? "AVAILABLE" : "UNAVAILABLE"} />
        <Metric label="LIDAR" value={diagnostics?.lidar ? "ACTIVE" : "UNAVAILABLE"} />
        <Metric label="MQTT / VDA5050" value={vdaStatus} />
        <Metric label="VDA5050 ENABLED" value={vdaRuntime?.enabled ? "YES" : "NO"} />
        <Metric label="MQTT BROKER" value={vdaRuntime ? `${vdaRuntime.host}:${vdaRuntime.port}` : "N/A"} mono />
        <Metric label="ALLOW TASK" value={vdaRuntime?.allowTask ? "ENABLED" : "BLOCKED"} />
        <Metric label="INSTANT ACTIONS" value={vdaRuntime?.instantActionsSupported ? "SUPPORTED" : "NOT IMPLEMENTED"} />
        <Metric label="MAP REVISION" value={mapRevision ?? "N/A"} mono />
        <Metric label="LOCAL ACTIVE MAP" value={activeLocalMapId ?? "CANONICAL"} mono />
        <Metric label="LOCAL MAP REVISION" value={activeLocalMapRevision ?? "N/A"} mono />
        <Metric label="CANONICAL REVISION" value={mapRevision ?? "N/A"} mono />
        <Metric label="MAP SYNC STATUS" value={localMapSyncStatus ?? diagnostics?.map_state?.map_sync_status ?? "UNKNOWN"} />
        <Metric label="CONTROL MODE" value={command?.active_control_mode ?? "UNKNOWN"} />
        <Metric label="ACTIVE COMMAND SOURCE" value={command?.active_command_source ?? "UNKNOWN"} mono />
        <Metric label="LAST COMMAND AGE" value={command?.last_command_age == null ? "N/A" : valueNumber(command.last_command_age, 2, " s")} mono />
        <Metric label="MANUAL SOURCE" value={command?.manual_source_active ? "ACTIVE" : "IDLE"} />
        <Metric label="NAV SOURCE" value={command?.nav_source_active ? "ACTIVE" : "IDLE"} />
        <Metric label="TAG SOURCE" value={command?.tag_source_active ? "ACTIVE" : "IDLE"} />
        <Metric label="E-STOP" value={command?.estop_active ? "ACTIVE" : "CLEAR"} />
        <Metric label="LIDAR SOURCE FPS" value={stream?.source_fps == null ? "N/A" : valueNumber(stream.source_fps, 2, " Hz")} mono />
        <Metric label="LIDAR WEB OUTPUT FPS" value={stream?.web_output_fps == null ? "N/A" : valueNumber(stream.web_output_fps, 2, " Hz")} mono />
        <Metric label="LIDAR POINT COUNT" value={stream?.point_count ?? "N/A"} mono />
        <Metric label="LIDAR DROPPED FRAMES" value={stream?.dropped_frames ?? "N/A"} mono />
        <Metric label="SIMULATION TIME" value={valueNumber(diagnostics?.simulation_time, 3, " s")} mono />
        <Metric label="GAZEBO RTF" value={valueNumber(diagnostics?.gazebo_rtf ?? metrics.gazebo_rtf, 3)} mono />
        <Metric label="WEBSOCKET LATENCY" value={valueNumber(diagnostics?.websocket_latency_ms, 0, " ms")} mono />
        <Metric label="MQTT LAST ERROR" value={vdaError || "NONE"} />
      </div>
      <div className="local-diagnostic-runtime"><Metric label="SELECTED ROBOT" value={robotId} mono /><Metric label="RUNTIME MODE" value={runtimeState} /></div>
    </SectionPanel>
    <SectionPanel title="LIVE ROS GRAPH SNAPSHOT">
      <div className="local-status-grid">
        <Metric label="ACTIVE NODES" value={diagnostics?.nodes?.length ?? 0} mono />
        <Metric label="TOPICS" value={diagnostics?.topics?.length ?? 0} mono />
        <Metric label="CONTROLLERS" value={(diagnostics?.controllers ?? []).map((row) => `${row.name}:${row.state}`).join(" · ") || "N/A"} />
      </div>
      <div className="local-topic-list"><code>/map · /scan · /lidar/points · /lidar/points_filtered</code><code>/tf · /tf_static · /odom · /odometry/filtered</code><code>/navigate_to_pose · /compute_path_to_pose · /cmd_vel_selected</code></div>
    </SectionPanel>
    <SectionPanel title="LAST REPORTED ERRORS">
      {errors.length === 0 ? <div className="local-empty">No runtime errors reported.</div> : errors.map((item, index) => <div className={`robot-detail-error-row ${classForStatus(item.severity)}`} key={`${item.code ?? item.message}-${index}`}><div><b>{item.severity}</b><span>{item.message}</span></div><small>{item.timestamp ?? "N/A"}</small></div>)}
    </SectionPanel>
  </SectionFrame>;
}
