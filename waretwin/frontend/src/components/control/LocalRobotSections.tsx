import { useCallback, useEffect, useMemo, useRef, useState, type MouseEvent, type ReactNode } from "react";
import {
  applyVda5050Configuration,
  getLocalRobotMaps,
  getVda5050Configuration,
  initializeLocalRobotPose,
  loadLocalRobotMap,
  saveLocalRobotMap,
  getLocalRuntimeMode,
  requestLocalRuntimeMode,
  setMappingState,
  testVda5050Connection,
  type LocalRobotMap,
  type LocalRuntimeModeStatus,
  type Vda5050Configuration,
} from "../../services/api";
import type { RobotDetailError, RobotDetailMapSnapshot, RobotSystemDiagnostics, RobotState } from "../../schema/twin_state";
import { createWorldTransform, worldToScreen, screenToWorld, type WorldBounds } from "../../layout/coordinates";

type SectionName = "MAPPING" | "LOCALIZATION" | "VDA5050" | "DIAGNOSTICS";
type Pose = { x: number; y: number; yaw: number };
type Props = {
  section: SectionName;
  robotId: string;
  robot?: RobotState;
  mapSnapshot: RobotDetailMapSnapshot | null;
  diagnostics: RobotSystemDiagnostics | null;
  errors: RobotDetailError[];
  controlOnline: boolean;
  controlMode: "MANUAL" | "AUTONOMOUS";
  runtimeState: string;
  localization: unknown;
  websocketState: string;
  mapRevision: number | null;
};

function valueText(value: unknown, fallback = "N/A") {
  return value === null || value === undefined || value === "" ? fallback : String(value);
}

function valueNumber(value: unknown, digits = 2, suffix = "") {
  const number = Number(value);
  return Number.isFinite(number) ? `${number.toFixed(digits)}${suffix}` : "N/A";
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

function MappingPanel({ robotId, robot, mapSnapshot, controlOnline, controlMode, runtimeState }: Props) {
  const [maps, setMaps] = useState<LocalRobotMap[]>([]);
  const [selected, setSelected] = useState("");
  const [name, setName] = useState("");
  const [mappingState, setMappingStateValue] = useState("UNKNOWN");
  const [mappingDuration, setMappingDuration] = useState(0);
  const [activeLocalMapId, setActiveLocalMapId] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");
  const [notice, setNotice] = useState("");
  const [modeTransition, setModeTransition] = useState<LocalRuntimeModeStatus | null>(null);

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

  const run = async (action: () => Promise<unknown>, success: (result: unknown) => string) => {
    setBusy(true); setError(""); setNotice("");
    try { const result = await action(); setNotice(success(result)); await refresh(); }
    catch (caught) { setError(caught instanceof Error ? caught.message : "Robot map operation failed"); }
    finally { setBusy(false); }
  };

  const save = () => void run(() => saveLocalRobotMap(robotId, name), (raw) => {
    const result = raw as { map: LocalRobotMap };
    setName(""); setSelected(result.map.id);
    return `SAVE SUCCESS · ${result.map.name} · revision ${result.map.revision}`;
  });
  const load = () => void run(() => loadLocalRobotMap(robotId, selected), (raw) => {
    const result = raw as { active_map: LocalRobotMap; message: string };
    setActiveLocalMapId(result.active_map.id);
    return `MAP LOADED · ${result.active_map.name} · ${result.message}`;
  });
  const changeMapping = (action: "start" | "stop") => void run(
    () => setMappingState(robotId, action),
    (raw) => {
      const result = raw as { mapping_state: string };
      setMappingStateValue(result.mapping_state);
      return `MAPPING ${result.mapping_state}`;
    },
  );

  const switchRuntimeMode = async () => {
    const target = runtimeState === "MAPPING" ? "NAVIGATION" : "MAPPING";
    if (!window.confirm(`Switch ${robotId} to ${target}? ROS/Gazebo will restart and the simulated robot will respawn at its configured start pose.`)) return;
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
  return <SectionFrame>
    <SectionPanel title="MAPPING SESSION" className="local-mapping-state">
      <div className="local-status-grid">
        <Metric label="ROBOT" value={robotId} mono />
        <Metric label="RUNTIME MODE" value={runtimeState} />
        <Metric label="MAPPING STATE" value={isMapping ? mappingState : "INACTIVE · NAVIGATION MODE"} />
        <Metric label="SESSION DURATION" value={`${valueNumber(mappingDuration, 1, " s")}${isMapping && !paused ? " · LIVE" : ""}`} mono />
        <Metric label="MAP SIZE" value={mapSnapshot ? `${mapSnapshot.width} × ${mapSnapshot.height} cells` : "WAITING FOR MAP"} mono />
        <Metric label="RESOLUTION" value={valueNumber(mapSnapshot?.resolution, 3, " m/cell")} mono />
        <Metric label="ACTIVE LOCAL MAP" value={activeLocalMapId ?? "CANONICAL"} mono />
        <Metric label="AVAILABLE MAPS" value={maps.length} mono />
      </div>
      <div className="local-action-row">
        <button type="button" disabled={!controlOnline || controlMode !== "MANUAL" || busy || !["READY", "ROLLED_BACK", "ERROR"].includes(modeTransition?.status ?? "")} onClick={() => void switchRuntimeMode()}>{runtimeState === "MAPPING" ? "SWITCH TO NAVIGATION" : "START MAPPING MODE"}</button>
        <button type="button" className="robot-console-primary" disabled={!controlOnline || !isMapping || busy || !paused} onClick={() => changeMapping("start")}>{paused ? "RESUME MAPPING" : "MAPPING ACTIVE"}</button>
        <button type="button" disabled={!controlOnline || !isMapping || busy || paused} onClick={() => changeMapping("stop")}>STOP MAPPING</button>
        <span className="local-help">Mode changes restart ROS/Gazebo after a MANUAL stop; the simulated robot returns to its configured start pose.</span>
      </div>
      {modeTransition && modeTransition.status !== "READY" && <div className={`local-feedback ${["ERROR", "ROLLED_BACK"].includes(modeTransition.status) ? "error" : "warning"}`} role="status">MODE TRANSITION · {modeTransition.status} · {modeTransition.message ?? "waiting for ROS/Gazebo readiness"}</div>}
    </SectionPanel>
    <SectionPanel title="SAVE NAV2 MAP">
      <div className="local-form-row">
        <label className="local-field local-field-grow"><span>MAP NAME</span><input value={name} maxLength={64} onChange={(event) => setName(event.target.value)} placeholder="warehouse_floor_1" /></label>
        <button type="button" className="robot-console-primary" disabled={!controlOnline || !isMapping || busy || !name.trim()} onClick={save}>SAVE MAP</button>
      </div>
      <p className="local-help">SLAM Toolbox saves the live occupancy grid. The backend registers the map only after its YAML and image files exist.</p>
    </SectionPanel>
    <SectionPanel title="AVAILABLE MAPS · THIS ROBOT">
      {maps.length === 0 ? <div className="local-empty">No saved maps for {robotId}.</div> : <div className="local-map-list">
        {maps.map((map) => <button type="button" className={`local-map-row ${selected === map.id ? "is-selected" : ""}`} key={map.id} onClick={() => setSelected(map.id)}>
          <span><b>{map.name}</b><small>{map.created_at} · {map.resolution.toFixed(3)} m/cell</small></span>
          <small>r{map.revision}</small>
        </button>)}
      </div>}
      <div className="local-action-row">
        <button type="button" disabled={!controlOnline || controlMode !== "MANUAL" || runtimeState !== "NAVIGATION" || busy || !selected} onClick={load}>LOAD MAP INTO NAV2</button>
        {activeLocalMapId && <Status value="MAP OUT OF SYNC · GOALS BLOCKED" />}
      </div>
      <p className="local-help">Loading changes the live Nav2 map. Goals stay blocked until the selected map is published into the canonical warehouse map bundle.</p>
    </SectionPanel>
    <SectionPanel title="LIVE MAP PREVIEW">
      {mapSnapshot ? <PosePickerMap map={mapSnapshot} robot={robot} pose={{
        x: Number(robot?.position?.[0] ?? 0), y: Number(robot?.position?.[2] ?? 0),
        yaw: Number(robot?.heading ?? 0),
      }} active={false} onPick={() => undefined} /> : <div className="local-empty">Waiting for the selected robot’s ROS occupancy grid.</div>}
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
      <p className="local-help">The pose is applied through the current authoritative robot_localization EKF service. This does not move the Gazebo entity.</p>
    </SectionPanel>
    {error && <div className="local-feedback error" role="alert">{error}</div>}
    {notice && <div className="local-feedback ok" role="status">{notice}</div>}
  </SectionFrame>;
}

type MapProps = { map: RobotDetailMapSnapshot; robot?: RobotState; pose: Pose; active: boolean; onPick: (point: Pick<Pose, "x" | "y">) => void };
function PosePickerMap({ map, robot, pose, active, onPick }: MapProps) {
  const hostRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [size, setSize] = useState({ width: 0, height: 0 });
  const raster = useMemo(() => {
    if (typeof document === "undefined") return null;
    const canvas = document.createElement("canvas");
    canvas.width = map.width; canvas.height = map.height;
    const context = canvas.getContext("2d");
    if (!context) return null;
    const image = context.createImageData(map.width, map.height);
    for (let row = 0; row < map.height; row += 1) for (let column = 0; column < map.width; column += 1) {
      const value = map.data[row * map.width + column] ?? -1;
      const index = ((map.height - row - 1) * map.width + column) * 4;
      if (value < 0) { image.data[index + 3] = 0; continue; }
      const wall = value >= 65;
      image.data[index] = wall ? 226 : 35;
      image.data[index + 1] = wall ? 92 : 75;
      image.data[index + 2] = wall ? 92 : 99;
      image.data[index + 3] = wall ? 230 : 105;
    }
    context.putImageData(image, 0, 0);
    return canvas;
  }, [map]);
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
    if (raster) {
      const origin = worldToScreen({ x: map.origin.x, y: map.origin.y }, transform);
      context.save(); context.translate(origin.x, origin.y); context.rotate(-map.origin.yaw);
      context.scale(transform.scale * map.resolution, -transform.scale * map.resolution);
      context.imageSmoothingEnabled = false; context.drawImage(raster, 0, -map.height); context.restore();
    }
    if (robot) drawPose(context, transform, robot.position[0], robot.position[2], robot.heading, "#42dfd2");
    drawPose(context, transform, pose.x, pose.y, pose.yaw, "#f6cf4f");
    context.fillStyle = "#8aa4bf"; context.font = "10px JetBrains Mono, monospace";
    context.fillText(active ? "CLICK TO SET XY · YAW CONTROLS BELOW" : "MAP FRAME · METRES", 10, size.height - 10);
  }, [active, map, pose, raster, robot, size, transform]);
  const click = (event: MouseEvent<HTMLCanvasElement>) => {
    if (!active) return;
    const rect = event.currentTarget.getBoundingClientRect();
    onPick(screenToWorld({ x: event.clientX - rect.left, y: event.clientY - rect.top }, transform));
  };
  return <div ref={hostRef} className={`local-pose-map ${active ? "is-picking" : ""}`}><canvas ref={canvasRef} onClick={click} aria-label="Select map frame initial robot position" /></div>;
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
  "mqtt_protocol_version", "allow_task", "allow_instant_actions", "auto_reconnect",
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
        <label className="local-field local-check-field"><span>ALLOW INSTANT ACTIONS</span><input type="checkbox" checked={draft.allow_instant_actions} onChange={(event) => update("allow_instant_actions", event.target.checked)} /></label>
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

function DiagnosticsPanel({ robotId, diagnostics, errors, controlOnline, runtimeState, localization, websocketState, mapRevision }: Props) {
  const [vdaStatus, setVdaStatus] = useState("UNKNOWN");
  const [vdaError, setVdaError] = useState("");
  useEffect(() => {
    let active = true;
    const refreshConnection = () => getVda5050Configuration(robotId).then((config) => {
      if (active) { setVdaStatus(config.connection_status); setVdaError(config.last_error ?? ""); }
    }).catch(() => { if (active) setVdaStatus("UNAVAILABLE"); });
    void refreshConnection();
    const timer = window.setInterval(() => { void refreshConnection(); }, 3000);
    return () => { active = false; window.clearInterval(timer); };
  }, [robotId]);
  const metrics = diagnostics?.metrics ?? {};
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
        <Metric label="MAP REVISION" value={mapRevision ?? "N/A"} mono />
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
      <div className="local-topic-list"><code>/map · /scan · /lidar/points · /lidar/points_filtered</code><code>/tf · /tf_static · /odom · /odometry/filtered</code><code>/navigate_to_pose · /compute_path_to_pose</code></div>
    </SectionPanel>
    <SectionPanel title="LAST REPORTED ERRORS">
      {errors.length === 0 ? <div className="local-empty">No runtime errors reported.</div> : errors.map((item, index) => <div className={`robot-detail-error-row ${classForStatus(item.severity)}`} key={`${item.code ?? item.message}-${index}`}><div><b>{item.severity}</b><span>{item.message}</span></div><small>{item.timestamp ?? "N/A"}</small></div>)}
    </SectionPanel>
  </SectionFrame>;
}
