import { Component, useCallback, useEffect, useMemo, useRef, useState, type ErrorInfo, type ReactNode } from "react";
import { Sidebar } from "../shell/Sidebar";
import { MapView2D } from "../views/MapView2D";
import { useStore, type TagGraph } from "../../state/store";
import type { TwinState } from "../../schema/twin_state";
import { clearEmergencyStop, emergencyStop, missionAction, navigationApi, startTagMission } from "../../services/api";
import { wsManualCommand, wsSetRobotMode, type ManualAction } from "../../services/ws";

const EMPTY_ROBOTS: TwinState["robots"] = {};
const EMPTY_TAGS: TagGraph["tags"] = [];
const EMPTY_TOPICS: string[] = [];
const EMPTY_ROUTE: number[] = [];
const EMPTY_LOCALIZATION = { state: "UNANCHORED", lastTagId: null, expectedTagId: null, tagVisible: false, lastTagSeenAt: null };
const EMPTY_DETECTION = { visible: false, tagId: null, offsetX: null, offsetY: null, yaw: null, timestamp: null };

class RobotControlBoundary extends Component<{ children: ReactNode }, { error: Error | null }> {
  state = { error: null as Error | null };
  static getDerivedStateFromError(error: Error) { return { error }; }
  componentDidCatch(error: Error, info: ErrorInfo) { console.error("[RobotControl] render error", error, info); }
  render() {
    if (!this.state.error) return this.props.children;
    return (
      <div className="min-h-screen bg-[#05080f] p-10 text-slate-200" role="alert">
        <div className="mx-auto max-w-3xl rounded-2xl border border-rose-500/20 bg-[#0b111c] p-6 shadow-2xl">
          <h1 className="text-xl font-semibold text-white">Robot Control could not be rendered</h1>
          <pre className="mt-4 overflow-auto rounded-xl bg-black/30 p-4 text-xs leading-5 text-rose-200">{this.state.error.message}</pre>
        </div>
      </div>
    );
  }
}

class MapBoundary extends Component<{ children: ReactNode }, { error: Error | null }> {
  state = { error: null as Error | null };
  static getDerivedStateFromError(error: Error) { return { error }; }
  componentDidCatch(error: Error, info: ErrorInfo) { console.error("[RobotControl] map render error", error, info); }
  render() { return this.state.error ? <div className="control-map-error">Warehouse map unavailable: {this.state.error.message}</div> : this.props.children; }
}

const MANUAL_BUTTONS: Array<{ action: ManualAction; label: string; title: string }> = [
  { action: "FORWARD", label: "▲", title: "Forward (W / ↑)" },
  { action: "LEFT", label: "◀", title: "Strafe left (A / ←)" },
  { action: "STOP", label: "■", title: "Stop" },
  { action: "RIGHT", label: "▶", title: "Strafe right (D / →)" },
  { action: "BACKWARD", label: "▼", title: "Backward (S / ↓)" },
  { action: "ROTATE_LEFT", label: "↺", title: "Rotate left (Q)" },
  { action: "ROTATE_RIGHT", label: "↻", title: "Rotate right (E)" },
];

function errorText(error: unknown): string {
  return error instanceof Error ? error.message : "Request failed";
}

function RobotControlContent() {
  // Read raw Zustand values only. Any fallback that allocates an array/object
  // inside a selector makes useSyncExternalStore see a new snapshot each render.
  const twin = useStore((state) => state.twin);
  const selected = useStore((state) => state.selectedRobot);
  const select = useStore((state) => state.select);
  const mission = useStore((state) => state.tagNavigation);
  const target = useStore((state) => state.targetTagId);
  const setTarget = useStore((state) => state.setTargetTagId);
  const rawLocalization = useStore((state) => state.localization);
  const rawDetection = useStore((state) => state.tagDetection);
  const setGraph = useStore((state) => state.setTagGraph);
  const ros = useStore((state) => state.rosConnected);
  const runtimeMode = useStore((state) => state.runtimeMode);
  const diagnostics = useStore((state) => state.rosDiagnostics);
  const websocketState = useStore((state) => state.websocketState);
  const tagGraph = useStore((state) => state.tagGraph);

  const robots = twin?.robots && typeof twin.robots === "object" && !Array.isArray(twin.robots) ? twin.robots : EMPTY_ROBOTS;
  const localization = rawLocalization ?? EMPTY_LOCALIZATION;
  const detection = rawDetection ?? EMPTY_DETECTION;
  const tags = Array.isArray(tagGraph?.tags) ? tagGraph.tags : EMPTY_TAGS;
  const measuredTopics = Array.isArray(diagnostics?.topics) ? diagnostics.topics : EMPTY_TOPICS;
  const missionRoute = Array.isArray(mission?.route) ? mission.route : EMPTY_ROUTE;
  const missionProgress = typeof mission?.progress_percent === "number" ? mission.progress_percent : 0;

  const [collapsed, setCollapsed] = useState(false);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");
  const [controlMode, setControlMode] = useState<"MANUAL" | "AUTONOMOUS">("AUTONOMOUS");
  const manualTimer = useRef<number | null>(null);
  const manualActive = useRef(false);
  const previousRobot = useRef<string | null>(null);
  const robotIds = useMemo(() => Object.keys(robots), [robots]);
  const robotId = selected ?? robotIds[0] ?? "";
  const controlOnline = runtimeMode !== "LOCAL_SIM" && ros && websocketState === "CONNECTED";
  const missionControlAvailable = runtimeMode === "LOCAL_SIM" || controlOnline;
  const odomReady = Boolean(diagnostics?.ros && measuredTopics.some((topic) => topic === "/odom" || topic === "/odometry/filtered" || topic.endsWith("/odom") || topic.endsWith("/odometry/filtered")));
  const imuReady = Boolean(diagnostics?.ros && measuredTopics.some((topic) => topic === "/imu/data" || topic.endsWith("/imu/data")));
  const lidarReady = diagnostics?.lidar === true;

  useEffect(() => {
    void navigationApi("tag-graph").then(setGraph).catch((reason) => setError(errorText(reason)));
  }, [setGraph]);

  const active = Boolean(mission && !["ARRIVED", "CANCELLED", "FAILED", "EMERGENCY_STOPPED"].includes(mission.status));

  const stopManual = useCallback(() => {
    if (manualTimer.current !== null) {
      window.clearInterval(manualTimer.current);
      manualTimer.current = null;
    }
    if (manualActive.current && robotId && controlOnline) wsManualCommand(robotId, "STOP");
    manualActive.current = false;
  }, [controlOnline, robotId]);

  useEffect(() => {
    if (previousRobot.current === robotId) return;
    stopManual();
    previousRobot.current = robotId;
    setControlMode(robots[robotId]?.control_mode ?? "AUTONOMOUS");
  }, [robotId, robots, stopManual]);

  useEffect(() => () => stopManual(), [stopManual]);

  const setMode = (next: "MANUAL" | "AUTONOMOUS") => {
    if (next === controlMode) return;
    stopManual();
    if (!robotId || !controlOnline) {
      setError("Robot control requires a connected ROS bridge and WebSocket");
      return;
    }
    if (!wsSetRobotMode(robotId, next)) {
      setError("Robot control channel is not connected");
      return;
    }
    setError("");
    setControlMode(next);
  };

  const holdManual = useCallback((action: ManualAction) => {
    if (action === "STOP") {
      stopManual();
      return;
    }
    if (controlMode !== "MANUAL") {
      setError("Switch the robot to MANUAL before driving");
      return;
    }
    if (!robotId || !controlOnline) {
      setError("Manual control requires a connected ROS bridge and WebSocket");
      return;
    }
    if (!wsManualCommand(robotId, action)) {
      setError("Manual command could not be sent");
      return;
    }
    if (manualTimer.current !== null) window.clearInterval(manualTimer.current);
    manualActive.current = true;
    manualTimer.current = window.setInterval(() => {
      if (!wsManualCommand(robotId, action)) stopManual();
    }, 100);
  }, [controlMode, controlOnline, robotId, stopManual]);

  useEffect(() => {
    const keyActions: Record<string, ManualAction> = {
      ArrowUp: "FORWARD", w: "FORWARD", W: "FORWARD", ArrowDown: "BACKWARD", s: "BACKWARD", S: "BACKWARD",
      ArrowLeft: "LEFT", a: "LEFT", A: "LEFT", ArrowRight: "RIGHT", d: "RIGHT", D: "RIGHT",
      q: "ROTATE_LEFT", Q: "ROTATE_LEFT", e: "ROTATE_RIGHT", E: "ROTATE_RIGHT",
    };
    const onKeyDown = (event: KeyboardEvent) => {
      const action = keyActions[event.key];
      if (!action || event.repeat) return;
      event.preventDefault();
      holdManual(action);
    };
    const onKeyUp = (event: KeyboardEvent) => {
      if (keyActions[event.key]) {
        event.preventDefault();
        stopManual();
      }
    };
    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
      stopManual();
    };
  }, [holdManual, stopManual]);

  const act = async (fn: () => Promise<unknown>) => {
    setBusy(true);
    setError("");
    try {
      await fn();
    } catch (reason) {
      setError(errorText(reason));
    } finally {
      setBusy(false);
    }
  };
  const start = () => {
    if (controlMode !== "AUTONOMOUS") {
      setError("Switch the robot to AUTONOMOUS before starting a mission");
      return;
    }
    if (target == null || !robotId) return;
    void act(async () => {
      const nextMission = await startTagMission(robotId, target);
      useStore.getState().setTagNavigation(nextMission);
    });
  };
  const performMissionAction = (action: "pause" | "resume" | "cancel" | "replan") => {
    if (mission) void act(() => missionAction(mission.id, action));
  };
  const label = useMemo(() => target == null ? "Select tag" : `Tag ${target}`, [target]);
  const buttonEvents = (action: ManualAction) => ({
    onPointerDown: () => holdManual(action),
    onPointerUp: stopManual,
    onPointerLeave: stopManual,
    onPointerCancel: stopManual,
  });

  return <div className="overview-shell wt-has-sidebar wt-control-page">
    <Sidebar collapsed={collapsed} onToggle={() => setCollapsed((value) => !value)} />
    <header className="overview-topbar">
      <div className="overview-brand"><b><i>Ware</i>Twin</b><span>Robot Control</span></div>
      <div className="control-connection">ROS BRIDGE: <b className={controlOnline ? "good" : "bad"}>{controlOnline ? "CONNECTED" : "DISCONNECTED"}</b> · ROS {ros ? "CONNECTED" : "OFFLINE"} · {runtimeMode}</div>
    </header>
    <main className="control-main">
      <div className="control-strip">
        <select value={robotId} onChange={(event) => select(event.target.value)}><option value="">Robot</option>{robotIds.map((id) => <option key={id}>{id}</option>)}</select>
        <span className="control-mode-badge">{controlMode}</span><span>{mission?.status ?? "IDLE"}</span>
        <button className="danger" disabled={busy || !robotId} onClick={() => robotId && void act(() => emergencyStop(robotId))}>EMERGENCY STOP</button>
        {mission?.status === "EMERGENCY_STOPPED" && <button className="control-clear-stop" disabled={busy || !robotId || !controlOnline} onClick={() => robotId && void act(() => clearEmergencyStop(robotId))}>CLEAR STOP</button>}
      </div>
      <div className="control-grid">
        <section className="control-map"><MapBoundary><MapView2D mode="MAP" /></MapBoundary></section>
        <aside className="control-mission">
          <h3>MISSION</h3>
          <label>Target Tag<select value={target ?? ""} onChange={(event) => setTarget(event.target.value ? Number(event.target.value) : null)}><option value="">Select tag</option>{tags.map((tag) => <option key={tag.tag_id} value={tag.tag_id}>{tag.tag_id} {tag.label ? `· ${tag.label}` : ""}</option>)}</select></label>
          <p>Current: <b>{mission?.current_tag_id ?? "—"}</b></p><p>Next: <b>{mission?.next_tag_id ?? "—"}</b></p><p className="route-text">{missionRoute.join(" → ") || label}</p>
          <div className="control-actions">
            <button disabled={busy || !robotId || target == null || active || controlMode !== "AUTONOMOUS" || !missionControlAvailable} onClick={start}>START</button>
            {mission?.status === "PAUSED" ? <button disabled={busy} onClick={() => performMissionAction("resume")}>RESUME</button> : <button disabled={busy || !active} onClick={() => performMissionAction("pause")}>PAUSE</button>}
            <button disabled={busy || !active} onClick={() => performMissionAction("cancel")}>CANCEL</button><button disabled={busy || !active} onClick={() => performMissionAction("replan")}>REPLAN</button>
          </div>
          {error && <div className="control-error">{error}</div>}
        </aside>
      </div>
      <section className="manual-panel">
        <div className="manual-header"><div><h3>ROBOT CONTROL</h3><p>Manual drive uses a 0.4 s dead-man timeout; release the key/button to stop.</p></div><div className="manual-mode-actions"><button className={controlMode === "MANUAL" ? "selected" : ""} disabled={!controlOnline || busy} onClick={() => setMode("MANUAL")}>MANUAL</button><button className={controlMode === "AUTONOMOUS" ? "selected" : ""} disabled={!controlOnline || busy} onClick={() => setMode("AUTONOMOUS")}>AUTONOMOUS</button></div></div>
        <div className="manual-body"><div className="manual-grid">{MANUAL_BUTTONS.map((button) => <button key={button.action} className={`manual-btn manual-${button.action.toLowerCase()}`} disabled={button.action !== "STOP" && (!controlOnline || controlMode !== "MANUAL")} title={button.title} {...buttonEvents(button.action)}>{button.label}</button>)}</div><div className="manual-help">Keyboard: W/A/S/D or arrow keys · Q/E rotate · STOP is always available in MANUAL mode.</div></div>
      </section>
      <div className="control-bottom">
        <section><h3>LOCALIZATION</h3><b>{localization.state}</b><p>Last tag {localization.lastTagId ?? "—"} · Expected {localization.expectedTagId ?? mission?.next_tag_id ?? "—"}</p><p>Tag visible {detection.visible ? "YES" : "NO"}</p></section>
        <section><h3>ROBOT STATUS</h3><p>Odom {odomReady ? "OK" : "UNKNOWN"} · IMU {imuReady ? "OK" : "UNKNOWN"} · LiDAR {lidarReady ? "OK" : "UNKNOWN"} · V30E {detection.visible ? "OK" : "WAITING"}</p><p>Progress {missionProgress.toFixed(1)}%</p></section>
        <section><h3>EVENT LOG</h3><p>{mission ? `Mission target=${mission.target_tag_id} · ${mission.status}` : "No active tag mission"}</p></section>
      </div>
    </main>
  </div>;
}

/** Error boundary for the complete route, including sidebar and control panels. */
export function RobotControlPage() {
  return <RobotControlBoundary><RobotControlContent /></RobotControlBoundary>;
}
