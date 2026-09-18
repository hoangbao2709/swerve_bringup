import { Component, useEffect, useMemo, useState, type ErrorInfo, type ReactNode } from "react";
import { Sidebar } from "../shell/Sidebar";
import { MapView2D } from "../views/MapView2D";
import { useStore } from "../../state/store";
import { emergencyStop, missionAction, navigationApi, startTagMission } from "../../services/api";

class MapBoundary extends Component<{ children: ReactNode }, { error: Error | null }> {
  state = { error: null as Error | null };
  static getDerivedStateFromError(error: Error) { return { error }; }
  componentDidCatch(error: Error, info: ErrorInfo) { console.error("[RobotControl] map render error", error, info); }
  render() { return this.state.error ? <div className="control-map-error">Warehouse map unavailable: {this.state.error.message}</div> : this.props.children; }
}

// Zustand selectors must return the same reference when state is unchanged.
// In particular, do not use `?? []` inside a selector: it creates a new array
// on every snapshot while tagGraph is still loading.
const EMPTY_TAGS: Array<{ tag_id: number; label?: string }> = [];

export function RobotControlPage() {
  const robots = useStore(s => s.twin.robots); const selected = useStore(s => s.selectedRobot); const select = useStore(s => s.select);
  const mission = useStore(s => s.tagNavigation); const target = useStore(s => s.targetTagId); const setTarget = useStore(s => s.setTargetTagId);
  const localization = useStore(s => s.localization); const detection = useStore(s => s.tagDetection);
  const setGraph = useStore(s => s.setTagGraph); const ros = useStore(s => s.rosConnected); const runtimeMode = useStore(s => s.runtimeMode);
  const [collapsed, setCollapsed] = useState(false); const [busy, setBusy] = useState(false); const [error, setError] = useState("");
  const robotIds = Object.keys(robots); const robotId = selected ?? robotIds[0] ?? "";
  useEffect(() => { void navigationApi("tag-graph").then(setGraph).catch(e => setError(e.message)); }, [setGraph]);
  const tags = useStore(s => s.tagGraph ? s.tagGraph.tags : EMPTY_TAGS);
  const active = mission && ["ARRIVED", "CANCELLED", "FAILED", "EMERGENCY_STOPPED"].indexOf(mission.status) < 0;
  const act = async (fn: () => Promise<unknown>) => { setBusy(true); setError(""); try { await fn(); } catch (e) { setError(e instanceof Error ? e.message : "Request failed"); } finally { setBusy(false); } };
  const start = () => target == null || !robotId ? undefined : act(async () => { const m = await startTagMission(robotId, target); useStore.getState().setTagNavigation(m); });
  const label = useMemo(() => target == null ? "Select tag" : `Tag ${target}`, [target]);
  return <div className="overview-shell wt-has-sidebar wt-control-page">
    <Sidebar collapsed={collapsed} onToggle={() => setCollapsed(v => !v)} />
    <header className="overview-topbar"><div className="overview-brand"><b><i>Ware</i>Twin</b><span>Robot Control</span></div><div className="control-connection">ROS BRIDGE: <b className={ros ? "good" : "bad"}>{ros ? "CONNECTED" : "OFFLINE"}</b> · {runtimeMode}</div></header>
    <main className="control-main">
      <div className="control-strip"><select value={robotId} onChange={e => select(e.target.value)}><option value="">Robot</option>{robotIds.map(id => <option key={id}>{id}</option>)}</select><span>AUTO</span><span>{mission?.status ?? "IDLE"}</span><button className="danger" onClick={() => robotId && act(() => emergencyStop(robotId))}>EMERGENCY STOP</button></div>
      <div className="control-grid"><section className="control-map"><MapBoundary><MapView2D mode="MAP" /></MapBoundary></section><aside className="control-mission"><h3>MISSION</h3><label>Target Tag<select value={target ?? ""} onChange={e => setTarget(e.target.value ? Number(e.target.value) : null)}><option value="">Select tag</option>{tags.map(t => <option key={t.tag_id} value={t.tag_id}>{t.tag_id} {t.label ? `· ${t.label}` : ""}</option>)}</select></label><p>Current: <b>{mission?.current_tag_id ?? "—"}</b></p><p>Next: <b>{mission?.next_tag_id ?? "—"}</b></p><p className="route-text">{mission?.route?.join(" → ") ?? label}</p><div className="control-actions"><button disabled={busy || !robotId || target == null || !!active || (!ros && runtimeMode !== "LOCAL_SIM")} onClick={start}>START</button>{mission && mission.status === "PAUSED" ? <button disabled={busy} onClick={() => act(() => missionAction(mission.id, "resume"))}>RESUME</button> : <button disabled={busy || !active} onClick={() => act(() => missionAction(mission!.id, "pause"))}>PAUSE</button>}<button disabled={busy || !active} onClick={() => act(() => missionAction(mission!.id, "cancel"))}>CANCEL</button><button disabled={busy || !active} onClick={() => act(() => missionAction(mission!.id, "replan"))}>REPLAN</button></div>{error && <div className="control-error">{error}</div>}</aside></div>
      <div className="control-bottom"><section><h3>LOCALIZATION</h3><b>{localization.state}</b><p>Last tag {localization.lastTagId ?? "—"} · Expected {localization.expectedTagId ?? mission?.next_tag_id ?? "—"}</p><p>Tag visible {detection.visible ? "YES" : "NO"}</p></section><section><h3>ROBOT STATUS</h3><p>Odom OK · IMU OK · LiDAR OK · V30E {detection.visible ? "OK" : "WAITING"}</p><p>Progress {mission?.progress_percent?.toFixed(1) ?? "0.0"}%</p></section><section><h3>EVENT LOG</h3><p>{mission ? `Mission target=${mission.target_tag_id} · ${mission.status}` : "No active tag mission"}</p></section></div>
    </main>
    
  </div>;
}
