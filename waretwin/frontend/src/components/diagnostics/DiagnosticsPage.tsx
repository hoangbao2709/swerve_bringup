import { useEffect, useMemo, useState } from "react";
import { Sidebar } from "../shell/Sidebar";
import { apiFetch } from "../../services/api";
import { useStore, type WebSocketState } from "../../state/store";

type HealthPayload = {
  status?: string;
  database?: boolean;
  ros_bridge?: boolean;
  websocket?: boolean;
  ros?: boolean;
  gazebo?: boolean;
  mode?: string;
  runtime_state?: string;
  timestamp?: string;
  version?: string;
  components?: { diagnostics?: Record<string, unknown>; bridge_state?: string; [key: string]: unknown };
};
type StatusPayload = {
  system?: { cpu_load_1m?: number | null; cpu_count?: number; memory?: Record<string, number | null>; disk?: Record<string, number | null>; uptime_s?: number | null };
  ros?: Record<string, unknown>;
  runtime?: Record<string, unknown>;
};

function stateFor(value: unknown): "OK" | "WARNING" | "ERROR" | "DISCONNECTED" {
  if (value === true || value === "CONNECTED" || value === "OK") return "OK";
  if (value === "DISCONNECTED" || value === false || value === "OFFLINE") return "DISCONNECTED";
  if (value === "WARNING" || value === "DEGRADED" || value === "RECONNECTING") return "WARNING";
  return "ERROR";
}

function Badge({ value }: { value: unknown }) {
  const state = stateFor(value);
  return <span className={`diag-badge ${state.toLowerCase()}`}>{state}</span>;
}

function formatBytes(value: unknown) {
  const number = Number(value);
  if (!Number.isFinite(number)) return "—";
  const units = ["B", "KB", "MB", "GB", "TB"];
  let current = number;
  let index = 0;
  while (current >= 1024 && index < units.length - 1) { current /= 1024; index += 1; }
  return `${current.toFixed(index ? 1 : 0)} ${units[index]}`;
}

function formatUptime(value: unknown) {
  const seconds = Number(value);
  if (!Number.isFinite(seconds)) return "—";
  const hours = Math.floor(seconds / 3600);
  const minutes = Math.floor((seconds % 3600) / 60);
  return `${hours}h ${minutes}m`;
}

function wsLabel(state: WebSocketState) {
  return state === "CONNECTED" ? "CONNECTED" : state;
}

export function DiagnosticsPage() {
  const [sidebarCollapsed, setSidebarCollapsed] = useState(false);
  const [health, setHealth] = useState<HealthPayload | null>(null);
  const [system, setSystem] = useState<StatusPayload | null>(null);
  const [error, setError] = useState<string | null>(null);
  const wsState = useStore((s) => s.websocketState);
  const bridgeState = useStore((s) => s.bridgeState);
  const runtimeState = useStore((s) => s.runtimeState);
  const rosDiagnostics = useStore((s) => s.rosDiagnostics);
  const source = useStore((s) => s.source);

  useEffect(() => {
    let cancelled = false;
    const load = async () => {
      try {
        const [healthResponse, statusResponse] = await Promise.all([
          apiFetch("/api/health/"),
          apiFetch("/api/system/status/"),
        ]);
        if (!healthResponse.ok || !statusResponse.ok) throw new Error(`backend ${healthResponse.status}/${statusResponse.status}`);
        const [healthBody, statusBody] = await Promise.all([healthResponse.json(), statusResponse.json()]);
        if (!cancelled) { setHealth(healthBody as HealthPayload); setSystem(statusBody as StatusPayload); setError(null); }
      } catch (caught) {
        if (!cancelled) setError(caught instanceof Error ? caught.message : "diagnostics unavailable");
      }
    };
    void load();
    const timer = window.setInterval(() => void load(), 5000);
    return () => { cancelled = true; window.clearInterval(timer); };
  }, []);

  const ros = (health?.components?.diagnostics ?? system?.ros ?? rosDiagnostics ?? {}) as Record<string, unknown>;
  const nodes = Array.isArray(ros.nodes) ? ros.nodes.map(String) : [];
  const topics = Array.isArray(ros.topics) ? ros.topics.map(String) : [];
  const controllers = Array.isArray(ros.controllers) ? ros.controllers as Array<{ name?: string; state?: string }> : [];
  const metrics = system?.system ?? {};
  const runtime = system?.runtime ?? {};
  const rows = useMemo(() => [
    ["Backend", health?.status === "ok"],
    ["Database", health?.database],
    ["WebSocket", health?.websocket],
    ["ROS bridge", health?.ros_bridge],
    ["ROS graph", health?.ros],
    ["Gazebo", health?.gazebo],
    ["SLAM", ros.slam],
    ["Nav2", ros.nav2],
    ["controller_manager", ros.controller_manager],
    ["TF", ros.tf],
    ["LiDAR", ros.lidar],
  ] as Array<[string, unknown]>, [health, ros]);

  return <div className="diagnostics-shell wt-has-sidebar">
    <Sidebar collapsed={sidebarCollapsed} onToggle={() => setSidebarCollapsed((value) => !value)} />
    <main className="diagnostics-main">
      <header className="diagnostics-header">
        <div><small>SYSTEM DIAGNOSTICS</small><h1>Runtime, ROS and sensor health</h1><p>Measured status only. A disconnected component is shown as disconnected, not green.</p></div>
        <div className="diagnostics-header-state"><span className="diag-badge muted">MODE {health?.mode ?? runtimeState}</span><span className="diag-badge muted">WS {wsLabel(wsState)}</span><span className="diag-badge muted">SOURCE {source.toUpperCase()}</span></div>
      </header>
      {error && <div className="diagnostics-error">Backend diagnostics unavailable: {error}</div>}
      <section className="diagnostics-grid">
        <article className="diagnostics-card diagnostics-wide"><div className="diagnostics-title">Components</div><div className="diagnostics-component-grid">{rows.map(([label, value]) => <div className="diagnostics-component" key={label}><span>{label}</span><Badge value={value} /></div>)}</div></article>
        <article className="diagnostics-card"><div className="diagnostics-title">Runtime</div><dl className="diagnostics-list"><div><dt>Runtime state</dt><dd>{health?.runtime_state ?? runtimeState}</dd></div><div><dt>Bridge state</dt><dd>{String(health?.components?.bridge_state ?? bridgeState)}</dd></div><div><dt>Nav2 state</dt><dd>{String(runtime.nav2_state ?? "—")}</dd></div><div><dt>Last telemetry</dt><dd>{String(runtime.last_telemetry_at ?? "—")}</dd></div><div><dt>Last health</dt><dd>{health?.timestamp ? new Date(health.timestamp).toLocaleTimeString() : "—"}</dd></div></dl></article>
        <article className="diagnostics-card"><div className="diagnostics-title">Host metrics</div><dl className="diagnostics-list"><div><dt>CPU load / cores</dt><dd>{metrics.cpu_load_1m == null ? "—" : `${Number(metrics.cpu_load_1m).toFixed(2)} / ${metrics.cpu_count ?? "—"}`}</dd></div><div><dt>RAM used</dt><dd>{formatBytes((metrics.memory as Record<string, unknown> | undefined)?.used_bytes)} / {formatBytes((metrics.memory as Record<string, unknown> | undefined)?.total_bytes)}</dd></div><div><dt>RAM usage</dt><dd>{(metrics.memory as Record<string, unknown> | undefined)?.used_percent == null ? "—" : `${Number((metrics.memory as Record<string, unknown>).used_percent).toFixed(1)}%`}</dd></div><div><dt>Disk usage</dt><dd>{(metrics.disk as Record<string, unknown> | undefined)?.used_percent == null ? "—" : `${Number((metrics.disk as Record<string, unknown>).used_percent).toFixed(1)}%`}</dd></div><div><dt>Host uptime</dt><dd>{formatUptime(metrics.uptime_s)}</dd></div></dl></article>
        <article className="diagnostics-card"><div className="diagnostics-title">LiDAR metrics</div><dl className="diagnostics-list"><div><dt>Frequency</dt><dd>{ros.metrics && typeof ros.metrics === "object" && (ros.metrics as Record<string, unknown>).lidar_frequency_hz != null ? `${Number((ros.metrics as Record<string, unknown>).lidar_frequency_hz).toFixed(1)} Hz` : "—"}</dd></div><div><dt>Frame</dt><dd>{String(ros.metrics && typeof ros.metrics === "object" ? (ros.metrics as Record<string, unknown>).lidar_frame_id ?? "—" : "—")}</dd></div><div><dt>Age</dt><dd>{ros.metrics && typeof ros.metrics === "object" && (ros.metrics as Record<string, unknown>).lidar_age_s != null ? `${Number((ros.metrics as Record<string, unknown>).lidar_age_s).toFixed(2)} s` : "—"}</dd></div><div><dt>Dropped</dt><dd>{String(ros.metrics && typeof ros.metrics === "object" ? (ros.metrics as Record<string, unknown>).lidar_dropped_messages ?? "unknown" : "unknown")}</dd></div></dl></article>
        <article className="diagnostics-card"><div className="diagnostics-title">Controller state</div>{controllers.length ? <div className="diagnostics-table">{controllers.map((controller) => <div key={controller.name}><span>{controller.name}</span><Badge value={controller.state === "active" ? "OK" : "WARNING"} /></div>)}</div> : <div className="diagnostics-empty">No controller list measured.</div>}</article>
        <article className="diagnostics-card diagnostics-wide"><div className="diagnostics-title">ROS graph</div><div className="diagnostics-columns"><div><b>Nodes ({nodes.length})</b><pre>{nodes.length ? nodes.join("\n") : "No ROS nodes reported."}</pre></div><div><b>Topics ({topics.length})</b><pre>{topics.length ? topics.join("\n") : "No ROS topics reported."}</pre></div></div></article>
      </section>
      <footer className="diagnostics-footer">WareTwin {health?.version ?? "—"} · refreshed every 5 s · ROS diagnostics arrive over the authenticated bridge WebSocket</footer>
    </main>
  </div>;
}
