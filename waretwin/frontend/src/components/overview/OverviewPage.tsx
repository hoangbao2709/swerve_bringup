import { useMemo } from "react";
import { useStore } from "../../state/store";
import { logout } from "../../services/auth";
import { OverviewWarehouseView } from "./OverviewWarehouseView";
import { overviewRobots, runtimeRobotOnline } from "./overviewRuntime";

function reported(value: unknown, fallback = "UNKNOWN"): string {
  return value === undefined || value === null || value === "" ? fallback : String(value);
}

function navigate(path: string) {
  window.history.pushState({}, "", path);
  window.dispatchEvent(new PopStateEvent("popstate"));
}

function runtimeFlag(value: boolean | undefined): string {
  return value === true ? "READY" : value === false ? "UNAVAILABLE" : "UNKNOWN";
}

export function OverviewPage() {
  const authUser = useStore((state) => state.authUser);
  const twin = useStore((state) => state.twin);
  const layout = useStore((state) => state.layout);
  const publishedRevision = useStore((state) => state.mapSync.publishedRevision);
  const mapSyncStatus = useStore((state) => state.mapSync.status);
  const runtimeMode = useStore((state) => state.runtimeMode);
  const runtimeState = useStore((state) => state.runtimeState);
  const bridgeState = useStore((state) => state.bridgeState);
  const websocketState = useStore((state) => state.websocketState);
  const rosConnected = useStore((state) => state.rosConnected);
  const connectedRobotIds = useStore((state) => state.connectedRobotIds);
  const diagnostics = useStore((state) => state.rosDiagnostics);
  const localization = useStore((state) => state.localization);
  const lastTelemetryAt = useStore((state) => state.lastTelemetryAt);
  const robots = useMemo(() => overviewRobots(twin, layout, publishedRevision), [twin, layout, publishedRevision]);
  const onOpenRobot = (robotId: string) => navigate(`/robots/${encodeURIComponent(robotId)}/control`);

  return <div className="overview-shell">
    <aside className="overview-sidebar">
      <div className="overview-brand"><span className="overview-brand-mark">W</span><span><b>WareTwin</b><small>ROBOT OPERATIONS</small></span></div>
      <div className="overview-nav-group"><div className="overview-nav-label">WORKSPACE</div>
        <button type="button" className="overview-nav-item is-active" onClick={() => navigate("/")}><span>◫</span>Overview</button>
        <button type="button" className="overview-nav-item" onClick={() => navigate("/control")}><span>⌘</span>Robot Control</button>
      </div>
      <div className="overview-sidebar-spacer" />
      <div className="overview-account"><div className="overview-nav-label">ACCOUNT</div>
        <div className="overview-account-user"><span className="overview-avatar">{authUser?.username?.slice(0, 1).toUpperCase() ?? "?"}</span>
          <span className="overview-account-name">{reported(authUser?.username)}</span></div>
        <button type="button" className="overview-logout" onClick={() => void logout()}>Log out <span>↗</span></button>
      </div>
      <div className="overview-sidebar-foot"><span className="overview-live-dot" /> LIVE RUNTIME · {reported(runtimeMode)}</div>
    </aside>

    <div className="overview-workspace">
      <header className="overview-topbar">
        <div><div className="overview-breadcrumb">WORKSPACE <span>/</span> OVERVIEW</div><h1>Warehouse Overview</h1></div>
        <div className="overview-runtime-status" aria-label="Runtime status">
          <div><span>RUNTIME</span><b>{reported(runtimeMode)}</b></div>
          <div><span>STATE</span><b>{reported(runtimeState)}</b></div>
          <div><span>ROS BRIDGE</span><b>{reported(bridgeState)}</b></div>
          <div><span>WEBSOCKET</span><b>{websocketState}</b></div>
          <div><span>LIDAR</span><b>{runtimeFlag(diagnostics?.lidar)}</b></div>
          <div><span>MAP SYNC</span><b>{reported(mapSyncStatus)}</b></div>
        </div>
      </header>

      <main className="overview-main">
        <div className="overview-section-heading"><div><span className="overview-eyebrow">LIVE WAREHOUSE</span><h2>{layout?.name ?? "Active warehouse"}</h2></div>
          <div className="overview-data-badge"><span className={websocketState === "CONNECTED" ? "is-live" : ""} /> BACKEND / ROS DATA</div></div>
        <div className="overview-content-grid">
          <section className="overview-map-panel" aria-label="Warehouse visualization">
            <OverviewWarehouseView layout={layout} robots={robots} onOpenRobot={onOpenRobot} />
            <div className="overview-map-footer"><span>MAP REVISION <b>{publishedRevision ?? "UNKNOWN"}</b></span><span>GEOMETRY <b>{layout ? "CANONICAL MAP" : "UNAVAILABLE"}</b></span><span>POSE SOURCE <b>ROS TF</b></span></div>
          </section>
          <aside className="overview-runtime-panel">
            <div className="overview-panel-heading"><span>RUNTIME STATUS</span><small>{robots.length} REPORTED ROBOTS</small></div>
            <div className="overview-runtime-row"><span>ROS bridge</span><b>{reported(bridgeState)}</b></div>
            <div className="overview-runtime-row"><span>Telemetry</span><b>{reported(lastTelemetryAt ? "RECEIVED" : null)}</b></div>
            <div className="overview-runtime-row"><span>Localization</span><b>{reported(localization?.state)}</b></div>
            <div className="overview-runtime-row"><span>LiDAR</span><b>{runtimeFlag(diagnostics?.lidar)}</b></div>
            <div className="overview-runtime-row"><span>TF</span><b>{runtimeFlag(diagnostics?.tf)}</b></div>
            <div className="overview-panel-divider" />
            <div className="overview-panel-heading"><span>ROBOT TELEMETRY</span><small>SELECT TO CONTROL</small></div>
            {robots.length ? <div className="overview-robot-list">{robots.map(({ id, state, pose }) => {
              const online = runtimeRobotOnline(id, websocketState, rosConnected, connectedRobotIds);
              return <button type="button" className="overview-robot-row" key={id} onClick={() => onOpenRobot(id)}>
                <span className={`overview-robot-status ${online ? "is-online" : ""}`} />
                <span className="overview-robot-row-main"><b>{id}</b><small>{online ? "ONLINE" : state.status === "OFFLINE" ? "OFFLINE" : "UNKNOWN"}</small></span>
                <span className="overview-robot-row-pose">{pose ? `${pose.canonical_pose!.x.toFixed(2)}, ${pose.canonical_pose!.y.toFixed(2)}` : "POSE · UNAVAILABLE"}</span>
                <span className="overview-open-arrow">›</span>
              </button>;
            })}</div> : <div className="overview-no-robots" role="status"><b>NO ROBOT TELEMETRY</b><span>Robot identities and positions appear only after the backend reports runtime telemetry.</span></div>}
          </aside>
        </div>
      </main>
    </div>
  </div>;
}
