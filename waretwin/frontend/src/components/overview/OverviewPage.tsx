import { useEffect, useMemo, useState } from "react";
import { useStore } from "../../state/store";
import { backendActions } from "../../services/backendRuntime";
import { Viewport } from "../views/Viewport";
import { Sidebar } from "../shell/Sidebar";
import { Modals } from "../ops/Modals";
import { schedulerApi, type SchedulerOverview } from "../../services/scheduler";
import { onScheduleUpdated } from "../../services/ws";

function fmtTime(s?: string | null) { return s ? new Date(s).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" }) : "—"; }

export function OverviewPage() {
  const twin = useStore((s) => s.twin);
  const paused = useStore((s) => s.paused);
  const setModal = useStore((s) => s.setModal);
  const source = useStore((s) => s.source);
  const connectedRobotIds = useStore((s) => s.connectedRobotIds);
  const lastTelemetryAt = useStore((s) => s.lastTelemetryAt);
  const diagnostics = useStore((s) => s.rosDiagnostics);
  const [scheduler, setScheduler] = useState<SchedulerOverview | null>(null);
  const [schedulerError, setSchedulerError] = useState(false);
  const [viewportFullscreen, setViewportFullscreen] = useState(false);
  const [sidebarCollapsed, setSidebarCollapsed] = useState(false);

  const load = async () => {
    try { setScheduler(await schedulerApi.overview()); setSchedulerError(false); }
    catch { setSchedulerError(true); }
  };
  useEffect(() => {
    void load();
    const timer = window.setInterval(() => void load(), 5000);
    const unsub = onScheduleUpdated(() => void load());
    return () => { clearInterval(timer); unsub(); };
  }, []);

  const alerts = useMemo(() => Object.values(twin.alerts).filter((a) => a.resolved_tick === null), [twin.alerts]);
  const robots = Object.values(twin?.robots ?? {});
  const fleetReported = source === "online" && Boolean(lastTelemetryAt) && robots.length > 0;
  const activeRobots = robots.filter((robot) => connectedRobotIds.includes(robot.id) && robot.status === "ACTIVE").length;
  const idleRobots = robots.filter((robot) => connectedRobotIds.includes(robot.id) && robot.navigation_state === "IDLE").length;
  const failed = robots.filter((robot) => ["WARNING", "ERROR", "OFFLINE"].includes(robot.status)).length;
  const fleetCount = fleetReported ? activeRobots : "—";
  const fleetBreakdown = fleetReported ? `${idleRobots} idle · ${robots.length} total` : "UNKNOWN";
  const fleetIssueCount = fleetReported ? failed : "—";
  const operationValue = "—";
  const scheduleUnknown = schedulerError ? "UNAVAILABLE" : "UNKNOWN";
  const inboundActive = scheduler?.flows?.inbound.active ?? null;
  const outboundActive = scheduler?.flows?.outbound.active ?? null;
  const inboundRunning = scheduler?.flows?.inbound.running ?? null;
  const outboundRunning = scheduler?.flows?.outbound.running ?? null;
  const inboundCompleted = scheduler?.flows?.inbound.completed ?? null;
  const outboundCompleted = scheduler?.flows?.outbound.completed ?? null;
  const activeFlowCount = inboundActive === null || outboundActive === null
    ? null : inboundActive + outboundActive;
  const completedFlowCount = inboundCompleted === null || outboundCompleted === null
    ? null : inboundCompleted + outboundCompleted;
  const flowRows: Array<[string, number | null]> = [
    ["Inbound", inboundActive], ["Outbound", outboundActive],
    ["Running IN", inboundRunning], ["Running OUT", outboundRunning],
    ["Completed", completedFlowCount],
  ];

  return <div className={`overview-shell wt-has-sidebar${sidebarCollapsed ? " wt-sidebar-collapsed" : ""}${viewportFullscreen ? " viewport-expanded" : ""}`}>
    <Sidebar collapsed={sidebarCollapsed} onToggle={() => setSidebarCollapsed((value) => !value)} />
    <header className="overview-topbar">
      <div className="overview-brand"><b><i>Ware</i>Twin</b><span>Operations Overview</span></div>
      <div className="flex items-center">
      <section className="overview-header-kpis" aria-label="Overview KPIs">
        <div><small>ROBOTS ACTIVE</small><b>{fleetCount}</b><span>{fleetBreakdown}</span></div>
        <div><small>INBOUND / OUTBOUND</small><b title={activeFlowCount === null ? scheduleUnknown : undefined}>{activeFlowCount ?? "—"}</b><span>{activeFlowCount === null ? scheduleUnknown : `${inboundActive} inbound · ${outboundActive} outbound`}</span></div>
        <div><small>ACTIVE SCHEDULES</small><b title={scheduler ? undefined : scheduleUnknown}>{scheduler?.schedules.active ?? "—"}</b><span>{scheduler ? `${scheduler.schedules.running} executing now` : scheduleUnknown}</span></div>
        <div className={failed ? "warn" : ""}><small>FLEET ISSUES</small><b>{fleetIssueCount}</b><span title={diagnostics ? undefined : "UNAVAILABLE"}>{diagnostics ? `${alerts.length} active alerts` : "UNKNOWN · diagnostics unavailable"}</span></div>
        <div><small>THROUGHPUT</small><b>{operationValue}</b><span>NOT REPORTED</span></div>
      </section>
      </div>

      <nav className="overview-nav">

      </nav>
      <div className="overview-sim"><button onClick={() => paused ? backendActions.play() : backendActions.pause()}>{paused ? "▶ Play" : "Ⅱ Pause"}</button></div>
    </header>

    <main className="overview-main">
      <section className="overview-kpis">
        <div><small>ROBOTS ACTIVE</small><b>{fleetCount}</b><span>{fleetBreakdown}</span></div>
        <div><small>INBOUND / OUTBOUND</small><b>{activeFlowCount ?? "—"}</b><span>{activeFlowCount === null ? scheduleUnknown : `${inboundActive} inbound · ${outboundActive} outbound`}</span></div>
        <div><small>ACTIVE SCHEDULES</small><b>{scheduler?.schedules.active ?? "—"}</b><span>{scheduler ? `${scheduler.schedules.running} executing now` : scheduleUnknown}</span></div>
        <div className={failed ? "warn" : ""}><small>FLEET ISSUES</small><b>{fleetIssueCount}</b><span title={diagnostics ? undefined : "UNAVAILABLE"}>{diagnostics ? `${alerts.length} active alerts` : "UNKNOWN · diagnostics unavailable"}</span></div>
        <div><small>THROUGHPUT</small><b>{operationValue}</b><span>NOT REPORTED</span></div>
      </section>

      <section className="overview-grid">
        <div><Viewport onFullscreenChange={setViewportFullscreen} /></div>
        <aside className="overview-side">
          <div className="overview-panel">
            <div className="overview-card-head"><div><small>SCHEDULER</small><b>Next assignments</b></div><button onClick={() => setModal("scheduler")}>Open</button></div>
            {schedulerError && <div className="overview-empty bad">Scheduler API unavailable</div>}
            {!schedulerError && scheduler && !scheduler.schedules.next.length && <div className="overview-empty">No upcoming schedules</div>}
            {!schedulerError && !scheduler && <div className="overview-empty">UNKNOWN · schedule data unavailable</div>}
            {scheduler?.schedules.next.map((s) => <button className="overview-schedule" key={s.id} onClick={() => setModal("scheduler")}>
              <div><b>{s.robot_id}</b><span>{s.order_no}</span></div><strong>{fmtTime(s.planned_start)}</strong><small>{s.status} · {s.estimated_distance_m.toFixed(1)} m</small>
            </button>)}
          </div>
          <div className="overview-panel compact">
            <div className="overview-card-head"><div><small>WAREHOUSE FLOW</small><b>Inbound / outbound</b></div><button onClick={() => setModal("flows")}>Manage</button></div>
            <div className="overview-order-bars">
              {flowRows.map(([label, value]) => <div key={label}><span>{label}</span><i><em style={{width:`${value === null ? 0 : Math.min(100, value * 12)}%`}}/></i><b title={value === null ? scheduleUnknown : undefined}>{value ?? "—"}</b></div>)}
            </div>
          </div>
          <div className="overview-panel compact">
            <div className="overview-card-head"><div><small>ALERTS</small><b>Needs attention</b></div></div>
            {alerts.slice(0,4).map((a) => <div className={`overview-alert ${a.severity.toLowerCase()}`} key={a.id}><b>{a.title}</b><span>{a.detail}</span></div>)}
            {!alerts.length && <div className={`overview-empty${diagnostics ? " good" : ""}`}>{diagnostics ? "No active alerts" : "UNKNOWN · diagnostics unavailable"}</div>}
          </div>
        </aside>
      </section>
    </main>
    <Modals />
  </div>;
}
