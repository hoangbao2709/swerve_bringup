import { useEffect, useMemo, useState } from "react";
import { useStore } from "../../state/store";
import { useSimulationRunner, simControl } from "../../simulation/runner";
import { Viewport } from "../views/Viewport";
import { Sidebar } from "../shell/Sidebar";
import { Modals } from "../ops/Modals";
import { schedulerApi, type SchedulerOverview } from "../../services/scheduler";
import { onScheduleUpdated } from "../../services/ws";

function fmtTime(s?: string | null) { return s ? new Date(s).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" }) : "—"; }

export function OverviewPage() {
  useSimulationRunner();
  const twin = useStore((s) => s.twin);
  const paused = useStore((s) => s.paused);
  const setModal = useStore((s) => s.setModal);
  const authUser = useStore((s) => s.authUser);
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
  const activeRobots = twin.kpi.fleet.active;
  const idleRobots = twin.kpi.fleet.idle;
  const failed = twin.kpi.fleet.error + twin.kpi.fleet.offline;

  return <div className={`overview-shell wt-has-sidebar${sidebarCollapsed ? " wt-sidebar-collapsed" : ""}${viewportFullscreen ? " viewport-expanded" : ""}`}>
    <Sidebar collapsed={sidebarCollapsed} onToggle={() => setSidebarCollapsed((value) => !value)} />
    <header className="overview-topbar">
      <div className="overview-brand"><b><i>Ware</i>Twin</b><span>Operations Overview</span></div>
      <div className="flex items-center">
      <section className="overview-header-kpis" aria-label="Overview KPIs">
        <div><small>ROBOTS ACTIVE</small><b>{activeRobots}</b><span>{idleRobots} idle · {twin.kpi.fleet.total} total</span></div>
        <div><small>INBOUND / OUTBOUND</small><b>{(scheduler?.flows?.inbound.active ?? 0) + (scheduler?.flows?.outbound.active ?? 0)}</b><span>{scheduler?.flows?.inbound.active ?? 0} inbound · {scheduler?.flows?.outbound.active ?? 0} outbound</span></div>
        <div><small>ACTIVE SCHEDULES</small><b>{scheduler?.schedules.active ?? 0}</b><span>{scheduler?.schedules.running ?? 0} executing now</span></div>
        <div className={failed ? "warn" : ""}><small>FLEET ISSUES</small><b>{failed}</b><span>{alerts.length} active alerts</span></div>
        <div><small>THROUGHPUT</small><b>{twin.kpi.operation.throughput_per_min.toFixed(1)}</b><span>tasks / min</span></div>
      </section>
      </div>

      <nav className="overview-nav">

      </nav>
      <div className="overview-sim"><button onClick={() => paused ? simControl.play() : simControl.pause()}>{paused ? "▶ Play" : "Ⅱ Pause"}</button><span>{authUser?.username ?? "operator"}</span></div>
    </header>

    <main className="overview-main">
      <section className="overview-kpis">
        <div><small>ROBOTS ACTIVE</small><b>{activeRobots}</b><span>{idleRobots} idle · {twin.kpi.fleet.total} total</span></div>
        <div><small>INBOUND / OUTBOUND</small><b>{(scheduler?.flows?.inbound.active ?? 0) + (scheduler?.flows?.outbound.active ?? 0)}</b><span>{scheduler?.flows?.inbound.active ?? 0} inbound · {scheduler?.flows?.outbound.active ?? 0} outbound</span></div>
        <div><small>ACTIVE SCHEDULES</small><b>{scheduler?.schedules.active ?? 0}</b><span>{scheduler?.schedules.running ?? 0} executing now</span></div>
        <div className={failed ? "warn" : ""}><small>FLEET ISSUES</small><b>{failed}</b><span>{alerts.length} active alerts</span></div>
        <div><small>THROUGHPUT</small><b>{twin.kpi.operation.throughput_per_min.toFixed(1)}</b><span>tasks / min</span></div>
      </section>

      <section className="overview-grid">
        <div><Viewport onFullscreenChange={setViewportFullscreen} /></div>
        <aside className="overview-side">
          <div className="overview-panel">
            <div className="overview-card-head"><div><small>SCHEDULER</small><b>Next assignments</b></div><button onClick={() => setModal("scheduler")}>Open</button></div>
            {schedulerError && <div className="overview-empty bad">Scheduler API unavailable</div>}
            {!schedulerError && !(scheduler?.schedules.next.length) && <div className="overview-empty">No upcoming schedules</div>}
            {scheduler?.schedules.next.map((s) => <button className="overview-schedule" key={s.id} onClick={() => setModal("scheduler")}>
              <div><b>{s.robot_id}</b><span>{s.order_no}</span></div><strong>{fmtTime(s.planned_start)}</strong><small>{s.status} · {s.estimated_distance_m.toFixed(1)} m</small>
            </button>)}
          </div>
          <div className="overview-panel compact">
            <div className="overview-card-head"><div><small>WAREHOUSE FLOW</small><b>Inbound / outbound</b></div><button onClick={() => setModal("flows")}>Manage</button></div>
            <div className="overview-order-bars">
              {[["Inbound", scheduler?.flows?.inbound.active ?? 0], ["Outbound", scheduler?.flows?.outbound.active ?? 0], ["Running IN", scheduler?.flows?.inbound.running ?? 0], ["Running OUT", scheduler?.flows?.outbound.running ?? 0], ["Completed", (scheduler?.flows?.inbound.completed ?? 0) + (scheduler?.flows?.outbound.completed ?? 0)]].map(([label,value]) => <div key={String(label)}><span>{label}</span><i><em style={{width:`${Math.min(100, Number(value) * 12)}%`}}/></i><b>{value}</b></div>)}
            </div>
          </div>
          <div className="overview-panel compact">
            <div className="overview-card-head"><div><small>ALERTS</small><b>Needs attention</b></div></div>
            {alerts.slice(0,4).map((a) => <div className={`overview-alert ${a.severity.toLowerCase()}`} key={a.id}><b>{a.title}</b><span>{a.detail}</span></div>)}
            {!alerts.length && <div className="overview-empty good">All systems normal</div>}
          </div>
        </aside>
      </section>
    </main>
    <Modals />
  </div>;
}
