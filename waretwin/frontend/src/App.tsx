import { Component, type ErrorInfo, type ReactNode, useEffect, useState } from "react";
import { useStore } from "./state/store";
import { TopBar } from "./components/shell/TopBar";
import { Sidebar } from "./components/shell/Sidebar";
import { useBackendRealtime, useSimulationRunner } from "./simulation/runner";
import { AlertsPanel, FleetOverviewPanel, SystemStatusPanel, TaskOverviewPanel } from "./components/panels/LeftPanels";
import { EventLogPanel, LiveCameraPanel } from "./components/panels/RightPanels";
import { RobotStatusPanel, TaskQueuePanel, ThroughputPanel } from "./components/panels/BottomPanels";
import { ScenariosDrawer } from "./components/ops/ScenariosDrawer";
import { OpsDrawer } from "./components/ops/OpsDrawer";
import { Modals } from "./components/ops/Modals";
import { WhatIfDrawer } from "./components/ops/WhatIfDrawer";
import { AuthPage } from "./components/auth/AuthPage";
import { AdminPage } from "./components/admin/AdminPage";
import { WarehouseEditorPage } from "./components/admin/WarehouseEditorPage";
import { WarehouseManagementPage } from "./components/admin/WarehouseManagementPage";
import { OverviewPage } from "./components/overview/OverviewPage";
import { bootstrapAuth } from "./services/auth";
import { DEMO_MODE, DEMO_USER } from "./config";
import { RuntimeAwareRobotControlPage } from "./components/control/RobotControlPage";
import { RobotControlDetailPage } from "./components/control/RobotControlDetailPage";
import { DiagnosticsPage } from "./components/diagnostics/DiagnosticsPage";

/**
 * 版面以 1536×860 CSS px 為基準設計；視窗更小時整體等比縮小，確保所有面板完整可見
 * (Windows 125% 縮放的 1080p ≈ 1536×750)。用 transform 而不是 CSS zoom：zoom 會讓 R3F 量到的畫布尺寸被縮兩次。
 */
const DESIGN_W = 1536, DESIGN_H = 860;
function useFitScale() {
  useEffect(() => {
    const root = document.documentElement;
    const apply = () => {
      const z = Math.min(1, window.innerWidth / DESIGN_W, window.innerHeight / DESIGN_H);
      root.style.setProperty("--ui-scale", z < 0.995 ? z.toFixed(4) : "1");
    };
    apply();
    window.addEventListener("resize", apply);
    return () => window.removeEventListener("resize", apply);
  }, []);
}

/** 後端回 RATE_LIMITED / TOO_LARGE 等訊息時的短暫提示 */
function Notice() {
  const notice = useStore((s) => s.notice);
  const setNotice = useStore((s) => s.setNotice);
  useEffect(() => { if (!notice) return; const t = setTimeout(() => setNotice(null), Math.max(0, notice.until - Date.now())); return () => clearTimeout(t); }, [notice, setNotice]);
  if (!notice) return null;
  return <div className={"notice " + notice.kind} onClick={() => setNotice(null)}>{notice.text}</div>;
}

/** 這是 desktop 營運中心；窄螢幕（手機）整體縮到 0.3 倍根本看不清，先給提示，可選擇仍然繼續。
 *  門檻 1024：平板橫向（1024–1279）仍可用縮小版；手機一律提示。 */
const GATE_W = 1024;
function NarrowScreenGate({ children }: { children: React.ReactNode }) {
  const [dismissed, setDismissed] = useState(false);
  const [narrow, setNarrow] = useState(() => window.innerWidth < GATE_W);
  useEffect(() => { const f = () => setNarrow(window.innerWidth < GATE_W); window.addEventListener("resize", f); return () => window.removeEventListener("resize", f); }, []);
  if (narrow && !dismissed) {
    return (
      <div className="narrow-gate">
        <div className="brand"><span className="ai">Ware</span><span>Twin</span></div>
        <h2>Designed for desktop</h2>
        <p>WareTwin is a 3D operations console that works best on screens ≥ 1280 px wide (it still runs, scaled down, from 1024 px). On a phone the interface would shrink to about a quarter of its size and become unreadable.</p>
        <p>Open <b>ware-twin.vercel.app</b> on a laptop or desktop browser for the full experience.</p>
        <button className="btn" onClick={() => setDismissed(true)}>Continue anyway</button>
      </div>
    );
  }
  return <>{children}</>;
}


class WarehouseEditorBoundary extends Component<{ children: ReactNode }, { error: Error | null }> {
  state = { error: null as Error | null };
  static getDerivedStateFromError(error: Error) { return { error }; }
  componentDidCatch(error: Error, info: ErrorInfo) { console.error("[WarehouseEditor] runtime error", error, info); }
  render() {
    if (!this.state.error) return this.props.children;
    return (
      <div className="min-h-screen bg-[#05080f] p-10 text-slate-200">
        <div className="mx-auto max-w-3xl rounded-2xl border border-rose-500/20 bg-[#0b111c] p-6 shadow-2xl">
          <div className="mb-2 text-[11px] font-bold uppercase tracking-[0.16em] text-rose-300">Warehouse Editor Error</div>
          <h1 className="text-xl font-semibold text-white">Warehouse Editor could not be rendered</h1>
          <pre className="mt-4 overflow-auto rounded-xl bg-black/30 p-4 text-xs leading-5 text-rose-200">{this.state.error.message}\n\n{this.state.error.stack ?? ""}</pre>
          <button className="mt-4 rounded-lg border border-white/10 bg-white/5 px-4 py-2 text-sm hover:bg-white/10" onClick={() => window.location.reload()}>Reload editor</button>
        </div>
      </div>
    );
  }
}

function usePathname() {
  const [path, setPath] = useState(() => window.location.pathname);
  useEffect(() => {
    const onPop = () => setPath(window.location.pathname);
    window.addEventListener("popstate", onPop);
    return () => window.removeEventListener("popstate", onPop);
  }, []);
  return [path, (next: string) => { window.history.pushState({}, "", next); setPath(next); }] as const;
}

function decodeRouteSegment(value: string | undefined): string | null {
  if (!value) return null;
  try { return decodeURIComponent(value); } catch { return value; }
}

/** 模擬 runner 放在 gate 內層：手機提示頁顯示時不啟動 WebSocket 與本地引擎，不白耗 CPU */
function Console() {
  useFitScale();
  useSimulationRunner();
  const [sidebarCollapsed, setSidebarCollapsed] = useState(false);
  const layoutRevision = useStore((s) => s.layoutRevision);
  const activeWarehouseId = useStore((s) => s.activeWarehouseId);
  const layoutUpdatedAt = useStore((s) => s.layoutUpdatedAt);
  const goAdmin = (path: string) => {
    window.history.pushState({}, "", path);
    window.dispatchEvent(new PopStateEvent("popstate"));
  };
  return (
    <div className={`shell wt-has-sidebar${sidebarCollapsed ? " wt-sidebar-collapsed" : ""}`} key={`layout-${layoutRevision}`}>
      <Sidebar collapsed={sidebarCollapsed} onToggle={() => setSidebarCollapsed((value) => !value)} />
      <TopBar />
      <div className="operations-legacy-meta" style={{position:"fixed",top:8,right:18,zIndex:1200,display:"flex",gap:6,alignItems:"center",fontSize:11}}>
        <span style={{padding:"5px 8px",border:"1px solid rgba(34,197,94,.35)",borderRadius:8,background:"rgba(3,10,20,.88)",color:"#86efac"}}>
          DB MAP · r{layoutRevision}{activeWarehouseId ? ` · WH#${activeWarehouseId}` : ""}{layoutUpdatedAt ? " · synced" : ""}
        </span>
        <button className="btn" onClick={() => goAdmin("/")}>Overview</button>
        <button className="btn" onClick={() => useStore.getState().setModal("flows")}>Inbound / Outbound</button>
        <button className="btn" onClick={() => useStore.getState().setModal("scheduler")}>Robot Schedule</button>
        <button className="btn" onClick={() => goAdmin("/admin/warehouse")}>Warehouse Data</button>
        <button className="btn" onClick={() => goAdmin("/admin/warehouse-editor")}>Map Editor</button>
      </div>
      <div className="shell-body operations-no-map">
        <aside className="col-left">
          <FleetOverviewPanel />
          <TaskOverviewPanel />
          <SystemStatusPanel />
          <AlertsPanel />
        </aside>
        <aside className="col-right">
          <LiveCameraPanel />
          <EventLogPanel />
        </aside>
        <footer className="bottom">
          <TaskQueuePanel />
          <ThroughputPanel />
          <RobotStatusPanel />
        </footer>
      </div>
      <ScenariosDrawer />
      <OpsDrawer />
      <WhatIfDrawer />
      <Modals />
      <Notice />
    </div>
  );
}

export default function App() {
  useBackendRealtime();
  const [path, navigate] = usePathname();
  const detailMatch = path.match(/^\/robots\/([^/]+)\/control\/?$/);
  const detailRobotId = detailMatch ? decodeRouteSegment(detailMatch[1]) : null;
  const authStatus = useStore((s) => s.authStatus);
  const authUser = useStore((s) => s.authUser);

  // In frontend demo mode, authentication and backend availability are deliberately
  // removed from the critical rendering path. This makes direct navigation to
  // /admin/* and /admin/warehouse-editor work with no FastAPI server and no login.
  useEffect(() => {
    if (DEMO_MODE) {
      useStore.getState().setAuth({
        status: "authenticated",
        token: null,
        user: DEMO_USER,
      });
      return;
    }
    void bootstrapAuth();
  }, []);

  useEffect(() => {
    // Demo mode intentionally has no authentication gate.
    if (DEMO_MODE) return;

    if (authStatus === "loading") return;
    const loggedIn = authStatus === "authenticated" && !!authUser;
    if (!loggedIn && path !== "/login" && path !== "/register") {
      if (detailRobotId) {
        try { window.sessionStorage.setItem("waretwin.robot-control.return-path", path); } catch { /* storage may be disabled */ }
      }
      navigate("/login");
    }
    if (loggedIn && path.startsWith("/admin") && authUser?.role !== "admin") navigate("/");
  }, [authStatus, authUser, path, navigate]);

  // Demo frontend: render the application immediately. No session restore,
  // login page, JWT, or backend request is required for navigation.
  if (DEMO_MODE) {
    if (path.startsWith("/admin")) {
      if (path === "/admin/warehouse-editor") return <WarehouseEditorBoundary><WarehouseEditorPage /></WarehouseEditorBoundary>;
      if (path === "/admin/warehouse") return <WarehouseManagementPage />;
      return <AdminPage />;
    }
    if (path === "/operations") return <NarrowScreenGate><Console /></NarrowScreenGate>;
    if (path === "/control") return <NarrowScreenGate><RuntimeAwareRobotControlPage /></NarrowScreenGate>;
    if (detailRobotId) return <NarrowScreenGate><RobotControlDetailPage robotId={detailRobotId} /></NarrowScreenGate>;
    if (path === "/diagnostics") return <DiagnosticsPage />;
    return <OverviewPage />;
  }

  if (authStatus === "loading") {
    return <div className="auth-shell"><div className="auth-card"><div className="brand"><span className="ai">Ware</span><span>Twin</span></div><p className="hint">Restoring session...</p></div></div>;
  }

  const loggedIn = authStatus === "authenticated" && !!authUser;
  if (!loggedIn) {
    return <AuthPage mode={path === "/register" ? "register" : "login"} />;
  }
  if (path.startsWith("/admin")) {
    if (authUser?.role !== "admin") {
      return <div className="auth-shell"><div className="auth-card"><h1>403</h1><p className="hint">Admin access only.</p><button className="btn primary" onClick={() => navigate("/")}>Back to console</button></div></div>;
    }
    if (path === "/admin/warehouse-editor") return <WarehouseEditorBoundary><WarehouseEditorPage /></WarehouseEditorBoundary>;
    if (path === "/admin/warehouse") return <WarehouseManagementPage />;
    return <AdminPage />;
  }
  if (path === "/operations") return <NarrowScreenGate><Console /></NarrowScreenGate>;
  if (path === "/control") return <NarrowScreenGate><RuntimeAwareRobotControlPage /></NarrowScreenGate>;
  if (detailRobotId) return <NarrowScreenGate><RobotControlDetailPage robotId={detailRobotId} /></NarrowScreenGate>;
  if (path === "/diagnostics") return <DiagnosticsPage />;
  return <OverviewPage />;
}
