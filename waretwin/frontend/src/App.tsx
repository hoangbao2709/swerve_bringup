import { useEffect, useState } from "react";
import { useStore } from "./state/store";
import { useBackendRealtime } from "./services/backendRuntime";
import { AuthPage } from "./components/auth/AuthPage";
import { OverviewPage } from "./components/overview/OverviewPage";
import { RobotControlPage } from "./components/control/RobotControlPage";
import { RobotControlDetailPage } from "./components/control/RobotControlDetailPage";
import { bootstrapAuth } from "./services/auth";

/** The retained console is designed for desktop, with the original compact
 * fallback message kept for narrower Robot Control screens. */
const GATE_W = 1024;
function NarrowScreenGate({ children }: { children: React.ReactNode }) {
  const [dismissed, setDismissed] = useState(false);
  const [narrow, setNarrow] = useState(() => window.innerWidth < GATE_W);
  useEffect(() => {
    const onResize = () => setNarrow(window.innerWidth < GATE_W);
    window.addEventListener("resize", onResize);
    return () => window.removeEventListener("resize", onResize);
  }, []);
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

export default function App() {
  useBackendRealtime();
  const [path, navigate] = usePathname();
  const detailMatch = path.match(/^\/robots\/([^/]+)\/control\/?$/);
  const detailRobotId = detailMatch ? decodeRouteSegment(detailMatch[1]) : null;
  const authStatus = useStore((state) => state.authStatus);
  const authUser = useStore((state) => state.authUser);

  useEffect(() => { void bootstrapAuth(); }, []);

  useEffect(() => {
    if (authStatus === "loading") return;
    const loggedIn = authStatus === "authenticated" && Boolean(authUser);
    if (!loggedIn) {
      if (path !== "/login" && path !== "/register") {
        if (detailRobotId) {
          try { window.sessionStorage.setItem("waretwin.robot-control.return-path", path); } catch { /* storage may be disabled */ }
        }
        navigate("/login");
      }
      return;
    }
    const retainedRoute = path === "/" || path === "/control" || Boolean(detailRobotId);
    if (!retainedRoute) navigate("/");
  }, [authStatus, authUser, detailRobotId, path]);

  if (authStatus === "loading") {
    return <div className="auth-shell"><div className="auth-card"><div className="brand"><span className="ai">Ware</span><span>Twin</span></div><p className="hint">Restoring session...</p></div></div>;
  }

  const loggedIn = authStatus === "authenticated" && Boolean(authUser);
  if (!loggedIn) return <AuthPage mode={path === "/register" ? "register" : "login"} />;
  if (path === "/control") return <NarrowScreenGate><RobotControlPage /></NarrowScreenGate>;
  if (detailRobotId) return <NarrowScreenGate><RobotControlDetailPage robotId={detailRobotId} /></NarrowScreenGate>;
  return <OverviewPage />;
}
