import { useCallback, useEffect, useState } from "react";
import { useStore } from "./state/store";
import { AuthPage } from "./components/auth/AuthPage";
import { RobotControlPage } from "./components/control/RobotControlPage";
import { RobotControlDetailPage } from "./components/control/RobotControlDetailPage";
import { OverviewPage } from "./components/overview/OverviewPage";
import { bootstrapAuth } from "./services/auth";
import { useBackendRealtime } from "./services/backendRealtime";

function usePathname() {
  const [path, setPath] = useState(() => window.location.pathname);
  useEffect(() => {
    const sync = () => setPath(window.location.pathname);
    sync();
    window.addEventListener("popstate", sync);
    return () => window.removeEventListener("popstate", sync);
  }, [setPath]);

  const navigate = useCallback((next: string) => {
    if (window.location.pathname !== next) window.history.pushState({}, "", next);
    setPath(next);
  }, [setPath]);
  return [path, navigate] as const;
}

function decodeRouteSegment(value: string): string {
  try { return decodeURIComponent(value); } catch { return value; }
}

function AuthLoading() {
  return <main className="auth-shell" role="status">
    <section className="auth-card">
      <div className="brand"><span className="ai">PTAGV</span><span> / WareTwin</span></div>
      <p className="hint">Restoring authenticated session…</p>
    </section>
  </main>;
}

function Notice() {
  const notice = useStore((state) => state.notice);
  const setNotice = useStore((state) => state.setNotice);
  useEffect(() => {
    if (!notice) return;
    const timer = window.setTimeout(() => setNotice(null), Math.max(0, notice.until - Date.now()));
    return () => window.clearTimeout(timer);
  }, [notice, setNotice]);
  if (!notice) return null;
  return <div className={`app-notice ${notice.kind}`} role="status" onClick={() => setNotice(null)}>{notice.text}</div>;
}

export default function App() {
  useBackendRealtime();
  const [path, navigate] = usePathname();
  const authStatus = useStore((state) => state.authStatus);
  const authUser = useStore((state) => state.authUser);

  useEffect(() => { void bootstrapAuth(); }, []);

  const loggedIn = authStatus === "authenticated" && authUser !== null;
  useEffect(() => {
    if (authStatus === "loading") return;
    if (!loggedIn && path !== "/login") navigate("/login");
    else if (loggedIn && path !== "/" && path !== "/control" && !/^\/robots\/[^/]+\/control\/?$/.test(path)) navigate("/");
  }, [authStatus, loggedIn, path, navigate]);

  if (authStatus === "loading") return <AuthLoading />;
  if (!loggedIn) return <AuthPage />;

  const detailMatch = path.match(/^\/robots\/([^/]+)\/control\/?$/);
  if (detailMatch) return <><RobotControlDetailPage robotId={decodeRouteSegment(detailMatch[1])} /><Notice /></>;
  if (path === "/control") return <><RobotControlPage /><Notice /></>;
  if (path === "/") return <><OverviewPage /><Notice /></>;
  return <><OverviewPage /><Notice /></>;
}
