import { useEffect, useState } from "react";
import { useBackendRealtime } from "./services/backendRuntime";
import { useStore } from "./state/store";
import { RobotControlPage } from "./components/control/RobotControlPage";

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
        <p>Robot Control works best on screens ≥ 1280 px wide (it still runs, scaled down, from 1024 px). On a phone the controls and map would become difficult to read.</p>
        <p>Open the control console on a laptop or desktop browser for the full experience.</p>
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
  return [path, setPath] as const;
}

function decodeRouteSegment(value: string | undefined): string | null {
  if (!value) return null;
  try { return decodeURIComponent(value); } catch { return value; }
}

export default function App() {
  useBackendRealtime();
  const [path, setPath] = usePathname();
  const detailMatch = path.match(/^\/robots\/([^/]+)\/control\/?$/);
  const detailRobotId = detailMatch ? decodeRouteSegment(detailMatch[1]) : null;

  useEffect(() => {
    if (detailRobotId) useStore.getState().select(detailRobotId);
    if (path !== "/control") {
      window.history.replaceState({}, "", "/control");
      setPath("/control");
    }
  }, [detailRobotId, path, setPath]);

  if (path === "/control") return <NarrowScreenGate><RobotControlPage /></NarrowScreenGate>;
  return null;
}
