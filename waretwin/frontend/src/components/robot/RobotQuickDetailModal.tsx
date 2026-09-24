import { useEffect, useMemo, useState } from "react";
import { createPortal } from "react-dom";
import { clearEmergencyStop, emergencyStop } from "../../services/api";
import { useStore } from "../../state/store";
import type { RobotSystemDiagnostics } from "../../schema/twin_state";

function fmt(value: unknown, digits = 2, suffix = "") {
  const number = Number(value);
  return Number.isFinite(number) ? `${number.toFixed(digits)}${suffix}` : "N/A";
}

function pushRoute(path: string) {
  window.history.pushState({}, "", path);
  window.dispatchEvent(new PopStateEvent("popstate"));
}

export function RobotQuickDetailModal() {
  const robotId = useStore((state) => state.quickDetailRobotId);
  const twin = useStore((state) => state.twin);
  const close = useStore((state) => state.closeRobotQuickDetail);
  const robots = twin?.robots && typeof twin.robots === "object" && !Array.isArray(twin.robots) ? twin.robots : {};
  const robot = robotId ? robots[robotId] : undefined;
  const tasks = twin?.tasks && typeof twin.tasks === "object" && !Array.isArray(twin.tasks) ? twin.tasks : {};
  const task = robot?.current_task_id ? tasks[robot.current_task_id] : undefined;
  const mission = useStore((state) => state.tagNavigation);
  const ros = useStore((state) => state.rosDiagnostics);
  const detail = useStore((state) => (robotId ? state.robotDetail[robotId] : undefined));
  const localization = useStore((state) => state.localization);
  const lastTelemetryAt = useStore((state) => state.lastTelemetryAt);
  const runtimeMode = useStore((state) => state.runtimeMode);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");

  useEffect(() => {
    if (!robotId) return;
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key === "Escape") close();
    };
    window.addEventListener("keydown", onKeyDown);
    return () => window.removeEventListener("keydown", onKeyDown);
  }, [close, robotId]);

  const online = useMemo(() => {
    if (!robot) return false;
    if (robot.status === "OFFLINE") return false;
    if (runtimeMode === "LOCAL_SIM") return true;
    if (!robot.last_telemetry_at) return false;
    return Date.now() - Date.parse(robot.last_telemetry_at) < 5000;
  }, [robot, runtimeMode]);
  const activeMission = mission?.robot_id === robotId ? mission : null;
  const robotDiagnostics = detail?.diagnostics ?? (ros as RobotSystemDiagnostics | null);
  const controllers = robotDiagnostics?.controllers ?? [];
  const controllerState = controllers.length ? (controllers.every((item) => item.state === "active") ? "ACTIVE" : "ERROR") : "N/A";
  const lidarState = robotDiagnostics ? (robotDiagnostics.lidar ? "OK" : "OFFLINE") : "N/A";
  const localizationState = robotDiagnostics?.localization ?? localization?.state ?? "N/A";

  if (!robotId || typeof document === "undefined") return null;

  const action = async (fn: () => Promise<unknown>) => {
    setBusy(true);
    setError("");
    try {
      await fn();
    } catch (caught) {
      setError(caught instanceof Error ? caught.message : "Request failed");
    } finally {
      setBusy(false);
    }
  };

  return createPortal(
    <div className="robot-quick-modal-root" role="presentation" onMouseDown={(event) => { if (event.target === event.currentTarget) close(); }}>
      <section className="robot-quick-modal" role="dialog" aria-modal="true" aria-labelledby="robot-quick-title">
        <header className="robot-quick-head">
          <div>
            <span className="robot-console-kicker">ROBOT QUICK DETAIL</span>
            <h2 id="robot-quick-title">{robotId}</h2>
            <p>{robot?.model ?? "Robot data unavailable"} · {robot?.id ? "live twin record" : "waiting for telemetry"}</p>
          </div>
          <button type="button" className="robot-quick-close" aria-label="Close robot quick detail" onClick={close}>×</button>
        </header>

        <div className="robot-quick-statusline">
          <span className={`robot-console-status ${online ? "ok" : "offline"}`}><i />{online ? "ONLINE" : "OFFLINE"}</span>
          <span className="robot-console-mode">{robot?.control_mode ?? "N/A"}</span>
          <span className="robot-quick-muted">{robot?.navigation_state ?? "N/A"}</span>
        </div>

        <div className="robot-quick-grid">
          <QuickField label="Robot ID" value={robotId} />
          <QuickField label="Robot name" value={robot?.model ?? "N/A"} />
          <QuickField label="Current mission" value={activeMission?.status ?? task?.status ?? "N/A"} />
          <QuickField label="Current task" value={task ? `${task.id} · ${task.type}` : robot?.current_task_id ?? "N/A"} />
          <QuickField label="x" value={fmt(robot?.position?.[0], 3, " m")} mono />
          <QuickField label="y" value={fmt(robot?.position?.[2], 3, " m")} mono />
          <QuickField label="yaw" value={fmt(robot?.heading, 3, " rad")} mono />
          <QuickField label="vx / vy / wz" value={`${fmt(robot?.vx, 3)} / ${fmt(robot?.vy, 3)} / ${fmt(robot?.wz, 3)}`} mono />
          <QuickField label="Battery" value={fmt(robot?.battery, 1, "%")} />
          <QuickField label="ROS status" value={robotDiagnostics ? (robotDiagnostics.ros ? "CONNECTED" : "OFFLINE") : "N/A"} />
          <QuickField label="Controller status" value={controllerState} />
          <QuickField label="LiDAR status" value={lidarState} />
          <QuickField label="Localization status" value={localizationState} />
          <QuickField label="Last seen" value={robot?.last_telemetry_at ?? lastTelemetryAt ?? "N/A"} mono />
        </div>

        {error && <div className="robot-quick-error" role="alert">{error}</div>}
        <footer className="robot-quick-actions">
          <button type="button" className="robot-console-danger" disabled={busy || !robotId} onClick={() => void action(() => emergencyStop(robotId))}>EMERGENCY STOP</button>
          <button type="button" disabled={busy || !robotId} onClick={() => void action(() => clearEmergencyStop(robotId))}>CLEAR STOP</button>
          <span className="robot-quick-action-spacer" />
          <button type="button" className="robot-console-primary" disabled={busy} onClick={() => { close(); pushRoute(`/robots/${encodeURIComponent(robotId)}/control`); }}>CONTROL ROBOT DETAIL</button>
          <button type="button" onClick={close}>CLOSE</button>
        </footer>
      </section>
    </div>,
    document.body,
  );
}

function QuickField({ label, value, mono = false }: { label: string; value: string; mono?: boolean }) {
  return <div className="robot-quick-field"><span>{label}</span><b className={mono ? "mono" : ""}>{value}</b></div>;
}
