import { useMemo } from "react";
import type { RobotState, RobotSystemDiagnostics } from "../../schema/twin_state";
import { logout } from "../../services/auth";
import { useStore } from "../../state/store";

function reportedText(value: unknown, fallback = "UNKNOWN"): string {
  return value === null || value === undefined || value === "" ? fallback : String(value);
}

function measuredPose(robot: RobotState): string {
  const pose = robot.active_map_pose;
  if (!pose?.valid || pose.frame_id !== "map"
      || ![pose.x, pose.y, pose.yaw].every(Number.isFinite)
      || !pose.map_id || !pose.map_revision) return "UNAVAILABLE";
  return `${pose.x.toFixed(3)}, ${pose.y.toFixed(3)} · ${pose.yaw.toFixed(3)} rad · ${pose.map_id} r${pose.map_revision}`;
}

function reportedBattery(robot: RobotState): string {
  return robot.battery_reported === true && typeof robot.battery === "number" && Number.isFinite(robot.battery)
    ? `${robot.battery.toFixed(0)}%` : "NOT REPORTED";
}

function measuredSensorState(value: boolean | undefined): string {
  return value === true ? "READY" : value === false ? "UNAVAILABLE" : "UNKNOWN";
}

function robotEstopState(diagnostics: RobotSystemDiagnostics | null | undefined): string {
  const active = diagnostics?.command_ownership?.estop_active;
  return typeof active === "boolean" ? (active ? "ACTIVE" : "CLEAR") : "UNKNOWN";
}

function RobotCard({ robotId, robot }: { robotId: string; robot: RobotState }) {
  const websocketState = useStore((state) => state.websocketState);
  const connectedRobotIds = useStore((state) => state.connectedRobotIds);
  const rosConnected = useStore((state) => state.rosConnected);
  const runtimeState = useStore((state) => state.runtimeState);
  const diagnostics = useStore((state) => state.robotDetail[robotId]?.diagnostics
    ?? state.rosDiagnostics) as RobotSystemDiagnostics | null;
  const appliedMode = useStore((state) => state.robotDetail[robotId]?.appliedMode);
  const robotDetail = useStore((state) => state.robotDetail[robotId]);
  const bridgeOnline = websocketState === "CONNECTED" && rosConnected && connectedRobotIds.includes(robotId);
  const mode = appliedMode ?? (robot.last_telemetry_at ? robot.control_mode : null);
  const navigationState = robotDetail?.navigationStatus ?? (robot.last_telemetry_at ? robot.navigation_state : null);
  const localization = diagnostics?.localization;
  const topics = diagnostics?.topics ?? [];
  const odometryReported = topics.some((topic) => topic === "/odom" || topic === "/odometry/filtered"
    || topic.endsWith("/odom") || topic.endsWith("/odometry/filtered"));

  return <article className="control-robot-card" data-robot-id={robotId}>
    <div className="control-robot-card-heading">
      <div><span className="control-robot-kicker">ROBOT ID</span><h2>{robotId}</h2></div>
      <span className={`control-state-pill ${bridgeOnline ? "is-online" : "is-offline"}`}>
        {bridgeOnline ? "ONLINE" : robot.status === "OFFLINE" ? "OFFLINE" : "UNKNOWN"}
      </span>
    </div>

    <dl className="control-robot-metrics">
      <div><dt>ROS bridge</dt><dd>{bridgeOnline ? "CONNECTED" : websocketState === "CONNECTED" ? "DISCONNECTED" : websocketState}</dd></div>
      <div><dt>Runtime state</dt><dd>{reportedText(runtimeState)}</dd></div>
      <div><dt>Control mode</dt><dd>{reportedText(mode)}</dd></div>
      <div><dt>Mission state</dt><dd>{reportedText(navigationState)}</dd></div>
      <div><dt>Pose · map</dt><dd>{measuredPose(robot)}</dd></div>
      <div><dt>Battery</dt><dd>{reportedBattery(robot)}</dd></div>
      <div><dt>E-stop</dt><dd>{robotEstopState(diagnostics)}</dd></div>
      <div><dt>Localization</dt><dd>{reportedText(localization)}</dd></div>
      <div><dt>LiDAR</dt><dd>{measuredSensorState(diagnostics?.lidar)}</dd></div>
      <div><dt>Odometry</dt><dd>{diagnostics ? odometryReported ? "REPORTED" : "NOT REPORTED" : "UNKNOWN"}</dd></div>
    </dl>

    <a className="control-robot-open" href={`/robots/${encodeURIComponent(robotId)}/control`}>
      Open robot control <span aria-hidden="true">→</span>
    </a>
  </article>;
}

export function RobotControlPage() {
  const robots = useStore((state) => state.twin?.robots);
  const entries = useMemo(() => Object.entries(robots ?? {}).sort(([left], [right]) => left.localeCompare(right)), [robots]);
  const runtimeMode = useStore((state) => state.runtimeMode);
  const runtimeState = useStore((state) => state.runtimeState);
  const bridgeState = useStore((state) => state.bridgeState);
  const websocketState = useStore((state) => state.websocketState);
  const authUser = useStore((state) => state.authUser);

  return <div className="robot-control-overview">
    <header className="control-app-header">
      <div className="control-app-brand"><strong>PTAGV</strong><span>/</span><b>WareTwin</b></div>
      <h1>Robot Control</h1>
      <div className="control-app-runtime" aria-label="Runtime and connection status">
        <span>RUNTIME <b>{reportedText(runtimeMode)}</b></span>
        <span>STATE <b>{reportedText(runtimeState)}</b></span>
        <span>ROS BRIDGE <b>{reportedText(bridgeState)}</b></span>
        <span>WEBSOCKET <b>{websocketState}</b></span>
      </div>
      <div className="control-app-account">
        <span>{authUser?.username ?? "UNKNOWN"}</span>
        <button type="button" onClick={() => void logout()}>Log out</button>
      </div>
    </header>

    <main className="control-overview-main">
      <div className="control-overview-heading">
        <div><span className="control-robot-kicker">LIVE RUNTIME DATA</span><h2>Robots</h2></div>
        <span>{entries.length} reported {entries.length === 1 ? "robot" : "robots"}</span>
      </div>
      {entries.length ? <section className="control-robot-grid" aria-label="Robot overview">
        {entries.map(([robotId, robot]) => <RobotCard key={robotId} robotId={robotId} robot={robot} />)}
      </section> : <section className="control-overview-empty" role="status">
        <strong>Waiting for robot runtime data</strong>
        <span>Robot status will appear when it is reported by the backend runtime.</span>
        <span>Runtime: {reportedText(runtimeMode)} · WebSocket: {websocketState}</span>
      </section>}
    </main>
  </div>;
}
