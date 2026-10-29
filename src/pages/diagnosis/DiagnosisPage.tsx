import { useEffect, useState } from "react";
import { getSystemStatus, getVda5050Configuration, type SystemStatus } from "../../services/api";
import type { RobotDetailError, RobotLidarStreamDiagnostics, RobotSystemDiagnostics } from "../../schema/twin_state";
import { useStore } from "../../state/store";
import { IS_SHOWCASE, IS_STATIC_UI as STATIC_UI_SETTING } from "../../config";
import { displayRuntimeRows, displayRuntimeValue, STATIC_UI_EMPTY_VALUE } from "../../mock/staticPresentation";
import { useLocalAppRuntime, type LocalAppRuntime } from "../../app/AppLayout";
import { Card, CardBody, CardHeader, EmptyState, MetricRow, SectionHeading } from "../../components/shared/ui/HmiPrimitives";
import { Status, classForStatus, valueNumber } from "../../components/shared/ui/RobotPagePrimitives";

const IS_STATIC_UI = import.meta.env.DEV && STATIC_UI_SETTING;

type DiagnosisPanelProps = { robotId: string; diagnostics: RobotSystemDiagnostics | null; errors: RobotDetailError[];
  controlOnline: boolean; runtimeState: string; localization: unknown; websocketState: string; mapRevision: number | null;
  estopState: LocalAppRuntime["estopState"];
  lidarStreamDiagnostics: RobotLidarStreamDiagnostics | null; activeLocalMapId: string | null;
  activeLocalMapRevision: string | null; localMapSyncStatus: string | null; hostStatus: SystemStatus | null };

export function DiagnosisPage() {
  const runtime = useLocalAppRuntime();
  const robotId = runtime.robotId;
  if (!robotId) return null;
  return <DiagnosisPageContent robotId={robotId} runtime={runtime} />;
}

function DiagnosisPageContent({ robotId, runtime }: { robotId: string; runtime: LocalAppRuntime }) {
  const [host, setHost] = useState<SystemStatus | null>(null);
  useEffect(() => {
    if (IS_STATIC_UI) { setHost(null); return; }
    let cancelled = false;
    const loadHostStatus = async () => {
      try {
        const body = await getSystemStatus();
        if (!cancelled) setHost(body);
      } catch { if (!cancelled) setHost(null); }
    };
    void loadHostStatus();
    const timer = IS_SHOWCASE ? undefined : window.setInterval(() => void loadHostStatus(), 5000);
    return () => { cancelled = true; if (timer !== undefined) window.clearInterval(timer); };
  }, []);
  const detail = useStore((state) => state.robotDetail[robotId]);
  const rawDiagnostics = useStore((state) => state.rosDiagnostics);
  const diagnostics = (detail?.diagnostics ?? runtime.diagnostics ?? rawDiagnostics) as RobotSystemDiagnostics | null;
  const errors = detail?.errors?.length ? detail.errors : detail?.diagnostics?.errors ?? runtime.diagnostics?.errors ?? [];
  const runtimeState = useStore((state) => state.runtimeState);
  const localizationState = useStore((state) => state.localization?.state);
  const localization = runtime.diagnostics?.localization ?? localizationState ?? null;
  const websocketState = useStore((state) => state.websocketState);
  const mapRevision = useStore((state) => state.mapSync.publishedRevision);
  return <main className="flex min-h-0 min-w-0 flex-1 flex-col gap-hmi-2 overflow-auto bg-ptagv-page p-hmi-2">
    <DiagnosticsPanel robotId={robotId} diagnostics={diagnostics} errors={errors} controlOnline={runtime.controlOnline}
      runtimeState={runtimeState} localization={localization} websocketState={websocketState} mapRevision={mapRevision}
      estopState={runtime.estopState}
      lidarStreamDiagnostics={detail?.lidarStreamDiagnostics ?? null} activeLocalMapId={detail?.activeLocalMapId ?? null}
      activeLocalMapRevision={detail?.activeLocalMapRevision ?? null} localMapSyncStatus={detail?.localMapSyncStatus ?? null}
      hostStatus={host} />
  </main>;
}

function DiagnosticsPanel({ robotId, diagnostics, errors, controlOnline, runtimeState, localization, websocketState, mapRevision, estopState, lidarStreamDiagnostics, activeLocalMapId, activeLocalMapRevision, localMapSyncStatus, hostStatus }: DiagnosisPanelProps) {
  const [vdaStatus, setVdaStatus] = useState("UNKNOWN");
  const [vdaError, setVdaError] = useState("");
  const [vdaRuntime, setVdaRuntime] = useState<{ enabled: boolean; host: string; port: number; allowTask: boolean; instantActionsSupported: boolean } | null>(null);
  useEffect(() => {
    if (IS_STATIC_UI) {
      let active = true;
      void import("../../mock/fixtures").then(({ STATIC_VDA5050_CONFIGURATION }) => {
        if (!active) return;
        setVdaStatus(STATIC_VDA5050_CONFIGURATION.connection_status);
        setVdaError("");
        setVdaRuntime({ enabled: STATIC_VDA5050_CONFIGURATION.enabled, host: STATIC_VDA5050_CONFIGURATION.mqtt_host,
          port: STATIC_VDA5050_CONFIGURATION.mqtt_port, allowTask: STATIC_VDA5050_CONFIGURATION.allow_task,
          instantActionsSupported: STATIC_VDA5050_CONFIGURATION.instant_actions_supported });
      });
      return () => { active = false; };
    }
    let active = true;
    const refreshConnection = () => getVda5050Configuration(robotId).then((config) => {
      if (active) {
        setVdaStatus(config.connection_status); setVdaError(config.last_error ?? "");
        setVdaRuntime({ enabled: config.enabled, host: config.mqtt_host, port: config.mqtt_port, allowTask: config.allow_task, instantActionsSupported: config.instant_actions_supported });
      }
    }).catch(() => { if (active) setVdaStatus("UNAVAILABLE"); });
    void refreshConnection();
    const timer = IS_SHOWCASE ? undefined : window.setInterval(() => { void refreshConnection(); }, 3000);
    return () => { active = false; if (timer !== undefined) window.clearInterval(timer); };
  }, [robotId]);
  const metrics = diagnostics?.metrics ?? {};
  const command = diagnostics?.command_ownership;
  const stream = lidarStreamDiagnostics;
  const slamState = IS_STATIC_UI ? "UNAVAILABLE" : diagnostics?.mapping?.slam_state === "ACTIVE" && diagnostics.mapping.map_live
    ? "LIVE" : diagnostics?.mapping?.slam_state ?? (diagnostics?.slam ? "STARTING" : "INACTIVE");
  const nav2State = IS_STATIC_UI ? "UNAVAILABLE" : diagnostics?.nav2_ready ? "READY" : diagnostics?.nav2 ? "STARTING" : "INACTIVE";
  const displayedActiveMapId = IS_STATIC_UI ? "SAMPLE MAP" : activeLocalMapId ?? "NONE REPORTED";
  const cpuLoad = hostStatus?.system?.cpu_load_1m;
  const ramUsed = hostStatus?.system?.memory?.used_percent;
  const systemRows = displayRuntimeRows(IS_STATIC_UI, [
    ["ROS", diagnostics?.ros ? "READY" : "OFFLINE"],
    ["Gazebo", diagnostics?.gazebo ? "READY" : "OFFLINE"],
    ["ROS Bridge", controlOnline ? "CONNECTED" : "DISCONNECTED"],
    ["WebSocket", websocketState],
    ["Controllers", diagnostics?.controller_manager ? "ACTIVE" : "UNKNOWN"],
    ["SLAM", slamState], ["Nav2", nav2State],
    ["LiDAR", diagnostics?.lidar ? "ACTIVE" : "UNAVAILABLE"],
  ]);
  const detailRows = displayRuntimeRows(IS_STATIC_UI, [
    ["LOCALIZATION", String(localization ?? "UNKNOWN")],
    [IS_STATIC_UI ? "MAP DATA" : "ACTIVE LOCAL MAP", displayedActiveMapId],
    ["MAP SYNC", localMapSyncStatus ?? diagnostics?.map_state?.map_sync_status ?? "UNKNOWN"],
    ["CONTROL MODE", command?.active_control_mode ?? "UNKNOWN"],
    ["COMMAND SOURCE", command?.active_command_source ?? "NONE"],
    ["LIDAR STREAM", stream?.source_fps == null ? (diagnostics?.lidar ? "ACTIVE" : "UNAVAILABLE") : valueNumber(stream.source_fps, 1, " Hz")],
    ["ODOMETRY", IS_STATIC_UI ? "UNAVAILABLE" : diagnostics?.mapping?.odom_live ? "RECEIVING" : "WAITING"],
    ["TF", diagnostics?.tf ? "AVAILABLE" : "UNAVAILABLE"],
    ["VDA5050 MQTT", vdaStatus],
    ["E-STOP", estopState],
  ]);
  return <div className="grid min-h-0 min-w-0 flex-1 auto-rows-max content-start items-stretch gap-hmi-3 overflow-auto xl:grid-cols-2">
    <Card className="min-w-0 overflow-hidden" aria-labelledby="diagnosis-subsystem-title">
      <CardHeader><SectionHeading id="diagnosis-subsystem-title" className="text-hmi-kicker uppercase tracking-wider">SUBSYSTEM STATUS</SectionHeading></CardHeader>
      <CardBody className="flex-1 gap-hmi-3">
      <div className="grid min-w-0 grid-cols-1 gap-x-hmi-4 sm:grid-cols-2">
        {systemRows.map(([name, value]) => <div className="grid min-h-8 min-w-0 grid-cols-[minmax(0,1fr)_auto] items-center gap-hmi-2 border-b border-ptagv-border-soft px-1 py-hmi-1 text-sm" key={name}><span className="min-w-0 leading-snug text-ptagv-text">{name}</span><Status value={value} /></div>)}
      </div>
      <div className="grid min-w-0 grid-cols-3 gap-x-hmi-2 border-t border-ptagv-border-soft pt-hmi-2">
        <MetricRow layout="stacked" label="CPU LOAD" value={displayRuntimeValue(IS_STATIC_UI, cpuLoad == null ? "N/A" : valueNumber(cpuLoad, 2))} mono className="border-r-0 px-1 py-1" />
        <MetricRow layout="stacked" label="RAM USED" value={displayRuntimeValue(IS_STATIC_UI, ramUsed == null ? "N/A" : valueNumber(ramUsed, 1, "%"))} mono className="border-r-0 px-1 py-1" />
        <MetricRow layout="stacked" label="GAZEBO RTF" value={displayRuntimeValue(IS_STATIC_UI, valueNumber(diagnostics?.gazebo_rtf ?? metrics.gazebo_rtf, 3))} mono className="border-r-0 px-1 py-1" />
      </div>
      <details className="mt-auto flex min-w-0 flex-none flex-col-reverse border-t border-ptagv-border-soft">
        <summary className="flex min-h-9 cursor-pointer items-center py-hmi-2 text-xs font-bold text-ptagv-muted">ADVANCED RUNTIME DETAILS</summary>
        <div className="grid min-w-0 grid-cols-1 gap-x-hmi-4 sm:grid-cols-2">
          <MetricRow label="ROBOT" value={displayRuntimeValue(IS_STATIC_UI, "LOCAL")} mono /><MetricRow label="RUNTIME MODE" value={displayRuntimeValue(IS_STATIC_UI, runtimeState)} />
          <MetricRow label="LOCALIZATION" value={displayRuntimeValue(IS_STATIC_UI, String(localization ?? "UNKNOWN"))} /><MetricRow label="TF MAP → BASE" value={displayRuntimeValue(IS_STATIC_UI, diagnostics?.tf ? "AVAILABLE" : "UNAVAILABLE")} />
          <MetricRow label="VDA5050 MQTT" value={displayRuntimeValue(IS_STATIC_UI, vdaStatus)} /><MetricRow label="VDA5050 ENABLED" value={displayRuntimeValue(IS_STATIC_UI, vdaRuntime?.enabled ? "YES" : "NO")} />
          <MetricRow label="MQTT BROKER" value={displayRuntimeValue(IS_STATIC_UI, vdaRuntime ? `${vdaRuntime.host}:${vdaRuntime.port}` : "N/A")} mono />
          <MetricRow label="ALLOW TASK" value={displayRuntimeValue(IS_STATIC_UI, vdaRuntime?.allowTask ? "ENABLED" : "BLOCKED")} />
          <MetricRow label="INSTANT ACTIONS" value={displayRuntimeValue(IS_STATIC_UI, vdaRuntime?.instantActionsSupported ? "SUPPORTED" : "NOT IMPLEMENTED")} />
          <MetricRow label="MAP REVISION" value={displayRuntimeValue(IS_STATIC_UI, mapRevision ?? "N/A")} mono /><MetricRow label={IS_STATIC_UI ? "MAP DATA" : "LOCAL ACTIVE MAP"} value={displayRuntimeValue(IS_STATIC_UI, displayedActiveMapId)} mono />
          <MetricRow label="LOCAL MAP REVISION" value={displayRuntimeValue(IS_STATIC_UI, activeLocalMapRevision ?? "N/A")} mono /><MetricRow label="CANONICAL REVISION" value={displayRuntimeValue(IS_STATIC_UI, mapRevision ?? "N/A")} mono />
          <MetricRow label="MAP SYNC STATUS" value={displayRuntimeValue(IS_STATIC_UI, localMapSyncStatus ?? diagnostics?.map_state?.map_sync_status ?? "UNKNOWN")} />
          <MetricRow label="CONTROL MODE" value={displayRuntimeValue(IS_STATIC_UI, command?.active_control_mode ?? "UNKNOWN")} />
          <MetricRow label="ACTIVE COMMAND SOURCE" value={displayRuntimeValue(IS_STATIC_UI, command?.active_command_source ?? "UNKNOWN")} mono />
          <MetricRow label="LAST COMMAND AGE" value={displayRuntimeValue(IS_STATIC_UI, command?.last_command_age == null ? "N/A" : valueNumber(command.last_command_age, 2, " s"))} mono />
          <MetricRow label="MANUAL SOURCE" value={displayRuntimeValue(IS_STATIC_UI, command?.manual_source_active ? "ACTIVE" : "IDLE")} />
          <MetricRow label="NAV SOURCE" value={displayRuntimeValue(IS_STATIC_UI, command?.nav_source_active ? "ACTIVE" : "IDLE")} />
          <MetricRow label="E-STOP" value={displayRuntimeValue(IS_STATIC_UI, estopState)} />
          <MetricRow label="LIDAR SOURCE FPS" value={displayRuntimeValue(IS_STATIC_UI, stream?.source_fps == null ? "N/A" : valueNumber(stream.source_fps, 2, " Hz"))} mono />
          <MetricRow label="LIDAR WEB OUTPUT FPS" value={displayRuntimeValue(IS_STATIC_UI, stream?.web_output_fps == null ? "N/A" : valueNumber(stream.web_output_fps, 2, " Hz"))} mono />
          <MetricRow label="LIDAR POINT COUNT" value={displayRuntimeValue(IS_STATIC_UI, stream?.point_count ?? "N/A")} mono />
          <MetricRow label="LIDAR DROPPED FRAMES" value={displayRuntimeValue(IS_STATIC_UI, stream?.dropped_frames ?? "N/A")} mono />
          <MetricRow label="SIMULATION TIME" value={displayRuntimeValue(IS_STATIC_UI, valueNumber(diagnostics?.simulation_time, 3, " s"))} mono />
          <MetricRow label="WEBSOCKET LATENCY" value={displayRuntimeValue(IS_STATIC_UI, valueNumber(diagnostics?.websocket_latency_ms, 0, " ms"))} mono />
          <MetricRow label="MQTT LAST ERROR" value={displayRuntimeValue(IS_STATIC_UI, vdaError || "NONE")} />
        </div>
      </details>
      </CardBody>
    </Card>
    <Card className="min-w-0 overflow-hidden" aria-labelledby="diagnosis-runtime-title">
      <CardHeader><SectionHeading id="diagnosis-runtime-title" className="text-hmi-kicker uppercase tracking-wider">RUNTIME / SENSOR STATUS</SectionHeading></CardHeader>
      <CardBody className="flex-1 gap-hmi-3">
      <div className="grid min-w-0 grid-cols-1 gap-x-hmi-4 sm:grid-cols-2">
        {detailRows.map(([name, value]) => <div className="grid min-h-8 min-w-0 grid-cols-[minmax(0,1fr)_auto] items-center gap-hmi-2 border-b border-ptagv-border-soft px-1 py-hmi-1 text-sm" key={name}><span className="min-w-0 leading-snug text-ptagv-text">{name}</span><Status value={value} /></div>)}
      </div>
      <details className="mt-auto flex min-w-0 flex-none flex-col-reverse border-t border-ptagv-border-soft">
        <summary className="flex min-h-9 cursor-pointer items-center py-hmi-2 text-xs font-bold text-ptagv-muted">ADVANCED ROS GRAPH</summary>
        <div className="grid min-w-0 grid-cols-1 gap-x-hmi-4">
          <MetricRow label="ACTIVE NODES" value={displayRuntimeValue(IS_STATIC_UI, diagnostics?.nodes?.length ?? 0)} mono />
          <MetricRow label="TOPICS" value={displayRuntimeValue(IS_STATIC_UI, diagnostics?.topics?.length ?? 0)} mono />
          <MetricRow label="CONTROLLERS" value={displayRuntimeValue(IS_STATIC_UI, (diagnostics?.controllers ?? []).map((row) => `${row.name}:${row.state}`).join(" · ") || "N/A")} />
        </div>
        <div className="grid gap-hmi-1 pt-hmi-2 font-mono text-xs text-ptagv-muted"><code>/map · /scan · /lidar/points · /lidar/points_filtered</code><code>/tf · /tf_static · /odom · /odometry/filtered</code><code>/navigate_to_pose · /compute_path_to_pose · /cmd_vel_selected</code></div>
      </details>
      </CardBody>
    </Card>
    <Card className="min-w-0 overflow-hidden xl:col-span-2" aria-labelledby="diagnosis-errors-title">
      <CardHeader><SectionHeading id="diagnosis-errors-title" className="text-hmi-kicker uppercase tracking-wider">LIVE ERROR LOG</SectionHeading></CardHeader>
      <CardBody className="max-h-52 overflow-auto">
        {IS_STATIC_UI || errors.length === 0 ? <EmptyState className="min-h-16">{IS_STATIC_UI ? STATIC_UI_EMPTY_VALUE : "No runtime errors reported."}</EmptyState> : errors.map((item, index) => {
          const severity = classForStatus(item.severity);
          const severityClass: Record<string, string> = { ok: "text-ptagv-success", warning: "text-ptagv-warning", error: "text-ptagv-error", neutral: "text-ptagv-muted" };
          return <div className="grid min-w-0 grid-cols-[minmax(0,1fr)_auto] gap-hmi-2 border-b border-ptagv-border-soft py-hmi-2" key={`${item.code ?? item.message}-${index}`}>
            <div className="flex min-w-0 flex-col gap-1"><b className={`text-xs ${severityClass[severity]}`}>{item.severity}</b><span className="text-sm leading-relaxed text-ptagv-text [overflow-wrap:anywhere]">{item.message}</span></div>
            <small className="text-xs text-ptagv-muted">{item.timestamp ?? "N/A"}</small>
          </div>;
        })}
      </CardBody>
    </Card>
  </div>;
}
