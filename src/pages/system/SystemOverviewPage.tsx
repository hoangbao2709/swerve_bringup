import { useEffect, useMemo, useState } from "react";
import { useLocalAppRuntime } from "../../app/AppLayout";
import { Alert, Badge, Card, CardBody, CardHeader, MetricRow, SectionHeading } from "../../components/shared/ui/HmiPrimitives";
import { IS_SHOWCASE, IS_STATIC_UI, RUNTIME_MODE } from "../../config";
import { getLocalAppSettings } from "../../services/settingsPreview";
import { getSystemStatus, type SystemStatus } from "../../services/api";
import { LOCAL_APP_PATHS, handleLocalLinkClick } from "../../routing/localRoutes";
import { useStore } from "../../state/store";

function displayed(value: string | number | null | undefined): string {
  return value === null || value === undefined || value === "" ? "—" : String(value);
}

function formatBytes(value: number | null | undefined): string {
  if (value === null || value === undefined || !Number.isFinite(value) || value < 0) return "—";
  const units = ["B", "KB", "MB", "GB", "TB"];
  let amount = value;
  let index = 0;
  while (amount >= 1024 && index < units.length - 1) { amount /= 1024; index += 1; }
  return `${amount.toFixed(index === 0 ? 0 : 1)} ${units[index]}`;
}

function formatClock(date: Date, timezone?: string): string {
  try {
    return new Intl.DateTimeFormat("en-GB", {
      dateStyle: "medium", timeStyle: "medium", ...(timezone ? { timeZone: timezone } : {}),
    }).format(date);
  } catch {
    return date.toLocaleString();
  }
}

function ResourceMetric({ title, value, detail }: { title: string; value: string; detail: string }) {
  return <div className="grid min-w-0 gap-1 border-b border-r border-ptagv-border-soft px-3 py-2 last:border-r-0">
    <span className="text-[0.62rem] font-bold tracking-wide text-ptagv-muted">{title}</span>
    <b className="text-sm text-ptagv-text">{value}</b>
    <small className="text-xs text-ptagv-muted">{detail}</small>
  </div>;
}

function InfoLink({ href, children }: { href: string; children: string }) {
  return <a href={href} onClick={(event) => handleLocalLinkClick(event, href)}
    className="inline-flex min-h-9 items-center border border-ptagv-border bg-white px-3 text-xs font-bold text-ptagv-blue no-underline hover:bg-slate-50 focus-visible:outline-2 focus-visible:outline-offset-1 focus-visible:outline-ptagv-blue">
    {children}
  </a>;
}

export function SystemOverviewPage() {
  const runtime = useLocalAppRuntime();
  const identityVerified = useStore((state) => state.localIdentityVerified);
  const [host, setHost] = useState<SystemStatus | null>(null);
  const [loading, setLoading] = useState(!IS_STATIC_UI);
  const [error, setError] = useState("");
  const [clockNow, setClockNow] = useState(() => new Date());
  const localSettings = useMemo(() => getLocalAppSettings(), []);
  const browserTimezone = IS_STATIC_UI || IS_SHOWCASE
    ? localSettings.ntp.timezone
    : Intl.DateTimeFormat().resolvedOptions().timeZone;

  useEffect(() => {
    const timer = window.setInterval(() => setClockNow(new Date()), 1000);
    return () => window.clearInterval(timer);
  }, []);

  useEffect(() => {
    if (IS_STATIC_UI) {
      setHost(null);
      setLoading(false);
      return;
    }
    let active = true;
    let timer: number | undefined;
    let controller: AbortController | null = null;
    const refresh = async () => {
      controller = new AbortController();
      try {
        const result = await getSystemStatus(controller.signal);
        if (active) { setHost(result); setError(""); }
      } catch (caught) {
        if (active && !(caught instanceof DOMException && caught.name === "AbortError")) {
          setError(caught instanceof Error ? caught.message : "System status could not be loaded.");
        }
      } finally {
        if (active) {
          setLoading(false);
          if (!IS_SHOWCASE) timer = window.setTimeout(() => void refresh(), 10_000);
        }
      }
    };
    void refresh();
    return () => {
      active = false;
      controller?.abort();
      if (timer !== undefined) window.clearTimeout(timer);
    };
  }, []);

  const system = host?.system;
  const memory = system?.memory;
  const disk = system?.disk;
  const robotId = IS_SHOWCASE ? runtime.robotId : !IS_STATIC_UI && identityVerified ? runtime.robotId : null;
  const robot = !IS_STATIC_UI ? runtime.robot : undefined;
  const backendTimestamp = host?.timestamp ? formatClock(new Date(host.timestamp), "UTC") : "—";
  const cpuLoad = system?.cpu_load_1m;
  const cpuValue = typeof cpuLoad === "number" && Number.isFinite(cpuLoad)
    ? `${cpuLoad.toFixed(2)}${system?.cpu_count ? ` / ${system.cpu_count} cores` : ""}`
    : "—";

  return <main aria-label="System overview" className="flex min-h-0 min-w-0 flex-1 flex-col gap-hmi-3 overflow-auto bg-ptagv-page p-hmi-3">
    <div className="flex flex-wrap items-center justify-between gap-hmi-3">
      <div className="flex flex-wrap gap-2" role="group" aria-label="Related system pages">
        <InfoLink href={LOCAL_APP_PATHS.DIAGNOSIS}>DIAGNOSIS</InfoLink>
        <InfoLink href={LOCAL_APP_PATHS.SETTINGS}>SETTINGS</InfoLink>
      </div>
    </div>

    {error && <Alert tone="error" role="alert">System status could not be refreshed. {error}</Alert>}

    <div className="grid min-w-0 grid-cols-1 gap-hmi-3 xl:grid-cols-2">
      <Card className="min-w-0">
        <CardHeader><SectionHeading className="text-hmi-kicker uppercase tracking-wider">ROBOT & RUNTIME</SectionHeading></CardHeader>
        <CardBody className="gap-0">
          <MetricRow label="ROBOT IDENTITY" value={displayed(robotId)} mono />
          <MetricRow label="ROBOT MODEL" value={displayed(robot?.model)} />
          <MetricRow label="ROBOT STATE" value={displayed(robot?.status)} />
          <MetricRow label="CONTROL MODE" value={displayed(robot?.control_mode)} />
          <MetricRow label="RUNTIME PROFILE" value={displayed(IS_SHOWCASE ? "STANDALONE SAMPLE" : host?.runtime_mode ?? (IS_STATIC_UI ? RUNTIME_MODE : null))} />
          <MetricRow label="MAPPING / NAVIGATION MODE" value={displayed(host?.mode)} />
          <MetricRow label="APPLICATION VERSION" value={displayed(IS_SHOWCASE ? "Web Local" : host?.version)} mono />
        </CardBody>
      </Card>

      <Card className="min-w-0">
        <CardHeader><SectionHeading className="text-hmi-kicker uppercase tracking-wider">SYSTEM HEALTH</SectionHeading></CardHeader>
        <CardBody className="gap-0">
          <MetricRow label="SYSTEM STATUS" value={host ? <Badge tone={host.system_ready ? "success" : "warning"}>{displayed(host.system_status)}</Badge> : loading ? "Loading" : "—"} />
          <MetricRow label="BACKEND STATUS" value={host ? <Badge tone={host.backend_ready ? "success" : "warning"}>{displayed(host.backend_status)}</Badge> : loading ? "Loading" : "—"} />
          <MetricRow label="DATABASE" value={host?.database === true ? "Ready" : host?.database === false ? "Needs attention" : "—"} />
          <MetricRow label="ROBOT BRIDGE" value={displayed(host?.runtime?.bridge_state)} />
          <MetricRow label="ROS CONNECTION" value={host?.runtime?.ros_connected === true ? "Connected" : host?.runtime?.ros_connected === false ? "Not connected" : "—"} />
          <MetricRow label="ONLINE ROBOTS" value={displayed(host?.runtime?.online_robot_ids?.join(", "))} />
        </CardBody>
      </Card>

      <Card className="min-w-0">
        <CardHeader><SectionHeading className="text-hmi-kicker uppercase tracking-wider">HOST RESOURCES</SectionHeading></CardHeader>
        <CardBody className="gap-0">
          <div className="grid min-w-0 grid-cols-1 border border-ptagv-border-soft sm:grid-cols-3">
            <ResourceMetric title="CPU LOAD · 1 MIN" value={cpuValue} detail="Host load average; not CPU percentage." />
            <ResourceMetric title="MEMORY USED" value={memory?.used_percent == null ? "—" : `${memory.used_percent.toFixed(1)}%`}
              detail={memory?.used_bytes == null || memory.total_bytes == null ? "Host memory details" : `${formatBytes(memory.used_bytes)} of ${formatBytes(memory.total_bytes)}`} />
            <ResourceMetric title="STORAGE USED" value={disk?.used_percent == null ? "—" : `${disk.used_percent.toFixed(1)}%`}
              detail={disk?.free_bytes == null ? "Host storage details" : `${formatBytes(disk.free_bytes)} free`} />
          </div>
          <MetricRow label="HOST UPTIME" value={system?.uptime_s == null ? "—" : `${Math.floor(system.uptime_s / 3600)} h`} mono />
        </CardBody>
      </Card>

      <Card className="min-w-0">
        <CardHeader><SectionHeading className="text-hmi-kicker uppercase tracking-wider">TIME & NETWORK</SectionHeading></CardHeader>
        <CardBody className="gap-0">
          <MetricRow label="BROWSER CLOCK" value={formatClock(clockNow, browserTimezone)} mono />
          <MetricRow label="BROWSER TIMEZONE" value={displayed(browserTimezone)} mono />
          <MetricRow label="ROBOT TIMEZONE" value="—" />
          <MetricRow label="LAST HOST STATUS" value={backendTimestamp} mono />
          <MetricRow label="HOST NETWORK INTERFACES" value={IS_SHOWCASE ? "NOT CONNECTED" : "NOT REPORTED"} />
          <MetricRow label="CONTROL TRANSPORT" value={displayed(host?.runtime?.bridge_state)} />
        </CardBody>
      </Card>
    </div>

    {!IS_SHOWCASE && <Alert tone="neutral" role="note" className="text-xs">
      {IS_STATIC_UI
        ? "Host measurements and robot identity are not reported in this browser session. The displayed clock is the browser clock formatted with the selected local timezone."
        : "System status is refreshed from the backend. The browser clock and timezone do not represent the robot host clock unless the host service reports them."}
    </Alert>}
  </main>;
}
