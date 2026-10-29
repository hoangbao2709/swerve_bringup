import { useEffect, useRef, useState } from "react";
import { applyVda5050Configuration, getVda5050Configuration, testVda5050Connection, type Vda5050Configuration } from "../../services/api";
import { IS_SHOWCASE, IS_STATIC_UI as STATIC_UI_SETTING } from "../../config";
import { Alert, Button, Card, CardBody, CardHeader, Input, LoadingState, SectionHeading, Select } from "../../components/shared/ui/HmiPrimitives";
import { Status } from "../../components/shared/ui/RobotPagePrimitives";
import { useLocalAppRuntime } from "../../app/AppLayout";

const IS_STATIC_UI = import.meta.env.DEV && STATIC_UI_SETTING;

export function Vda5050Page() {
  const { robotId } = useLocalAppRuntime();
  if (!robotId) return null;

  return <main className="flex min-h-0 min-w-0 flex-1 flex-col overflow-hidden bg-ptagv-page p-hmi-3">
    <Vda5050Panel robotId={robotId} />
  </main>;
}

type VdaDraft = Omit<Vda5050Configuration, "connection_status" | "last_error" | "ignored_orders" | "updated_at"> & { mqtt_password: string };
type VdaField = "mqtt_host" | "mqtt_port" | "protocol_version" | "mqtt_protocol_version" | "reconnect_interval" | "connection_timeout" | "keepalive";
type VdaFieldErrors = Partial<Record<VdaField, string>>;
type SectionIconKind = "broker" | "identity" | "runtime";

const editableVdaFields = [
  "enabled", "mqtt_host", "mqtt_port", "mqtt_username", "tls_enabled", "topic_prefix",
  "interface_name", "manufacturer", "serial_number", "protocol_version",
  "mqtt_protocol_version", "allow_task", "auto_reconnect",
  "reconnect_interval", "connection_timeout", "keepalive", "client_id",
] as const;

function vdaSnapshot(draft: VdaDraft): string {
  return JSON.stringify(Object.fromEntries(editableVdaFields.map((key) => [key, draft[key]])));
}

function validateDraft(draft: VdaDraft): VdaFieldErrors {
  const errors: VdaFieldErrors = {};
  const host = draft.mqtt_host.trim();
  const port = Number(draft.mqtt_port);
  const reconnectInterval = Number(draft.reconnect_interval);
  const connectionTimeout = Number(draft.connection_timeout);
  const keepalive = Number(draft.keepalive);

  if (draft.enabled && !host) errors.mqtt_host = "Enter a broker host while the connection is enabled.";
  else if (host && (host.length > 253 || /\s/.test(host))) errors.mqtt_host = "Enter a hostname or IP address without spaces.";
  if (!Number.isInteger(port) || port < 1 || port > 65535) errors.mqtt_port = "Port must be a whole number from 1 to 65535.";
  if (!["2.0.0", "2.1.0", "3.0.0"].includes(draft.protocol_version)) errors.protocol_version = "Select a supported VDA5050 protocol version.";
  if (!["3.1.1", "5.0"].includes(draft.mqtt_protocol_version)) errors.mqtt_protocol_version = "Select a supported MQTT version.";
  if (!Number.isInteger(reconnectInterval) || reconnectInterval < 1 || reconnectInterval > 300) errors.reconnect_interval = "Use a whole number from 1 to 300 seconds.";
  if (!Number.isInteger(connectionTimeout) || connectionTimeout < 1 || connectionTimeout > 60) errors.connection_timeout = "Use a whole number from 1 to 60 seconds.";
  if (!Number.isInteger(keepalive) || keepalive < 5 || keepalive > 3600) errors.keepalive = "Use a whole number from 5 to 3600 seconds.";
  return errors;
}

function formatRuntimeLabel(value: string): string {
  if (!value) return "Unknown";
  const words = value.replace(/_/g, " ").toLowerCase();
  return words.charAt(0).toUpperCase() + words.slice(1);
}

function formatConnectionStatus(value: string): string {
  const normalized = value.trim().toUpperCase();
  const labels: Record<string, string> = {
    CONNECTED: "Connected",
    CONNECTING: "Connecting",
    DISCONNECTED: "Disconnected",
    DISABLED: "Disabled",
    ERROR: "Failed",
    FAILED: "Failed",
    UNKNOWN: "Unknown",
    UNAVAILABLE: "Status unavailable",
  };
  return labels[normalized] ?? formatRuntimeLabel(value);
}

function VdaSectionIcon({ kind }: { kind: SectionIconKind }) {
  return <svg aria-hidden="true" viewBox="0 0 20 20" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round" className="size-4 shrink-0 text-ptagv-blue">
    {kind === "broker" ? <>
      <rect x="3.5" y="3.5" width="13" height="5" rx="1" />
      <rect x="3.5" y="11.5" width="13" height="5" rx="1" />
      <path d="M6 6h.01M6 14h.01M9 6h4M9 14h4" />
    </> : kind === "identity" ? <>
      <path d="M4 4.5h12v11H4z" />
      <circle cx="8" cy="8" r="1.5" />
      <path d="M6.5 13c.4-1.3 2.6-1.3 3 0M12 8h2M12 11h2M12 14h2" />
    </> : <>
      <path d="M4 5h12M4 10h12M4 15h12" />
      <circle cx="8" cy="5" r="1.4" fill="currentColor" />
      <circle cx="13" cy="10" r="1.4" fill="currentColor" />
      <circle cx="7" cy="15" r="1.4" fill="currentColor" />
    </>}
  </svg>;
}

function VdaSection({ id, title, icon, children }: { id: string; title: string; icon: SectionIconKind; children: React.ReactNode }) {
  return <Card className="min-w-0 overflow-hidden" aria-labelledby={id}>
    <CardHeader className="min-h-11 shrink-0 bg-white px-hmi-3 py-hmi-2">
      <div className="flex min-w-0 items-center gap-2.5">
        <VdaSectionIcon kind={icon} />
        <SectionHeading id={id} className="text-sm font-bold">{title}</SectionHeading>
      </div>
    </CardHeader>
    <CardBody className="min-w-0 gap-hmi-3 p-hmi-3">{children}</CardBody>
  </Card>;
}

function Field({ label, error, errorId, children }: { label: string; error?: string; errorId?: string; children: React.ReactNode }) {
  return <label className="grid min-w-0 content-start gap-1.5 text-sm font-semibold text-ptagv-text">
    <span>{label}</span>
    {children}
    {error && <span id={errorId} className="text-xs font-medium leading-snug text-ptagv-error" role="alert">{error}</span>}
  </label>;
}

function SwitchField({ label, checked, disabled, onChange }: { label: string; checked: boolean; disabled: boolean; onChange: (checked: boolean) => void }) {
  return <label className={"flex min-h-12 min-w-0 cursor-pointer items-center justify-between gap-hmi-3 border border-ptagv-border-soft bg-white px-hmi-3 py-hmi-2 transition-colors hover:bg-slate-50 " + (disabled ? "cursor-not-allowed opacity-60" : "")}>
    <span className="min-w-0 text-sm font-semibold text-ptagv-text">{label}</span>
    <span className="relative inline-flex shrink-0">
      <input className="peer sr-only" type="checkbox" role="switch" aria-label={label} checked={checked} disabled={disabled}
        onChange={(event) => onChange(event.target.checked)} />
      <span aria-hidden="true" className={"relative inline-flex h-6 w-11 items-center rounded-full transition-colors peer-focus-visible:ring-2 peer-focus-visible:ring-offset-2 peer-focus-visible:ring-ptagv-blue " + (checked ? "bg-ptagv-blue" : "bg-slate-300")}>
        <span className={"absolute left-1 size-4 rounded-full bg-white shadow-sm transition-transform " + (checked ? "translate-x-5" : "translate-x-0")} />
      </span>
    </span>
  </label>;
}

function SummaryMetric({ label, value, divided = true }: { label: string; value: React.ReactNode; divided?: boolean }) {
  return <div className={"grid min-w-0 content-center gap-1 " + (divided ? "border-t border-ptagv-border-soft pt-hmi-2 sm:border-t-0 sm:pt-0 xl:border-l xl:pl-hmi-3" : "")}>
    <span className="text-xs font-semibold text-ptagv-muted">{label}</span>
    <strong className="min-w-0 text-sm font-bold text-ptagv-text [overflow-wrap:anywhere]">{value}</strong>
  </div>;
}

function Vda5050Panel({ robotId }: { robotId: string }) {
  const pageContentRef = useRef<HTMLDivElement>(null);
  const overviewRef = useRef<HTMLElement>(null);
  const formRef = useRef<HTMLFormElement>(null);
  const sectionsRef = useRef<HTMLDivElement>(null);
  const actionBarRef = useRef<HTMLDivElement>(null);
  const operationInFlightRef = useRef(false);
  const [draft, setDraft] = useState<VdaDraft | null>(null);
  const [savedDraft, setSavedDraft] = useState<VdaDraft | null>(null);
  const [baseline, setBaseline] = useState("");
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");
  const [notice, setNotice] = useState("");
  const [noticeTone, setNoticeTone] = useState<"success" | "info">("info");
  const [connectionStatus, setConnectionStatus] = useState("UNKNOWN");
  const [connectionLoaded, setConnectionLoaded] = useState(false);
  const [longForm, setLongForm] = useState(false);
  const [runtimeError, setRuntimeError] = useState("");
  const [testResult, setTestResult] = useState("");

  useEffect(() => {
    let active = true;
    const configPromise = IS_STATIC_UI
      ? import("../../mock/fixtures").then(({ STATIC_VDA5050_CONFIGURATION }) => STATIC_VDA5050_CONFIGURATION)
      : getVda5050Configuration(robotId);
    void configPromise.then((config) => {
      if (!active) return;
      const displayConfig = IS_STATIC_UI
        ? { ...config, mqtt_host: "", mqtt_username: "", serial_number: "", client_id: "", password_configured: false }
        : config;
      const next = { ...displayConfig, mqtt_password: "" } as VdaDraft;
      setDraft(next);
      setSavedDraft(next);
      setBaseline(vdaSnapshot(next));
      setConnectionStatus(config.connection_status);
      setConnectionLoaded(true);
      setRuntimeError(IS_STATIC_UI ? "" : config.last_error ?? "");
    }).catch((caught: unknown) => {
      if (active) {
        setError(caught instanceof Error ? caught.message : "VDA5050 configuration is unavailable");
        setConnectionStatus("UNAVAILABLE");
        setConnectionLoaded(true);
      }
    });
    return () => { active = false; };
  }, [robotId]);

  useEffect(() => {
    if (IS_SHOWCASE) return;
    if (IS_STATIC_UI) {
      let active = true;
      void import("../../mock/fixtures").then(({ STATIC_VDA5050_CONFIGURATION }) => {
        if (!active) return;
        setConnectionStatus(STATIC_VDA5050_CONFIGURATION.connection_status);
        setConnectionLoaded(true);
        setRuntimeError("");
      });
      return () => { active = false; };
    }
    let active = true;
    let requestInFlight = false;
    const refreshConnection = async () => {
      if (requestInFlight) return;
      requestInFlight = true;
      try {
        const config = await getVda5050Configuration(robotId);
        if (active) {
          setConnectionStatus(config.connection_status);
          setConnectionLoaded(true);
          setRuntimeError(config.last_error ?? "");
        }
      } catch (caught: unknown) {
        if (active) {
          setConnectionStatus("UNAVAILABLE");
          setConnectionLoaded(true);
          setRuntimeError(caught instanceof Error ? caught.message : "Connection status could not be refreshed.");
        }
      } finally {
        requestInFlight = false;
      }
    };
    const timer = window.setInterval(() => { void refreshConnection(); }, 3000);
    return () => { active = false; window.clearInterval(timer); };
  }, [robotId]);

  useEffect(() => {
    const measure = () => {
      const page = pageContentRef.current;
      const form = formRef.current;
      const sections = sectionsRef.current;
      const actionBar = actionBarRef.current;
      if (!page || !form || !sections || !actionBar) {
        setLongForm(false);
        return;
      }

      const pageGap = Number.parseFloat(window.getComputedStyle(page).rowGap) || 0;
      const formGap = Number.parseFloat(window.getComputedStyle(form).rowGap) || 0;
      const outsideFormHeight = Array.from(page.children)
        .filter((child) => child !== form)
        .reduce((height, child) => height + child.getBoundingClientRect().height, 0);
      const naturalHeight = outsideFormHeight + sections.scrollHeight + actionBar.getBoundingClientRect().height
        + formGap + Math.max(0, page.children.length - 1) * pageGap;
      const shouldScroll = naturalHeight > page.clientHeight + 1;
      setLongForm((current) => current === shouldScroll ? current : shouldScroll);
    };

    measure();
    const observed = [pageContentRef.current, overviewRef.current, sectionsRef.current, actionBarRef.current]
      .filter((element): element is HTMLElement => element !== null);
    const observer = typeof ResizeObserver === "undefined" ? null : new ResizeObserver(measure);
    observed.forEach((element) => observer?.observe(element));
    window.addEventListener("resize", measure);
    return () => {
      observer?.disconnect();
      window.removeEventListener("resize", measure);
    };
  }, [draft !== null, error, runtimeError, notice, testResult, busy]);

  const fieldErrors = draft ? validateDraft(draft) : {};
  const invalidDraft = Object.keys(fieldErrors).length > 0;
  const dirty = Boolean(draft && (vdaSnapshot(draft) !== baseline || draft.mqtt_password.length > 0));
  const brokerHost = draft?.mqtt_host.trim() ?? "";
  const configuredBrokerHost = savedDraft?.mqtt_host.trim() ?? "";
  const brokerAddress = configuredBrokerHost ? configuredBrokerHost + ":" + String(savedDraft?.mqtt_port ?? "") : "Not configured";
  const statusLabel = IS_STATIC_UI ? "Not checked" : connectionLoaded ? formatConnectionStatus(connectionStatus) : "Loading status";
  const protocolLabel = savedDraft?.protocol_version ?? (draft ? "Not configured" : "Loading");
  const mqttLabel = savedDraft?.mqtt_protocol_version ?? (draft ? "Not configured" : "Loading");
  const instantActionLabel = draft?.instant_actions_status === "NOT_IMPLEMENTED" || IS_STATIC_UI
    ? "Not implemented"
    : draft?.instant_actions_supported ? "Supported" : formatRuntimeLabel(draft?.instant_actions_status ?? "Unknown");
  const canTestConnection = IS_STATIC_UI || IS_SHOWCASE || Boolean(brokerHost && !fieldErrors.mqtt_host && !fieldErrors.mqtt_port);

  const update = <K extends keyof VdaDraft,>(key: K, value: VdaDraft[K]) => {
    setDraft((previous) => previous ? { ...previous, [key]: value } : previous);
    setError("");
    setNotice("");
    setTestResult("");
  };

  const discardChanges = () => {
    if (!savedDraft || busy) return;
    setDraft({ ...savedDraft, mqtt_password: "" });
    setBaseline(vdaSnapshot(savedDraft));
    setError("");
    setNotice("");
    setTestResult("");
  };

  const save = async () => {
    if (!draft || busy || invalidDraft) return;
    if (IS_STATIC_UI) {
      const preview = { ...draft, mqtt_password: "" };
      setDraft(preview);
      setSavedDraft(preview);
      setBaseline(vdaSnapshot(preview));
      setError("");
      setNotice("Changes saved on this page. The broker was not contacted.");
      setNoticeTone("info");
      setTestResult("");
      return;
    }
    if (operationInFlightRef.current) return;
    operationInFlightRef.current = true;
    setBusy(true);
    setError("");
    setNotice("");
    setTestResult("");
    try {
      const body = { ...Object.fromEntries(editableVdaFields.map((key) => [key, draft[key]])), mqtt_password: draft.mqtt_password };
      const result = await applyVda5050Configuration(robotId, body);
      if (!result.ok) throw new Error("The backend did not confirm the configuration request.");
      const next = { ...result.configuration, mqtt_password: "" } as VdaDraft;
      setDraft(next);
      setSavedDraft(next);
      setBaseline(vdaSnapshot(next));
      setConnectionStatus(result.configuration.connection_status);
      setRuntimeError(result.configuration.last_error ?? "");
      setNotice(IS_SHOWCASE
        ? "Saved in this browser. MQTT broker was not contacted."
        : result.applied
        ? "Configuration saved and runtime application confirmed."
        : "Configuration saved; runtime application is not confirmed.");
      setNoticeTone(result.applied && !IS_SHOWCASE ? "success" : "info");
    } catch (caught) {
      setError(caught instanceof Error ? caught.message : "VDA5050 configuration was not applied");
    } finally {
      operationInFlightRef.current = false;
      setBusy(false);
    }
  };

  const test = async () => {
    if (!draft || busy || (!IS_STATIC_UI && !canTestConnection)) return;
    if (IS_STATIC_UI) {
      setError("");
      setNotice("");
      setTestResult("Broker not contacted.");
      return;
    }
    if (operationInFlightRef.current) return;
    operationInFlightRef.current = true;
    setBusy(true);
    setError("");
    setNotice("");
    setTestResult("");
    try {
      const body = { ...Object.fromEntries(editableVdaFields.map((key) => [key, draft[key]])), mqtt_password: draft.mqtt_password };
      const result = await testVda5050Connection(robotId, body);
      if (result.error_code === "STANDALONE_NO_BROKER") setTestResult("Not tested · broker was not contacted.");
      else if (result.ok) setTestResult("Test passed · " + (result.latency_ms ?? "N/A") + " ms · " + (result.broker ?? "broker"));
      else setTestResult("Test failed · " + (result.error_code ?? "MQTT_ERROR") + " · " + (result.message ?? "broker connection failed"));
    } catch (caught) {
      setTestResult("Test failed · " + (caught instanceof Error ? caught.message : "MQTT connection test failed"));
    } finally {
      operationInFlightRef.current = false;
      setBusy(false);
    }
  };

  const hostErrorId = "vda-mqtt-host-error";
  const portErrorId = "vda-mqtt-port-error";
  const protocolErrorId = "vda-protocol-version-error";
  const mqttVersionErrorId = "vda-mqtt-version-error";
  const reconnectErrorId = "vda-reconnect-interval-error";
  const timeoutErrorId = "vda-connection-timeout-error";
  const keepaliveErrorId = "vda-keepalive-error";

  return <div ref={pageContentRef} className="flex min-h-0 min-w-0 flex-1 flex-col gap-hmi-3 overflow-hidden" data-testid="vda5050-page-content" data-long-form={longForm ? "true" : "false"}>
    <section ref={overviewRef} className="grid min-w-0 shrink-0 grid-cols-1 gap-x-hmi-4 gap-y-hmi-2 border border-ptagv-border bg-white p-hmi-3 sm:grid-cols-2 xl:grid-cols-[minmax(0,1.5fr)_minmax(0,1.4fr)_minmax(0,1fr)_minmax(0,1fr)]"
      aria-label="Connection overview" data-testid="vda5050-connection-overview">
      <div className="flex min-h-10 min-w-0 items-center gap-hmi-2">
        <span className="grid size-9 shrink-0 place-items-center bg-blue-50 text-ptagv-blue" aria-hidden="true">
          <svg viewBox="0 0 20 20" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" className="size-5">
            <path d="M3 7.5c4.2-3.7 9.8-3.7 14 0M5.5 10c2.7-2.3 6.3-2.3 9 0M8.2 12.6c1.1-.9 2.5-.9 3.6 0" />
            <circle cx="10" cy="15.5" r="1" fill="currentColor" stroke="none" />
          </svg>
        </span>
        <div className="grid min-w-0 gap-1">
          <span className="text-xs font-semibold text-ptagv-muted">MQTT connection</span>
          <Status value={statusLabel} />
        </div>
      </div>
      <SummaryMetric label="Broker endpoint" value={brokerAddress} />
      <SummaryMetric label="VDA5050 protocol" value={protocolLabel} />
      <SummaryMetric label="MQTT version" value={mqttLabel} />
    </section>

    {(error || runtimeError) && <Alert tone="error" role="alert" className="shrink-0">
      <strong>{runtimeError ? "Connection error: " : "Configuration error: "}</strong>{runtimeError || error}
    </Alert>}

    {!draft ? <div className="min-h-0 flex-1 overflow-auto border border-ptagv-border bg-white px-hmi-3 py-hmi-2">
      {error ? <p className="m-0 text-sm text-ptagv-error">{error}</p> : <LoadingState>Loading VDA5050 configuration…</LoadingState>}
    </div> : <form ref={formRef} className={"flex min-w-0 flex-col gap-hmi-3 " + (longForm ? "min-h-0 flex-1" : "shrink-0")} aria-label="VDA5050 configuration" onSubmit={(event) => { event.preventDefault(); void save(); }}>
      <div ref={sectionsRef} className={"min-w-0 " + (longForm ? "min-h-0 flex-1 overflow-y-auto" : "shrink-0 overflow-visible")}>
        <div className="grid min-w-0 content-start gap-hmi-3">
          <div className="grid min-w-0 grid-cols-1 items-stretch gap-hmi-4 xl:grid-cols-2">
        <VdaSection id="vda-broker-heading" title="Broker Connection" icon="broker">
          <SwitchField label="Connection enabled" checked={draft.enabled} disabled={busy} onChange={(value) => update("enabled", value)} />
          <div className="grid min-w-0 grid-cols-1 gap-hmi-3 sm:grid-cols-[minmax(0,1fr)_9rem]">
            <Field label="MQTT host" error={fieldErrors.mqtt_host} errorId={hostErrorId}>
              <Input aria-invalid={Boolean(fieldErrors.mqtt_host)} aria-describedby={fieldErrors.mqtt_host ? hostErrorId : undefined}
                className={fieldErrors.mqtt_host ? "border-red-500 focus-visible:outline-red-600" : ""}
                value={draft.mqtt_host} maxLength={253} autoComplete="url" disabled={busy}
                placeholder={IS_STATIC_UI ? undefined : "broker.local"} onChange={(event) => update("mqtt_host", event.target.value)} />
            </Field>
            <Field label="MQTT port" error={fieldErrors.mqtt_port} errorId={portErrorId}>
              <Input aria-invalid={Boolean(fieldErrors.mqtt_port)} aria-describedby={fieldErrors.mqtt_port ? portErrorId : undefined}
                className={fieldErrors.mqtt_port ? "border-red-500 focus-visible:outline-red-600" : ""}
                type="number" min={1} max={65535} step={1} value={draft.mqtt_port} disabled={busy}
                onChange={(event) => update("mqtt_port", Number(event.target.value))} />
            </Field>
          </div>
          <div className="grid min-w-0 grid-cols-1 gap-hmi-3 sm:grid-cols-2">
            <Field label="Username">
              <Input autoComplete="username" value={draft.mqtt_username} disabled={busy}
                onChange={(event) => update("mqtt_username", event.target.value)} />
            </Field>
            <Field label="Password">
              <Input type="password" autoComplete="new-password" value={draft.mqtt_password} disabled={busy}
                placeholder={IS_STATIC_UI ? undefined : draft.password_configured ? "Stored · blank keeps current" : "Not configured"}
                onChange={(event) => update("mqtt_password", event.target.value)} />
            </Field>
          </div>
          {draft.password_configured && <p className="m-0 text-xs leading-relaxed text-ptagv-muted">A broker password is stored. Leave the password blank to keep it unchanged.</p>}
          <div className="grid min-w-0 grid-cols-1 items-end gap-hmi-3 sm:grid-cols-[minmax(0,0.75fr)_minmax(0,1.25fr)]">
            <SwitchField label="TLS" checked={draft.tls_enabled} disabled={busy} onChange={(value) => update("tls_enabled", value)} />
            <Field label="MQTT version" error={fieldErrors.mqtt_protocol_version} errorId={mqttVersionErrorId}>
              <Select aria-invalid={Boolean(fieldErrors.mqtt_protocol_version)} aria-describedby={fieldErrors.mqtt_protocol_version ? mqttVersionErrorId : undefined}
                value={draft.mqtt_protocol_version} disabled={busy} onChange={(event) => update("mqtt_protocol_version", event.target.value as VdaDraft["mqtt_protocol_version"])}>
                <option value="3.1.1">3.1.1</option><option value="5.0">5.0</option>
              </Select>
            </Field>
          </div>
        </VdaSection>

        <VdaSection id="vda-identity-heading" title="VDA5050 Protocol Identity" icon="identity">
          <div className="grid min-w-0 grid-cols-1 gap-hmi-3 sm:grid-cols-2">
            <Field label="VDA5050 version" error={fieldErrors.protocol_version} errorId={protocolErrorId}>
              <Select aria-invalid={Boolean(fieldErrors.protocol_version)} aria-describedby={fieldErrors.protocol_version ? protocolErrorId : undefined}
                value={draft.protocol_version} disabled={busy} onChange={(event) => update("protocol_version", event.target.value as VdaDraft["protocol_version"])}>
                <option value="2.0.0">2.0.0</option><option value="2.1.0">2.1.0</option><option value="3.0.0">3.0.0</option>
              </Select>
            </Field>
            <Field label="Topic prefix">
              <Input value={draft.topic_prefix} disabled={busy} onChange={(event) => update("topic_prefix", event.target.value)} />
            </Field>
          </div>
          <div className="grid min-w-0 grid-cols-1 gap-hmi-3 sm:grid-cols-2">
            <Field label="Interface name">
              <Input value={draft.interface_name} disabled={busy} onChange={(event) => update("interface_name", event.target.value)} />
            </Field>
            <Field label="Manufacturer">
              <Input value={draft.manufacturer} disabled={busy} onChange={(event) => update("manufacturer", event.target.value)} />
            </Field>
            <Field label="Serial number">
              <Input value={draft.serial_number} disabled={busy} onChange={(event) => update("serial_number", event.target.value)} />
            </Field>
            <Field label="Client ID">
              <Input value={draft.client_id} disabled={busy} placeholder={IS_STATIC_UI ? undefined : "ptagv-web-local"}
                onChange={(event) => update("client_id", event.target.value)} />
            </Field>
          </div>
        </VdaSection>
      </div>

      <VdaSection id="vda-runtime-heading" title="Runtime & Task Policy" icon="runtime">
        <div className="grid min-w-0 grid-cols-1 gap-hmi-3 lg:grid-cols-[minmax(0,1.1fr)_minmax(0,1fr)_minmax(0,1fr)]">
          <section className="grid min-w-0 content-start gap-hmi-2 border-b border-ptagv-border-soft pb-hmi-3 lg:border-b-0 lg:border-r lg:pr-hmi-3">
            <h3 className="m-0 text-xs font-bold uppercase tracking-wide text-ptagv-muted">Task permissions</h3>
            <SwitchField label="Allow Task" checked={draft.allow_task} disabled={busy} onChange={(value) => update("allow_task", value)} />
            <div className="local-vda-capability grid min-w-0 grid-cols-[minmax(0,1fr)_auto] items-center gap-x-hmi-3 gap-y-1 border border-ptagv-border-soft bg-slate-50 px-hmi-3 py-hmi-2">
              <span className="min-w-0 text-sm font-semibold text-ptagv-text">Instant Actions</span>
              <Status value={instantActionLabel} />
              <small className="col-span-full text-xs leading-relaxed text-ptagv-muted">
                Support: {draft.instant_actions_supported ? "Supported" : "Not supported"}. Execution is not implemented in this interface.
              </small>
            </div>
          </section>
          <section className="grid min-w-0 content-start gap-hmi-2 border-b border-ptagv-border-soft pb-hmi-3 lg:border-b-0 lg:border-r lg:pr-hmi-3">
            <h3 className="m-0 text-xs font-bold uppercase tracking-wide text-ptagv-muted">Reconnect behavior</h3>
            <SwitchField label="Auto reconnect" checked={draft.auto_reconnect} disabled={busy} onChange={(value) => update("auto_reconnect", value)} />
            <Field label="Reconnect interval (s)" error={fieldErrors.reconnect_interval} errorId={reconnectErrorId}>
              <Input aria-invalid={Boolean(fieldErrors.reconnect_interval)} aria-describedby={fieldErrors.reconnect_interval ? reconnectErrorId : undefined}
                className={fieldErrors.reconnect_interval ? "border-red-500 focus-visible:outline-red-600" : ""}
                type="number" min={1} max={300} step={1} value={draft.reconnect_interval} disabled={busy}
                onChange={(event) => update("reconnect_interval", Number(event.target.value))} />
            </Field>
          </section>
          <section className="grid min-w-0 content-start gap-hmi-2">
            <h3 className="m-0 text-xs font-bold uppercase tracking-wide text-ptagv-muted">Connection timing</h3>
            <div className="grid min-w-0 grid-cols-1 gap-hmi-2 sm:grid-cols-2">
              <Field label="Connection timeout (s)" error={fieldErrors.connection_timeout} errorId={timeoutErrorId}>
                <Input aria-invalid={Boolean(fieldErrors.connection_timeout)} aria-describedby={fieldErrors.connection_timeout ? timeoutErrorId : undefined}
                  className={fieldErrors.connection_timeout ? "border-red-500 focus-visible:outline-red-600" : ""}
                  type="number" min={1} max={60} step={1} value={draft.connection_timeout} disabled={busy}
                  onChange={(event) => update("connection_timeout", Number(event.target.value))} />
              </Field>
              <Field label="Keepalive (s)" error={fieldErrors.keepalive} errorId={keepaliveErrorId}>
                <Input aria-invalid={Boolean(fieldErrors.keepalive)} aria-describedby={fieldErrors.keepalive ? keepaliveErrorId : undefined}
                  className={fieldErrors.keepalive ? "border-red-500 focus-visible:outline-red-600" : ""}
                  type="number" min={5} max={3600} step={1} value={draft.keepalive} disabled={busy}
                  onChange={(event) => update("keepalive", Number(event.target.value))} />
              </Field>
            </div>
          </section>
        </div>
        <p className="m-0 text-xs leading-relaxed text-ptagv-muted">
          {IS_STATIC_UI
            ? "These preferences stay local to this page and do not send robot commands."
            : IS_SHOWCASE ? "Browser configuration only. MQTT and robot connections remain disconnected."
            : draft.allow_task
              ? "Incoming VDA5050 orders are permitted by this task policy."
              : "New VDA5050 orders are blocked for this robot."}
        </p>
          </VdaSection>
        </div>
      </div>

      <div ref={actionBarRef} className="flex min-w-0 shrink-0 flex-col gap-hmi-3 border-t border-ptagv-border bg-white px-hmi-3 py-hmi-2 sm:flex-row sm:items-center sm:justify-between"
        data-testid="vda5050-action-bar">
        <div className="grid min-w-0 gap-1" aria-live="polite">
          {notice && <Alert tone={noticeTone} role="status" className="py-1 text-xs">{notice}</Alert>}
          {testResult && <Alert tone={testResult.startsWith("Test failed") ? "error" : testResult.startsWith("Test passed") ? "success" : "neutral"}
            role={testResult.startsWith("Test failed") ? "alert" : "status"} className="py-1 text-xs">{testResult}</Alert>}
          {!notice && !testResult && dirty && <span className="text-xs font-semibold text-ptagv-warning" role="status">Unsaved changes</span>}
          {!notice && !testResult && !dirty && <span className="text-xs text-ptagv-muted">Configuration is up to date.</span>}
        </div>
        <div className="flex shrink-0 flex-wrap items-center justify-end gap-hmi-2" role="group" aria-label="Configuration actions">
          <Button type="button" disabled={busy || (!IS_STATIC_UI && !canTestConnection)} onClick={() => void test()}>TEST CONNECTION</Button>
          <Button type="button" disabled={busy || !dirty} onClick={discardChanges}>DISCARD CHANGES</Button>
          <Button type="submit" tone="primary" disabled={busy || !dirty || invalidDraft}
            className="min-h-11 min-w-36 px-hmi-5 text-sm">{busy ? "APPLYING…" : "SAVE & APPLY"}</Button>
        </div>
      </div>
    </form>}
  </div>;
}
