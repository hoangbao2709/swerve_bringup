import { useEffect, useRef, useState } from "react";
import { Alert, Button, Input, Select } from "../../components/shared/ui/HmiPrimitives";
import { IS_SHOWCASE } from "../../config";
import { getHostNtpSettings, requestHostNtpSync, updateHostNtpSettings, type HostNtpStatus } from "../../services/api";
import {
  getLocalAppSettings,
  LOCAL_APP_SETTINGS_RESET_EVENT,
  isSettingsPreviewEnabled,
  isValidNtpServer,
  isValidTimezone,
  saveLocalAppSettings,
  type NtpSettings,
} from "../../services/settingsPreview";
import { SettingsCard, SettingsPageFrame, SettingsSwitch } from "./SettingsPrimitives";

const TIMEZONES = [
  "Asia/Ho_Chi_Minh", "Asia/Singapore", "Asia/Tokyo", "Australia/Sydney",
  "UTC", "Europe/London", "Europe/Berlin", "America/New_York", "America/Los_Angeles",
];

function formatTime(date: Date, timezone: string): string {
  try {
    return new Intl.DateTimeFormat("en-GB", { dateStyle: "full", timeStyle: "medium", timeZone: timezone }).format(date);
  } catch {
    return date.toLocaleString();
  }
}

function validate(settings: NtpSettings, serverConfigurable = true): string {
  if (serverConfigurable && !isValidNtpServer(settings.primaryServer)) return "Enter a valid primary NTP hostname or IPv4 address.";
  if (serverConfigurable && settings.secondaryServer.trim() && !isValidNtpServer(settings.secondaryServer)) return "Enter a valid secondary NTP hostname or IPv4 address, or leave it blank.";
  if (!isValidTimezone(settings.timezone)) return "Choose a supported timezone.";
  return "";
}

export function NtpSettingsPage({ embedded = false, active = true }: { embedded?: boolean; active?: boolean } = {}) {
  const canPreview = isSettingsPreviewEnabled();
  const [saved, setSaved] = useState<NtpSettings>(() => canPreview ? getLocalAppSettings().ntp : {
    enabled: false, primaryServer: "", secondaryServer: "", timezone: "UTC", lastCheckAt: null,
  });
  const [draft, setDraft] = useState<NtpSettings>(() => ({ ...saved }));
  const [clockNow, setClockNow] = useState(() => new Date());
  const [hostStatus, setHostStatus] = useState<HostNtpStatus | null>(null);
  const [hostAvailable, setHostAvailable] = useState(canPreview);
  const [serverConfigurable, setServerConfigurable] = useState(canPreview);
  const loadedHostStatus = useRef(false);
  const [error, setError] = useState("");
  const [notice, setNotice] = useState("");
  const dirty = draft.enabled !== saved.enabled || draft.primaryServer !== saved.primaryServer
    || draft.secondaryServer !== saved.secondaryServer || draft.timezone !== saved.timezone;

  useEffect(() => {
    if (!active) return;
    if (!canPreview) {
      if (loadedHostStatus.current) return;
      loadedHostStatus.current = true;
      let alive = true;
      void getHostNtpSettings().then((result) => {
        if (!alive) return;
        const next: NtpSettings = {
          enabled: result.enabled, primaryServer: result.primary_server,
          secondaryServer: result.secondary_server, timezone: result.timezone, lastCheckAt: null,
        };
        setSaved(next);
        setDraft({ ...next });
        setHostStatus(result);
        setHostAvailable(true);
        setServerConfigurable(result.server_configurable);
        setClockNow(new Date(result.system_time));
        setError("");
      }).catch((caught: unknown) => {
        if (!alive) return;
        setHostAvailable(false);
        setError(caught instanceof Error ? caught.message : "Host time settings are unavailable.");
      });
      return () => { alive = false; };
    }
    const timer = window.setInterval(() => setClockNow(new Date()), 1000);
    return () => window.clearInterval(timer);
  }, [active, canPreview]);

  useEffect(() => {
    if (!canPreview) return;
    const reset = () => {
      const next = getLocalAppSettings().ntp;
      setSaved(next);
      setDraft({ ...next });
      setClockNow(new Date());
      setError("");
      setNotice("");
    };
    window.addEventListener(LOCAL_APP_SETTINGS_RESET_EVENT, reset);
    return () => window.removeEventListener(LOCAL_APP_SETTINGS_RESET_EVENT, reset);
  }, [canPreview]);

  const save = async () => {
    if (!canPreview && !hostAvailable) return;
    const validation = validate(draft, canPreview || serverConfigurable);
    if (validation) { setError(validation); setNotice(""); return; }
    try {
      let next: NtpSettings;
      if (canPreview) {
        next = saveLocalAppSettings({
          ...getLocalAppSettings(), ntp: { ...draft, lastCheckAt: saved.lastCheckAt },
        }).ntp;
      } else {
        const result = await updateHostNtpSettings({
          enabled: draft.enabled, timezone: draft.timezone,
          primary_server: draft.primaryServer, secondary_server: draft.secondaryServer,
        });
        next = {
          enabled: result.enabled, primaryServer: result.primary_server,
          secondaryServer: result.secondary_server, timezone: result.timezone, lastCheckAt: null,
        };
        setHostStatus(result);
        setServerConfigurable(result.server_configurable);
        setClockNow(new Date(result.system_time));
      }
      setSaved(next);
      setDraft({ ...next });
      setError("");
      setNotice(canPreview ? "Saved in browser; robot clock unchanged." : "Host time settings updated.");
    } catch (caught) {
      setError(caught instanceof Error ? caught.message : "NTP preferences could not be saved.");
      setNotice("");
    }
  };

  const cancel = () => {
    setDraft({ ...saved });
    setError("");
    setNotice("Unsaved changes were discarded.");
  };

  const checkSystemTime = async () => {
    if (canPreview) {
      const validation = validate(draft);
      if (validation) { setError(validation); setNotice(""); return; }
      const checkedAt = new Date().toISOString();
      const next = saveLocalAppSettings({
        ...getLocalAppSettings(), ntp: { ...draft, lastCheckAt: checkedAt },
      }).ntp;
      setSaved(next);
      setDraft({ ...next });
      setError("");
      setNotice(IS_SHOWCASE ? "Browser clock checked. Host and robot time are unchanged." : "Preview settings checked; host and robot time are unchanged.");
      return;
    }
    if (!hostAvailable) return;
    setError("");
    setNotice("");
    try {
      const result = await requestHostNtpSync();
      setHostStatus(result);
      setClockNow(new Date(result.system_time));
      setServerConfigurable(result.server_configurable);
      const next: NtpSettings = {
        enabled: result.enabled, primaryServer: result.primary_server,
        secondaryServer: result.secondary_server, timezone: result.timezone, lastCheckAt: null,
      };
      setSaved(next);
      if (!dirty) {
        setDraft({ ...next });
      }
      setNotice(result.synchronized
        ? "Host reports time synchronized."
        : result.request_accepted ? "Synchronization requested; waiting for host confirmation." : "Host synchronization is not confirmed.");
    } catch (caught) {
      setError(caught instanceof Error ? caught.message : "Host synchronization request failed.");
    }
  };
  const validationMessage = validate(draft, canPreview || serverConfigurable);

  return <SettingsPageFrame title="NTP" embedded={embedded} active={active}
    sectionId={embedded ? "settings-ntp" : undefined}>
    {error && <Alert tone="error" role="alert">{error}</Alert>}
    {notice && <Alert tone="success" role="status">{notice}</Alert>}

    <div className="grid min-w-0 grid-cols-1 gap-hmi-3 xl:grid-cols-2 xl:items-start">
      <SettingsCard title="SYNCHRONIZATION">
        <div className="flex flex-wrap items-center justify-between gap-hmi-3 border-b border-ptagv-border-soft pb-hmi-2">
          <div className="grid gap-1"><b className="text-sm text-ptagv-text">Auto-sync preference</b></div>
          <SettingsSwitch checked={draft.enabled} label="Automatic NTP synchronization" disabled={!canPreview && !hostAvailable}
            onChange={(enabled) => { setDraft((current) => ({ ...current, enabled })); setNotice(""); }} />
        </div>
        {!canPreview && hostAvailable && !serverConfigurable && <p className="m-0 text-xs text-ptagv-warning">Custom NTP servers are unavailable for {hostStatus?.provider ?? "the active host time service"}.</p>}
        <label className="grid gap-1 text-xs font-bold text-ptagv-muted">PRIMARY NTP SERVER
          <Input autoComplete="off" value={draft.primaryServer} disabled={!canPreview ? !hostAvailable || !serverConfigurable : false}
            aria-invalid={Boolean(error && !isValidNtpServer(draft.primaryServer))}
            onChange={(event) => { setDraft((current) => ({ ...current, primaryServer: event.target.value })); setNotice(""); }} />
        </label>
        <label className="grid gap-1 text-xs font-bold text-ptagv-muted">SECONDARY NTP SERVER
          <Input autoComplete="off" value={draft.secondaryServer} disabled={!canPreview ? !hostAvailable || !serverConfigurable : false}
            aria-invalid={Boolean(error && draft.secondaryServer.trim() && !isValidNtpServer(draft.secondaryServer))}
            onChange={(event) => { setDraft((current) => ({ ...current, secondaryServer: event.target.value })); setNotice(""); }} />
        </label>
        <label className="grid gap-1 text-xs font-bold text-ptagv-muted">TIMEZONE
          <Select value={draft.timezone} disabled={!canPreview && !hostAvailable} aria-invalid={Boolean(error && !isValidTimezone(draft.timezone))}
            onChange={(event) => { setDraft((current) => ({ ...current, timezone: event.target.value })); setNotice(""); }}>
            {[...new Set([...TIMEZONES, ...(draft.timezone ? [draft.timezone] : [])])].map((timezone) => <option key={timezone} value={timezone}>{timezone}</option>)}
          </Select>
        </label>
        {validationMessage && dirty && <p className="m-0 text-xs text-ptagv-error" role="alert">{validationMessage}</p>}
        <div className="flex flex-wrap gap-2 border-t border-ptagv-border-soft pt-hmi-2">
          <Button tone="primary" disabled={(!canPreview && !hostAvailable) || !dirty || Boolean(validationMessage)} onClick={() => void save()}>SAVE SETTINGS</Button>
          <Button disabled={!canPreview && !hostAvailable} onClick={() => void checkSystemTime()}>{canPreview ? IS_SHOWCASE ? "CHECK BROWSER CLOCK" : "CHECK PREVIEW" : "REQUEST SYNC"}</Button>
          <Button disabled={!dirty} onClick={cancel}>CANCEL CHANGES</Button>
        </div>
      </SettingsCard>

      <SettingsCard title="TIME SOURCE">
        <div className="grid min-w-0 gap-2 border-b border-ptagv-border-soft pb-hmi-2">
          <span className="text-xs font-bold tracking-wide text-ptagv-muted">{canPreview ? "BROWSER TIME" : "BACKEND HOST TIME"}</span>
          <b className="text-base leading-relaxed text-ptagv-text" aria-live="polite">{canPreview ? formatTime(clockNow, draft.timezone)
            : hostStatus ? formatTime(clockNow, hostStatus.timezone) : "NOT REPORTED"}</b>
        </div>
        <div className="grid gap-2 pt-hmi-1">
          <div className="flex items-center justify-between gap-2"><span className="text-xs text-ptagv-muted">SETTINGS</span><b className="text-xs text-ptagv-text">{dirty ? "UNSAVED" : "SAVED"}</b></div>
          <div className="flex items-center justify-between gap-2"><span className="text-xs text-ptagv-muted">{canPreview ? IS_SHOWCASE ? "LAST BROWSER CHECK" : "LAST PREVIEW CHECK" : "NTP SYNC"}</span><b className="text-right text-xs text-ptagv-text">{canPreview
            ? saved.lastCheckAt ? formatTime(new Date(saved.lastCheckAt), "UTC") + " UTC" : "NOT RECORDED"
            : hostStatus ? hostStatus.synchronized ? "SYNCHRONIZED" : hostStatus.enabled ? "PENDING / UNKNOWN" : "DISABLED" : "UNAVAILABLE"}</b></div>
          <div className="flex items-center justify-between gap-2"><span className="text-xs text-ptagv-muted">ROBOT CLOCK</span><b className="text-xs text-ptagv-text">NOT REPORTED</b></div>
        </div>
      </SettingsCard>
    </div>
  </SettingsPageFrame>;
}
