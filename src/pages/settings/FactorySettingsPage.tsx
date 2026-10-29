import { useState } from "react";
import { useFocusTrap } from "../../components/ui/useFocusTrap";
import { Alert, Button } from "../../components/shared/ui/HmiPrimitives";
import { isSettingsPreviewEnabled, resetLocalAppSettings, resetUiPreferences } from "../../services/settingsPreview";
import { useStore } from "../../state/store";
import { SettingsCard, SettingsPageFrame } from "./SettingsPrimitives";

type ResetScope = "ui" | "settings";

export function FactorySettingsPage({ embedded = false, active = true }: { embedded?: boolean; active?: boolean } = {}) {
  const canResetAppSettings = isSettingsPreviewEnabled();
  const resetLabelLayers = useStore((state) => state.resetLabelLayers);
  const [scope, setScope] = useState<ResetScope | null>(null);
  const [phrase, setPhrase] = useState("");
  const [error, setError] = useState("");
  const [notice, setNotice] = useState("");
  const dialogRef = useFocusTrap<HTMLDivElement>(scope !== null && active);

  const open = (target: ResetScope) => {
    setScope(target);
    setPhrase("");
    setError("");
    setNotice("");
  };

  const confirm = () => {
    if (phrase !== "RESET" || !scope) return;
    try {
      if (scope === "ui") {
        resetUiPreferences();
        resetLabelLayers();
        setNotice("Display preferences reset. Robot settings and maps unchanged.");
      } else {
        resetLocalAppSettings();
        setNotice("Browser settings reset. Robot settings and maps unchanged.");
      }
      setScope(null);
      setPhrase("");
      setError("");
    } catch (caught) {
      setError(caught instanceof Error ? caught.message : "Reset could not be completed.");
    }
  };

  const dialogTitle = scope === "ui" ? "Reset display preferences" : "Reset saved settings";
  const dialogScope = scope === "ui"
    ? "Resets this browser's map label preferences. Robot settings, maps, runtime, and safety state are unchanged."
    : "Resets browser Wi-Fi, volume, and NTP preferences. Robot settings, host services, and maps are unchanged.";

  return <SettingsPageFrame title="Factory Setting" embedded={embedded} active={active}
    sectionId={embedded ? "settings-factory" : undefined}>
    {error && !scope && <Alert tone="error" role="alert">{error}</Alert>}
    {notice && <Alert tone="success" role="status">{notice}</Alert>}

    <div className="grid min-w-0 grid-cols-1 gap-hmi-3 xl:grid-cols-3">
      <SettingsCard title="DISPLAY PREFERENCES">
        <p className="m-0 text-sm leading-relaxed text-ptagv-muted">Reset map label preferences.</p>
        <Button tone="secondary" onClick={() => open("ui")}>RESET DISPLAY PREFS</Button>
      </SettingsCard>

      <SettingsCard title="SAVED SETTINGS">
        <p className="m-0 text-sm leading-relaxed text-ptagv-muted">Reset saved Wi-Fi, volume, and NTP preferences.</p>
        {!canResetAppSettings && <p className="m-0 text-xs text-ptagv-warning">Local preference storage is unavailable.</p>}
        <Button tone="secondary" disabled={!canResetAppSettings} onClick={() => open("settings")}>RESET SAVED SETTINGS</Button>
      </SettingsCard>

      <SettingsCard title="ROBOT / SYSTEM RESET">
        <p className="m-0 text-xs leading-relaxed text-ptagv-warning">Could alter robot settings or erase data.</p>
        <p className="m-0 text-xs leading-relaxed text-ptagv-muted">No authorized reset service. No command will be sent.</p>
        <Button tone="danger" disabled title="No supported factory-reset service is configured">FACTORY RESET ROBOT / SYSTEM</Button>
      </SettingsCard>
    </div>

    {scope && <div className="fixed inset-0 z-50 grid place-items-center bg-slate-950/45 p-hmi-3" role="presentation">
      <div ref={dialogRef} role="dialog" aria-modal="true" aria-labelledby="factory-reset-title" tabIndex={-1}
        className="grid w-full max-w-lg gap-hmi-3 border border-ptagv-border bg-white p-hmi-4 shadow-xl">
        <h3 id="factory-reset-title" className="m-0 text-base font-bold text-ptagv-text">{dialogTitle}</h3>
        <p className="m-0 text-sm leading-relaxed text-ptagv-muted">{dialogScope}</p>
        <label className="grid gap-1 text-xs font-bold text-ptagv-muted">TYPE RESET TO CONFIRM
          <input autoComplete="off" value={phrase} onChange={(event) => setPhrase(event.target.value)}
            className="min-h-10 w-full border border-ptagv-border px-hmi-2 text-sm text-ptagv-text focus-visible:outline-2 focus-visible:outline-ptagv-blue" />
        </label>
        {error && <Alert tone="error" role="alert">{error}</Alert>}
        <div className="flex flex-wrap justify-end gap-2">
          <Button onClick={() => { setScope(null); setPhrase(""); setError(""); }}>CANCEL</Button>
          <Button tone="danger" disabled={phrase !== "RESET" || (scope === "settings" && !canResetAppSettings)} onClick={confirm}>CONFIRM RESET</Button>
        </div>
      </div>
    </div>}
  </SettingsPageFrame>;
}
