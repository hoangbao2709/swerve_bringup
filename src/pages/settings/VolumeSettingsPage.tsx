import { useEffect, useRef, useState } from "react";
import { Alert, Button } from "../../components/shared/ui/HmiPrimitives";
import { getHostVolumeSettings, testHostVolume, updateHostVolumeSettings } from "../../services/api";
import {
  defaultLocalAppSettings,
  getLocalAppSettings,
  LOCAL_APP_SETTINGS_RESET_EVENT,
  isSettingsPreviewEnabled,
  playBrowserTestTone,
  saveLocalAppSettings,
  type VolumeSettings,
} from "../../services/settingsPreview";
import { SettingsCard, SettingsPageFrame, SettingsSwitch } from "./SettingsPrimitives";

export function VolumeSettingsPage({ embedded = false, active = true }: { embedded?: boolean; active?: boolean } = {}) {
  const canPreview = isSettingsPreviewEnabled();
  const [saved, setSaved] = useState(() => canPreview ? getLocalAppSettings().volume : { level: 0, muted: false });
  const [draft, setDraft] = useState<VolumeSettings>(() => ({ ...saved }));
  const [hostAvailable, setHostAvailable] = useState(canPreview);
  const [notice, setNotice] = useState("");
  const [error, setError] = useState("");
  const [testing, setTesting] = useState(false);
  const [saving, setSaving] = useState(false);
  const loadedHostSettings = useRef(false);

  useEffect(() => {
    if (!active || canPreview || loadedHostSettings.current) return;
    loadedHostSettings.current = true;
    let alive = true;
    void getHostVolumeSettings().then((result) => {
      if (!alive) return;
      const next = { level: result.level, muted: result.muted };
      setSaved(next);
      setDraft({ ...next });
      setHostAvailable(true);
      setError("");
    }).catch((caught: unknown) => {
      if (!alive) return;
      setHostAvailable(false);
      setError(caught instanceof Error ? caught.message : "Host audio settings are unavailable.");
    });
    return () => { alive = false; };
  }, [active, canPreview]);

  useEffect(() => {
    if (!canPreview) return;
    const reset = () => {
      const next = getLocalAppSettings().volume;
      setSaved(next);
      setDraft({ ...next });
      setError("");
      setNotice("");
    };
    window.addEventListener(LOCAL_APP_SETTINGS_RESET_EVENT, reset);
    return () => window.removeEventListener(LOCAL_APP_SETTINGS_RESET_EVENT, reset);
  }, [canPreview]);

  const save = async () => {
    if ((!canPreview && !hostAvailable) || saving) return;
    setSaving(true);
    try {
      const next = canPreview
        ? saveLocalAppSettings({ ...getLocalAppSettings(), volume: draft }).volume
        : await updateHostVolumeSettings(draft);
      setSaved(next);
      setDraft({ ...next });
      setNotice(canPreview ? "Saved in browser." : "Host volume updated.");
      setError("");
    } catch (caught) {
      setError(caught instanceof Error ? caught.message : "Volume preference could not be saved.");
      setNotice("");
    } finally {
      setSaving(false);
    }
  };

  const reset = () => {
    if (!canPreview) return;
    const defaults = defaultLocalAppSettings().volume;
    try {
      const next = saveLocalAppSettings({ ...getLocalAppSettings(), volume: defaults }).volume;
      setSaved(next);
      setDraft({ ...next });
      setNotice("Volume reset.");
      setError("");
    } catch (caught) {
      setError(caught instanceof Error ? caught.message : "Volume preference could not be reset.");
      setNotice("");
    }
  };

  const play = async () => {
    if (testing) return;
    if (!canPreview && !hostAvailable) return;
    setTesting(true);
    setNotice("");
    setError("");
    try {
      if (canPreview && (draft.muted || draft.level === 0)) {
        setNotice("Muted or set to 0%; no tone was played.");
      } else if (canPreview && await playBrowserTestTone(draft.level)) {
        setNotice("A short test tone played through this browser.");
      } else if (!canPreview) {
        const result = await testHostVolume();
        setNotice(result.message);
      } else {
        setError("This browser does not support the test tone audio interface.");
      }
    } catch {
      setError("The browser could not start audio. Use the test button after checking browser audio permissions.");
    } finally {
      setTesting(false);
    }
  };

  const dirty = draft.level !== saved.level || draft.muted !== saved.muted;

  return <SettingsPageFrame title="Volume" embedded={embedded} active={active}
    sectionId={embedded ? "settings-volume" : undefined}>
    {error && <Alert tone="error" role="alert">{error}</Alert>}
    {notice && <Alert tone="success" role="status">{notice}</Alert>}

    <SettingsCard title="AUDIO LEVEL">
      <div className="grid min-w-0 gap-hmi-4 xl:grid-cols-[minmax(0,1fr)_14rem] xl:items-center">
        <label className="grid min-w-0 gap-2 text-sm font-bold text-ptagv-text">
          <span className="flex items-center justify-between gap-3"><span>Test tone level</span><output aria-live="polite">{draft.level}%</output></span>
          <input type="range" min={0} max={100} step={1} value={draft.level} disabled={!canPreview && !hostAvailable}
            aria-label="Test tone percentage"
            className="h-8 w-full accent-blue-700 focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-ptagv-blue disabled:opacity-60"
            onChange={(event) => { setDraft((current) => ({ ...current, level: Number(event.target.value) })); setNotice(""); }} />
          <span className="flex justify-between text-xs font-normal text-ptagv-muted"><span>0%</span><span>100%</span></span>
        </label>
        <div className="flex flex-wrap items-center justify-between gap-hmi-2 border-t border-ptagv-border-soft pt-hmi-2 xl:border-l xl:border-t-0 xl:pl-hmi-4 xl:pt-0">
          <span className="text-sm font-bold text-ptagv-text">Mute browser tone</span>
          <SettingsSwitch checked={draft.muted} onChange={(muted) => { setDraft((current) => ({ ...current, muted })); setNotice(""); }}
            label={canPreview ? "Mute browser tone" : "Mute host audio"} disabled={!canPreview && !hostAvailable} />
        </div>
      </div>
      <div className="flex flex-wrap gap-2 border-t border-ptagv-border-soft pt-hmi-2">
        <Button tone="primary" disabled={(!canPreview && !hostAvailable) || !dirty || saving} onClick={() => void save()}>{saving ? "APPLYING…" : "SAVE PREFERENCE"}</Button>
        <Button disabled={(!canPreview && !hostAvailable) || testing} onClick={() => void play()}>{testing ? "PLAYING…" : "TEST SOUND"}</Button>
        <Button disabled={!canPreview} onClick={reset}>RESET TO DEFAULT</Button>
      </div>
    </SettingsCard>
  </SettingsPageFrame>;
}
