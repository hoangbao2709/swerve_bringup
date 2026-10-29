import { IS_SHOWCASE, IS_STATIC_UI } from "../config";

export const PREVIEW_SETTINGS_STORAGE_KEY = "ptagv.web-local.settings.v1";
export const UI_PREFERENCES_STORAGE_KEY = "ptagv.web-local.ui-preferences.v1";
export const LOCAL_APP_SETTINGS_RESET_EVENT = "ptagv:local-app-settings-reset";

export type WifiSecurity = "WPA2-Personal" | "WPA3-Personal" | "Open";
export type WifiNetwork = {
  ssid: string;
  security: WifiSecurity;
  signalPercent: number;
};

export type WifiSettings = {
  enabled: boolean;
  connectedSsid: string | null;
  savedSsids: string[];
};

export type VolumeSettings = {
  level: number;
  muted: boolean;
};

export type NtpSettings = {
  enabled: boolean;
  primaryServer: string;
  secondaryServer: string;
  timezone: string;
  lastCheckAt: string | null;
};

export type LocalAppSettings = {
  wifi: WifiSettings;
  volume: VolumeSettings;
  ntp: NtpSettings;
};

const DEFAULT_SETTINGS: LocalAppSettings = {
  wifi: { enabled: true, connectedSsid: null, savedSsids: [] },
  volume: { level: 65, muted: false },
  ntp: {
    enabled: true,
    primaryServer: "pool.ntp.org",
    secondaryServer: "time.google.com",
    timezone: "Asia/Ho_Chi_Minh",
    lastCheckAt: null,
  },
};

const WIFI_FIXTURES: WifiNetwork[] = [
  { ssid: "PTAGV-Maintenance", security: "WPA2-Personal", signalPercent: 91 },
  { ssid: "Warehouse-Operations", security: "WPA2-Personal", signalPercent: 76 },
  { ssid: "Guest-Network", security: "Open", signalPercent: 48 },
];

export function isSettingsPreviewEnabled(): boolean {
  return IS_SHOWCASE || (import.meta.env.DEV && IS_STATIC_UI);
}

export function isShowcaseSettings(): boolean { return IS_SHOWCASE; }

function assertSettingsPreview(): void {
  if (!isSettingsPreviewEnabled()) {
    throw new Error("Local settings actions are disabled outside the frontend preview.");
  }
}

function clampPercent(value: unknown, fallback: number): number {
  const number = Number(value);
  return Number.isFinite(number) ? Math.min(100, Math.max(0, Math.round(number))) : fallback;
}

function cleanSavedSsids(value: unknown): string[] {
  if (!Array.isArray(value)) return [];
  return [...new Set(value.filter((item): item is string => typeof item === "string")
    .map((ssid) => ssid.trim()).filter((ssid) => ssid.length > 0 && ssid.length <= 32))];
}

function normalizeSettings(value: unknown): LocalAppSettings {
  const raw = value && typeof value === "object" ? value as Partial<LocalAppSettings> : {};
  const wifi = raw.wifi && typeof raw.wifi === "object" ? raw.wifi : DEFAULT_SETTINGS.wifi;
  const volume = raw.volume && typeof raw.volume === "object" ? raw.volume : DEFAULT_SETTINGS.volume;
  const ntp = raw.ntp && typeof raw.ntp === "object" ? raw.ntp : DEFAULT_SETTINGS.ntp;
  const savedSsids = cleanSavedSsids(wifi.savedSsids);
  const connectedSsid = typeof wifi.connectedSsid === "string" && wifi.connectedSsid.length <= 32
    ? wifi.connectedSsid
    : null;
  return {
    wifi: {
      enabled: wifi.enabled !== false,
      connectedSsid: wifi.enabled === false ? null : connectedSsid,
      savedSsids,
    },
    volume: {
      level: clampPercent(volume.level, DEFAULT_SETTINGS.volume.level),
      muted: volume.muted === true,
    },
    ntp: {
      enabled: ntp.enabled !== false,
      primaryServer: typeof ntp.primaryServer === "string" ? ntp.primaryServer : DEFAULT_SETTINGS.ntp.primaryServer,
      secondaryServer: typeof ntp.secondaryServer === "string" ? ntp.secondaryServer : DEFAULT_SETTINGS.ntp.secondaryServer,
      timezone: typeof ntp.timezone === "string" && isValidTimezone(ntp.timezone)
        ? ntp.timezone
        : DEFAULT_SETTINGS.ntp.timezone,
      lastCheckAt: typeof ntp.lastCheckAt === "string" ? ntp.lastCheckAt : null,
    },
  };
}

function readStoredSettings(): LocalAppSettings {
  if (typeof window === "undefined") return defaultLocalAppSettings();
  try {
    return normalizeSettings(JSON.parse(window.localStorage.getItem(PREVIEW_SETTINGS_STORAGE_KEY) ?? "null"));
  } catch {
    return defaultLocalAppSettings();
  }
}

export function defaultLocalAppSettings(): LocalAppSettings {
  return {
    wifi: { ...DEFAULT_SETTINGS.wifi, savedSsids: [] },
    volume: { ...DEFAULT_SETTINGS.volume },
    ntp: { ...DEFAULT_SETTINGS.ntp },
  };
}

/** Stored settings are read only in standalone browser profiles. */
export function getLocalAppSettings(): LocalAppSettings {
  return isSettingsPreviewEnabled() ? readStoredSettings() : defaultLocalAppSettings();
}

export function saveLocalAppSettings(value: LocalAppSettings): LocalAppSettings {
  assertSettingsPreview();
  const normalized = normalizeSettings(value);
  try {
    window.localStorage.setItem(PREVIEW_SETTINGS_STORAGE_KEY, JSON.stringify(normalized));
  } catch {
    throw new Error("Browser storage is unavailable; settings were not saved.");
  }
  return normalized;
}

export function resetLocalAppSettings(): LocalAppSettings {
  assertSettingsPreview();
  const defaults = defaultLocalAppSettings();
  try {
    window.localStorage.removeItem(PREVIEW_SETTINGS_STORAGE_KEY);
  } catch {
    throw new Error("Browser storage is unavailable; settings could not be reset.");
  }
  window.dispatchEvent(new Event(LOCAL_APP_SETTINGS_RESET_EVENT));
  return defaults;
}

export async function scanWifiNetworks(): Promise<WifiNetwork[]> {
  assertSettingsPreview();
  await new Promise<void>((resolve) => window.setTimeout(resolve, 120));
  return WIFI_FIXTURES.map((network) => ({ ...network }));
}

export function connectWifiPreview(ssid: string): LocalAppSettings {
  assertSettingsPreview();
  const cleanSsid = ssid.trim();
  if (!cleanSsid || cleanSsid.length > 32) throw new Error("Enter an SSID between 1 and 32 characters.");
  const current = getLocalAppSettings();
  const next: LocalAppSettings = {
    ...current,
    wifi: {
      ...current.wifi,
      enabled: true,
      connectedSsid: cleanSsid,
      savedSsids: cleanSavedSsids([...current.wifi.savedSsids, cleanSsid]),
    },
  };
  return saveLocalAppSettings(next);
}

export function disconnectWifiPreview(): LocalAppSettings {
  assertSettingsPreview();
  const current = getLocalAppSettings();
  return saveLocalAppSettings({ ...current, wifi: { ...current.wifi, connectedSsid: null } });
}

export function setWifiEnabledPreview(enabled: boolean): LocalAppSettings {
  assertSettingsPreview();
  const current = getLocalAppSettings();
  return saveLocalAppSettings({
    ...current,
    wifi: { ...current.wifi, enabled, connectedSsid: enabled ? current.wifi.connectedSsid : null },
  });
}

export function forgetWifiNetworkPreview(ssid: string): LocalAppSettings {
  assertSettingsPreview();
  const current = getLocalAppSettings();
  return saveLocalAppSettings({
    ...current,
    wifi: {
      ...current.wifi,
      connectedSsid: current.wifi.connectedSsid === ssid ? null : current.wifi.connectedSsid,
      savedSsids: current.wifi.savedSsids.filter((saved) => saved !== ssid),
    },
  });
}

export function isValidNtpServer(value: string): boolean {
  const host = value.trim();
  if (!host || host.length > 253 || /[\s/:?#@]/.test(host)) return false;
  const ipv4Parts = host.split(".");
  if (ipv4Parts.length === 4 && ipv4Parts.every((part) => /^\d{1,3}$/.test(part))) {
    return ipv4Parts.every((part) => Number(part) >= 0 && Number(part) <= 255);
  }
  const labels = host.replace(/\.$/, "").split(".");
  return labels.length >= 2 && labels.every((label) => label.length > 0 && label.length <= 63
    && /^[A-Za-z0-9](?:[A-Za-z0-9-]*[A-Za-z0-9])?$/.test(label));
}

export function isValidTimezone(value: string): boolean {
  try {
    new Intl.DateTimeFormat("en", { timeZone: value });
    return true;
  } catch {
    return false;
  }
}

export function resetUiPreferences(): void {
  if (typeof window === "undefined") return;
  try {
    window.localStorage.removeItem(UI_PREFERENCES_STORAGE_KEY);
  } catch {
    throw new Error("Browser storage is unavailable; UI preferences could not be reset.");
  }
}

/** Plays only in this browser and only when called directly from a user gesture. */
export async function playBrowserTestTone(level: number): Promise<boolean> {
  if (typeof window === "undefined" || typeof window.AudioContext === "undefined") return false;
  const context = new window.AudioContext();
  const oscillator = context.createOscillator();
  const gain = context.createGain();
  gain.gain.value = Math.min(100, Math.max(0, level)) / 100 * 0.12;
  oscillator.frequency.value = 660;
  oscillator.type = "sine";
  oscillator.connect(gain);
  gain.connect(context.destination);
  oscillator.onended = () => { void context.close(); };
  oscillator.start();
  oscillator.stop(context.currentTime + 0.35);
  return true;
}
