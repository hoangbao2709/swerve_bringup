import { useEffect, useState } from "react";
import { Alert, Button, Input, Select, StatusIndicator } from "../../components/shared/ui/HmiPrimitives";
import { connectHostWifi, disconnectHostWifi, forgetHostWifi, getHostWifiSettings, scanHostWifiNetworks, setHostWifiRadio, type HostWifiNetwork } from "../../services/api";
import {
  connectWifiPreview,
  disconnectWifiPreview,
  forgetWifiNetworkPreview,
  getLocalAppSettings,
  LOCAL_APP_SETTINGS_RESET_EVENT,
  isSettingsPreviewEnabled,
  isShowcaseSettings,
  saveLocalAppSettings,
  scanWifiNetworks,
  setWifiEnabledPreview,
  type WifiNetwork,
  type WifiSecurity,
} from "../../services/settingsPreview";
import { SettingsCard, SettingsPageFrame, SettingsSwitch } from "./SettingsPrimitives";

type NetworkChoice = WifiNetwork | (Omit<HostWifiNetwork, "signal_percent"> & { signalPercent: number });

function validSsid(value: string): boolean {
  const bytes = new TextEncoder().encode(value.trim()).length;
  return bytes > 0 && bytes <= 32;
}

function passwordError(security: WifiSecurity, password: string): string {
  if (security === "Open") return "";
  if (password.length >= 8 && password.length <= 63) return "";
  if (/^[0-9a-fA-F]{64}$/.test(password)) return "";
  return "Enter a Wi-Fi password with 8–63 characters or a 64-character hexadecimal key.";
}

export function WifiSettingsPage({ embedded = false, active = true }: { embedded?: boolean; active?: boolean } = {}) {
  const canPreview = isSettingsPreviewEnabled();
  const showcase = isShowcaseSettings();
  const [wifi, setWifi] = useState(() => canPreview
    ? getLocalAppSettings().wifi
    : { enabled: false, connectedSsid: null, savedSsids: [] as string[] });
  const [hostAvailable, setHostAvailable] = useState(canPreview);
  const [wifiDeviceAvailable, setWifiDeviceAvailable] = useState(canPreview);
  const [networks, setNetworks] = useState<NetworkChoice[]>([]);
  const [selectedSsid, setSelectedSsid] = useState("");
  const [manualSsid, setManualSsid] = useState("");
  const [security, setSecurity] = useState<WifiSecurity>("WPA2-Personal");
  const [password, setPassword] = useState("");
  const [showPassword, setShowPassword] = useState(false);
  const [scanning, setScanning] = useState(false);
  const [connecting, setConnecting] = useState(false);
  const [error, setError] = useState("");
  const [notice, setNotice] = useState("");

  async function loadHostWifi() {
    setError("");
    try {
      const result = await getHostWifiSettings();
      setWifi({ enabled: result.enabled, connectedSsid: result.connected_ssid, savedSsids: result.saved_ssids });
      setWifiDeviceAvailable(result.device_available);
      setHostAvailable(true);
    } catch (caught) {
      setHostAvailable(false);
      setWifiDeviceAvailable(false);
      setError(caught instanceof Error ? `Host Wi-Fi unavailable: ${caught.message}` : "Host Wi-Fi is unavailable.");
    }
  }

  async function refreshNetworks() {
    if (!canPreview && !hostAvailable) return;
    setScanning(true);
    setError("");
    setNotice("");
    try {
      if (canPreview) {
        setNetworks(await scanWifiNetworks());
      } else {
        const result = await scanHostWifiNetworks();
        setNetworks(result.networks.map(({ ssid, security, signal_percent }) => ({ ssid, security, signalPercent: signal_percent })));
      }
    } catch (caught) {
      setError(caught instanceof Error ? caught.message : "Network scan could not be completed.");
    } finally {
      setScanning(false);
    }
  }

  useEffect(() => {
    if (!active) return;
    if (canPreview) void refreshNetworks();
    else void loadHostWifi();
  }, [active, canPreview]);

  useEffect(() => {
    if (!canPreview) return;
    const reset = () => {
      setWifi(getLocalAppSettings().wifi);
      setSelectedSsid("");
      setManualSsid("");
      setPassword("");
      setShowPassword(false);
      setError("");
      setNotice("");
    };
    window.addEventListener(LOCAL_APP_SETTINGS_RESET_EVENT, reset);
    return () => window.removeEventListener(LOCAL_APP_SETTINGS_RESET_EVENT, reset);
  }, [canPreview]);

  const chooseNetwork = (network: NetworkChoice) => {
    if (network.security === "Unsupported") return;
    setSelectedSsid(network.ssid);
    setManualSsid(network.ssid);
    setSecurity(network.security);
    setPassword("");
    setError("");
  };

  const toggleWifi = async (enabled: boolean) => {
    try {
      if (canPreview) {
        setWifi(setWifiEnabledPreview(enabled).wifi);
      } else {
        const result = await setHostWifiRadio(enabled);
        setWifi({ enabled: result.enabled, connectedSsid: result.connected_ssid, savedSsids: result.saved_ssids });
        setWifiDeviceAvailable(result.device_available);
      }
      setNotice(canPreview
        ? enabled ? "Wi-Fi preference enabled." : "Wi-Fi preference disabled."
        : enabled ? "Host Wi-Fi enabled." : "Host Wi-Fi disabled.");
      setError("");
    } catch (caught) {
      setError(caught instanceof Error ? caught.message : "Wi-Fi setting could not be saved.");
    }
  };

  const connect = async () => {
    if ((!canPreview && !hostAvailable) || connecting) return;
    const ssid = manualSsid.trim() || selectedSsid;
    if (!validSsid(ssid)) { setError("Enter an SSID with 1–32 UTF-8 bytes."); return; }
    const credentialError = passwordError(security, password);
    if (credentialError) { setError(credentialError); return; }
    setConnecting(true);
    setError("");
    setNotice("");
    try {
      if (canPreview) {
        await new Promise<void>((resolve) => window.setTimeout(resolve, 120));
        const next = connectWifiPreview(ssid);
        const preferences = showcase
          ? saveLocalAppSettings({ ...next, wifi: { ...next.wifi, connectedSsid: null } })
          : next;
        setWifi(preferences.wifi);
        setNotice(showcase ? `Saved network preference: ${ssid}. No connection was made.` : `Browser preview connected: ${ssid}.`);
      } else {
        const result = await connectHostWifi({ ssid, security, password });
        setWifi({ enabled: result.enabled, connectedSsid: result.connected_ssid, savedSsids: result.saved_ssids });
        setWifiDeviceAvailable(result.device_available);
        setNotice(`Connected to ${ssid}.`);
      }
      setPassword("");
    } catch (caught) {
      setError(caught instanceof Error ? caught.message : "Connection workflow failed.");
    } finally {
      setConnecting(false);
    }
  };

  const disconnect = async () => {
    try {
      if (canPreview) {
        setWifi(disconnectWifiPreview().wifi);
        setNotice(showcase ? "Saved network selection cleared. Host network unchanged." : "Browser preview disconnected.");
      } else {
        const result = await disconnectHostWifi();
        setWifi({ enabled: result.enabled, connectedSsid: result.connected_ssid, savedSsids: result.saved_ssids });
        setNotice("Host Wi-Fi disconnected.");
      }
      setError("");
    } catch (caught) {
      setError(caught instanceof Error ? caught.message : "Disconnect could not be completed.");
    }
  };

  const forget = async (ssid: string) => {
    try {
      if (canPreview) {
        setWifi(forgetWifiNetworkPreview(ssid).wifi);
      } else {
        const result = await forgetHostWifi(ssid);
        setWifi({ enabled: result.enabled, connectedSsid: result.connected_ssid, savedSsids: result.saved_ssids });
      }
      setNotice(`Removed ${ssid} from saved networks.`);
      setError("");
    } catch (caught) {
      setError(caught instanceof Error ? caught.message : "Saved network could not be removed.");
    }
  };

  const connected = wifi.enabled ? wifi.connectedSsid : null;
  const controlsAvailable = canPreview || hostAvailable;

  return <SettingsPageFrame title="Wi-Fi" embedded={embedded} active={active}
    sectionId={embedded ? "settings-wifi" : undefined}>

    {error && <Alert tone="error" role="alert">{error}</Alert>}
    {notice && <Alert tone="success" role="status">{notice}</Alert>}

    <div className="grid min-w-0 grid-cols-1 gap-hmi-3 xl:grid-cols-[minmax(0,1.15fr)_minmax(19rem,.85fr)] xl:items-start">
      <SettingsCard title="CONNECTION">
        <div className="flex flex-wrap items-center justify-between gap-hmi-3 border-b border-ptagv-border-soft pb-hmi-2">
          <div className="grid gap-1">
            <span className="text-sm font-bold text-ptagv-text">Wi-Fi radio</span>
            <small className="text-xs text-ptagv-muted">Current connection: {connected ?? (wifi.enabled ? "No network selected" : "Radio disabled")}</small>
          </div>
          <SettingsSwitch checked={wifi.enabled} onChange={(value) => void toggleWifi(value)} label="Wi-Fi enabled" disabled={!controlsAvailable} />
        </div>
        <div className="flex flex-wrap items-center justify-between gap-hmi-2">
          <div className="flex items-center gap-2">
            <StatusIndicator tone={connected ? "success" : "neutral"}>{connected ? "CONNECTED" : "NOT CONNECTED"}</StatusIndicator>
          </div>
          <div className="flex gap-2">
            <Button size="compact" disabled={!controlsAvailable || scanning || !wifi.enabled || (!canPreview && !wifiDeviceAvailable)} onClick={() => void refreshNetworks()}>
              {scanning ? "SCANNING…" : "REFRESH NETWORKS"}
            </Button>
            <Button size="compact" disabled={!controlsAvailable || !connected} onClick={() => void disconnect()}>DISCONNECT</Button>
          </div>
        </div>

        <div className="grid min-w-0 gap-1" aria-label="Available Wi-Fi networks">
          <h3 className="m-0 py-1 text-xs font-bold tracking-wide text-ptagv-muted">AVAILABLE NETWORKS</h3>
          {!canPreview && !hostAvailable ? <p className="m-0 text-sm text-ptagv-muted">NetworkManager unavailable.</p>
            : scanning && networks.length === 0 ? <p className="m-0 text-sm text-ptagv-muted" role="status">Scanning for networks…</p>
              : networks.map((network) => <button key={network.ssid} type="button" aria-pressed={selectedSsid === network.ssid}
                disabled={!wifi.enabled || network.security === "Unsupported"} onClick={() => chooseNetwork(network)}
                className={`grid min-h-12 min-w-0 grid-cols-[minmax(0,1fr)_auto] items-center gap-x-3 border px-2 py-1.5 text-left focus-visible:outline-2 focus-visible:outline-ptagv-blue disabled:opacity-50 ${selectedSsid === network.ssid ? "border-blue-400 bg-blue-50" : "border-ptagv-border-soft bg-white hover:bg-slate-50"}`}>
                <span className="min-w-0">
                  <b className="block truncate text-sm text-ptagv-text">{network.ssid}</b>
                  <small className="text-xs text-ptagv-muted">{network.security === "Unsupported" ? "Security unsupported" : network.security}</small>
                </span>
                <span className="text-right text-xs font-semibold text-ptagv-muted">{network.signalPercent}% signal</span>
              </button>)}
          {canPreview && !scanning && networks.length === 0 && <p className="m-0 text-sm text-ptagv-muted">No networks found.</p>}
        </div>
      </SettingsCard>

      <div className="flex min-w-0 flex-col gap-hmi-3">
        <SettingsCard title="CONNECT TO A NETWORK">
          <label className="grid gap-1 text-xs font-bold text-ptagv-muted">
            SSID
            <Input autoComplete="off" maxLength={32} value={manualSsid} disabled={!controlsAvailable || !wifi.enabled || (!canPreview && !wifiDeviceAvailable)}
              onChange={(event) => { setManualSsid(event.target.value); setSelectedSsid(""); }} placeholder="Enter network name" />
          </label>
          <label className="grid gap-1 text-xs font-bold text-ptagv-muted">
            SECURITY
            <Select value={security} disabled={!controlsAvailable || !wifi.enabled} onChange={(event) => {
              setSecurity(event.target.value as WifiSecurity);
              if (event.target.value === "Open") setPassword("");
            }}>
              <option value="WPA2-Personal">WPA2-Personal</option>
              <option value="WPA3-Personal">WPA3-Personal</option>
              <option value="Open">Open</option>
            </Select>
          </label>
          {security !== "Open" && <label className="grid gap-1 text-xs font-bold text-ptagv-muted">
            PASSWORD
            <span className="grid grid-cols-[minmax(0,1fr)_auto] gap-2">
              <Input type={showPassword ? "text" : "password"} autoComplete="new-password" value={password}
                disabled={!controlsAvailable || !wifi.enabled} onChange={(event) => setPassword(event.target.value)} />
              <Button size="compact" disabled={!controlsAvailable || !wifi.enabled} aria-pressed={showPassword}
                aria-label={showPassword ? "Hide password" : "Show password"} onClick={() => setShowPassword((shown) => !shown)}>
                {showPassword ? "HIDE" : "SHOW"}
              </Button>
            </span>
          </label>}
          <Button tone="primary" disabled={!controlsAvailable || !wifi.enabled || connecting || (!canPreview && !wifiDeviceAvailable)} onClick={() => void connect()}>
            {connecting ? "CONNECTING…" : "CONNECT"}
          </Button>
        </SettingsCard>

        <SettingsCard title="SAVED NETWORKS">
          {wifi.savedSsids.length === 0 ? <p className="m-0 text-sm text-ptagv-muted">No saved networks.</p>
            : wifi.savedSsids.map((ssid) => <div key={ssid} className="flex min-w-0 items-center justify-between gap-2 border-b border-ptagv-border-soft py-2 last:border-b-0">
              <span className="min-w-0 break-all text-sm font-semibold text-ptagv-text">{ssid}</span>
              <Button size="compact" disabled={!controlsAvailable} onClick={() => void forget(ssid)}>FORGET</Button>
            </div>)}
          <small className="text-xs leading-relaxed text-ptagv-muted">Passwords are never stored.</small>
        </SettingsCard>
      </div>
    </div>
  </SettingsPageFrame>;
}
