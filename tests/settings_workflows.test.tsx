// @vitest-environment jsdom
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { FactorySettingsPage } from "../src/pages/settings/FactorySettingsPage";
import { NtpSettingsPage } from "../src/pages/settings/NtpSettingsPage";
import { VolumeSettingsPage } from "../src/pages/settings/VolumeSettingsPage";
import { WifiSettingsPage } from "../src/pages/settings/WifiSettingsPage";
import {
  getLocalAppSettings,
  PREVIEW_SETTINGS_STORAGE_KEY,
} from "../src/services/settingsPreview";
import { useStore } from "../src/state/store";

vi.mock("../src/config", async (importOriginal) => {
  const config = await importOriginal<typeof import("../src/config")>();
  return { ...config, UI_MODE: "STATIC_UI", IS_STATIC_UI: true };
});

let root: Root;
let container: HTMLDivElement;
const initialState = useStore.getState();

function mount(node: React.ReactNode) { act(() => root.render(node)); }
function button(label: string): HTMLButtonElement | undefined {
  return Array.from(container.querySelectorAll<HTMLButtonElement>("button"))
    .find((item) => item.textContent?.trim() === label || item.getAttribute("aria-label") === label);
}
function labeledInput(label: string): HTMLInputElement {
  const field = Array.from(container.querySelectorAll("label"))
    .find((item) => item.textContent?.trim().startsWith(label));
  const input = field?.querySelector<HTMLInputElement>("input");
  if (!input) throw new Error(`Input ${label} was not found.`);
  return input;
}
function setValue(element: HTMLInputElement | HTMLSelectElement, value: string) {
  const prototype = element instanceof HTMLInputElement ? HTMLInputElement.prototype : HTMLSelectElement.prototype;
  Object.getOwnPropertyDescriptor(prototype, "value")?.set?.call(element, value);
  element.dispatchEvent(new Event("input", { bubbles: true }));
  element.dispatchEvent(new Event("change", { bubbles: true }));
}
async function settle(milliseconds = 0) {
  await act(async () => { await new Promise((resolve) => window.setTimeout(resolve, milliseconds)); });
}

beforeEach(() => {
  globalThis.IS_REACT_ACT_ENVIRONMENT = true;
  window.localStorage.clear();
  useStore.setState(initialState);
  container = document.createElement("div");
  document.body.appendChild(container);
  root = createRoot(container);
});

afterEach(() => {
  act(() => root.unmount());
  container.remove();
  useStore.setState(initialState);
  window.localStorage.clear();
  vi.unstubAllGlobals();
});

describe("browser-local setting workflows", () => {
  it("scans, validates, connects, disconnects, and forgets Wi-Fi without storing a password", async () => {
    mount(<WifiSettingsPage />);
    await settle(140);
    const network = Array.from(container.querySelectorAll<HTMLButtonElement>("button"))
      .find((item) => item.textContent?.includes("PTAGV-Maintenance"));
    expect(network).toBeDefined();
    act(() => network?.click());
    act(() => button("CONNECT")?.click());
    expect(container.querySelector('[role="alert"]')?.textContent).toContain("8–63 characters");

    const password = container.querySelector<HTMLInputElement>('input[type="password"]');
    expect(password).not.toBeNull();
    if (!password) return;
    const secret = "temporary-secret-123";
    act(() => setValue(password, secret));
    act(() => button("Show password")?.click());
    expect(container.querySelector<HTMLInputElement>('input[type="text"]')?.value).toBe(secret);
    act(() => button("CONNECT")?.click());
    await settle(140);
    expect(container.querySelector('[role="status"]')?.textContent).toContain("Browser preview connected");
    expect(getLocalAppSettings().wifi.connectedSsid).toBe("PTAGV-Maintenance");
    expect(window.localStorage.getItem(PREVIEW_SETTINGS_STORAGE_KEY)).not.toContain(secret);

    act(() => button("DISCONNECT")?.click());
    expect(getLocalAppSettings().wifi.connectedSsid).toBeNull();
    act(() => button("FORGET")?.click());
    expect(getLocalAppSettings().wifi.savedSsids).not.toContain("PTAGV-Maintenance");
  });

  it("saves volume, mute, and reset preferences and starts test audio only on button activation", async () => {
    const audioStarted: number[] = [];
    class FakeAudioContext {
      currentTime = 1;
      destination = {};
      createOscillator() {
        const oscillator = { frequency: { value: 0 }, type: "sine", connect: () => undefined,
          start: () => audioStarted.push(oscillator.frequency.value), stop: () => undefined, onended: null as (() => void) | null };
        return oscillator;
      }
      createGain() { return { gain: { value: 0 }, connect: () => undefined }; }
      close() { return Promise.resolve(); }
    }
    vi.stubGlobal("AudioContext", FakeAudioContext);
    mount(<VolumeSettingsPage />);
    const slider = container.querySelector<HTMLInputElement>('input[aria-label="Test tone percentage"]');
    expect(slider?.value).toBe("65");
    if (!slider) return;
    act(() => setValue(slider, "42"));
    expect(button("SAVE PREFERENCE")?.disabled).toBe(false);
    act(() => button("SAVE PREFERENCE")?.click());
    expect(getLocalAppSettings().volume.level).toBe(42);
    act(() => button("Mute browser tone")?.click());
    act(() => button("SAVE PREFERENCE")?.click());
    expect(getLocalAppSettings().volume.muted).toBe(true);
    await act(async () => { button("TEST SOUND")?.click(); await Promise.resolve(); });
    expect(audioStarted).toEqual([]);
    expect(container.querySelector('[role="status"]')?.textContent).toContain("no tone was played");
    act(() => button("Mute browser tone")?.click());
    act(() => button("SAVE PREFERENCE")?.click());
    await act(async () => { button("TEST SOUND")?.click(); await Promise.resolve(); });
    expect(audioStarted).toEqual([660]);
    act(() => button("RESET TO DEFAULT")?.click());
    expect(getLocalAppSettings().volume).toEqual({ level: 65, muted: false });
  });

  it("validates, saves, cancels, and checks preview NTP settings without changing the host clock", async () => {
    mount(<NtpSettingsPage />);
    const primary = labeledInput("PRIMARY NTP SERVER");
    act(() => setValue(primary, "bad host"));
    expect(button("SAVE SETTINGS")?.disabled).toBe(true);
    expect(container.textContent).toContain("valid primary NTP hostname");

    act(() => setValue(primary, "time.example.org"));
    const timezone = Array.from(container.querySelectorAll<HTMLLabelElement>("label"))
      .find((item) => item.textContent?.trim().startsWith("TIMEZONE"))?.querySelector("select");
    expect(timezone).not.toBeNull();
    if (!timezone) return;
    act(() => setValue(timezone, "UTC"));
    act(() => button("SAVE SETTINGS")?.click());
    expect(getLocalAppSettings().ntp.primaryServer).toBe("time.example.org");
    expect(getLocalAppSettings().ntp.timezone).toBe("UTC");

    act(() => setValue(primary, "discard.example.org"));
    act(() => button("CANCEL CHANGES")?.click());
    expect(primary.value).toBe("time.example.org");

    await act(async () => { button("CHECK PREVIEW")?.click(); await Promise.resolve(); });
    expect(getLocalAppSettings().ntp.lastCheckAt).toMatch(/^\d{4}-\d\d-\d\dT/);
    expect(container.querySelector('[role="status"]')?.textContent).toContain("robot time are unchanged");
  });

  it("requires RESET confirmation and preserves UI-independent map records", () => {
    const initial = getLocalAppSettings();
    window.localStorage.setItem(PREVIEW_SETTINGS_STORAGE_KEY, JSON.stringify({ ...initial, volume: { level: 12, muted: true } }));
    window.localStorage.setItem("ptagv.map-registry.v1", "preserve-map-record");
    mount(<FactorySettingsPage />);
    expect(button("FACTORY RESET ROBOT / SYSTEM")?.disabled).toBe(true);
    act(() => button("RESET SAVED SETTINGS")?.click());
    expect(container.querySelector('[role="dialog"]')).not.toBeNull();
    expect(button("CONFIRM RESET")?.disabled).toBe(true);
    const confirmation = labeledInput("TYPE RESET TO CONFIRM");
    act(() => setValue(confirmation, "RESET "));
    expect(button("CONFIRM RESET")?.disabled).toBe(true);
    act(() => setValue(confirmation, "RESET"));
    act(() => button("CONFIRM RESET")?.click());
    expect(window.localStorage.getItem(PREVIEW_SETTINGS_STORAGE_KEY)).toBeNull();
    expect(window.localStorage.getItem("ptagv.map-registry.v1")).toBe("preserve-map-record");
    expect(container.querySelector('[role="dialog"]')).toBeNull();
    expect(container.querySelector('[role="status"]')?.textContent).toContain("Browser settings reset");
  });

  it("refreshes mounted preference panels after factory settings are reset", () => {
    const initial = getLocalAppSettings();
    window.localStorage.setItem(PREVIEW_SETTINGS_STORAGE_KEY, JSON.stringify({
      ...initial,
      volume: { level: 18, muted: true },
    }));
    mount(<><VolumeSettingsPage embedded /><FactorySettingsPage embedded /></>);
    expect(container.querySelector<HTMLInputElement>('input[aria-label="Test tone percentage"]')?.value).toBe("18");

    act(() => button("RESET SAVED SETTINGS")?.click());
    const confirmation = labeledInput("TYPE RESET TO CONFIRM");
    act(() => setValue(confirmation, "RESET"));
    act(() => button("CONFIRM RESET")?.click());

    expect(container.querySelector<HTMLInputElement>('input[aria-label="Test tone percentage"]')?.value).toBe("65");
    expect(container.querySelector<HTMLElement>('[role="switch"][aria-label="Mute browser tone"]')?.getAttribute("aria-checked")).toBe("false");
  });
});
