// @vitest-environment jsdom
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import { afterAll, afterEach, beforeEach, describe, expect, it, vi } from "vitest";

vi.mock("../src/config", async (importOriginal) => {
  const config = await importOriginal<typeof import("../src/config")>();
  return { ...config, UI_MODE: "STATIC_UI", IS_STATIC_UI: true };
});
const { default: App } = await import("../src/App");
const { useStore } = await import("../src/state/store");
const { resolveUiMode } = await import("../src/config");
const { STATIC_ROBOT_ID } = await import("../src/mock/fixtures");
const { UI_PREFERENCES_STORAGE_KEY } = await import("../src/services/settingsPreview");
const initialState = useStore.getState();

class ForbiddenWebSocket {
  static attempts: string[] = [];
  constructor(url: string | URL) {
    ForbiddenWebSocket.attempts.push(String(url));
    throw new Error("STATIC_UI must not create WebSockets.");
  }
}

const fetchSpy = vi.fn<typeof fetch>();
let root: Root;
let container: HTMLDivElement;

async function mountStaticApp(path = "/control") {
  window.history.replaceState({}, "", path);
  await import("../src/mock/StaticUiApp");
  await act(async () => { root.render(<App />); });
  await act(async () => { await new Promise((resolve) => window.setTimeout(resolve, 0)); });
}

async function selectSection(label: string) {
  const pathBySection: Record<string, string> = {
    SYSTEM: "/system", SETTINGS: "/settings", WIFI: "/settings/wifi", VOLUME: "/settings/volume",
    NTP: "/settings/ntp", OPERATION_MODE: "/settings/operation-mode", FACTORY: "/settings/factory",
    SETTINGS_MAP: "/settings/map",
    CONTROL: "/control", MAPPING: "/mapping", LOCALIZATION: "/localization",
    VDA5050: "/vda5050", DIAGNOSIS: "/diagnosis",
  };
  const settingLabel: Record<string, string> = {
    WIFI: "Wi-Fi", VOLUME: "Volume", NTP: "NTP",
    OPERATION_MODE: "Operation Mode", FACTORY: "Factory Setting",
  };
  if (settingLabel[label] || label === "SETTINGS_MAP") {
    if (!container.querySelector("main[aria-label='Settings workspace']")) {
      await act(async () => {
        container.querySelector<HTMLAnchorElement>(`.wt-sidebar-item[href="${pathBySection.SETTINGS}"]`)?.click();
        await new Promise((resolve) => window.setTimeout(resolve, 0));
      });
    }
    const settingsAction = label === "SETTINGS_MAP"
      ? container.querySelector<HTMLAnchorElement>("a.settings-map-link")
      : container.querySelector<HTMLButtonElement>(`nav[aria-label='Settings sections'] button[aria-label='${settingLabel[label]}']`);
    await act(async () => {
      settingsAction?.click();
      await new Promise((resolve) => window.setTimeout(resolve, 0));
    });
    return;
  }
  await act(async () => {
    const link = container.querySelector<HTMLAnchorElement>(`.wt-sidebar-item[href="${pathBySection[label]}"]`);
    link?.click();
    await new Promise((resolve) => window.setTimeout(resolve, 0));
  });
}

function expectCleanStaticPresentation() {
  const renderedStrings = Array.from(container.querySelectorAll<HTMLElement>("*"))
    .flatMap((element) => [element.textContent ?? "", element.getAttribute("aria-label") ?? "",
      element.getAttribute("title") ?? "", element.getAttribute("placeholder") ?? "",
      element.getAttribute("value") ?? "", element instanceof HTMLInputElement ? element.value : ""]);
  expect(renderedStrings.join("\n")).not.toMatch(/STATIC\s*UI\s*\/\s*NO\s*BACKEND|NO\s+BACKEND|\bSTATIC_UI\b|PREVIEW\s+ONLY|PREVIEW\s+MODE|DEMO\s+MODE|MOCK\s+MODE|GOALS\s+UNAVAILABLE|NAV\s+UNKNOWN|\bOFFLINE\b|\bDISCONNECTED\b|\bUNAVAILABLE\b|\bSAMPLE\b|\bSTATIC\s+FIXTURE\b|RESET\s+TO\s+SAMPLE/i);
  expect(container.querySelectorAll(".static-ui-badge")).toHaveLength(0);
}

function setTextInputValue(input: HTMLInputElement, value: string) {
  Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value")?.set?.call(input, value);
  input.dispatchEvent(new Event("input", { bubbles: true }));
  input.dispatchEvent(new Event("change", { bubbles: true }));
}

beforeEach(() => {
  globalThis.IS_REACT_ACT_ENVIRONMENT = true;
  window.innerWidth = 1280;
  useStore.setState(initialState);
  container = document.createElement("div");
  document.body.appendChild(container);
  root = createRoot(container);
  fetchSpy.mockReset().mockRejectedValue(new Error("Unexpected network request in STATIC_UI"));
  vi.stubGlobal("fetch", fetchSpy);
  ForbiddenWebSocket.attempts = [];
  vi.stubGlobal("WebSocket", ForbiddenWebSocket);
  vi.stubGlobal("ResizeObserver", class {
    observe() {}
    disconnect() {}
  });
  vi.spyOn(HTMLCanvasElement.prototype, "getContext").mockReturnValue(null);
  vi.spyOn(window, "setInterval");
});

afterEach(() => {
  act(() => root.unmount());
  container.remove();
  useStore.setState(initialState);
  vi.restoreAllMocks();
});

afterAll(() => {
  vi.unstubAllGlobals();
});

describe("STATIC_UI mode", () => {
  it("defaults to backend-connected and rejects static mode for production", () => {
    expect(resolveUiMode(undefined, true)).toBe("BACKEND_CONNECTED");
    expect(resolveUiMode("STATIC_UI", true)).toBe("STATIC_UI");
    expect(() => resolveUiMode("STATIC_UI", false)).toThrow("development server");
    expect(resolveUiMode("SHOWCASE", false, true)).toBe("SHOWCASE");
    expect(() => resolveUiMode("SHOWCASE", true, false)).toThrow("dedicated Vite showcase build");
  });

  it("renders the top-level HMI groups without static-mode notices or fake runtime status", async () => {
    await mountStaticApp();

    expect(container.querySelectorAll(".wt-sidebar-item")).toHaveLength(7);
    expectCleanStaticPresentation();
    expect(container.querySelectorAll(".local-app-header")).toHaveLength(1);
    expect(container.querySelectorAll(".hmi-section-heading")).toHaveLength(0);
    expect(container.querySelector(".local-app-header-title")?.textContent).toBe("CONTROL");
    expect(container.querySelector(".wt-sidebar-brand")?.getAttribute("aria-label")).toBe("PTAGV");
    expect(container.querySelector(".wt-sidebar-mark")?.textContent).toBe("P");
    expect(container.textContent).not.toContain("WAITING FOR LOCAL ROBOT CONFIGURATION");
    expect(container.textContent).toContain("PTAGV");
    expect(container.textContent).not.toContain("R01");
    expect(container.querySelector(".hmi-estop")).not.toBeNull();
    expect(container.querySelector(".hmi-clear-stop")).not.toBeNull();
    expect(useStore.getState().localRobotId).toBeNull();
    expect(useStore.getState().localIdentityVerified).toBe(false);
    expect(container.querySelector('[data-testid="active-navigation-map-2d"]')).not.toBeNull();

    await selectSection("MAPPING");
    expect(container.querySelector(".local-app-header-title")?.textContent).toBe("MAPPING");
    expect(container.textContent).not.toContain("OCCUPANCY MAP");
    expect(container.textContent).not.toContain("MAP WORKSPACE");
    expect(container.querySelector(".mapping-layer-toolbar")).not.toBeNull();
    expect(container.querySelector(".mapping-map-card h2")).toBeNull();
    expect(container.querySelector(".mapping-map-card .local-pose-map canvas")).not.toBeNull();
    expect(container.querySelector(".mapping-map-card canvas")?.getAttribute("data-robot-layer")).toBe("disabled");
    expect(container.querySelectorAll(".mapping-map-option")).toHaveLength(1);
    expect(container.querySelector(".mapping-map-status")).toBeNull();
    expectCleanStaticPresentation();
    expect(container.querySelector<HTMLButtonElement>(".mapping-save-action")?.disabled).toBe(true);
    expect(container.querySelector<HTMLButtonElement>(".hmi-map-load-button")?.disabled).toBe(true);
    const scanLayer = Array.from(container.querySelectorAll<HTMLButtonElement>(".mapping-layer-toolbar button"))
      .find((button) => button.textContent?.trim() === "SCAN");
    expect(scanLayer?.getAttribute("aria-pressed")).toBe("true");
    await act(async () => { scanLayer?.click(); });
    expect(scanLayer?.getAttribute("aria-pressed")).toBe("false");
    const robotLayer = Array.from(container.querySelectorAll<HTMLButtonElement>(".mapping-layer-toolbar button"))
      .find((button) => button.textContent?.trim() === "ROBOT");
    await act(async () => { robotLayer?.click(); });
    expect(container.querySelector(".mapping-map-card canvas")?.getAttribute("data-robot-layer")).toBe("disabled");
    await act(async () => { container.querySelector<HTMLButtonElement>('.mapping-selected-actions button[aria-controls="mapping-map-preview"]')?.click(); });
    expect(container.querySelector(".mapping-preview-drawer .local-pose-map canvas")).not.toBeNull();
    expect(container.querySelector(".mapping-preview-heading")?.textContent).toBe("MAP PREVIEW");
    expect(container.querySelector<HTMLButtonElement>('.mapping-selected-actions button[aria-controls="mapping-map-preview"]')?.textContent).toBe("HIDE PREVIEW");
    expect(fetchSpy).not.toHaveBeenCalled();
    expect(ForbiddenWebSocket.attempts).toEqual([]);
    expect(container.textContent).not.toContain("R01");

    await selectSection("LOCALIZATION");
    expect(container.querySelector(".local-app-header-title")?.textContent).toBe("LOCALIZATION");
    expect(container.textContent).toContain("LOCALIZATION STATUS");
    expect(container.textContent).toContain("CURRENT ROBOT POSE");
    expect(container.textContent).not.toContain("LOCALIZATION MAP");
    expect(container.textContent).not.toMatch(/(?:^|\s)[A-D] · (?:LOCALIZATION|CURRENT|INITIAL|TF)/);
    expect(container.querySelector('[data-testid="localization-estop-state"]')).toBeNull();
    expect(container.querySelector('[data-testid="localization-state-badge"]')?.textContent).toBe("—");
    expectCleanStaticPresentation();
    expect(container.textContent).not.toContain("R01");

    await selectSection("VDA5050");
    expect(container.querySelector(".local-app-header-title")?.textContent).toBe("VDA5050");
    expect(container.textContent).toContain("Broker Connection");
    expect(container.textContent).toContain("VDA5050 Protocol Identity");
    expect(container.textContent).toContain("Runtime & Task Policy");
    expect(container.textContent).not.toContain("BACK TO DIAGNOSIS");
    const hostInput = Array.from(container.querySelectorAll("label"))
      .find((label) => label.textContent?.includes("MQTT host"))
      ?.querySelector<HTMLInputElement>("input");
    expect(hostInput?.value).toBe("");
    expectCleanStaticPresentation();
    expect(container.textContent).not.toContain("R01");

    await selectSection("DIAGNOSIS");
    expect(container.querySelector(".local-app-header-title")?.textContent).toBe("DIAGNOSIS");
    expect(container.textContent).toContain("ROS");
    expect(container.textContent).not.toContain("ACTIVE LOCAL MAP");
    expect(Array.from(container.querySelectorAll<HTMLElement>(".robot-detail-status"))
      .some((status) => status.classList.contains("text-ptagv-success"))).toBe(false);
    expectCleanStaticPresentation();
    expect(container.textContent).not.toContain("R01");
  });

  it.each([
    ["/system", "SYSTEM"],
    ["/control", "CONTROL"],
    ["/mapping", "MAPPING"],
    ["/localization", "LOCALIZATION"],
    ["/vda5050", "VDA5050"],
    ["/diagnosis", "DIAGNOSIS"],
    ["/settings", "SETTINGS"],
    ["/settings/wifi", "SETTINGS"],
    ["/settings/volume", "SETTINGS"],
    ["/settings/ntp", "SETTINGS"],
    ["/settings/operation-mode", "SETTINGS"],
    ["/settings/factory", "SETTINGS"],
    ["/settings/map", "SETTING · MAP"],
  ])("opens static deep link %s directly at %s", async (path, title) => {
    await mountStaticApp(path);
    expect(window.location.pathname).toBe(path);
    expect(container.querySelector(".local-app-header-title")?.textContent).toBe(title);
    expect(container.querySelectorAll("h1")).toHaveLength(1);
    expect(container.querySelector(".settings-page > h2")).toBeNull();
    const sidebarPath = path.startsWith("/settings") ? "/settings" : path;
    expect(container.querySelector(`.wt-sidebar-item[href="${sidebarPath}"]`)?.getAttribute("aria-current")).toBe("page");
    if (path.startsWith("/settings") && path !== "/settings/map") {
      const tabLabel = path === "/settings" ? "Wi-Fi" : ({
        "/settings/wifi": "Wi-Fi",
        "/settings/volume": "Volume",
        "/settings/ntp": "NTP",
        "/settings/operation-mode": "Operation Mode",
        "/settings/factory": "Factory Setting",
      } as Record<string, string>)[path];
      expect(container.querySelector(`nav[aria-label='Settings sections'] button[aria-label='${tabLabel}'][aria-current='location']`)).not.toBeNull();
    }
    const header = container.querySelector(".local-app-header");
    expect(container.querySelectorAll(".local-app-header")).toHaveLength(1);
    expect(header?.textContent).not.toContain("PTAGV");
    expect(header?.textContent).not.toContain("|");
    expect(container.querySelector(".wt-sidebar-brand")?.textContent).toContain("PTAGV");
    expect(header?.textContent).not.toMatch(/WARETWIN|LOCAL ROBOT|ROBOT R01|R01/i);
    expectCleanStaticPresentation();
    expect(container.textContent).not.toContain("R01");
    expect(fetchSpy).not.toHaveBeenCalled();
    expect(ForbiddenWebSocket.attempts).toEqual([]);
  });

  it("shows System and the five settings sections without requesting backend state", async () => {
    await mountStaticApp("/system");
    expect(container.querySelector(".local-app-header-title")?.textContent).toBe("SYSTEM");
    expect(container.textContent).not.toContain("System overview");
    expect(container.querySelector('main[aria-label="System overview"]')).not.toBeNull();
    expect(container.textContent).toContain("BROWSER CLOCK");
    expect(container.textContent).toContain("ROBOT IDENTITY");
    expect(container.textContent).not.toContain("R01");
    expect(fetchSpy).not.toHaveBeenCalled();

    await selectSection("SETTINGS");
    expect(container.querySelector(".local-app-header-title")?.textContent).toBe("SETTINGS");
    const destinations = ["Wi-Fi", "Volume", "NTP", "Operation Mode", "Factory Setting"];
    for (const destination of destinations) {
      expect(container.querySelector(`button[aria-label="${destination}"]`)).not.toBeNull();
      expect(container.querySelector(`[data-settings-section="${destination}"]`)).not.toBeNull();
    }
    expect(container.querySelectorAll(".wt-sidebar-item[href^='/settings']")).toHaveLength(1);
    expect(container.querySelector("button[aria-label='Toggle Settings pages']")).toBeNull();
    expect(container.querySelector("a.settings-map-link[href='/settings/map']")?.textContent).toContain("MAP SETTINGS");
    expect(container.querySelectorAll(".settings-workspace-section")).toHaveLength(5);
    expect(container.querySelectorAll(".settings-workspace-section:not([hidden])")).toHaveLength(1);
    expect(container.querySelector(".settings-workspace-section[data-settings-section='Wi-Fi']")?.hidden).toBe(false);
    expect(container.querySelectorAll(".settings-route-link")).toHaveLength(0);
    expect(container.textContent).not.toContain("SETTING DESTINATIONS");
    expect(fetchSpy).not.toHaveBeenCalled();
    expect(ForbiddenWebSocket.attempts).toEqual([]);
  });

  it("shows one settings section at a time and preserves drafts across tabs", async () => {
    window.localStorage.clear();
    await mountStaticApp("/settings/wifi");
    const visibleSection = () => container.querySelectorAll(".settings-workspace-section:not([hidden])");
    const selectTab = async (label: string) => {
      await act(async () => {
        container.querySelector<HTMLButtonElement>(`nav[aria-label='Settings sections'] button[aria-label='${label}']`)?.click();
        await new Promise((resolve) => window.setTimeout(resolve, 0));
      });
      expect(visibleSection()).toHaveLength(1);
      expect(visibleSection()[0]?.getAttribute("data-settings-section")).toBe(label);
    };

    const ssid = container.querySelector<HTMLInputElement>("#settings-wifi input");
    expect(ssid).not.toBeNull();
    if (!ssid) return;
    setTextInputValue(ssid, "Draft Network");
    await selectTab("Volume");
    const volume = container.querySelector<HTMLInputElement>('#settings-volume input[aria-label="Test tone percentage"]');
    expect(volume).not.toBeNull();
    if (!volume) return;
    setTextInputValue(volume, "42");
    await selectTab("NTP");
    const ntp = container.querySelector<HTMLInputElement>("#settings-ntp input");
    expect(ntp).not.toBeNull();
    if (!ntp) return;
    setTextInputValue(ntp, "draft.time.example.org");

    await selectTab("Wi-Fi");
    expect(ssid.value).toBe("Draft Network");
    await selectTab("Volume");
    expect(volume.value).toBe("42");
    await selectTab("NTP");
    expect(ntp.value).toBe("draft.time.example.org");
    expect(fetchSpy).not.toHaveBeenCalled();
    expect(ForbiddenWebSocket.attempts).toEqual([]);
  });

  it("runs the NTP clock timer only while the NTP tab is active", async () => {
    await mountStaticApp("/settings/volume");
    const intervals = vi.mocked(window.setInterval);
    const clockTimerCount = () => intervals.mock.calls.filter(([, delay]) => delay === 1000).length;
    const initialClockTimerCount = clockTimerCount();
    const clearIntervalSpy = vi.spyOn(window, "clearInterval");

    const intervalCallsBeforeNtp = intervals.mock.calls.length;
    await act(async () => {
      container.querySelector<HTMLButtonElement>('nav[aria-label="Settings sections"] button[aria-label="NTP"]')?.click();
      await new Promise((resolve) => window.setTimeout(resolve, 0));
    });
    expect(clockTimerCount()).toBe(initialClockTimerCount + 1);
    const ntpTimerIndex = intervals.mock.calls.findIndex(([, delay], index) => index >= intervalCallsBeforeNtp && delay === 1000);
    const ntpTimerId = intervals.mock.results[ntpTimerIndex]?.value;

    await act(async () => {
      container.querySelector<HTMLButtonElement>('nav[aria-label="Settings sections"] button[aria-label="Factory Setting"]')?.click();
      await new Promise((resolve) => window.setTimeout(resolve, 0));
    });
    expect(clockTimerCount()).toBe(initialClockTimerCount + 1);
    expect(clearIntervalSpy).toHaveBeenCalledWith(ntpTimerId);
  });

  it("demonstrates requested and applied operation mode in browser state only", async () => {
    await mountStaticApp("/settings/operation-mode");
    expect(container.textContent).toContain("ROBOT CONTROL MODE");
    expect(container.textContent).toContain("MAPPING / NAVIGATION RUNTIME");
    const manualButton = Array.from(container.querySelectorAll<HTMLButtonElement>("button"))
      .find((button) => button.querySelector("b")?.textContent?.trim() === "MANUAL");
    expect(manualButton?.disabled).toBe(false);
    await act(async () => { manualButton?.click(); });
    expect(container.querySelector('[role="dialog"]')).not.toBeNull();
    await act(async () => { Array.from(container.querySelectorAll<HTMLButtonElement>("button"))
      .find((button) => button.textContent?.trim() === "CONFIRM REQUEST")?.click(); });
    expect(container.textContent).toContain("REQUESTED · MANUAL");
    await act(async () => { Array.from(container.querySelectorAll<HTMLButtonElement>("button"))
      .find((button) => button.textContent?.trim() === "CONFIRM BROWSER SELECTION")?.click(); });
    expect(container.textContent).toContain("BROWSER SELECTION");
    expect(container.textContent).toContain("MANUAL");
    expect(useStore.getState().twin?.robots?.[STATIC_ROBOT_ID]?.control_mode).toBe("AUTONOMOUS");

    await act(async () => { Array.from(container.querySelectorAll<HTMLButtonElement>("button"))
      .find((button) => button.textContent?.trim() === "SELECT MAPPING")?.click(); });
    expect(useStore.getState().runtimeState).toBe("MAPPING");
    expect(fetchSpy).not.toHaveBeenCalled();
    expect(ForbiddenWebSocket.attempts).toEqual([]);
  });

  it("resets display preferences only after exact RESET confirmation", async () => {
    window.localStorage.setItem(UI_PREFERENCES_STORAGE_KEY, JSON.stringify({ legacyPreference: true }));
    useStore.getState().setLabelLayerVisible("robots", false);
    await mountStaticApp("/settings/factory");
    expect(container.querySelector(".wt-sidebar-item[href='/settings'][aria-current='page']")).not.toBeNull();
    expect(container.querySelector("button[aria-label='Toggle Settings pages']")).toBeNull();
    await act(async () => { Array.from(container.querySelectorAll<HTMLButtonElement>("button"))
      .find((button) => button.textContent?.trim() === "RESET DISPLAY PREFS")?.click(); });
    const phrase = container.querySelector<HTMLInputElement>('[role="dialog"] input[autocomplete="off"]');
    expect(phrase).not.toBeNull();
    if (!phrase) return;
    const setPhrase = (value: string) => {
      Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value")?.set?.call(phrase, value);
      phrase.dispatchEvent(new Event("input", { bubbles: true }));
      phrase.dispatchEvent(new Event("change", { bubbles: true }));
    };
    await act(async () => {
      setPhrase("RESET ");
    });
    expect(Array.from(container.querySelectorAll<HTMLButtonElement>("button"))
      .find((button) => button.textContent?.trim() === "CONFIRM RESET")?.disabled).toBe(true);
    await act(async () => {
      setPhrase("RESET");
    });
    expect(Array.from(container.querySelectorAll<HTMLButtonElement>("button"))
      .find((button) => button.textContent?.trim() === "CONFIRM RESET")?.disabled).toBe(false);
    await act(async () => { Array.from(container.querySelectorAll<HTMLButtonElement>("button"))
      .find((button) => button.textContent?.trim() === "CONFIRM RESET")?.click(); });
    expect(window.localStorage.getItem(UI_PREFERENCES_STORAGE_KEY)).toBeNull();
    expect(useStore.getState().labelLayers.robots.visible).toBe(true);
    expect(container.querySelector('[role="status"]')?.textContent).toContain("Robot settings and maps unchanged");
  });

  it("keeps Diagnosis reachable from VDA5050 through the sidebar", async () => {
    await mountStaticApp("/vda5050");
    expect(container.textContent).not.toContain("BACK TO DIAGNOSIS");
    const diagnosisLink = container.querySelector<HTMLAnchorElement>('.wt-sidebar-item[href="/diagnosis"]');
    await act(async () => {
      diagnosisLink?.click();
      await new Promise((resolve) => window.setTimeout(resolve, 0));
    });

    expect(window.location.pathname).toBe("/diagnosis");
    expect(container.querySelector(".local-app-header-title")?.textContent).toBe("DIAGNOSIS");
    expect(fetchSpy).not.toHaveBeenCalled();
    expect(ForbiddenWebSocket.attempts).toEqual([]);
  });

  it("keeps preview interactions local and blocks HTTP, WebSocket, and reconnect polling", async () => {
    await mountStaticApp();
    await selectSection("VDA5050");

    const hostInput = Array.from(container.querySelectorAll("label"))
      .find((label) => label.textContent?.includes("MQTT host"))
      ?.querySelector<HTMLInputElement>("input");
    expect(hostInput).not.toBeNull();
    if (hostInput) {
      await act(async () => {
        setTextInputValue(hostInput, "preview.local");
      });
      expect(hostInput.value).toBe("preview.local");
    }
    expect(container.querySelector('[data-testid="vda5050-connection-overview"]')?.textContent).toContain("Not configured");
    expect(container.querySelector('[data-testid="vda5050-connection-overview"]')?.textContent).not.toContain("preview.local:1883");
    const saveButton = Array.from(container.querySelectorAll<HTMLButtonElement>("button"))
      .find((button) => button.textContent?.trim() === "SAVE & APPLY");
    expect(saveButton?.disabled).toBe(false);
    await act(async () => { Array.from(container.querySelectorAll<HTMLButtonElement>("button"))
      .find((button) => button.textContent?.trim() === "DISCARD CHANGES")?.click(); });
    expect(hostInput?.value).toBe("");
    expect(saveButton?.disabled).toBe(true);
    if (hostInput) {
      await act(async () => {
        setTextInputValue(hostInput, "preview.local");
      });
    }
    await act(async () => { Array.from(container.querySelectorAll("button")).find((button) => button.textContent?.trim() === "TEST CONNECTION")?.click(); });
    expect(container.textContent).toContain("Broker not contacted.");
    await act(async () => { Array.from(container.querySelectorAll("button")).find((button) => button.textContent?.trim() === "SAVE & APPLY")?.click(); });
    expect(container.querySelector('[data-testid="vda5050-connection-overview"]')?.textContent).toContain("preview.local:1883");
    expect(container.textContent).not.toMatch(/CONFIGURATION APPLIED|CONNECTED\s+·/i);
    expect(container.querySelector('[data-testid="vda5050-connection-overview"]')?.textContent).toContain("Not checked");
    expect(container.querySelector('[data-testid="vda5050-connection-overview"]')?.textContent).toContain("2.1.0");
    expect(container.querySelector('[data-testid="vda5050-connection-overview"]')?.textContent).toContain("5.0");
    expect(container.querySelector('[data-testid="vda5050-page-content"]')?.textContent).toContain("Task permissions");
    expect(container.querySelector('[data-testid="vda5050-page-content"]')?.textContent).toContain("Reconnect behavior");
    expect(container.querySelector('[data-testid="vda5050-page-content"]')?.textContent).toContain("Connection timing");
    expect(container.querySelector('[data-testid="vda5050-page-content"]')?.textContent).not.toMatch(/OFFLINE|UNAVAILABLE|DISCONNECTED/i);
    expect(container.querySelector('[data-testid="vda5050-page-content"]')?.textContent).not.toContain("MQTT CONNECTED");
    expectCleanStaticPresentation();

    await selectSection("CONTROL");
    const estop = container.querySelector<HTMLButtonElement>(".hmi-estop");
    expect(estop?.disabled).toBe(true);
    await act(async () => {
      estop?.click();
      await new Promise((resolve) => window.setTimeout(resolve, 0));
    });
    expect(container.querySelector('[data-testid="global-notification-region"]')).toBeNull();
    expect(container.querySelector('[data-testid="global-notification-region"] [role="alert"]')).toBeNull();
    expectCleanStaticPresentation();
    expect(fetchSpy).not.toHaveBeenCalled();
    expect(ForbiddenWebSocket.attempts).toEqual([]);
    expect(vi.mocked(window.setInterval).mock.calls.some(([, delay]) => delay === 3000 || delay === 5000)).toBe(false);

    await selectSection("LOCALIZATION");
    expect(container.querySelector('[data-testid="localization-estop-state"]')).toBeNull();

    const { wsConnect, wsSend } = await import("../src/services/ws");
    useStore.setState({
      localRobotId: STATIC_ROBOT_ID,
      localIdentityVerified: true,
      connectedRobotIds: [STATIC_ROBOT_ID],
      rosConnected: true,
      websocketState: "CONNECTED",
    });
    wsConnect(() => undefined);
    expect(wsSend({ type: "NAV_CANCEL", robot_id: STATIC_ROBOT_ID })).toBe(false);
    expect(wsSend({ type: "NAV_GOAL", robot_id: STATIC_ROBOT_ID, x: 1, y: 1, yaw: 0 })).toBe(false);
    expect(wsSend({ type: "ROBOT_MANUAL", robot_id: STATIC_ROBOT_ID, action: "FORWARD" })).toBe(false);
    expect(ForbiddenWebSocket.attempts).toEqual([]);

    const { apiFetch } = await import("../src/services/api");
    const commandResponse = await apiFetch(`/api/robots/${STATIC_ROBOT_ID}/emergency-stop`, { method: "POST" });
    expect(commandResponse.status).toBe(409);
    expect(await commandResponse.json()).toMatchObject({ code: "STATIC_PREVIEW_ONLY" });
    expect(fetchSpy).not.toHaveBeenCalled();
  });

  it("keeps two-point destination and yaw selection local without a notification", async () => {
    const { occupancyRasters } = await import("../src/components/shared/maps/occupancyRaster");
    vi.spyOn(occupancyRasters, "get").mockResolvedValue(document.createElement("canvas"));
    const context = new Proxy({
      createImageData: (width: number, height: number) => ({ data: new Uint8ClampedArray(width * height * 4) }),
    } as Record<string, unknown>, {
      get(target, key) { return target[key as string] ?? (() => undefined); },
      set(target, key, value) { target[key as string] = value; return true; },
    }) as unknown as CanvasRenderingContext2D;
    vi.spyOn(HTMLCanvasElement.prototype, "getContext").mockReturnValue(context);
    vi.spyOn(HTMLCanvasElement.prototype, "getBoundingClientRect").mockReturnValue({
      x: 0, y: 0, left: 0, top: 0, right: 800, bottom: 600, width: 800, height: 600,
      toJSON: () => ({}),
    } as DOMRect);
    vi.stubGlobal("ResizeObserver", class {
      constructor(private readonly callback: ResizeObserverCallback) {}
      observe(target: Element) {
        Object.defineProperty(target, "clientWidth", { configurable: true, value: 800 });
        Object.defineProperty(target, "clientHeight", { configurable: true, value: 600 });
        this.callback([], this as unknown as ResizeObserver);
      }
      disconnect() {}
      unobserve() {}
    });

    await mountStaticApp();
    const canvas = container.querySelector<HTMLCanvasElement>('[aria-label="Active navigation map 2D view"]');
    expect(canvas).not.toBeNull();
    for (let attempt = 0; attempt < 10 && !canvas?.dataset.interactiveMapContentKey; attempt += 1) {
      await act(async () => { await new Promise((resolve) => window.setTimeout(resolve, 0)); });
    }
    expect(canvas?.dataset.interactiveMapContentKey).toBeTruthy();
    expect(canvas?.dataset.interactionMode).toBe("POINT");
    const toolbarButtons = Array.from(container.querySelectorAll<HTMLButtonElement>("[data-testid=robot-map-layers] button"));
    expect(toolbarButtons.map((button) => button.textContent?.trim())).toEqual(["PAN", "POINT", "LAYERS"]);
    expect(toolbarButtons.map((button) => button.querySelectorAll('svg[aria-hidden="true"]').length)).toEqual([1, 1, 1]);
    expect(toolbarButtons[0]?.getAttribute("aria-pressed")).toBe("false");
    expect(toolbarButtons[1]?.getAttribute("aria-pressed")).toBe("true");
    expect(toolbarButtons[2]?.getAttribute("aria-expanded")).toBe("false");
    expect(container.querySelector('[data-testid="map-pick-feedback"]')).toBeNull();

    const clickWorldPoint = async (x: number, y: number) => {
      const centerX = Number(canvas?.dataset.viewportCenterX);
      const centerY = Number(canvas?.dataset.viewportCenterY);
      const scale = Number(canvas?.dataset.viewportScalePxPerMeter);
      const clientX = 400 + (x - centerX) * scale;
      const clientY = 300 - (y - centerY) * scale;
      await act(async () => {
        canvas?.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX, clientY }));
      });
    };

    await clickWorldPoint(30.5, 17.5);
    expect(canvas?.dataset.pointSelectionState).toBe("SELECT_YAW");
    expect(container.querySelector('[data-testid="selected-point-status"]')?.textContent).toBe("SELECT YAW");
    expect(container.querySelector('[data-testid="map-pick-feedback"]')).toBeNull();
    await clickWorldPoint(32.5, 19.5);
    expect(container.querySelector('[data-testid="selected-point-status"]')?.textContent).toBe("POINT SELECTED");
    expect(container.querySelector<HTMLButtonElement>('[aria-label="SEND GOAL"]')?.disabled).toBe(true);
    expect(container.querySelector('[data-testid="global-notification-region"]')).toBeNull();
    expect(container.querySelector('[data-testid="global-notification-region"] [role="alert"]')).toBeNull();
    expectCleanStaticPresentation();
    expect(fetchSpy).not.toHaveBeenCalled();
    expect(ForbiddenWebSocket.attempts).toEqual([]);
  });

  it("allows STATIC_UI localization pose preview without any backend request or robot-state mutation", async () => {
    const { occupancyRasters } = await import("../src/components/shared/maps/occupancyRaster");
    vi.spyOn(occupancyRasters, "get").mockResolvedValue(document.createElement("canvas"));
    const context = new Proxy({
      createImageData: (width: number, height: number) => ({ data: new Uint8ClampedArray(width * height * 4) }),
    } as Record<string, unknown>, {
      get(target, key) { return target[key as string] ?? (() => undefined); },
      set(target, key, value) { target[key as string] = value; return true; },
    }) as unknown as CanvasRenderingContext2D;
    vi.spyOn(HTMLCanvasElement.prototype, "getContext").mockReturnValue(context);
    vi.spyOn(HTMLCanvasElement.prototype, "getBoundingClientRect").mockReturnValue({
      x: 0, y: 0, left: 0, top: 0, right: 800, bottom: 600, width: 800, height: 600,
      toJSON: () => ({}),
    } as DOMRect);
    vi.stubGlobal("ResizeObserver", class {
      constructor(private readonly callback: ResizeObserverCallback) {}
      observe(target: Element) {
        Object.defineProperty(target, "clientWidth", { configurable: true, value: 800 });
        Object.defineProperty(target, "clientHeight", { configurable: true, value: 600 });
        this.callback([], this as unknown as ResizeObserver);
      }
      disconnect() {}
      unobserve() {}
    });

    await mountStaticApp("/localization");
    const { STATIC_ROBOT } = await import("../src/mock/fixtures");
    const canvas = container.querySelector<HTMLCanvasElement>('[aria-label="Localization map and initial pose picker"]');
    expect(canvas).not.toBeNull();
    for (let attempt = 0; attempt < 10 && !canvas?.dataset.interactiveMapContentKey; attempt += 1) {
      await act(async () => { await new Promise((resolve) => window.setTimeout(resolve, 0)); });
    }
    expectCleanStaticPresentation();
    const clickWorldPoint = async (x: number, y: number) => {
      const centerX = Number(canvas?.dataset.viewportCenterX);
      const centerY = Number(canvas?.dataset.viewportCenterY);
      const scale = Number(canvas?.dataset.viewportScalePxPerMeter);
      const clientX = 400 + (x - centerX) * scale;
      const clientY = 300 - (y - centerY) * scale;
      await act(async () => { canvas?.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX, clientY })); });
    };
    await act(async () => { container.querySelector<HTMLButtonElement>('[data-testid="map-mode-point"]')?.click(); });
    await clickWorldPoint(50.5, 35.5);
    expect(canvas?.dataset.pointSelectionState).toBe("SELECT_YAW");
    await clickWorldPoint(52.5, 35.5);
    expect(container.querySelector<HTMLInputElement>('[aria-label="Initial pose X in meters"]')?.value).toBe("50.500");
    expect(container.querySelector<HTMLInputElement>('[aria-label="Initial pose yaw in degrees"]')?.value).toBe("0.000");
    const applyButton = container.querySelector<HTMLButtonElement>('[data-testid="set-initial-pose"]');
    expect(applyButton?.disabled).toBe(true);
    await act(async () => { applyButton?.click(); });
    expect(container.querySelector('[role="dialog"]')).toBeNull();
    expect(useStore.getState().twin?.robots?.[STATIC_ROBOT_ID]?.active_map_pose).toEqual(STATIC_ROBOT.active_map_pose);
    expect(fetchSpy).not.toHaveBeenCalled();
    expect(ForbiddenWebSocket.attempts).toEqual([]);
  });
});
