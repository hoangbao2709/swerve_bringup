// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import { useStore } from "../src/state/store";

vi.mock("../src/services/api", () => ({
  getLocalIdentity: vi.fn(async () => ({ robot_id: "R01", mode: "SINGLE_LOCAL_ROBOT" })),
}));
vi.mock("../src/services/ws", () => ({
  wsConnect: vi.fn(),
  wsDisconnect: vi.fn(),
  wsSend: vi.fn(() => false),
}));

vi.mock("../src/pages/control/ControlPage", () => ({ ControlPage: () => <main data-page="CONTROL" /> }));
vi.mock("../src/pages/mapping/MappingPage", () => ({ MappingPage: () => <main data-page="MAPPING" /> }));
vi.mock("../src/pages/localization/LocalizationPage", () => ({ LocalizationPage: () => <main data-page="LOCALIZATION" /> }));
vi.mock("../src/pages/vda5050/Vda5050Page", () => ({ Vda5050Page: () => <main data-page="VDA5050" /> }));
vi.mock("../src/pages/diagnosis/DiagnosisPage", () => ({ DiagnosisPage: () => <main data-page="DIAGNOSIS" /> }));
vi.mock("../src/pages/system/SystemOverviewPage", () => ({ SystemOverviewPage: () => <main data-page="SYSTEM" /> }));
vi.mock("../src/pages/settings/WifiSettingsPage", () => ({ WifiSettingsPage: ({ active = true }: { active?: boolean }) => <section id="settings-wifi" hidden={!active} data-setting="Wi-Fi" /> }));
vi.mock("../src/pages/settings/VolumeSettingsPage", () => ({ VolumeSettingsPage: ({ active = true }: { active?: boolean }) => <section id="settings-volume" hidden={!active} data-setting="Volume" /> }));
vi.mock("../src/pages/settings/NtpSettingsPage", () => ({ NtpSettingsPage: ({ active = true }: { active?: boolean }) => <section id="settings-ntp" hidden={!active} data-setting="NTP" /> }));
vi.mock("../src/pages/settings/OperationModeSettingsPage", () => ({ OperationModeSettingsPage: ({ active = true }: { active?: boolean }) => <section id="settings-operation-mode" hidden={!active} data-setting="Operation Mode" /> }));
vi.mock("../src/pages/settings/FactorySettingsPage", () => ({ FactorySettingsPage: ({ active = true }: { active?: boolean }) => <section id="settings-factory" hidden={!active} data-setting="Factory Setting" /> }));

import App from "../src/App";
import { wsConnect, wsDisconnect } from "../src/services/ws";

let root: Root;
let container: HTMLDivElement;

function mount(path: string) {
  window.history.replaceState({}, "", path);
  act(() => root.render(<App />));
}

function clickRoute(path: string) {
  if (path.startsWith("/settings/")) {
    if (!container.querySelector("main[aria-label='Settings workspace']")) {
      const settingsLink = container.querySelector<HTMLAnchorElement>(".wt-sidebar-item[href='/settings']");
      act(() => settingsLink?.click());
    }
    if (path === "/settings/map") {
      const mapLink = container.querySelector<HTMLAnchorElement>("a.settings-map-link");
      expect(mapLink).not.toBeNull();
      act(() => mapLink?.click());
      return;
    }
    const labelByPath: Record<string, string> = {
      "/settings/wifi": "Wi-Fi",
      "/settings/volume": "Volume",
      "/settings/ntp": "NTP",
      "/settings/operation-mode": "Operation Mode",
      "/settings/factory": "Factory Setting",
    };
    const sectionButton = container.querySelector<HTMLButtonElement>(`button[aria-label='${labelByPath[path]}']`);
    expect(sectionButton).not.toBeNull();
    act(() => sectionButton?.click());
    return;
  }
  const link = container.querySelector<HTMLAnchorElement>(`.wt-sidebar-item[href="${path}"]`);
  expect(link).not.toBeNull();
  act(() => link?.click());
}

function expectActiveRoute(path: string, section: string) {
  expect(window.location.pathname).toBe(path);
  expect(section === "SETTINGS"
    ? container.querySelector("main[aria-label='Settings workspace']")
    : container.querySelector(`[data-page="${section}"]`)).not.toBeNull();
  if (section === "SETTINGS") {
    const activeSection = path === "/settings" ? "Wi-Fi" : ({
      "/settings/wifi": "Wi-Fi",
      "/settings/volume": "Volume",
      "/settings/ntp": "NTP",
      "/settings/operation-mode": "Operation Mode",
      "/settings/factory": "Factory Setting",
    } as Record<string, string>)[path];
    expect(container.querySelector(`button[aria-label="${activeSection}"][aria-current="location"]`)).not.toBeNull();
  }
  const active = container.querySelector<HTMLAnchorElement>(".wt-sidebar-item[aria-current='page']");
  expect(active?.getAttribute("href")).toBe(path.startsWith("/settings") ? "/settings" : path);
  expect(container.querySelectorAll(".wt-sidebar-item[aria-current='page']")).toHaveLength(1);
}

function waitForPopstatePath(path: string): Promise<void> {
  if (window.location.pathname === path) return Promise.resolve();
  return new Promise((resolve) => {
    const timeout = window.setTimeout(() => {
      window.removeEventListener("popstate", checkPath);
      resolve();
    }, 100);
    const checkPath = () => {
      if (window.location.pathname !== path) return;
      window.clearTimeout(timeout);
      window.removeEventListener("popstate", checkPath);
      resolve();
    };
    window.addEventListener("popstate", checkPath);
  });
}

beforeEach(() => {
  globalThis.IS_REACT_ACT_ENVIRONMENT = true;
  container = document.createElement("div");
  document.body.appendChild(container);
  root = createRoot(container);
  vi.mocked(wsConnect).mockClear();
  vi.mocked(wsDisconnect).mockClear();
  useStore.setState({ localRobotId: "R01", localIdentityError: null, selectedRobot: null });
});

afterEach(() => {
  act(() => root.unmount());
  container.remove();
  useStore.setState({ localRobotId: null, localIdentityError: null, selectedRobot: null });
  vi.clearAllMocks();
});

describe("single-robot local SPA routes", () => {
  it("replaces the root URL with CONTROL", () => {
    mount("/");
    expectActiveRoute("/control", "CONTROL");
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
    ["/settings/map", "MAPPING"],
  ])("opens %s directly and selects %s", (path, section) => {
    mount(path);
    expectActiveRoute(path, section);
  });

  it("uses real accessible links and changes the route without reloading", () => {
    mount("/control");
    const links = Array.from(container.querySelectorAll<HTMLAnchorElement>(".wt-sidebar-item"));
    expect(links.map((link) => link.getAttribute("href"))).toEqual([
      "/control", "/mapping", "/localization", "/system", "/diagnosis", "/vda5050", "/settings",
    ]);
    expect(links.every((link) => link.tagName === "A" && link.getAttribute("role") === null)).toBe(true);
    const navigation = container.querySelector(".wt-sidebar-nav");
    expect(Array.from(navigation?.querySelectorAll("section[aria-label$='navigation'] h2") ?? [])
      .map((heading) => heading.textContent?.trim())).toEqual(["ROBOT OPERATION", "MONITORING", "INTEGRATION"]);
    expect(navigation?.querySelectorAll("a[aria-current='page']")).toHaveLength(1);

    clickRoute("/mapping");
    expectActiveRoute("/mapping", "MAPPING");
    expect(container.querySelector(".local-app-header-title")?.textContent).toBe("MAPPING");
    expect(container.querySelector(".local-app-header")?.textContent).not.toMatch(/R01|ROBOT/i);
  });

  it("renders the reference transform underline for active and hovered navigation rows", () => {
    mount("/control");
    const controlLink = container.querySelector<HTMLAnchorElement>(".wt-sidebar-item[href='/control']");
    const mappingLink = container.querySelector<HTMLAnchorElement>(".wt-sidebar-item[href='/mapping']");
    const controlUnderline = controlLink?.querySelector<HTMLElement>("span[aria-hidden='true']");
    const mappingUnderline = mappingLink?.querySelector<HTMLElement>("span[aria-hidden='true']");

    expect(controlLink?.classList.contains("relative")).toBe(true);
    expect(controlLink?.classList.contains("group")).toBe(true);
    expect(controlLink?.classList.contains("overflow-hidden")).toBe(true);
    expect(controlUnderline?.className).toContain("bottom-0 left-0 h-[3px] w-full origin-left");
    expect(controlUnderline?.className).toContain("transition-transform duration-400 ease-linear");
    expect(controlUnderline?.className).toContain("motion-reduce:transition-none");
    expect(controlUnderline?.className).toContain("origin-left bg-red-500");
    expect(controlUnderline?.className).toContain("scale-x-100");
    expect(controlUnderline?.className).not.toContain("opacity-");
    expect(mappingUnderline?.className).toContain("scale-x-0 group-hover:scale-x-100 group-focus-visible:scale-x-100");
    expect(mappingUnderline?.className).not.toContain("opacity-");

    clickRoute("/settings");
    const settingsUnderline = container.querySelector<HTMLAnchorElement>(".wt-sidebar-item[href='/settings']")
      ?.querySelector<HTMLElement>("span[aria-hidden='true']");
    expect(settingsUnderline?.className).toContain("scale-x-100");
    expect(container.querySelectorAll(".wt-sidebar-item[aria-current='page']")).toHaveLength(1);
  });

  it("keeps all five settings sections in one workspace and routes its internal buttons", () => {
    mount("/settings");
    expect(container.querySelectorAll("main[aria-label='Settings workspace']")).toHaveLength(1);
    expect(Array.from(container.querySelectorAll("[data-setting]")).map((section) => section.getAttribute("data-setting")))
      .toEqual(["Wi-Fi", "Volume", "NTP", "Operation Mode", "Factory Setting"]);
    expect(container.querySelectorAll("[data-setting]:not([hidden])")).toHaveLength(1);
    const originalScrollIntoView = HTMLElement.prototype.scrollIntoView;
    const scrollIntoView = vi.fn();
    HTMLElement.prototype.scrollIntoView = scrollIntoView;
    try {
      const ntpButton = container.querySelector<HTMLButtonElement>('button[aria-label="NTP"]');
      expect(ntpButton?.getAttribute("aria-current")).toBeNull();
      act(() => ntpButton?.click());
      expect(window.location.pathname).toBe("/settings/ntp");
      expect(container.querySelector<HTMLButtonElement>('button[aria-label="NTP"]')?.getAttribute("aria-current")).toBe("location");
      expect(scrollIntoView).toHaveBeenCalledWith({ behavior: "smooth", block: "start" });
    } finally {
      if (originalScrollIntoView) HTMLElement.prototype.scrollIntoView = originalScrollIntoView;
      else delete HTMLElement.prototype.scrollIntoView;
    }
  });

  it("renders only one Settings sidebar item and routes settings tabs and Map from the workspace", () => {
    mount("/control");
    const settingsLink = container.querySelector<HTMLAnchorElement>(".wt-sidebar-item[href='/settings']");
    expect(settingsLink).not.toBeNull();
    expect(container.querySelectorAll(".wt-sidebar-item[href^='/settings']")).toHaveLength(1);
    expect(container.querySelector("button[aria-label='Toggle Settings pages']")).toBeNull();
    expect(container.querySelector(".wt-sidebar-subitem, #ptagv-setting-links")).toBeNull();
    expect(window.location.pathname).toBe("/control");

    clickRoute("/settings/wifi");
    expectActiveRoute("/settings/wifi", "SETTINGS");
    expect(container.querySelector(".wt-sidebar-item[href='/settings'][aria-current='page']")).not.toBeNull();
    expect(container.querySelectorAll(".wt-sidebar-item[aria-current='page']")).toHaveLength(1);

    clickRoute("/settings/map");
    expectActiveRoute("/settings/map", "MAPPING");
    expect(container.querySelector(".wt-sidebar-item[href='/settings'][aria-current='page']")).not.toBeNull();
    expect(container.querySelector(".wt-sidebar-item[href='/mapping'][aria-current='page']")).toBeNull();
    expect(container.querySelectorAll(".wt-sidebar-item[aria-current='page']")).toHaveLength(1);
  });

  it("updates pages with browser Back and Forward", async () => {
    mount("/control");
    clickRoute("/mapping");
    expectActiveRoute("/mapping", "MAPPING");

    await act(async () => {
      window.history.back();
      await waitForPopstatePath("/control");
    });
    expectActiveRoute("/control", "CONTROL");

    await act(async () => {
      window.history.forward();
      await waitForPopstatePath("/mapping");
    });
    expectActiveRoute("/mapping", "MAPPING");

    clickRoute("/settings");
    clickRoute("/settings/ntp");
    await act(async () => {
      window.history.back();
      await waitForPopstatePath("/settings");
    });
    expectActiveRoute("/settings", "SETTINGS");
    await act(async () => {
      window.history.forward();
      await waitForPopstatePath("/settings/ntp");
    });
    expectActiveRoute("/settings/ntp", "SETTINGS");
  });

  it("keeps a deep link after mounting a fresh application root", () => {
    mount("/diagnosis");
    act(() => root.unmount());
    root = createRoot(container);
    act(() => root.render(<App />));
    expectActiveRoute("/diagnosis", "DIAGNOSIS");
  });

  it("renders a clear not-found fallback without accepting robot-ID routes", () => {
    mount("/robots/R01/control");
    expect(window.location.pathname).toBe("/robots/R01/control");
    expect(container.querySelector("[role='alert']")?.textContent).toContain("PAGE NOT FOUND");
    expect(container.textContent).not.toContain("R01");
    expect(container.querySelector("[data-page]")).toBeNull();

    const controlLink = container.querySelector<HTMLAnchorElement>("a[href='/control']");
    act(() => controlLink?.click());
    expectActiveRoute("/control", "CONTROL");
  });

  it("keeps exactly one backend WebSocket connection lifecycle across route changes", () => {
    mount("/control");
    clickRoute("/mapping");
    clickRoute("/diagnosis");
    expect(wsConnect).toHaveBeenCalledTimes(1);
    expect(wsDisconnect).not.toHaveBeenCalled();
  });
});
