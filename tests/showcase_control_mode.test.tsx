// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";

const wsMocks = vi.hoisted(() => ({
  wsSetRobotMode: vi.fn(() => true),
  wsSend: vi.fn(() => true),
  wsManualCommand: vi.fn(() => true),
  wsConnect: vi.fn(),
  wsDisconnect: vi.fn(),
}));

vi.mock("../src/config", async (importOriginal) => {
  const config = await importOriginal<typeof import("../src/config")>();
  return { ...config, UI_MODE: "SHOWCASE", IS_SHOWCASE: true, IS_STATIC_UI: false, IS_STANDALONE_UI: true };
});
vi.mock("../src/services/ws", async (importOriginal) => ({
  ...await importOriginal<typeof import("../src/services/ws")>(),
  ...wsMocks,
}));

import App from "../src/App";
import { navigateLocalPath } from "../src/routing/localRoutes";
import { useStore } from "../src/state/store";

let root: Root;
let container: HTMLDivElement;
const initialStoreState = useStore.getState();

function headerModeButton(mode: "MANUAL" | "AUTONOMOUS"): HTMLButtonElement {
  const buttons = Array.from(container.querySelectorAll<HTMLButtonElement>(
    ".local-app-header [aria-label='Robot control actions'] button",
  ));
  const match = buttons.find((item) => item.textContent?.trim() === mode);
  if (!match) throw new Error(`Header ${mode} button was not found: ${container.textContent?.slice(0, 500)}`);
  return match;
}

async function settle() {
  await act(async () => { await new Promise((resolve) => window.setTimeout(resolve, 20)); });
}

async function visit(path: string) {
  act(() => { navigateLocalPath(path); });
  await settle();
}

beforeEach(async () => {
  globalThis.IS_REACT_ACT_ENVIRONMENT = true;
  window.history.replaceState({}, "", "/control");
  Object.defineProperty(window, "innerWidth", { configurable: true, value: 1440 });
  useStore.setState({ ...initialStoreState, showcaseControlMode: "AUTONOMOUS", robotDetail: {} });
  container = document.createElement("div");
  document.body.appendChild(container);
  root = createRoot(container);
  vi.stubGlobal("fetch", vi.fn());
  vi.clearAllMocks();
  vi.spyOn(HTMLCanvasElement.prototype, "getContext").mockImplementation(() => null as never);
  await import("../src/mock/StaticUiApp");
  await act(async () => {
    root.render(<App />);
    for (let attempt = 0; attempt < 20 && !container.querySelector(".local-app-header"); attempt += 1) {
      await new Promise((resolve) => window.setTimeout(resolve, 10));
    }
  });
});

afterEach(() => {
  act(() => root.unmount());
  container.remove();
  useStore.setState(initialStoreState);
  vi.unstubAllGlobals();
  vi.restoreAllMocks();
  vi.clearAllMocks();
});

describe("standalone showcase control mode", () => {
  it("starts AUTONOMOUS and synchronizes immediate local selection across header, control, settings, and routes", async () => {
    expect(useStore.getState().showcaseControlMode).toBe("AUTONOMOUS");
    expect(headerModeButton("AUTONOMOUS").getAttribute("aria-pressed")).toBe("true");
    expect(headerModeButton("MANUAL").disabled).toBe(false);
    expect(headerModeButton("AUTONOMOUS").disabled).toBe(false);
    expect(container.querySelector("[data-testid='control-operation-panels']")?.getAttribute("data-control-mode"))
      .toBe("AUTONOMOUS");
    expect(container.querySelector("[aria-label='Robot operation panel']")).not.toBeNull();
    expect(container.querySelector("[data-testid='manual-jog-panel']")).toBeNull();

    act(() => headerModeButton("MANUAL").click());
    expect(useStore.getState().showcaseControlMode).toBe("MANUAL");
    expect(headerModeButton("MANUAL").getAttribute("aria-pressed")).toBe("true");
    expect(headerModeButton("AUTONOMOUS").getAttribute("aria-pressed")).toBe("false");
    expect(container.querySelector("[data-testid='control-operation-panels']")?.getAttribute("data-control-mode"))
      .toBe("MANUAL");
    expect(container.querySelector("[data-testid='manual-jog-panel']")).not.toBeNull();
    expect(container.querySelector("[aria-label='Robot operation panel']")).toBeNull();

    act(() => headerModeButton("AUTONOMOUS").click());
    expect(container.querySelector("[aria-label='Robot operation panel']")).not.toBeNull();
    expect(container.querySelector("[data-testid='manual-jog-panel']")).toBeNull();
    act(() => headerModeButton("MANUAL").click());
    act(() => headerModeButton("AUTONOMOUS").click());
    expect(useStore.getState().showcaseControlMode).toBe("AUTONOMOUS");

    await visit("/settings");
    const operationModeTab = container.querySelector<HTMLButtonElement>("button[aria-label='Operation Mode']");
    expect(operationModeTab).not.toBeNull();
    act(() => operationModeTab?.click());
    expect(container.querySelector("[data-settings-section='Operation Mode']")?.textContent).toContain("AUTONOMOUS");
    const settingsManual = container.querySelector<HTMLButtonElement>(
      "[data-settings-section='Operation Mode'] button[aria-pressed][type='button']",
    );
    const settingsButtons = Array.from(container.querySelectorAll<HTMLButtonElement>(
      "[data-settings-section='Operation Mode'] button[aria-pressed]",
    ));
    expect(settingsButtons.map((button) => button.textContent?.replace(/\s+/g, " ").trim()))
      .toEqual(["MANUALOperator motion", "AUTONOMOUSAutonomy"]);
    expect(settingsManual?.disabled).toBe(false);
    act(() => settingsManual?.click());
    expect(container.querySelector("[data-settings-section='Operation Mode'] [aria-pressed='true']")?.textContent)
      .toContain("MANUAL");
    expect(headerModeButton("MANUAL").getAttribute("aria-pressed")).toBe("true");
    expect(container.querySelector("[role='dialog']")).toBeNull();

    await visit("/mapping");
    await visit("/control");
    expect(container.querySelector("[data-testid='control-operation-panels']")?.getAttribute("data-control-mode"))
      .toBe("MANUAL");
    expect(container.querySelector("[data-testid='manual-jog-panel']")).not.toBeNull();
    act(() => headerModeButton("AUTONOMOUS").click());
    expect(container.querySelector("[aria-label='Robot operation panel']")).not.toBeNull();
    expect(container.querySelector("[data-testid='manual-jog-panel']")).toBeNull();

    expect(wsMocks.wsSetRobotMode).not.toHaveBeenCalled();
    expect(wsMocks.wsSend).not.toHaveBeenCalled();
    expect(wsMocks.wsManualCommand).not.toHaveBeenCalled();
    expect(wsMocks.wsConnect).not.toHaveBeenCalled();
    expect(wsMocks.wsDisconnect).not.toHaveBeenCalled();
    expect(fetch).not.toHaveBeenCalled();
  });
});
