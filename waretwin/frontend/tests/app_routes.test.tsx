// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";

vi.mock("../src/services/backendRuntime", () => ({ useBackendRealtime: () => undefined }));
vi.mock("../src/components/control/RobotControlPage", () => ({ RobotControlPage: () => <div data-page="control" /> }));

import App from "../src/App";
import { useStore } from "../src/state/store";

let root: Root;
let container: HTMLDivElement;

function mount(path: string) {
  window.history.replaceState({}, "", path);
  act(() => root.render(<App />));
}

beforeEach(() => {
  globalThis.IS_REACT_ACT_ENVIRONMENT = true;
  container = document.createElement("div");
  document.body.appendChild(container);
  root = createRoot(container);
  useStore.setState({ selectedRobot: null });
});

afterEach(() => {
  act(() => root.unmount());
  container.remove();
  useStore.setState({ selectedRobot: null });
  vi.clearAllMocks();
});

describe("direct local application routes", () => {
  it("redirects / to Robot Control without rendering Overview", () => {
    mount("/");
    expect(window.location.pathname).toBe("/control");
    expect(container.querySelector('[data-page="control"]')).not.toBeNull();
    expect(container.textContent).not.toContain("Overview");
  });

  it("renders Robot Control at /control", () => {
    mount("/control");
    expect(container.querySelector('[data-page="control"]')).not.toBeNull();
  });

  it("redirects the compatibility robot route to /control and preserves selection", () => {
    mount("/robots/AMR-17/control");
    expect(window.location.pathname).toBe("/control");
    expect(useStore.getState().selectedRobot).toBe("AMR-17");
    expect(container.querySelector('[data-page="control"]')).not.toBeNull();
  });

  it.each(["/overview", "/login", "/register", "/account", "/profile", "/operations", "/unknown"]) (
    "redirects removed or unknown route %s to /control",
    (path) => {
    mount(path);
    expect(window.location.pathname).toBe("/control");
    expect(container.querySelector('[data-page="control"]')).not.toBeNull();
    },
  );
});
