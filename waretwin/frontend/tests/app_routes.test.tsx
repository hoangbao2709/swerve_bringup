// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";

vi.mock("../src/services/backendRuntime", () => ({ useBackendRealtime: () => undefined }));
vi.mock("../src/components/overview/OverviewPage", () => ({ OverviewPage: () => <div data-page="overview" /> }));
vi.mock("../src/components/control/RobotControlPage", () => ({ RobotControlPage: () => <div data-page="control" /> }));
vi.mock("../src/components/control/RobotControlDetailPage", () => ({ RobotControlDetailPage: ({ robotId }: { robotId: string }) => <div data-page={`robot-${robotId}`} /> }));

import App from "../src/App";

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
});

afterEach(() => {
  act(() => root.unmount());
  container.remove();
  vi.clearAllMocks();
});

describe("direct local application routes", () => {
  it("opens the original Overview directly at / without a user session", () => {
    mount("/");
    expect(window.location.pathname).toBe("/");
    expect(container.querySelector('[data-page="overview"]')).not.toBeNull();
  });

  it("renders Robot Control at /control", () => {
    mount("/control");
    expect(container.querySelector('[data-page="control"]')).not.toBeNull();
  });

  it("passes a dynamic robot id to the detail route", () => {
    mount("/robots/AMR-17/control");
    expect(container.querySelector('[data-page="robot-AMR-17"]')).not.toBeNull();
  });

  it.each(["/login", "/register", "/account", "/profile", "/operations", "/unknown"]) (
    "redirects removed or unknown route %s to /",
    (path) => {
    mount(path);
    expect(window.location.pathname).toBe("/");
    expect(container.querySelector('[data-page="overview"]')).not.toBeNull();
    },
  );
});
