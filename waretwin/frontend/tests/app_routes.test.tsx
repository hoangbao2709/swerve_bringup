// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import { useStore } from "../src/state/store";

vi.mock("../src/services/auth", () => ({ bootstrapAuth: vi.fn(async () => null) }));
vi.mock("../src/services/backendRuntime", () => ({ useBackendRealtime: () => undefined }));
vi.mock("../src/components/auth/AuthPage", () => ({ AuthPage: ({ mode }: { mode: string }) => <div data-page={`auth-${mode}`} /> }));
vi.mock("../src/components/overview/OverviewPage", () => ({ OverviewPage: () => <div data-page="overview" /> }));
vi.mock("../src/components/control/RobotControlPage", () => ({ RobotControlPage: () => <div data-page="control" /> }));
vi.mock("../src/components/control/RobotControlDetailPage", () => ({ RobotControlDetailPage: ({ robotId }: { robotId: string }) => <div data-page={`robot-${robotId}`} /> }));

import App from "../src/App";

const initialState = useStore.getState();
let root: Root;
let container: HTMLDivElement;

function mount(path: string, authStatus: "guest" | "authenticated", username: string | null = null) {
  window.history.replaceState({}, "", path);
  useStore.setState({
    ...initialState,
    authStatus,
    authToken: authStatus === "authenticated" ? "test-token" : null,
    authUser: username ? { id: 1, username, email: `${username}@example.test`, role: "user", is_active: true } : null,
  });
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
  useStore.setState(initialState);
  vi.clearAllMocks();
});

describe("retained authenticated routes", () => {
  it("keeps Login available at /login", () => {
    mount("/login", "guest");
    expect(window.location.pathname).toBe("/login");
    expect(container.querySelector('[data-page="auth-login"]')).not.toBeNull();
  });

  it("redirects an unauthenticated root request to login", () => {
    mount("/", "guest");
    expect(window.location.pathname).toBe("/login");
    expect(container.querySelector('[data-page="auth-login"]')).not.toBeNull();
  });

  it("renders the original overview at / after authentication", () => {
    mount("/", "authenticated", "operator");
    expect(container.querySelector('[data-page="overview"]')).not.toBeNull();
  });

  it("renders Robot Control at /control", () => {
    mount("/control", "authenticated", "operator");
    expect(container.querySelector('[data-page="control"]')).not.toBeNull();
  });

  it("passes a dynamic robot id to the detail route", () => {
    mount("/robots/AMR-17/control", "authenticated", "operator");
    expect(container.querySelector('[data-page="robot-AMR-17"]')).not.toBeNull();
  });

  it("redirects removed or unknown authenticated routes to /", () => {
    mount("/operations", "authenticated", "operator");
    expect(window.location.pathname).toBe("/");
    expect(container.querySelector('[data-page="overview"]')).not.toBeNull();
  });
});
