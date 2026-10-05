// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import { useStore } from "../src/state/store";

vi.mock("../src/services/backendRealtime", () => ({ useBackendRealtime: () => undefined }));
vi.mock("../src/components/control/RobotControlPage", () => ({
  RobotControlPage: () => <main data-testid="control-overview">Robot Control Overview</main>,
}));
vi.mock("../src/components/control/RobotControlDetailPage", () => ({
  RobotControlDetailPage: ({ robotId }: { robotId: string }) => <main data-testid="robot-detail">Robot detail · {robotId}</main>,
}));

import App from "../src/App";

const USER = { id: 7, username: "operator", email: "operator@example.test", role: "user" as const };
const SESSION_KEY = "waretwin.auth";
let root: Root;
let container: HTMLDivElement;
let fetchMock: ReturnType<typeof vi.fn>;

function jsonResponse(body: unknown, status = 200) {
  return new Response(JSON.stringify(body), { status, headers: { "content-type": "application/json" } });
}

function setPath(path: string) {
  window.history.replaceState({}, "", path);
  window.dispatchEvent(new PopStateEvent("popstate"));
}

async function renderApp(path: string) {
  setPath(path);
  await act(async () => {
    root.render(<App />);
    await Promise.resolve();
  });
  await act(async () => {
    await new Promise((resolve) => window.setTimeout(resolve, 0));
  });
}

function changeInput(input: HTMLInputElement, value: string) {
  const setter = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value")?.set;
  setter?.call(input, value);
  input.dispatchEvent(new Event("input", { bubbles: true }));
}

beforeEach(() => {
  globalThis.IS_REACT_ACT_ENVIRONMENT = true;
  localStorage.clear();
  useStore.setState({ authStatus: "loading", authToken: null, authUser: null });
  fetchMock = vi.fn(async (input: RequestInfo | URL) => {
    const url = String(input);
    if (url.endsWith("/api/auth/me")) return jsonResponse(USER);
    if (url.endsWith("/api/auth/login")) return jsonResponse({ access_token: "access-1", token_type: "bearer", user: USER });
    return jsonResponse({ detail: "Not found" }, 404);
  });
  vi.stubGlobal("fetch", fetchMock);
  container = document.createElement("div");
  document.body.appendChild(container);
  root = createRoot(container);
});

afterEach(() => {
  act(() => root.unmount());
  container.remove();
  localStorage.clear();
  useStore.setState({ authStatus: "loading", authToken: null, authUser: null });
  vi.unstubAllGlobals();
  vi.restoreAllMocks();
});

describe("application routes and authentication", () => {
  it("sends an unauthenticated root and protected robot URL to Login", async () => {
    await renderApp("/");
    expect(window.location.pathname).toBe("/login");
    expect(container.querySelector("form.auth-form")).not.toBeNull();

    await renderApp("/robots/robot-A/control");
    expect(window.location.pathname).toBe("/login");
  });

  it("logs in through the backend and opens the control overview", async () => {
    await renderApp("/login");
    const username = container.querySelector<HTMLInputElement>('input[autocomplete="username"]')!;
    const password = container.querySelector<HTMLInputElement>('input[autocomplete="current-password"]')!;
    await act(async () => {
      changeInput(username, " operator ");
      changeInput(password, "secret");
      container.querySelector<HTMLButtonElement>('button[type="submit"]')!.click();
      await new Promise((resolve) => window.setTimeout(resolve, 0));
    });

    expect(fetchMock).toHaveBeenCalledWith(expect.stringContaining("/api/auth/login"), expect.objectContaining({ method: "POST" }));
    expect(useStore.getState().authToken).toBe("access-1");
    expect(window.location.pathname).toBe("/control");
    expect(container.querySelector('[data-testid="control-overview"]')?.textContent).toContain("Robot Control Overview");
  });

  it("shows a backend login error and does not authenticate when the service is unavailable", async () => {
    fetchMock.mockRejectedValueOnce(new Error("Backend unavailable"));
    await renderApp("/login");
    const username = container.querySelector<HTMLInputElement>('input[autocomplete="username"]')!;
    const password = container.querySelector<HTMLInputElement>('input[autocomplete="current-password"]')!;
    await act(async () => {
      changeInput(username, "operator");
      changeInput(password, "secret");
      container.querySelector<HTMLButtonElement>('button[type="submit"]')!.click();
      await new Promise((resolve) => window.setTimeout(resolve, 0));
    });

    expect(container.textContent).toContain("Backend unavailable");
    expect(useStore.getState().authStatus).toBe("guest");
    expect(window.location.pathname).toBe("/login");
  });

  it("routes authenticated root and unknown paths to Control and preserves dynamic robot IDs", async () => {
    localStorage.setItem(SESSION_KEY, JSON.stringify({ token: "stored-access", user: USER }));
    await renderApp("/");
    expect(window.location.pathname).toBe("/control");
    expect(container.querySelector('[data-testid="control-overview"]')).not.toBeNull();

    await renderApp("/admin/warehouse-editor");
    expect(window.location.pathname).toBe("/control");

    await renderApp("/robots/robot-B-12/control");
    expect(window.location.pathname).toBe("/robots/robot-B-12/control");
    expect(container.querySelector('[data-testid="robot-detail"]')?.textContent).toContain("robot-B-12");
  });

  it("redirects an invalid stored session to Login", async () => {
    localStorage.setItem(SESSION_KEY, JSON.stringify({ token: "expired", user: USER }));
    fetchMock.mockImplementationOnce(async () => jsonResponse({ detail: "Invalid token" }, 401));
    await renderApp("/control");
    expect(window.location.pathname).toBe("/login");
    expect(useStore.getState().authStatus).toBe("guest");
  });
});
