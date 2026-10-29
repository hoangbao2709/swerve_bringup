// @vitest-environment jsdom
import { act } from "react";
import { createRoot } from "react-dom/client";
import { afterEach, beforeEach, expect, it, vi } from "vitest";
import { useStore } from "../src/state/store";

const mocks = vi.hoisted(() => ({ identity: vi.fn(), connection: null as null | ((state: string) => void) }));
vi.mock("../src/services/api", () => ({ getLocalIdentity: mocks.identity }));
vi.mock("../src/services/ws", () => ({
  wsConnect: (callback: (state: string) => void) => { mocks.connection = callback; },
  wsDisconnect: vi.fn(), wsSend: vi.fn(),
}));
import { useBackendRealtime } from "../src/services/backendRuntime";
const initialState = useStore.getState();
let root: ReturnType<typeof createRoot>;
let container: HTMLDivElement;
function Harness() { useBackendRealtime(); return null; }
const identity = { deployment_mode: "SINGLE_LOCAL_ROBOT", robot_id: "AMR-17", configured: true, bridge_online: false };

beforeEach(() => {
  globalThis.IS_REACT_ACT_ENVIRONMENT = true;
  useStore.setState(initialState);
  mocks.identity.mockReset().mockResolvedValue(identity);
  container = document.createElement("div");
  root = createRoot(container);
});
afterEach(() => { act(() => root.unmount()); useStore.setState(initialState); });

it("resolves deployment identity without telemetry and prevents selection overrides", async () => {
  await act(async () => { root.render(<Harness />); });
  expect(useStore.getState().localRobotId).toBe("AMR-17");
  useStore.getState().select("R02");
  expect(useStore.getState().selectedRobot).toBe("AMR-17");
  useStore.getState().openRobotQuickDetail("R02");
  expect(useStore.getState().quickDetailRobotId).toBe("AMR-17");
});

it("pins identity through disconnect, foreign telemetry and reconnect", async () => {
  await act(async () => { root.render(<Harness />); });
  act(() => { mocks.connection!("offline"); });
  expect(useStore.getState().localIdentityVerified).toBe(false);
  useStore.setState({ twin: { ...initialState.twin, robots: { R02: { id: "R02" } as never } } });
  await act(async () => { mocks.connection!("online"); });
  expect(useStore.getState().localRobotId).toBe("AMR-17");
  expect(useStore.getState().localIdentityVerified).toBe(true);
});

it("fails closed when a restarted backend reports a different identity", async () => {
  await act(async () => { root.render(<Harness />); });
  mocks.identity.mockResolvedValue({ ...identity, robot_id: "R02" });
  await act(async () => { mocks.connection!("online"); });
  expect(useStore.getState().localRobotId).toBe("AMR-17");
  expect(useStore.getState().localIdentityVerified).toBe(false);
  expect(useStore.getState().localIdentityError).toContain("LOCAL_ROBOT_ID_CHANGED");
});

it("does not infer missing identity from browser selection or telemetry", async () => {
  mocks.identity.mockRejectedValue(new Error("LOCAL_ROBOT_ID_UNCONFIGURED"));
  useStore.setState({ selectedRobot: "R02", twin: { ...initialState.twin, robots: { R02: { id: "R02" } as never } } });
  await act(async () => { root.render(<Harness />); });
  expect(useStore.getState().localRobotId).toBeNull();
  expect(useStore.getState().localIdentityVerified).toBe(false);
});
