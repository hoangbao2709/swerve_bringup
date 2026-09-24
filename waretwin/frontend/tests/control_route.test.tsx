// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import type { RobotState, TwinState } from "../src/schema/twin_state";
import { useStore } from "../src/state/store";
import { FLOOR_ELEV } from "../src/components/scene/Mezzanine";
import { resolveBackendUrls } from "../src/services/ws";

vi.mock("../src/components/views/MapView2D", () => ({
  MapView2D: () => <div data-testid="warehouse-map">Warehouse map</div>,
}));
vi.mock("../src/services/api", () => ({
  navigationApi: () => new Promise(() => undefined),
  startTagMission: vi.fn(),
  missionAction: vi.fn(),
  emergencyStop: vi.fn(),
  clearEmergencyStop: vi.fn(),
}));

import { RobotControlPage } from "../src/components/control/RobotControlPage";

const initialState = useStore.getState();
let root: Root;
let container: HTMLDivElement;

function r01(): RobotState {
  return {
    id: "R01", model: "AMR-L", floor: 1, lift_id: null, lift_stage: null,
    position: [40, 0, 64], heading: 0, velocity: 0, max_speed: 1, battery: 70,
    status: "IDLE", fsm: "IDLE", health: 100, current_task_id: null, destination: null,
    path: [], path_index: 0, load: { current: 0, capacity: 1 }, zone: null, eta_s: null,
    fsm_since_tick: 0, stats: { distance_m: 0, tasks_completed: 0, energy_wh: 0, busy_ticks: 0, wait_ticks: 0 },
    perception: { state: "CLEAR", ahead_m: 4, nearest_m: null, obstacles: [] }, control_mode: "AUTONOMOUS",
  };
}

function renderControl(overrides: Partial<ReturnType<typeof useStore.getState>> = {}) {
  act(() => {
    useStore.setState({ ...initialState, ...overrides });
    root.render(<RobotControlPage />);
  });
}

beforeEach(() => {
  globalThis.IS_REACT_ACT_ENVIRONMENT = true;
  window.history.replaceState({}, "", "/control");
  useStore.setState(initialState);
  container = document.createElement("div");
  document.body.appendChild(container);
  root = createRoot(container);
});

afterEach(() => {
  act(() => root.unmount());
  container.remove();
  useStore.setState(initialState);
  vi.restoreAllMocks();
});

describe("/control direct render", () => {
  it("renders sidebar, header, map, mission and disabled controls from an empty disconnected store", () => {
    renderControl({
      twin: null as unknown as TwinState,
      tagGraph: null,
      rosDiagnostics: null,
      tagNavigation: null,
      rosConnected: false,
      websocketState: "DISCONNECTED",
    });
    expect(container.textContent).toContain("WareTwin");
    expect(container.textContent).toContain("Robot Control");
    expect(container.textContent).toContain("Warehouse map");
    expect(container.textContent).toContain("MISSION");
    expect(container.textContent).toContain("ROBOT CONTROL");
    expect(container.textContent).toContain("DISCONNECTED");
    expect(container.querySelector("button.manual-btn")?.hasAttribute("disabled")).toBe(true);
  });

  it("renders R01 when ROS and WebSocket are online", () => {
    renderControl({
      twin: { ...initialState.twin, robots: { R01: r01() } },
      rosConnected: true,
      websocketState: "CONNECTED",
      tagGraph: { warehouse_id: 1, tags: [], edges: [] },
    });
    expect(container.textContent).toContain("R01");
    expect(container.textContent).toContain("CONNECTED");
  });

  it("accepts repeated store updates without creating a snapshot render loop", () => {
    const consoleError = vi.spyOn(console, "error").mockImplementation(() => undefined);
    renderControl({ twin: null as unknown as TwinState, tagGraph: null, rosDiagnostics: null, rosConnected: false, websocketState: "RECONNECTING" });
    for (let tick = 0; tick < 16; tick += 1) {
      act(() => useStore.setState({
        twin: { ...initialState.twin, sim: { ...initialState.twin.sim, tick }, robots: tick % 2 ? { R01: r01() } : {} },
        tagGraph: null,
        rosDiagnostics: null,
        tagNavigation: null,
      }));
    }
    expect(container.textContent).toContain("Robot Control");
    expect(consoleError.mock.calls.flat().join(" ")).not.toContain("getSnapshot should be cached");
    expect(consoleError.mock.calls.flat().join(" ")).not.toContain("Maximum update depth exceeded");
  });
});

describe("control runtime configuration guards", () => {
  it("derives Channels from the configured REST host and port", () => {
    expect(resolveBackendUrls({ VITE_API_BASE_URL: "http://localhost:8001" }, { protocol: "http:", hostname: "localhost" })).toEqual({
      apiUrl: "http://localhost:8001",
      wsUrl: "ws://localhost:8001/ws",
    });
  });

  it("allows React Refresh to read Symbol properties from FLOOR_ELEV", () => {
    expect(() => Reflect.get(FLOOR_ELEV, Symbol.toStringTag)).not.toThrow();
    expect(() => Reflect.get(FLOOR_ELEV, Symbol.for("react.refresh"))).not.toThrow();
  });
});
