// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, type ReactNode } from "react";
import { createRoot, type Root } from "react-dom/client";
import type { RobotState } from "../src/schema/twin_state";
import { useStore } from "../src/state/store";

vi.mock("../src/services/api", () => ({
  apiFetch: vi.fn(() => Promise.resolve({ ok: false, status: 503 })),
  emergencyStop: vi.fn(() => Promise.resolve({ ok: true })),
  clearEmergencyStop: vi.fn(() => Promise.resolve({ ok: true })),
}));
vi.mock("../src/simulation/runner", () => ({ useSimulationRunner: () => undefined }));

import { RobotQuickDetailModal } from "../src/components/robot/RobotQuickDetailModal";
import { RobotControlDetailPage } from "../src/components/control/RobotControlDetailPage";

const initialState = useStore.getState();
let root: Root;
let container: HTMLDivElement;

function r01(): RobotState {
  return {
    id: "R01", model: "AMR-L", floor: 1, lift_id: null, lift_stage: null,
    position: [40, 0, 64], heading: 0.2, velocity: 0.1, max_speed: 1, battery: 70,
    status: "IDLE", fsm: "IDLE", health: 100, current_task_id: null, destination: null,
    path: [], path_index: 0, load: { current: 0, capacity: 1 }, zone: null, eta_s: null,
    fsm_since_tick: 0, stats: { distance_m: 0, tasks_completed: 0, energy_wh: 0, busy_ticks: 0, wait_ticks: 0 },
    perception: { state: "CLEAR", ahead_m: 4, nearest_m: null, obstacles: [] }, control_mode: "MANUAL",
    vx: 0.1, vy: 0, wz: 0, navigation_state: "IDLE", last_telemetry_at: new Date().toISOString(),
  };
}

function renderNode(node: ReactNode) {
  act(() => root.render(node));
}

beforeEach(() => {
  globalThis.IS_REACT_ACT_ENVIRONMENT = true;
  window.history.replaceState({}, "", "/");
  useStore.setState({ ...initialState, twin: { ...initialState.twin, robots: { R01: r01() } }, quickDetailRobotId: null, robotDetail: {} });
  container = document.createElement("div");
  document.body.appendChild(container);
  root = createRoot(container);
  vi.spyOn(HTMLCanvasElement.prototype, "getContext").mockReturnValue(null);
});

afterEach(() => {
  act(() => root.unmount());
  container.remove();
  useStore.setState(initialState);
  vi.restoreAllMocks();
});

describe("robot quick detail workflow", () => {
  it("opens from the shared robot selection action and displays the robot ID", () => {
    act(() => useStore.getState().openRobotQuickDetail("R01"));
    renderNode(<RobotQuickDetailModal />);
    expect(document.body.textContent).toContain("ROBOT QUICK DETAIL");
    expect(document.body.textContent).toContain("R01");
    expect(document.body.textContent).toContain("Robot ID");
    expect(document.body.textContent).toContain("EMERGENCY STOP");
  });

  it("routes CONTROL ROBOT DETAIL to the selected robot URL", () => {
    act(() => useStore.getState().openRobotQuickDetail("R01"));
    renderNode(<RobotQuickDetailModal />);
    const control = Array.from(document.body.querySelectorAll("button")).find((button) => button.textContent?.includes("CONTROL ROBOT DETAIL"));
    expect(control).toBeTruthy();
    act(() => control?.click());
    expect(window.location.pathname).toBe("/robots/R01/control");
  });
});

describe("robot detail route stability", () => {
  it("renders the direct URL with null map/scan and disables manual motion while disconnected", () => {
    useStore.setState({ twin: null as never, rosDiagnostics: null, rosConnected: false, websocketState: "DISCONNECTED", robotDetail: {} });
    renderNode(<RobotControlDetailPage robotId="R01" />);
    expect(container.textContent).toContain("SYSTEM INPUTS");
    expect(container.textContent).toContain("STATE");
    expect(container.textContent).toContain("SYSTEM");
    expect(container.textContent).toContain("LIDAR OUTPUTS");
    expect(container.textContent).toContain("ERROR MESSAGES");
    expect(container.textContent).toContain("No errors reported");
    expect(container.querySelector(".manual-key-forward")?.hasAttribute("disabled")).toBe(true);
  });

  it("renders live telemetry values without requiring a map snapshot", () => {
    renderNode(<RobotControlDetailPage robotId="R01" />);
    expect(container.textContent).toContain("40.000 m");
    expect(container.textContent).toContain("64.000 m");
    expect(container.textContent).toContain("LiDAR WAITING");
    expect(container.textContent).toContain("N/A");
  });

  it("enables control only for the robot with a live ROS bridge", () => {
    useStore.setState({
      runtimeMode: "GAZEBO_ROS",
      rosConnected: true,
      websocketState: "CONNECTED",
      connectedRobotIds: ["R01"],
    });
    renderNode(<RobotControlDetailPage robotId="R01" />);
    expect(container.querySelector(".manual-key-forward")?.hasAttribute("disabled")).toBe(false);
    act(() => root.unmount());

    const r02 = { ...r01(), id: "R02", status: "ACTIVE" as const };
    useStore.setState({
      twin: { ...initialState.twin, robots: { R01: r01(), R02: r02 } },
      connectedRobotIds: ["R01"],
    });
    root = createRoot(container);
    renderNode(<RobotControlDetailPage robotId="R02" />);
    expect(container.textContent).toContain("OFFLINE");
    expect(container.querySelector(".manual-key-forward")?.hasAttribute("disabled")).toBe(true);
  });
});
