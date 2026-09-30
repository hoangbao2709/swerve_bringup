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
  getLocalRobotMaps: vi.fn(() => Promise.resolve({ maps: [], mapping_state: "MAPPING", active_local_map_id: null })),
  getLocalRuntimeMode: vi.fn(() => Promise.resolve({
    robot_id: "R01", current_mode: "MAPPING",
    transition: { robot_id: "R01", mode: "mapping", status: "READY", message: "mapping mode is ready" },
  })),
  requestLocalRuntimeMode: vi.fn(() => Promise.resolve({
    ok: true, current_mode: "MAPPING", requested_mode: "NAVIGATION", request_id: "test-request",
    status: "REQUESTED", message: "restart requested",
  })),
  getVda5050Configuration: vi.fn(() => Promise.resolve({
    robot_id: "R01", enabled: false, mqtt_host: "", mqtt_port: 1883, mqtt_username: "",
    password_configured: false, tls_enabled: false, topic_prefix: "vda5050", interface_name: "uagv",
    manufacturer: "PTAGV", serial_number: "R01", protocol_version: "2.0.0", mqtt_protocol_version: "3.1.1",
    allow_task: true, allow_instant_actions: true, auto_reconnect: true, reconnect_interval: 5,
    connection_timeout: 5, keepalive: 30, client_id: "", connection_status: "DISABLED",
    last_error: null, ignored_orders: 0, updated_at: null,
  })),
  setMappingState: vi.fn(), saveLocalRobotMap: vi.fn(), loadLocalRobotMap: vi.fn(),
  initializeLocalRobotPose: vi.fn(), applyVda5050Configuration: vi.fn(), testVda5050Connection: vi.fn(),
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

  it("keeps local mapping and VDA5050 sections inside the selected robot detail page", async () => {
    renderNode(<RobotControlDetailPage robotId="R01" />);
    expect(container.querySelector('[role="tablist"][aria-label="Local robot control sections"]')).toBeTruthy();
    const mappingTab = Array.from(container.querySelectorAll("[role=tab]")).find((tab) => tab.textContent === "MAPPING");
    await act(async () => { (mappingTab as HTMLElement).click(); });
    expect(container.textContent).toContain("MAPPING SESSION");
    expect(container.textContent).toContain("SAVE NAV2 MAP");
    const vdaTab = Array.from(container.querySelectorAll("[role=tab]")).find((tab) => tab.textContent === "VDA5050");
    await act(async () => { (vdaTab as HTMLElement).click(); });
    expect(container.textContent).toContain("VDA5050 CONFIGURATION");
    expect(container.textContent).toContain("MQTT HOST");
    expect(container.textContent).toContain("ALLOW TASK");
    expect(container.querySelector('input[type="password"]')?.getAttribute("type")).toBe("password");
    expect(container.querySelector("h1")?.textContent).toBe("R01");
  });

  it("offers only Global and LiDAR as primary map modes, with 2D/3D under LiDAR", () => {
    renderNode(<RobotControlDetailPage robotId="R01" />);
    const sourceTabs = container.querySelector('[role="tablist"][aria-label="Primary map source"]');
    expect(sourceTabs?.textContent).toBe("GLOBAL MAPLIDAR MAP");
    act(() => Array.from(sourceTabs?.querySelectorAll("button") ?? []).find((button) => button.textContent === "LIDAR MAP")?.click());
    expect(container.querySelector('[role="tablist"][aria-label="LiDAR view dimension"]')?.textContent).toBe("2D3D");
    expect(container.querySelector(".robot-lidar-view")).toBeTruthy();
  });
});
