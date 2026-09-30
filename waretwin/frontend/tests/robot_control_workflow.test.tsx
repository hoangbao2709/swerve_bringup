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
    allow_task: true, allow_instant_actions: false, instant_actions_supported: false,
    instant_actions_status: "NOT_IMPLEMENTED", auto_reconnect: true, reconnect_interval: 5,
    connection_timeout: 5, keepalive: 30, client_id: "", connection_status: "DISABLED",
    last_error: null, ignored_orders: 0, updated_at: null,
  })),
  setMappingState: vi.fn(() => Promise.resolve({ ok: true, mapping_state: "MAPPING" })),
  saveLocalRobotMap: vi.fn(), loadLocalRobotMap: vi.fn(),
  initializeLocalRobotPose: vi.fn(), applyVda5050Configuration: vi.fn(),
  testVda5050Connection: vi.fn(() => Promise.resolve({ ok: true, latency_ms: 12, broker: "broker.local:1883" })),
}));
vi.mock("../src/services/ws", () => ({
  wsSend: vi.fn(() => true), wsManualCommand: vi.fn(() => true), wsSetRobotMode: vi.fn(() => true),
}));
vi.mock("../src/components/control/RobotLidarViews", async () => {
  const React = await import("react");
  return {
    localLidarPointToMap: (point: { x: number; y: number }) => point,
    previewLidarPath: () => null,
    RobotLidar2DView: () => React.createElement("div", { className: "robot-lidar-view", "data-testid": "lidar-2d" }),
    RobotLidar3DView: () => React.createElement("div", { className: "robot-lidar-view robot-lidar-3d-view", "data-testid": "lidar-3d" }),
  };
});
vi.mock("../src/simulation/runner", () => ({ useSimulationRunner: () => undefined }));

import { RobotQuickDetailModal } from "../src/components/robot/RobotQuickDetailModal";
import { RobotControlDetailPage } from "../src/components/control/RobotControlDetailPage";
import * as api from "../src/services/api";
import { wsSend } from "../src/services/ws";
import type { LocalRobotMap } from "../src/services/api";

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

function setOnlineRobot(runtimeState = "NAVIGATION") {
  const mapSnapshot = {
    robot_id: "R01", frame_id: "map", map_revision: 21, active_map_id: "CANONICAL",
    active_map_revision: "21", canonical_map_revision: 21,
    width: 10, height: 10, resolution: 1, origin: { x: -5, y: -5, yaw: 0 }, data: Array(100).fill(0),
  };
  useStore.setState({
    runtimeMode: "GAZEBO_ROS", runtimeState, rosConnected: true, websocketState: "CONNECTED",
    connectedRobotIds: ["R01"], mapSync: {
      ...initialState.mapSync, publishedRevision: 21, rosRevision: 21,
      nav2Revision: 21, status: "SYNCED", tfStatus: true,
      robots: { R01: { rosRevision: 21, gazeboRevision: 21, nav2Revision: 21,
        tagMapRevision: 21, tfStatus: true, error: null, status: "SYNCED" } },
    },
  });
  useStore.getState().setRobotDetail("R01", { map: mapSnapshot });
}

function buttonNamed(name: string): HTMLButtonElement | undefined {
  return Array.from(container.querySelectorAll("button")).find((button) => button.textContent?.trim() === name);
}

function setInputValue(input: HTMLInputElement, value: string) {
  const setter = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value")?.set;
  setter?.call(input, value);
  input.dispatchEvent(new Event("input", { bubbles: true }));
  input.dispatchEvent(new Event("change", { bubbles: true }));
}

function settleUi() { return new Promise<void>((resolve) => window.setTimeout(resolve, 0)); }

beforeEach(() => {
  globalThis.IS_REACT_ACT_ENVIRONMENT = true;
  window.history.replaceState({}, "", "/");
  useStore.setState({ ...initialState, twin: { ...initialState.twin, robots: { R01: r01() } }, quickDetailRobotId: null, robotDetail: {} });
  container = document.createElement("div");
  document.body.appendChild(container);
  root = createRoot(container);
  vi.spyOn(HTMLCanvasElement.prototype, "getContext").mockReturnValue(null);
  vi.spyOn(HTMLElement.prototype, "clientWidth", "get").mockReturnValue(640);
  vi.spyOn(HTMLElement.prototype, "clientHeight", "get").mockReturnValue(360);
  vi.spyOn(HTMLCanvasElement.prototype, "getBoundingClientRect").mockReturnValue({
    x: 0, y: 0, left: 0, top: 0, right: 640, bottom: 360, width: 640, height: 360,
    toJSON: () => ({}),
  } as DOMRect);
  vi.mocked(wsSend).mockClear();
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
    act(() => buttonNamed("3D")?.click());
    expect(container.querySelector("[data-testid='lidar-3d']")).toBeTruthy();
  });

  it("requests a Nav2 preview from the map click and gates Send Goal on the matching current approval", async () => {
    setOnlineRobot();
    useStore.setState({ twin: { ...initialState.twin, robots: { R01: { ...r01(), control_mode: "AUTONOMOUS" } } } });
    renderNode(<RobotControlDetailPage robotId="R01" />);

    const canvas = container.querySelector<HTMLCanvasElement>('[aria-label="World metre map and LiDAR renderer"]');
    expect(canvas).toBeTruthy();
    await act(async () => { canvas?.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    const request = vi.mocked(wsSend).mock.calls.map(([message]) => message)
      .find((message) => message.type === "PATH_PREVIEW_REQUEST");
    expect(request?.type).toBe("PATH_PREVIEW_REQUEST");
    if (!request || request.type !== "PATH_PREVIEW_REQUEST") throw new Error("path preview request was not emitted");
    expect(request).toMatchObject({ robot_id: "R01", frame_id: "map", active_map_id: "CANONICAL", active_map_revision: "21" });
    const send = buttonNamed("SEND GOAL");
    expect(send?.disabled).toBe(true);

    const approved = {
      robot_id: "R01", request_id: request.request_id, status: "VALID" as const,
      frame_id: "map" as const, path: [[0, 0], [1, 1]] as Array<[number, number]>,
      path_length_m: 1.4, goal: { x: request.x, y: request.y, yaw: request.yaw },
      timestamp: new Date().toISOString(), active_map_id: "CANONICAL", active_map_revision: "21",
      canonical_map_revision: 21,
    };
    act(() => useStore.getState().setRobotDetail("R01", { pathPreview: approved }));
    expect(buttonNamed("SEND GOAL")?.disabled).toBe(false);
    await act(async () => { buttonNamed("SEND GOAL")?.click(); });
    const goalMessage = vi.mocked(wsSend).mock.calls.map(([message]) => message)
      .find((message) => message.type === "NAV_GOAL");
    expect(goalMessage).toMatchObject({ type: "NAV_GOAL", robot_id: "R01", preview_request_id: request.request_id,
      active_map_id: "CANONICAL", active_map_revision: "21" });

    act(() => useStore.getState().setRobotDetail("R01", { pathPreview: { ...approved, path: [] } }));
    expect(buttonNamed("SEND GOAL")?.disabled).toBe(true);
    act(() => useStore.getState().setRobotDetail("R01", {
      pathPreview: { ...approved, timestamp: new Date(Date.now() - 121_000).toISOString() },
    }));
    expect(buttonNamed("SEND GOAL")?.disabled).toBe(true);
  });

  it("runs the mapping lifecycle and routes save/list/load to robot-scoped APIs", async () => {
    setOnlineRobot("MAPPING");
    let mappingState = "MAPPING";
    let maps: LocalRobotMap[] = [];
    const savedMap: LocalRobotMap = {
      id: "local-map-1", name: "floor_1", robot_id: "R01", created_at: "2026-09-30T00:00:00Z",
      resolution: 0.05, origin: [0, 0, 0], revision: "rev-2", frame_id: "map", width: 100, height: 100,
    };
    vi.mocked(api.getLocalRobotMaps).mockImplementation(async () => ({
      robot_id: "R01", maps, runtime_mode: "GAZEBO_ROS", mapping_state: mappingState,
      mapping_duration_s: 12, active_local_map_id: null, local_active_map_id: null,
      local_active_map_revision: null, active_map_id: "CANONICAL", active_map_revision: "21",
      canonical_map_revision: 21, map_sync_status: "CANONICAL",
    }));
    vi.mocked(api.setMappingState).mockImplementation(async (_robot, action) => {
      mappingState = action === "start" ? "MAPPING" : "PAUSED";
      return { ok: true, mapping_state: mappingState };
    });
    vi.mocked(api.saveLocalRobotMap).mockImplementation(async (_robot, name) => {
      const map = { ...savedMap, name };
      maps = [map];
      return { ok: true, map };
    });
    vi.mocked(api.loadLocalRobotMap).mockResolvedValue({
      ok: true, active_map: savedMap, active_map_id: savedMap.id,
      active_map_revision: savedMap.revision, canonical_map_revision: 21,
      map_sync_status: "LOCAL_ONLY", message: "local navigation enabled",
    });
    renderNode(<RobotControlDetailPage robotId="R01" />);
    await act(async () => { buttonNamed("MAPPING")?.click(); await Promise.resolve(); });
    expect(container.textContent).toContain("MAPPING SESSION");
    await act(async () => { buttonNamed("STOP MAPPING")?.click(); await settleUi(); });
    expect(api.setMappingState).toHaveBeenCalledWith("R01", "stop");
    expect(container.textContent).toContain("MAPPING PAUSED");
    await act(async () => { buttonNamed("RESUME MAPPING")?.click(); await settleUi(); });
    expect(api.setMappingState).toHaveBeenCalledWith("R01", "start");

    const nameInput = container.querySelector<HTMLInputElement>('input[placeholder="warehouse_floor_1"]');
    expect(nameInput).toBeTruthy();
    await act(async () => {
      if (nameInput) setInputValue(nameInput, "floor_1");
    });
    await act(async () => { buttonNamed("SAVE MAP")?.click(); await settleUi(); });
    expect(api.saveLocalRobotMap).toHaveBeenCalledWith("R01", "floor_1");
    expect(container.textContent).toContain("floor_1");
    expect(container.querySelector(".local-map-row")?.textContent).toContain("floor_1");

    act(() => useStore.setState({ runtimeState: "NAVIGATION" }));
    let finishLoad!: (value: Awaited<ReturnType<typeof api.loadLocalRobotMap>>) => void;
    const pendingLoad = new Promise<Awaited<ReturnType<typeof api.loadLocalRobotMap>>>((resolve) => {
      finishLoad = resolve;
    });
    vi.mocked(api.loadLocalRobotMap).mockReturnValueOnce(pendingLoad);
    await act(async () => { buttonNamed("LOAD MAP INTO NAV2")?.click(); await settleUi(); });
    expect(api.loadLocalRobotMap).toHaveBeenCalledWith("R01", "local-map-1");
    expect(container.textContent).toContain("LOADING");
    await act(async () => {
      finishLoad({
        ok: true, active_map: savedMap, active_map_id: savedMap.id,
        active_map_revision: savedMap.revision, canonical_map_revision: 21,
        map_sync_status: "LOCAL_ONLY", message: "local navigation enabled",
      });
      await settleUi();
    });
    expect(container.textContent).toContain("MAP LOADED");

    vi.mocked(api.loadLocalRobotMap).mockRejectedValueOnce(new Error("map_server rejected map"));
    await act(async () => { buttonNamed("LOAD MAP INTO NAV2")?.click(); await settleUi(); });
    expect(container.textContent).toContain("map_server rejected map");
    expect(container.textContent).toContain("ERROR");
  });

  it("picks and adjusts an initial pose, then reports localization apply failure", async () => {
    setOnlineRobot();
    vi.spyOn(window, "confirm").mockReturnValue(true);
    vi.mocked(api.initializeLocalRobotPose).mockResolvedValueOnce({
      ok: true, pose: { x: 0, y: 0, yaw: 0.2, frame_id: "map" }, localization_owner: "ekf_v30e",
    }).mockRejectedValueOnce(new Error("localization service unavailable"));
    renderNode(<RobotControlDetailPage robotId="R01" />);
    await act(async () => { buttonNamed("LOCALIZATION")?.click(); await Promise.resolve(); });
    await act(async () => { buttonNamed("PICK ON MAP")?.click(); });
    const picker = container.querySelector<HTMLCanvasElement>('[aria-label="Select map frame initial robot position"]');
    await act(async () => { picker?.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    await act(async () => { buttonNamed("YAW +")?.click(); });
    await act(async () => { buttonNamed("SET INITIAL POSE")?.click(); await Promise.resolve(); });
    expect(api.initializeLocalRobotPose).toHaveBeenCalledTimes(1);
    expect(api.initializeLocalRobotPose.mock.calls[0][1]).toMatchObject({ frame_id: "map" });
    expect(Number.isFinite(api.initializeLocalRobotPose.mock.calls[0][1].x)).toBe(true);
    expect(Number.isFinite(api.initializeLocalRobotPose.mock.calls[0][1].y)).toBe(true);
    expect(api.initializeLocalRobotPose.mock.calls[0][1].yaw).not.toBe(r01().heading);
    expect(container.textContent).toContain("INITIAL POSE ACCEPTED BY ekf_v30e");
    await act(async () => { buttonNamed("SET INITIAL POSE")?.click(); await Promise.resolve(); });
    expect(container.textContent).toContain("localization service unavailable");
  });

  it("keeps VDA5050 secret masked, shows dirty/apply and task behavior, and reports test/apply outcomes", async () => {
    vi.spyOn(window, "confirm").mockReturnValue(true);
    const config = {
      robot_id: "R01", enabled: false, mqtt_host: "broker.local", mqtt_port: 1883,
      mqtt_username: "robot01", password_configured: true, tls_enabled: false,
      topic_prefix: "vda5050", interface_name: "uagv", manufacturer: "PTAGV", serial_number: "R01",
      protocol_version: "2.0.0" as const, mqtt_protocol_version: "3.1.1" as const,
      allow_task: true, allow_instant_actions: false, instant_actions_supported: false,
      instant_actions_status: "NOT_IMPLEMENTED", auto_reconnect: true, reconnect_interval: 5,
      connection_timeout: 5, keepalive: 30, client_id: "", connection_status: "DISABLED",
      last_error: null, ignored_orders: 0, updated_at: null,
    };
    vi.mocked(api.getVda5050Configuration).mockResolvedValue(config);
    vi.mocked(api.testVda5050Connection).mockResolvedValueOnce({
      ok: false, latency_ms: 9, broker: "broker.local:1883", error_code: "REFUSED", message: "connection refused",
    });
    vi.mocked(api.applyVda5050Configuration).mockRejectedValueOnce(new Error("broker apply failed"));
    renderNode(<RobotControlDetailPage robotId="R01" />);
    await act(async () => { buttonNamed("VDA5050")?.click(); await Promise.resolve(); });
    expect(container.querySelector('input[type="password"]')?.getAttribute("type")).toBe("password");
    expect(container.textContent).toContain("INSTANT ACTION EXECUTION");
    expect(container.textContent).toContain("NOT IMPLEMENTED");
    expect(buttonNamed("SAVE & APPLY")?.disabled).toBe(true);

    const host = Array.from(container.querySelectorAll("label")).find((label) => label.textContent?.includes("MQTT HOST"))?.querySelector("input");
    await act(async () => {
      if (host) setInputValue(host, "new-broker.local");
    });
    expect(buttonNamed("SAVE & APPLY")?.disabled).toBe(false);
    const taskToggle = Array.from(container.querySelectorAll("label")).find((label) => label.textContent?.includes("ALLOW TASK"))?.querySelector<HTMLInputElement>('input[type="checkbox"]');
    await act(async () => { taskToggle?.click(); });
    expect(container.textContent).toContain("New VDA5050 orders are blocked for this robot");
    await act(async () => { buttonNamed("TEST CONNECTION")?.click(); await Promise.resolve(); });
    expect(api.testVda5050Connection).toHaveBeenCalledWith("R01", expect.objectContaining({ mqtt_host: "new-broker.local" }));
    expect(container.textContent).toContain("FAILED · REFUSED · connection refused");
    await act(async () => { buttonNamed("SAVE & APPLY")?.click(); await Promise.resolve(); });
    expect(api.applyVda5050Configuration).toHaveBeenCalledWith("R01", expect.objectContaining({
      mqtt_host: "new-broker.local", allow_task: false, mqtt_password: "",
    }));
    expect(container.textContent).toContain("broker apply failed");
  });
});
