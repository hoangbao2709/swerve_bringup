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
  saveLocalRobotMap: vi.fn(), loadLocalRobotMap: vi.fn(), resumeLocalRobotSlamSession: vi.fn(),
  initializeLocalRobotPose: vi.fn(), applyVda5050Configuration: vi.fn(),
  testVda5050Connection: vi.fn(() => Promise.resolve({ ok: true, latency_ms: 12, broker: "broker.local:1883" })),
}));
vi.mock("../src/services/ws", () => ({
  WS_URL: "ws://127.0.0.1:8001/ws",
  wsSend: vi.fn(() => true), wsManualCommand: vi.fn(() => true), wsSetRobotMode: vi.fn(() => true),
}));
vi.mock("../src/components/control/RobotLidarViews", async () => {
  const React = await import("react");
  return {
    localLidarPointToMap: (point: { x: number; y: number }) => point,
    previewLidarPath: () => null,
    RobotLidar2DView: () => React.createElement("div", { className: "robot-lidar-view", "data-testid": "lidar-2d" }),
    RobotLidar3DView: ({ frame }: { frame: { point_count: number } | null }) => React.createElement("div", {
      className: "robot-lidar-view robot-lidar-3d-view", "data-testid": "slam-map-3d",
      "data-accumulated-points": String(frame?.point_count ?? 0),
    }),
  };
});
vi.mock("../src/simulation/runner", () => ({ useSimulationRunner: () => undefined }));

import { RobotQuickDetailModal } from "../src/components/robot/RobotQuickDetailModal";
import { RobotControlDetailPage } from "../src/components/control/RobotControlDetailPage";
import * as api from "../src/services/api";
import { wsSend, wsManualCommand, wsSetRobotMode } from "../src/services/ws";
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
    canonical_pose: { x: 15, y: 5.5, yaw: 0.7, frame_id: "map", map_id: "CANONICAL",
      map_revision: "21", map_source: "CANONICAL", pose_source: "GAZEBO_MODEL_STATES", valid: true,
      source_frame_id: "world", transform_source: "VALIDATED_CANONICAL_WORLD_BUNDLE", timestamp: new Date().toISOString() },
    slam_pose: { x: 2.25, y: 3.5, yaw: -0.2, frame_id: "map", map_id: "SLAM-session-1",
      map_revision: "slam-r1", map_source: "SLAM_TOOLBOX", pose_source: "TF", valid: true,
      mapping_session_id: "session-1", timestamp: new Date().toISOString() },
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
  useStore.getState().setRobotDetail("R01", { runtimeMapSnapshot: mapSnapshot });
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
  it("keeps applied mode while a requested transition is pending", () => {
    useStore.setState({ runtimeMode: "GAZEBO_ROS", rosConnected: true, websocketState: "CONNECTED", connectedRobotIds: ["R01"] });
    renderNode(<RobotControlDetailPage robotId="R01" />);
    act(() => buttonNamed("AUTONOMOUS")?.click());
    expect(wsSetRobotMode).toHaveBeenCalledWith("R01", "AUTONOMOUS");
    expect(container.querySelector(".robot-detail-mode")?.textContent).toContain("MANUAL → AUTONOMOUS REQUESTED");
    act(() => useStore.getState().setRobotDetail("R01", { appliedMode: "AUTONOMOUS", modeTransitionState: "APPLIED" }));
    expect(container.querySelector(".robot-detail-mode")?.textContent).toContain("AUTONOMOUS APPLIED");
  });

  it("pointer hold captures and release, cancel or leave sends STOP", () => {
    useStore.setState({ runtimeMode: "GAZEBO_ROS", rosConnected: true, websocketState: "CONNECTED", connectedRobotIds: ["R01"] });
    renderNode(<RobotControlDetailPage robotId="R01" />);
    const forward = container.querySelector<HTMLButtonElement>(".manual-key-forward")!;
    const capture = vi.fn(); forward.setPointerCapture = capture;
    for (const release of ["pointerup", "pointercancel", "pointerout"]) {
      const down = new Event("pointerdown", { bubbles: true });
      Object.defineProperty(down, "pointerId", { value: 42 });
      act(() => forward.dispatchEvent(down));
      expect(capture).toHaveBeenCalledWith(42);
      expect(wsManualCommand).toHaveBeenLastCalledWith("R01", "FORWARD");
      act(() => forward.dispatchEvent(new Event(release, { bubbles: true })));
      expect(wsManualCommand).toHaveBeenLastCalledWith("R01", "STOP");
    }
  });
  it("routes manual hold refresh through the dedicated worker and stops it on release", async () => {
    const postMessage = vi.fn();
    const terminate = vi.fn();
    const constructWorker = vi.fn();
    class FakeWorker {
      onmessage: ((event: MessageEvent) => void) | null = null;
      onerror: ((event: ErrorEvent) => void) | null = null;
      postMessage = postMessage;
      terminate = terminate;
      constructor(url: URL, options: WorkerOptions) { constructWorker(url, options); }
    }
    vi.stubGlobal("Worker", FakeWorker);
    useStore.setState({ authToken: "test-access-token" });
    setOnlineRobot();
    vi.mocked(wsManualCommand).mockClear();
    renderNode(<RobotControlDetailPage robotId="R01" />);
    await act(async () => { await settleUi(); });
    expect(constructWorker).toHaveBeenCalledWith(expect.any(URL), { type: "module" });
    expect(postMessage).toHaveBeenCalledWith({ type: "CONNECT", url: "ws://127.0.0.1:8001/ws", token: "test-access-token" });

    const forward = container.querySelector<HTMLButtonElement>(".manual-key-forward")!;
    const down = new Event("pointerdown", { bubbles: true });
    Object.defineProperty(down, "pointerId", { value: 42 });
    act(() => forward.dispatchEvent(down));
    expect(postMessage).toHaveBeenCalledWith({ type: "HOLD", robot_id: "R01", action: "FORWARD" });
    act(() => forward.dispatchEvent(new Event("pointerup", { bubbles: true })));
    expect(postMessage).toHaveBeenCalledWith({ type: "STOP", robot_id: "R01" });
    expect(wsManualCommand).not.toHaveBeenCalled();

    act(() => root.unmount());
    root = createRoot(container);
    expect(postMessage).toHaveBeenCalledWith({ type: "DISCONNECT", robot_id: "R01" });
  });
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
    expect(container.textContent).toContain("LIDAR OUTPUTS");
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
    expect(container.textContent).toContain("SAVE NAVIGATION MAP + SLAM SESSION");
    const vdaTab = Array.from(container.querySelectorAll("[role=tab]")).find((tab) => tab.textContent === "VDA5050");
    await act(async () => { (vdaTab as HTMLElement).click(); });
    expect(container.textContent).toContain("VDA5050 CONFIGURATION");
    expect(container.textContent).toContain("MQTT HOST");
    expect(container.textContent).toContain("ALLOW TASK");
    expect(container.querySelector('input[type="password"]')?.getAttribute("type")).toBe("password");
    expect(container.querySelector("h1")?.textContent).toBe("R01");
  });

  it("offers three separate canonical and SLAM map views", () => {
    renderNode(<RobotControlDetailPage robotId="R01" />);
    const sourceTabs = container.querySelector('[role="tablist"][aria-label="Robot map view"]');
    expect(sourceTabs?.textContent).toBe("GLOBAL MAPMAP VIEW 2DMAP VIEW 3D");
    act(() => buttonNamed("MAP VIEW 2D")?.click());
    expect(container.querySelector('[data-testid="slam-map-2d-empty"]')).toBeTruthy();
    act(() => buttonNamed("MAP VIEW 3D")?.click());
    expect(container.querySelector("[data-testid='slam-map-3d']")).toBeTruthy();
  });

  it("switches views immediately without clearing the selected robot's cached frames", () => {
    setOnlineRobot("MAPPING");
    const slamMap = { robot_id: "R01", frame_id: "map", map_source: "SLAM_TOOLBOX" as const,
      mapping_session_id: "session-1", active_map_id: "SLAM-session-1", width: 2, height: 2,
      resolution: .05, origin: { x: 0, y: 0, yaw: 0 }, data: [-1, 0, 100, -1] };
    const frame3d = { robot_id: "R01", frame_id: "map", source_frame_id: "lidar_link", point_count: 2,
      points: [[1, 0, 0], [2, 1, 0.1]] as [number, number, number][], bounds: null,
      epoch: "bridge", revision: 1, accumulated: true as const, accumulation_mode: "SLAM_VISUALIZATION_VOXEL_MAP" as const,
      slam_pose: { ...r01().slam_pose!, timestamp: new Date().toISOString() } };
    useStore.getState().setRobotDetail("R01", { mappingSessionId: "session-1", slam2dMap: slamMap,
      slam3dAccumulatedCloud: frame3d });
    renderNode(<RobotControlDetailPage robotId="R01" />);
    const globalMap = container.querySelector('[data-testid="global-warehouse-map"]');
    for (const name of ["MAP VIEW 2D", "MAP VIEW 3D", "MAP VIEW 2D", "GLOBAL MAP"]) act(() => buttonNamed(name)?.click());
    expect(useStore.getState().robotDetail.R01.slam2dMap).toBe(slamMap);
    expect(useStore.getState().robotDetail.R01.slam3dAccumulatedCloud).toBe(frame3d);
    expect(container.querySelector('[data-testid="global-warehouse-map"]')).toBe(globalMap);
    expect(useStore.getState().robotDetail.R02).toBeUndefined();
    expect(container.querySelector('[data-testid="slam-map-3d"]')?.getAttribute("data-accumulated-points")).toBe("2");
  });

  it("renders the cached accumulated cloud immediately when switching back to 3D", () => {
    setOnlineRobot("MAPPING");
    const cloud = { robot_id: "R01", frame_id: "map", source_frame_id: "lidar_link", point_count: 12,
      points: Array.from({ length: 12 }, (_, index) => [index, 0, 0] as [number, number, number]),
      bounds: null, epoch: "bridge", revision: 8, accumulated: true as const,
      accumulation_mode: "SLAM_VISUALIZATION_VOXEL_MAP" as const,
      slam_pose: { ...r01().slam_pose!, timestamp: new Date().toISOString() } };
    useStore.getState().setRobotDetail("R01", { mappingSessionId: "session-1", slam3dAccumulatedCloud: cloud });
    renderNode(<RobotControlDetailPage robotId="R01" />);
    act(() => buttonNamed("MAP VIEW 3D")?.click());
    expect(container.querySelector('[data-testid="slam-map-3d"]')?.getAttribute("data-accumulated-points")).toBe("12");
    act(() => buttonNamed("MAP VIEW 2D")?.click());
    act(() => buttonNamed("MAP VIEW 3D")?.click());
    expect(container.querySelector('[data-testid="slam-map-3d"]')?.getAttribute("data-accumulated-points")).toBe("12");
  });

  it("keeps the canonical warehouse and canonical pose selected while SLAM telemetry changes", () => {
    setOnlineRobot("MAPPING");
    const slamMap = { robot_id: "R01", frame_id: "map", map_source: "SLAM_TOOLBOX" as const,
      mapping_session_id: "session-1", active_map_id: "SLAM-session-1", width: 2, height: 2,
      resolution: .05, origin: { x: -1, y: -1, yaw: 0 }, data: [-1, 0, 100, -1] };
    const runtimeMap = { ...slamMap, map_source: "NAV2_MAP" as const, mapping_session_id: null, active_map_id: "CANONICAL" };
    useStore.getState().setRobotDetail("R01", { mappingSessionId: "session-1", slam2dMap: slamMap, runtimeMapSnapshot: runtimeMap });
    renderNode(<RobotControlDetailPage robotId="R01" />);
    const canvas = container.querySelector<HTMLCanvasElement>('[data-testid="global-warehouse-map"]');
    expect(canvas?.dataset.mapSource).toBe("CANONICAL_WAREHOUSE");
    expect(canvas?.dataset.poseSource).toBe("GAZEBO_MODEL_STATES");
    expect(canvas?.dataset.renderX).toBe("15");
    act(() => buttonNamed("MAP VIEW 2D")?.click());
    expect(container.querySelector<HTMLCanvasElement>('[data-testid="slam-map-2d-canvas"]')?.dataset.mapSource).toBe("SLAM_TOOLBOX");
    act(() => useStore.setState({ twin: { ...useStore.getState().twin!, robots: {
      ...useStore.getState().twin!.robots, R01: { ...r01(), position: [90, 0, 90],
        canonical_pose: null, slam_pose: { ...r01().slam_pose!, x: 4.5, y: 6.25, timestamp: new Date().toISOString() } },
    } } }));
    expect(container.querySelector('[data-testid="global-warehouse-map"]')).toBe(canvas);
    expect(canvas?.dataset.renderX).toBe("15");
    expect(canvas?.dataset.mapSource).toBe("CANONICAL_WAREHOUSE");
  });

  it("requests a Nav2 preview from the map click and gates Send Goal on the matching current approval", async () => {
    setOnlineRobot();
    useStore.setState({ twin: { ...initialState.twin, robots: { R01: { ...r01(), control_mode: "AUTONOMOUS" } } } });
    renderNode(<RobotControlDetailPage robotId="R01" />);

    const canvas = container.querySelector<HTMLCanvasElement>('[aria-label="Canonical warehouse and robot pose map"]');
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
    useStore.getState().setRobotDetail("R01", { slam2dMap: {
      robot_id: "R01", frame_id: "map", map_source: "SLAM_TOOLBOX",
      mapping_session_id: "session-ui", active_map_id: "SLAM-session-ui",
      active_map_revision: "revision-1", width: 10, height: 10, resolution: 0.05,
      origin: { x: 0, y: 0, yaw: 0 }, data: Array(100).fill(-1), known_cells: 1,
    } });
    let mappingState = "MAPPING";
    let maps: LocalRobotMap[] = [];
    let currentMode = "MAPPING";
    let robotControlMode = "MANUAL";
    const savedMap: LocalRobotMap = {
      id: "local-map-1", name: "floor_1", robot_id: "R01", created_at: "2026-09-30T00:00:00Z",
      resolution: 0.05, origin: [0, 0, 0], revision: "rev-2", frame_id: "map", width: 100, height: 100,
      slam_session_state: { status: "AVAILABLE", engine: "SLAM_TOOLBOX", artifact_id: "session-artifact" },
    };
    vi.mocked(api.getLocalRobotMaps).mockImplementation(async () => ({
      robot_id: "R01", maps, runtime_mode: "GAZEBO_ROS", mapping_state: mappingState,
      mapping_duration_s: 12, active_local_map_id: null, local_active_map_id: null,
      local_active_map_revision: null, active_map_id: "CANONICAL", active_map_revision: "21",
      canonical_map_revision: 21, map_sync_status: "CANONICAL",
      robot_control_mode: robotControlMode, robot_stopped: true,
    }));
    vi.mocked(api.getLocalRuntimeMode).mockImplementation(async () => ({
      robot_id: "R01", current_mode: currentMode,
      transition: { robot_id: "R01", mode: currentMode.toLowerCase(), status: "READY", message: "runtime ready" },
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
    vi.mocked(api.loadLocalRobotMap).mockImplementationOnce(async () => {
      currentMode = "NAVIGATION";
      mappingState = "PAUSED";
      return {
        ok: true, status: "TRANSITIONING", active_map: savedMap,
        active_map_id: savedMap.id, active_map_revision: savedMap.revision,
        request_id: "nav-transition-1", mapping_state: "PAUSED",
        message: "waiting for navigation readiness",
      };
    });
    vi.mocked(wsSetRobotMode).mockImplementation((_robotId, mode) => {
      robotControlMode = mode;
      return true;
    });
    renderNode(<RobotControlDetailPage robotId="R01" />);
    await act(async () => { buttonNamed("MAPPING")?.click(); await Promise.resolve(); });
    expect(container.textContent).toContain("MAPPING SESSION");
    await act(async () => { buttonNamed("STOP MAPPING")?.click(); await settleUi(); });
    expect(api.setMappingState).toHaveBeenCalledWith("R01", "stop");
    expect(container.textContent).toContain("MAPPING PAUSED");
    const nameInput = container.querySelector<HTMLInputElement>('input[placeholder="warehouse_floor_1"]');
    expect(nameInput).toBeTruthy();
    await act(async () => {
      if (nameInput) setInputValue(nameInput, "floor_1");
    });
    await act(async () => { buttonNamed("SAVE MAP")?.click(); await settleUi(); });
    expect(api.saveLocalRobotMap).toHaveBeenCalledWith("R01", "floor_1");
    expect(container.textContent).toContain("floor_1");
    expect(container.querySelector(".local-map-row")?.textContent).toContain("floor_1");

    await act(async () => { buttonNamed("RESUME MAPPING")?.click(); await settleUi(); });
    expect(api.setMappingState).toHaveBeenCalledWith("R01", "start");

    let finishLoad!: (value: Awaited<ReturnType<typeof api.loadLocalRobotMap>>) => void;
    const pendingLoad = new Promise<Awaited<ReturnType<typeof api.loadLocalRobotMap>>>((resolve) => {
      finishLoad = resolve;
    });
    vi.mocked(api.loadLocalRobotMap).mockReturnValueOnce(pendingLoad);
    await act(async () => { buttonNamed("LOAD SAVED MAP FOR NAVIGATION")?.click(); await settleUi(); });
    expect(api.loadLocalRobotMap).toHaveBeenCalledWith("R01", "local-map-1");
    expect(wsSetRobotMode).toHaveBeenCalledWith("R01", "MANUAL");
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
    await act(async () => { buttonNamed("LOAD SAVED MAP FOR NAVIGATION")?.click(); await settleUi(); });
    expect(container.textContent).toContain("map_server rejected map");
    expect(container.textContent).toContain("ERROR");

    const confirm = vi.spyOn(window, "confirm").mockReturnValue(true);
    vi.mocked(api.resumeLocalRobotSlamSession).mockResolvedValue({
      ok: true, status: "RESUMED", map: savedMap, map_id: savedMap.id,
      mapping_state: "MAPPING", request_id: "slam-resume-1",
      transition: { robot_id: "R01", mode: "mapping", status: "READY" },
      restore_evidence: { passed: true, live_known_cells: 62_000,
        known_overlap_ratio: 0.97, saved_coverage_ratio: 1, cell_class_agreement_ratio: 0.96 },
      message: "The prior SLAM map is restored and live mapping has resumed.",
    });
    await act(async () => { buttonNamed("RESUME SAVED SLAM SESSION")?.click(); await settleUi(); });
    expect(api.resumeLocalRobotSlamSession).toHaveBeenCalledWith("R01", "local-map-1");
    expect(container.textContent).toContain("SLAM SESSION RESTORED");
    expect(container.textContent).toContain("97.0%");
    confirm.mockRestore();
  });

  it("does not present a cached Nav2/canonical grid as the accumulated SLAM map", async () => {
    setOnlineRobot("MAPPING");
    useStore.getState().setRobotDetail("R01", { runtimeMapSnapshot: {
      robot_id: "R01", frame_id: "map", map_source: "NAV2_MAP",
      width: 1, height: 1, resolution: 0.05, origin: { x: 0, y: 0, yaw: 0 }, data: [100],
    } });
    renderNode(<RobotControlDetailPage robotId="R01" />);
    await act(async () => { buttonNamed("MAPPING")?.click(); await Promise.resolve(); });

    expect(container.textContent).toContain("Waiting for a fresh accumulated SLAM Toolbox /map");
    expect(container.querySelector('[aria-label="Select map frame initial robot position"]')).toBeNull();
  });

  it("shows separate accumulated SLAM, transformed scan, TF robot and bounded trajectory layers", async () => {
    setOnlineRobot("MAPPING");
    useStore.getState().setRobotDetail("R01", {
      slam2dMap: {
        robot_id: "R01", frame_id: "map", map_source: "SLAM_TOOLBOX",
        mapping_session_id: "session-1", active_map_id: "SLAM-session-1",
        active_map_revision: "rev-1", map_version: 1,
        width: 2, height: 2, resolution: 0.05, origin: { x: -1, y: -1, yaw: 0 },
        known_cells: 2, unknown_cells: 2, free_cells: 1, occupied_cells: 1,
        explored_area_m2: 0.005, data: [-1, 0, 100, -1],
      },
      scan: {
        robot_id: "R01", topic: "/scan", source_frame_id: "lidar_link", frame_id: "map",
        mapping_session_id: "session-1",
        sensor_pose: { x: 0.1, y: 0.2, yaw: 0.3 }, timestamp: new Date().toISOString(),
        angle_min: 0, angle_max: 1, angle_increment: 0.5,
        range_min: 0.1, range_max: 10, point_count: 1, points: [[1, 2]],
      },
      diagnostics: { robot_id: "R01", mapping: {
        slam_state: "ACTIVE", mapping_session_id: "session-1", scan_live: true, scan_hz: 5, scan_frame: "lidar_link",
        odom_live: true, odom_hz: 10, odom_frame: "odom", base_frame: "base_footprint",
        tf_valid: true, tf_lidar_to_map_valid: true, map_live: true, map_hz: 0.5,
        map_width_cells: 2, map_height_cells: 2, map_version: 1,
      } },
    });
    renderNode(<RobotControlDetailPage robotId="R01" />);
    await act(async () => { buttonNamed("MAPPING")?.click(); await settleUi(); });

    expect(container.textContent).toContain("ACCUMULATED SLAM MAP · /map + CURRENT /scan");
    expect(container.textContent).toContain("EXPLORED AREA");
    expect(container.textContent).toContain("TF · map ← lidar");
    const toggles = container.querySelectorAll(".local-map-toggles button");
    expect(toggles).toHaveLength(4);
    expect(Array.from(toggles).map((button) => button.textContent?.trim())).toEqual([
      "✓ ROBOT", "✓ SCAN", "✓ TRAJECTORY", "□ GRID",
    ]);
    expect(Array.from(toggles).slice(0, 3).every((button) => button.getAttribute("aria-pressed") === "true")).toBe(true);
  });

  it("does not show a cached SLAM map from a previous mapping session", async () => {
    setOnlineRobot("MAPPING");
    useStore.getState().setRobotDetail("R01", {
      mappingSessionId: "session-current",
      slam2dMap: {
        robot_id: "R01", frame_id: "map", map_source: "SLAM_TOOLBOX",
        mapping_session_id: "session-old", active_map_id: "SLAM-session-old",
        width: 1, height: 1, resolution: 0.05, origin: { x: 0, y: 0, yaw: 0 }, data: [100],
      },
    });
    renderNode(<RobotControlDetailPage robotId="R01" />);
    await act(async () => { buttonNamed("MAPPING")?.click(); await settleUi(); });

    expect(container.textContent).toContain("Waiting for a fresh accumulated SLAM Toolbox /map");
    expect(container.querySelector('[aria-label="Select map frame initial robot position"]')).toBeNull();
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
