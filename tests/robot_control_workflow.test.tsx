// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, type ReactNode } from "react";
import { createRoot, type Root } from "react-dom/client";
import type { RobotState } from "../src/schema/twin_state";
import { useStore } from "../src/state/store";

vi.mock("../src/services/api", () => ({
  apiFetch: vi.fn(() => Promise.resolve({ ok: false, status: 503 })),
  emergencyStop: vi.fn(() => Promise.resolve({ ok: true })),
  clearEmergencyStop: vi.fn(() => Promise.resolve({ ok: true, code: "CLEAR_ESTOP_APPLIED",
    robot_id: "R01", emergency_stop_active: false, pre_stop_navigation_terminal: true })),
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
  wsSend: vi.fn(() => true), wsAcquireManual: vi.fn(() => true),
  wsManualCommand: vi.fn(() => true), wsSetRobotMode: vi.fn(() => true),
}));
import { RobotQuickDetailModal } from "../src/components/robot/RobotQuickDetailModal";
import { AccumulatedSlamMap2DView } from "../src/components/shared/maps/LocalMapViews";
import { ControlPage } from "../src/pages/control/ControlPage";
import { MappingPage } from "../src/pages/mapping/MappingPage";
import { LocalizationPage } from "../src/pages/localization/LocalizationPage";
import { Vda5050Page } from "../src/pages/vda5050/Vda5050Page";
import { DiagnosisPage } from "../src/pages/diagnosis/DiagnosisPage";
import { AppLayout } from "../src/app/AppLayout";
import { sectionForPathname, useLocalPathname } from "../src/routing/localRoutes";
import { occupancyRasterKey, occupancyRasters } from "../src/components/shared/maps/occupancyRaster";
import * as api from "../src/services/api";
import { wsSend, wsAcquireManual, wsManualCommand, wsSetRobotMode } from "../src/services/ws";
import type { LocalInitialPoseResult, LocalRobotMap, LocalSlamResumeResult } from "../src/services/api";

const initialState = useStore.getState();
let root: Root;
let container: HTMLDivElement;

function ControlDetailHarness({ robotId: _robotId }: { robotId: string }) {
  const pathname = useLocalPathname();
  const activeSection = sectionForPathname(pathname) ?? "CONTROL";
  const page = activeSection === "CONTROL" ? <ControlPage />
    : activeSection === "MAPPING" ? <MappingPage />
      : activeSection === "LOCALIZATION" ? <LocalizationPage />
        : activeSection === "VDA5050" ? <Vda5050Page /> : <DiagnosisPage />;
  return <AppLayout pageTitle={activeSection} isControlPage={activeSection === "CONTROL"}>{page}</AppLayout>;
}

function r01(): RobotState {
  return {
    id: "R01", model: "AMR-L", floor: 1, lift_id: null, lift_stage: null,
    position: [40, 0, 64], heading: 0.2, velocity: 0, max_speed: 1, battery: 70,
    status: "IDLE", fsm: "IDLE", health: 100, current_task_id: null, destination: null,
    path: [], path_index: 0, load: { current: 0, capacity: 1 }, zone: null, eta_s: null,
    fsm_since_tick: 0, stats: { distance_m: 0, tasks_completed: 0, energy_wh: 0, busy_ticks: 0, wait_ticks: 0 },
    perception: { state: "CLEAR", ahead_m: 4, nearest_m: null, obstacles: [] }, control_mode: "MANUAL",
    vx: 0, vy: 0, wz: 0, navigation_state: "IDLE", last_telemetry_at: new Date().toISOString(),
    canonical_pose: { x: 15, y: 5.5, yaw: 0.7, frame_id: "map", map_id: "CANONICAL",
      map_revision: "21", map_source: "CANONICAL", pose_source: "GAZEBO_MODEL_STATES", valid: true,
      source_frame_id: "world", transform_source: "VALIDATED_CANONICAL_WORLD_BUNDLE", timestamp: new Date().toISOString() },
    active_map_pose: { x: 15, y: 5.5, yaw: 0.7, frame_id: "map", map_id: "CANONICAL",
      map_revision: "21", map_source: "CANONICAL", pose_source: "GAZEBO_MODEL_STATES", valid: true,
      source_frame_id: "world", transform_source: "VALIDATED_CANONICAL_WORLD_BUNDLE", timestamp: new Date().toISOString() },
    slam_pose: { x: 2.25, y: 3.5, yaw: -0.2, frame_id: "map", map_id: "SLAM-session-1",
      map_revision: "session-session-1", map_content_revision: "slam-r1",
      map_source: "SLAM_TOOLBOX", pose_source: "TF", valid: true,
      mapping_session_id: "session-1", timestamp: new Date().toISOString() },
  };
}

function renderNode(node: ReactNode) {
  act(() => root.render(node));
}

function setOnlineRobot(runtimeState = "NAVIGATION") {
  const telemetryAt = new Date().toISOString();
  const mapSnapshot = {
    robot_id: "R01", frame_id: "map", map_source: "NAV2_MAP" as const, map_revision: 21, active_map_id: "NAV2-R01-map",
    active_map_revision: "nav2-r21", canonical_map_revision: 21,
    width: 10, height: 10, resolution: 1, origin: { x: -5, y: -5, yaw: 0 }, data: Array(100).fill(0),
  };
  useStore.setState({
    runtimeMode: "GAZEBO_ROS", runtimeState, rosConnected: true, websocketState: "CONNECTED",
    connectedRobotIds: ["R01"], lastTelemetryAt: telemetryAt,
    rosDiagnostics: { command_ownership: { estop_active: false } } as never,
    twin: { ...useStore.getState().twin, robots: { R01: { ...r01(), last_telemetry_at: telemetryAt } } }, mapSync: {
      ...initialState.mapSync, publishedRevision: 21, rosRevision: 21,
      nav2Revision: 21, status: "SYNCED", tfStatus: true,
      robots: { R01: { rosRevision: 21, gazeboRevision: 21, nav2Revision: 21,
        tagMapRevision: 21, tfStatus: true, error: null, status: "SYNCED" } },
    },
  });
  useStore.getState().setRobotDetail("R01", { runtimeMapSnapshot: mapSnapshot });
}

function setNavReadyCapabilities() {
  useStore.setState({ robotCapabilities: { R01: {
    mapping_available: true, mapping_active: false, nav2_available: true, nav2_ready: true,
    manual_available: false, goal_available: true, goal_blocker_code: null,
    goal_blocker_reason: null, map_ready: true, tag_navigation_available: false,
    registration_revision: null, registration_source: null,
  } }, rosDiagnostics: {
    ros: true, gazebo: true, controller_manager: true, slam: false, nav2: true,
    nav2_ready: true, nav2_actions_ready: true, nav2_lifecycle_ready: true,
    nav2_lifecycle_states: { map_server: "active", controller_server: "active", planner_server: "active",
      behavior_server: "active", bt_navigator: "active", waypoint_follower: "active" },
    tf: true, lidar: true, nodes: [], topics: [], controllers: [],
    command_ownership: { estop_active: false }, simulation_time: null, last_update_at: new Date().toISOString(),
  } });
}

function setNavBlockedCapabilities() {
  const current = useStore.getState();
  const capabilities = current.robotCapabilities?.R01;
  useStore.setState({
    robotCapabilities: capabilities ? { ...current.robotCapabilities, R01: { ...capabilities,
      nav2_ready: false, goal_available: false, goal_blocker_code: "NAV2_LIFECYCLE_NOT_ACTIVE",
      goal_blocker_reason: "Required Nav2 lifecycle node is inactive." } } : current.robotCapabilities,
    rosDiagnostics: current.rosDiagnostics ? { ...current.rosDiagnostics, nav2_ready: false,
      nav2_lifecycle_ready: false, nav2_lifecycle_blocker_code: "NAV2_LIFECYCLE_NOT_ACTIVE",
      nav2_lifecycle_blocker_reason: "Required Nav2 lifecycle node is inactive." } : current.rosDiagnostics,
  });
}


function buttonNamed(name: string): HTMLButtonElement | undefined {
  const button = Array.from(container.querySelectorAll("button")).find((candidate) =>
    candidate.getAttribute("aria-label") === name || candidate.textContent?.trim() === name);
  if (button) return button;
  const link = container.querySelector<HTMLAnchorElement>(`.wt-sidebar-item[aria-label="${name}"]`);
  return link ? link as unknown as HTMLButtonElement : undefined;
}

function dashboardStatus(panelTitle: string, label: string): string | undefined {
  const panel = container.querySelector<HTMLElement>(`.hmi-dashboard-status-panel[aria-label="${panelTitle}"]`);
  const expand = panel?.querySelector<HTMLButtonElement>(`button[aria-label="Expand ${panelTitle}"]`);
  if (expand && panel?.querySelector<HTMLElement>(".hmi-floating-content")?.dataset.expanded !== "true") {
    act(() => expand.click());
  }
  const term = Array.from(panel?.querySelectorAll("dt") ?? []).find((element) => element.textContent?.trim() === label);
  return term?.nextElementSibling?.textContent?.trim();
}

function setInputValue(input: HTMLInputElement, value: string) {
  const setter = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value")?.set;
  setter?.call(input, value);
  input.dispatchEvent(new Event("input", { bubbles: true }));
  input.dispatchEvent(new Event("change", { bubbles: true }));
}

function settleUi() { return new Promise<void>((resolve) => window.setTimeout(resolve, 0)); }

function canvasWorldPixel(canvas: HTMLCanvasElement, world: { x: number; y: number }) {
  const width = Number(canvas.dataset.viewportWidth);
  const height = Number(canvas.dataset.viewportHeight);
  const centerX = Number(canvas.dataset.viewportCenterX);
  const centerY = Number(canvas.dataset.viewportCenterY);
  const scale = Number(canvas.dataset.viewportScalePxPerMeter);
  return { x: width / 2 + (world.x - centerX) * scale, y: height / 2 - (world.y - centerY) * scale };
}

function canvasScreenWorld(canvas: HTMLCanvasElement, screen: { x: number; y: number }) {
  const width = Number(canvas.dataset.viewportWidth);
  const height = Number(canvas.dataset.viewportHeight);
  const centerX = Number(canvas.dataset.viewportCenterX);
  const centerY = Number(canvas.dataset.viewportCenterY);
  const scale = Number(canvas.dataset.viewportScalePxPerMeter);
  return { x: centerX + (screen.x - width / 2) / scale, y: centerY - (screen.y - height / 2) / scale };
}

function mapPointer(canvas: HTMLCanvasElement, type: string, x: number, y: number) {
  const event = new MouseEvent(type, { bubbles: true, cancelable: true, button: 0, clientX: x, clientY: y });
  Object.defineProperty(event, "pointerId", { value: 1 });
  canvas.dispatchEvent(event);
}

function manualPointer(button: HTMLButtonElement, type: string) {
  // React's onPointerLeave is synthesized from the native pointerout event.
  mapPointer(button as unknown as HTMLCanvasElement, type === "pointerleave" ? "pointerout" : type, 0, 0);
}

beforeEach(() => {
  globalThis.IS_REACT_ACT_ENVIRONMENT = true;
  vi.stubGlobal("Worker", undefined);
  window.history.replaceState({}, "", "/control");
  useStore.setState({ ...initialState, localRobotId: "R01", localIdentityVerified: true, twin: { ...initialState.twin, robots: { R01: r01() } }, quickDetailRobotId: null, robotDetail: {} });
  container = document.createElement("div");
  document.body.appendChild(container);
  root = createRoot(container);
  vi.spyOn(HTMLCanvasElement.prototype, "getContext").mockReturnValue(null);
  vi.spyOn(occupancyRasters, "get").mockResolvedValue(document.createElement("canvas"));
  vi.spyOn(HTMLElement.prototype, "clientWidth", "get").mockReturnValue(640);
  vi.spyOn(HTMLElement.prototype, "clientHeight", "get").mockReturnValue(360);
  vi.spyOn(HTMLCanvasElement.prototype, "getBoundingClientRect").mockReturnValue({
    x: 0, y: 0, left: 0, top: 0, right: 640, bottom: 360, width: 640, height: 360,
    toJSON: () => ({}),
  } as DOMRect);
  vi.mocked(wsSend).mockClear();
  vi.mocked(wsAcquireManual).mockClear();
  vi.mocked(wsManualCommand).mockClear();
  vi.mocked(wsSetRobotMode).mockClear();
  vi.mocked(api.initializeLocalRobotPose).mockReset();
  vi.mocked(api.emergencyStop).mockReset().mockResolvedValue({ ok: true });
  vi.mocked(api.clearEmergencyStop).mockReset().mockResolvedValue({ ok: true, code: "CLEAR_ESTOP_APPLIED",
    robot_id: "R01", emergency_stop_active: false, pre_stop_navigation_terminal: true });
});

afterEach(() => {
  act(() => root.unmount());
  container.remove();
  useStore.setState(initialState);
  vi.mocked(api.getLocalRobotMaps).mockReset().mockResolvedValue({
    robot_id: "R01", maps: [], runtime_mode: "GAZEBO_ROS", mapping_state: "MAPPING",
    mapping_duration_s: 0, active_local_map_id: null, local_active_map_id: null,
    local_active_map_revision: null, active_map_id: null, active_map_revision: null,
    canonical_map_revision: null, map_sync_status: null, robot_control_mode: "MANUAL", robot_stopped: false,
  });
  vi.mocked(api.getLocalRuntimeMode).mockReset().mockResolvedValue({
    robot_id: "R01", current_mode: "MAPPING",
    transition: { robot_id: "R01", mode: "mapping", status: "READY" },
  });
  vi.mocked(api.loadLocalRobotMap).mockReset();
  vi.useRealTimers();
  vi.restoreAllMocks();
  vi.unstubAllGlobals();
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

  it("suppresses the map context menu without cancelling an idle POINT selection", async () => {
    setOnlineRobot();
    setNavReadyCapabilities();
    useStore.setState({ twin: { ...initialState.twin, robots: { R01: { ...r01(), control_mode: "AUTONOMOUS" } } } });
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { await settleUi(); await settleUi(); });
    const canvas = container.querySelector<HTMLCanvasElement>('[data-testid="active-navigation-map-2d"] canvas')!;
    vi.mocked(wsSend).mockClear();

    const contextMenu = new MouseEvent("contextmenu", { bubbles: true, cancelable: true, button: 2,
      clientX: 320, clientY: 180 });
    await act(async () => { canvas.dispatchEvent(contextMenu); });

    expect(contextMenu.defaultPrevented).toBe(true);
    expect(canvas.dataset.pointSelectionState).toBe("WAITING_FOR_DESTINATION");
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).not.toContain("PATH_PREVIEW_INVALIDATE");
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).not.toContain("NAV_CANCEL");
  });

  it("right-click cancels SELECT YAW and returns POINT mode to waiting", async () => {
    setOnlineRobot();
    setNavReadyCapabilities();
    useStore.setState({ twin: { ...initialState.twin, robots: { R01: { ...r01(), control_mode: "AUTONOMOUS" } } } });
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { await settleUi(); await settleUi(); });
    const canvas = container.querySelector<HTMLCanvasElement>('[data-testid="active-navigation-map-2d"] canvas')!;
    await act(async () => { canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    expect(canvas.dataset.pointSelectionState).toBe("SELECT_YAW");
    vi.mocked(wsSend).mockClear();

    const contextMenu = new MouseEvent("contextmenu", { bubbles: true, cancelable: true, button: 2,
      clientX: 320, clientY: 180 });
    await act(async () => { canvas.dispatchEvent(contextMenu); await settleUi(); });

    expect(contextMenu.defaultPrevented).toBe(true);
    expect(canvas.dataset.pointSelectionState).toBe("WAITING_FOR_DESTINATION");
    expect(container.querySelector('[data-testid="selected-point-status"]')).toBeNull();
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).toContain("PATH_PREVIEW_INVALIDATE");
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).not.toContain("NAV_CANCEL");
  });

  it("right-click cancels a PLANNING preview and clears the selected target", async () => {
    setOnlineRobot();
    setNavReadyCapabilities();
    useStore.setState({ twin: { ...initialState.twin, robots: { R01: { ...r01(), control_mode: "AUTONOMOUS" } } } });
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { await settleUi(); await settleUi(); });
    const canvas = container.querySelector<HTMLCanvasElement>('[data-testid="active-navigation-map-2d"] canvas')!;
    await act(async () => { canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    await act(async () => { canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 380, clientY: 220 })); });
    expect(container.querySelector('[data-testid="point-navigation-state"]')?.textContent).toBe("PLANNING");
    vi.mocked(wsSend).mockClear();

    const contextMenu = new MouseEvent("contextmenu", { bubbles: true, cancelable: true, button: 2,
      clientX: 380, clientY: 220 });
    await act(async () => { canvas.dispatchEvent(contextMenu); await settleUi(); });

    expect(contextMenu.defaultPrevented).toBe(true);
    expect(container.querySelector('[data-testid="point-navigation-state"]')?.textContent).toBe("WAITING FOR DESTINATION");
    expect(container.querySelector('[data-testid="selected-point-status"]')).toBeNull();
    expect(useStore.getState().robotDetail.R01?.pathPreview).toBeNull();
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).toContain("PATH_PREVIEW_INVALIDATE");
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).not.toContain("NAV_CANCEL");
  });

  it("right-click cancels a valid preview but never cancels an already-sent Nav2 goal", async () => {
    setOnlineRobot();
    setNavReadyCapabilities();
    useStore.setState({ twin: { ...initialState.twin, robots: { R01: { ...r01(), control_mode: "AUTONOMOUS" } } } });
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { await settleUi(); await settleUi(); });
    const canvas = container.querySelector<HTMLCanvasElement>('[data-testid="active-navigation-map-2d"] canvas')!;
    await act(async () => { canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    await act(async () => { canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 380, clientY: 220 })); });
    const request = vi.mocked(wsSend).mock.calls.map(([message]) => message)
      .find((message) => message.type === "PATH_PREVIEW_REQUEST");
    if (!request || request.type !== "PATH_PREVIEW_REQUEST") throw new Error("path preview request was not emitted");
    const approved = {
      robot_id: "R01", request_id: request.request_id, status: "VALID" as const,
      source_type: "ACTIVE_MAP_POINT" as const, source_map_id: "NAV2-R01-map", source_map_revision: "nav2-r21",
      frame_id: "map" as const, path: [[0, 0], [1, 1]] as Array<[number, number]>, path_length_m: 1.4,
      goal: { x: request.x!, y: request.y!, yaw: request.yaw! }, timestamp: new Date().toISOString(),
      active_map_id: "NAV2-R01-map", active_map_revision: "nav2-r21",
    };
    act(() => useStore.getState().setRobotDetail("R01", { pathPreview: approved }));
    expect(container.querySelector('[data-testid="point-navigation-state"]')?.textContent).toContain("PREVIEW VALID");
    vi.mocked(wsSend).mockClear();

    const contextMenu = new MouseEvent("contextmenu", { bubbles: true, cancelable: true, button: 2,
      clientX: 380, clientY: 220 });
    await act(async () => { canvas.dispatchEvent(contextMenu); await settleUi(); });

    expect(contextMenu.defaultPrevented).toBe(true);
    expect(container.querySelector('[data-testid="point-navigation-state"]')?.textContent).toBe("WAITING FOR DESTINATION");
    expect(useStore.getState().robotDetail.R01?.pathPreview).toBeNull();
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).toContain("PATH_PREVIEW_INVALIDATE");
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).not.toContain("NAV_CANCEL");

    // Start and approve another goal, send it, then verify right-click cannot
    // reach the separate NAV_CANCEL command path.
    await act(async () => { canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    await act(async () => { canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 380, clientY: 220 })); });
    const nextRequest = vi.mocked(wsSend).mock.calls.map(([message]) => message)
      .filter((message) => message.type === "PATH_PREVIEW_REQUEST").at(-1);
    if (!nextRequest || nextRequest.type !== "PATH_PREVIEW_REQUEST") throw new Error("replacement path preview request was not emitted");
    act(() => useStore.getState().setRobotDetail("R01", { pathPreview: { ...approved,
      request_id: nextRequest.request_id,
      goal: { x: nextRequest.x!, y: nextRequest.y!, yaw: nextRequest.yaw! },
    } }));
    await act(async () => { buttonNamed("SEND GOAL")?.click(); });
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).toContain("NAV_GOAL");
    vi.mocked(wsSend).mockClear();
    const activeGoalContextMenu = new MouseEvent("contextmenu", { bubbles: true, cancelable: true, button: 2,
      clientX: 380, clientY: 220 });
    await act(async () => { canvas.dispatchEvent(activeGoalContextMenu); });
    expect(activeGoalContextMenu.defaultPrevented).toBe(true);
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).not.toContain("NAV_CANCEL");
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).not.toContain("PATH_PREVIEW_INVALIDATE");
  });

  it("uses PAN and POINT modes for drag, click, yaw preview, and Ctrl-wheel zoom", async () => {
    setOnlineRobot();
    setNavReadyCapabilities();
    useStore.setState({ twin: { ...useStore.getState().twin, robots: { R01: { ...r01(), control_mode: "AUTONOMOUS" } } } });
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { await settleUi(); });
    const canvas = container.querySelector<HTMLCanvasElement>(".local-pose-map canvas")!;
    const capture = vi.fn();
    Object.assign(canvas, { setPointerCapture: capture, hasPointerCapture: () => true, releasePointerCapture: vi.fn() });
    expect(buttonNamed("POINT")?.getAttribute("aria-pressed")).toBe("true");
    expect(buttonNamed("PAN")?.getAttribute("aria-pressed")).toBe("false");
    act(() => {
      mapPointer(canvas, "pointerdown", 400, 200);
      mapPointer(canvas, "pointermove", 402, 201);
      mapPointer(canvas, "pointerup", 402, 201);
      canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 402, clientY: 201 }));
    });
    expect(capture).toHaveBeenCalledWith(1);
    expect(container.querySelector('[data-testid="selected-point-status"]')).toBeTruthy();
    const pointCoordinates = Array.from(container.querySelectorAll(".hmi-coordinate-grid b")).map((node) => node.textContent);
    const clickedPoint = canvasScreenWorld(canvas, { x: 402, y: 201 });
    expect(Number.parseFloat(pointCoordinates[0]!)).toBeCloseTo(clickedPoint.x, 2);
    expect(Number.parseFloat(pointCoordinates[1]!)).toBeCloseTo(clickedPoint.y, 2);
    expect(pointCoordinates[2]).toContain("0.00");
    act(() => buttonNamed("CANCEL")?.click());
    act(() => buttonNamed("PAN")?.click());
    expect(buttonNamed("PAN")?.getAttribute("aria-pressed")).toBe("true");
    expect(buttonNamed("POINT")?.getAttribute("aria-pressed")).toBe("false");
    const before = { x: Number(canvas.dataset.viewportCenterX), y: Number(canvas.dataset.viewportCenterY), scale: Number(canvas.dataset.viewportScalePxPerMeter) };
    act(() => {
      mapPointer(canvas, "pointerdown", 400, 200);
      mapPointer(canvas, "pointermove", 500, 250);
      mapPointer(canvas, "pointerup", 500, 250);
      canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 500, clientY: 250 }));
    });
    expect(Number(canvas.dataset.viewportCenterX)).toBeCloseTo(before.x - 100 / before.scale);
    expect(Number(canvas.dataset.viewportCenterY)).toBeCloseTo(before.y + 50 / before.scale);
    expect(container.querySelector('[data-testid="selected-point-status"]')).toBeNull();
    const normalWheel = new WheelEvent("wheel", { bubbles: true, cancelable: true, deltaY: -120, clientX: 450, clientY: 220 });
    act(() => canvas.dispatchEvent(normalWheel));
    expect(normalWheel.defaultPrevented).toBe(false);
    expect(Number(canvas.dataset.viewportScalePxPerMeter)).toBe(before.scale);
    const anchoredWorld = { x: Number(canvas.dataset.viewportCenterX) + (450 - 320) / before.scale,
      y: Number(canvas.dataset.viewportCenterY) - (220 - 180) / before.scale };
    const ctrlWheel = new WheelEvent("wheel", { bubbles: true, cancelable: true, ctrlKey: true, deltaY: -120, clientX: 450, clientY: 220 });
    act(() => canvas.dispatchEvent(ctrlWheel));
    expect(ctrlWheel.defaultPrevented).toBe(true);
    expect(Number(canvas.dataset.viewportScalePxPerMeter)).toBeCloseTo(before.scale * 1.12);
    const pixel = canvasWorldPixel(canvas, anchoredWorld);
    expect(Math.hypot(pixel.x - 450, pixel.y - 220)).toBeLessThan(1);

    act(() => { buttonNamed("FIT")?.click(); buttonNamed("POINT")?.click(); });
    expect(buttonNamed("POINT")?.getAttribute("aria-pressed")).toBe("true");
    act(() => canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })));
    expect(canvas.dataset.pointSelectionState).toBe("SELECT_YAW");
    act(() => mapPointer(canvas, "pointermove", 350, 200));
    expect(canvas.dataset.directionPreview).toBe("visible");
    const startWorld = canvasScreenWorld(canvas, { x: 320, y: 180 });
    const endWorld = canvasScreenWorld(canvas, { x: 350, y: 200 });
    const selectedYaw = Math.atan2(endWorld.y - startWorld.y, endWorld.x - startWorld.x);
    expect(Number(canvas.dataset.directionPreviewYaw)).toBeCloseTo(selectedYaw, 6);
    act(() => canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 350, clientY: 200 })));
    expect(canvas.dataset.directionPreview).toBe("hidden");
    const yawText = Array.from(container.querySelectorAll(".hmi-coordinate-grid b"))[2]?.textContent ?? "";
    expect(Number.parseFloat(yawText)).toBeCloseTo(selectedYaw, 2);
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).toContain("PATH_PREVIEW_REQUEST");
    const pointModeScale = Number(canvas.dataset.viewportScalePxPerMeter);
    const noCtrlInPointMode = new WheelEvent("wheel", { bubbles: true, cancelable: true, deltaY: 120, clientX: 450, clientY: 220 });
    act(() => canvas.dispatchEvent(noCtrlInPointMode));
    expect(noCtrlInPointMode.defaultPrevented).toBe(false);
    expect(Number(canvas.dataset.viewportScalePxPerMeter)).toBe(pointModeScale);
    const ctrlInPointMode = new WheelEvent("wheel", { bubbles: true, cancelable: true, ctrlKey: true, deltaY: -120, clientX: 450, clientY: 220 });
    act(() => canvas.dispatchEvent(ctrlInPointMode));
    expect(ctrlInPointMode.defaultPrevented).toBe(true);
    expect(Number(canvas.dataset.viewportScalePxPerMeter)).toBeCloseTo(pointModeScale * 1.12);
  });

  it("accepts POINT selections only from known-free occupancy cells", async () => {
    setOnlineRobot();
    setNavReadyCapabilities();
    useStore.setState({ twin: { ...useStore.getState().twin, robots: { R01: { ...r01(), control_mode: "AUTONOMOUS" } } } });
    const snapshot = useStore.getState().robotDetail.R01.runtimeMapSnapshot!;
    useStore.getState().setRobotDetail("R01", { runtimeMapSnapshot: { ...snapshot, data: Array(100).fill(100) } });
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { await settleUi(); });
    const canvas = container.querySelector<HTMLCanvasElement>(".local-pose-map canvas")!;
    await act(async () => { canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    expect(container.querySelector('[data-testid="selected-point-status"]')).toBeNull();
    expect(buttonNamed("SEND GOAL")?.disabled).toBe(true);
    expect(buttonNamed("PREVIEW PATH")).toBeUndefined();

    const occupiedSnapshot = useStore.getState().robotDetail.R01.runtimeMapSnapshot!;
    await act(async () => {
      useStore.getState().setRobotDetail("R01", { runtimeMapSnapshot: { ...occupiedSnapshot, data: Array(100).fill(0) } });
      await settleUi();
    });
    await act(async () => { canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    expect(container.querySelector('[data-testid="selected-point-status"]')).toBeTruthy();
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).not.toContain("PATH_PREVIEW_REQUEST");
    await act(async () => { canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).toContain("PATH_PREVIEW_REQUEST");
  });

  it("cancels a pending point and yaw transaction when switching to PAN", async () => {
    setOnlineRobot();
    setNavReadyCapabilities();
    useStore.setState({ twin: { ...useStore.getState().twin, robots: { R01: { ...r01(), control_mode: "AUTONOMOUS" } } } });
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { await settleUi(); });
    const canvas = container.querySelector<HTMLCanvasElement>(".local-pose-map canvas")!;
    await act(async () => { canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    expect(container.querySelector('[data-testid="selected-point-status"]')?.textContent).toBe("SELECT YAW");
    expect(canvas.dataset.pointSelectionState).toBe("SELECT_YAW");

    await act(async () => { buttonNamed("PAN")?.click(); });
    expect(container.querySelector('[data-testid="selected-point-status"]')).toBeNull();
    expect(canvas.dataset.directionPreview).toBe("hidden");
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).toContain("PATH_PREVIEW_INVALIDATE");
    await act(async () => { buttonNamed("POINT")?.click(); });
    expect(canvas.dataset.pointSelectionState).toBe("WAITING_FOR_DESTINATION");
  });

  it("uses one map workspace and shows the manual panel only in authoritative MANUAL mode", async () => {
    setOnlineRobot();
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { await settleUi(); });

    const main = container.querySelector<HTMLElement>(".hmi-control-main");
    expect(main?.children[0]?.classList.contains("robot-detail-map-panel")).toBe(true);
    expect(main?.children).toHaveLength(1);
    const operation = main?.querySelector<HTMLElement>('[aria-label="Robot operation panel"]');
    expect(operation).toBeNull();
    const manualPanel = main?.querySelector<HTMLElement>('[data-testid="manual-jog-panel"]');
    expect(manualPanel?.parentElement).toBe(main?.querySelector('[data-testid="control-operation-panels"]'));
    expect(manualPanel?.dataset.expanded).toBe("true");
    expect(container.querySelector(".robot-map-source-bar")).toBeNull();
    expect(container.querySelectorAll('[data-testid="manual-jog-panel"]')).toHaveLength(1);
    expect(main?.querySelectorAll(".manual-key")).toHaveLength(11);
    expect(buttonNamed("YAW +")).toBeUndefined();
    expect(buttonNamed("YAW −")).toBeUndefined();
    expect(buttonNamed("PAN")?.getAttribute("aria-pressed")).toBe("false");
    expect(buttonNamed("POINT")?.getAttribute("aria-pressed")).toBe("true");
    const errorLogButton = container.querySelector<HTMLButtonElement>('[data-testid="control-error-log-trigger"]');
    const mapToolbar = container.querySelector<HTMLElement>('[data-testid="robot-map-toolbar"]');
    const centerRobotButton = buttonNamed("CENTER ROBOT");
    expect(errorLogButton?.textContent?.trim()).toBe("ERROR LOG");
    expect(centerRobotButton?.parentElement).toBe(mapToolbar);
    expect(centerRobotButton?.nextElementSibling).toBe(errorLogButton);
    expect(errorLogButton?.classList.contains("is-neutral")).toBe(true);
    expect(container.querySelector('[data-testid="control-error-log"]')).toBeNull();
    expect(container.querySelector(".hmi-error-log-footer")).toBeNull();
    expect(container.querySelectorAll('[data-testid="control-error-log-trigger"]')).toHaveLength(1);
    expect(container.querySelectorAll(".hmi-dashboard-status-panel")).toHaveLength(2);
    expect(container.textContent).toContain("SYSTEM INPUTS");
    expect(container.textContent).toContain("ACTIVE MAP");
    expect(container.textContent).toContain("STATE");
  });

  it("folds and reopens each overlay without remounting the map or changing hold state", () => {
    setOnlineRobot();
    renderNode(<ControlDetailHarness robotId="R01" />);
    const canvas = container.querySelector("canvas");
    act(() => manualPointer(container.querySelector<HTMLButtonElement>(".manual-key-forward")!, "pointerdown"));
    const manualPanel = container.querySelector('[data-testid="manual-jog-panel"]');
    expect(manualPanel?.textContent).toContain("FORWARD · HOLDING");
    vi.mocked(wsManualCommand).mockClear();
    expect(container.querySelector('[aria-label="Robot operation panel"]')).toBeNull();
    for (const [title, initiallyExpanded] of [["SYSTEM INPUTS", true], ["STATE", true], ["Manual jog", true]] as const) {
      const toggle = buttonNamed(`${initiallyExpanded ? "Collapse" : "Expand"} ${title}`)!;
      const content = document.getElementById(toggle.getAttribute("aria-controls")!)!;
      expect(content.dataset.expanded).toBe(String(initiallyExpanded));
      expect(content.getAttribute("aria-hidden")).toBe(String(!initiallyExpanded));
      expect(content.hasAttribute("inert")).toBe(!initiallyExpanded);
      expect(content.hidden).toBe(false);
      act(() => toggle.click());
      expect(toggle.getAttribute("aria-expanded")).toBe(String(!initiallyExpanded));
      expect(content.dataset.expanded).toBe(String(!initiallyExpanded));
      expect(content.getAttribute("aria-hidden")).toBe(String(initiallyExpanded));
      expect(content.hasAttribute("inert")).toBe(initiallyExpanded);
      act(() => toggle.click());
      expect(content.dataset.expanded).toBe(String(initiallyExpanded));
      expect(content.hidden).toBe(false);
      expect(container.querySelector("canvas")).toBe(canvas);
    }
    expect(container.querySelector('[data-testid="manual-jog-panel"]')).toBe(manualPanel);
    expect(manualPanel?.textContent).toContain("FORWARD · HOLDING");
    expect(wsManualCommand).not.toHaveBeenCalled();
    act(() => buttonNamed("Stop")?.click());
    expect(wsManualCommand).toHaveBeenLastCalledWith("R01", "STOP", expect.any(String));
  });

  it("switches floating operation panels only after authoritative mode confirmation and expands the newly active panel", async () => {
    setOnlineRobot();
    renderNode(<ControlDetailHarness robotId="R01" />);
    const manual = container.querySelector<HTMLElement>('[data-testid="manual-jog-panel"]');
    expect(manual).toBeTruthy();
    expect(container.querySelector('[aria-label="Robot operation panel"]')).toBeNull();

    act(() => buttonNamed("Collapse Manual jog")?.click());
    expect(manual?.dataset.expanded).toBe("false");
    act(() => buttonNamed("AUTONOMOUS")?.click());
    expect(wsSetRobotMode).toHaveBeenLastCalledWith("R01", "AUTONOMOUS");
    expect(container.querySelector('[data-testid="manual-jog-panel"]')).toBeTruthy();
    expect(container.querySelector('[aria-label="Robot operation panel"]')).toBeNull();

    await act(async () => {
      useStore.getState().setRobotDetail("R01", { appliedMode: "AUTONOMOUS", modeTransitionState: "APPLIED" });
      await settleUi();
    });
    const operation = container.querySelector<HTMLElement>('[aria-label="Robot operation panel"]');
    expect(operation?.dataset.expanded).toBe("true");
    expect(container.querySelector('[data-testid="manual-jog-panel"]')).toBeNull();

    act(() => buttonNamed("Collapse Mission")?.click());
    expect(operation?.dataset.expanded).toBe("false");
    act(() => buttonNamed("MANUAL")?.click());
    expect(container.querySelector('[aria-label="Robot operation panel"]')).toBeTruthy();
    expect(container.querySelector('[data-testid="manual-jog-panel"]')).toBeNull();
    await act(async () => {
      useStore.getState().setRobotDetail("R01", { appliedMode: "MANUAL", modeTransitionState: "APPLIED" });
      await settleUi();
    });
    expect(container.querySelector('[aria-label="Robot operation panel"]')).toBeNull();
    expect(container.querySelector<HTMLElement>('[data-testid="manual-jog-panel"]')?.dataset.expanded).toBe("true");
  });

  it("opens the error modal with newest real runtime errors first and uses a dash when no source is provided", () => {
    setOnlineRobot();
    useStore.getState().setRobotDetail("R01", { errors: [
      { severity: "WARNING", message: "Older diagnostic", timestamp: "2026-10-07T10:00:00Z" },
      { severity: "ERROR", message: "Latest runtime error", timestamp: "2026-10-07T10:02:00Z" },
    ] });
    renderNode(<ControlDetailHarness robotId="R01" />);
    const errorLogButton = container.querySelector<HTMLButtonElement>('[data-testid="control-error-log-trigger"]');
    expect(errorLogButton?.textContent?.trim()).toBe("ERROR LOG (2)");
    expect(errorLogButton?.classList.contains("is-error")).toBe(true);
    act(() => errorLogButton?.click());

    const rows = Array.from(container.querySelectorAll<HTMLTableRowElement>(".hmi-error-log-table tbody tr"));
    expect(container.querySelector('[role="dialog"][aria-label="Error log"]')).toBeTruthy();
    expect(rows).toHaveLength(2);
    expect(rows[0]?.textContent).toContain("Latest runtime error");
    expect(rows[0]?.textContent).toContain("ERROR");
    expect(rows[0]?.textContent).toContain("—");
    expect(rows[1]?.textContent).toContain("Older diagnostic");
    expect(rows[1]?.querySelector(".hmi-error-level")?.classList.contains("is-warning")).toBe(true);
  });

  it("closes the Error Log modal by its close button, Escape, and backdrop without closing on inside clicks", () => {
    setOnlineRobot();
    renderNode(<ControlDetailHarness robotId="R01" />);
    const open = () => act(() => container.querySelector<HTMLButtonElement>('[data-testid="control-error-log-trigger"]')?.click());
    open();
    expect(container.querySelector('[role="dialog"][aria-label="Error log"]')).toBeTruthy();
    act(() => container.querySelector<HTMLElement>('[role="dialog"]')?.click());
    expect(container.querySelector('[role="dialog"][aria-label="Error log"]')).toBeTruthy();
    act(() => buttonNamed("Close error log")?.click());
    expect(container.querySelector('[role="dialog"][aria-label="Error log"]')).toBeNull();

    open();
    act(() => document.dispatchEvent(new KeyboardEvent("keydown", { key: "Escape", bubbles: true })));
    expect(container.querySelector('[role="dialog"][aria-label="Error log"]')).toBeNull();

    open();
    act(() => container.querySelector<HTMLElement>('[data-testid="control-error-log-backdrop"]')?.click());
    expect(container.querySelector('[role="dialog"][aria-label="Error log"]')).toBeNull();
    expect(container.querySelectorAll('[data-testid="control-error-log-trigger"]')).toHaveLength(1);
    expect(container.querySelector(".hmi-error-log-footer")).toBeNull();
  });

  it("keeps all eleven manual actions in the right floating group and sends their existing commands", () => {
    setOnlineRobot("MAPPING");
    renderNode(<ControlDetailHarness robotId="R01" />);
    const actions = [
      ["forward_left", "FORWARD_LEFT"], ["forward", "FORWARD"], ["forward_right", "FORWARD_RIGHT"],
      ["left", "LEFT"], ["stop", "STOP"], ["right", "RIGHT"],
      ["backward_left", "BACKWARD_LEFT"], ["backward", "BACKWARD"], ["backward_right", "BACKWARD_RIGHT"],
      ["rotate_left", "ROTATE_LEFT"], ["rotate_right", "ROTATE_RIGHT"],
    ] as const;
    for (const [buttonClass, action] of actions) {
      const button = container.querySelector<HTMLButtonElement>(`.hmi-manual-jog-panel .manual-key-${buttonClass}`);
      expect(button).toBeTruthy();
      expect(button?.disabled).toBe(false);
      if (action === "STOP") {
        act(() => button?.click());
        expect(wsManualCommand).toHaveBeenLastCalledWith("R01", "STOP", undefined);
      } else {
        act(() => manualPointer(button!, "pointerdown"));
        expect(wsManualCommand).toHaveBeenLastCalledWith("R01", action, expect.any(String));
        act(() => manualPointer(button!, "pointerup"));
        expect(wsManualCommand).toHaveBeenLastCalledWith("R01", "STOP", expect.any(String));
      }
    }
  });

  it("keeps applied mode while a requested transition is pending", () => {
    setOnlineRobot();
    renderNode(<ControlDetailHarness robotId="R01" />);
    act(() => buttonNamed("AUTONOMOUS")?.click());
    expect(wsSetRobotMode).toHaveBeenCalledWith("R01", "AUTONOMOUS");
    expect(buttonNamed("MANUAL")?.className).toContain("bg-blue-100");
    expect(buttonNamed("AUTONOMOUS")?.className).not.toContain("bg-blue-100");
    act(() => useStore.getState().setRobotDetail("R01", { appliedMode: "AUTONOMOUS", modeTransitionState: "APPLIED" }));
    expect(buttonNamed("AUTONOMOUS")?.className).toContain("bg-blue-100");
  });

  it("shows CLEAR STOP as applied only after the correlated runtime result", async () => {
    setOnlineRobot();
    renderNode(<ControlDetailHarness robotId="R01" />);
    vi.mocked(api.clearEmergencyStop).mockResolvedValueOnce({ ok: true, code: "CLEAR_ESTOP_APPLIED",
      robot_id: "R01", emergency_stop_active: false, pre_stop_navigation_terminal: true });
    await act(async () => { buttonNamed("CLEAR STOP")?.click(); await settleUi(); });
    expect(container.querySelector('[role="status"]')?.textContent)
      .toContain("CLEAR_ESTOP_APPLIED · bridge confirmed latch clear");

    vi.mocked(api.clearEmergencyStop).mockRejectedValueOnce(new Error("CLEAR_ESTOP_REJECTED_GOAL_PENDING"));
    await act(async () => { buttonNamed("CLEAR STOP")?.click(); await settleUi(); });
    expect(container.textContent).toContain("CLEAR_ESTOP_REJECTED_GOAL_PENDING");
    expect(container.querySelector('[role="status"]')?.textContent ?? "").not.toContain("CLEAR_ESTOP_APPLIED");
  });

  it("keeps EMERGENCY STOP available while CLEAR STOP is pending and coalesces duplicate E-STOP clicks", async () => {
    setOnlineRobot();
    let resolveClear!: (value: Awaited<ReturnType<typeof api.clearEmergencyStop>>) => void;
    let resolveEmergency!: (value: { ok: true }) => void;
    vi.mocked(api.clearEmergencyStop).mockReturnValueOnce(new Promise((resolve) => { resolveClear = resolve; }));
    vi.mocked(api.emergencyStop).mockReturnValueOnce(new Promise((resolve) => { resolveEmergency = resolve; }));
    renderNode(<ControlDetailHarness robotId="R01" />);

    await act(async () => { buttonNamed("CLEAR STOP")?.click(); await settleUi(); });
    const emergencyButton = buttonNamed("EMERGENCY STOP");
    expect(buttonNamed("CLEAR STOP")?.disabled).toBe(true);
    expect(emergencyButton?.disabled).toBe(false);

    act(() => { emergencyButton?.click(); emergencyButton?.click(); });
    expect(api.emergencyStop).toHaveBeenCalledTimes(1);
    expect(emergencyButton?.disabled).toBe(true);

    await act(async () => { resolveEmergency({ ok: true }); await settleUi(); });
    expect(container.textContent).toContain("EMERGENCY STOP REQUEST ACCEPTED");
    resolveClear({ ok: true, code: "CLEAR_ESTOP_APPLIED", robot_id: "R01",
      emergency_stop_active: false, pre_stop_navigation_terminal: true });
    await act(async () => { await settleUi(); });
  });

  it("shows the actual E-STOP request failure when the backend is unavailable", async () => {
    setOnlineRobot();
    useStore.setState({ rosConnected: false, websocketState: "DISCONNECTED" });
    vi.mocked(api.emergencyStop).mockRejectedValueOnce(new Error("Failed to fetch"));
    renderNode(<ControlDetailHarness robotId="R01" />);

    const emergencyButton = buttonNamed("EMERGENCY STOP");
    expect(emergencyButton?.disabled).toBe(false);
    await act(async () => { emergencyButton?.click(); await settleUi(); });

    expect(container.querySelector('[role="alert"]')?.textContent).toContain("Failed to fetch");
    expect(container.textContent).not.toContain("EMERGENCY STOP REQUEST ACCEPTED");
  });

  it("blocks manual motion until fresh telemetry explicitly confirms E-STOP clear", () => {
    setOnlineRobot("MAPPING");
    useStore.setState({ rosDiagnostics: null, lastTelemetryAt: null });
    renderNode(<ControlDetailHarness robotId="R01" />);

    expect(dashboardStatus("STATE", "E-STOP")).toBe("UNKNOWN");
    const forward = container.querySelector<HTMLButtonElement>(".manual-key-forward")!;
    act(() => manualPointer(forward, "pointerdown"));
    expect(wsAcquireManual).not.toHaveBeenCalled();
    expect(wsManualCommand).not.toHaveBeenCalledWith("R01", "FORWARD", expect.anything());
  });

  it("reports stale E-STOP telemetry as UNKNOWN and fails closed for manual motion", () => {
    setOnlineRobot("MAPPING");
    const staleAt = new Date(Date.now() - 6_000).toISOString();
    const state = useStore.getState();
    useStore.setState({
      lastTelemetryAt: staleAt,
      twin: { ...state.twin, robots: { R01: { ...state.twin.robots.R01!, last_telemetry_at: staleAt } } },
    });
    renderNode(<ControlDetailHarness robotId="R01" />);

    expect(dashboardStatus("STATE", "E-STOP")).toBe("UNKNOWN");
    const forward = container.querySelector<HTMLButtonElement>(".manual-key-forward")!;
    act(() => manualPointer(forward, "pointerdown"));
    expect(wsAcquireManual).not.toHaveBeenCalled();
    expect(wsManualCommand).not.toHaveBeenCalledWith("R01", "FORWARD", expect.anything());
  });

  it("shows UNKNOWN in DIAGNOSIS until fresh false E-STOP telemetry arrives", async () => {
    setOnlineRobot();
    useStore.setState({ rosDiagnostics: null, lastTelemetryAt: null });
    window.history.replaceState({}, "", "/diagnosis");
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { await settleUi(); });

    const diagnosisEstopRow = () => Array.from(container.querySelectorAll("div"))
      .find((element) => element.firstElementChild?.textContent === "E-STOP"
        && element.querySelector(".robot-detail-status"));
    expect(diagnosisEstopRow()?.querySelector(".robot-detail-status")?.textContent).toContain("UNKNOWN");

    const freshAt = new Date().toISOString();
    act(() => useStore.setState({ lastTelemetryAt: freshAt,
      rosDiagnostics: { command_ownership: { estop_active: false } } as never }));
    expect(diagnosisEstopRow()?.querySelector(".robot-detail-status")?.textContent).toContain("CLEAR");
  });

  it("holds Mapping teleop only while the pointer is down and stops on release or cancellation", () => {
    setOnlineRobot("MAPPING");
    renderNode(<ControlDetailHarness robotId="R01" />);
    const forward = container.querySelector<HTMLButtonElement>(".manual-key-forward")!;
    expect(forward.disabled).toBe(false);
    act(() => manualPointer(forward, "pointerdown"));
    expect(wsAcquireManual).toHaveBeenLastCalledWith("R01", expect.any(String));
    expect(wsManualCommand).toHaveBeenLastCalledWith("R01", "FORWARD", expect.any(String));
    expect(forward.getAttribute("aria-pressed")).toBe("true");
    act(() => manualPointer(forward, "pointerup"));
    expect(wsManualCommand).toHaveBeenLastCalledWith("R01", "STOP", expect.any(String));
    expect(forward.getAttribute("aria-pressed")).toBe("false");

    for (const release of ["pointercancel", "pointerleave", "lostpointercapture"]) {
      act(() => manualPointer(forward, "pointerdown"));
      act(() => manualPointer(forward, release));
      expect(wsManualCommand.mock.calls.at(-1)?.[1], `release: ${release}`).toBe("STOP");
    }
    expect(forward.getAttribute("aria-pressed")).toBe("false");
  });
  it("sends explicit acquire, hold, and stop frames through the worker", async () => {
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
    setOnlineRobot();
    vi.mocked(wsManualCommand).mockClear();
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { await settleUi(); });
    expect(constructWorker).toHaveBeenCalledWith(expect.any(URL), { type: "module" });
    expect(postMessage).toHaveBeenCalledWith({ type: "CONNECT", url: "ws://127.0.0.1:8001/ws" });

    const forward = container.querySelector<HTMLButtonElement>(".manual-key-forward")!;
    const left = container.querySelector<HTMLButtonElement>(".manual-key-left")!;
    act(() => manualPointer(forward, "pointerdown"));
    expect(postMessage).toHaveBeenCalledWith({ type: "HOLD", robot_id: "R01", action: "FORWARD" });
    act(() => useStore.getState().setRobotDetail("R01", {
      appliedMode: "MANUAL", modeTransitionState: "APPLIED", modeRequestId: "manual-applied",
    }));
    expect(postMessage).not.toHaveBeenCalledWith({ type: "STOP", robot_id: "R01" });
    act(() => manualPointer(forward, "pointerup"));
    expect(postMessage).toHaveBeenCalledWith({ type: "STOP", robot_id: "R01" });
    expect(wsManualCommand).not.toHaveBeenCalled();

    act(() => root.unmount());
    root = createRoot(container);
    expect(postMessage).toHaveBeenCalledWith({ type: "DISCONNECT", robot_id: "R01" });
  });
  it("does not resume a worker command when E-STOP clears", () => {
    const workers: Array<{ messages: Array<Record<string, unknown>> }> = [];
    class FakeWorker {
      onmessage: ((event: MessageEvent) => void) | null = null;
      onerror: ((event: ErrorEvent) => void) | null = null;
      messages: Array<Record<string, unknown>> = [];
      postMessage = (request: Record<string, unknown>) => this.messages.push(request);
      terminate = vi.fn();
      constructor() { workers.push(this); }
    }
    vi.stubGlobal("Worker", FakeWorker);
    setOnlineRobot("MAPPING");
    renderNode(<ControlDetailHarness robotId="R01" />);
    const forward = container.querySelector<HTMLButtonElement>(".manual-key-forward")!;
    act(() => manualPointer(forward, "pointerdown"));
    act(() => useStore.getState().setRobotDetail("R01", {
      diagnostics: { command_ownership: { estop_active: true } } as never,
    }));
    expect(workers[0]?.messages).toContainEqual({ type: "STOP", robot_id: "R01" });
    const holdCount = workers[0]?.messages.filter((request) => request.type === "HOLD").length;

    act(() => useStore.getState().setRobotDetail("R01", {
      diagnostics: { command_ownership: { estop_active: false } } as never,
    }));

    expect(workers[0]?.messages.filter((request) => request.type === "HOLD")).toHaveLength(holdCount);
    expect(container.querySelector<HTMLButtonElement>(".manual-key-forward")?.getAttribute("aria-pressed")).toBe("false");
  });
  it("runs keyboard commands on keydown and stops on keyup; Space always stops", () => {
    setOnlineRobot("MAPPING");
    renderNode(<ControlDetailHarness robotId="R01" />);
    const down = (key: string, repeat = false) => window.dispatchEvent(new KeyboardEvent("keydown", { key, repeat, bubbles: true, cancelable: true }));
    act(() => down("w"));
    expect(wsManualCommand).toHaveBeenLastCalledWith("R01", "FORWARD", expect.any(String));
    act(() => window.dispatchEvent(new KeyboardEvent("keyup", { key: "w", bubbles: true, cancelable: true })));
    expect(wsManualCommand).toHaveBeenLastCalledWith("R01", "STOP", expect.any(String));
    const commandCount = vi.mocked(wsManualCommand).mock.calls.length;
    act(() => down("w", true));
    expect(wsManualCommand).toHaveBeenCalledTimes(commandCount);
    act(() => down("w"));
    expect(wsManualCommand).toHaveBeenLastCalledWith("R01", "FORWARD", expect.any(String));
    act(() => down("a"));
    act(() => window.dispatchEvent(new KeyboardEvent("keydown", { key: " ", code: "Space", bubbles: true, cancelable: true })));
    expect(wsManualCommand).toHaveBeenLastCalledWith("R01", "STOP", expect.any(String));
  });
  it("stops a held command before mode change, page exit, disconnect, and E-STOP", async () => {
    setOnlineRobot("MAPPING");
    renderNode(<ControlDetailHarness robotId="R01" />);
    const forward = container.querySelector<HTMLButtonElement>(".manual-key-forward")!;
    act(() => manualPointer(forward, "pointerdown"));
    act(() => buttonNamed("AUTONOMOUS")?.click());
    expect(wsManualCommand).toHaveBeenLastCalledWith("R01", "STOP", expect.any(String));
    expect(wsSetRobotMode).toHaveBeenCalledWith("R01", "AUTONOMOUS");

    act(() => useStore.getState().setRobotDetail("R01", { appliedMode: "MANUAL", modeTransitionState: "APPLIED" }));
    act(() => manualPointer(forward, "pointerdown"));
    act(() => window.dispatchEvent(new Event("pagehide")));
    expect(wsManualCommand).toHaveBeenLastCalledWith("R01", "STOP", expect.any(String));

    act(() => useStore.getState().setRobotDetail("R01", { appliedMode: "MANUAL", modeTransitionState: "APPLIED" }));
    act(() => manualPointer(forward, "pointerdown"));
    act(() => buttonNamed("EMERGENCY STOP")?.click());
    expect(wsManualCommand).toHaveBeenLastCalledWith("R01", "STOP", expect.any(String));
    expect(api.emergencyStop).toHaveBeenCalledWith("R01");

    act(() => useStore.getState().setRobotDetail("R01", { appliedMode: "MANUAL", modeTransitionState: "APPLIED" }));
    act(() => manualPointer(forward, "pointerdown"));
    act(() => useStore.setState({ websocketState: "DISCONNECTED" }));
    expect(wsManualCommand).toHaveBeenLastCalledWith("R01", "STOP", expect.any(String));
  });

  it("stops a held manual lease when SPA navigation leaves CONTROL without sending a motion command", async () => {
    setOnlineRobot("MAPPING");
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { await settleUi(); });
    const forward = container.querySelector<HTMLButtonElement>(".manual-key-forward")!;
    act(() => manualPointer(forward, "pointerdown"));
    expect(wsManualCommand).toHaveBeenLastCalledWith("R01", "FORWARD", expect.any(String));
    vi.mocked(wsManualCommand).mockClear();

    const mappingLink = container.querySelector<HTMLAnchorElement>('.wt-sidebar-item[href="/mapping"]');
    act(() => mappingLink?.click());

    expect(window.location.pathname).toBe("/mapping");
    expect(wsManualCommand.mock.calls.map(([, action]) => action)).toEqual(["STOP"]);
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).not.toContain("NAV_GOAL");
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).not.toContain("NAV_CANCEL");
  });

  it("invalidates a pending navigation preview on route change without cancelling an active goal", async () => {
    setOnlineRobot();
    setNavReadyCapabilities();
    useStore.setState({ twin: { ...initialState.twin, robots: { R01: { ...r01(), control_mode: "AUTONOMOUS" } } } });
    const globalPath = { robot_id: "R01", frame_id: "map", active_map_id: "NAV2-R01-map",
      active_map_revision: "nav2-r21", points: [[0, 0], [1, 1]] as Array<[number, number]> };
    const localPath = { ...globalPath, points: [[0, 0], [0.5, 0.5]] as Array<[number, number]> };
    const activeGoal = { robot_id: "R01", frame_id: "map", x: 1, y: 1, yaw: 0,
      active_map_id: "NAV2-R01-map", active_map_revision: "nav2-r21", status: "ACTIVE" };
    useStore.getState().setRobotDetail("R01", { globalPath, localPath, goal: activeGoal, navigationStatus: "ACTIVE" });
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { await settleUi(); await settleUi(); });
    const canvas = container.querySelector<HTMLCanvasElement>('[data-testid="active-navigation-map-2d"] canvas')!;
    await act(async () => { canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    await act(async () => { canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 380, clientY: 220 })); });
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).toContain("PATH_PREVIEW_REQUEST");
    vi.mocked(wsSend).mockClear();

    const mappingLink = container.querySelector<HTMLAnchorElement>('.wt-sidebar-item[href="/mapping"]');
    await act(async () => { mappingLink?.click(); await settleUi(); await settleUi(); });

    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).toEqual(["PATH_PREVIEW_INVALIDATE"]);
    expect(useStore.getState().robotDetail.R01).toMatchObject({
      pathPreview: null, globalPath, localPath, goal: activeGoal, navigationStatus: "ACTIVE",
    });
  });

  it("preserves active navigation state when CONTROL unmounts and when control mode changes", () => {
    setOnlineRobot();
    const globalPath = { robot_id: "R01", frame_id: "map", active_map_id: "NAV2-R01-map",
      active_map_revision: "nav2-r21", points: [[0, 0], [1, 1]] as Array<[number, number]> };
    const localPath = { ...globalPath, points: [[0, 0], [0.5, 0.5]] as Array<[number, number]> };
    const activeGoal = { robot_id: "R01", frame_id: "map", x: 1, y: 1, yaw: 0,
      active_map_id: "NAV2-R01-map", active_map_revision: "nav2-r21", status: "ACTIVE" };
    useStore.getState().setRobotDetail("R01", { globalPath, localPath, goal: activeGoal, navigationStatus: "ACTIVE" });
    renderNode(<ControlDetailHarness robotId="R01" />);

    act(() => buttonNamed("AUTONOMOUS")?.click());
    expect(useStore.getState().robotDetail.R01).toMatchObject({ globalPath, localPath, goal: activeGoal });
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).not.toContain("NAV_CANCEL");

    act(() => container.querySelector<HTMLAnchorElement>('.wt-sidebar-item[href="/mapping"]')?.click());
    expect(useStore.getState().robotDetail.R01).toMatchObject({ globalPath, localPath, goal: activeGoal });
  });

  it("invalidates an unaccepted preview on mode change and rejects a late result", async () => {
    setOnlineRobot();
    setNavReadyCapabilities();
    useStore.setState({ twin: { ...initialState.twin, robots: { R01: { ...r01(), control_mode: "AUTONOMOUS" } } } });
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { await settleUi(); await settleUi(); });
    const canvas = container.querySelector<HTMLCanvasElement>('[data-testid="active-navigation-map-2d"] canvas')!;
    await act(async () => { canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    await act(async () => { canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 380, clientY: 220 })); });
    const request = vi.mocked(wsSend).mock.calls.map(([message]) => message)
      .find((message) => message.type === "PATH_PREVIEW_REQUEST");
    if (!request || request.type !== "PATH_PREVIEW_REQUEST") throw new Error("path preview request was not emitted");
    const stalePreview = {
      robot_id: "R01", request_id: request.request_id, status: "VALID" as const,
      source_type: "ACTIVE_MAP_POINT" as const, source_map_id: "NAV2-R01-map", source_map_revision: "nav2-r21",
      frame_id: "map" as const, path: [[0, 0], [1, 1]] as Array<[number, number]>, path_length_m: 1.4,
      goal: { x: request.x!, y: request.y!, yaw: request.yaw! }, timestamp: new Date().toISOString(),
      active_map_id: "NAV2-R01-map", active_map_revision: "nav2-r21",
    };
    act(() => useStore.getState().setRobotDetail("R01", { pathPreview: stalePreview }));
    expect(container.textContent).toContain("PREVIEW VALID");
    vi.mocked(wsSend).mockClear();

    act(() => buttonNamed("MANUAL")?.click());
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).toContain("PATH_PREVIEW_INVALIDATE");
    act(() => useStore.getState().setRobotDetail("R01", { pathPreview: stalePreview }));
    act(() => buttonNamed("SEND GOAL")?.click());
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).not.toContain("NAV_GOAL");
  });

  it("clears hold state on authoritative E-STOP and does not resume when the latch clears", () => {
    setOnlineRobot("MAPPING");
    renderNode(<ControlDetailHarness robotId="R01" />);
    const forward = container.querySelector<HTMLButtonElement>(".manual-key-forward")!;
    act(() => manualPointer(forward, "pointerdown"));
    expect(forward.getAttribute("aria-pressed")).toBe("true");
    act(() => useStore.getState().setRobotDetail("R01", {
      diagnostics: { command_ownership: { estop_active: true } } as never,
    }));
    expect(wsManualCommand).toHaveBeenLastCalledWith("R01", "STOP", expect.any(String));
    expect(container.querySelector<HTMLButtonElement>(".manual-key-forward")?.getAttribute("aria-pressed")).toBe("false");
    const countAfterStop = vi.mocked(wsManualCommand).mock.calls.length;
    act(() => useStore.getState().setRobotDetail("R01", {
      diagnostics: { command_ownership: { estop_active: false } } as never,
    }));
    expect(vi.mocked(wsManualCommand).mock.calls).toHaveLength(countAfterStop);
    expect(container.querySelector<HTMLButtonElement>(".manual-key-forward")?.getAttribute("aria-pressed")).toBe("false");
  });
  it("renders the direct URL with null map/scan and keeps MANUAL JOG hidden in autonomous mode", () => {
    useStore.setState({ twin: null as never, rosDiagnostics: null, rosConnected: false, websocketState: "DISCONNECTED", robotDetail: {} });
    renderNode(<ControlDetailHarness robotId="R01" />);
    expect(container.querySelector('[data-testid="slam-map-2d-empty"]')).toBeTruthy();
    expect(container.querySelector('[data-testid="robot-map-layers"]')).toBeNull();
    expect(container.querySelector('[data-testid="manual-jog-panel"]')).toBeNull();
    expect(container.querySelector('[aria-label="Robot operation panel"]')).toBeTruthy();
    act(() => buttonNamed("DIAGNOSIS")?.click());
    expect(container.textContent).toContain("SUBSYSTEM STATUS");
    expect(container.textContent).toContain("ROS Bridge");
    expect(container.textContent).toContain("LIDAR");
    expect(container.textContent).toContain("LIVE ERROR LOG");
    expect(container.textContent).toContain("No runtime errors reported");
  });

  it("does not present a robot pose as current without a matching map snapshot", () => {
    renderNode(<ControlDetailHarness robotId="R01" />);
    act(() => buttonNamed("LOCALIZATION")?.click());
    expect(container.textContent).toContain("LOCALIZATION STATUS");
    expect(container.querySelector('[data-testid="localization-current-pose-unavailable"]')?.textContent)
      .toContain("FRESH MAP-MATCHED ROBOT POSE UNAVAILABLE");
    expect(container.textContent).not.toContain("15.000 m");
  });

  it("labels the Robot Control pose with the active SLAM map", () => {
    setOnlineRobot("MAPPING");
    const slamPose = { ...r01().slam_pose!, x: 4.5, y: 6.25, timestamp: new Date().toISOString() };
    useStore.setState({ twin: { ...initialState.twin, robots: { R01: {
      ...r01(), position: [90, 0, 90], active_map_pose: slamPose, slam_pose: slamPose,
    } } } });
    useStore.getState().setRobotDetail("R01", { slam2dMap: {
      robot_id: "R01", frame_id: "map", map_source: "SLAM_TOOLBOX", mapping_session_id: "session-1",
      active_map_id: "SLAM-session-1", active_map_revision: "session-session-1",
      width: 10, height: 10, resolution: 1, origin: { x: 0, y: 0, yaw: 0 }, data: Array(100).fill(0),
    } });
    renderNode(<ControlDetailHarness robotId="R01" />);
    act(() => buttonNamed("LOCALIZATION")?.click());
    expect(container.textContent).toContain("LOCALIZATION STATUS");
    expect(container.textContent).toContain("4.500 m");
    expect(container.querySelector('[data-testid="global-warehouse-map"]')).toBeNull();
  });

  it("enables control only for the robot with a live ROS bridge", () => {
    useStore.setState({
      runtimeMode: "GAZEBO_ROS",
      rosConnected: true,
      websocketState: "CONNECTED",
      connectedRobotIds: ["R01"],
    });
    renderNode(<ControlDetailHarness robotId="R01" />);
    expect(container.querySelector(".manual-key-forward")?.hasAttribute("disabled")).toBe(false);
    act(() => root.unmount());

    const r02 = { ...r01(), id: "R02", status: "ACTIVE" as const };
    useStore.setState({
      twin: { ...initialState.twin, robots: { R01: r01(), R02: r02 } },
      connectedRobotIds: ["R01"],
    });
    root = createRoot(container);
    renderNode(<ControlDetailHarness robotId="R02" />);
    expect(container.textContent).not.toContain("R02");
    expect(container.querySelector(".manual-key-forward")?.hasAttribute("disabled")).toBe(false);
  });

  it("renders Mapping as a map-first workspace with compact saved-map management", async () => {
    renderNode(<ControlDetailHarness robotId="R01" />);
    expect(container.querySelector('nav[aria-label="Web Local pages"]')).toBeTruthy();
    const mappingTab = container.querySelector<HTMLAnchorElement>('.wt-sidebar-item[href="/mapping"]');
    await act(async () => { (mappingTab as HTMLElement).click(); });
    expect(container.textContent).toContain("LIVE SLAM MAP");
    expect(container.textContent).toContain("SAVE MAP");
    expect(container.querySelectorAll(".wt-sidebar-item")).toHaveLength(7);
    expect(container.querySelector('.wt-sidebar-item[aria-label="MAPS"]')).toBeNull();
    for (const label of ["Mapping Session", "Save Map", "Saved Maps", "Map Details / Load", "ADVANCED DIAGNOSTICS"]) {
      expect(container.textContent).toContain(label);
    }
    expect(container.querySelectorAll(".mapping-workspace > *")).toHaveLength(2);
    expect(container.querySelector(".mapping-map-viewport")).toBeTruthy();
    expect(container.querySelector(".mapping-operation-column")).toBeTruthy();
    expect(container.querySelectorAll(".mapping-layer-toolbar button")).toHaveLength(4);
    expect(container.querySelector("details.mapping-advanced-details")?.hasAttribute("open")).toBe(false);
    expect(container.querySelector(".mapping-preview-drawer")).toBeNull();
    expect(container.querySelectorAll(".hmi-map-load-button")).toHaveLength(1);
    await act(async () => { buttonNamed("DIAGNOSIS")?.click(); });
    expect(container.textContent).toContain("SUBSYSTEM STATUS");
    expect(container.querySelector<HTMLAnchorElement>('.wt-sidebar-item[href="/diagnosis"]')?.getAttribute("aria-current")).toBe("page");
    expect(container.querySelector<HTMLAnchorElement>('.wt-sidebar-item[href="/vda5050"]')?.getAttribute("aria-current")).toBeNull();
    expect(container.textContent).not.toContain("MQTT host");
    await act(async () => { buttonNamed("VDA5050")?.click(); await settleUi(); });
    expect(container.querySelector<HTMLAnchorElement>('.wt-sidebar-item[href="/vda5050"]')?.getAttribute("aria-current")).toBe("page");
    expect(container.textContent).toContain("Broker Connection");
    expect(container.textContent).toContain("VDA5050 Protocol Identity");
    expect(container.textContent).toContain("Runtime & Task Policy");
    expect(container.textContent).toContain("MQTT host");
    expect(container.textContent).toContain("Allow Task");
    expect(container.querySelector('input[type="password"]')?.getAttribute("type")).toBe("password");
    expect(container.querySelector(".local-app-header-title")?.textContent).toBe("VDA5050");
    for (const field of ["Connection enabled", "MQTT host", "MQTT port", "Username", "Password", "TLS", "MQTT version",
      "VDA5050 version", "Topic prefix", "Interface name", "Manufacturer", "Serial number", "Client ID", "Allow Task",
      "Auto reconnect", "Reconnect interval (s)", "Connection timeout (s)", "Keepalive (s)", "MQTT connection"]) {
      expect(container.textContent).toContain(field);
    }
    expect(buttonNamed("TEST CONNECTION")).toBeTruthy();
    expect(buttonNamed("SAVE & APPLY")).toBeTruthy();
  });

  it("shows an active saved map preview only when the selected registry entry matches the confirmed ROS snapshot", async () => {
    setOnlineRobot("NAVIGATION");
    const savedMap: LocalRobotMap = {
      id: "saved-floor-map", name: "floor map", robot_id: "R01", created_at: "2026-10-02T09:00:00Z",
      resolution: 0.05, origin: [-1, -2, 0], revision: "rev-5", frame_id: "map", width: 80, height: 100,
      known_cells: 3200, free_cells: 2700, occupied_cells: 500, explored_area_m2: 8,
      map_kind: "SAVED_LOCAL_MAP", canonical_map_promoted: false,
      slam_session_state: { status: "AVAILABLE" },
    };
    const localSnapshot = {
      robot_id: "R01", frame_id: "map", map_source: "LOCAL_MAP" as const,
      active_map_id: savedMap.id, active_map_revision: savedMap.revision,
      width: 80, height: 100, resolution: savedMap.resolution,
      origin: { x: -1, y: -2, yaw: 0 }, data: Array(8000).fill(0),
    };
    vi.mocked(api.getLocalRobotMaps).mockResolvedValue({
      robot_id: "R01", maps: [savedMap], runtime_mode: "GAZEBO_ROS", mapping_state: "PAUSED",
      mapping_duration_s: 20, active_local_map_id: savedMap.id, local_active_map_id: savedMap.id,
      local_active_map_revision: savedMap.revision, active_map_id: savedMap.id, active_map_revision: savedMap.revision,
      canonical_map_revision: 21, map_sync_status: "LOCAL_ONLY", robot_control_mode: "MANUAL", robot_stopped: true,
    });
    useStore.getState().setRobotDetail("R01", {
      activeLocalMapId: savedMap.id, activeLocalMapRevision: savedMap.revision,
      runtimeMapSnapshot: localSnapshot,
    });
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { buttonNamed("MAPPING")?.click(); await settleUi(); await settleUi(); });
    expect(container.querySelector('[data-testid="active-navigation-map-2d"]')).toBeNull();
    await act(async () => { buttonNamed("PREVIEW MAP")?.click(); await settleUi(); });
    expect(container.querySelector(".hmi-mapping-preview .local-pose-map")).toBeTruthy();
    expect(container.querySelector(".hmi-map-detail-state")?.textContent).toContain("ACTIVE MAP");
    expect(container.querySelector(".hmi-map-load-button")?.textContent).toContain("RELOAD ACTIVE MAP");
    expect(container.querySelector(".hmi-map-details-grid")?.textContent).toContain("LOCAL_ONLY");
  });

  it("shows only the active 2D occupancy map and point controls", async () => {
    setOnlineRobot("MAPPING");
    setNavReadyCapabilities();
    const slamMap = { robot_id: "R01", frame_id: "map", map_source: "SLAM_TOOLBOX" as const,
      mapping_session_id: "session-1", active_map_id: "SLAM-session-1",
      active_map_revision: "session-session-1", map_content_revision: "cells-a",
      width: 2, height: 2, resolution: .05, origin: { x: 0, y: 0, yaw: 0 }, data: [-1, 0, 100, -1] };
    useStore.getState().setRobotDetail("R01", { mappingSessionId: "session-1", slam2dMap: slamMap, scan: {
      robot_id: "R01", topic: "/scan", source_frame_id: "lidar_link", mapping_session_id: "session-1",
      frame_id: "map", angle_min: -Math.PI, angle_max: Math.PI, angle_increment: 0.1,
      range_min: 0.1, range_max: 10, point_count: 2, points: [[1, 0], [2, 0]],
      trajectory: [[0, 0], [0.5, 0.5]], timestamp: new Date().toISOString(),
    } });
    useStore.setState({ twin: { ...useStore.getState().twin!, robots: {
      ...useStore.getState().twin!.robots, R01: { ...r01(), control_mode: "AUTONOMOUS" },
    } } });
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { await settleUi(); });

    expect(container.textContent).not.toContain("SLAM /map");
    expect(container.querySelector(".hmi-map-info")).toBeNull();
    expect(container.querySelector('[data-testid="slam-map-2d-metrics"]')?.getAttribute("data-map-source")).toBe("SLAM_TOOLBOX");
    expect(container.textContent).not.toContain("2D SLAM OCCUPANCY MAP");
    expect(container.textContent).not.toContain("GLOBAL MAP");
    expect(container.textContent).not.toContain("MAP VIEW 3D");
    expect(container.textContent).not.toContain("Destination Tag");
    expect(container.textContent).not.toContain("Target Tag");
    expect(container.querySelector('[aria-label="Navigation target method"]')).toBeNull();
    expect(container.querySelectorAll('[data-testid="robot-map-layers"] button')).toHaveLength(3);
    expect(buttonNamed("PAN")?.getAttribute("aria-pressed")).toBe("false");
    expect(buttonNamed("POINT")?.getAttribute("aria-pressed")).toBe("true");
    expect(container.textContent).not.toContain("MQTT host");
    expect(container.querySelector('[data-testid="active-navigation-map-2d"]')).toBeTruthy();
    expect(container.querySelector<HTMLCanvasElement>('[data-testid="active-navigation-map-2d"] canvas')?.dataset.mapId)
      .toBe("SLAM-session-1");
    const mapCanvas = container.querySelector<HTMLCanvasElement>('[data-testid="active-navigation-map-2d"] canvas')!;
    const toolbarButtons = Array.from(container.querySelectorAll<HTMLButtonElement>('[data-testid="robot-map-layers"] button'));
    expect(toolbarButtons.map((button) => button.textContent?.trim())).toEqual(["PAN", "POINT", "LAYERS"]);
    expect(container.querySelector('[data-testid="map-layer-options"]')).toBeNull();
    expect(mapCanvas.dataset.scanLayer).toBe("low-opacity-current-scan-overlay");
    expect(mapCanvas.dataset.trajectoryLayer).toBe("disabled");
    await act(async () => { toolbarButtons[2].click(); });
    expect(toolbarButtons[2].getAttribute("aria-expanded")).toBe("true");
    expect(container.querySelectorAll('[data-testid="map-layer-options"] input[type="checkbox"]')).toHaveLength(5);
    const layerInputs = ["robot", "scan", "path", "trajectory", "grid"].map((layer) =>
      container.querySelector<HTMLInputElement>(`[data-testid="map-layer-${layer}"]`)!);
    expect(layerInputs.map((input) => input.checked)).toEqual([true, true, true, false, false]);
    await act(async () => { layerInputs[0].click(); layerInputs[1].click(); layerInputs[3].click(); layerInputs[4].click(); });
    expect(mapCanvas.dataset.robotLayer).toBe("disabled");
    expect(mapCanvas.dataset.scanLayer).toBe("disabled");
    expect(mapCanvas.dataset.gridLayer).toBe("visible");
    expect(mapCanvas.dataset.trajectoryLayer).toBe("visible");
    expect(layerInputs.map((input) => input.checked)).toEqual([false, false, true, true, true]);
    expect(mapCanvas.dataset.pointSelectionState).toBe("WAITING_FOR_DESTINATION");
    await act(async () => { document.dispatchEvent(new KeyboardEvent("keydown", { key: "Escape", bubbles: true })); });
    expect(container.querySelector('[data-testid="map-layer-options"]')).toBeNull();
    expect(document.activeElement).toBe(toolbarButtons[2]);
    await act(async () => { toolbarButtons[2].click(); });
    await act(async () => { document.body.dispatchEvent(new MouseEvent("pointerdown", { bubbles: true })); });
    expect(container.querySelector('[data-testid="map-layer-options"]')).toBeNull();
  });



  it("keeps accumulated occupancy as the LIDAR 2D base and updates only the restrained scan overlay", async () => {
    setOnlineRobot("MAPPING");
    const slamMap = { robot_id: "R01", frame_id: "map", map_source: "SLAM_TOOLBOX" as const,
      mapping_session_id: "session-1", active_map_id: "SLAM-session-1",
      active_map_revision: "session-session-1", map_content_revision: "cells-before",
      width: 2, height: 2, resolution: .05, origin: { x: 0, y: 0, yaw: 0 },
      data: [-1, 0, 100, -1], known_cells: 2, unknown_cells: 2, occupied_cells: 1, free_cells: 1 };
    const scan = { robot_id: "R01", topic: "/scan", source_frame_id: "lidar_link",
      mapping_session_id: "session-1", frame_id: "map", angle_min: 0, angle_max: 1,
      angle_increment: .1, range_min: .1, range_max: 10, point_count: 1,
      points: [[1, 1] as [number, number]], timestamp: new Date().toISOString() };
    useStore.getState().setRobotDetail("R01", { mappingSessionId: "session-1",
      slam2dMap: slamMap, scan });
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { await settleUi(); });

    const canvas = container.querySelector<HTMLCanvasElement>('[data-testid="slam-map-2d-canvas"]');
    const baseMap = useStore.getState().robotDetail.R01.slam2dMap;
    expect(canvas?.dataset.occupancyLayer).toBe("accumulated-map-snapshot");
    expect(canvas?.dataset.scanLayer).toBe("low-opacity-current-scan-overlay");
    expect(canvas?.dataset.scanRenderMode).toBe("points-only");
    expect(baseMap?.map_content_revision).toBe("cells-before");

    await act(async () => { useStore.getState().setRobotDetail("R01", { scan: {
      ...scan, point_count: 3, points: [[1, 1], [1.1, 1], [1.2, 1]] as [number, number][],
      timestamp: new Date(Date.now() + 1).toISOString(),
    } }); await settleUi(); });
    expect(canvas?.dataset.occupancyLayer).toBe("accumulated-map-snapshot");
    expect(canvas?.dataset.scanLayer).toBe("low-opacity-current-scan-overlay");
    expect(useStore.getState().robotDetail.R01.slam2dMap).toBe(baseMap);
    expect(useStore.getState().robotDetail.R01.slam2dMap?.map_content_revision).toBe("cells-before");
  });

  it("keeps the same-session map camera fixed while accumulated occupancy bounds and cells change", async () => {
    const drawnRasterRevisions: string[] = [];
    const drawingContext = {
      setTransform: vi.fn(), fillRect: vi.fn(), save: vi.fn(), translate: vi.fn(), rotate: vi.fn(),
      scale: vi.fn(), restore: vi.fn(), fillText: vi.fn(), beginPath: vi.fn(), moveTo: vi.fn(),
      lineTo: vi.fn(), closePath: vi.fn(), fill: vi.fn(), stroke: vi.fn(),
      drawImage: vi.fn((image: CanvasImageSource) => {
        if (image instanceof HTMLCanvasElement) drawnRasterRevisions.push(image.dataset.mapContentRevision ?? "");
      }),
    } as unknown as CanvasRenderingContext2D;
    vi.spyOn(HTMLCanvasElement.prototype, "getContext").mockReturnValue(drawingContext);
    vi.mocked(occupancyRasters.get).mockImplementation(async (map) => {
      const raster = document.createElement("canvas");
      raster.dataset.mapContentRevision = map?.map_content_revision ?? "";
      return raster;
    });
    const common = { robot_id: "R01", frame_id: "map", map_source: "SLAM_TOOLBOX" as const,
      mapping_session_id: "session-camera", active_map_id: "SLAM-session-camera",
      active_map_revision: "session-session-camera", resolution: .05 };
    const frameA = { ...common, map_content_revision: "cells-a", width: 400, height: 500,
      origin: { x: -5, y: -10, yaw: 0 }, data: [-1, 0, 100, -1] };
    const frameB = { ...common, map_content_revision: "cells-b", width: 600, height: 700,
      origin: { x: -8, y: -15, yaw: 0 }, data: [0, 0, 100, 100] };
    const frameC = { ...common, map_content_revision: "cells-c", width: 800, height: 900,
      origin: { x: -12, y: -20, yaw: 0 }, data: [100, 0, -1, 100] };

    const robot = { ...r01(), slam_pose: { ...r01().slam_pose!, mapping_session_id: "session-camera",
      map_id: "SLAM-session-camera", map_revision: "session-session-camera" } };
    renderNode(<AccumulatedSlamMap2DView map={frameA} robot={robot} scan={null} />);
    await act(async () => { await settleUi(); });
    const canvas = container.querySelector<HTMLCanvasElement>('[data-testid="slam-map-2d-canvas"]')!;
    const worldPoint = { x: 5, y: 5 };
    const pixelA = canvasWorldPixel(canvas, worldPoint);

    renderNode(<AccumulatedSlamMap2DView map={frameB} robot={robot} scan={null} />);
    await act(async () => { await settleUi(); });
    const pixelB = canvasWorldPixel(canvas, worldPoint);
    expect(container.querySelector(".robot-detail-view-readout")?.textContent).toContain("600 × 700");

    renderNode(<AccumulatedSlamMap2DView map={frameC} robot={robot} scan={null} />);
    await act(async () => { await settleUi(); });
    const pixelC = canvasWorldPixel(canvas, worldPoint);
    expect(container.querySelector(".robot-detail-view-readout")?.textContent).toContain("800 × 900");
    expect(Math.max(Math.hypot(pixelB.x - pixelA.x, pixelB.y - pixelA.y),
      Math.hypot(pixelC.x - pixelA.x, pixelC.y - pixelA.y))).toBeLessThanOrEqual(1);
    expect(occupancyRasters.get).toHaveBeenCalledWith(frameB);
    expect(occupancyRasters.get).toHaveBeenCalledWith(frameC);
    expect(drawnRasterRevisions).toContain("cells-b");
    expect(drawnRasterRevisions).toContain("cells-c");

    const scaleBeforeZoom = canvas.dataset.viewportScalePxPerMeter;
    await act(async () => { buttonNamed("Zoom in")?.click(); });
    const zoomedScale = canvas.dataset.viewportScalePxPerMeter;
    expect(Number(zoomedScale)).toBeGreaterThan(Number(scaleBeforeZoom));
    renderNode(<AccumulatedSlamMap2DView map={frameB} robot={robot} scan={null} />);
    await act(async () => { await settleUi(); });
    expect(canvas.dataset.viewportScalePxPerMeter).toBe(zoomedScale);

    const centeredScale = canvas.dataset.viewportScalePxPerMeter;
    await act(async () => { buttonNamed("CENTER ROBOT")?.click(); });
    expect(Number(canvas.dataset.viewportCenterX)).toBeCloseTo(robot.slam_pose!.x, 8);
    expect(Number(canvas.dataset.viewportCenterY)).toBeCloseTo(robot.slam_pose!.y, 8);
    expect(canvas.dataset.viewportScalePxPerMeter).toBe(centeredScale);
    renderNode(<AccumulatedSlamMap2DView map={frameC} robot={robot} scan={null} />);
    await act(async () => { await settleUi(); });
    expect(Number(canvas.dataset.viewportCenterX)).toBeCloseTo(robot.slam_pose!.x, 8);

    const beforeFit = canvas.dataset.viewportScalePxPerMeter;
    await act(async () => { buttonNamed("FIT")?.click(); });
    expect(canvas.dataset.viewportScalePxPerMeter).not.toBe(beforeFit);
  });

  it("shows live SLAM and ready Nav2 together in Unified without disabling goals because SLAM is active", async () => {
    setOnlineRobot("UNIFIED");
    const slamMap = { robot_id: "R01", frame_id: "map", map_source: "SLAM_TOOLBOX" as const,
      mapping_session_id: "session-1", active_map_id: "SLAM-session-1",
      active_map_revision: "session-session-1", map_content_revision: "cells-a",
      width: 2, height: 2, resolution: .05, origin: { x: 0, y: 0, yaw: 0 }, data: [-1, 0, 100, -1] };
    const cloud = { robot_id: "R01", frame_id: "map", source_frame_id: "lidar_link", point_count: 3,
      points: [[1, 0, 0], [2, 1, 0.1], [3, 2, 0.2]] as [number, number, number][], bounds: null,
      epoch: "bridge", revision: 1, accumulated: true as const, accumulation_mode: "SLAM_VISUALIZATION_VOXEL_MAP" as const,
      slam_pose: { ...r01().slam_pose!, timestamp: new Date().toISOString() }, path: [], goal: null };
    useStore.setState({ robotCapabilities: { R01: {
      mapping_available: true, mapping_active: true, nav2_available: true, nav2_ready: true,
      manual_available: true, goal_available: true, map_ready: true, tag_navigation_available: false,
    } }, rosDiagnostics: {
      ros: true, gazebo: true, controller_manager: true, slam: true, nav2: true, nav2_ready: true,
      tf: true, lidar: true, nodes: ["slam_toolbox", "controller_server"], topics: ["/map", "/scan"],
      controllers: [], simulation_time: 10, last_update_at: new Date().toISOString(),
      mapping: { slam_state: "ACTIVE", mapping_session_id: "session-1", map_live: true, scan_live: true },
    } });
    useStore.setState({ twin: { ...useStore.getState().twin!, robots: {
      ...useStore.getState().twin!.robots, R01: { ...r01(), control_mode: "AUTONOMOUS" },
    } } });
    useStore.getState().setRobotDetail("R01", { mappingSessionId: "session-1", mappingState: "MAPPING",
      slam2dMap: slamMap, slam3dAccumulatedCloud: cloud });
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { await settleUi(); });

    expect(dashboardStatus("SYSTEM INPUTS", "SLAM")).toBe("LIVE");
    expect(dashboardStatus("SYSTEM INPUTS", "NAV2")).toBe("READY");
    expect(container.textContent).toContain("ACTIVE MAP");
    expect(container.textContent).not.toContain("session-session-1");
    expect(container.textContent).not.toContain("GOALS DISABLED");

    await act(async () => { await settleUi(); });
    const liveMapCanvas = container.querySelector<HTMLCanvasElement>('[data-testid="active-navigation-map-2d"] canvas');
    expect(liveMapCanvas?.dataset.mapId).toBe("SLAM-session-1");
    await act(async () => { liveMapCanvas?.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 400, clientY: 340 })); });
    expect(container.querySelector('[data-testid="selected-point-status"]')?.textContent).toBe("SELECT YAW");
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).not.toContain("PATH_PREVIEW_REQUEST");
    await act(async () => { liveMapCanvas?.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 400, clientY: 340 })); });
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).toContain("PATH_PREVIEW_REQUEST");
    expect(container.querySelector('[data-testid="slam-map-3d"]')).toBeNull();

    await act(async () => { buttonNamed("MAPPING")?.click(); await settleUi(); });
    expect(container.textContent).toContain("Mapping Session");
    expect(container.textContent).not.toContain("SWITCH TO NAVIGATION");
    const mappingAdvanced = container.querySelector<HTMLDetailsElement>(".hmi-advanced-details")!;
    expect(mappingAdvanced.open).toBe(false);
    await act(async () => { mappingAdvanced.open = true; mappingAdvanced.dispatchEvent(new Event("toggle", { bubbles: true })); });
    expect(container.textContent).toContain("supervised SLAM-to-Navigation handoff");
    expect(container.textContent).not.toContain("in-place saved-map localization");

    expect(container.querySelector<HTMLCanvasElement>('[data-testid="slam-map-2d-canvas"]')?.dataset.mapSource).toBe("SLAM_TOOLBOX");
  });

  it("fails closed when lifecycle diagnostics say bt_navigator is inactive", () => {
    setOnlineRobot("UNIFIED");
    setNavReadyCapabilities();
    useStore.setState({ twin: { ...useStore.getState().twin!, robots: { R01: { ...r01(), control_mode: "AUTONOMOUS" } } } });
    useStore.setState({ rosDiagnostics: {
      ros: true, gazebo: true, controller_manager: true, slam: true, nav2: true,
      nav2_ready: false, nav2_actions_ready: true, nav2_lifecycle_ready: false,
      nav2_lifecycle_states: {
        canonical_map_server: "active", controller_server: "active", planner_server: "active",
        behavior_server: "active", bt_navigator: "inactive", waypoint_follower: "active",
      }, nav2_lifecycle_blocker_code: "NAV2_LIFECYCLE_NOT_ACTIVE",
      nav2_lifecycle_blocker_reason: "Required Nav2 lifecycle node(s) must be ACTIVE: /bt_navigator=inactive.",
      tf: true, lidar: true, nodes: ["bt_navigator"], topics: [], controllers: [],
      simulation_time: 10, last_update_at: new Date().toISOString(),
    } });
    renderNode(<ControlDetailHarness robotId="R01" />);

    expect(dashboardStatus("SYSTEM INPUTS", "NAV2")).toBe("BLOCKED");
    expect(container.textContent).not.toContain("NAV2 READY");
    expect(container.textContent).toContain("NAV2_LIFECYCLE_NOT_ACTIVE");
    expect(container.textContent).toContain("/bt_navigator=inactive");
    expect(buttonNamed("SEND GOAL")?.disabled).toBe(true);
  });


  it("allows the first POINT click while Nav2 is blocked and waits with the confirmed target until Nav2 returns", async () => {
    setOnlineRobot();
    setNavReadyCapabilities();
    setNavBlockedCapabilities();
    useStore.setState({ twin: { ...initialState.twin, robots: { R01: { ...r01(), control_mode: "AUTONOMOUS" } } } });
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { await settleUi(); await settleUi(); });

    const canvas = container.querySelector<HTMLCanvasElement>('[data-testid="active-navigation-map-2d"] canvas')!;
    expect(canvas).toBeTruthy();
    const targetWorld = canvasScreenWorld(canvas, { x: 320, y: 180 });
    const headingWorld = canvasScreenWorld(canvas, { x: 380, y: 220 });
    const expectedYaw = Math.atan2(headingWorld.y - targetWorld.y, headingWorld.x - targetWorld.x);
    await act(async () => { canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    expect(container.querySelector('[data-testid="selected-point-status"]')?.textContent).toBe("SELECT YAW");
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).not.toContain("PATH_PREVIEW_REQUEST");

    await act(async () => {
      mapPointer(canvas, "pointermove", 380, 220);
      canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 380, clientY: 220 }));
    });
    expect(container.querySelector('[data-testid="selected-point-status"]')?.textContent).toBe("POINT SELECTED");
    expect(container.querySelector('[data-testid="point-navigation-state"]')?.textContent).toBe("WAITING FOR NAV2");
    expect(container.textContent).toContain("0.00 m");
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).not.toContain("PATH_PREVIEW_REQUEST");
    expect(buttonNamed("SEND GOAL")?.disabled).toBe(true);

    act(() => setNavReadyCapabilities());
    await act(async () => { await settleUi(); });
    const previewRequest = vi.mocked(wsSend).mock.calls.map(([message]) => message)
      .find((message) => message.type === "PATH_PREVIEW_REQUEST");
    expect(previewRequest?.type).toBe("PATH_PREVIEW_REQUEST");
    if (previewRequest?.type === "PATH_PREVIEW_REQUEST") expect(previewRequest.yaw).toBeCloseTo(expectedYaw, 2);
    expect(container.querySelector('[data-testid="selected-point-status"]')?.textContent).toBe("POINT SELECTED");
    expect(container.querySelector('[data-testid="point-navigation-state"]')?.textContent).toBe("PLANNING");
  });

  it("reports unknown and occupied cells while retaining the free-cell selection guard", async () => {
    setOnlineRobot();
    setNavReadyCapabilities();
    useStore.setState({ twin: { ...initialState.twin, robots: { R01: { ...r01(), control_mode: "AUTONOMOUS" } } } });
    const base = useStore.getState().robotDetail.R01!.runtimeMapSnapshot!;
    const data = Array(100).fill(0);
    data[55] = -1;
    useStore.getState().setRobotDetail("R01", { runtimeMapSnapshot: { ...base, data } });
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { await settleUi(); await settleUi(); });
    let canvas = container.querySelector<HTMLCanvasElement>('[data-testid="active-navigation-map-2d"] canvas')!;
    await act(async () => { canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    expect(container.querySelector('[data-testid="map-pick-feedback"]')?.textContent).toContain("UNKNOWN / UNMAPPED CELL");
    expect(container.querySelector('[data-testid="selected-point-status"]')).toBeNull();

    data[55] = 100;
    act(() => useStore.getState().setRobotDetail("R01", { runtimeMapSnapshot: { ...base, data: [...data] } }));
    await act(async () => { await settleUi(); await settleUi(); });
    canvas = container.querySelector<HTMLCanvasElement>('[data-testid="active-navigation-map-2d"] canvas')!;
    await act(async () => { canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    expect(container.querySelector('[data-testid="map-pick-feedback"]')?.textContent).toContain("OCCUPIED CELL");
    expect(container.querySelector('[data-testid="selected-point-status"]')).toBeNull();

    data[55] = 0;
    act(() => useStore.getState().setRobotDetail("R01", { runtimeMapSnapshot: { ...base, data: [...data] } }));
    await act(async () => { await settleUi(); await settleUi(); });
    canvas = container.querySelector<HTMLCanvasElement>('[data-testid="active-navigation-map-2d"] canvas')!;
    await act(async () => { canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    expect(container.querySelector('[data-testid="selected-point-status"]')?.textContent).toBe("SELECT YAW");
  });

  it("keeps the last complete SLAM snapshot interactive while the next raster is still building", async () => {
    setOnlineRobot("UNIFIED");
    setNavReadyCapabilities();
    setNavBlockedCapabilities();
    const common = { robot_id: "R01", frame_id: "map", map_source: "SLAM_TOOLBOX" as const,
      mapping_session_id: "point-session", active_map_id: "SLAM-point-session",
      active_map_revision: "session-point-session", resolution: 1 };
    const firstMap = { ...common, map_content_revision: "point-a", width: 10, height: 10,
      origin: { x: -5, y: -5, yaw: 0 }, data: Array(100).fill(0) };
    const nextCells = Array(144).fill(100);
    const nextMap = { ...common, map_content_revision: "point-b", width: 12, height: 12,
      origin: { x: -6, y: -6, yaw: 0 }, data: nextCells };
    const robot = { ...r01(), control_mode: "AUTONOMOUS" as const,
      slam_pose: { ...r01().slam_pose!, mapping_session_id: "point-session", map_id: "SLAM-point-session",
        map_revision: "session-point-session" } };
    useStore.setState({ twin: { ...initialState.twin, robots: { R01: robot } }, rosDiagnostics: {
      ...useStore.getState().rosDiagnostics!, mapping: { slam_state: "ACTIVE", mapping_session_id: "point-session", map_live: true, scan_live: true },
    } });
    useStore.getState().setRobotDetail("R01", { mappingSessionId: "point-session", slam2dMap: firstMap });
    let resolveFirstRaster!: (value: HTMLCanvasElement | null) => void;
    let resolveNextRaster!: (value: HTMLCanvasElement | null) => void;
    vi.mocked(occupancyRasters.get).mockImplementation((snapshot) => snapshot === firstMap
      ? new Promise((resolve) => { resolveFirstRaster = resolve; })
      : new Promise((resolve) => { resolveNextRaster = resolve; }));
    renderNode(<ControlDetailHarness robotId="R01" />);
    let canvas = container.querySelector<HTMLCanvasElement>('[data-testid="active-navigation-map-2d"] canvas')!;
    await act(async () => { canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    expect(container.querySelector('[data-testid="map-pick-feedback"]')?.textContent).toContain("MAP UPDATING");
    await act(async () => { resolveFirstRaster(document.createElement("canvas")); await settleUi(); await settleUi(); });
    canvas = container.querySelector<HTMLCanvasElement>('[data-testid="active-navigation-map-2d"] canvas')!;
    expect(canvas.dataset.interactiveMapContentRevision).toBe("point-a");
    expect(canvas.dataset.interactiveMapContentKey).toBe(canvas.dataset.rasterMapContentKey);

    act(() => useStore.getState().setRobotDetail("R01", { slam2dMap: nextMap }));
    await act(async () => { await settleUi(); });
    canvas = container.querySelector<HTMLCanvasElement>('[data-testid="active-navigation-map-2d"] canvas')!;
    expect(canvas.dataset.interactiveMapContentRevision).toBe("point-a");
    expect(container.querySelector('[data-testid="map-pick-feedback"]')).toBeNull();
    await act(async () => { canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    expect(container.querySelector('[data-testid="selected-point-status"]')?.textContent).toBe("SELECT YAW");
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).not.toContain("PATH_PREVIEW_REQUEST");

    await act(async () => { resolveNextRaster(document.createElement("canvas")); await settleUi(); });
    canvas = container.querySelector<HTMLCanvasElement>('[data-testid="active-navigation-map-2d"] canvas')!;
    expect(canvas.dataset.interactiveMapContentRevision).toBe("point-b");
    expect(canvas.dataset.interactiveMapContentKey).toBe(occupancyRasterKey(nextMap));
    expect(canvas.dataset.interactiveMapContentKey).toBe(canvas.dataset.rasterMapContentKey);
    await act(async () => { buttonNamed("PAN")?.click(); buttonNamed("POINT")?.click(); });
    await act(async () => { canvas.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    expect(container.querySelector('[data-testid="map-pick-feedback"]')?.textContent).toContain("OCCUPIED CELL");
    expect(container.querySelector('[data-testid="selected-point-status"]')).toBeNull();
  });


  it("selects only on the active 2D map and requires an approved path before sending", async () => {
    setOnlineRobot();
    setNavReadyCapabilities();
    useStore.setState({ twin: { ...initialState.twin, robots: { R01: { ...r01(), control_mode: "AUTONOMOUS" } } } });
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { await settleUi(); });

    const canvas = container.querySelector<HTMLCanvasElement>('[data-testid="active-navigation-map-2d"] canvas');
    expect(canvas).toBeTruthy();
    expect(container.querySelector('[data-testid="global-warehouse-map"]')).toBeNull();
    await act(async () => { canvas?.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    expect(container.querySelector('[data-testid="selected-point-status"]')).toBeTruthy();
    expect(container.querySelector('[data-testid="selected-point-status"]')?.textContent).toBe("SELECT YAW");
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).not.toContain("PATH_PREVIEW_REQUEST");
    expect(buttonNamed("SEND GOAL")?.disabled).toBe(true);
    expect(buttonNamed("PREVIEW PATH")).toBeUndefined();

    const pickStart = { x: 320, y: 180 }, pickEnd = { x: 380, y: 220 };
    const startWorld = canvasScreenWorld(canvas!, pickStart), endWorld = canvasScreenWorld(canvas!, pickEnd);
    const selectedYaw = Math.atan2(endWorld.y - startWorld.y, endWorld.x - startWorld.x);
    await act(async () => { mapPointer(canvas!, "pointermove", pickEnd.x, pickEnd.y); });
    expect(canvas?.dataset.directionPreview).toBe("visible");
    await act(async () => { canvas?.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: pickEnd.x, clientY: pickEnd.y })); });
    const request = vi.mocked(wsSend).mock.calls.map(([message]) => message)
      .filter((message) => message.type === "PATH_PREVIEW_REQUEST").at(-1);
    expect(request?.type).toBe("PATH_PREVIEW_REQUEST");
    if (!request || request.type !== "PATH_PREVIEW_REQUEST") throw new Error("path preview request was not emitted");
    expect(request).toMatchObject({ robot_id: "R01", frame_id: "map", active_map_id: "NAV2-R01-map",
      active_map_revision: "nav2-r21", map_id: "NAV2-R01-map", map_revision: "nav2-r21",
      source_type: "ACTIVE_MAP_POINT", source_map_id: "NAV2-R01-map", source_map_revision: "nav2-r21" });
    expect(request.yaw).toBeCloseTo(selectedYaw, 2);
    expect(buttonNamed("SEND GOAL")?.disabled).toBe(true);

    const approved = {
      robot_id: "R01", request_id: request.request_id, status: "VALID" as const,
      source_type: "ACTIVE_MAP_POINT" as const, source_map_id: "NAV2-R01-map", source_map_revision: "nav2-r21",
      frame_id: "map" as const, path: [[0, 0], [1, 1]] as Array<[number, number]>,
      path_length_m: 1.4, goal: { x: request.x!, y: request.y!, yaw: request.yaw! },
      timestamp: new Date().toISOString(), active_map_id: "NAV2-R01-map", active_map_revision: "nav2-r21",
    };
    act(() => useStore.getState().setRobotDetail("R01", { pathPreview: approved }));
    expect(buttonNamed("SEND GOAL")?.disabled).toBe(false);

    await act(async () => { canvas?.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 330, clientY: 180 })); });
    expect(buttonNamed("SEND GOAL")?.disabled).toBe(true);
    expect(container.querySelector('[data-testid="point-navigation-state"]')?.textContent).toBe("SELECT YAW");
    await act(async () => { canvas?.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 330, clientY: 180 })); });
    const replacementRequest = vi.mocked(wsSend).mock.calls.map(([message]) => message)
      .filter((message) => message.type === "PATH_PREVIEW_REQUEST").at(-1);
    expect(replacementRequest?.type).toBe("PATH_PREVIEW_REQUEST");
    if (!replacementRequest || replacementRequest.type !== "PATH_PREVIEW_REQUEST") throw new Error("replacement preview was not emitted");
    expect(replacementRequest.request_id).not.toBe(request.request_id);

    const approvedReplacement = { ...approved, request_id: replacementRequest.request_id,
      goal: { x: replacementRequest.x!, y: replacementRequest.y!, yaw: replacementRequest.yaw! } };
    act(() => useStore.getState().setRobotDetail("R01", { pathPreview: approvedReplacement }));
    expect(buttonNamed("SEND GOAL")?.disabled).toBe(false);
    await act(async () => { buttonNamed("SEND GOAL")?.click(); });
    const goalMessage = vi.mocked(wsSend).mock.calls.map(([message]) => message)
      .find((message) => message.type === "NAV_GOAL");
    expect(goalMessage).toMatchObject({ type: "NAV_GOAL", robot_id: "R01",
      x: replacementRequest.x, y: replacementRequest.y, yaw: replacementRequest.yaw,
      preview_request_id: replacementRequest.request_id, frame_id: "map",
      active_map_id: "NAV2-R01-map", active_map_revision: "nav2-r21",
      source_type: "ACTIVE_MAP_POINT", source_map_id: "NAV2-R01-map", source_map_revision: "nav2-r21" });

    await act(async () => { canvas?.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 400, clientY: 220 })); });
    expect(container.querySelector('[data-testid="point-navigation-state"]')?.textContent).toBe("SELECT YAW");
    await act(async () => { buttonNamed("CANCEL")?.click(); });
    expect(container.querySelector('[data-testid="point-navigation-state"]')?.textContent).toBe("WAITING FOR DESTINATION");
    expect(container.querySelector('[data-testid="selected-point-status"]')).toBeNull();
    expect(buttonNamed("SEND GOAL")?.disabled).toBe(true);

    act(() => useStore.getState().setRobotDetail("R01", { pathPreview: { ...approvedReplacement, path: [] } }));
    expect(buttonNamed("SEND GOAL")?.disabled).toBe(true);
    act(() => useStore.getState().setRobotDetail("R01", {
      pathPreview: { ...approvedReplacement, timestamp: new Date(Date.now() - 121_000).toISOString() },
    }));
    expect(buttonNamed("SEND GOAL")?.disabled).toBe(true);
  });



  it("picks and previews directly on the active saved local map, then clears selection if its identity changes", async () => {
    setOnlineRobot();
    setNavReadyCapabilities();
    const localMap = { robot_id: "R01", frame_id: "map", map_source: "LOCAL_MAP" as const,
      active_map_id: "saved-R01-1", active_map_revision: "artifact-1", width: 20, height: 20,
      resolution: 0.1, origin: { x: -1, y: -1, yaw: 0 }, data: Array(400).fill(0) };
    const localPose = { x: 0.25, y: 0.5, yaw: 0.4, frame_id: "map", map_id: "saved-R01-1",
      map_revision: "artifact-1", map_source: "LOCAL_MAP", pose_source: "TF", valid: true,
      timestamp: new Date().toISOString() };
    useStore.getState().setRobotDetail("R01", { runtimeMapSnapshot: localMap,
      activeLocalMapId: "saved-R01-1", activeLocalMapRevision: "artifact-1", localMapSyncStatus: "LOCAL_ONLY" });
    useStore.setState({ twin: { ...useStore.getState().twin!, robots: {
      ...useStore.getState().twin!.robots, R01: { ...r01(), control_mode: "AUTONOMOUS", active_map_pose: localPose },
    } } });
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { await settleUi(); });
    const localCanvas = container.querySelector<HTMLCanvasElement>('[data-testid="active-navigation-map-2d"] canvas');
    expect(localCanvas?.dataset.mapId).toBe("saved-R01-1");
    expect(container.querySelector('[data-testid="global-warehouse-map"]')).toBeNull();
    await act(async () => { localCanvas?.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    expect(container.querySelector('[data-testid="selected-point-status"]')).toBeTruthy();
    await act(async () => { localCanvas?.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    const request = vi.mocked(wsSend).mock.calls.map(([message]) => message)
      .filter((message) => message.type === "PATH_PREVIEW_REQUEST").at(-1);
    expect(request).toMatchObject({ type: "PATH_PREVIEW_REQUEST", frame_id: "map",
      source_type: "ACTIVE_MAP_POINT", source_map_id: "saved-R01-1", source_map_revision: "artifact-1",
      map_id: "saved-R01-1", map_revision: "artifact-1",
      active_map_id: "saved-R01-1", active_map_revision: "artifact-1" });
    if (!request || request.type !== "PATH_PREVIEW_REQUEST") throw new Error("local path preview request was not emitted");

    const approved = { robot_id: "R01", request_id: request.request_id, status: "VALID" as const,
      source_type: "ACTIVE_MAP_POINT" as const, source_map_id: "saved-R01-1", source_map_revision: "artifact-1",
      frame_id: "map" as const, path: [[0, 0], [1, 1]] as Array<[number, number]>, path_length_m: 1.4,
      goal: { x: request.x!, y: request.y!, yaw: request.yaw! }, timestamp: new Date().toISOString(),
      active_map_id: "saved-R01-1", active_map_revision: "artifact-1" };
    act(() => useStore.getState().setRobotDetail("R01", { pathPreview: approved }));
    expect(buttonNamed("SEND GOAL")?.disabled).toBe(false);

    const replacementMap = { ...localMap, active_map_id: "saved-R01-2", active_map_revision: "artifact-2" };
    await act(async () => { useStore.getState().setRobotDetail("R01", { runtimeMapSnapshot: replacementMap,
      activeLocalMapId: "saved-R01-2", activeLocalMapRevision: "artifact-2" }); await settleUi(); });
    expect(buttonNamed("SEND GOAL")?.disabled).toBe(true);
    expect(container.querySelector('[data-testid="selected-point-status"]')).toBeNull();
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type).filter((type) => type === "PATH_PREVIEW_INVALIDATE").length).toBeGreaterThan(0);
  });

  it("requests a fresh detail map after saved-map identity changes and reconnects, and fails closed while it is missing", async () => {
    setOnlineRobot();
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { await settleUi(); });
    const requestsBeforeMapChange = vi.mocked(wsSend).mock.calls
      .filter(([message]) => message.type === "ROBOT_DETAIL_VIEW").length;

    await act(async () => {
      useStore.getState().setRobotDetail("R01", {
        activeLocalMapId: "saved-R01-1", activeLocalMapRevision: "artifact-1",
      });
      await settleUi();
    });
    await act(async () => { buttonNamed("LOCALIZATION")?.click(); await settleUi(); });
    expect(container.querySelector('[data-testid="localization-map-waiting"]')?.textContent)
      .toBe("WAITING FOR ACTIVE SAVED MAP SNAPSHOT...");
    expect(buttonNamed("SET POSE")).toBeUndefined();
    const requestsAfterMapChange = vi.mocked(wsSend).mock.calls
      .filter(([message]) => message.type === "ROBOT_DETAIL_VIEW");
    expect(requestsAfterMapChange.length).toBeGreaterThan(requestsBeforeMapChange);
    expect(requestsAfterMapChange.at(-1)?.[0]).toMatchObject({ view: "LIDAR_2D", robot_id: "R01" });

    await act(async () => { useStore.setState({ websocketState: "RECONNECTING" }); await settleUi(); });
    const requestsWhileReconnecting = vi.mocked(wsSend).mock.calls
      .filter(([message]) => message.type === "ROBOT_DETAIL_VIEW").length;
    await act(async () => { useStore.setState({ websocketState: "CONNECTED" }); await settleUi(); });
    expect(vi.mocked(wsSend).mock.calls.filter(([message]) => message.type === "ROBOT_DETAIL_VIEW").length)
      .toBeGreaterThan(requestsWhileReconnecting);
  });

  it("renders only a matching saved-map snapshot for localization and enables map picking", async () => {
    setOnlineRobot();
    setNavReadyCapabilities();
    const localMap = { robot_id: "R01", frame_id: "map", map_source: "LOCAL_MAP" as const,
      active_map_id: "saved-R01-1", active_map_revision: "artifact-1", width: 20, height: 20,
      resolution: 0.1, origin: { x: -1, y: -1, yaw: 0 }, data: Array(400).fill(0) };
    useStore.getState().setRobotDetail("R01", {
      runtimeMapSnapshot: null, activeLocalMapId: localMap.active_map_id,
      activeLocalMapRevision: localMap.active_map_revision,
    });
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { buttonNamed("LOCALIZATION")?.click(); await settleUi(); });
    expect(buttonNamed("SET POSE")).toBeUndefined();

    await act(async () => {
      useStore.getState().setRobotDetail("R01", { runtimeMapSnapshot: localMap });
      await settleUi();
    });
    let canvas = container.querySelector<HTMLCanvasElement>('.local-pose-map canvas[data-map-source="LOCAL_MAP"]');
    expect(canvas).toBeTruthy();
    expect(buttonNamed("SET POSE")?.disabled).toBe(false);
    await act(async () => { buttonNamed("SET POSE")?.click(); });
    await act(async () => { canvas?.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 350, clientY: 160 })); });
    const coordinateInput = (name: "X" | "Y") => container.querySelector<HTMLInputElement>(`[aria-label="Initial pose ${name} in meters"]`);
    expect(Number.isFinite(Number(coordinateInput("X")?.value))).toBe(true);
    expect(Number.isFinite(Number(coordinateInput("Y")?.value))).toBe(true);
    expect(coordinateInput("X")?.value).not.toBe("");
    expect(coordinateInput("Y")?.value).not.toBe("");

    const picked = { x: coordinateInput("X")?.value, y: coordinateInput("Y")?.value };
    act(() => {
      mapPointer(canvas!, "pointerdown", 350, 160);
      mapPointer(canvas!, "pointermove", 380, 190);
      mapPointer(canvas!, "pointerup", 380, 190);
      canvas!.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 380, clientY: 190 }));
    });
    expect(coordinateInput("X")?.value).toBe(picked.x);
    expect(coordinateInput("Y")?.value).toBe(picked.y);
    act(() => {
      mapPointer(canvas!, "pointerdown", 400, 180);
      mapPointer(canvas!, "pointermove", 402, 180);
      mapPointer(canvas!, "pointerup", 402, 180);
      canvas!.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 402, clientY: 180 }));
    });
    expect(coordinateInput("X")?.value).toBe(picked.x);
    expect(Number(container.querySelector<HTMLInputElement>('[aria-label="Initial pose yaw in degrees"]')?.value)).not.toBe(0);

    await act(async () => {
      useStore.getState().setRobotDetail("R01", {
        activeLocalMapId: "saved-R01-2", activeLocalMapRevision: "artifact-2",
      });
      await settleUi();
    });
    expect(container.querySelector('.local-pose-map canvas[data-map-source="LOCAL_MAP"]')).toBeNull();
    expect(buttonNamed("SET POSE")).toBeUndefined();
    expect(container.querySelector('[data-testid="localization-map-waiting"]')?.textContent)
      .toBe("WAITING FOR ACTIVE SAVED MAP SNAPSHOT...");

    const wrongRevision = { ...localMap, active_map_id: "saved-R01-2", active_map_revision: "stale-revision" };
    await act(async () => { useStore.getState().setRobotDetail("R01", { runtimeMapSnapshot: wrongRevision }); await settleUi(); });
    expect(container.querySelector('.local-pose-map canvas[data-map-source="LOCAL_MAP"]')).toBeNull();
    expect(buttonNamed("SET POSE")).toBeUndefined();

    const matchingReplacement = { ...wrongRevision, active_map_revision: "artifact-2" };
    await act(async () => { useStore.getState().setRobotDetail("R01", { runtimeMapSnapshot: matchingReplacement }); await settleUi(); });
    canvas = container.querySelector<HTMLCanvasElement>('.local-pose-map canvas[data-map-source="LOCAL_MAP"]');
    expect(canvas).toBeTruthy();
    expect(buttonNamed("SET POSE")?.disabled).toBe(false);
  });

  it("normalizes near-zero localization coordinates for display without changing the pose", async () => {
    setOnlineRobot();
    setNavReadyCapabilities();
    const localMap = { robot_id: "R01", frame_id: "map", map_source: "LOCAL_MAP" as const,
      active_map_id: "saved-R01-1", active_map_revision: "artifact-1", width: 20, height: 20,
      resolution: 0.1, origin: { x: -1, y: -1, yaw: 0 }, data: Array(400).fill(0) };
    useStore.getState().setRobotDetail("R01", { runtimeMapSnapshot: localMap,
      activeLocalMapId: localMap.active_map_id, activeLocalMapRevision: localMap.active_map_revision });
    useStore.setState({ robotCapabilities: { R01: { ...useStore.getState().robotCapabilities.R01!, localization_ready: true } },
      twin: { ...useStore.getState().twin!, robots: { R01: { ...r01(), active_map_pose: {
        x: -0.0001, y: -0.0001, yaw: -0.0001, frame_id: "map", map_id: localMap.active_map_id,
        map_revision: localMap.active_map_revision, map_source: "LOCAL_MAP", pose_source: "TF", valid: true,
        timestamp: new Date().toISOString(),
      } } } } });
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { buttonNamed("LOCALIZATION")?.click(); await settleUi(); });
    const currentPoseCard = container.querySelector("#localization-current-pose-heading")?.closest("section");
    expect(currentPoseCard?.textContent).toContain("0.000 m");
    expect(currentPoseCard?.textContent).not.toContain("-0.000");
    expect(useStore.getState().twin.robots.R01.active_map_pose?.x).toBe(-0.0001);
  });

  it("saves a Unified SLAM map without offering a full-stack load or pose-graph restart", async () => {
    setOnlineRobot("UNIFIED");
    useStore.setState({ robotCapabilities: { R01: {
      mapping_available: true, mapping_active: true, nav2_available: true, nav2_ready: true,
      manual_available: true, goal_available: false, map_ready: true, tag_navigation_available: false,
    } } });
    useStore.getState().setRobotDetail("R01", { slam2dMap: {
      robot_id: "R01", frame_id: "map", map_source: "SLAM_TOOLBOX",
      mapping_session_id: "session-ui", active_map_id: "SLAM-session-ui",
      active_map_revision: "revision-1", width: 10, height: 10, resolution: 0.05,
      origin: { x: 0, y: 0, yaw: 0 }, data: Array(100).fill(-1), known_cells: 1,
    }, mappingSessionId: "session-ui", mappingState: "MAPPING" });
    let mappingState = "MAPPING";
    let maps: LocalRobotMap[] = [];
    const savedMap: LocalRobotMap = {
      id: "local-map-1", name: "floor_1", robot_id: "R01", created_at: "2026-09-30T00:00:00Z",
      resolution: 0.05, origin: [0, 0, 0], revision: "rev-2", frame_id: "map", width: 100, height: 100,
      slam_session_state: { status: "AVAILABLE", engine: "SLAM_TOOLBOX", artifact_id: "session-artifact" },
    };
    vi.mocked(api.getLocalRobotMaps).mockImplementation(async () => ({
      robot_id: "R01", maps, runtime_mode: "GAZEBO_ROS", mapping_state: mappingState,
      mapping_duration_s: 12, active_local_map_id: null, local_active_map_id: null,
      local_active_map_revision: null, active_map_id: "SLAM-session-ui", active_map_revision: "revision-1",
      canonical_map_revision: 21, map_sync_status: "LOCAL_ONLY",
      robot_control_mode: "MANUAL", robot_stopped: true,
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
    vi.mocked(api.loadLocalRobotMap).mockClear();
    vi.mocked(api.resumeLocalRobotSlamSession).mockClear();
    vi.mocked(api.requestLocalRuntimeMode).mockClear();
    vi.mocked(wsSetRobotMode).mockClear();
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { buttonNamed("MAPPING")?.click(); await settleUi(); });
    expect(container.textContent).toContain("LIVE SLAM MAP");
    await act(async () => { buttonNamed("PAUSE MAPPING")?.click(); await settleUi(); });
    expect(api.setMappingState).toHaveBeenCalledWith("R01", "stop");
    expect(container.textContent).toContain("MAPPING PAUSED");
    const nameInput = container.querySelector<HTMLInputElement>('input[placeholder="warehouse_floor_1"]');
    expect(nameInput).toBeTruthy();
    await act(async () => { if (nameInput) setInputValue(nameInput, "floor_1"); });
    await act(async () => { buttonNamed("SAVE MAP")?.click(); await settleUi(); });
    expect(api.saveLocalRobotMap).toHaveBeenCalledWith("R01", "floor_1");
    expect(container.textContent).toContain("floor_1");
    expect(container.querySelector(".mapping-map-option")?.textContent).toContain("floor_1");

    await act(async () => { buttonNamed("RESUME MAPPING")?.click(); await settleUi(); });
    expect(api.setMappingState).toHaveBeenCalledWith("R01", "start");
    expect(buttonNamed("LOAD SAVED MAP")?.disabled).toBe(false);
    expect(buttonNamed("RESUME SAVED SLAM SESSION")?.disabled).toBe(true);
    const advancedDetails = container.querySelector<HTMLDetailsElement>(".hmi-advanced-details")!;
    await act(async () => { advancedDetails.open = true; advancedDetails.dispatchEvent(new Event("toggle", { bubbles: true })); });
    expect(container.textContent).toContain("supervised SLAM-to-Navigation handoff");
    expect(container.textContent).not.toContain("SWITCH TO NAVIGATION");
    expect(api.loadLocalRobotMap).not.toHaveBeenCalled();
    expect(api.resumeLocalRobotSlamSession).not.toHaveBeenCalled();
    expect(api.requestLocalRuntimeMode).not.toHaveBeenCalled();
    expect(wsSetRobotMode).not.toHaveBeenCalled();
  });

  it("restores an available saved SLAM session through the supervised backend transition", async () => {
    vi.useFakeTimers();
    setOnlineRobot("NAVIGATION");
    const savedMap: LocalRobotMap = {
      id: "saved-slam-session-a", name: "north-floor", robot_id: "R01", created_at: "2026-10-01T10:00:00Z",
      resolution: 0.05, origin: [-2, -3, 0], revision: "rev-slam-a", frame_id: "map", width: 80, height: 120,
      slam_session_state: { status: "AVAILABLE", engine: "SLAM_TOOLBOX", artifact_id: "session-a" },
    };
    vi.mocked(api.getLocalRobotMaps).mockResolvedValue({
      robot_id: "R01", maps: [savedMap], runtime_mode: "GAZEBO_ROS", mapping_state: "PAUSED",
      mapping_duration_s: 20, active_local_map_id: null, local_active_map_id: null,
      local_active_map_revision: null, active_map_id: null, active_map_revision: null,
      canonical_map_revision: 21, map_sync_status: null, robot_control_mode: "MANUAL", robot_stopped: true,
    });
    vi.mocked(api.getLocalRuntimeMode).mockResolvedValue({
      robot_id: "R01", current_mode: "NAVIGATION",
      transition: { robot_id: "R01", request_id: "resume-slam-a", mode: "mapping", status: "READY" },
    });
    const transitioning: LocalSlamResumeResult = {
      ok: true, status: "TRANSITIONING", robot_id: "R01", map: savedMap, map_id: savedMap.id,
      request_id: "resume-slam-a", mapping_state: "RESTORING_SESSION", message: "restoring saved graph",
    };
    vi.mocked(api.resumeLocalRobotSlamSession)
      .mockResolvedValueOnce(transitioning)
      .mockResolvedValueOnce(transitioning)
      .mockResolvedValueOnce({
        ok: true, status: "RESUMED", robot_id: "R01", map: savedMap, map_id: savedMap.id,
        request_id: "resume-slam-a", mapping_state: "MAPPING", restore_evidence: { passed: true },
        message: "saved map restored",
      });
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { buttonNamed("MAPPING")?.click(); await Promise.resolve(); await Promise.resolve(); });
    const resumeButton = buttonNamed("RESUME SAVED SLAM SESSION");
    expect(resumeButton?.disabled).toBe(false);
    await act(async () => { resumeButton?.click(); await Promise.resolve(); });
    expect(container.querySelector('[role="dialog"][aria-labelledby="confirm-slam-resume-title"]')).not.toBeNull();
    expect(api.resumeLocalRobotSlamSession).not.toHaveBeenCalled();
    await act(async () => {
      buttonNamed("RESTORE SLAM SESSION")?.click();
      await Promise.resolve();
      await vi.advanceTimersByTimeAsync(2000);
    });
    expect(api.resumeLocalRobotSlamSession).toHaveBeenCalledTimes(3);
    expect(api.resumeLocalRobotSlamSession).toHaveBeenNthCalledWith(1, "R01", savedMap.id);
    expect(container.textContent).toContain("SLAM SESSION RESTORED · north-floor");
    expect(useStore.getState().robotDetail.R01?.activeLocalMapId ?? null).toBeNull();
    expect(container.querySelector('[role="dialog"][aria-labelledby="confirm-slam-resume-title"]')).toBeNull();
  });

  it("loads a selected saved map from Unified through a 202 supervised transition", async () => {
    vi.useFakeTimers();
    setOnlineRobot("UNIFIED");
    setNavReadyCapabilities();
    useStore.setState({ twin: { ...useStore.getState().twin!, robots: {
      ...useStore.getState().twin!.robots,
      R01: { ...r01(), control_mode: "MANUAL", navigation_state: "MANUAL", vx: 0, vy: 0, wz: 0 },
    } } });
    const savedMap: LocalRobotMap = {
      id: "saved-floor-a", name: "floor_a", robot_id: "R01", created_at: "2026-10-01T10:00:00Z",
      resolution: 0.05, origin: [-2, -3, 0], revision: "rev-a", frame_id: "map", width: 80, height: 120,
      map_kind: "SAVED_LOCAL_MAP", canonical_map_promoted: false,
    };
    let activeId: string | null = null;
    vi.mocked(api.getLocalRobotMaps).mockImplementation(async () => ({
      robot_id: "R01", maps: [savedMap], runtime_mode: "GAZEBO_ROS", mapping_state: "MAPPING",
      mapping_duration_s: 20, active_local_map_id: activeId, local_active_map_id: activeId,
      local_active_map_revision: activeId ? savedMap.revision : null,
      active_map_id: activeId, active_map_revision: activeId ? savedMap.revision : null,
      canonical_map_revision: 21, map_sync_status: "LOCAL_ONLY",
      robot_control_mode: "MANUAL", robot_stopped: true,
    }));
    vi.mocked(api.getLocalRuntimeMode).mockResolvedValue({
      robot_id: "R01", current_mode: "NAVIGATION",
      transition: { robot_id: "R01", request_id: "load-a-1", mode: "navigation", status: "READY" },
    });
    vi.mocked(api.loadLocalRobotMap)
      .mockResolvedValueOnce({
        ok: true, status: "TRANSITIONING", robot_id: "R01", active_map: savedMap,
        active_map_id: savedMap.id, active_map_revision: savedMap.revision,
        request_id: "load-a-1", phase: "PAUSING_MAPPING",
        transition: { robot_id: "R01", request_id: "load-a-1", mode: "navigation", status: "REQUESTED" },
        message: "supervised transition queued",
      })
      .mockImplementationOnce(async () => {
        activeId = savedMap.id;
        return { ok: true, status: "LOADED", robot_id: "R01", active_map: savedMap,
          active_map_id: savedMap.id, active_map_revision: savedMap.revision,
          map_source: "LOCAL_MAP", map_sync_status: "LOCAL_ONLY", message: "confirmed" };
      });
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { buttonNamed("MAPPING")?.click(); await Promise.resolve(); await Promise.resolve(); });
    expect(buttonNamed("LOAD SAVED MAP")?.disabled).toBe(false);
    await act(async () => {
      buttonNamed("LOAD SAVED MAP")?.click();
      await Promise.resolve();
    });
    expect(container.querySelector('[role="dialog"][aria-labelledby="confirm-saved-map-title"]')).not.toBeNull();
    expect(api.loadLocalRobotMap).not.toHaveBeenCalled();
    expect(useStore.getState().robotDetail.R01?.activeLocalMapId ?? null).toBeNull();
    await act(async () => {
      buttonNamed("CONFIRM LOAD MAP")?.click();
      await Promise.resolve();
      await vi.advanceTimersByTimeAsync(1000);
      await Promise.resolve();
    });
    expect(api.loadLocalRobotMap).toHaveBeenCalledTimes(2);
    expect(container.textContent).toContain("MAP LOADED · floor_a");
    expect(container.querySelector(".local-map-active-badge")?.textContent).toContain("ACTIVE");
  });

  it("restores the existing MANUAL control mode after runtime restart before continuing map load", async () => {
    vi.useFakeTimers();
    setOnlineRobot("UNIFIED");
    setNavReadyCapabilities();
    useStore.setState({ twin: { ...useStore.getState().twin!, robots: {
      ...useStore.getState().twin!.robots,
      R01: { ...r01(), control_mode: "MANUAL", navigation_state: "MANUAL", vx: 0, vy: 0, wz: 0 },
    } } });
    const savedMap: LocalRobotMap = {
      id: "saved-floor-manual", name: "floor_manual", robot_id: "R01", created_at: "2026-10-01T10:00:00Z",
      resolution: 0.05, origin: [-2, -3, 0], revision: "rev-manual", frame_id: "map", width: 80, height: 120,
      map_kind: "SAVED_LOCAL_MAP", canonical_map_promoted: false,
    };
    vi.mocked(api.getLocalRobotMaps).mockResolvedValue({
      robot_id: "R01", maps: [savedMap], runtime_mode: "GAZEBO_ROS", mapping_state: "PAUSED",
      mapping_duration_s: 20, active_local_map_id: null, local_active_map_id: null,
      local_active_map_revision: null, active_map_id: "SLAM-session", active_map_revision: "session-rev",
      canonical_map_revision: 21, map_sync_status: "LOCAL_ONLY", robot_control_mode: "MANUAL",
      robot_stopped: true,
    });
    vi.mocked(api.getLocalRuntimeMode).mockResolvedValue({
      robot_id: "R01", current_mode: "NAVIGATION",
      transition: { robot_id: "R01", request_id: "load-manual-1", mode: "navigation", status: "READY" },
    });
    vi.mocked(api.loadLocalRobotMap)
      .mockResolvedValueOnce({
        ok: true, status: "TRANSITIONING", robot_id: "R01", active_map: savedMap,
        active_map_id: savedMap.id, active_map_revision: savedMap.revision,
        request_id: "load-manual-1", phase: "WAITING_FOR_MANUAL",
        transition: { robot_id: "R01", request_id: "load-manual-1", mode: "navigation", status: "READY" },
        message: "restore MANUAL before map load",
      })
      .mockResolvedValueOnce({
        ok: true, status: "LOADED", robot_id: "R01", active_map: savedMap,
        active_map_id: savedMap.id, active_map_revision: savedMap.revision,
        map_source: "LOCAL_MAP", map_sync_status: "LOCAL_ONLY", message: "confirmed",
      });
    vi.mocked(wsSetRobotMode).mockClear();
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { buttonNamed("MAPPING")?.click(); await Promise.resolve(); await Promise.resolve(); });
    await act(async () => {
      buttonNamed("LOAD SAVED MAP")?.click();
      await Promise.resolve();
    });
    expect(container.querySelector('[role="dialog"][aria-labelledby="confirm-saved-map-title"]')).not.toBeNull();
    await act(async () => {
      buttonNamed("CONFIRM LOAD MAP")?.click();
      await Promise.resolve();
      await vi.advanceTimersByTimeAsync(1000);
      await Promise.resolve();
    });
    expect(wsSetRobotMode).toHaveBeenCalledWith("R01", "MANUAL");
    expect(api.loadLocalRobotMap).toHaveBeenCalledTimes(2);
    expect(container.textContent).toContain("MAP LOADED · floor_manual");
  });

  it("rediscovers the active saved-map identity from the backend after browser refresh", async () => {
    setOnlineRobot("NAVIGATION");
    const savedMap: LocalRobotMap = {
      id: "saved-floor-refresh", name: "floor_refresh", robot_id: "R01", created_at: "2026-10-01T10:00:00Z",
      resolution: 0.05, origin: [-2, -3, 0], revision: "rev-refresh", frame_id: "map", width: 80, height: 120,
      map_kind: "SAVED_LOCAL_MAP", canonical_map_promoted: false,
    };
    vi.mocked(api.getLocalRobotMaps).mockResolvedValue({
      robot_id: "R01", maps: [savedMap], runtime_mode: "GAZEBO_ROS", mapping_state: "INACTIVE",
      mapping_duration_s: 0, active_local_map_id: savedMap.id, local_active_map_id: savedMap.id,
      local_active_map_revision: savedMap.revision, active_map_id: savedMap.id,
      active_map_revision: savedMap.revision, canonical_map_revision: 21,
      map_sync_status: "LOCAL_ONLY", robot_control_mode: "MANUAL", robot_stopped: true,
    });
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { buttonNamed("MAPPING")?.click(); await settleUi(); });
    expect(useStore.getState().robotDetail.R01?.activeLocalMapId).toBe(savedMap.id);
    expect(useStore.getState().robotDetail.R01?.activeLocalMapRevision).toBe(savedMap.revision);
    expect(container.querySelector(".local-map-active-badge")?.textContent).toContain("ACTIVE");
  });

  it("does not present a cached Nav2/canonical grid as the accumulated SLAM map", async () => {
    setOnlineRobot("MAPPING");
    useStore.getState().setRobotDetail("R01", { runtimeMapSnapshot: {
      robot_id: "R01", frame_id: "map", map_source: "NAV2_MAP",
      width: 1, height: 1, resolution: 0.05, origin: { x: 0, y: 0, yaw: 0 }, data: [100],
    } });
    renderNode(<ControlDetailHarness robotId="R01" />);
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
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { buttonNamed("MAPPING")?.click(); await settleUi(); });

    expect(container.textContent).toContain("LIVE SLAM MAP");
    expect(container.textContent).toContain("EXPLORED AREA");
    expect(container.textContent).toContain("TF VALIDITY");
    const toggles = container.querySelectorAll(".mapping-layer-toolbar button");
    expect(toggles).toHaveLength(4);
    expect(Array.from(toggles).map((button) => button.textContent?.trim())).toEqual([
      "ROBOT", "SCAN", "TRAJECTORY", "GRID",
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
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { buttonNamed("MAPPING")?.click(); await settleUi(); });

    expect(container.textContent).toContain("Waiting for a fresh accumulated SLAM Toolbox /map");
    expect(container.querySelector('[aria-label="Select map frame initial robot position"]')).toBeNull();
  });

  it("uses the two-click initial pose workflow and submits only after dialog confirmation", async () => {
    setOnlineRobot();
    setNavReadyCapabilities();
    const liveTwin = useStore.getState().twin!;
    useStore.setState({ twin: { ...liveTwin, robots: { ...liveTwin.robots, R01: {
      ...liveTwin.robots.R01!, control_mode: "MANUAL", navigation_state: "IDLE", vx: 0, vy: 0, wz: 0,
    } } } });
    vi.mocked(api.initializeLocalRobotPose).mockImplementationOnce(async (_robotId, pose) => ({
      ok: true, robot_id: "R01", frame_id: "map", pose,
      active_map_id: "NAV2-R01-map", active_map_revision: "nav2-r21",
      localization_owner: "amcl", initial_pose_interface: "/initialpose",
      request_stamp_s: 10, tf_stamp_s: 10.3, tf_age_s: 0.01, position_error_m: 0.01,
      yaw_error_rad: 0.01, position_tolerance_m: 0.05, yaw_tolerance_rad: 0.05,
      tf_confirmation_duration_sim_s: 0.3, tf_confirmation_samples: 3,
    })).mockRejectedValueOnce(new Error("localization service unavailable"));
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { buttonNamed("LOCALIZATION")?.click(); await Promise.resolve(); });
    const picker = container.querySelector<HTMLCanvasElement>('[aria-label="Localization map and initial pose picker"]');
    expect(picker).toBeTruthy();
    await act(async () => { await settleUi(); await settleUi(); });
    expect(api.initializeLocalRobotPose).not.toHaveBeenCalled();
    await act(async () => { buttonNamed("SET POSE")?.click(); });
    await act(async () => { picker?.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    expect(picker?.dataset.pointSelectionState).toBe("SELECT_YAW");
    expect(api.initializeLocalRobotPose).not.toHaveBeenCalled();
    await act(async () => {
      mapPointer(picker!, "pointermove", 380, 180);
      picker?.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 380, clientY: 180 }));
    });
    expect(picker?.dataset.pointSelectionState).toBe("WAITING_FOR_DESTINATION");
    expect(container.querySelector<HTMLInputElement>('[aria-label="Initial pose yaw in degrees"]')?.value).toBe("0.000");
    await act(async () => { buttonNamed("+15°")?.click(); });
    expect(container.querySelector<HTMLInputElement>('[aria-label="Initial pose yaw in degrees"]')?.value).toBe("15.000");
    await act(async () => { buttonNamed("RAD")?.click(); });
    expect(Number(container.querySelector<HTMLInputElement>('[aria-label="Initial pose yaw in radians"]')?.value))
      .toBeCloseTo(Math.PI / 12, 3);
    await act(async () => { buttonNamed("SET INITIAL POSE")?.click(); await Promise.resolve(); });
    expect(container.querySelector('[role="dialog"]')?.textContent).toContain("NAV2-R01-map");
    expect(api.initializeLocalRobotPose).not.toHaveBeenCalled();
    await act(async () => { buttonNamed("CONFIRM SET POSE")?.click(); await Promise.resolve(); });
    expect(api.initializeLocalRobotPose).toHaveBeenCalledTimes(1);
    expect(api.initializeLocalRobotPose.mock.calls[0][1]).toMatchObject({ frame_id: "map",
      active_map_id: "NAV2-R01-map", active_map_revision: "nav2-r21" });
    expect(Number.isFinite(api.initializeLocalRobotPose.mock.calls[0][1].x)).toBe(true);
    expect(Number.isFinite(api.initializeLocalRobotPose.mock.calls[0][1].y)).toBe(true);
    expect(api.initializeLocalRobotPose.mock.calls[0][1].yaw).toBeCloseTo(Math.PI / 12, 4);
    expect(container.textContent).toContain("REQUEST ACCEPTED · WAITING FOR LOCALIZATION CONFIRMATION");
    expect(container.querySelector('[data-testid="localization-state-badge"]')?.textContent)
      .toBe("LOCALIZATION DEGRADED");
    await act(async () => { buttonNamed("SET INITIAL POSE")?.click(); await Promise.resolve(); });
    await act(async () => { buttonNamed("CONFIRM SET POSE")?.click(); await Promise.resolve(); });
    expect(container.textContent).toContain("localization service unavailable");
  });

  it("clears an unsubmitted pose draft when the active map revision changes", async () => {
    setOnlineRobot();
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { buttonNamed("LOCALIZATION")?.click(); await settleUi(); });
    const picker = container.querySelector<HTMLCanvasElement>('[aria-label="Localization map and initial pose picker"]')!;
    await act(async () => { await settleUi(); await settleUi(); });
    await act(async () => { buttonNamed("SET POSE")?.click(); });
    await act(async () => { picker.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    expect(container.querySelector('[data-testid="initial-pose-validation"]')?.textContent)
      .toContain("CLICK A SECOND TIME TO CONFIRM THE HEADING");

    const detail = useStore.getState().robotDetail.R01!;
    useStore.getState().setRobotDetail("R01", { runtimeMapSnapshot: {
      ...detail.runtimeMapSnapshot!, active_map_revision: "nav2-r22",
    } });
    await act(async () => { await settleUi(); await settleUi(); });

    expect(container.querySelector('[data-testid="initial-pose-validation"]')?.textContent)
      .toContain("SELECT A MAP POSITION OR ENTER X, Y AND YAW");
    expect(api.initializeLocalRobotPose).not.toHaveBeenCalled();
    const history = container.querySelector<HTMLDetailsElement>('[data-testid="localization-event-history"]');
    if (history) history.open = true;
    expect(history?.textContent).toContain("MAP REVISION CHANGED");
  });

  it("prevents duplicate pose requests and keeps an accepted request pending after robot disconnect", async () => {
    setOnlineRobot();
    setNavReadyCapabilities();
    const liveTwin = useStore.getState().twin!;
    useStore.setState({ twin: { ...liveTwin, robots: { ...liveTwin.robots, R01: {
      ...liveTwin.robots.R01!, control_mode: "MANUAL", navigation_state: "IDLE", vx: 0, vy: 0, wz: 0,
    } } } });
    let resolveRequest: (result: LocalInitialPoseResult) => void = () => undefined;
    vi.mocked(api.initializeLocalRobotPose).mockImplementationOnce(() => new Promise((resolve) => { resolveRequest = resolve; }));
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { buttonNamed("LOCALIZATION")?.click(); await settleUi(); });
    const picker = container.querySelector<HTMLCanvasElement>('[aria-label="Localization map and initial pose picker"]')!;
    await act(async () => { await settleUi(); await settleUi(); });
    await act(async () => { buttonNamed("SET POSE")?.click(); });
    await act(async () => { picker.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 320, clientY: 180 })); });
    await act(async () => {
      mapPointer(picker, "pointermove", 380, 180);
      picker.dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: 380, clientY: 180 }));
    });
    await act(async () => { buttonNamed("SET INITIAL POSE")?.click(); });
    await act(async () => { buttonNamed("CONFIRM SET POSE")?.click(); });
    expect(api.initializeLocalRobotPose).toHaveBeenCalledTimes(1);
    expect(container.querySelector<HTMLButtonElement>('[role="dialog"] button:last-child')?.disabled).toBe(true);
    await act(async () => { buttonNamed("CONFIRM SET POSE")?.click(); });
    expect(api.initializeLocalRobotPose).toHaveBeenCalledTimes(1);

    await act(async () => { useStore.setState({ websocketState: "RECONNECTING" }); await settleUi(); });
    const requestPose = vi.mocked(api.initializeLocalRobotPose).mock.calls[0][1];
    await act(async () => {
      resolveRequest({ ok: true, robot_id: "R01", frame_id: "map",
        pose: { x: requestPose.x, y: requestPose.y, yaw: requestPose.yaw },
        active_map_id: requestPose.active_map_id, active_map_revision: requestPose.active_map_revision,
        localization_owner: "amcl", initial_pose_interface: "/initialpose",
        request_stamp_s: 10, tf_stamp_s: 10.3, tf_age_s: 0.01,
        position_error_m: 0.01, yaw_error_rad: 0.01, position_tolerance_m: 0.05, yaw_tolerance_rad: 0.05,
        tf_confirmation_duration_sim_s: 0.3, tf_confirmation_samples: 3 });
      await settleUi();
    });
    expect(container.querySelector('[data-testid="initial-pose-request-state"]')?.textContent)
      .toContain("REQUEST ACCEPTED · WAITING FOR ROBOT TELEMETRY");
    expect(container.querySelector('[data-testid="localization-state-badge"]')?.textContent)
      .toBe("UNAVAILABLE");
    expect(vi.mocked(wsSend).mock.calls.map(([message]) => message.type)).not.toContain("NAV_GOAL");
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
    renderNode(<ControlDetailHarness robotId="R01" />);
    await act(async () => { buttonNamed("VDA5050")?.click(); });
    await act(async () => { await Promise.resolve(); });
    expect(container.querySelector('input[type="password"]')?.getAttribute("type")).toBe("password");
    expect(container.textContent).toContain("Instant Actions");
    expect(container.textContent).toContain("Not implemented");
    expect(container.textContent).not.toContain("NOT_IMPLEMENTED");
    const capability = container.querySelector(".local-vda-capability");
    expect(capability?.textContent).toContain("Instant Actions");
    expect(capability?.textContent).toContain("Not implemented");
    expect(capability?.querySelector("small")?.textContent)
      .toBe("Support: Not supported. Execution is not implemented in this interface.");
    expect(container.querySelector('[data-testid="vda5050-connection-overview"]')?.textContent).toContain("Disabled");
    expect(container.querySelector('[data-testid="vda5050-connection-overview"]')?.textContent).toContain("broker.local:1883");
    expect(config.instant_actions_supported).toBe(false);
    expect(buttonNamed("SAVE & APPLY")?.disabled).toBe(true);

    const host = Array.from(container.querySelectorAll("label")).find((label) => label.textContent?.includes("MQTT host"))?.querySelector("input");
    await act(async () => {
      if (host) setInputValue(host, "new-broker.local");
    });
    expect(buttonNamed("SAVE & APPLY")?.disabled).toBe(false);
    const taskToggle = Array.from(container.querySelectorAll("label")).find((label) => label.textContent?.includes("Allow Task"))?.querySelector<HTMLInputElement>('input[type="checkbox"]');
    await act(async () => { taskToggle?.click(); });
    expect(container.textContent).toContain("New VDA5050 orders are blocked for this robot");
    await act(async () => { buttonNamed("TEST CONNECTION")?.click(); await Promise.resolve(); });
    expect(api.testVda5050Connection).toHaveBeenCalledWith("R01", expect.objectContaining({ mqtt_host: "new-broker.local" }));
    expect(container.textContent).toContain("Test failed · REFUSED · connection refused");
    await act(async () => { buttonNamed("SAVE & APPLY")?.click(); await Promise.resolve(); });
    expect(api.applyVda5050Configuration).toHaveBeenCalledWith("R01", expect.objectContaining({
      mqtt_host: "new-broker.local", allow_task: false, mqtt_password: "",
    }));
    expect(container.textContent).toContain("broker apply failed");
  });
});
