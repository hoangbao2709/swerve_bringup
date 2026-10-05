// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import type { RobotState } from "../src/schema/twin_state";
import { useStore } from "../src/state/store";

vi.mock("../src/components/views/Viewport", () => ({ Viewport: () => <div data-testid="viewport" /> }));
vi.mock("../src/components/shell/Sidebar", () => ({ Sidebar: () => <aside data-testid="sidebar" /> }));
vi.mock("../src/components/ops/Modals", () => ({ Modals: () => null }));
vi.mock("../src/services/backendRuntime", () => ({ backendActions: { play: vi.fn(), pause: vi.fn() } }));
vi.mock("../src/services/scheduler", () => ({ schedulerApi: { overview: () => new Promise(() => undefined) } }));
vi.mock("../src/services/ws", () => ({ onScheduleUpdated: () => () => undefined }));

import { OverviewPage } from "../src/components/overview/OverviewPage";

const initialState = useStore.getState();
let root: Root;
let container: HTMLDivElement;

function runtimeRobot(id: string): RobotState {
  const now = new Date().toISOString();
  const pose = {
    x: 14, y: 7.5, yaw: 0.3, frame_id: "map", map_id: "CANONICAL", map_revision: initialState.layoutRevision,
    map_source: "CANONICAL", pose_source: "TF", timestamp: now, valid: true,
  };
  return {
    id, model: "AMR-L", floor: 1, lift_id: null, lift_stage: null, position: [14, 0, 7.5], heading: 0.3,
    velocity: 0, max_speed: null, battery: null, status: "ACTIVE", fsm: "UNKNOWN", health: null,
    current_task_id: null, destination: null, path: [], path_index: 0, load: null, zone: null, eta_s: null,
    fsm_since_tick: 0, stats: null, perception: null, control_mode: null, navigation_state: "IDLE",
    last_telemetry_at: now, pose_frame_id: "map", pose_map_id: "CANONICAL",
    pose_map_revision: initialState.layoutRevision, pose_map_source: "CANONICAL", pose_source: "TF",
    active_map_pose: pose, canonical_pose: pose, slam_pose: null,
  };
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

describe("Overview backend runtime data", () => {
  it("does not create robots before telemetry and labels unavailable values honestly", async () => {
    useStore.setState({ ...initialState, source: "online", twin: { ...initialState.twin, robots: {} }, lastTelemetryAt: null });
    await act(async () => { root.render(<OverviewPage />); await Promise.resolve(); });
    expect(Object.keys(useStore.getState().twin.robots)).toEqual([]);
    expect(container.textContent).toContain("UNKNOWN");
    expect(container.querySelector('[title="UNAVAILABLE"]')).not.toBeNull();
  });

  it("renders the robot identity and fleet status received from backend state", async () => {
    const robot = runtimeRobot("AMR-17");
    useStore.setState({
      ...initialState,
      source: "online",
      lastTelemetryAt: robot.last_telemetry_at ?? null,
      connectedRobotIds: [robot.id],
      twin: { ...initialState.twin, robots: { [robot.id]: robot } },
    });
    await act(async () => { root.render(<OverviewPage />); await Promise.resolve(); });
    expect(container.textContent).toContain("1 idle · 1 total");
    expect(Object.keys(useStore.getState().twin.robots)).toEqual(["AMR-17"]);
  });
});
