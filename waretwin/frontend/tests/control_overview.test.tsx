// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import type { RobotState, TwinState } from "../src/schema/twin_state";
import { RobotControlPage } from "../src/components/control/RobotControlPage";
import { useStore } from "../src/state/store";

const initialState = useStore.getState();
let root: Root;
let container: HTMLDivElement;

function robot(id: string, overrides: Partial<RobotState> = {}): RobotState {
  return {
    id, model: "AMR-L", floor: 1, lift_id: null, lift_stage: null,
    position: [12, 0, 8], heading: 1.2, velocity: 0, max_speed: 1,
    battery: 70, battery_reported: false, status: "ACTIVE", fsm: "IDLE", health: 100,
    current_task_id: null, destination: null, path: [], path_index: 0,
    load: { current: 0, capacity: 1 }, zone: null, eta_s: null, fsm_since_tick: 0,
    stats: { distance_m: 0, tasks_completed: 0, energy_wh: 0, busy_ticks: 0, wait_ticks: 0 },
    perception: { state: "OFF", ahead_m: 0, nearest_m: null, obstacles: [] },
    control_mode: "MANUAL", last_telemetry_at: "2026-10-05T00:00:00Z",
    ...overrides,
  };
}

function render(overrides: Partial<ReturnType<typeof useStore.getState>> = {}) {
  act(() => {
    useStore.setState({ ...initialState, ...overrides });
    root.render(<RobotControlPage />);
  });
}

beforeEach(() => {
  globalThis.IS_REACT_ACT_ENVIRONMENT = true;
  useStore.setState(initialState);
  container = document.createElement("div");
  document.body.appendChild(container);
  root = createRoot(container);
});

afterEach(() => {
  act(() => root.unmount());
  container.remove();
  useStore.setState(initialState);
});

describe("Robot Control overview", () => {
  it("shows runtime status and links each live robot using its dynamic ID", () => {
    render({
      runtimeMode: "GAZEBO_ROS", runtimeState: "UNIFIED", rosConnected: true,
      websocketState: "CONNECTED", connectedRobotIds: ["robot-A"],
      twin: { ...initialState.twin, robots: { "robot-A": robot("robot-A") } } as TwinState,
    });

    expect(container.textContent).toContain("PTAGV");
    expect(container.textContent).toContain("WareTwin");
    expect(container.textContent).toContain("Robot Control");
    expect(container.textContent).toContain("UNIFIED");
    expect(container.textContent).toContain("CONNECTED");
    expect(container.querySelector('a[href="/robots/robot-A/control"]')).not.toBeNull();
    expect(container.textContent).toContain("MANUAL");
    expect(container.textContent).toContain("ONLINE");
  });

  it("uses only map-pose telemetry and marks unreported battery unavailable", () => {
    render({
      runtimeMode: "GAZEBO_ROS", runtimeState: "NAVIGATION", rosConnected: true,
      websocketState: "CONNECTED", connectedRobotIds: ["robot-B"],
      twin: { ...initialState.twin, robots: { "robot-B": robot("robot-B", {
        active_map_pose: null, battery: 70, battery_reported: false,
      }) } } as TwinState,
    });

    expect(container.textContent).toContain("NOT REPORTED");
    expect(container.textContent).not.toContain("70%");
    expect(container.textContent).not.toContain("12.000");

    act(() => useStore.setState({
      twin: { ...useStore.getState().twin!, robots: { "robot-B": robot("robot-B", {
        active_map_pose: {
          x: 1.25, y: 2.5, yaw: 0.3, frame_id: "map", map_id: "CANONICAL",
          map_revision: "22", map_source: "CANONICAL", pose_source: "TF",
          timestamp: "2026-10-05T00:00:01Z", valid: true,
        },
        battery: 81, battery_reported: true,
      }) } } as TwinState,
    }));

    expect(container.textContent).toContain("1.250");
    expect(container.textContent).toContain("81%");
  });

  it("does not invent a robot or metrics before runtime telemetry arrives", () => {
    render({ twin: null as unknown as TwinState, connectedRobotIds: [], rosConnected: false });
    expect(container.textContent).toContain("Waiting for robot runtime data");
    expect(container.textContent).toContain("UNKNOWN");
    expect(container.querySelectorAll("a[href^='/robots/']")).toHaveLength(0);
    expect(container.textContent).not.toContain("R01");
  });
});
