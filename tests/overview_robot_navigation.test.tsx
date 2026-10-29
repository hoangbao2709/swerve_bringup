// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import type { RobotState } from "../src/schema/twin_state";
import { layout, useStore } from "../src/state/store";
import { MapView2D } from "../src/components/views/MapView2D";

const initialState = useStore.getState();
const initialLayout = layout;
let root: Root;
let container: HTMLDivElement;

function runtimeRobot(id: string): RobotState {
  const now = new Date().toISOString();
  const pose = {
    x: 14, y: 7.5, yaw: Math.PI / 2, frame_id: "map", map_id: "CANONICAL",
    map_revision: 21, map_source: "CANONICAL", pose_source: "TF",
    timestamp: now, valid: true,
  };
  return {
    id, model: "AMR-L", floor: 1, lift_id: null, lift_stage: null, position: [14, 0, 7.5], heading: pose.yaw,
    velocity: 0, max_speed: null, battery: null, status: "ACTIVE", fsm: "UNKNOWN", health: null,
    current_task_id: null, destination: null, path: [], path_index: 0, load: null, zone: null, eta_s: null,
    fsm_since_tick: 0, stats: null, perception: null, control_mode: null, navigation_state: "IDLE",
    last_telemetry_at: now, pose_frame_id: "map", pose_map_id: "CANONICAL",
    pose_map_revision: 21, pose_map_source: "CANONICAL", pose_source: "TF",
    active_map_pose: pose, canonical_pose: pose, slam_pose: null,
  };
}

beforeEach(() => {
  globalThis.IS_REACT_ACT_ENVIRONMENT = true;
  container = document.createElement("div");
  document.body.appendChild(container);
  root = createRoot(container);
  window.history.replaceState({}, "", "/");
  useStore.getState().setLayout({ ...initialLayout, coordinate_system: { unit: "meter", frame: "map", yaw_unit: "radian" } }, { revision: 21 });
  useStore.setState({ ...initialState, layoutRevision: 21, twin: { ...initialState.twin, robots: { "AMR-17": runtimeRobot("AMR-17") } } });
});

afterEach(() => {
  act(() => root.unmount());
  container.remove();
  useStore.getState().setLayout(initialLayout, { revision: initialState.layoutRevision });
  useStore.setState(initialState);
});

describe("Overview robot selection", () => {
  it("opens the backend robot's exact detail route when its map marker is clicked", () => {
    act(() => root.render(<MapView2D mode="MAP" />));
    const marker = container.querySelector('[data-robot-id="AMR-17"]');
    expect(marker).not.toBeNull();
    act(() => marker?.dispatchEvent(new MouseEvent("click", { bubbles: true })));
    expect(window.location.pathname).toBe("/robots/AMR-17/control");
  });
});
