// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import type { RobotState, TwinState } from "../src/schema/twin_state";
import type { WarehouseLayout } from "../src/layout/types";
import { useStore } from "../src/state/store";
import { overviewRobots, runtimeRobotOnline } from "../src/components/overview/overviewRuntime";

vi.mock("../src/services/auth", () => ({ logout: vi.fn() }));
vi.mock("../src/components/overview/OverviewWarehouseView", () => ({
  OverviewWarehouseView: ({ robots, onOpenRobot }: { robots: Array<{ id: string }>; onOpenRobot: (id: string) => void }) =>
    <div data-testid="warehouse-renderer" data-robot-ids={robots.map(({ id }) => id).join(",")}>
      {robots.map(({ id }) => <button key={id} type="button" onClick={() => onOpenRobot(id)}>{id}</button>)}
    </div>,
}));

import { OverviewPage } from "../src/components/overview/OverviewPage";

const layout = {
  id: "active-layout", name: "Active Warehouse", units: "m", schema_version: 1,
  size: { width: 20, depth: 12, height: 4 }, grid: { cell_size: 1, cols: 20, rows: 12 },
  floors: [{ id: 1, name: "Ground", elevation: 0 }], lifts: [], zones: [], docks: [], racks: [], conveyors: [],
  stations: [], charging_stations: [], parking: [], restricted_areas: [], walkways: [], cameras: [], sensors: [],
  locations: [], obstacles: [], spawn: { robots: [] },
} as WarehouseLayout;

function robot(id: string, telemetryAt: string | null): RobotState {
  return {
    id, model: "runtime", floor: 1, lift_id: null, lift_stage: null, position: [0, 0, 0], heading: 0,
    velocity: 0, max_speed: 1, battery: null, battery_reported: false, status: "OFFLINE", fsm: "OFFLINE",
    health: 0, current_task_id: null, destination: null, path: [], path_index: 0,
    load: { current: 0, capacity: 1 }, zone: null, eta_s: null, fsm_since_tick: 0,
    stats: { distance_m: 0, tasks_completed: 0, energy_wh: 0, busy_ticks: 0, wait_ticks: 0 },
    perception: {} as RobotState["perception"], last_telemetry_at: telemetryAt,
  };
}

function twin(robots: Record<string, RobotState>): TwinState {
  return { robots } as TwinState;
}

let root: Root | null = null;
let container: HTMLDivElement | null = null;

afterEach(() => {
  if (root) act(() => root?.unmount());
  root = null;
  container?.remove();
  container = null;
  useStore.setState({ authUser: null, twin: null, layout: null, mapSync: {
    publishedRevision: null, publishedVersion: 0, rosRevision: null, gazeboRevision: null,
    nav2Revision: null, tagMapRevision: null, tfStatus: false, status: "UNKNOWN", error: null, robots: {},
  } });
  vi.restoreAllMocks();
});

describe("Overview runtime data", () => {
  it("includes only robot identities actually reported by backend telemetry", () => {
    const state = twin({ "seed-only": robot("seed-only", null), "AGV-west/3": robot("AGV-west/3", new Date().toISOString()) });
    const rows = overviewRobots(state, layout, 12);
    expect(rows.map(({ id }) => id)).toEqual(["AGV-west/3"]);
    expect(rows[0].pose).toBeNull();
  });

  it("does not claim an online robot unless backend ROS bridge state confirms its ID", () => {
    expect(runtimeRobotOnline("AGV-17", "CONNECTED", true, ["AGV-17"])).toBe(true);
    expect(runtimeRobotOnline("AGV-17", "CONNECTED", true, [])).toBe(false);
    expect(runtimeRobotOnline("AGV-17", "DISCONNECTED", true, ["AGV-17"])).toBe(false);
  });

  it("renders backend robot identity and opens that exact dynamic detail route", async () => {
    globalThis.IS_REACT_ACT_ENVIRONMENT = true;
    useStore.setState({ authUser: { id: 9, username: "operator", email: "", role: "user" },
      twin: twin({ "AGV-west/3": robot("AGV-west/3", new Date().toISOString()) }), layout,
      mapSync: { ...useStore.getState().mapSync, publishedRevision: 12 } });
    container = document.createElement("div");
    document.body.appendChild(container);
    root = createRoot(container);
    await act(async () => { root?.render(<OverviewPage />); });
    expect(container.querySelector('[data-testid="warehouse-renderer"]')?.getAttribute("data-robot-ids")).toBe("AGV-west/3");
    expect(container.textContent).toContain("operator");
    await act(async () => {
      [...(container?.querySelectorAll<HTMLButtonElement>("button") ?? [])]
        .find((button) => button.textContent === "AGV-west/3")?.click();
    });
    expect(window.location.pathname).toBe("/robots/AGV-west%2F3/control");
  });
});
