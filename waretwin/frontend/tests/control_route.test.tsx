// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import { useStore } from "../src/state/store";

vi.mock("../src/components/control/RobotControlDetailPage", () => ({
  RobotControlDetailPage: ({ robotId }: { robotId: string }) => <div data-testid="active-robot-control">Robot Control · {robotId}</div>,
}));

import { RobotControlPage } from "../src/components/control/RobotControlPage";

const initialState = useStore.getState();
let root: Root;
let container: HTMLDivElement;

beforeEach(() => {
  globalThis.IS_REACT_ACT_ENVIRONMENT = true;
  useStore.setState({ ...initialState, twin: { ...initialState.twin, robots: {} }, selectedRobot: null });
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

describe("single-page Robot Control route", () => {
  it("shows the industrial tool rail without an Overview entry", () => {
    act(() => root.render(<RobotControlPage />));
    const navigation = container.querySelector(".wt-sidebar-nav");
    expect(navigation?.textContent).toContain("CONTROL");
    expect(navigation?.textContent).toContain("MAPPING");
    expect(navigation?.textContent).toContain("LOCALIZATION");
    expect(navigation?.textContent).toContain("SYSTEM");
    expect(navigation?.textContent).toContain("VDA5050");
    expect(navigation?.textContent).not.toContain("Overview");
    expect(navigation?.querySelectorAll(".wt-sidebar-item")).toHaveLength(5);
    expect(container.textContent).toContain("WAITING FOR ROBOT TELEMETRY");
  });

  it("keeps the selected robot in the one control workflow", () => {
    useStore.setState({
      twin: { ...initialState.twin, robots: { R01: { id: "R01" } as never } },
      selectedRobot: "R01",
    });
    act(() => root.render(<RobotControlPage />));
    expect(container.querySelector('[data-testid="active-robot-control"]')?.textContent)
      .toBe("Robot Control · R01");
    expect(container.querySelectorAll(".wt-sidebar-item")).toHaveLength(5);
  });
});
