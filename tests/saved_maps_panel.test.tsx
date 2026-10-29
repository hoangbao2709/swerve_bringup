// @vitest-environment jsdom
import { act, useState } from "react";
import { createRoot, type Root } from "react-dom/client";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import type { LocalRobotMap } from "../src/services/api";
import { SavedMapsPanel } from "../src/pages/mapping/components/SavedMapsPanel";

let root: Root;
let container: HTMLDivElement;
const maps: LocalRobotMap[] = [
  { id: "saved-a", name: "north-floor", robot_id: "R01", created_at: "2026-10-01", resolution: 0.05,
    origin: [0, 0, 0], revision: "rev-a", frame_id: "map", width: 40, height: 20 },
  { id: "saved-b", name: "south-floor", robot_id: "R01", created_at: "2026-10-02", resolution: 0.1,
    origin: [1, 2, 0], revision: "rev-b", frame_id: "map", width: 80, height: 60 },
];
const resumeProps = {
  resumeDisabledReason: "SAVED SLAM POSE GRAPH UNAVAILABLE",
  resumeBusy: false,
  resumeState: "",
  resumeError: "",
  resumeNotice: "",
  onResume: vi.fn(),
};

beforeEach(() => {
  globalThis.IS_REACT_ACT_ENVIRONMENT = true;
  container = document.createElement("div");
  document.body.appendChild(container);
  root = createRoot(container);
});

afterEach(() => {
  act(() => root.unmount());
  container.remove();
});

describe("saved map registry selection", () => {
  it("filters by map identity or revision and only selects without applying", () => {
    const onSelect = vi.fn();
    function Harness() {
      const [selected, setSelected] = useState("saved-a");
      return <SavedMapsPanel {...resumeProps} maps={maps} selected={selected} activeMapId="saved-a" isStatic={false} onSelect={(id) => {
        onSelect(id);
        setSelected(id);
      }} />;
    }
    act(() => root.render(<Harness />));
    expect(container.querySelectorAll('[role="option"]')).toHaveLength(2);
    const search = container.querySelector<HTMLInputElement>('input[aria-label="Filter saved maps"]');
    expect(search).not.toBeNull();
    if (!search) return;
    const setter = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value")?.set;
    act(() => {
      setter?.call(search, "rev-b");
      search.dispatchEvent(new Event("input", { bubbles: true }));
      search.dispatchEvent(new Event("change", { bubbles: true }));
    });
    expect(container.querySelectorAll('[role="option"]')).toHaveLength(1);
    expect(container.querySelector('[role="option"]')?.textContent).toContain("south-floor");
    act(() => container.querySelector<HTMLButtonElement>('[role="option"]')?.click());
    expect(onSelect).toHaveBeenCalledWith("saved-b");
    expect(container.querySelector('[role="option"]')?.getAttribute("aria-selected")).toBe("true");
    expect(container.querySelector('[role="option"]')?.textContent).toContain("south-floor");
    expect(container.querySelector('[role="option"] .mapping-map-status')?.textContent).toBe("STORED");
  });

  it("offers confirmed SLAM restore only when the selected saved map has a pose graph", () => {
    const onResume = vi.fn();
    const mapWithSession: LocalRobotMap = {
      ...maps[0], slam_session_state: { status: "AVAILABLE", engine: "SLAM_TOOLBOX", artifact_id: "session-a" },
    };
    act(() => root.render(<SavedMapsPanel {...resumeProps} resumeDisabledReason="" maps={[mapWithSession]} selected={mapWithSession.id}
      activeMapId={null} isStatic={false} onSelect={vi.fn()} onResume={onResume} />));
    const resume = Array.from(container.querySelectorAll("button")).find((button) => button.textContent?.trim() === "RESUME SAVED SLAM SESSION");
    expect(resume?.disabled).toBe(false);
    act(() => resume?.click());
    expect(container.querySelector('[role="dialog"][aria-labelledby="confirm-slam-resume-title"]')).not.toBeNull();
    expect(onResume).not.toHaveBeenCalled();
    act(() => {
      Array.from(container.querySelectorAll("button")).find((button) => button.textContent?.trim() === "RESTORE SLAM SESSION")?.click();
    });
    expect(onResume).toHaveBeenCalledTimes(1);
    expect(container.querySelector('[role="dialog"][aria-labelledby="confirm-slam-resume-title"]')).toBeNull();
  });

  it("keeps session restore disabled when pose-graph artifacts are unavailable", () => {
    act(() => root.render(<SavedMapsPanel {...resumeProps} maps={maps} selected="saved-a" activeMapId={null} isStatic={false} onSelect={vi.fn()} />));
    const resume = Array.from(container.querySelectorAll("button")).find((button) => button.textContent?.trim() === "RESUME SAVED SLAM SESSION");
    expect(resume?.disabled).toBe(true);
    expect(container.textContent).toContain("SAVED SLAM POSE GRAPH UNAVAILABLE");
  });
});
