// @vitest-environment jsdom
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { PosePickerMap } from "../src/components/shared/maps/LocalMapViews";
import { SHOWCASE_LIDAR_3D, SHOWCASE_MAP_SNAPSHOT, SHOWCASE_ROBOT } from "../src/mock/fixtures";

let root: Root;
let container: HTMLDivElement;

beforeEach(() => {
  globalThis.IS_REACT_ACT_ENVIRONMENT = true;
  container = document.createElement("div");
  container.style.width = "800px";
  container.style.height = "600px";
  document.body.appendChild(container);
  root = createRoot(container);
  vi.stubGlobal("ResizeObserver", class {
    observe() {}
    disconnect() {}
  });
  vi.spyOn(HTMLCanvasElement.prototype, "getContext").mockReturnValue(null);
});

afterEach(() => {
  act(() => root.unmount());
  container.remove();
  vi.restoreAllMocks();
});

describe("standalone showcase map pose", () => {
  it("uses a stable planar sample for the optional 3D view without implying live lidar", () => {
    expect(SHOWCASE_LIDAR_3D.robot_id).toBe(SHOWCASE_ROBOT.id);
    expect(SHOWCASE_LIDAR_3D.point_count).toBe(SHOWCASE_LIDAR_3D.points.length);
    expect(SHOWCASE_LIDAR_3D.points.every((point) => point[2] === 0)).toBe(true);
    expect(SHOWCASE_LIDAR_3D.accumulated).toBe(false);
    expect(SHOWCASE_ROBOT.status).toBe("OFFLINE");
  });

  it("renders the explicit local sample robot pose while keeping robot status offline", async () => {
    await act(async () => {
      root.render(<PosePickerMap
        map={SHOWCASE_MAP_SNAPSHOT}
        robot={SHOWCASE_ROBOT}
        fixtureRobotPose={SHOWCASE_ROBOT.active_map_pose ?? undefined}
        pose={{ x: 30, y: 17, yaw: 0 }}
        active={false}
        onPick={() => undefined}
        ariaLabel="Showcase map sample"
      />);
    });

    const canvas = container.querySelector("canvas");
    expect(SHOWCASE_ROBOT.status).toBe("OFFLINE");
    expect(canvas?.getAttribute("data-robot-layer")).toBe("visible");
    expect(canvas?.getAttribute("data-pose-source")).toBe("LOCAL_SAMPLE");
    expect(canvas?.getAttribute("data-render-x")).toBe("30");
    expect(canvas?.getAttribute("data-render-y")).toBe("17");
  });
});
