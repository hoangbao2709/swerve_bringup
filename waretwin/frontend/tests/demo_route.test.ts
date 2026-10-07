import { describe, expect, it } from "vitest";
import { resolveDemoRoute } from "../src/components/control/demoRoute";
import type { RobotDetailPath, RobotDetailPathPreview } from "../src/schema/twin_state";

const path: RobotDetailPath = {
  robot_id: "R01", frame_id: "map", map_source: "SLAM_TOOLBOX",
  active_map_id: "SLAM-session-a", active_map_revision: "session-a",
  map_content_revision: "content-a", navigation_map_id: "NAV-session-a",
  navigation_map_revision: "registered-a", registration_revision: 7,
  points: [[0, 0], [1, 1], [2, 1]],
};
const preview: RobotDetailPathPreview = {
  robot_id: "R01", request_id: "preview-1", status: "VALID", path: [[0, 0], [1, 1]],
  goal: { x: 1, y: 1, yaw: 0 }, path_length_m: 1.4,
  active_map_id: "SLAM-session-a", active_map_revision: "session-a", frame_id: "map",
};
const common = {
  activeMapId: "SLAM-session-a", activeMapRevision: "session-a", mapSource: "SLAM_TOOLBOX",
  mapContentRevision: "content-a",
  approvedPreview: preview, globalPath: path,
};

describe("demo route source", () => {
  it("uses the approved path preview before SEND", () => {
    expect(resolveDemoRoute({ ...common, navigationStatus: "IDLE" })).toEqual({ kind: "PREVIEW", points: preview.path });
  });

  it("uses the actual map-matched Nav2 global path while navigating", () => {
    expect(resolveDemoRoute({ ...common, navigationStatus: "ACTIVE", registrationRevision: 7 }))
      .toEqual({ kind: "ACTIVE_NAV", points: path.points });
  });

  it("hides a stale or differently registered active route without falling back to preview", () => {
    expect(resolveDemoRoute({ ...common, navigationStatus: "ACTIVE", registrationRevision: 8 })).toBeNull();
    expect(resolveDemoRoute({ ...common, navigationStatus: "ACTIVE", activeMapRevision: "session-b", registrationRevision: 7 })).toBeNull();
    expect(resolveDemoRoute({ ...common, navigationStatus: "ACTIVE", mapContentRevision: "content-b", registrationRevision: 7 })).toBeNull();
  });

  it("hides non-approved previews", () => {
    expect(resolveDemoRoute({ ...common, navigationStatus: "IDLE", approvedPreview: { ...preview, status: "NO_PATH" } })).toBeNull();
  });
});
