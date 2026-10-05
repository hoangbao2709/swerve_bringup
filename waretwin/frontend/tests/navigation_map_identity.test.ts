import { describe, expect, it } from "vitest";
import type { RobotDetailMapSnapshot } from "../src/schema/twin_state";
import { displayedNavigationMapIdentity, mapPointPreviewPayload, mapPointTarget } from "../src/components/control/navigationMapIdentity";

const localMap: RobotDetailMapSnapshot = {
  robot_id: "R01", frame_id: "map", map_source: "LOCAL_MAP",
  active_map_id: "saved-R01-1", active_map_revision: "artifact-1",
  width: 2, height: 2, resolution: 0.05, origin: { x: 0, y: 0, yaw: 0 }, data: [0, 0, 100, 0],
};

describe("displayed navigation map identity", () => {
  it("binds Global map points to the canonical warehouse revision", () => {
    expect(displayedNavigationMapIdentity({ view: "GLOBAL", active_map_id: "CANONICAL",
      active_map_revision: "21", canonical_revision: 21, map_snapshot: null })).toEqual({
      frame_id: "map", map_id: "CANONICAL", map_revision: "21", source_type: "CANONICAL_MAP_POINT",
    });
  });

  it("allows local map points only when the rendered snapshot is the active local map", () => {
    expect(displayedNavigationMapIdentity({ view: "LIDAR_2D", active_map_id: "saved-R01-1",
      active_map_revision: "artifact-1", canonical_revision: 21, map_snapshot: localMap })).toEqual({
      frame_id: "map", map_id: "saved-R01-1", map_revision: "artifact-1", source_type: "ACTIVE_MAP_POINT",
    });
    expect(displayedNavigationMapIdentity({ view: "GLOBAL", active_map_id: "saved-R01-1",
      active_map_revision: "artifact-1", canonical_revision: 21, map_snapshot: localMap })).toEqual({
      frame_id: "map", map_id: "CANONICAL", map_revision: "21", source_type: "CANONICAL_MAP_POINT",
    });
    expect(displayedNavigationMapIdentity({ view: "LIDAR_2D", active_map_id: "saved-R01-2",
      active_map_revision: "artifact-2", canonical_revision: 21, map_snapshot: localMap })).toBeNull();
    expect(displayedNavigationMapIdentity({ view: "LIDAR_3D", active_map_id: "saved-R01-1",
      active_map_revision: "artifact-1", canonical_revision: 21, map_snapshot: localMap })).toBeNull();
  });

  it("keeps frame and map identity on the selected point sent to preview", () => {
    const identity = displayedNavigationMapIdentity({ view: "LIDAR_2D", active_map_id: "saved-R01-1",
      active_map_revision: "artifact-1", canonical_revision: 21, map_snapshot: localMap });
    expect(identity && mapPointTarget(identity, { x: 1.25, y: -0.5, yaw: 0.3 })).toEqual({
      frame_id: "map", map_id: "saved-R01-1", map_revision: "artifact-1",
      source_type: "ACTIVE_MAP_POINT", source_map_id: "saved-R01-1", source_map_revision: "artifact-1",
      x: 1.25, y: -0.5, yaw: 0.3,
    });
  });

  it("binds a GLOBAL click to canonical coordinates even while active navigation is on live SLAM", () => {
    const identity = displayedNavigationMapIdentity({ view: "GLOBAL", active_map_id: "SLAM-session-1",
      active_map_revision: "session-session-1", canonical_revision: 21, map_snapshot: null });
    expect(identity?.source_type).toBe("CANONICAL_MAP_POINT");
    expect(identity && mapPointTarget(identity, { x: 10, y: 5, yaw: 0 })).toMatchObject({
      map_id: "CANONICAL", map_revision: "21", source_map_id: "CANONICAL", source_map_revision: "21",
    });
    const target = identity && mapPointTarget(identity, { x: 10, y: 5, yaw: 0 });
    expect(target && mapPointPreviewPayload(target,
      { map_id: "SLAM-session-1", map_revision: "session-session-1" }, "cells-c")).toEqual({
      frame_id: "map", active_map_id: "SLAM-session-1", active_map_revision: "session-session-1",
      map_id: "SLAM-session-1", map_revision: "session-session-1", map_content_revision: "cells-c",
      source_type: "CANONICAL_MAP_POINT", source_map_id: "CANONICAL", source_map_revision: "21",
      x: 10, y: 5, yaw: 0,
    });
  });

  it("keeps active SLAM 2D clicks direct and independent of canonical registration", () => {
    const identity = displayedNavigationMapIdentity({ view: "LIDAR_2D", active_map_id: "SLAM-session-1",
      active_map_revision: "session-session-1", canonical_revision: 21,
      map_snapshot: { ...localMap, map_source: "SLAM_TOOLBOX", mapping_session_id: "session-1",
        active_map_id: "SLAM-session-1", active_map_revision: "session-session-1" } });
    const target = identity && mapPointTarget(identity, { x: 1.25, y: -0.5, yaw: 0.3 });
    expect(target && mapPointPreviewPayload(target,
      { map_id: "SLAM-session-1", map_revision: "session-session-1" })).toMatchObject({
      source_type: "ACTIVE_MAP_POINT", source_map_id: "SLAM-session-1",
      active_map_id: "SLAM-session-1", x: 1.25, y: -0.5, yaw: 0.3,
    });
  });
});
