import { describe, expect, it } from "vitest";
import type { RobotDetailMapSnapshot } from "../src/schema/twin_state";
import { displayedNavigationMapIdentity, mapPointPreviewPayload, mapPointTarget } from "../src/components/control/navigationMapIdentity";

const activeMap: RobotDetailMapSnapshot = {
  robot_id: "R01", frame_id: "map", map_source: "LOCAL_MAP",
  active_map_id: "saved-R01-1", active_map_revision: "artifact-1",
  width: 2, height: 2, resolution: 0.05, origin: { x: 0, y: 0, yaw: 0 }, data: [0, 0, 100, 0],
};

describe("active navigation map identity", () => {
  it("binds selection to the active 2D map snapshot and its current revision", () => {
    expect(displayedNavigationMapIdentity({ active_map_id: "saved-R01-1",
      active_map_revision: "artifact-1", map_snapshot: activeMap })).toEqual({
      frame_id: "map", map_id: "saved-R01-1", map_revision: "artifact-1", source_type: "ACTIVE_MAP_POINT",
    });
  });

  it("rejects stale, mismatched, unsupported, or non-map snapshots", () => {
    expect(displayedNavigationMapIdentity({ active_map_id: "saved-R01-2",
      active_map_revision: "artifact-2", map_snapshot: activeMap })).toBeNull();
    expect(displayedNavigationMapIdentity({ active_map_id: "saved-R01-1",
      active_map_revision: "artifact-2", map_snapshot: activeMap })).toBeNull();
    expect(displayedNavigationMapIdentity({ active_map_id: "saved-R01-1",
      active_map_revision: "artifact-1", map_snapshot: { ...activeMap, frame_id: "odom" } })).toBeNull();
    expect(displayedNavigationMapIdentity({ active_map_id: "saved-R01-1",
      active_map_revision: "artifact-1", map_snapshot: { ...activeMap, map_source: undefined } })).toBeNull();
    expect(displayedNavigationMapIdentity({ active_map_id: null,
      active_map_revision: "artifact-1", map_snapshot: activeMap })).toBeNull();
  });

  it("includes active map identity and map content revision in the point preview request", () => {
    const identity = displayedNavigationMapIdentity({ active_map_id: "saved-R01-1",
      active_map_revision: "artifact-1", map_snapshot: activeMap });
    expect(identity && mapPointTarget(identity, { x: 1.25, y: -0.5, yaw: 0.3 })).toEqual({
      frame_id: "map", map_id: "saved-R01-1", map_revision: "artifact-1",
      source_type: "ACTIVE_MAP_POINT", source_map_id: "saved-R01-1", source_map_revision: "artifact-1",
      x: 1.25, y: -0.5, yaw: 0.3,
    });
    const target = identity && mapPointTarget(identity, { x: 1.25, y: -0.5, yaw: 0.3 });
    expect(target && mapPointPreviewPayload(target,
      { map_id: "saved-R01-1", map_revision: "artifact-1" }, "cells-c")).toEqual({
      frame_id: "map", active_map_id: "saved-R01-1", active_map_revision: "artifact-1",
      map_id: "saved-R01-1", map_revision: "artifact-1", map_content_revision: "cells-c",
      source_type: "ACTIVE_MAP_POINT", source_map_id: "saved-R01-1", source_map_revision: "artifact-1",
      x: 1.25, y: -0.5, yaw: 0.3,
    });
    expect(target && mapPointPreviewPayload(target,
      { map_id: "saved-R01-1", map_revision: "artifact-2" })).toBeNull();
    expect(target && mapPointPreviewPayload({ ...target, source_map_id: "CANONICAL" },
      { map_id: "saved-R01-1", map_revision: "artifact-1" })).toBeNull();
  });

  it("requires a correctly identified active SLAM map for live point selection", () => {
    const slamMap: RobotDetailMapSnapshot = { ...activeMap, map_source: "SLAM_TOOLBOX",
      mapping_session_id: "session-1", active_map_id: "SLAM-session-1",
      active_map_revision: "session-session-1" };
    const identity = displayedNavigationMapIdentity({ active_map_id: "SLAM-session-1",
      active_map_revision: "session-session-1", map_snapshot: slamMap });
    expect(identity?.source_type).toBe("ACTIVE_MAP_POINT");
    expect(displayedNavigationMapIdentity({ active_map_id: "SLAM-session-2",
      active_map_revision: "session-session-2", map_snapshot: slamMap })).toBeNull();
    expect(displayedNavigationMapIdentity({ active_map_id: "SLAM-session-1",
      active_map_revision: "session-session-1", map_snapshot: { ...slamMap, mapping_session_id: null } })).toBeNull();
  });
});
