import { describe, expect, it } from "vitest";
import { MapSnapshotOrderGuard } from "../src/layout/mapSnapshotOrder";
import type { RobotDetailMapSnapshot } from "../src/schema/twin_state";

function mapSnapshot(mapVersion: number, stamp: number, mapContentRevision: string): RobotDetailMapSnapshot {
  return {
    robot_id: "R01", frame_id: "map", map_source: "SLAM_TOOLBOX",
    mapping_session_id: "session-1", active_map_id: "SLAM-session-1",
    active_map_revision: "session-session-1", map_version: mapVersion, stamp,
    map_content_revision: mapContentRevision, width: 10, height: 10, resolution: 0.05,
    origin: { x: -1, y: -1, yaw: 0 }, data: Array(100).fill(0),
  };
}

describe("SLAM MAP_SNAPSHOT ordering", () => {
  it("rejects an older version from the same mapping session", () => {
    const guard = new MapSnapshotOrderGuard();
    const latest = mapSnapshot(102, 1818.2, "revision-102");
    expect(guard.accept(latest, null, "session-1").accepted).toBe(true);
    expect(guard.accept(mapSnapshot(101, 1818.3, "late-revision-101"), latest, "session-1"))
      .toEqual({ accepted: false, reason: "older_map_version" });
  });

  it("uses source stamp as a secondary version order without blocking new content", () => {
    const guard = new MapSnapshotOrderGuard();
    const first = mapSnapshot(102, 1818.2, "revision-102a");
    const newerContent = mapSnapshot(102, 1818.3, "revision-102b");
    guard.accept(first, null, "session-1");
    expect(guard.accept(mapSnapshot(102, 1818.1, "late-revision-102"), first, "session-1"))
      .toEqual({ accepted: false, reason: "older_source_stamp" });
    expect(guard.accept(newerContent, first, "session-1").accepted).toBe(true);
  });

  it("fails closed when a snapshot does not match the current mapping session", () => {
    const guard = new MapSnapshotOrderGuard();
    expect(guard.accept(mapSnapshot(1, 1, "old-session"), null, "session-current"))
      .toEqual({ accepted: false, reason: "mapping_session_mismatch" });
  });
});
