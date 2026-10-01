import { describe, it, expect, vi } from "vitest";
import { EMPTY_ROBOT_DETAIL } from "../src/state/store";
import { detailFramePatch, detailViewStatusPatch } from "../src/components/control/detailViewState";
import { OccupancyRasterCache } from "../src/components/control/occupancyRaster";
import type { RobotDetailMapSnapshot, RobotDetailLidar2D, RobotDetailViewStatus } from "../src/schema/twin_state";

const map: RobotDetailMapSnapshot = { robot_id: "R01", frame_id: "map", active_map_id: "CANONICAL", active_map_revision: "21", width: 2, height: 2, resolution: .05, origin: { x: 0, y: 0, yaw: 0 }, data: [0, 0, 0, 100] };
const ack: RobotDetailViewStatus = { robot_id: "R01", requested_view: "LIDAR_2D", applied_view: "LIDAR_2D", request_id: "new", view_epoch: 3, bridge_epoch: "bridge-a", state: "APPLIED" };
const frame: RobotDetailLidar2D = { robot_id: "R01", frame_id: "base_footprint", source_frame_id: "laser", point_count: 1, points: [[1, 0]], path: [], goal: null, request_id: "new", view_epoch: 3, bridge_epoch: "bridge-a" };

describe("display-only view transitions", () => {
  it("ignores old ACKs and accepts only the requested robot/view/request", () => {
    const current = { ...EMPTY_ROBOT_DETAIL, viewStatus: { ...ack, state: "REQUESTED" as const } };
    expect(detailViewStatusPatch(current, { ...ack, request_id: "old" })).toEqual({});
    expect(detailViewStatusPatch(current, { ...ack, robot_id: "R02" })).toEqual({});
    expect(detailViewStatusPatch(current, ack).viewStatus?.state).toBe("APPLIED");
  });
  it("keeps cached frames, but old epochs/requests never make a view FRESH", () => {
    const current = { ...EMPTY_ROBOT_DETAIL, lidar2d: frame, viewStatus: ack };
    expect(detailFramePatch(current, "LIDAR_2D", { ...frame, view_epoch: 2 })).toEqual({});
    expect(detailFramePatch(current, "LIDAR_2D", { ...frame, request_id: "old" }).viewStatus).toBeUndefined();
    expect(detailFramePatch(current, "LIDAR_2D", { ...frame, bridge_epoch: "old-bridge" }).viewStatus).toBeUndefined();
    expect(detailFramePatch(current, "LIDAR_2D", frame).viewStatus?.state).toBe("FRESH");
    expect(detailFramePatch(current, "LIDAR_2D", { ...frame, robot_id: "R02" })).toEqual({});
  });
  it("does not call a frame fresh before the bridge applies the request", () => {
    expect(detailFramePatch({ ...EMPTY_ROBOT_DETAIL, viewStatus: { ...ack, state: "REQUESTED" } }, "LIDAR_2D", frame).viewStatus).toBeUndefined();
  });
});

describe("bounded occupancy raster reuse", () => {
  it("deduplicates identical payloads, including concurrent remounts", async () => {
    const build = vi.fn(async () => ({ raster: true }));
    const cache = new OccupancyRasterCache(build);
    const a = cache.get(map), b = cache.get({ ...map });
    expect(a).toBe(b); expect(await a).toBe(cache.peek(map)); expect(build).toHaveBeenCalledTimes(1);
  });
  it("invalidates content, revision, geometry, and robot identity", async () => {
    const build = vi.fn(async () => ({})); const cache = new OccupancyRasterCache(build);
    for (const changed of [map, { ...map, data: [1, 0, 0, 100] }, { ...map, active_map_revision: "22" }, { ...map, resolution: .1 }, { ...map, robot_id: "R02" }]) await cache.get(changed);
    expect(build).toHaveBeenCalledTimes(5); expect(cache.size).toBe(4);
  });
  it("evicts old entries under repeated map updates", async () => {
    const cache = new OccupancyRasterCache(async () => ({}));
    for (let i = 0; i < 30; i++) await cache.get({ ...map, active_map_revision: String(i) });
    expect(cache.size).toBe(4); expect(cache.cells).toBe(16);
  });
});
