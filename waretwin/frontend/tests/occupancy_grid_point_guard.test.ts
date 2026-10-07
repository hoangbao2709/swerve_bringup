import { describe, expect, it } from "vitest";
import { isFreeOccupancyPoint } from "../src/components/control/occupancyGrid";

describe("free-cell point selection guard", () => {
  const map = { width: 2, height: 2, resolution: 1, origin: { x: 0, y: 0, yaw: 0 } };
  const cells = Int8Array.from([-1, 0, 100, 65]);

  it("accepts known free cells and rejects unknown, occupied, and out-of-map points", () => {
    expect(isFreeOccupancyPoint(map, cells, { x: 1.5, y: 0.5 })).toBe(true);
    expect(isFreeOccupancyPoint(map, cells, { x: 1.5, y: 1.5 })).toBe(true);
    expect(isFreeOccupancyPoint(map, cells, { x: 0.5, y: 0.5 })).toBe(false);
    expect(isFreeOccupancyPoint(map, cells, { x: 0.5, y: 1.5 })).toBe(false);
    expect(isFreeOccupancyPoint(map, cells, { x: 2, y: 1 })).toBe(false);
  });

  it("uses the map origin yaw when locating the occupancy cell", () => {
    const rotated = { ...map, origin: { x: 10, y: 20, yaw: Math.PI / 2 } };
    expect(isFreeOccupancyPoint(rotated, cells, { x: 9.5, y: 21.5 })).toBe(true);
    expect(isFreeOccupancyPoint(rotated, cells, { x: 8.5, y: 20.5 })).toBe(false);
  });
});
