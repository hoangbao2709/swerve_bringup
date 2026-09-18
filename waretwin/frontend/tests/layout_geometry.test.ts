import { describe, expect, it } from "vitest";
import { buildAisleFootprint, polygonContainedInFloor, validateAisleCenterline, validateFloorPolygon } from "../src/layout/geometry";

const p = (x: number, y: number) => ({ x, y });

describe("floor and aisle geometry", () => {
  it("accepts rectangle, L-shape and concave floor polygons", () => {
    expect(validateFloorPolygon([p(0, 0), p(20, 0), p(20, 10), p(0, 10)])).toEqual([]);
    expect(validateFloorPolygon([p(0, 0), p(20, 0), p(20, 10), p(30, 10), p(30, 30), p(0, 30)])).toEqual([]);
    expect(validateFloorPolygon([p(0, 0), p(30, 0), p(30, 30), p(20, 30), p(20, 10), p(0, 10)])).toEqual([]);
    expect(validateFloorPolygon([p(0, 0), p(10, 10), p(0, 10), p(10, 0)]).some((error) => error.includes("self-intersects"))).toBe(true);
  });

  it("validates aisle centerlines independently of polygon area", () => {
    expect(validateAisleCenterline([p(2, 5), p(18, 5)], 2)).toEqual([]);
    expect(validateAisleCenterline([p(2, 5), p(10, 5), p(18, 5)], 2)).toEqual([]);
    expect(validateAisleCenterline([p(2, 2), p(2, 10), p(10, 10)], 2)).toEqual([]);
    expect(validateAisleCenterline([p(1, 1)], 2).length).toBeGreaterThan(0);
    expect(validateAisleCenterline([p(1, 1), p(1, 1)], 2).length).toBeGreaterThan(0);
  });

  it("builds a straight/L footprint and checks width containment/holes", () => {
    const floor = [p(0, 0), p(20, 0), p(20, 20), p(0, 20)];
    const hole = [p(8, 8), p(12, 8), p(12, 12), p(8, 12)];
    expect(polygonContainedInFloor(buildAisleFootprint([p(2, 5), p(18, 5)], 2), floor)).toBe(true);
    expect(polygonContainedInFloor(buildAisleFootprint([p(2, 0.5), p(18, 0.5)], 2), floor)).toBe(false);
    expect(polygonContainedInFloor(buildAisleFootprint([p(2, 10), p(18, 10)], 2), floor, [hole])).toBe(false);
    expect(polygonContainedInFloor(buildAisleFootprint([p(2, 2), p(2, 6), p(6, 6)], 2), floor)).toBe(true);
  });
});
