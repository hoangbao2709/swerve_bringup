import { describe, expect, it } from "vitest";
import { buildAisleFootprint, polygonContainedInFloor, rackFootprint2D, resolveNavigationEdgeEndpoints, snapCoordinate, snapPoint, validateAisleCenterline, validateFloorPolygon, zoneLabelLayout } from "../src/layout/geometry";
import { buildNavGrid } from "../src/layout/navgrid";
import { canonicalFloorId, resolveRuntimeFloorIndex, type WarehouseLayout } from "../src/layout/types";
import layoutJson from "../src/layout/warehouse_layout.json";

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

  it("uses the configured snap step and leaves coordinates untouched when snapping is off", () => {
    expect(snapCoordinate(12.347, null)).toBe(12.347);
    expect(snapCoordinate(12.347, 0.05)).toBeCloseTo(12.35);
    expect(snapPoint(p(12.347, 8.223), 0.5)).toEqual(p(12.5, 8));
  });

  it("keeps rack origin semantics consistent with the canonical exporter", () => {
    expect(rackFootprint2D([6, 0, 7.5], [6, 2.5, 4])).toEqual({ x: 6, y: 7.5, width: 6, depth: 4 });
  });

  it("uses a compact, centered zone label layout for large maps", () => {
    const label = zoneLabelLayout([p(0, 0), p(15, 0), p(15, 8), p(0, 8)], 30);
    expect(label.x).toBe(7.5);
    expect(label.fontSize).toBeLessThanOrEqual(1.3);
    expect(label.fontSize).toBeGreaterThanOrEqual(0.7);
    expect(label.y).toBeGreaterThan(0);
  });

  it("resolves canonical navigation edges by UUID without a runtime graph", () => {
    const tags = [
      { uuid: "tag-a", tag_id: 1101, x: 2, y: 3 },
      { uuid: "tag-b", tag_id: 1102, x: 8, y: 3 },
    ];
    expect(resolveNavigationEdgeEndpoints(tags, { from_tag_uuid: "tag-a", to_tag_uuid: "tag-b" })?.from.tag_id).toBe(1101);
    expect(resolveNavigationEdgeEndpoints(tags, { from_tag_id: 1101, to_tag_id: 1102 })?.to.x).toBe(8);
    expect(resolveNavigationEdgeEndpoints(tags, { from_tag_uuid: "missing", to_tag_uuid: "tag-b" })).toBeNull();
  });

  it("adapts canonical string floor IDs to numeric runtime layers without coercion", () => {
    const layout = structuredClone(layoutJson) as unknown as WarehouseLayout;
    layout.floors = [
      { id: "F1", name: "F1", elevation: 0, footprint: [[0, 0], [20, 0], [20, 20], [0, 20]] },
      { id: "F2", name: "F2", elevation: 4, footprint: [[0, 0], [10, 0], [10, 10], [0, 10]] },
    ];
    expect(resolveRuntimeFloorIndex(layout, "F1")).toBe(1);
    expect(resolveRuntimeFloorIndex(layout, "F2")).toBe(2);
    expect(canonicalFloorId(layout, 2)).toBe("F2");
    expect(buildNavGrid(layout, "F2").cells.some((cell) => cell === 0)).toBe(true);
  });
});
