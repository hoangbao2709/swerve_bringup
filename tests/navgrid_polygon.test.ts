import { describe, expect, it } from "vitest";
import { buildNavGrid } from "../src/layout/navgrid";
import type { WarehouseLayout } from "../src/layout/types";

const polygonFloor = (id: number | string, boundary: [number, number][], holes: [number, number][][] = []) => ({ id, name: String(id), elevation: 0, boundary, holes });

function layout(floors: ReturnType<typeof polygonFloor>[], overrides: Partial<WarehouseLayout> = {}): WarehouseLayout {
  return {
    schema_version: 2, id: "navgrid-test", name: "navgrid-test", units: "m",
    coordinate_system: { unit: "meter", frame: "warehouse", yaw_unit: "radian" },
    size: { width: 12, depth: 12, height: 4 }, grid: { cell_size: 1, cols: 12, rows: 12 }, floors,
    aisles: [], navigation_tags: [], navigation_edges: [], lifts: [], zones: [], docks: [], racks: [], conveyors: [], stations: [],
    charging_stations: [], parking: [], restricted_areas: [], walkways: [], cameras: [], sensors: [], locations: [], obstacles: [], spawn: { robots: [] },
    ...overrides,
  };
}

const valueAt = (grid: ReturnType<typeof buildNavGrid>, x: number, y: number) => grid.cells[y * grid.cols + x];
const noClearance = { robot_radius: 0, safety_margin: 0 };

describe("polygon-aware navigation grid", () => {
  it("uses the floor polygon rather than its bounding box", () => {
    const floor = polygonFloor(1, [[0, 0], [6, 0], [6, 6], [0, 6]]);
    const grid = buildNavGrid(layout([floor]), 1, noClearance);
    expect(valueAt(grid, 1, 1)).toBe(0);
    expect(valueAt(grid, 7, 7)).toBe(1);
  });

  it("blocks concave L and U sections outside the polygon", () => {
    const l = polygonFloor(1, [[0, 0], [8, 0], [8, 3], [3, 3], [3, 8], [0, 8]]);
    const lGrid = buildNavGrid(layout([l]), 1, noClearance);
    expect(valueAt(lGrid, 1, 6)).toBe(0);
    expect(valueAt(lGrid, 6, 6)).toBe(1);
    const u = polygonFloor(1, [[0, 0], [8, 0], [8, 8], [6, 8], [6, 2], [2, 2], [2, 8], [0, 8]]);
    const uGrid = buildNavGrid(layout([u]), 1, noClearance);
    expect(valueAt(uGrid, 1, 5)).toBe(0);
    expect(valueAt(uGrid, 4, 5)).toBe(1);
  });

  it("blocks holes and applies hole clearance", () => {
    const floor = polygonFloor(1, [[0, 0], [10, 0], [10, 10], [0, 10]], [[[4, 3], [6, 3], [6, 7], [4, 7]]]);
    const exact = buildNavGrid(layout([floor]), 1, noClearance);
    expect(valueAt(exact, 5, 5)).toBe(1);
    expect(valueAt(exact, 3, 5)).toBe(0);
    const inflated = buildNavGrid(layout([floor]), 1, { robot_radius: 0.6, safety_margin: 0 });
    expect(valueAt(inflated, 3, 5)).toBe(1);
    expect(valueAt(inflated, 2, 5)).toBe(0);
  });

  it("inflates the outer boundary and physical racks, but not aisles", () => {
    const floor = polygonFloor(1, [[0, 0], [10, 0], [10, 10], [0, 10]]);
    const testLayout = layout([floor], {
      racks: [{ id: "R1", zone: "", position: [4, 0, 4], size: [1, 1, 1], rotation: 0, levels: 1, model: "rack", blocks_grid: true }],
      aisles: [{ id: "A1", floor_id: 1, centerline: [{ x: 0, y: 8 }, { x: 10, y: 8 }], width: 2, direction: "bidirectional" }],
    });
    const grid = buildNavGrid(testLayout, 1, { robot_radius: 0.6, safety_margin: 0 });
    expect(valueAt(grid, 0, 5)).toBe(1); // centre is too close to outer wall
    expect(valueAt(grid, 3, 4)).toBe(1); // centre is within rack clearance
    expect(valueAt(grid, 2, 4)).toBe(0);
    expect(valueAt(grid, 5, 8)).toBe(0); // aisle remains semantic, not an obstacle
  });

  it("keeps string-ID floors and their obstacles isolated", () => {
    const f1 = polygonFloor("F1", [[0, 0], [6, 0], [6, 6], [0, 6]]);
    const f2 = polygonFloor("F2", [[6, 0], [12, 0], [12, 6], [6, 6]]);
    const testLayout = layout([f1, f2], {
      racks: [{ id: "R2", zone: "", position: [8, 0, 2], size: [1, 1, 1], rotation: 0, levels: 1, model: "rack", blocks_grid: true, floor: "F2" }],
    });
    const first = buildNavGrid(testLayout, "F1", noClearance);
    const second = buildNavGrid(testLayout, "F2", noClearance);
    expect(valueAt(first, 1, 1)).toBe(0);
    expect(valueAt(first, 8, 1)).toBe(1);
    expect(valueAt(second, 8, 2)).toBe(1);
    expect(valueAt(second, 10, 1)).toBe(0);
  });
});
