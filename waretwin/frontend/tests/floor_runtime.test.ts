import { describe, expect, it } from "vitest";
import layoutJson from "../src/layout/warehouse_layout.json";
import { SimEngine } from "../src/simulation/engine";
import type { WarehouseLayout } from "../src/layout/types";

describe("canonical floor IDs at runtime", () => {
  it("initializes numeric runtime grids for string canonical floor IDs", () => {
    const layout = structuredClone(layoutJson) as unknown as WarehouseLayout;
    layout.floors = [
      { id: "F1", name: "Floor 1", elevation: 0, footprint: [[0, 0], [120, 0], [120, 80], [0, 80]] },
      { id: "F2", name: "Floor 2", elevation: 4, footprint: [[0, 0], [60, 0], [60, 40], [0, 40]] },
    ];
    const engine = new SimEngine(layout, { seed: 1 });
    expect(engine.grids[1]).toBeTruthy();
    expect(engine.grids[2]).toBeTruthy();
  });
});
