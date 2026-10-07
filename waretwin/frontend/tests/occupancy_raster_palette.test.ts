import { describe, expect, it } from "vitest";
import { occupancyCellColor } from "../src/components/control/occupancyRaster";

describe("conventional occupancy-grid palette", () => {
  it("renders unknown cells medium gray", () => {
    expect(occupancyCellColor(-1)).toEqual([145, 145, 145, 255]);
  });

  it("renders free cells near white", () => {
    expect(occupancyCellColor(0)).toEqual([245, 245, 245, 255]);
    expect(occupancyCellColor(65)).toEqual([245, 245, 245, 255]);
  });

  it("renders occupied cells near black", () => {
    expect(occupancyCellColor(66)).toEqual([20, 20, 20, 255]);
    expect(occupancyCellColor(100)).toEqual([20, 20, 20, 255]);
  });
});
