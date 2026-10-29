import { describe, expect, it } from "vitest";
import { validatePhysicalObjectOverlaps } from "../src/layout/validation";

describe("physical overlap validation", () => {
  it("does not treat floor containment, aisles, or navigation tags as collisions", () => {
    expect(validatePhysicalObjectOverlaps([
      { kind: "floor", id: "F1", box: { x: 0, z: 0, w: 20, h: 20 } },
      { kind: "aisle", id: "A1", box: { x: 2, z: 5, w: 16, h: 2 } },
      { kind: "navigation-tag", id: "tag-1001", box: { x: 5, z: 5, w: 0.6, h: 0.6 } },
    ])).toEqual([]);
  });

  it("still reports overlapping physical objects", () => {
    expect(validatePhysicalObjectOverlaps([
      { kind: "rack", id: "RACK-A", box: { x: 2, z: 2, w: 3, h: 3 } },
      { kind: "rack", id: "RACK-B", box: { x: 4, z: 4, w: 3, h: 3 } },
    ])).toEqual(["RACK-A overlaps RACK-B"]);
  });
});
