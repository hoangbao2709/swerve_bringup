import { describe, expect, it } from "vitest";
import { generateNavigationEdges, validateNavigationEdges } from "../src/layout/navigation_graph";
import type { LayoutAisle, LayoutNavigationEdge, LayoutNavigationTag } from "../src/layout/types";

const aisle = (id: string, direction: LayoutAisle["direction"] = "bidirectional", floor_id: number | string = 1, centerline = [{ x: 0, y: 0 }, { x: 10, y: 0 }]): LayoutAisle => ({ id, floor_id, centerline, width: 1, direction, speed_limit: 2 });
const tag = (uuid: string, x: number, y: number, floor_id: number | string = 1, source_aisles?: string[]): LayoutNavigationTag => ({ uuid, id: uuid, tag_id: Number(uuid.replace(/\D/g, "")) || 1, floor_id, x, y, yaw: 0, placement: "auto", locked: false, source_aisles });

describe("navigation graph generation", () => {
  it("connects only consecutive tags on an aisle", () => {
    const edges = generateNavigationEdges([aisle("A")], [tag("T1", 1, 0, 1, ["A"]), tag("T2", 5, 0, 1, ["A"]), tag("T3", 9, 0, 1, ["A"])], []);
    expect(edges).toHaveLength(2);
    expect(edges.map((edge) => [edge.from_tag_uuid, edge.to_tag_uuid])).toEqual([["T1", "T2"], ["T2", "T3"]]);
    expect(edges.every((edge) => edge.cost > 0 && edge.distance > 0)).toBe(true);
  });

  it("uses polyline arc length and direction semantics", () => {
    const line = [{ x: 0, y: 0 }, { x: 3, y: 4 }, { x: 3, y: 10 }];
    const tags = [tag("T1", 0, 0, 1, ["A"]), tag("T2", 3, 4, 1, ["A"]), tag("T3", 3, 10, 1, ["A"])];
    const bidirectional = generateNavigationEdges([aisle("A", "bidirectional", 1, line)], tags);
    expect(bidirectional.map((e) => e.distance)).toEqual([5, 6]);
    expect(bidirectional.map((e) => e.cost)).toEqual([2.5, 3]);
    expect(generateNavigationEdges([aisle("A", "forward", 1, line)], tags).map((e) => [e.from_tag_uuid, e.to_tag_uuid])).toEqual([["T1", "T2"], ["T2", "T3"]]);
    expect(generateNavigationEdges([aisle("A", "reverse", 1, line)], tags).map((e) => [e.from_tag_uuid, e.to_tag_uuid])).toEqual([["T2", "T1"], ["T3", "T2"]]);
  });

  it("shares one intersection tag across crossing aisles and never crosses floors", () => {
    const a = aisle("A", "bidirectional", 1, [{ x: 0, y: 5 }, { x: 10, y: 5 }]);
    const b = aisle("B", "bidirectional", 1, [{ x: 5, y: 0 }, { x: 5, y: 10 }]);
    const tags = [tag("A1", 1, 5, 1, ["A"]), tag("X", 5, 5, 1, ["A", "B"]), tag("A2", 9, 5, 1, ["A"]), tag("B1", 5, 1, 1, ["B"]), tag("B2", 5, 9, 1, ["B"]), tag("F2", 1, 0, 2, ["F2-A"])];
    const edges = generateNavigationEdges([a, b, aisle("F2-A", "bidirectional", 2)], tags);
    expect(new Set(edges.flatMap((edge) => [edge.from_tag_uuid, edge.to_tag_uuid])).has("X")).toBe(true);
    expect(edges.filter((edge) => edge.from_tag_uuid === "F2" || edge.to_tag_uuid === "F2")).toHaveLength(0);
    expect(edges.filter((edge) => edge.from_tag_uuid === "X" || edge.to_tag_uuid === "X")).toHaveLength(4);
  });

  it("keeps stable identities and manual/locked overrides", () => {
    const tags = [tag("T1", 1, 0, 1, ["A"]), tag("T2", 5, 0, 1, ["A"])];
    const first = generateNavigationEdges([aisle("A")], tags);
    const second = generateNavigationEdges([aisle("A")], tags, first);
    expect(second[0].uuid).toBe(first[0].uuid);
    const manual: LayoutNavigationEdge = { ...first[0], uuid: "manual-edge", cost: 99, placement: "manual", locked: true };
    const preserved = generateNavigationEdges([aisle("A")], tags, [manual]);
    expect(preserved).toEqual([manual]);
  });

  it("rejects missing, self and duplicate edges while warning on orphans", () => {
    const tags = [tag("T1", 1, 0), tag("T2", 5, 0), tag("T3", 8, 0), tag("T4", 9, 4)];
    const base = generateNavigationEdges([aisle("A")], tags);
    const duplicate = { ...base[0], uuid: "duplicate" };
    const invalid = { ...base[1], uuid: "self", from_tag_uuid: "T2", to_tag_uuid: "T2" };
    const missing = { ...base[1], uuid: "missing", from_tag_uuid: "missing-tag" };
    const result = validateNavigationEdges(tags, [...base, duplicate, invalid, missing], [aisle("A")]);
    expect(result.errors.some((error) => error.includes("Duplicate"))).toBe(true);
    expect(result.errors.some((error) => error.includes("self-edge"))).toBe(true);
    expect(result.errors.some((error) => error.includes("missing tag"))).toBe(true);
    expect(result.warnings).toContain("Navigation tag T4 is orphaned");
  });
});
