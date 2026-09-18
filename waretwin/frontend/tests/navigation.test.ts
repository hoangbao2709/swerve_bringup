import { describe, expect, it } from "vitest";
import { findAisleIntersections, generateAisleTags } from "../src/layout/navigation";
import type { LayoutAisle, LayoutNavigationTag } from "../src/layout/types";

const aisle = (id: string, floor_id: number, centerline: { x: number; y: number }[], spacing: number): LayoutAisle => ({ id, floor_id, centerline, width: 2, direction: "bidirectional", tag_rule: { enabled: true, spacing, start_offset: 1, end_offset: 1 } });
const base = (id: string, floor_id: number, x: number, y: number, tag_id: number): LayoutNavigationTag => ({ id, uuid: id, floor_id, x, y, tag_id, yaw: 0, placement: "auto", locked: false });

describe("aisle intersections and navigation tags", () => {
  it("finds cross, T and shared-endpoint intersections and deduplicates", () => {
    const a = aisle("A", 1, [{ x: 1, y: 5 }, { x: 9, y: 5 }], 3);
    const b = aisle("B", 1, [{ x: 5, y: 1 }, { x: 5, y: 9 }], 3);
    const c = aisle("C", 1, [{ x: 5, y: 5 }, { x: 9, y: 5 }], 3);
    expect(findAisleIntersections([a, b])).toHaveLength(1);
    expect(findAisleIntersections([a, b, c])).toHaveLength(1);
    expect(findAisleIntersections([a, aisle("D", 1, [{ x: 1, y: 1 }, { x: 9, y: 1 }], 3)])).toHaveLength(0);
    expect(findAisleIntersections([a, aisle("F2", 2, [{ x: 5, y: 1 }, { x: 5, y: 9 }], 3)])).toHaveLength(0);
  });

  it("uses per-aisle spacing and offsets, with floor-aware generated tags", () => {
    const aisles = [aisle("A", 1, [{ x: 1, y: 5 }, { x: 9, y: 5 }], 1), aisle("B", 2, [{ x: 1, y: 3 }, { x: 9, y: 3 }], 3)];
    const tags = generateAisleTags(aisles, []);
    expect(tags.filter((tag) => tag.floor_id === 1).length).toBeGreaterThan(tags.filter((tag) => tag.floor_id === 2).length);
    expect(tags.every((tag) => tag.generated_from)).toBe(true);
    const disabled = { ...aisles[0], tag_rule: { enabled: false, spacing: 1 } };
    expect(generateAisleTags([disabled], []).length).toBe(0);
  });

  it("preserves UUID/tag_id and allocates new IDs without renumbering", () => {
    const a = aisle("A", 1, [{ x: 1, y: 5 }, { x: 9, y: 5 }], 2);
    const first = generateAisleTags([a], []);
    const saved = first.map((tag) => ({ uuid: tag.uuid, tag_id: tag.tag_id }));
    const second = generateAisleTags([a], first);
    expect(second.map((tag) => tag.uuid)).toEqual(first.map((tag) => tag.uuid));
    expect(second.map((tag) => tag.tag_id)).toEqual(first.map((tag) => tag.tag_id));
    const changed = generateAisleTags([{ ...a, centerline: [{ x: 1, y: 5 }, { x: 11, y: 5 }] }], first);
    expect(saved.every((old) => changed.some((next) => next.uuid === old.uuid && next.tag_id === old.tag_id))).toBe(true);
  });

  it("preserves manual and locked tags during regeneration", () => {
    const a = aisle("A", 1, [{ x: 1, y: 5 }, { x: 9, y: 5 }], 2);
    const manual: LayoutNavigationTag = { id: "manual", uuid: "manual", tag_id: 77, floor_id: 1, x: 4, y: 4, yaw: 1, placement: "manual", locked: true };
    const locked: LayoutNavigationTag = { ...base("locked", 1, 6, 6, 78), locked: true };
    const result = generateAisleTags([a], [manual, locked]);
    expect(result.find((tag) => tag.uuid === "manual")).toEqual(manual);
    expect(result.find((tag) => tag.uuid === "locked")).toEqual(locked);
  });
});
