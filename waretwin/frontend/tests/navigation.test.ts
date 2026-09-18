import { describe, expect, it } from "vitest";
import { applyNavigationTagPhysicalOverride, findAisleIntersections, generateAisleTags, nextAisleId, validateNavigationTagsForLayout } from "../src/layout/navigation";
import type { LayoutAisle, LayoutNavigationTag } from "../src/layout/types";

const aisle = (id: string, floor_id: number | string, centerline: { x: number; y: number }[], spacing: number): LayoutAisle => ({ id, floor_id, centerline, width: 2, direction: "bidirectional", tag_rule: { enabled: true, spacing, start_offset: 1, end_offset: 1 } });
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

  it("uses a manual or locked tag as the override for its former auto candidate", () => {
    const a = aisle("A", 1, [{ x: 1, y: 5 }, { x: 9, y: 5 }], 2);
    const first = generateAisleTags([a], []);
    const original = first.find((tag) => tag.semantic_role === "spacing")!;
    const manual = { ...original, x: original.x + 0.5, placement: "manual" as const, locked: true };
    const regenerated = generateAisleTags([a], first.map((tag) => tag.uuid === manual.uuid ? manual : tag));
    expect(regenerated.filter((tag) => tag.logical_key === original.logical_key)).toEqual([manual]);
    expect(regenerated.find((tag) => tag.uuid === manual.uuid)).toMatchObject({ tag_id: manual.tag_id, x: manual.x, placement: "manual", locked: true });
  });

  it("keeps a crossing tag stable when regenerating either selected aisle", () => {
    const a = aisle("A", 1, [{ x: 1, y: 5 }, { x: 9, y: 5 }], 2);
    const b = aisle("B", 1, [{ x: 5, y: 1 }, { x: 5, y: 9 }], 2);
    const first = generateAisleTags([a, b], []);
    const intersection = first.find((tag) => tag.semantic_role === "intersection")!;
    const afterA = generateAisleTags([a, b], first, { onlyAisleId: "A" });
    const afterB = generateAisleTags([a, b], afterA, { onlyAisleId: "B" });
    expect(afterA.filter((tag) => tag.semantic_role === "intersection")).toHaveLength(1);
    expect(afterB.find((tag) => tag.semantic_role === "intersection")).toMatchObject({ uuid: intersection.uuid, tag_id: intersection.tag_id });
  });

  it("resolves generation collisions by priority before validation", () => {
    const a = aisle("A", 1, [{ x: 0, y: 5 }, { x: 10, y: 5 }], 5);
    a.tag_rule = { enabled: true, spacing: 5, start_offset: 0.1, end_offset: 0.1 };
    const b = aisle("B", 1, [{ x: 5, y: 0 }, { x: 5, y: 10 }], 20);
    b.tag_rule = { enabled: false, spacing: 20 };
    const tags = generateAisleTags([a, b], [], { minimumSeparation: 0.25 });
    expect(tags.find((tag) => tag.semantic_role === "intersection")).toBeTruthy();
    expect(tags.some((tag) => tag.semantic_role === "spacing" && Math.abs(tag.x - 5.1) < 1e-6)).toBe(false);
  });

  it("turns physical auto-tag edits into locked manual overrides", () => {
    const updated = applyNavigationTagPhysicalOverride(base("auto", 1, 2, 2, 1001));
    expect(updated).toMatchObject({ placement: "manual", locked: true });
  });

  it("allocates aisle IDs monotonically and validates string floor IDs", () => {
    expect(nextAisleId([aisle("aisle-001", 1, [{ x: 0, y: 0 }, { x: 1, y: 0 }], 1), aisle("aisle-003", 1, [{ x: 0, y: 1 }, { x: 1, y: 1 }], 1)])).toBe("aisle-004");
    const f1 = aisle("F1-A", "F1", [{ x: 1, y: 1 }, { x: 9, y: 1 }], 2);
    const f2 = aisle("F2-A", "F2", [{ x: 1, y: 3 }, { x: 9, y: 3 }], 2);
    const tags = generateAisleTags([f1, f2], []);
    expect(new Set(tags.map((tag) => tag.floor_id))).toEqual(new Set(["F1", "F2"]));
    expect(validateNavigationTagsForLayout([
      { id: "F1", name: "F1", elevation: 0, boundary: [{ x: 0, y: 0 }, { x: 10, y: 0 }, { x: 10, y: 10 }, { x: 0, y: 10 }] },
      { id: "F2", name: "F2", elevation: 0, boundary: [{ x: 0, y: 0 }, { x: 10, y: 0 }, { x: 10, y: 10 }, { x: 0, y: 10 }] },
    ], tags)).toEqual([]);
  });

  it("rejects a selected-regeneration candidate state without committing invalid tags", () => {
    const a = aisle("A", 1, [{ x: 1, y: 5 }, { x: 9, y: 5 }], 2);
    const tags = generateAisleTags([a], []);
    const invalid = [{ ...tags[0], x: 99, placement: "manual" as const, locked: true }, ...tags.slice(1)];
    const selectedCandidateState = generateAisleTags([a], invalid, { onlyAisleId: "A" });
    expect(validateNavigationTagsForLayout([{ id: 1, name: "F1", elevation: 0, boundary: [{ x: 0, y: 0 }, { x: 10, y: 0 }, { x: 10, y: 10 }, { x: 0, y: 10 }] }], selectedCandidateState)).toContain(`Tag ${invalid[0].tag_id}: outside floor or inside hole`);
  });
});
