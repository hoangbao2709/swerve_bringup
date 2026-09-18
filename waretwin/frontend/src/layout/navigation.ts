import { distance, pointInPolygon, type Point } from "./geometry";
import type { LayoutAisle, LayoutNavigationTag } from "./types";

const newUuid = () => `tag-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`;
function addCandidate(list: Point[], point: Point, minimum: number) { if (!list.some((existing) => distance(existing, point) < minimum)) list.push(point); }

export function generateAisleTags(aisles: LayoutAisle[], existing: LayoutNavigationTag[], spacing = 2, minimumSeparation = 0.4): LayoutNavigationTag[] {
  const generated: Point[] = [];
  for (const aisle of aisles) {
    const points = aisle.centerline;
    if (points.length < 2) continue;
    addCandidate(generated, points[0], minimumSeparation);
    points.slice(1, -1).forEach((point) => addCandidate(generated, point, minimumSeparation));
    addCandidate(generated, points[points.length - 1], minimumSeparation);
    for (let i = 1; i < points.length; i++) {
      const a = points[i - 1], b = points[i], length = distance(a, b), dx = (b.x - a.x) / (length || 1), dy = (b.y - a.y) / (length || 1);
      for (let cursor = spacing; cursor < length; cursor += spacing) addCandidate(generated, { x: a.x + dx * cursor, y: a.y + dy * cursor }, minimumSeparation);
    }
  }
  const previous = existing.filter((tag) => tag.placement === "manual" || tag.locked);
  return [...previous, ...generated.map((point, index) => ({ uuid: newUuid(), tag_id: 1000 + index, floor_id: aisles[0]?.floor_id, x: point.x, y: point.y, z: 0, yaw: 0, placement: "auto" as const, locked: false, generated_from: aisles.find((aisle) => aisle.centerline.some((p) => distance(p, point) < 1))?.id }))];
}

export function validateTags(tags: LayoutNavigationTag[], boundary: Point[], holes: Point[][] = [], minimumSeparation = 0.25): string[] {
  const errors: string[] = [], ids = new Set<number>();
  tags.forEach((tag) => { if (!Number.isInteger(tag.tag_id) || tag.tag_id < 0) errors.push(`Tag ${tag.uuid}: invalid tag ID`); if (ids.has(tag.tag_id)) errors.push(`Duplicate tag ID: ${tag.tag_id}`); ids.add(tag.tag_id); if (!pointInPolygon({ x: tag.x, y: tag.y }, boundary) || holes.some((hole) => pointInPolygon({ x: tag.x, y: tag.y }, hole))) errors.push(`Tag ${tag.tag_id}: outside floor or inside hole`); });
  for (let i = 0; i < tags.length; i++) for (let j = i + 1; j < tags.length; j++) if (distance(tags[i], tags[j]) < minimumSeparation) errors.push(`Tags ${tags[i].tag_id} and ${tags[j].tag_id} are too close`);
  return errors;
}
