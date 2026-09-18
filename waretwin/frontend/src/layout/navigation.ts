import { distance, pointInPolygon, type Point } from "./geometry";
import type { LayoutAisle, LayoutNavigationTag } from "./types";

export type AisleIntersection = { point: Point; aisleIds: string[]; floorId: string | number; segmentA: number; segmentB: number };
const EPS = 1e-9;

function segmentIntersection(a: Point, b: Point, c: Point, d: Point, tolerance: number): Point | null {
  const r = { x: b.x - a.x, y: b.y - a.y }, s = { x: d.x - c.x, y: d.y - c.y };
  const cross = r.x * s.y - r.y * s.x, q = { x: c.x - a.x, y: c.y - a.y };
  if (Math.abs(cross) <= EPS) {
    for (const p of [c, d, a, b]) if (distance(a, p) + distance(p, b) <= distance(a, b) + tolerance && distance(c, p) + distance(p, d) <= distance(c, d) + tolerance) return p;
    return null;
  }
  const t = (q.x * s.y - q.y * s.x) / cross, u = (q.x * r.y - q.y * r.x) / cross;
  if (t < -tolerance || t > 1 + tolerance || u < -tolerance || u > 1 + tolerance) return null;
  return { x: a.x + t * r.x, y: a.y + t * r.y };
}

export function findAisleIntersections(aisles: LayoutAisle[], tolerance = 0.01): AisleIntersection[] {
  const result: AisleIntersection[] = [];
  const add = (candidate: AisleIntersection) => {
    const existing = result.find((item) => String(item.floorId) === String(candidate.floorId) && distance(item.point, candidate.point) <= tolerance);
    if (existing) existing.aisleIds = [...new Set([...existing.aisleIds, ...candidate.aisleIds])].sort();
    else result.push(candidate);
  };
  for (let i = 0; i < aisles.length; i += 1) for (let j = i + 1; j < aisles.length; j += 1) {
    const a = aisles[i], b = aisles[j];
    if (String(a.floor_id ?? 1) !== String(b.floor_id ?? 1)) continue;
    for (let sa = 1; sa < a.centerline.length; sa += 1) for (let sb = 1; sb < b.centerline.length; sb += 1) {
      const point = segmentIntersection(a.centerline[sa - 1], a.centerline[sa], b.centerline[sb - 1], b.centerline[sb], tolerance);
      if (point) add({ point, aisleIds: [String(a.id), String(b.id)].sort(), floorId: a.floor_id ?? 1, segmentA: sa - 1, segmentB: sb - 1 });
    }
  }
  return result;
}

type Candidate = { point: Point; floorId: string | number; sourceAisles: string[]; generatedFrom: string; role: "intersection" | "turn" | "start" | "end" | "spacing"; distanceAlong: number; yaw: number };
function addCandidate(candidates: Candidate[], candidate: Candidate, tolerance: number) {
  const existing = candidates.find((item) => String(item.floorId) === String(candidate.floorId) && distance(item.point, candidate.point) <= tolerance);
  if (!existing) candidates.push(candidate);
  else {
    existing.sourceAisles = [...new Set([...existing.sourceAisles, ...candidate.sourceAisles])].sort();
    if (candidate.role === "intersection") { existing.role = candidate.role; existing.generatedFrom = candidate.generatedFrom; }
  }
}
function candidateKey(candidate: Candidate): string { return `${candidate.floorId}|${candidate.sourceAisles.slice().sort().join(",")}|${candidate.role}|${Math.round(candidate.point.x * 1000)}:${Math.round(candidate.point.y * 1000)}`; }
function nextTagId(existing: LayoutNavigationTag[]): number { const used = new Set(existing.map((tag) => Number(tag.tag_id)).filter(Number.isInteger)); let id = Math.max(1000, ...Array.from(used, (value) => value)) + 1; while (used.has(id)) id += 1; return id; }

function aisleCandidates(aisles: LayoutAisle[], tolerance: number): Candidate[] {
  const candidates: Candidate[] = [];
  for (const aisle of aisles) {
    const points = aisle.centerline; if (points.length < 2) continue;
    const floorId = aisle.floor_id ?? 1, source = String(aisle.id), rule = { enabled: true, spacing: 2, start_offset: 0, end_offset: 0, ...(aisle.tag_rule ?? {}) };
    if (!rule.enabled) continue;
    const prefix: number[] = [0]; for (let i = 1; i < points.length; i += 1) prefix.push(prefix[i - 1] + distance(points[i - 1], points[i]));
    const total = prefix[prefix.length - 1];
    const yawAt = (index: number) => Math.atan2(points[Math.min(index + 1, points.length - 1)].y - points[Math.max(0, index - 1)].y, points[Math.min(index + 1, points.length - 1)].x - points[Math.max(0, index - 1)].x);
    const addAt = (along: number, role: Candidate["role"]) => {
      const bounded = Math.max(0, Math.min(total, along)); let segment = 1; while (segment < prefix.length - 1 && prefix[segment] < bounded) segment += 1;
      const a = points[segment - 1], b = points[segment], length = distance(a, b) || 1, ratio = (bounded - prefix[segment - 1]) / length;
      addCandidate(candidates, { point: { x: a.x + (b.x - a.x) * ratio, y: a.y + (b.y - a.y) * ratio }, floorId, sourceAisles: [source], generatedFrom: source, role, distanceAlong: bounded, yaw: yawAt(segment - 1) }, tolerance);
    };
    addAt(rule.start_offset, "start");
    points.slice(1, -1).forEach((point, index) => addCandidate(candidates, { point, floorId, sourceAisles: [source], generatedFrom: source, role: "turn", distanceAlong: prefix[index + 1], yaw: yawAt(index + 1) }, tolerance));
    if (rule.spacing > 0) for (let along = rule.start_offset + rule.spacing; along < total - rule.end_offset - tolerance; along += rule.spacing) addAt(along, "spacing");
    if (total - rule.end_offset > rule.start_offset + tolerance) addAt(total - rule.end_offset, "end");
  }
  for (const intersection of findAisleIntersections(aisles, tolerance)) addCandidate(candidates, { point: intersection.point, floorId: intersection.floorId, sourceAisles: intersection.aisleIds, generatedFrom: intersection.aisleIds.join(","), role: "intersection", distanceAlong: 0, yaw: 0 }, tolerance);
  const priority: Record<Candidate["role"], number> = { intersection: 0, turn: 1, start: 2, end: 3, spacing: 4 };
  return candidates.sort((a, b) => priority[a.role] - priority[b.role] || candidateKey(a).localeCompare(candidateKey(b)));
}

/** Generate auto tags without time/random identity and preserve matched IDs. */
export function generateAisleTags(aisles: LayoutAisle[], existing: LayoutNavigationTag[], options: { tolerance?: number; onlyAisleId?: string } | number = {}): LayoutNavigationTag[] {
  const tolerance = typeof options === "number" ? options : options.tolerance ?? 0.01, onlyAisleId = typeof options === "number" ? undefined : options.onlyAisleId;
  const targetAisles = onlyAisleId ? aisles.filter((aisle) => String(aisle.id) === String(onlyAisleId)) : aisles;
  const preserved = existing.filter((tag) => tag.placement === "manual" || tag.locked || (onlyAisleId && tag.placement === "auto" && !String(tag.generated_from ?? "").split(",").includes(String(onlyAisleId))));
  const available = existing.filter((tag) => tag.placement === "auto" && !tag.locked && (!onlyAisleId || String(tag.generated_from ?? "").split(",").includes(String(onlyAisleId))));
  const candidates = aisleCandidates(targetAisles, tolerance), result = [...preserved], usedIds = new Set(preserved.map((tag) => tag.tag_id));
  for (const candidate of candidates) {
    const key = candidateKey(candidate), matched = available.find((tag) => !usedIds.has(tag.tag_id) && tag.logical_key === key) ?? available.find((tag) => !usedIds.has(tag.tag_id) && String(tag.floor_id) === String(candidate.floorId) && distance(tag, candidate.point) <= tolerance && String(tag.generated_from ?? "").split(",").some((id) => candidate.sourceAisles.includes(id)));
    const identity = matched?.uuid ?? `tag-${key.replace(/[^a-zA-Z0-9_-]/g, "_")}`;
    const tag: LayoutNavigationTag = { ...(matched ?? {}), uuid: identity, id: matched?.id ?? identity, tag_id: matched?.tag_id ?? nextTagId([...existing, ...result]), floor_id: candidate.floorId, x: candidate.point.x, y: candidate.point.y, z: matched?.z ?? 0, yaw: matched?.yaw ?? candidate.yaw, placement: "auto", locked: false, generated_from: candidate.generatedFrom, source_aisles: candidate.sourceAisles, semantic_role: candidate.role, logical_key: key };
    usedIds.add(tag.tag_id); result.push(tag);
  }
  return result;
}

export function validateTags(tags: LayoutNavigationTag[], boundary: Point[], holes: Point[][] = [], minimumSeparation = 0.25): string[] {
  const errors: string[] = [], ids = new Set<number>();
  tags.forEach((tag) => { if (!Number.isInteger(tag.tag_id) || tag.tag_id < 0) errors.push(`Tag ${tag.uuid}: invalid tag ID`); if (ids.has(tag.tag_id)) errors.push(`Duplicate tag ID: ${tag.tag_id}`); ids.add(tag.tag_id); if (boundary.length && (!pointInPolygon({ x: tag.x, y: tag.y }, boundary) || holes.some((hole) => pointInPolygon({ x: tag.x, y: tag.y }, hole)))) errors.push(`Tag ${tag.tag_id}: outside floor or inside hole`); });
  for (let i = 0; i < tags.length; i += 1) for (let j = i + 1; j < tags.length; j += 1) if (distance(tags[i], tags[j]) < minimumSeparation) errors.push(`Tags ${tags[i].tag_id} and ${tags[j].tag_id} are too close`);
  return errors;
}

export function validateGeneratedTags(tags: LayoutNavigationTag[]): string[] {
  const ids = new Set<number>(), errors: string[] = [];
  for (const tag of tags) { if (ids.has(tag.tag_id)) errors.push(`Duplicate tag ID: ${tag.tag_id}`); ids.add(tag.tag_id); }
  return errors;
}
