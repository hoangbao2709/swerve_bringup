import { distance, pointInPolygon, type Point } from "./geometry";
import type { LayoutAisle, LayoutFloor, LayoutNavigationTag } from "./types";

export type AisleIntersection = { point: Point; aisleIds: string[]; floorId: string | number; segmentA: number; segmentB: number };
export const DEFAULT_INTERSECTION_TOLERANCE = 0.01;
export const DEFAULT_TAG_MINIMUM_SEPARATION = 0.25;
const EPS = 1e-9;
type CandidateRole = "intersection" | "turn" | "start" | "end" | "spacing";
type Candidate = { point: Point; floorId: string | number; sourceAisles: string[]; generatedFrom: string; role: CandidateRole; distanceAlong: number; yaw: number };
const rolePriority: Record<CandidateRole, number> = { intersection: 0, turn: 1, start: 2, end: 2, spacing: 3 };

function segmentIntersection(a: Point, b: Point, c: Point, d: Point, tolerance: number): Point | null {
  const r = { x: b.x - a.x, y: b.y - a.y }, s = { x: d.x - c.x, y: d.y - c.y }, q = { x: c.x - a.x, y: c.y - a.y }, cross = r.x * s.y - r.y * s.x;
  if (Math.abs(cross) <= EPS) { for (const p of [c, d, a, b]) if (distance(a, p) + distance(p, b) <= distance(a, b) + tolerance && distance(c, p) + distance(p, d) <= distance(c, d) + tolerance) return p; return null; }
  const t = (q.x * s.y - q.y * s.x) / cross, u = (q.x * r.y - q.y * r.x) / cross;
  return t < -tolerance || t > 1 + tolerance || u < -tolerance || u > 1 + tolerance ? null : { x: a.x + t * r.x, y: a.y + t * r.y };
}
export function findAisleIntersections(aisles: LayoutAisle[], tolerance = DEFAULT_INTERSECTION_TOLERANCE): AisleIntersection[] {
  const result: AisleIntersection[] = [];
  const add = (candidate: AisleIntersection) => { const existing = result.find((item) => String(item.floorId) === String(candidate.floorId) && distance(item.point, candidate.point) <= tolerance); if (existing) existing.aisleIds = [...new Set([...existing.aisleIds, ...candidate.aisleIds])].sort(); else result.push(candidate); };
  for (let i = 0; i < aisles.length; i += 1) for (let j = i + 1; j < aisles.length; j += 1) {
    const a = aisles[i], b = aisles[j]; if (String(a.floor_id ?? 1) !== String(b.floor_id ?? 1)) continue;
    for (let sa = 1; sa < a.centerline.length; sa += 1) for (let sb = 1; sb < b.centerline.length; sb += 1) { const point = segmentIntersection(a.centerline[sa - 1], a.centerline[sa], b.centerline[sb - 1], b.centerline[sb], tolerance); if (point) add({ point, aisleIds: [String(a.id), String(b.id)].sort(), floorId: a.floor_id ?? 1, segmentA: sa - 1, segmentB: sb - 1 }); }
  }
  return result;
}
function candidateKey(c: Candidate): string { return String(c.floorId) + "|" + c.sourceAisles.slice().sort().join(",") + "|" + c.role + "|" + Math.round(c.point.x * 1000) + ":" + Math.round(c.point.y * 1000); }
function addCandidate(candidates: Candidate[], candidate: Candidate, tolerance: number) {
  const existing = candidates.find((item) => String(item.floorId) === String(candidate.floorId) && distance(item.point, candidate.point) <= tolerance);
  if (!existing) candidates.push(candidate); else { existing.sourceAisles = [...new Set([...existing.sourceAisles, ...candidate.sourceAisles])].sort(); if (rolePriority[candidate.role] < rolePriority[existing.role]) Object.assign(existing, { role: candidate.role, generatedFrom: candidate.generatedFrom, distanceAlong: candidate.distanceAlong, yaw: candidate.yaw }); }
}
function resolveCandidateSeparation(candidates: Candidate[], minimumSeparation: number): Candidate[] {
  const ordered = [...candidates].sort((a, b) => rolePriority[a.role] - rolePriority[b.role] || candidateKey(a).localeCompare(candidateKey(b)));
  return ordered.reduce<Candidate[]>((kept, candidate) => kept.some((other) => String(other.floorId) === String(candidate.floorId) && distance(other.point, candidate.point) < minimumSeparation) ? kept : [...kept, candidate], []);
}
function aisleCandidates(aisles: LayoutAisle[], tolerance: number, minimumSeparation: number): Candidate[] {
  const candidates: Candidate[] = [];
  for (const aisle of aisles) {
    const points = aisle.centerline; if (points.length < 2) continue;
    const floorId = aisle.floor_id ?? 1, source = String(aisle.id), rule = { enabled: true, spacing: 2, start_offset: 0, end_offset: 0, ...(aisle.tag_rule ?? {}) }; if (!rule.enabled) continue;
    const prefix: number[] = [0]; for (let i = 1; i < points.length; i += 1) prefix.push(prefix[i - 1] + distance(points[i - 1], points[i])); const total = prefix[prefix.length - 1];
    const yawAt = (index: number) => Math.atan2(points[Math.min(index + 1, points.length - 1)].y - points[Math.max(0, index - 1)].y, points[Math.min(index + 1, points.length - 1)].x - points[Math.max(0, index - 1)].x);
    const addAt = (along: number, role: CandidateRole) => { const bounded = Math.max(0, Math.min(total, along)); let segment = 1; while (segment < prefix.length - 1 && prefix[segment] < bounded) segment += 1; const a = points[segment - 1], b = points[segment], length = distance(a, b) || 1, ratio = (bounded - prefix[segment - 1]) / length; addCandidate(candidates, { point: { x: a.x + (b.x - a.x) * ratio, y: a.y + (b.y - a.y) * ratio }, floorId, sourceAisles: [source], generatedFrom: source, role, distanceAlong: bounded, yaw: yawAt(segment - 1) }, tolerance); };
    addAt(rule.start_offset, "start"); points.slice(1, -1).forEach((point, index) => addCandidate(candidates, { point, floorId, sourceAisles: [source], generatedFrom: source, role: "turn", distanceAlong: prefix[index + 1], yaw: yawAt(index + 1) }, tolerance));
    if (rule.spacing > 0) for (let along = rule.start_offset + rule.spacing; along < total - rule.end_offset - tolerance; along += rule.spacing) addAt(along, "spacing");
    if (total - rule.end_offset > rule.start_offset + tolerance) addAt(total - rule.end_offset, "end");
  }
  for (const intersection of findAisleIntersections(aisles, tolerance)) addCandidate(candidates, { point: intersection.point, floorId: intersection.floorId, sourceAisles: intersection.aisleIds, generatedFrom: intersection.aisleIds.join(","), role: "intersection", distanceAlong: 0, yaw: 0 }, tolerance);
  return resolveCandidateSeparation(candidates, minimumSeparation);
}
function tagSources(tag: LayoutNavigationTag): string[] { return tag.source_aisles?.map(String) ?? String(tag.generated_from ?? "").split(",").filter(Boolean); }
function matchesCandidate(tag: LayoutNavigationTag, c: Candidate, tolerance: number): boolean { return tag.logical_key ? tag.logical_key === candidateKey(c) : String(tag.floor_id ?? 1) === String(c.floorId) && tag.semantic_role === c.role && tagSources(tag).sort().join(",") === c.sourceAisles.slice().sort().join(",") && Math.abs((tag.distance_along_aisle ?? c.distanceAlong) - c.distanceAlong) <= tolerance; }
function tagIsInAisleScope(tag: LayoutNavigationTag, aisleId: string): boolean { return tagSources(tag).includes(String(aisleId)); }
function nextTagId(existing: LayoutNavigationTag[]): number { const used = new Set(existing.map((tag) => Number(tag.tag_id)).filter(Number.isInteger)); let id = Math.max(1000, ...Array.from(used)) + 1; while (used.has(id)) id += 1; return id; }

/** Generate from all aisles; selected regeneration limits replacement scope, never intersection discovery. */
export function generateAisleTags(aisles: LayoutAisle[], existing: LayoutNavigationTag[], options: { tolerance?: number; minimumSeparation?: number; onlyAisleId?: string } | number = {}): LayoutNavigationTag[] {
  const tolerance = typeof options === "number" ? options : options.tolerance ?? DEFAULT_INTERSECTION_TOLERANCE, minimumSeparation = typeof options === "number" ? DEFAULT_TAG_MINIMUM_SEPARATION : options.minimumSeparation ?? DEFAULT_TAG_MINIMUM_SEPARATION, onlyAisleId = typeof options === "number" ? undefined : options.onlyAisleId;
  const allCandidates = aisleCandidates(aisles, tolerance, minimumSeparation), candidates = onlyAisleId ? allCandidates.filter((candidate) => candidate.sourceAisles.includes(String(onlyAisleId))) : allCandidates;
  const preserved = existing.filter((tag) => tag.placement === "manual" || tag.locked || Boolean(onlyAisleId && !tagIsInAisleScope(tag, onlyAisleId))), available = existing.filter((tag) => tag.placement === "auto" && !tag.locked && (!onlyAisleId || tagIsInAisleScope(tag, onlyAisleId))), result = [...preserved], consumed = new Set<string>();
  for (const candidate of candidates) {
    if (preserved.some((tag) => matchesCandidate(tag, candidate, tolerance))) continue;
    const matched = available.find((tag) => !consumed.has(tag.uuid) && matchesCandidate(tag, candidate, tolerance)) ?? available.find((tag) => !consumed.has(tag.uuid) && String(tag.floor_id ?? 1) === String(candidate.floorId) && distance(tag, candidate.point) <= tolerance && tagSources(tag).some((id) => candidate.sourceAisles.includes(id)));
    if (matched) consumed.add(matched.uuid);
    const key = candidateKey(candidate), identity = matched?.uuid ?? "tag-" + key.replace(/[^a-zA-Z0-9_-]/g, "_");
    result.push({ ...(matched ?? {}), uuid: identity, id: matched?.id ?? identity, tag_id: matched?.tag_id ?? nextTagId([...existing, ...result]), floor_id: candidate.floorId, x: candidate.point.x, y: candidate.point.y, z: matched?.z ?? 0, yaw: matched?.yaw ?? candidate.yaw, placement: "auto", locked: false, generated_from: candidate.generatedFrom, source_aisles: candidate.sourceAisles, semantic_role: candidate.role, logical_key: key, distance_along_aisle: candidate.distanceAlong });
  }
  return result;
}
export function applyNavigationTagPhysicalOverride(tag: LayoutNavigationTag): LayoutNavigationTag { return tag.placement === "auto" ? { ...tag, placement: "manual", locked: true } : tag; }
function floorPoints(floor: LayoutFloor): Point[] { return (floor.boundary ?? floor.footprint ?? []).map((point) => Array.isArray(point) ? { x: point[0], y: point[1] } : point); }
function floorHoles(floor: LayoutFloor): Point[][] { return (floor.holes ?? []).map((hole) => hole.map((point) => Array.isArray(point) ? { x: point[0], y: point[1] } : point)); }
/** Shared generation validation for global and selected calls, before a draft can be committed. */
export function validateNavigationTagsForLayout(floors: LayoutFloor[], tags: LayoutNavigationTag[], minimumSeparation = DEFAULT_TAG_MINIMUM_SEPARATION): string[] {
  const errors: string[] = [], ids = new Set<number>(), uuids = new Set<string>(), floorById = new Map(floors.map((floor) => [String(floor.id), floor]));
  for (const tag of tags) {
    if (!Number.isInteger(tag.tag_id) || tag.tag_id < 0) errors.push("Tag " + tag.uuid + ": invalid tag ID"); if (ids.has(tag.tag_id)) errors.push("Duplicate tag ID: " + tag.tag_id); ids.add(tag.tag_id);
    if (!tag.uuid) errors.push("Navigation tag: missing UUID"); else if (uuids.has(tag.uuid)) errors.push("Duplicate tag UUID: " + tag.uuid); uuids.add(tag.uuid);
    if (![tag.x, tag.y, tag.z ?? 0, tag.yaw].every(Number.isFinite)) errors.push("Tag " + tag.tag_id + ": invalid x/y/z/yaw");
    const floor = floorById.get(String(tag.floor_id ?? 1)); if (!floor) { errors.push("Tag " + tag.tag_id + ": invalid floor reference"); continue; }
    const boundary = floorPoints(floor), holes = floorHoles(floor); if (boundary.length && (!pointInPolygon({ x: tag.x, y: tag.y }, boundary) || holes.some((hole) => pointInPolygon({ x: tag.x, y: tag.y }, hole)))) errors.push("Tag " + tag.tag_id + ": outside floor or inside hole");
  }
  for (let i = 0; i < tags.length; i += 1) for (let j = i + 1; j < tags.length; j += 1) if (String(tags[i].floor_id ?? 1) === String(tags[j].floor_id ?? 1) && distance(tags[i], tags[j]) < minimumSeparation) errors.push("Tags " + tags[i].tag_id + " and " + tags[j].tag_id + " are too close");
  return errors;
}
export function validateTags(tags: LayoutNavigationTag[], boundary: Point[], holes: Point[][] = [], minimumSeparation = DEFAULT_TAG_MINIMUM_SEPARATION): string[] { return validateNavigationTagsForLayout([{ id: 1, name: "floor", elevation: 0, boundary, holes }], tags.map((tag) => ({ ...tag, floor_id: 1 })), minimumSeparation); }
export function validateGeneratedTags(tags: LayoutNavigationTag[]): string[] { return validateNavigationTagsForLayout([{ id: 1, name: "floor", elevation: 0 }], tags.map((tag) => ({ ...tag, floor_id: 1 }))); }
/** IDs remain monotonic after deletions: aisle-001, aisle-003 => aisle-004. */
export function nextAisleId(aisles: LayoutAisle[], prefix = "aisle"): string {
  const escapedPrefix = prefix.replace(/[^a-zA-Z0-9_-]/g, "\\$&"), pattern = new RegExp("^" + escapedPrefix + "-([0-9]+)$");
  const maximum = aisles.reduce((max, aisle) => Math.max(max, Number(pattern.exec(String(aisle.id))?.[1]) || 0), 0);
  return prefix + "-" + String(maximum + 1).padStart(3, "0");
}
