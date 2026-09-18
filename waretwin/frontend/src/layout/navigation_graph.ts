import type { Point } from "./geometry";
import type { LayoutAisle, LayoutNavigationEdge, LayoutNavigationTag } from "./types";

export type NavigationEdgeDirection = "bidirectional" | "forward" | "reverse";
type Projection = { distance: number; along: number };

const VALID_DIRECTIONS: readonly NavigationEdgeDirection[] = ["bidirectional", "forward", "reverse"];
const EPSILON = 1e-9;
const PROJECTION_TOLERANCE = 0.35;

function floorOf(value: unknown): string { return String(value ?? 1); }
function tagAisles(tag: LayoutNavigationTag): string[] {
  const sources = Array.isArray(tag.source_aisles) && tag.source_aisles.length ? tag.source_aisles : String(tag.generated_from ?? "").split(",");
  return sources.map(String).map((value) => value.trim()).filter(Boolean);
}

/** Project a point onto a polyline and return perpendicular distance and arc-length offset. */
export function projectAlongCenterline(centerline: Point[], point: Point): Projection | null {
  if (centerline.length < 2) return null;
  let along = 0;
  let best: Projection | null = null;
  for (let index = 1; index < centerline.length; index += 1) {
    const start = centerline[index - 1];
    const end = centerline[index];
    const dx = end.x - start.x;
    const dy = end.y - start.y;
    const length = Math.hypot(dx, dy);
    if (length <= EPSILON) continue;
    const t = Math.max(0, Math.min(1, ((point.x - start.x) * dx + (point.y - start.y) * dy) / (length * length)));
    const projectedX = start.x + t * dx;
    const projectedY = start.y + t * dy;
    const candidate = { distance: Math.hypot(point.x - projectedX, point.y - projectedY), along: along + t * length };
    if (!best || candidate.distance < best.distance) best = candidate;
    along += length;
  }
  return best;
}

function edgeKey(edge: Pick<LayoutNavigationEdge, "floor_id" | "aisle_id" | "from_tag_uuid" | "to_tag_uuid" | "direction">): string {
  return [floorOf(edge.floor_id), edge.aisle_id, edge.from_tag_uuid, edge.to_tag_uuid, edge.direction].join("|");
}
function stableEdgeUuid(key: string): string { return `edge-${key.replace(/[^a-zA-Z0-9_-]/g, "_")}`; }
function isManual(edge: LayoutNavigationEdge): boolean { return edge.placement === "manual" || edge.locked === true; }

/** Generate adjacent aisle connections while preserving manual/locked records. */
export function generateNavigationEdges(aisles: LayoutAisle[], tags: LayoutNavigationTag[], existing: LayoutNavigationEdge[] = []): LayoutNavigationEdge[] {
  const result = existing.filter(isManual).map((edge) => ({ ...edge }));
  const occupied = new Set(result.map(edgeKey));
  for (const aisle of aisles) {
    const aisleFloor = floorOf(aisle.floor_id);
    const projected = tags
      .filter((tag) => floorOf(tag.floor_id) === aisleFloor)
      .map((tag) => ({ tag, projection: projectAlongCenterline(aisle.centerline, { x: tag.x, y: tag.y }) }))
      .filter((entry): entry is { tag: LayoutNavigationTag; projection: Projection } => entry.projection !== null && (tagAisles(entry.tag).includes(String(aisle.id)) || entry.projection.distance <= PROJECTION_TOLERANCE))
      .sort((left, right) => left.projection.along - right.projection.along || left.tag.uuid.localeCompare(right.tag.uuid));
    for (let index = 1; index < projected.length; index += 1) {
      const previous = projected[index - 1];
      const current = projected[index];
      const distance = current.projection.along - previous.projection.along;
      if (!Number.isFinite(distance) || distance <= EPSILON) continue;
      let from = previous.tag;
      let to = current.tag;
      if (aisle.direction === "reverse") [from, to] = [to, from];
      const direction = aisle.direction as NavigationEdgeDirection;
      const candidateKey = edgeKey({ floor_id: aisle.floor_id ?? 1, aisle_id: String(aisle.id), from_tag_uuid: from.uuid, to_tag_uuid: to.uuid, direction });
      if (occupied.has(candidateKey)) continue;
      const previousAuto = existing.find((edge) => !isManual(edge) && edgeKey(edge) === candidateKey);
      const speedLimit = Number(aisle.speed_limit);
      const speed = Number.isFinite(speedLimit) && speedLimit > 0 ? speedLimit : undefined;
      result.push({
        ...(previousAuto ?? {}),
        uuid: previousAuto?.uuid ?? stableEdgeUuid(candidateKey),
        from_tag_uuid: from.uuid,
        to_tag_uuid: to.uuid,
        from_tag_id: from.tag_id,
        to_tag_id: to.tag_id,
        aisle_id: String(aisle.id),
        floor_id: aisle.floor_id ?? 1,
        distance,
        cost: previousAuto?.cost && previousAuto.cost > 0 ? previousAuto.cost : speed ? distance / speed : distance,
        speed_limit: previousAuto?.speed_limit ?? speed,
        direction,
        bidirectional: direction === "bidirectional",
        enabled: previousAuto?.enabled ?? true,
        placement: "auto",
        locked: false,
      });
      occupied.add(candidateKey);
    }
  }
  return result;
}

export type NavigationEdgeValidation = { errors: string[]; warnings: string[] };

/** Validate references, direction, geometry metadata and graph topology before commit/publish. */
export function validateNavigationEdges(tags: LayoutNavigationTag[], edges: LayoutNavigationEdge[], aisles: LayoutAisle[] = []): NavigationEdgeValidation {
  const tagByUuid = new Map(tags.map((tag) => [tag.uuid, tag]));
  const aisleById = new Map(aisles.map((aisle) => [String(aisle.id), aisle]));
  const seen = new Set<string>();
  const logicalSeen = new Set<string>();
  const edgeUuids = new Set<string>();
  const errors: string[] = [];
  const warnings: string[] = [];
  for (const edge of edges) {
    const from = tagByUuid.get(edge.from_tag_uuid);
    const to = tagByUuid.get(edge.to_tag_uuid);
    const key = edgeKey(edge);
    if (!edge.uuid) errors.push("Navigation edge is missing UUID");
    const logicalKey = edge.direction === "bidirectional"
      ? [floorOf(edge.floor_id), edge.aisle_id, ...[edge.from_tag_uuid, edge.to_tag_uuid].sort(), edge.direction].join("|")
      : key;
    if (edgeUuids.has(edge.uuid)) errors.push(`Duplicate navigation edge UUID ${edge.uuid}`);
    edgeUuids.add(edge.uuid);
    if (!from || !to) errors.push(`Navigation edge ${edge.uuid} references a missing tag`);
    if (edge.from_tag_uuid === edge.to_tag_uuid) errors.push(`Navigation edge ${edge.uuid} is a self-edge`);
    if (seen.has(key)) errors.push(`Duplicate navigation edge ${key}`);
    seen.add(key);
    if (logicalSeen.has(logicalKey)) errors.push(`Duplicate navigation edge ${logicalKey}`);
    logicalSeen.add(logicalKey);
    if (!VALID_DIRECTIONS.includes(edge.direction)) errors.push(`Navigation edge ${edge.uuid} has invalid direction`);
    if (!Number.isFinite(edge.distance) || edge.distance <= 0) errors.push(`Navigation edge ${edge.uuid} has invalid distance`);
    if (!Number.isFinite(edge.cost) || edge.cost <= 0) errors.push(`Navigation edge ${edge.uuid} has invalid cost`);
    if (from && to && floorOf(from.floor_id) !== floorOf(to.floor_id)) errors.push(`Navigation edge ${edge.uuid} crosses floors`);
    const aisle = aisleById.get(String(edge.aisle_id));
    if (aisles.length > 0 && !aisle) errors.push(`Navigation edge ${edge.uuid} references a missing aisle`);
    if (aisle && floorOf(aisle.floor_id) !== floorOf(edge.floor_id)) errors.push(`Navigation edge ${edge.uuid} has an invalid floor reference`);
    if (from && floorOf(from.floor_id) !== floorOf(edge.floor_id)) errors.push(`Navigation edge ${edge.uuid} has an invalid source floor`);
    if (to && floorOf(to.floor_id) !== floorOf(edge.floor_id)) errors.push(`Navigation edge ${edge.uuid} has an invalid target floor`);
  }
  for (const tag of tags) if (!edges.some((edge) => edge.from_tag_uuid === tag.uuid || edge.to_tag_uuid === tag.uuid)) warnings.push(`Navigation tag ${tag.uuid} is orphaned`);
  return { errors, warnings };
}
