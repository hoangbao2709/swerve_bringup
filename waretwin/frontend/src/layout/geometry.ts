export type Point = { x: number; y: number };

export function distance(a: Point, b: Point): number { return Math.hypot(a.x - b.x, a.y - b.y); }
export function polylineLength(points: Point[]): number { return points.slice(1).reduce((sum, p, i) => sum + distance(points[i], p), 0); }
export function signedArea(points: Point[]): number { return points.reduce((sum, p, i) => { const q = points[(i + 1) % points.length]; return sum + p.x * q.y - q.x * p.y; }, 0) / 2; }
export function orientation(a: Point, b: Point, c: Point): number { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); }
export function onSegment(a: Point, b: Point, p: Point, eps = 1e-8): boolean { return Math.abs(orientation(a, b, p)) <= eps && p.x >= Math.min(a.x, b.x) - eps && p.x <= Math.max(a.x, b.x) + eps && p.y >= Math.min(a.y, b.y) - eps && p.y <= Math.max(a.y, b.y) + eps; }
export function segmentsIntersect(a: Point, b: Point, c: Point, d: Point): boolean {
  const o1 = orientation(a, b, c), o2 = orientation(a, b, d), o3 = orientation(c, d, a), o4 = orientation(c, d, b);
  if (((o1 > 1e-8 && o2 < -1e-8) || (o1 < -1e-8 && o2 > 1e-8)) && ((o3 > 1e-8 && o4 < -1e-8) || (o3 < -1e-8 && o4 > 1e-8))) return true;
  return (Math.abs(o1) <= 1e-8 && onSegment(a, b, c)) || (Math.abs(o2) <= 1e-8 && onSegment(a, b, d)) || (Math.abs(o3) <= 1e-8 && onSegment(c, d, a)) || (Math.abs(o4) <= 1e-8 && onSegment(c, d, b));
}
export function selfIntersects(points: Point[]): boolean { return points.some((a, i) => points.some((c, j) => { if (j <= i || j === i + 1 || (i === 0 && j === points.length - 1)) return false; return segmentsIntersect(a, points[(i + 1) % points.length], c, points[(j + 1) % points.length]); })); }
export function pointInPolygon(point: Point, polygon: Point[]): boolean { let inside = false; for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) { const a = polygon[i], b = polygon[j]; if (onSegment(a, b, point)) return true; if ((a.y > point.y) !== (b.y > point.y) && point.x < (b.x - a.x) * (point.y - a.y) / ((b.y - a.y) || 1e-30) + a.x) inside = !inside; } return inside; }
export function validatePolygon(points: Point[], label: string): string[] { const errors: string[] = []; if (points.length < 3) errors.push(`${label}: requires at least 3 vertices`); if (points.some((p) => !Number.isFinite(p.x) || !Number.isFinite(p.y))) errors.push(`${label}: invalid coordinates`); if (new Set(points.map((p) => `${p.x}:${p.y}`)).size !== points.length) errors.push(`${label}: duplicate point`); if (points.length >= 3 && Math.abs(signedArea(points)) < 1e-8) errors.push(`${label}: area must be non-zero`); if (points.length >= 3 && selfIntersects(points)) errors.push(`${label}: self-intersects`); return errors; }

export function validateFloorPolygon(points: Point[], label = "floor"): string[] {
  return validatePolygon(points, label);
}

export function validateHolePolygon(points: Point[], label = "hole"): string[] {
  return validatePolygon(points, label);
}

export function validateAisleCenterline(points: Point[], width: number, label = "aisle"): string[] {
  const errors: string[] = [];
  if (points.length < 2) errors.push(`${label}: requires at least 2 centerline points`);
  if (!Number.isFinite(width) || width <= 0) errors.push(`${label}: width must be positive and finite`);
  if (points.some((p) => !Number.isFinite(p.x) || !Number.isFinite(p.y))) errors.push(`${label}: centerline coordinates must be finite`);
  for (let i = 1; i < points.length; i += 1) {
    if (distance(points[i - 1], points[i]) <= 1e-9) errors.push(`${label}: consecutive points must not duplicate`);
  }
  return errors;
}

type Line = { point: Point; direction: Point };
function lineIntersection(a: Line, b: Line): Point | null {
  const cross = a.direction.x * b.direction.y - a.direction.y * b.direction.x;
  if (Math.abs(cross) <= 1e-10) return null;
  const dx = b.point.x - a.point.x, dy = b.point.y - a.point.y;
  const t = (dx * b.direction.y - dy * b.direction.x) / cross;
  return { x: a.point.x + a.direction.x * t, y: a.point.y + a.direction.y * t };
}

/**
 * Returns a deterministic polygonal buffer around a centerline.  Joins use a
 * bounded miter (with a bevel fallback for tight turns), so an L-turn does not
 * create the self-intersecting polygon produced by averaging adjacent normals.
 */
export function buildAisleFootprint(centerline: Point[], width: number): Point[] {
  if (validateAisleCenterline(centerline, width).length) return [];
  const half = width / 2;
  const directions = centerline.slice(1).map((p, i) => {
    const dx = p.x - centerline[i].x, dy = p.y - centerline[i].y, len = Math.hypot(dx, dy);
    return { x: dx / len, y: dy / len };
  });
  const normal = (d: Point, sign: number) => ({ x: -d.y * half * sign, y: d.x * half * sign });
  const side = (sign: number): Point[] => centerline.map((point, i) => {
    const prev = directions[Math.max(0, i - 1)], next = directions[Math.min(directions.length - 1, i)];
    if (i === 0) { const n = normal(next, sign); return { x: point.x + n.x, y: point.y + n.y }; }
    if (i === centerline.length - 1) { const n = normal(prev, sign); return { x: point.x + n.x, y: point.y + n.y }; }
    const pn = normal(prev, sign), nn = normal(next, sign);
    const intersection = lineIntersection(
      { point: { x: centerline[i - 1].x + pn.x, y: centerline[i - 1].y + pn.y }, direction: prev },
      { point: { x: point.x + nn.x, y: point.y + nn.y }, direction: next },
    );
    if (!intersection || distance(intersection, point) > half * 4) {
      return { x: point.x + (pn.x + nn.x) / 2, y: point.y + (pn.y + nn.y) / 2 };
    }
    return intersection;
  });
  return [...side(1), ...side(-1).reverse()];
}

export function polygonContainedInFloor(polygon: Point[], floor: Point[], holes: Point[][] = [], tolerance = 1e-8): boolean {
  if (polygon.length < 3 || floor.length < 3) return false;
  const strictlyInside = (point: Point, boundary: Point[]) => pointInPolygon(point, boundary) && !boundary.some((a, i) => onSegment(a, boundary[(i + 1) % boundary.length], point, tolerance));
  if (!polygon.every((p) => strictlyInside(p, floor) && !holes.some((hole) => pointInPolygon(p, hole)))) return false;
  for (let i = 0; i < polygon.length; i += 1) {
    const a = polygon[i], b = polygon[(i + 1) % polygon.length];
    for (let j = 0; j < floor.length; j += 1) {
      if (segmentsIntersect(a, b, floor[j], floor[(j + 1) % floor.length]) && !onSegment(floor[j], floor[(j + 1) % floor.length], a, tolerance) && !onSegment(floor[j], floor[(j + 1) % floor.length], b, tolerance)) return false;
    }
    for (const hole of holes) for (let j = 0; j < hole.length; j += 1) if (segmentsIntersect(a, b, hole[j], hole[(j + 1) % hole.length])) return false;
  }
  return true;
}

export function snapCoordinate(value: number, step: number | null | undefined): number {
  if (!Number.isFinite(value) || !step || !Number.isFinite(step) || step <= 0) return value;
  return Math.round(value / step) * step;
}

export function snapPoint(point: Point, step: number | null | undefined): Point {
  return { x: snapCoordinate(point.x, step), y: snapCoordinate(point.y, step) };
}
