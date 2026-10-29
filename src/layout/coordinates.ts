export type WarehouseXYZ = { x: number; y: number; z: number };
export type ScreenPoint = { x: number; y: number };
export type WorldBounds = { minX: number; maxX: number; minY: number; maxY: number };
export type WorldTransform = {
  scale: number;
  width: number;
  height: number;
  centerX: number;
  centerY: number;
};

/** One reversible metres-to-viewport transform; screen Y points downward. */
export function createWorldTransform(
  viewport: { width: number; height: number },
  bounds: WorldBounds,
  zoom = 1,
  center?: ScreenPoint | null,
  padding = 28,
): WorldTransform {
  const width = Math.max(0, viewport.width), height = Math.max(0, viewport.height);
  const spanX = Math.max(Number.EPSILON, bounds.maxX - bounds.minX);
  const spanY = Math.max(Number.EPSILON, bounds.maxY - bounds.minY);
  const safeZoom = Number.isFinite(zoom) && zoom > 0 ? zoom : 1;
  const scale = Math.min(
    Math.max(0, width - padding * 2) / spanX,
    Math.max(0, height - padding * 2) / spanY,
  ) * safeZoom;
  return {
    scale,
    width,
    height,
    centerX: center?.x ?? (bounds.minX + bounds.maxX) / 2,
    centerY: center?.y ?? (bounds.minY + bounds.maxY) / 2,
  };
}

export function worldToScreen(point: ScreenPoint, transform: WorldTransform): ScreenPoint {
  return {
    x: transform.width / 2 + (point.x - transform.centerX) * transform.scale,
    y: transform.height / 2 - (point.y - transform.centerY) * transform.scale,
  };
}

export function screenToWorld(point: ScreenPoint, transform: WorldTransform): ScreenPoint {
  if (!Number.isFinite(transform.scale) || transform.scale <= 0) {
    throw new Error("world transform scale must be finite and positive");
  }
  return {
    x: transform.centerX + (point.x - transform.width / 2) / transform.scale,
    y: transform.centerY - (point.y - transform.height / 2) / transform.scale,
  };
}

/** SVG group matrix that gives world +Y an upward visual direction. */
export function worldToSvgTransform(bounds: WorldBounds): string {
  return `translate(0 ${bounds.minY + bounds.maxY}) scale(1 -1)`;
}

/** Canonical convention: warehouse X/Y are floor-plane metres, Z is elevation. */
export function warehouseToScreen(point: ScreenPoint, origin: ScreenPoint, scale: number): ScreenPoint {
  return { x: (point.x - origin.x) * scale, y: -(point.y - origin.y) * scale };
}
export function screenToWarehouse(point: ScreenPoint, origin: ScreenPoint, scale: number): ScreenPoint {
  if (!Number.isFinite(scale) || scale === 0) throw new Error("scale must be finite and non-zero");
  return { x: point.x / scale + origin.x, y: -point.y / scale + origin.y };
}
export function warehouseToThree(point: WarehouseXYZ): [number, number, number] { return [point.x, point.z, point.y]; }
export function threeToWarehouse(point: [number, number, number]): WarehouseXYZ { return { x: point[0], y: point[2], z: point[1] }; }
export function warehouseToRos(point: WarehouseXYZ): WarehouseXYZ { return { ...point }; }
export function rosToWarehouse(point: WarehouseXYZ): WarehouseXYZ { return { ...point }; }

export type FloorPoint = { x: number; y: number } | [number, number];

export function pointXY(point: FloorPoint): { x: number; y: number } {
  return Array.isArray(point) ? { x: point[0], y: point[1] } : point;
}

export function floorBoundary(
  floor: { boundary?: FloorPoint[]; footprint?: [number, number][] },
  width: number,
  depth: number,
): { x: number; y: number }[] {
  const points = (floor.boundary ?? floor.footprint ?? [[0, 0], [width, 0], [width, depth], [0, depth]]) as FloorPoint[];
  return points.map(pointXY);
}

export function polygonPoints(points: FloorPoint[]): string {
  return points.map((point) => { const p = pointXY(point); return `${p.x},${p.y}`; }).join(" ");
}
