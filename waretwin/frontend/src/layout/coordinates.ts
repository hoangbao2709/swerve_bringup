export type WarehouseXYZ = { x: number; y: number; z: number };
export type ScreenPoint = { x: number; y: number };

/** Canonical convention: warehouse X/Y are floor-plane metres, Z is elevation. */
export function warehouseToScreen(point: ScreenPoint, origin: ScreenPoint, scale: number): ScreenPoint {
  return { x: (point.x - origin.x) * scale, y: (point.y - origin.y) * scale };
}
export function screenToWarehouse(point: ScreenPoint, origin: ScreenPoint, scale: number): ScreenPoint {
  if (!Number.isFinite(scale) || scale === 0) throw new Error("scale must be finite and non-zero");
  return { x: point.x / scale + origin.x, y: point.y / scale + origin.y };
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
