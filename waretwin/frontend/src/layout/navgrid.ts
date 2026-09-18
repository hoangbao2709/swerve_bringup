import { floorBoundary, pointXY } from "./coordinates";
import { pointInPolygon, type Point } from "./geometry";
import { sameFloor, type FloorId, type WarehouseLayout } from "./types";

/** Options shared by simulation and editor nav-grid generation. */
export interface NavGridBuildOptions {
  /** Radius of the robot footprint in metres. */
  robot_radius?: number;
  /** Additional clearance around geometry in metres. */
  safety_margin?: number;
}

// Existing maps historically used a centre-point occupancy mask. Keep the
// default compatible with those maps; simulation/editor callers can provide
// the calibrated robot dimensions through NavGridBuildOptions.
const DEFAULT_ROBOT_RADIUS = 0;
const DEFAULT_SAFETY_MARGIN = 0;
const EPSILON = 1e-9;

function points(raw: unknown): Point[] {
  if (!Array.isArray(raw)) return [];
  return raw.map((p) => pointXY(p as Parameters<typeof pointXY>[0])).filter((p) => Number.isFinite(p.x) && Number.isFinite(p.y));
}

function distanceToSegment(p: Point, a: Point, b: Point): number {
  const dx = b.x - a.x, dy = b.y - a.y;
  const lengthSquared = dx * dx + dy * dy;
  if (lengthSquared <= EPSILON) return Math.hypot(p.x - a.x, p.y - a.y);
  const t = Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / lengthSquared));
  return Math.hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

function distanceToPolygon(p: Point, polygon: Point[]): number {
  if (polygon.length < 2) return Infinity;
  let result = Infinity;
  for (let i = 0; i < polygon.length; i += 1) result = Math.min(result, distanceToSegment(p, polygon[i], polygon[(i + 1) % polygon.length]));
  return result;
}

function distanceToRect(p: Point, rect: [number, number, number, number]): number {
  const x0 = Math.min(rect[0], rect[2]), x1 = Math.max(rect[0], rect[2]);
  const y0 = Math.min(rect[1], rect[3]), y1 = Math.max(rect[1], rect[3]);
  const dx = p.x < x0 ? x0 - p.x : p.x > x1 ? p.x - x1 : 0;
  const dy = p.y < y0 ? y0 - p.y : p.y > y1 ? p.y - y1 : 0;
  return Math.hypot(dx, dy);
}

function rotatedRackBounds(r: WarehouseLayout["racks"][number]): [number, number, number, number] {
  const x = r.position[0], y = r.position[2], w = r.size[0], h = r.size[2];
  const angle = ((r.rotation ?? 0) * Math.PI) / 180;
  if (Math.abs(angle) < EPSILON) return [x, y, x + w, y + h];
  const cx = x + w / 2, cy = y + h / 2, c = Math.cos(angle), s = Math.sin(angle);
  const corners = [[x, y], [x + w, y], [x + w, y + h], [x, y + h]].map(([px, py]) => {
    const dx = px - cx, dy = py - cy;
    return [cx + dx * c + dy * s, cy - dx * s + dy * c];
  });
  const xs = corners.map((p) => p[0]), ys = corners.map((p) => p[1]);
  return [Math.min(...xs), Math.min(...ys), Math.max(...xs), Math.max(...ys)];
}

function floorGeometry(layout: WarehouseLayout, floor: FloorId): { boundary: Point[]; holes: Point[][] } | null {
  const entry = (layout.floors ?? []).find((candidate) => sameFloor(candidate.id, floor));
  if (!entry) return null;
  const boundary = floorBoundary(entry, layout.size.width, layout.size.depth);
  const holes = (entry.holes ?? []).map(points).filter((hole) => hole.length >= 3);
  return boundary.length >= 3 ? { boundary, holes } : null;
}

/**
 * Build a conservative occupancy grid in warehouse metres.
 * 0 = walkable, 1 = blocked, 2 = walkable slow area.
 *
 * Floor geometry is deliberately evaluated per cell centre.  A floor is not
 * its axis-aligned bounding box: concave sections and holes remain blocked.
 */
export function buildNavGrid(
  layout: WarehouseLayout,
  floor: FloorId = 1,
  options: NavGridBuildOptions = {},
): { cols: number; rows: number; cells: Uint8Array } {
  const { cols, rows, cell_size: cellSize } = layout.grid;
  const cells = new Uint8Array(Math.max(0, cols * rows));
  cells.fill(1);
  if (!Number.isFinite(cellSize) || cellSize <= 0 || cols <= 0 || rows <= 0) return { cols, rows, cells };

  const geometry = floorGeometry(layout, floor);
  if (!geometry) return { cols, rows, cells };
  const robotRadius = Number.isFinite(options.robot_radius) && (options.robot_radius ?? 0) >= 0 ? options.robot_radius as number : DEFAULT_ROBOT_RADIUS;
  const safetyMargin = Number.isFinite(options.safety_margin) && (options.safety_margin ?? 0) >= 0 ? options.safety_margin as number : DEFAULT_SAFETY_MARGIN;
  const inflation = robotRadius + safetyMargin;
  const firstFloor = (layout.floors ?? [])[0]?.id ?? 1;
  const isFirstFloor = sameFloor(floor, firstFloor);

  const center = (column: number, row: number): Point => ({ x: (column + 0.5) * cellSize, y: (row + 0.5) * cellSize });
  const insideFloor = (p: Point): boolean => pointInPolygon(p, geometry.boundary) && !geometry.holes.some((hole) => pointInPolygon(p, hole));
  const nearFloorBoundary = (p: Point): boolean => distanceToPolygon(p, geometry.boundary) < inflation - EPSILON || geometry.holes.some((hole) => distanceToPolygon(p, hole) < inflation - EPSILON);

  for (let row = 0; row < rows; row += 1) for (let column = 0; column < cols; column += 1) {
    const p = center(column, row);
    if (insideFloor(p) && !nearFloorBoundary(p)) cells[row * cols + column] = 0;
  }

  const floorMatches = (candidate: FloorId | null | undefined): boolean => sameFloor(candidate ?? firstFloor, floor);
  const forEachCell = (fn: (index: number, p: Point) => void) => {
    for (let row = 0; row < rows; row += 1) for (let column = 0; column < cols; column += 1) fn(row * cols + column, center(column, row));
  };
  const blockRect = (rect: [number, number, number, number], extraInflation = inflation, coverIntersectingCell = false) => {
    forEachCell((index, p) => {
      const cellOverlap = coverIntersectingCell ? cellSize / 2 : 0;
      if (cells[index] !== 1 && distanceToRect(p, rect) <= extraInflation + cellOverlap + EPSILON) cells[index] = 1;
    });
  };
  const blockPolygon = (polygon: Point[], extraInflation = inflation) => {
    if (polygon.length < 3) return;
    forEachCell((index, p) => {
      if (cells[index] !== 1 && (pointInPolygon(p, polygon) || distanceToPolygon(p, polygon) <= extraInflation + EPSILON)) cells[index] = 1;
    });
  };
  const markSlowPolygon = (polygon: Point[]) => {
    if (polygon.length < 3) return;
    forEachCell((index, p) => { if (cells[index] === 0 && pointInPolygon(p, polygon)) cells[index] = 2; });
  };

  // Aisles are semantic corridors, not obstacles. Walkways may be slow or
  // restricted, and are therefore applied after the floor mask.
  for (const walkway of layout.walkways ?? []) {
    const polygon = points(walkway.polygon);
    if (walkway.robots_allowed === false) blockPolygon(polygon);
    else markSlowPolygon(polygon);
  }
  for (const rack of layout.racks ?? []) if (rack.blocks_grid && floorMatches(rack.floor)) blockRect(rotatedRackBounds(rack));

  // These legacy physical objects are currently canonical F1 entities. They
  // must not leak into a separate floor's occupancy grid.
  if (isFirstFloor) {
    for (const conveyor of layout.conveyors ?? []) if (conveyor.blocks_grid) {
      const halfWidth = conveyor.width / 2;
      for (let i = 0; i < conveyor.path.length - 1; i += 1) {
        const a = conveyor.path[i], b = conveyor.path[i + 1];
        blockRect([Math.min(a[0], b[0]) - halfWidth, Math.min(a[1], b[1]) - halfWidth, Math.max(a[0], b[0]) + halfWidth, Math.max(a[1], b[1]) + halfWidth], inflation, true);
      }
    }
    for (const area of layout.restricted_areas ?? []) if (!area.robots_allowed) blockRect(area.rect);
    for (const station of layout.stations ?? []) blockRect(station.rect);
    for (const [x, y] of layout.columns ?? []) blockRect([x - 0.45, y - 0.45, x + 0.45, y + 0.45], inflation, true);
    for (const obstacle of layout.obstacles ?? []) blockRect(obstacle.rect, inflation, true);
    for (const charging of layout.charging_stations ?? []) blockRect([charging.position[0] - 0.45, charging.position[2] - 0.4, charging.position[0] + 0.45, charging.position[2] + 0.5]);
  }

  // Lift shafts have a deliberately conservative canonical envelope. Keep
  // the established envelope (which already includes structural clearance)
  // rather than expanding it a second time with robot inflation.
  for (const lift of layout.lifts ?? []) {
    const x = lift.cell[0] + 0.5, y = lift.cell[1] + 0.5;
    // Preserve the canonical 3×5 shaft envelope in grid-index space. Using
    // centre-distance here would lose the outer face cells at ±1.9 m due to
    // half-cell rounding.
    const c0 = Math.max(0, Math.floor((x - 1.4) / cellSize));
    const c1 = Math.min(cols - 1, Math.ceil((x + 1.4) / cellSize) - 1);
    const r0 = Math.max(0, Math.floor((y - 1.9) / cellSize));
    const r1 = Math.min(rows - 1, Math.ceil((y + 1.9) / cellSize) - 1);
    for (let row = r0; row <= r1; row += 1) for (let column = c0; column <= c1; column += 1) cells[row * cols + column] = 1;
  }
  return { cols, rows, cells };
}
