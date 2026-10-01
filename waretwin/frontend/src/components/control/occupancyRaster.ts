import type { RobotDetailMapSnapshot } from "../../schema/twin_state";
import { decodeOccupancyGrid } from "./occupancyGrid";
import { detailPerformance } from "./detailPerformance";

const rasterKeys = new WeakMap<RobotDetailMapSnapshot, string>();
export function occupancyRasterKey(map: RobotDetailMapSnapshot): string {
  const cached = rasterKeys.get(map);
  if (cached) return cached;
  let payload = map.data_zlib_base64;
  if (!payload) {
    let hash = 2166136261;
    for (const cell of map.data ?? []) hash = Math.imul(hash ^ (cell + 1), 16777619) >>> 0;
    payload = `raw:${map.data?.length}:${hash}`;
  }
  const key = JSON.stringify([map.robot_id, map.active_map_id, map.active_map_revision,
    map.map_revision, map.map_version, map.frame_id, map.width, map.height, map.resolution, map.origin, payload]);
  rasterKeys.set(map, key);
  return key;
}

/** Content-aware LRU: at most four rasters / 8M cells (32 MB RGBA). */
export class OccupancyRasterCache<T> {
  private entries = new Map<string, { cells: number; value: T | null; promise: Promise<T | null> }>();
  builds = 0;
  hits = 0;
  constructor(private build: (map: RobotDetailMapSnapshot) => Promise<T | null>) {}
  get size() { return this.entries.size; }
  get cells() { return [...this.entries.values()].reduce((sum, entry) => sum + entry.cells, 0); }
  peek(map: RobotDetailMapSnapshot | null) { return map ? this.entries.get(occupancyRasterKey(map))?.value ?? null : null; }
  get(map: RobotDetailMapSnapshot | null): Promise<T | null> {
    if (!map || map.width * map.height <= 0 || map.width * map.height > 4_000_000) return Promise.resolve(null);
    const key = occupancyRasterKey(map);
    const existing = this.entries.get(key);
    if (existing) {
      this.hits++;
      this.entries.delete(key); this.entries.set(key, existing);
      detailPerformance("occupancy_cache_hit", { entries: this.size, cells: this.cells });
      return existing.promise;
    }
    this.builds++;
    const entry: { cells: number; value: T | null; promise: Promise<T | null> } = { cells: map.width * map.height, value: null as T | null,
      promise: Promise.resolve(null as T | null) };
    this.entries.set(key, entry);
    while (this.size > 4 || this.cells > 8_000_000) this.entries.delete(this.entries.keys().next().value!);
    entry.promise = this.build(map).then(value => {
      if (this.entries.get(key) === entry) entry.value = value;
      return value;
    }).catch(() => { if (this.entries.get(key) === entry) this.entries.delete(key); return null; });
    return entry.promise;
  }
}

async function build(map: RobotDetailMapSnapshot): Promise<HTMLCanvasElement | null> {
  if (typeof document === "undefined") return null;
  const started = performance.now();
  const source = await decodeOccupancyGrid(map);
  detailPerformance("occupancy_decode", { duration_ms: performance.now() - started });
  if (!source) return null;
  const canvas = document.createElement("canvas");
  canvas.width = map.width; canvas.height = map.height;
  const context = canvas.getContext("2d");
  if (!context) return null;
  const image = context.createImageData(map.width, map.height);
  for (let row = 0; row < map.height; row++) for (let col = 0; col < map.width; col++) {
    const occupancy = source[row * map.width + col];
    if (occupancy < 0) continue;
    const alpha = Math.max(.08, Math.min(.9, occupancy / 100));
    const index = ((map.height - row - 1) * map.width + col) * 4;
    if (occupancy > 65) {
      image.data[index] = 232; image.data[index + 1] = 92; image.data[index + 2] = 92;
      image.data[index + 3] = Math.round(alpha * 255);
    } else {
      image.data[index] = 24; image.data[index + 1] = 54; image.data[index + 2] = 77;
      image.data[index + 3] = Math.round((.18 + alpha * .35) * 255);
    }
  }
  context.putImageData(image, 0, 0);
  detailPerformance("occupancy_raster", { duration_ms: performance.now() - started });
  return canvas;
}
export const occupancyRasters = new OccupancyRasterCache(build);
