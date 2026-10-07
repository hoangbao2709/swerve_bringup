import type { RobotDetailMapSnapshot } from "../../schema/twin_state";

const decodedGridPromises = new WeakMap<RobotDetailMapSnapshot, Promise<Int8Array | null>>();

/** Decode the bridge's compact OccupancyGrid transport for browser rendering. */
export function decodeOccupancyGrid(snapshot: RobotDetailMapSnapshot): Promise<Int8Array | null> {
  const cached = decodedGridPromises.get(snapshot);
  if (cached) return cached;
  const promise = decodeOccupancyGridSnapshot(snapshot);
  decodedGridPromises.set(snapshot, promise);
  return promise;
}

async function decodeOccupancyGridSnapshot(snapshot: RobotDetailMapSnapshot): Promise<Int8Array | null> {
  const cellCount = snapshot.width * snapshot.height;
  if (!Number.isSafeInteger(cellCount) || cellCount <= 0 || cellCount > 4_000_000) return null;
  if (Array.isArray(snapshot.data)) {
    if (snapshot.data.length !== cellCount) return null;
    return Int8Array.from(snapshot.data, (value) => Number.isFinite(value) ? value : -1);
  }
  if (snapshot.data_encoding !== "zlib-base64-offset1"
      || !snapshot.data_zlib_base64
      || snapshot.data_zlib_base64.length > 5_600_000
      || typeof DecompressionStream === "undefined") return null;
  try {
    const binary = atob(snapshot.data_zlib_base64);
    const buffer = new ArrayBuffer(binary.length);
    const compressed = new Uint8Array(buffer);
    for (let index = 0; index < binary.length; index += 1) {
      compressed[index] = binary.charCodeAt(index);
    }
    const stream = new Blob([buffer]).stream().pipeThrough(new DecompressionStream("deflate"));
    const decoded = await new Response(stream).arrayBuffer();
    if (decoded.byteLength !== cellCount) return null;
    const encodedCells = new Uint8Array(decoded);
    const occupancy = new Int8Array(cellCount);
    for (let index = 0; index < cellCount; index += 1) {
      occupancy[index] = encodedCells[index] - 1;
    }
    return occupancy;
  } catch {
    return null;
  }
}

/** True only for a known-free OccupancyGrid cell at a world-frame position. */
export function isFreeOccupancyPoint(
  snapshot: Pick<RobotDetailMapSnapshot, "width" | "height" | "resolution" | "origin">,
  cells: Int8Array,
  point: { x: number; y: number },
): boolean {
  const { width, height, resolution, origin } = snapshot;
  if (!Number.isFinite(resolution) || resolution <= 0 || !Number.isFinite(point.x) || !Number.isFinite(point.y)) return false;
  const dx = point.x - origin.x;
  const dy = point.y - origin.y;
  const cos = Math.cos(origin.yaw), sin = Math.sin(origin.yaw);
  const column = Math.floor((dx * cos + dy * sin) / resolution);
  const row = Math.floor((-dx * sin + dy * cos) / resolution);
  if (column < 0 || row < 0 || column >= width || row >= height || cells.length !== width * height) return false;
  const occupancy = cells[row * width + column];
  return occupancy >= 0 && occupancy <= 65;
}
