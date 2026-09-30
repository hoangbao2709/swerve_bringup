import type { RobotDetailMapSnapshot } from "../../schema/twin_state";

/** Decode the bridge's compact OccupancyGrid transport for browser rendering. */
export async function decodeOccupancyGrid(snapshot: RobotDetailMapSnapshot): Promise<Int8Array | null> {
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
