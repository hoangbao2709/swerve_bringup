import type { LayoutRack } from "./types";

/**
 * Operational contract for the warehouse demo:
 * - every physical shelf has exactly 8 order slots;
 * - one order therefore represents 12.5% occupancy;
 * - when an old/demo layout has no saved current_load yet, show a deterministic
 *   light starter load (0..3 orders) so the map is readable without looking full.
 */
export const SHELF_CAPACITY = 8;

export function starterLoadForRack(rackId: string): number {
  const starterPattern = [0, 1, 0, 2, 0, 1, 0, 3] as const;
  const match = rackId.match(/(\d+)$/);
  if (match) return starterPattern[Math.abs(Number(match[1]) - 1) % starterPattern.length];
  let hash = 0;
  for (let i = 0; i < rackId.length; i++) hash = (hash * 31 + rackId.charCodeAt(i)) | 0;
  return starterPattern[Math.abs(hash) % starterPattern.length];
}

export function rackOccupancy(rack: LayoutRack) {
  const raw = typeof rack.current_load === "number" && Number.isFinite(rack.current_load)
    ? rack.current_load
    : starterLoadForRack(rack.id);
  const load = Math.max(0, Math.min(SHELF_CAPACITY, Math.trunc(raw)));
  const percent = (load / SHELF_CAPACITY) * 100;
  return {
    capacity: SHELF_CAPACITY,
    load,
    percent,
    percentLabel: Number.isInteger(percent) ? String(percent) : percent.toFixed(1),
  };
}
