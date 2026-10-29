import type { RobotDetailMapSnapshot } from "../../schema/twin_state";
import { classifyOccupancyPoint, type OccupancyPointClassification } from "../../components/shared/maps/occupancyGrid";

export type InitialPose = { x: number; y: number; yaw: number };
export type AngleUnit = "RAD" | "DEG";
export type PoseValidationResult = { valid: true } | { valid: false; reason: string; classification?: OccupancyPointClassification };

export function localizationMapIdentityKey(robotId: string, map: RobotDetailMapSnapshot | null): string {
  if (!map) return "";
  return JSON.stringify([robotId, map.frame_id, map.map_source ?? "UNKNOWN", map.active_map_id ?? "",
    String(map.active_map_revision ?? ""), map.mapping_session_id ?? ""]);
}

export function normalizeYawRadians(yaw: number): number {
  if (!Number.isFinite(yaw)) return Number.NaN;
  const wrapped = ((yaw + Math.PI) % (Math.PI * 2) + Math.PI * 2) % (Math.PI * 2) - Math.PI;
  return Object.is(wrapped, -0) ? 0 : wrapped;
}

export function radiansToDisplayYaw(yaw: number, unit: AngleUnit): number {
  return unit === "DEG" ? yaw * 180 / Math.PI : yaw;
}

export function displayYawToRadians(yaw: number, unit: AngleUnit): number {
  return unit === "DEG" ? yaw * Math.PI / 180 : yaw;
}

export function validateInitialPose(pose: InitialPose, map: RobotDetailMapSnapshot | null,
  cells: Int8Array | null): PoseValidationResult {
  if (![pose.x, pose.y, pose.yaw].every(Number.isFinite)) return { valid: false, reason: "ENTER FINITE X, Y AND YAW VALUES" };
  if (pose.yaw < -Math.PI || pose.yaw > Math.PI) return { valid: false, reason: "YAW MUST BE WITHIN −π TO +π RAD" };
  if (!map || map.frame_id !== "map" || !map.active_map_id || !map.active_map_revision) {
    return { valid: false, reason: "CONFIRMED ACTIVE MAP IDENTITY IS REQUIRED" };
  }
  if (!cells) return { valid: false, reason: "MAP OCCUPANCY DATA IS NOT AVAILABLE" };
  const classification = classifyOccupancyPoint(map, cells, pose);
  if (classification !== "FREE") {
    const labels: Record<Exclude<OccupancyPointClassification, "FREE">, string> = {
      UNKNOWN: "POSE MUST BE IN A KNOWN FREE CELL",
      OCCUPIED: "POSE MUST NOT BE IN AN OCCUPIED CELL",
      OUTSIDE: "POSE IS OUTSIDE THE ACTIVE MAP",
    };
    return { valid: false, reason: labels[classification], classification };
  }
  return { valid: true };
}
