import type { RobotDetailMapSnapshot } from "../schema/twin_state";
import { createWorldTransform, screenToWorld, type ScreenPoint, type WorldBounds, type WorldTransform } from "./coordinates";

export const MIN_MAP_SCALE_PX_PER_METER = 0.01;
export const MAX_MAP_SCALE_PX_PER_METER = 4096;
export const MAP_PAN_THRESHOLD_PX = 5;

export type MapViewportCamera = {
  centerX: number;
  centerY: number;
  scalePxPerMeter: number;
  fitScalePxPerMeter: number;
};

export type MapViewportState = {
  sessionKey: string;
  camera: MapViewportCamera;
};

export type MapViewportSize = { width: number; height: number };

/**
 * SLAM map revisions and bounds are content, not camera identity. A live
 * mapping session keeps its viewport even when the OccupancyGrid grows.
 */
export function mapViewportSessionKey(map: Pick<RobotDetailMapSnapshot,
  "robot_id" | "map_source" | "mapping_session_id" | "active_map_id">): string {
  if (map.map_source === "SLAM_TOOLBOX" && map.mapping_session_id) {
    return `slam:${map.robot_id}:${map.mapping_session_id}`;
  }
  return `map:${map.robot_id}:${map.map_source ?? "unknown"}:${map.active_map_id ?? ""}`;
}

export function occupancyMapWorldBounds(map: Pick<RobotDetailMapSnapshot,
  "width" | "height" | "resolution" | "origin">): WorldBounds {
  const yaw = map.origin.yaw;
  const width = map.width * map.resolution;
  const height = map.height * map.resolution;
  const corners = [[0, 0], [width, 0], [0, height], [width, height]].map(([x, y]) => ({
    x: map.origin.x + x * Math.cos(yaw) - y * Math.sin(yaw),
    y: map.origin.y + x * Math.sin(yaw) + y * Math.cos(yaw),
  }));
  return {
    minX: Math.min(...corners.map((point) => point.x)),
    maxX: Math.max(...corners.map((point) => point.x)),
    minY: Math.min(...corners.map((point) => point.y)),
    maxY: Math.max(...corners.map((point) => point.y)),
  };
}

export function fitMapViewportCamera(size: MapViewportSize, bounds: WorldBounds): MapViewportCamera {
  const fitted = createWorldTransform(size, bounds, 1, null, 20);
  const scalePxPerMeter = Number.isFinite(fitted.scale) && fitted.scale > 0 ? fitted.scale : 1;
  return {
    centerX: fitted.centerX,
    centerY: fitted.centerY,
    scalePxPerMeter,
    fitScalePxPerMeter: scalePxPerMeter,
  };
}

export function resolveMapViewport(
  current: MapViewportState | null,
  map: Pick<RobotDetailMapSnapshot,
    "robot_id" | "map_source" | "mapping_session_id" | "active_map_id" | "width" | "height" | "resolution" | "origin">,
  size: MapViewportSize,
): MapViewportState | null {
  const sessionKey = mapViewportSessionKey(map);
  if (current?.sessionKey === sessionKey) return current;
  if (size.width <= 0 || size.height <= 0) return null;
  return { sessionKey, camera: fitMapViewportCamera(size, occupancyMapWorldBounds(map)) };
}

export function fixedWorldTransform(size: MapViewportSize, camera: MapViewportCamera): WorldTransform {
  return {
    width: Math.max(0, size.width),
    height: Math.max(0, size.height),
    centerX: camera.centerX,
    centerY: camera.centerY,
    scale: camera.scalePxPerMeter,
  };
}

export function zoomMapViewportCamera(camera: MapViewportCamera, factor: number): MapViewportCamera {
  if (!Number.isFinite(factor) || factor <= 0) return camera;
  // Keep exceptionally large/small maps' fitted view reachable as well.
  const minimumScale = Math.min(MIN_MAP_SCALE_PX_PER_METER, camera.fitScalePxPerMeter);
  const maximumScale = Math.max(MAX_MAP_SCALE_PX_PER_METER, camera.fitScalePxPerMeter);
  return {
    ...camera,
    scalePxPerMeter: Math.max(minimumScale, Math.min(maximumScale, camera.scalePxPerMeter * factor)),
  };
}

/** Pixel movement follows the hand; screen +Y is opposite to world +Y. */
export function panMapViewportCamera(camera: MapViewportCamera, delta: ScreenPoint): MapViewportCamera {
  if (![delta.x, delta.y, camera.scalePxPerMeter].every(Number.isFinite) || camera.scalePxPerMeter <= 0) return camera;
  return { ...camera, centerX: camera.centerX - delta.x / camera.scalePxPerMeter,
    centerY: camera.centerY + delta.y / camera.scalePxPerMeter };
}

/** Preserve the world point under the cursor while changing scale. */
export function zoomMapViewportCameraAt(camera: MapViewportCamera, factor: number,
  size: MapViewportSize, cursor: ScreenPoint): MapViewportCamera {
  if (![cursor.x, cursor.y].every(Number.isFinite)) return camera;
  const zoomed = zoomMapViewportCamera(camera, factor);
  if (zoomed.scalePxPerMeter === camera.scalePxPerMeter) return camera;
  const anchor = screenToWorld(cursor, fixedWorldTransform(size, camera));
  return { ...zoomed,
    centerX: anchor.x - (cursor.x - size.width / 2) / zoomed.scalePxPerMeter,
    centerY: anchor.y + (cursor.y - size.height / 2) / zoomed.scalePxPerMeter };
}

export function centerMapViewportCamera(camera: MapViewportCamera, point: ScreenPoint): MapViewportCamera {
  return { ...camera, centerX: point.x, centerY: point.y };
}
