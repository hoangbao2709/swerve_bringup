import type { RobotDetailMapSnapshot } from "../schema/twin_state";
import { createWorldTransform, type ScreenPoint, type WorldBounds, type WorldTransform } from "./coordinates";

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
  const minimumScale = camera.fitScalePxPerMeter * 0.5;
  const maximumScale = camera.fitScalePxPerMeter * 8;
  return {
    ...camera,
    scalePxPerMeter: Math.max(minimumScale, Math.min(maximumScale, camera.scalePxPerMeter * factor)),
  };
}

export function centerMapViewportCamera(camera: MapViewportCamera, point: ScreenPoint): MapViewportCamera {
  return { ...camera, centerX: point.x, centerY: point.y };
}
