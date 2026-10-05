import type { RobotDetailMapSnapshot } from "../../schema/twin_state";

export type NavigationMapView = "GLOBAL" | "LIDAR_2D" | "LIDAR_3D";

export type NavigationMapIdentity = {
  frame_id: string;
  map_id: string;
  map_revision: string;
  source_type: "ACTIVE_MAP_POINT" | "CANONICAL_MAP_POINT";
};

export type MapPointNavigationTarget = NavigationMapIdentity & {
  source_map_id: string;
  source_map_revision: string;
  x: number;
  y: number;
  yaw: number;
};

export type MapPointPreviewPayload = {
  frame_id: "map";
  active_map_id: string;
  active_map_revision: string;
  map_id: string;
  map_revision: string;
  map_content_revision?: string;
  source_type: "ACTIVE_MAP_POINT" | "CANONICAL_MAP_POINT";
  source_map_id: string;
  source_map_revision: string;
  x: number;
  y: number;
  yaw: number;
};

type DisplayedNavigationMapInput = {
  view: NavigationMapView;
  active_map_id: string | null;
  active_map_revision: string | null;
  canonical_revision: string | number | null;
  map_snapshot: RobotDetailMapSnapshot | null;
};

/** Identity of the map pixels currently being shown, only when they are the robot's active map. */
export function displayedNavigationMapIdentity(input: DisplayedNavigationMapInput): NavigationMapIdentity | null {
  const { view, active_map_id, active_map_revision, canonical_revision, map_snapshot } = input;
  if (!active_map_id || !active_map_revision) return null;

  if (view === "GLOBAL") {
    if (canonical_revision == null || (active_map_id === "CANONICAL"
        && String(active_map_revision) !== String(canonical_revision))) return null;
    return { frame_id: "map", map_id: "CANONICAL", map_revision: String(canonical_revision),
      source_type: "CANONICAL_MAP_POINT" };
  }

  if (view !== "LIDAR_2D" || !map_snapshot || map_snapshot.frame_id !== "map"
      || !map_snapshot.active_map_id || !map_snapshot.active_map_revision
      || map_snapshot.active_map_id !== active_map_id
      || String(map_snapshot.active_map_revision) !== String(active_map_revision)
      || !["SLAM_TOOLBOX", "LOCAL_MAP", "NAV2_MAP"].includes(map_snapshot.map_source ?? "")) return null;

  if (map_snapshot.map_source === "SLAM_TOOLBOX"
      && (!map_snapshot.mapping_session_id
        || map_snapshot.active_map_id !== `SLAM-${map_snapshot.mapping_session_id}`)) return null;

  return { frame_id: map_snapshot.frame_id, map_id: map_snapshot.active_map_id,
    map_revision: String(map_snapshot.active_map_revision), source_type: "ACTIVE_MAP_POINT" };
}

export function sameNavigationMapIdentity(
  left: NavigationMapIdentity | null | undefined,
  right: NavigationMapIdentity | null | undefined,
): boolean {
  return Boolean(left && right && left.frame_id === right.frame_id
    && left.map_id === right.map_id && left.map_revision === right.map_revision
    && left.source_type === right.source_type);
}

export function mapPointTarget(identity: NavigationMapIdentity, point: { x: number; y: number; yaw: number }): MapPointNavigationTarget {
  return { ...identity, source_map_id: identity.map_id,
    source_map_revision: identity.map_revision, x: point.x, y: point.y, yaw: point.yaw };
}

/** Keep source/display map coordinates separate from the active Nav2 identity. */
export function mapPointPreviewPayload(target: MapPointNavigationTarget,
  activeMap: { map_id: string; map_revision: string }, mapContentRevision?: string | null): MapPointPreviewPayload | null {
  if (target.frame_id !== "map" || !target.source_map_id || !target.source_map_revision
      || ![target.x, target.y, target.yaw].every(Number.isFinite)) return null;
  if (target.source_type === "ACTIVE_MAP_POINT"
      && (target.source_map_id !== activeMap.map_id || target.source_map_revision !== activeMap.map_revision)) return null;
  if (target.source_type === "CANONICAL_MAP_POINT" && target.source_map_id !== "CANONICAL") return null;
  return {
    frame_id: "map", active_map_id: activeMap.map_id, active_map_revision: activeMap.map_revision,
    map_id: activeMap.map_id, map_revision: activeMap.map_revision,
    ...(mapContentRevision ? { map_content_revision: mapContentRevision } : {}),
    source_type: target.source_type, source_map_id: target.source_map_id,
    source_map_revision: target.source_map_revision,
    x: target.x, y: target.y, yaw: target.yaw,
  };
}
