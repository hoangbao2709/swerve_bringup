import type { RobotDetailMapSnapshot } from "../../schema/twin_state";

export type NavigationMapView = "GLOBAL" | "LIDAR_2D" | "LIDAR_3D";

export type NavigationMapIdentity = {
  frame_id: string;
  map_id: string;
  map_revision: string;
};

export type MapPointNavigationTarget = NavigationMapIdentity & {
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
    if (active_map_id !== "CANONICAL" || canonical_revision == null
        || String(active_map_revision) !== String(canonical_revision)) return null;
    return { frame_id: "map", map_id: "CANONICAL", map_revision: String(active_map_revision) };
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
    map_revision: String(map_snapshot.active_map_revision) };
}

export function sameNavigationMapIdentity(
  left: NavigationMapIdentity | null | undefined,
  right: NavigationMapIdentity | null | undefined,
): boolean {
  return Boolean(left && right && left.frame_id === right.frame_id
    && left.map_id === right.map_id && left.map_revision === right.map_revision);
}

export function mapPointTarget(identity: NavigationMapIdentity, point: { x: number; y: number; yaw: number }): MapPointNavigationTarget {
  return { ...identity, x: point.x, y: point.y, yaw: point.yaw };
}
