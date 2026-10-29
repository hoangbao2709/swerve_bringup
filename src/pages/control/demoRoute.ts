import type { RobotDetailPath, RobotDetailPathPreview, RobotWorldPoint } from "../../schema/twin_state";

export type DemoRoute = { kind: "PREVIEW" | "ACTIVE_NAV"; points: RobotWorldPoint[] };

const ACTIVE_NAV_STATES = new Set(["ACTIVE", "NAVIGATING", "RUNNING", "PAUSED"]);

/** Choose only the latest approved preview or a real Nav2 route for this displayed map. */
export function resolveDemoRoute(input: {
  navigationStatus: string | null;
  activeMapId: string | null;
  activeMapRevision: string | null;
  mapSource: string | null;
  mapContentRevision?: string | null;
  registrationRevision?: number | null;
  approvedPreview: RobotDetailPathPreview | null;
  globalPath: RobotDetailPath | null;
}): DemoRoute | null {
  const { activeMapId, activeMapRevision } = input;
  if (!activeMapId || !activeMapRevision) return null;

  if (ACTIVE_NAV_STATES.has(String(input.navigationStatus ?? "").toUpperCase())) {
    const path = input.globalPath;
    const matchesIdentity = Boolean(path && path.frame_id === "map"
      && path.active_map_id === activeMapId
      && String(path.active_map_revision ?? "") === String(activeMapRevision)
      && Boolean(path.navigation_map_id && path.navigation_map_revision)
      && (!path.map_source || path.map_source === input.mapSource)
      && (input.mapContentRevision == null
        || String(path.map_content_revision ?? "") === String(input.mapContentRevision))
      && (input.registrationRevision == null
        || path.registration_revision === input.registrationRevision));
    if (matchesIdentity && path && path.points.length > 1) {
      return { kind: "ACTIVE_NAV", points: path.points };
    }
    // An active goal must never fall back to its old preview path.
    return null;
  }

  const preview = input.approvedPreview;
  if (preview?.status !== "VALID" || preview.frame_id !== "map"
      || preview.active_map_id !== activeMapId
      || String(preview.active_map_revision ?? "") !== String(activeMapRevision)) return null;
  const points = preview.active_path?.length ? preview.active_path : preview.path;
  return points.length > 1 ? { kind: "PREVIEW", points } : null;
}
