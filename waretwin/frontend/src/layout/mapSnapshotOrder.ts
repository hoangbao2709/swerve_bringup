import type { RobotDetailMapSnapshot } from "../schema/twin_state";

type SnapshotOrderingFields = Pick<RobotDetailMapSnapshot,
  "robot_id" | "mapping_session_id" | "active_map_id" | "map_version" | "stamp" | "timestamp">;

type SnapshotProgress = {
  sessionId: string;
  mapVersion: number | null;
  sourceStamp: number | null;
};

export type MapSnapshotOrderResult = { accepted: boolean; reason: string };

function finite(value: number | null | undefined): number | null {
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

function sourceStamp(map: SnapshotOrderingFields): number | null {
  const stamp = finite(map.stamp);
  if (stamp !== null) return stamp;
  if (!map.timestamp) return null;
  const parsed = Date.parse(map.timestamp);
  return Number.isFinite(parsed) ? parsed / 1000 : null;
}

function progress(map: SnapshotOrderingFields): SnapshotProgress {
  return {
    sessionId: map.mapping_session_id || map.active_map_id || "",
    mapVersion: finite(map.map_version),
    sourceStamp: sourceStamp(map),
  };
}

/** Prevent delayed snapshots from replacing newer content in the same map session. */
export class MapSnapshotOrderGuard {
  private readonly latestByRobot = new Map<string, SnapshotProgress>();

  accept(
    incoming: SnapshotOrderingFields,
    current: SnapshotOrderingFields | null,
    expectedSessionId?: string | null,
  ): MapSnapshotOrderResult {
    const next = progress(incoming);
    if (expectedSessionId && next.sessionId !== expectedSessionId) {
      return { accepted: false, reason: "mapping_session_mismatch" };
    }

    const remembered = this.latestByRobot.get(incoming.robot_id);
    const stored = current ? progress(current) : null;
    const previous = remembered?.sessionId === next.sessionId
      ? remembered
      : stored?.sessionId === next.sessionId ? stored : null;

    if (previous) {
      if (next.mapVersion !== null && previous.mapVersion !== null) {
        if (next.mapVersion < previous.mapVersion) {
          return { accepted: false, reason: "older_map_version" };
        }
        if (next.mapVersion === previous.mapVersion
            && next.sourceStamp !== null && previous.sourceStamp !== null
            && next.sourceStamp < previous.sourceStamp) {
          return { accepted: false, reason: "older_source_stamp" };
        }
      } else if (next.sourceStamp !== null && previous.sourceStamp !== null
          && next.sourceStamp < previous.sourceStamp) {
        return { accepted: false, reason: "older_source_stamp" };
      }
    }

    this.latestByRobot.set(incoming.robot_id, next);
    return { accepted: true, reason: "current_or_newer" };
  }
}
