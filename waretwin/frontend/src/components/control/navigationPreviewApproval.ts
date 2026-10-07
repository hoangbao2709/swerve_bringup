import type { RobotDetailPathPreview } from "../../schema/twin_state";
import type { MapPointNavigationTarget } from "./navigationMapIdentity";

export type PreviewApprovalGate =
  | "REQUEST_ID_MISMATCH"
  | "PREVIEW_NOT_VALID"
  | "PATH_EMPTY"
  | "ACTIVE_MAP_ID_MISMATCH"
  | "ACTIVE_MAP_REVISION_MISMATCH"
  | "MAP_SOURCE_MISMATCH"
  | "TARGET_POSITION_MISMATCH"
  | "TARGET_YAW_MISMATCH"
  | "PREVIEW_TIMESTAMP_INVALID"
  | "PREVIEW_TOO_OLD"
  | "PREVIEW_FROM_FUTURE";

export type PreviewApprovalInput = {
  pathPreview: RobotDetailPathPreview | null;
  requestId: string;
  target: MapPointNavigationTarget | null;
  activeMapId: string | null;
  activeMapRevision: string | null;
  now: number;
};

export type PreviewApprovalResult = {
  candidatePreview: RobotDetailPathPreview | null;
  approvedPreview: RobotDetailPathPreview | null;
  approved: boolean;
  rejectReason: PreviewApprovalGate | null;
  gates: {
    requestIdMatches: boolean;
    validStatus: boolean;
    pathNonEmpty: boolean;
    activeMapIdMatches: boolean;
    activeMapRevisionMatches: boolean;
    mapSourceMatches: boolean;
    targetPositionMatches: boolean;
    targetYawMatches: boolean;
    timestampValid: boolean;
    notTooOld: boolean;
    notFromFuture: boolean;
  };
};

const POSITION_TOLERANCE_M = 1e-4;
const ANGLE_TOLERANCE_RAD = 1e-4;
const MAX_PREVIEW_AGE_MS = 120_000;
const MAX_PREVIEW_FUTURE_SKEW_MS = 5_000;

/** Smallest directional angular difference, including the +/-pi wrap boundary. */
export function directionalAngleDistanceRad(a: number, b: number): number {
  if (!Number.isFinite(a) || !Number.isFinite(b)) return Number.POSITIVE_INFINITY;
  return Math.abs(Math.atan2(Math.sin(a - b), Math.cos(a - b)));
}

/** Approve only the latest preview for the selected point on the current active map. */
export function evaluatePreviewApproval(input: PreviewApprovalInput): PreviewApprovalResult {
  const preview = input.pathPreview;
  const candidatePreview = preview?.request_id === input.requestId && input.requestId ? preview : null;
  const target = input.target;
  const sourcePose = candidatePreview?.goal;
  const targetCoordinatesValid = Boolean(target && sourcePose
    && [target.x, target.y, target.yaw, sourcePose.x, sourcePose.y, sourcePose.yaw].every(Number.isFinite));
  const positionMatches = Boolean(targetCoordinatesValid && target && sourcePose
    && Math.abs(sourcePose.x - target.x) <= POSITION_TOLERANCE_M
    && Math.abs(sourcePose.y - target.y) <= POSITION_TOLERANCE_M);
  const yawMatches = Boolean(targetCoordinatesValid && target && sourcePose
    && directionalAngleDistanceRad(sourcePose.yaw, target.yaw) <= ANGLE_TOLERANCE_RAD);
  const mapSourceMatches = Boolean(target
    && target.frame_id === "map" && target.source_type === "ACTIVE_MAP_POINT"
    && target.map_id === input.activeMapId && target.map_revision === input.activeMapRevision
    && target.source_map_id === input.activeMapId && target.source_map_revision === input.activeMapRevision
    && candidatePreview?.frame_id === "map"
    && candidatePreview.source_type === "ACTIVE_MAP_POINT"
    && candidatePreview.source_map_id === target.source_map_id
    && candidatePreview.source_map_revision === target.source_map_revision);
  const timestampMs = candidatePreview?.timestamp ? Date.parse(candidatePreview.timestamp) : Number.NaN;
  const ageMs = input.now - timestampMs;
  const gates = {
    requestIdMatches: Boolean(candidatePreview),
    validStatus: candidatePreview?.status === "VALID",
    pathNonEmpty: Boolean(candidatePreview?.path.length),
    activeMapIdMatches: Boolean(candidatePreview?.active_map_id === input.activeMapId),
    activeMapRevisionMatches: Boolean(candidatePreview?.active_map_revision === input.activeMapRevision),
    mapSourceMatches,
    targetPositionMatches: positionMatches,
    targetYawMatches: yawMatches,
    timestampValid: Number.isFinite(timestampMs),
    notTooOld: Number.isFinite(timestampMs) && ageMs <= MAX_PREVIEW_AGE_MS,
    notFromFuture: Number.isFinite(timestampMs) && ageMs >= -MAX_PREVIEW_FUTURE_SKEW_MS,
  };
  const rejectionReasons: Array<[keyof typeof gates, PreviewApprovalGate]> = [
    ["requestIdMatches", "REQUEST_ID_MISMATCH"],
    ["validStatus", "PREVIEW_NOT_VALID"],
    ["pathNonEmpty", "PATH_EMPTY"],
    ["activeMapIdMatches", "ACTIVE_MAP_ID_MISMATCH"],
    ["activeMapRevisionMatches", "ACTIVE_MAP_REVISION_MISMATCH"],
    ["mapSourceMatches", "MAP_SOURCE_MISMATCH"],
    ["targetPositionMatches", "TARGET_POSITION_MISMATCH"],
    ["targetYawMatches", "TARGET_YAW_MISMATCH"],
    ["timestampValid", "PREVIEW_TIMESTAMP_INVALID"],
    ["notTooOld", "PREVIEW_TOO_OLD"],
    ["notFromFuture", "PREVIEW_FROM_FUTURE"],
  ];
  const rejectReason = rejectionReasons.find(([gate]) => !gates[gate])?.[1] ?? null;
  const approved = rejectReason === null;
  return { candidatePreview, approvedPreview: approved ? candidatePreview : null, approved, rejectReason, gates };
}
