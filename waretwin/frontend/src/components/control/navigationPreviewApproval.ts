import type { RobotDetailPathPreview } from "../../schema/twin_state";

export type PreviewTargetMethod = "TAG" | "MAP_POINT";
export type PreviewTarget = {
  x: number;
  y: number;
  yaw: number;
  frame_id?: string;
  map_id?: string;
  map_revision?: string;
  source_type?: string;
  source_map_id?: string;
  source_map_revision?: string;
};
export type PreviewTagIdentity = {
  tag_id: number;
  tag_revision: string;
  registration_revision?: number;
};

export type PreviewApprovalGate =
  | "REQUEST_ID_MISMATCH"
  | "PREVIEW_NOT_VALID"
  | "PATH_EMPTY"
  | "TAG_ROUTE_INCOMPLETE"
  | "ACTIVE_MAP_ID_MISMATCH"
  | "ACTIVE_MAP_REVISION_MISMATCH"
  | "MAP_SOURCE_MISMATCH"
  | "TARGET_POSITION_MISMATCH"
  | "TARGET_YAW_MISMATCH"
  | "TAG_SOURCE_MISMATCH"
  | "TAG_REGISTRATION_REVISION_MISMATCH"
  | "PREVIEW_TIMESTAMP_INVALID"
  | "PREVIEW_TOO_OLD"
  | "PREVIEW_FROM_FUTURE";

export type PreviewApprovalInput = {
  pathPreview: RobotDetailPathPreview | null;
  requestId: string;
  targetMethod: PreviewTargetMethod;
  target: PreviewTarget | null;
  activeMapId: string | null;
  activeMapRevision: string | null;
  canonicalMapRevision: string | number;
  selectedTag: PreviewTagIdentity | null;
  registryRevision: string | null;
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
    tagRouteComplete: boolean;
    activeMapIdMatches: boolean;
    activeMapRevisionMatches: boolean;
    mapSourceMatches: boolean;
    targetPositionMatches: boolean;
    targetYawMatches: boolean;
    tagIdentityMatches: boolean;
    tagRegistrationMatches: boolean;
    timestampValid: boolean;
    notTooOld: boolean;
    notFromFuture: boolean;
  };
};

const POSITION_TOLERANCE_M = 1e-4;
const ANGLE_TOLERANCE_RAD = 1e-4;
const MAX_PREVIEW_AGE_MS = 120_000;
const MAX_PREVIEW_FUTURE_SKEW_MS = 5_000;

/** Smallest absolute angular difference, including the +/-pi wrap boundary. */
export function angleDistanceRad(a: number, b: number): number {
  if (!Number.isFinite(a) || !Number.isFinite(b)) return Number.POSITIVE_INFINITY;
  return Math.abs(Math.atan2(Math.sin(a - b), Math.cos(a - b)));
}

export function evaluatePreviewApproval(input: PreviewApprovalInput): PreviewApprovalResult {
  const preview = input.pathPreview;
  const candidatePreview = preview?.request_id === input.requestId && input.requestId ? preview : null;
  const tagRouteComplete = input.targetMethod !== "TAG" || Boolean(
    candidatePreview?.route_revision && candidatePreview.route_nodes?.length
    && Array.isArray(candidatePreview.route_segments)
    && candidatePreview.active_path?.length && candidatePreview.canonical_path?.length
    && candidatePreview.active_route_points?.length && candidatePreview.canonical_route_points?.length,
  );
  const sourcePose = input.targetMethod === "MAP_POINT" && input.target?.source_type === "CANONICAL_MAP_POINT"
    ? candidatePreview?.source_goal : candidatePreview?.goal;
  const targetCoordinatesValid = Boolean(input.target && sourcePose
    && Number.isFinite(input.target.x) && Number.isFinite(input.target.y) && Number.isFinite(input.target.yaw)
    && Number.isFinite(sourcePose.x) && Number.isFinite(sourcePose.y) && Number.isFinite(sourcePose.yaw));
  const positionMatches = Boolean(targetCoordinatesValid && input.target && sourcePose
    && Math.abs(sourcePose.x - input.target.x) <= POSITION_TOLERANCE_M
    && Math.abs(sourcePose.y - input.target.y) <= POSITION_TOLERANCE_M);
  const yawMatches = Boolean(targetCoordinatesValid && input.target && sourcePose
    && angleDistanceRad(sourcePose.yaw, input.target.yaw) <= ANGLE_TOLERANCE_RAD);
  const mapSourceMatches = input.targetMethod === "TAG" || Boolean(input.target
    && input.target.frame_id === "map" && input.target.map_id && input.target.map_revision
    && input.target.source_type && input.target.source_map_id && input.target.source_map_revision
    && candidatePreview?.source_type === input.target.source_type
    && candidatePreview.source_map_id === input.target.source_map_id
    && candidatePreview.source_map_revision === input.target.source_map_revision
    && (input.target.source_type === "CANONICAL_MAP_POINT"
      ? input.target.source_map_id === "CANONICAL"
        && input.target.source_map_revision === String(input.canonicalMapRevision)
      : input.target.source_map_id === input.activeMapId
        && input.target.source_map_revision === input.activeMapRevision));
  const tagIdentityMatches = input.targetMethod !== "TAG" || Boolean(candidatePreview?.source_type === "TAG"
    && input.selectedTag
    && candidatePreview.tag_id === input.selectedTag.tag_id
    && candidatePreview.tag_revision === input.selectedTag.tag_revision
    && candidatePreview.registry_revision === input.registryRevision);
  const tagRegistrationMatches = input.targetMethod !== "TAG" || Boolean(input.selectedTag
    && candidatePreview?.registration_revision === input.selectedTag.registration_revision);
  const timestampMs = candidatePreview?.timestamp ? Date.parse(candidatePreview.timestamp) : Number.NaN;
  const ageMs = input.now - timestampMs;
  const gates = {
    requestIdMatches: Boolean(candidatePreview),
    validStatus: candidatePreview?.status === "VALID",
    pathNonEmpty: Boolean(candidatePreview?.path.length),
    tagRouteComplete,
    activeMapIdMatches: Boolean(candidatePreview?.active_map_id === input.activeMapId),
    activeMapRevisionMatches: Boolean(candidatePreview?.active_map_revision === input.activeMapRevision),
    mapSourceMatches,
    targetPositionMatches: positionMatches,
    targetYawMatches: yawMatches,
    tagIdentityMatches,
    tagRegistrationMatches,
    timestampValid: Number.isFinite(timestampMs),
    notTooOld: Number.isFinite(timestampMs) && ageMs <= MAX_PREVIEW_AGE_MS,
    notFromFuture: Number.isFinite(timestampMs) && ageMs >= -MAX_PREVIEW_FUTURE_SKEW_MS,
  };
  const rejectionReasons: Array<[keyof typeof gates, PreviewApprovalGate]> = [
    ["requestIdMatches", "REQUEST_ID_MISMATCH"],
    ["validStatus", "PREVIEW_NOT_VALID"],
    ["pathNonEmpty", "PATH_EMPTY"],
    ["tagRouteComplete", "TAG_ROUTE_INCOMPLETE"],
    ["activeMapIdMatches", "ACTIVE_MAP_ID_MISMATCH"],
    ["activeMapRevisionMatches", "ACTIVE_MAP_REVISION_MISMATCH"],
    ["mapSourceMatches", "MAP_SOURCE_MISMATCH"],
    ["targetPositionMatches", "TARGET_POSITION_MISMATCH"],
    ["targetYawMatches", "TARGET_YAW_MISMATCH"],
    ["tagIdentityMatches", "TAG_SOURCE_MISMATCH"],
    ["tagRegistrationMatches", "TAG_REGISTRATION_REVISION_MISMATCH"],
    ["timestampValid", "PREVIEW_TIMESTAMP_INVALID"],
    ["notTooOld", "PREVIEW_TOO_OLD"],
    ["notFromFuture", "PREVIEW_FROM_FUTURE"],
  ];
  const rejectReason = rejectionReasons.find(([gate]) => !gates[gate])?.[1] ?? null;
  const approved = rejectReason === null;
  return { candidatePreview, approvedPreview: approved ? candidatePreview : null, approved, rejectReason, gates };
}
