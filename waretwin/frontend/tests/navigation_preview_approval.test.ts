import { describe, expect, it } from "vitest";
import type { RobotDetailPathPreview } from "../src/schema/twin_state";
import { angleDistanceRad, evaluatePreviewApproval, type PreviewApprovalInput } from "../src/components/control/navigationPreviewApproval";

const tag = { tag_id: 1103, tag_revision: "tag-r7", registration_revision: 23 };
const timestamp = new Date("2026-10-07T00:00:00.000Z").toISOString();

function validPreview(overrides: Partial<RobotDetailPathPreview> = {}): RobotDetailPathPreview {
  return {
    robot_id: "R01", request_id: "preview-1", status: "VALID", frame_id: "map",
    path: [{ x: 1, y: 2 }, { x: 3, y: 4 }], active_path: [{ x: 1, y: 2 }, { x: 3, y: 4 }],
    canonical_path: [{ x: 1, y: 2 }, { x: 3, y: 4 }], route_nodes: [1102, 1103],
    route_segments: [{ from: 1102, to: 1103, axis: "X", length_m: 2 }],
    active_route_points: [{ x: 1, y: 2, yaw: 0, kind: "TAG", tag_id: 1102 }],
    canonical_route_points: [{ x: 1, y: 2, yaw: 0, kind: "TAG", tag_id: 1102 }],
    route_revision: "route-r2", graph_revision: "graph-r4", path_length_m: 2,
    goal: { x: 3, y: 4, yaw: -3.141440167775226 },
    source_type: "TAG", source_id: "1103", tag_id: 1103, tag_revision: "tag-r7",
    registry_revision: "registry-r9", registration_revision: 23,
    active_map_id: "SLAM-session-a", active_map_revision: "session-session-a",
    timestamp, ...overrides,
  };
}

function input(preview: RobotDetailPathPreview, overrides: Partial<PreviewApprovalInput> = {}): PreviewApprovalInput {
  return {
    pathPreview: preview, requestId: "preview-1", targetMethod: "TAG",
    target: { x: 3, y: 4, yaw: -3.141440167775226 },
    activeMapId: "SLAM-session-a", activeMapRevision: "session-session-a", canonicalMapRevision: "23",
    selectedTag: tag, registryRevision: "registry-r9", now: Date.parse(timestamp), ...overrides,
  };
}

describe("navigation preview approval", () => {
  it("accepts equivalent shelf yaw values across the -pi/+pi boundary", () => {
    const selectedYaw = -3.141440167775226;
    const wrappedPreviewYaw = selectedYaw + Math.PI * 2;
    const result = evaluatePreviewApproval(input(validPreview({ goal: { x: 3, y: 4, yaw: wrappedPreviewYaw } })));

    expect(angleDistanceRad(wrappedPreviewYaw, selectedYaw)).toBeLessThan(1e-4);
    expect(result.gates.targetPositionMatches).toBe(true);
    expect(result.gates.targetYawMatches).toBe(true);
    expect(result.approvedPreview?.status).toBe("VALID");
  });

  it("accepts the reverse +pi/-pi wrap direction", () => {
    const selectedYaw = Math.PI - 0.000152485814567;
    const wrappedPreviewYaw = selectedYaw - Math.PI * 2;
    const result = evaluatePreviewApproval(input(
      validPreview({ goal: { x: 3, y: 4, yaw: wrappedPreviewYaw } }),
      { target: { x: 3, y: 4, yaw: selectedYaw } },
    ));

    expect(result.gates.targetYawMatches).toBe(true);
    expect(result.approved).toBe(true);
  });

  it("still rejects real yaw, X, and Y mismatches without widening positional tolerance", () => {
    const preview = validPreview();
    const yawMismatch = evaluatePreviewApproval(input({
      ...preview, goal: { x: 3, y: 4, yaw: preview.goal!.yaw + 0.001 },
    }));
    const xMismatch = evaluatePreviewApproval(input({ ...preview, goal: { ...preview.goal!, x: 3.0002 } }));
    const yMismatch = evaluatePreviewApproval(input({ ...preview, goal: { ...preview.goal!, y: 4.0002 } }));

    expect(yawMismatch.approved).toBe(false);
    expect(yawMismatch.rejectReason).toBe("TARGET_YAW_MISMATCH");
    expect(xMismatch.approved).toBe(false);
    expect(xMismatch.rejectReason).toBe("TARGET_POSITION_MISMATCH");
    expect(yMismatch.approved).toBe(false);
    expect(yMismatch.rejectReason).toBe("TARGET_POSITION_MISMATCH");
  });

  it("rejects the captured live shelf result when its yaw is pi radians from the registered service pose", () => {
    const result = evaluatePreviewApproval(input(validPreview({
      goal: { x: 3, y: 4, yaw: 0.00015248581456694943 },
    })));

    expect(result.gates.targetPositionMatches).toBe(true);
    expect(result.gates.targetYawMatches).toBe(false);
    expect(result.rejectReason).toBe("TARGET_YAW_MISMATCH");
    expect(result.approved).toBe(false);
  });

  it("keeps request, source, map, registration, route completeness, and freshness gates fail-closed", () => {
    const preview = validPreview();
    expect(evaluatePreviewApproval(input(preview, { requestId: "other" })).rejectReason).toBe("REQUEST_ID_MISMATCH");
    expect(evaluatePreviewApproval(input(validPreview({ status: "INVALID" }))).rejectReason).toBe("PREVIEW_NOT_VALID");
    expect(evaluatePreviewApproval(input(validPreview({ path: [] }))).rejectReason).toBe("PATH_EMPTY");
    expect(evaluatePreviewApproval(input(validPreview({ route_revision: null }))).rejectReason).toBe("TAG_ROUTE_INCOMPLETE");
    expect(evaluatePreviewApproval(input(validPreview({ active_map_id: "SLAM-session-old" }))).rejectReason)
      .toBe("ACTIVE_MAP_ID_MISMATCH");
    expect(evaluatePreviewApproval(input(validPreview({ active_map_revision: "session-old" }))).rejectReason)
      .toBe("ACTIVE_MAP_REVISION_MISMATCH");
    expect(evaluatePreviewApproval(input(validPreview({ tag_id: 1303 }))).rejectReason).toBe("TAG_SOURCE_MISMATCH");
    expect(evaluatePreviewApproval(input(validPreview({ registry_revision: "registry-old" }))).rejectReason)
      .toBe("TAG_SOURCE_MISMATCH");
    expect(evaluatePreviewApproval(input(validPreview({ registration_revision: 22 }))).rejectReason)
      .toBe("TAG_REGISTRATION_REVISION_MISMATCH");
    expect(evaluatePreviewApproval(input(validPreview({ timestamp: "invalid" }))).rejectReason)
      .toBe("PREVIEW_TIMESTAMP_INVALID");
    expect(evaluatePreviewApproval(input(validPreview(), { now: Date.parse(timestamp) + 120_001 })).rejectReason)
      .toBe("PREVIEW_TOO_OLD");
    expect(evaluatePreviewApproval(input(validPreview(), { now: Date.parse(timestamp) - 5_001 })).rejectReason)
      .toBe("PREVIEW_FROM_FUTURE");
  });

  it("keeps canonical MAP_POINT source map identity and request-source validation", () => {
    const preview = validPreview({ source_type: "CANONICAL_MAP_POINT", source_id: null, tag_id: null,
      tag_revision: null, registry_revision: null, registration_revision: null,
      source_map_id: "CANONICAL", source_map_revision: "23",
      source_goal: { x: 3, y: 4, yaw: 0.5 }, goal: { x: 3, y: 4, yaw: 0.5 },
      route_revision: null, route_nodes: null, route_segments: null, active_route_points: null,
      canonical_route_points: null });
    const target = { x: 3, y: 4, yaw: 0.5, frame_id: "map", map_id: "SLAM-session-a",
      map_revision: "session-session-a", source_type: "CANONICAL_MAP_POINT",
      source_map_id: "CANONICAL", source_map_revision: "23" };
    const approvalInput: PreviewApprovalInput = { ...input(preview), targetMethod: "MAP_POINT",
      target, selectedTag: null, registryRevision: null };

    expect(evaluatePreviewApproval(approvalInput).approved).toBe(true);
    expect(evaluatePreviewApproval({ ...approvalInput,
      pathPreview: { ...preview, source_map_revision: "22" } }).rejectReason).toBe("MAP_SOURCE_MISMATCH");
    expect(evaluatePreviewApproval({ ...approvalInput,
      target: { ...target, source_map_revision: "22" } }).rejectReason).toBe("MAP_SOURCE_MISMATCH");
  });
});
