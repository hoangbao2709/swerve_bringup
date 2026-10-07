import { describe, expect, it } from "vitest";
import type { RobotDetailPathPreview } from "../src/schema/twin_state";
import { directionalAngleDistanceRad, evaluatePreviewApproval, type PreviewApprovalInput } from "../src/components/control/navigationPreviewApproval";

const timestamp = new Date("2026-10-07T00:00:00.000Z").toISOString();
const target = {
  frame_id: "map", map_id: "SLAM-session-a", map_revision: "session-session-a",
  source_type: "ACTIVE_MAP_POINT" as const, source_map_id: "SLAM-session-a",
  source_map_revision: "session-session-a", x: 3, y: 4, yaw: -3.141440167775226,
};

function validPreview(overrides: Partial<RobotDetailPathPreview> = {}): RobotDetailPathPreview {
  return {
    robot_id: "R01", request_id: "preview-1", status: "VALID", frame_id: "map",
    path: [{ x: 1, y: 2 }, { x: 3, y: 4 }], path_length_m: 2,
    goal: { x: 3, y: 4, yaw: target.yaw },
    source_type: "ACTIVE_MAP_POINT", source_map_id: target.source_map_id,
    source_map_revision: target.source_map_revision,
    active_map_id: "SLAM-session-a", active_map_revision: "session-session-a",
    timestamp, ...overrides,
  };
}

function input(preview: RobotDetailPathPreview, overrides: Partial<PreviewApprovalInput> = {}): PreviewApprovalInput {
  return {
    pathPreview: preview, requestId: "preview-1", target,
    activeMapId: "SLAM-session-a", activeMapRevision: "session-session-a",
    now: Date.parse(timestamp), ...overrides,
  };
}

describe("point navigation preview approval", () => {
  it("approves a valid preview for the selected point on the current active map", () => {
    const result = evaluatePreviewApproval(input(validPreview()));
    expect(result.approved).toBe(true);
    expect(result.approvedPreview?.status).toBe("VALID");
  });

  it("accepts equivalent headings across the -pi/+pi wrap boundary", () => {
    const selectedYaw = target.yaw;
    const wrappedPreviewYaw = selectedYaw + Math.PI * 2;
    const result = evaluatePreviewApproval(input(validPreview({
      goal: { x: target.x, y: target.y, yaw: wrappedPreviewYaw },
    })));
    expect(directionalAngleDistanceRad(wrappedPreviewYaw, selectedYaw)).toBeLessThan(1e-4);
    expect(result.gates.targetPositionMatches).toBe(true);
    expect(result.gates.targetYawMatches).toBe(true);
    expect(result.approved).toBe(true);
  });

  it("rejects real yaw and position mismatches without widening tolerances", () => {
    const yawMismatch = evaluatePreviewApproval(input(validPreview({
      goal: { x: target.x, y: target.y, yaw: target.yaw + 0.001 },
    })));
    const xMismatch = evaluatePreviewApproval(input(validPreview({
      goal: { x: target.x + 0.0002, y: target.y, yaw: target.yaw },
    })));
    const yMismatch = evaluatePreviewApproval(input(validPreview({
      goal: { x: target.x, y: target.y + 0.0002, yaw: target.yaw },
    })));
    expect(yawMismatch.rejectReason).toBe("TARGET_YAW_MISMATCH");
    expect(xMismatch.rejectReason).toBe("TARGET_POSITION_MISMATCH");
    expect(yMismatch.rejectReason).toBe("TARGET_POSITION_MISMATCH");
  });

  it("keeps request, map identity, active-map source, and freshness gates fail-closed", () => {
    const preview = validPreview();
    expect(evaluatePreviewApproval(input(preview, { requestId: "other" })).rejectReason).toBe("REQUEST_ID_MISMATCH");
    expect(evaluatePreviewApproval(input(validPreview({ status: "INVALID" }))).rejectReason).toBe("PREVIEW_NOT_VALID");
    expect(evaluatePreviewApproval(input(validPreview({ path: [] }))).rejectReason).toBe("PATH_EMPTY");
    expect(evaluatePreviewApproval(input(validPreview({ active_map_id: "SLAM-session-old" }))).rejectReason)
      .toBe("ACTIVE_MAP_ID_MISMATCH");
    expect(evaluatePreviewApproval(input(validPreview({ active_map_revision: "session-old" }))).rejectReason)
      .toBe("ACTIVE_MAP_REVISION_MISMATCH");
    expect(evaluatePreviewApproval(input(validPreview({ source_map_id: "CANONICAL" }))).rejectReason)
      .toBe("MAP_SOURCE_MISMATCH");
    expect(evaluatePreviewApproval(input(validPreview({ timestamp: "invalid" }))).rejectReason)
      .toBe("PREVIEW_TIMESTAMP_INVALID");
    expect(evaluatePreviewApproval(input(preview, { now: Date.parse(timestamp) + 120_001 })).rejectReason)
      .toBe("PREVIEW_TOO_OLD");
    expect(evaluatePreviewApproval(input(preview, { now: Date.parse(timestamp) - 5_001 })).rejectReason)
      .toBe("PREVIEW_FROM_FUTURE");
  });
});
