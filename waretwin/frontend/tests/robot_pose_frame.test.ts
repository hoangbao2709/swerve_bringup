import { describe, expect, it } from "vitest";
import { robotPoseMatchesMap, type MapPoseIdentity, type RobotPoseIdentity } from "../src/layout/robotPoseFrame";

const slamMap: MapPoseIdentity = {
  frame_id: "map",
  active_map_id: "SLAM-session-1",
  active_map_revision: "grid-new",
  map_source: "SLAM_TOOLBOX",
  mapping_session_id: "session-1",
};

const slamPose: RobotPoseIdentity = {
  pose_frame_id: "map",
  pose_map_id: "SLAM-session-1",
  pose_map_revision: "grid-old",
  pose_map_source: "SLAM_TOOLBOX",
  pose_source: "TF",
  pose_mapping_session_id: "session-1",
};

describe("robot pose and displayed map identity", () => {
  it("accepts the live TF pose for the same SLAM session as /map", () => {
    expect(robotPoseMatchesMap(slamPose, slamMap)).toBe(true);
  });

  it("rejects a different SLAM session even when both frames are named map", () => {
    expect(robotPoseMatchesMap({ ...slamPose, pose_map_id: "SLAM-session-2", pose_mapping_session_id: "session-2" }, slamMap)).toBe(false);
  });

  it("requires the canonical map revision before drawing on warehouse geometry", () => {
    const canonicalMap: MapPoseIdentity = {
      frame_id: "map", active_map_id: "CANONICAL", active_map_revision: 21, map_source: "CANONICAL",
    };
    const canonicalPose: RobotPoseIdentity = {
      pose_frame_id: "map", pose_map_id: "CANONICAL", pose_map_revision: "21",
      pose_map_source: "CANONICAL", pose_source: "TF",
    };
    expect(robotPoseMatchesMap(canonicalPose, canonicalMap)).toBe(true);
    expect(robotPoseMatchesMap({ ...canonicalPose, pose_map_revision: "20" }, canonicalMap)).toBe(false);
  });

  it("rejects missing pose provenance and frame mismatches", () => {
    expect(robotPoseMatchesMap({ ...slamPose, pose_source: undefined }, slamMap)).toBe(false);
    expect(robotPoseMatchesMap({ ...slamPose, pose_frame_id: "odom" }, slamMap)).toBe(false);
  });
});
