import { describe, expect, it } from "vitest";
import { displayedFramePose, robotPoseMatchesMap, robotForWarehouse, robotForDisplayedMap, stabilizeDisplayedFramePose,
  ROBOT_POSE_HOLD_TTL_MS, type MapPoseIdentity, type RobotPoseIdentity, type RetainedFramePose } from "../src/layout/robotPoseFrame";
import type { RobotState, FramePose } from "../src/schema/twin_state";

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
  const canonical: FramePose = { x: 15, y: 5.5, yaw: 0.7, frame_id: "map", map_id: "CANONICAL",
    map_revision: "21", map_source: "CANONICAL", pose_source: "GAZEBO_MODEL_STATES", valid: true,
    source_frame_id: "world", transform_source: "VALIDATED_CANONICAL_WORLD_BUNDLE", timestamp: new Date().toISOString() };
  const robot = { id: "R01", status: "ACTIVE", position: [0, 0, 0], heading: 0,
    canonical_pose: canonical, slam_pose: { ...canonical, x: 0, y: 0, yaw: 0,
      map_id: "SLAM-session-1", map_source: "SLAM_TOOLBOX", pose_source: "TF", mapping_session_id: "session-1" } } as RobotState;
  it("selects the same canonical pose for warehouse renderers while mapping uses SLAM", () => {
    expect(robotForWarehouse(robot, "GAZEBO_ROS", "map", 21)?.position).toEqual([15, 0, 5.5]);
    expect(robotForWarehouse(robot, "GAZEBO_ROS", "map", 21)?.heading).toBe(0.7);
    expect(robotForDisplayedMap(robot, slamMap)?.position).toEqual([0, 0, 0]);
  });
  it("hides an unavailable, stale, wrong-revision or unvalidated canonical pose", () => {
    for (const pose of [null, { ...canonical, map_revision: "20" },
      { ...canonical, timestamp: "2000-01-01T00:00:00Z" }, { ...canonical, transform_source: "ASSUMED_OFFSET" }]) {
      expect(robotForWarehouse({ ...robot, canonical_pose: pose }, "GAZEBO_ROS", "map", 21)).toBeUndefined();
    }
  });
  it("does not reinterpret a SLAM pose as canonical when canonical telemetry is missing", () => {
    expect(robotForWarehouse({ ...robot, canonical_pose: null, ...slamPose }, "GAZEBO_ROS", "map", 21)).toBeUndefined();
  });
  it("does not allow LOCAL_SIM to bypass real map-pose identity", () => {
    expect(robotForWarehouse({ ...robot, canonical_pose: null }, "LOCAL_SIM", "map", 21)).toBeUndefined();
  });
  it("rejects offline, nonfinite and wrong-session named poses", () => {
    expect(robotForWarehouse({ ...robot, status: "OFFLINE" }, "GAZEBO_ROS", "map", 21)).toBeUndefined();
    expect(robotForWarehouse({ ...robot, canonical_pose: { ...canonical, x: NaN } }, "GAZEBO_ROS", "map", 21)).toBeUndefined();
    expect(robotForDisplayedMap({ ...robot, slam_pose: { ...robot.slam_pose!, mapping_session_id: "old" } }, slamMap)).toBeUndefined();
    expect(robotForDisplayedMap({ ...robot, slam_pose: { ...robot.slam_pose!, frame_id: "world" } }, slamMap)).toBeUndefined();
  });
  it("accepts the live TF pose for the same SLAM session as /map", () => {
    expect(robotPoseMatchesMap(slamPose, slamMap)).toBe(true);
  });

  it("uses the active map pose only for the matching saved local map identity", () => {
    const localMap: MapPoseIdentity = { frame_id: "map", active_map_id: "saved-R01-1",
      active_map_revision: "artifact-1", map_source: "LOCAL_MAP" };
    const localPose: FramePose = { x: 2, y: 3, yaw: 0.4, frame_id: "map", map_id: "saved-R01-1",
      map_revision: "artifact-1", map_source: "LOCAL_MAP", pose_source: "TF", valid: true,
      timestamp: new Date().toISOString() };
    const localRobot = { ...robot, active_map_pose: localPose,
      canonical_pose: { ...canonical, x: 100, y: 100 }, slam_pose: { ...slamPose, x: 200, y: 200 } };
    expect(displayedFramePose(localRobot, localMap)).toBe(localPose);
    expect(displayedFramePose({ ...localRobot, active_map_pose: { ...localPose, map_id: "saved-other" } }, localMap)).toBeUndefined();
    expect(displayedFramePose({ ...localRobot, active_map_pose: { ...localPose, map_revision: "old" } }, localMap)).toBeUndefined();
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

  it("retains a compatible robot marker through transient pose loss and expires it after the TTL", () => {
    const start = Date.now();
    const liveRobot = { ...robot, canonical_pose: { ...canonical, timestamp: new Date(start).toISOString() } };
    const accepted = stabilizeDisplayedFramePose(liveRobot, {
      frame_id: "map", active_map_id: "CANONICAL", active_map_revision: "21", map_source: "CANONICAL",
    }, null, start);
    const missing = stabilizeDisplayedFramePose(undefined, {
      frame_id: "map", active_map_id: "CANONICAL", active_map_revision: "21", map_source: "CANONICAL",
    }, accepted.retained, start + 800);
    expect(missing.pose).toBe(accepted.pose);
    const expired = stabilizeDisplayedFramePose(undefined, {
      frame_id: "map", active_map_id: "CANONICAL", active_map_revision: "21", map_source: "CANONICAL",
    }, accepted.retained, start + ROBOT_POSE_HOLD_TTL_MS + 1);
    expect(expired.pose).toBeUndefined();
  });

  it("keeps marker visibility continuous across a 30-second motion stream with transient packets missing", () => {
    const start = Date.now();
    const map: MapPoseIdentity = { frame_id: "map", active_map_id: "CANONICAL",
      active_map_revision: "21", map_source: "CANONICAL" };
    let retained: RetainedFramePose | null = null;
    let visibleSamples = 0;
    const totalSamples = 301;
    for (let sample = 0; sample < totalSamples; sample++) {
      const elapsed = sample * 100;
      const pose = { ...canonical, x: 15 + elapsed / 1000, timestamp: new Date(start).toISOString() };
      const packet = sample % 20 === 10 ? undefined : { ...robot, canonical_pose: pose };
      const result = stabilizeDisplayedFramePose(packet, map, retained, start + elapsed);
      retained = result.retained;
      if (result.pose) visibleSamples++;
    }
    expect(totalSamples).toBe(301);
    expect(visibleSamples).toBe(totalSamples);
  });
});
