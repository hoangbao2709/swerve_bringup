import type { FramePose, RobotState } from "../schema/twin_state";

export type MapPoseIdentity = {
  frame_id?: string | null;
  active_map_id?: string | null;
  active_map_revision?: string | number | null;
  map_source?: string | null;
  mapping_session_id?: string | null;
};

export type RobotPoseIdentity = {
  pose_frame_id?: string | null;
  pose_map_id?: string | null;
  pose_map_revision?: string | number | null;
  pose_map_source?: string | null;
  pose_source?: string | null;
  pose_mapping_session_id?: string | null;
};

function sameRevision(a: string | number | null | undefined, b: string | number | null | undefined): boolean {
  return a !== null && a !== undefined && b !== null && b !== undefined && String(a) === String(b);
}

/** True only when a TF pose and the displayed map share a known map identity. */
export function robotPoseMatchesMap(robot: RobotPoseIdentity, map: MapPoseIdentity): boolean {
  if (!["TF", "GAZEBO_MODEL_STATES"].includes(robot.pose_source ?? "") || !robot.pose_frame_id || robot.pose_frame_id !== map.frame_id) return false;
  if (!robot.pose_map_id || robot.pose_map_id !== map.active_map_id) return false;

  if (map.active_map_id === "CANONICAL") {
    return robot.pose_map_source === "CANONICAL"
      && sameRevision(robot.pose_map_revision, map.active_map_revision);
  }

  if (map.map_source === "SLAM_TOOLBOX") {
    return robot.pose_source === "TF" && robot.pose_map_source === "SLAM_TOOLBOX"
      && Boolean(map.mapping_session_id)
      && robot.pose_mapping_session_id === map.mapping_session_id
      && map.active_map_id === `SLAM-${map.mapping_session_id}`;
  }

  return Boolean(map.active_map_id)
    && Boolean(map.map_source)
    && robot.pose_map_source === map.map_source
    && sameRevision(robot.pose_map_revision, map.active_map_revision);
}

/** One adapter for every renderer: select a pose in the displayed map, then
 * convert ROS planar x/y to the existing Three.js x/z convention. */
export function robotForDisplayedMap(robot: RobotState, map: MapPoseIdentity): RobotState | undefined {
  const pose: FramePose | null | undefined = map.active_map_id === "CANONICAL" ? robot.canonical_pose
    : map.map_source === "SLAM_TOOLBOX" ? robot.slam_pose : undefined;
  if (map.active_map_id !== "CANONICAL" && map.map_source !== "SLAM_TOOLBOX") {
    return robotPoseMatchesMap(robot, map) ? robot : undefined;
  }
  if (!pose?.valid || robot.status === "OFFLINE" || ![pose.x, pose.y, pose.yaw].every(Number.isFinite)) return undefined;
  const age = Date.now() - Date.parse(pose.timestamp);
  if (!Number.isFinite(age) || age < -1000 || age > 3000) return undefined;
  if (pose.pose_source === "GAZEBO_MODEL_STATES" && (pose.source_frame_id !== "world"
      || pose.transform_source !== "VALIDATED_CANONICAL_WORLD_BUNDLE")) return undefined;
  const projected = { ...robot, position: [pose.x, robot.position[1], pose.y] as RobotState["position"],
    heading: pose.yaw, pose_frame_id: pose.frame_id, pose_map_id: pose.map_id,
    pose_map_revision: pose.map_revision, pose_map_source: pose.map_source,
    pose_source: pose.pose_source, pose_mapping_session_id: pose.mapping_session_id };
  return robotPoseMatchesMap(projected, map) ? projected : undefined;
}

export function robotForWarehouse(robot: RobotState, runtimeMode: string, frame: string, revision: string | number): RobotState | undefined {
  if (runtimeMode === "LOCAL_SIM") return robot;
  return robotForDisplayedMap(robot, { frame_id: frame, active_map_id: "CANONICAL",
    active_map_revision: revision, map_source: "CANONICAL" });
}
