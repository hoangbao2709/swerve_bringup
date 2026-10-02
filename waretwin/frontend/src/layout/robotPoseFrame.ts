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
  if (robot.pose_source !== "TF" || !robot.pose_frame_id || robot.pose_frame_id !== map.frame_id) return false;
  if (!robot.pose_map_id || robot.pose_map_id !== map.active_map_id) return false;

  if (map.active_map_id === "CANONICAL") {
    return robot.pose_map_source === "CANONICAL"
      && sameRevision(robot.pose_map_revision, map.active_map_revision);
  }

  if (map.map_source === "SLAM_TOOLBOX") {
    return robot.pose_map_source === "SLAM_TOOLBOX"
      && Boolean(map.mapping_session_id)
      && robot.pose_mapping_session_id === map.mapping_session_id
      && map.active_map_id === `SLAM-${map.mapping_session_id}`;
  }

  return Boolean(map.active_map_id)
    && Boolean(map.map_source)
    && robot.pose_map_source === map.map_source
    && sameRevision(robot.pose_map_revision, map.active_map_revision);
}
