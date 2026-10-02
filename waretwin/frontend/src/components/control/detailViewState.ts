import type { RobotDetailState, RobotDetailViewStatus, RobotDetailLidar2D, RobotDetailLidar3D } from "../../schema/twin_state";

export function detailViewStatusPatch(current: RobotDetailState | undefined, status: RobotDetailViewStatus): Partial<RobotDetailState> {
  const wanted = current?.viewStatus;
  if (!wanted || status.robot_id !== wanted.robot_id || status.request_id !== wanted.request_id || status.applied_view !== wanted.requested_view) return {};
  if (wanted.bridge_epoch === status.bridge_epoch && (status.view_epoch ?? -1) < (wanted.view_epoch ?? -1)) return {};
  const map = current?.slam2dMap;
  const hasMap = Boolean(map?.map_source === "SLAM_TOOLBOX" && map.frame_id === "map"
    && map.mapping_session_id && map.active_map_id === `SLAM-${map.mapping_session_id}`
    && current?.mappingSessionId === map.mapping_session_id);
  return { viewStatus: { ...status, state: status.applied_view === "GLOBAL"
    || (status.applied_view === "LIDAR_2D" && hasMap) ? "FRESH" : "APPLIED" } };
}

export function detailFramePatch(current: RobotDetailState | undefined, view: "LIDAR_2D" | "LIDAR_3D", frame: RobotDetailLidar2D | RobotDetailLidar3D): Partial<RobotDetailState> {
  if (current?.viewStatus && current.viewStatus.robot_id !== frame.robot_id) return {};
  if (view === "LIDAR_3D") {
    const cloud = frame as RobotDetailLidar3D;
    const pose = cloud.slam_pose;
    if (!cloud.accumulated || cloud.accumulation_mode !== "SLAM_VISUALIZATION_VOXEL_MAP"
        || cloud.frame_id !== "map" || cloud.points.length > 20_000
        || !pose?.valid || pose.pose_source !== "TF" || pose.map_source !== "SLAM_TOOLBOX"
        || pose.frame_id !== cloud.frame_id || !pose.mapping_session_id
        || pose.map_id !== `SLAM-${pose.mapping_session_id}`) return {};
  }
  const cached = view === "LIDAR_2D" ? current?.lidar2dSensorFrame : current?.slam3dAccumulatedCloud;
  if (cached?.bridge_epoch === frame.bridge_epoch && (frame.view_epoch ?? -1) < (cached?.view_epoch ?? -1)) return {};
  const wanted = current?.viewStatus;
  if (wanted && wanted.state !== "REQUESTED" && wanted.requested_view === view
      && (wanted.bridge_epoch !== frame.bridge_epoch || wanted.request_id !== frame.request_id
        || wanted.view_epoch !== frame.view_epoch)) return {};
  const fresh = wanted && wanted.state !== "REQUESTED" && wanted.requested_view === view
    && wanted.applied_view === view && wanted.request_id === frame.request_id
    && wanted.view_epoch === frame.view_epoch && wanted.bridge_epoch === frame.bridge_epoch;
  return { ...(view === "LIDAR_2D" ? { lidar2dSensorFrame: frame as RobotDetailLidar2D } : { slam3dAccumulatedCloud: frame as RobotDetailLidar3D }),
    ...(fresh ? { viewStatus: { ...wanted, state: "FRESH" as const } } : {}) };
}
