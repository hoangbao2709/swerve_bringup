import { useStore } from "../state/store";
import type { RobotSystemDiagnostics } from "../schema/twin_state";
import {
  SHOWCASE_LOCAL_MAP,
  SHOWCASE_LIDAR_3D,
  SHOWCASE_MAP_SNAPSHOT,
  SHOWCASE_ROBOT,
  STATIC_CAPABILITIES,
  STATIC_DIAGNOSTICS,
  STATIC_LIDAR_DIAGNOSTICS,
  STATIC_SCAN,
} from "./fixtures";

const showcaseDiagnostics: RobotSystemDiagnostics = {
  ...STATIC_DIAGNOSTICS,
  metrics: { sample: "SHOWCASE" },
  errors: [
    { severity: "WARNING", code: "ROBOT_LINK_DISCONNECTED", message: "Robot link is disconnected; hardware commands are unavailable.", timestamp: "SAMPLE" },
    { severity: "INFO", code: "MAP_LOCAL_COPY", message: "North Hall is a browser sample map; no robot map is active.", timestamp: "SAMPLE" },
  ],
};

/** Seeds sample presentation state without claiming a robot or runtime connection. */
export function initializeShowcaseRuntime(): void {
  useStore.setState((state) => ({
    twin: {
      ...state.twin,
      robots: { [SHOWCASE_ROBOT.id]: SHOWCASE_ROBOT },
      kpi: {
        ...state.twin.kpi,
        fleet: { total: 1, active: 0, charging: 0, idle: 0, warning: 0, error: 0, offline: 1 },
      },
    },
    localRobotId: SHOWCASE_ROBOT.id,
    localIdentityError: null,
    localIdentityVerified: false,
    selectedRobot: null,
    robotDetail: {},
    runtimeState: "IDLE",
    source: "offline",
    rosDiagnostics: showcaseDiagnostics,
    rosConnected: false,
    bridgeState: "DISCONNECTED",
    websocketState: "DISCONNECTED",
    connectedRobotIds: [],
    robotCapabilities: {
      [SHOWCASE_ROBOT.id]: {
        ...STATIC_CAPABILITIES,
        goal_blocker_code: "ROBOT_DISCONNECTED",
        goal_blocker_reason: "Robot connection is unavailable in this standalone view.",
      },
    },
    lastTelemetryAt: null,
    mapSync: { ...state.mapSync, status: "UNAVAILABLE", error: null, tfStatus: false },
    localization: {
      state: "UNKNOWN",
      lastTagId: null,
      expectedTagId: null,
      tagVisible: false,
      lastTagSeenAt: null,
    },
  }));
  useStore.getState().setRobotDetail(SHOWCASE_ROBOT.id, {
    scan: { ...STATIC_SCAN, topic: "LOCAL_SAMPLE" },
    lidar2dSensorFrame: null,
    slam3dAccumulatedCloud: SHOWCASE_LIDAR_3D,
    slam2dMap: null,
    runtimeMapSnapshot: SHOWCASE_MAP_SNAPSHOT,
    globalPath: null,
    localPath: null,
    goal: null,
    controller: null,
    diagnostics: showcaseDiagnostics,
    errors: [],
    navigationStatus: "UNAVAILABLE",
    remainingDistanceM: null,
    pathPreview: null,
    mappingState: "UNAVAILABLE",
    mappingSessionId: null,
    activeLocalMapId: SHOWCASE_LOCAL_MAP.id,
    activeLocalMapRevision: SHOWCASE_LOCAL_MAP.revision,
    localMapSyncStatus: "UNAVAILABLE",
    lidarStreamDiagnostics: { ...STATIC_LIDAR_DIAGNOSTICS, view: "LOCAL_SAMPLE" },
    appliedMode: "AUTONOMOUS",
  });
  useStore.getState().setMapSync({ status: "UNAVAILABLE", publishedRevision: null, publishedVersion: 0,
    rosRevision: null, gazeboRevision: null, nav2Revision: null, tagMapRevision: null, error: null,
    robots: { [SHOWCASE_ROBOT.id]: { rosRevision: null, gazeboRevision: null, nav2Revision: null,
      tagMapRevision: null, tfStatus: false, error: null, status: "UNAVAILABLE" } } });
}
