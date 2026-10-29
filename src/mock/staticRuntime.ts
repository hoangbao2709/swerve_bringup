import { useStore } from "../state/store";
import {
  createStaticTwin,
  STATIC_CAPABILITIES,
  STATIC_DIAGNOSTICS,
  STATIC_LIDAR_DIAGNOSTICS,
  STATIC_MAP_ID,
  STATIC_MAP_REVISION,
  STATIC_MAP_SNAPSHOT,
  STATIC_ROBOT_ID,
  STATIC_SCAN,
} from "./fixtures";

/** Seeds UI-only fixtures while keeping deployment identity and online state unverified. */
export function initializeStaticRuntime(): void {
  useStore.setState((state) => ({
    twin: createStaticTwin(state.twin),
    localRobotId: null,
    localIdentityError: null,
    localIdentityVerified: false,
    selectedRobot: null,
    robotDetail: {},
    runtimeState: "IDLE",
    source: "offline",
    rosDiagnostics: STATIC_DIAGNOSTICS,
    rosConnected: false,
    bridgeState: "DISCONNECTED",
    websocketState: "DISCONNECTED",
    connectedRobotIds: [],
    robotCapabilities: { [STATIC_ROBOT_ID]: STATIC_CAPABILITIES },
    lastTelemetryAt: null,
    mapSync: { ...state.mapSync, status: "UNAVAILABLE", error: null },
    localization: {
      state: "UNKNOWN",
      lastTagId: null,
      expectedTagId: null,
      tagVisible: false,
      lastTagSeenAt: null,
    },
  }));
  useStore.getState().setRobotDetail(STATIC_ROBOT_ID, {
    scan: STATIC_SCAN,
    lidar2dSensorFrame: null,
    slam3dAccumulatedCloud: null,
    slam2dMap: null,
    runtimeMapSnapshot: STATIC_MAP_SNAPSHOT,
    globalPath: null,
    localPath: null,
    goal: null,
    controller: null,
    diagnostics: STATIC_DIAGNOSTICS,
    errors: [],
    navigationStatus: "UNKNOWN",
    remainingDistanceM: null,
    pathPreview: null,
    mappingState: "UNAVAILABLE",
    mappingSessionId: null,
    activeLocalMapId: STATIC_MAP_ID,
    activeLocalMapRevision: STATIC_MAP_REVISION,
    localMapSyncStatus: "UNAVAILABLE",
    lidarStreamDiagnostics: STATIC_LIDAR_DIAGNOSTICS,
    appliedMode: "AUTONOMOUS",
  });
}
