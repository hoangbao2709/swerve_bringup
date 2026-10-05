import type { WarehouseLayout } from "../../layout/types";
import { robotForWarehouse } from "../../layout/robotPoseFrame";
import type { RobotState, TwinState } from "../../schema/twin_state";

export type OverviewRobot = {
  id: string;
  state: RobotState;
  pose: RobotState | null;
};

/** Only backend-reported identities belong in Overview; canonical-map markers
 * additionally require fresh ROS TF pose data for the exact published map. */
export function overviewRobots(
  twin: TwinState | null,
  layout: WarehouseLayout | null,
  publishedRevision: number | null,
): OverviewRobot[] {
  if (!twin) return [];
  return Object.entries(twin.robots)
    .filter(([, robot]) => typeof robot.last_telemetry_at === "string" && robot.last_telemetry_at.length > 0)
    .sort(([left], [right]) => left.localeCompare(right))
    .map(([id, state]) => {
      const measured = layout && publishedRevision !== null
        ? robotForWarehouse(state, "map", publishedRevision)
        : undefined;
      const pose = measured?.canonical_pose?.pose_source === "TF" ? measured : undefined;
      return { id, state, pose: pose ?? null };
    });
}

export function runtimeRobotOnline(
  id: string,
  websocketState: string,
  rosConnected: boolean,
  connectedRobotIds: string[],
): boolean {
  return websocketState === "CONNECTED" && rosConnected && connectedRobotIds.includes(id);
}
