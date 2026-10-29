import type { RobotState } from "../schema/twin_state";

const SAFE_TERMINAL_NAVIGATION_STATES = new Set([
  "IDLE", "MANUAL", "SUCCEEDED", "COMPLETED", "FAILED", "ERROR", "CANCELED", "CANCELLED", "REJECTED",
]);

export function isRobotStopped(robot: RobotState | undefined): boolean {
  if (!robot) return false;
  const navigationState = String(robot.navigation_state ?? "").toUpperCase();
  const velocities = [robot.vx, robot.vy, robot.wz].map(Number);
  return SAFE_TERMINAL_NAVIGATION_STATES.has(navigationState)
    && velocities.every((velocity) => Number.isFinite(velocity) && Math.abs(velocity) <= 0.05);
}

export function controlModeBlockReason({
  online,
  robot,
  estopClear,
  pending,
  busy = false,
}: {
  online: boolean;
  robot: RobotState | undefined;
  estopClear: boolean;
  pending: boolean;
  busy?: boolean;
}): string | null {
  if (!online) return "Robot control requires a verified online robot bridge.";
  if (busy) return "Wait for the current robot action to finish.";
  if (pending) return "Wait for the current control-mode transition to finish.";
  if (!robot) return "Robot status is not available.";
  if (!estopClear) return "Wait for fresh telemetry confirming E-STOP is clear.";
  const navigationState = String(robot.navigation_state ?? "").toUpperCase();
  if (!SAFE_TERMINAL_NAVIGATION_STATES.has(navigationState)) {
    return "Cancel the active navigation goal and wait for a stopped state before changing control mode.";
  }
  if (!isRobotStopped(robot)) {
    return "Stop the robot and confirm zero velocity before changing control mode.";
  }
  return null;
}
