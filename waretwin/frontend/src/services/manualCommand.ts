import type { ManualAction } from "./ws";

export type ActiveManualCommand = Exclude<ManualAction, "STOP">;

/** A repeated command click toggles to STOP; a different command switches directly. */
export function nextManualCommand(
  activeManualCommand: ActiveManualCommand | null,
  requestedCommand: ActiveManualCommand,
): ActiveManualCommand | null {
  return activeManualCommand === requestedCommand ? null : requestedCommand;
}

/** Keep the existing 400 ms backend lease fresh with a 100 ms Web refresh. */
export const MANUAL_COMMAND_REFRESH_MS = 100;
