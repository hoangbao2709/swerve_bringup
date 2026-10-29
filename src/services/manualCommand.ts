import type { ManualAction } from "./ws";

export type ActiveManualCommand = Exclude<ManualAction, "STOP">;

/** Keep the existing 400 ms backend lease fresh with a 100 ms Web refresh. */
export const MANUAL_COMMAND_REFRESH_MS = 100;
