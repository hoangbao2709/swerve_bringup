export type EStopState = "ACTIVE" | "CLEAR" | "UNKNOWN";

export const MAX_ESTOP_TELEMETRY_AGE_MS = 5_000;

/** E-STOP is safe to report clear only from fresh, explicit boolean telemetry. */
export function resolveEStopState(
  estopActive: boolean | null | undefined,
  online: boolean,
  lastTelemetryAt: string | null | undefined,
  now = Date.now(),
): EStopState {
  if (!online || typeof estopActive !== "boolean" || !lastTelemetryAt) return "UNKNOWN";
  const timestamp = Date.parse(lastTelemetryAt);
  if (!Number.isFinite(timestamp)) return "UNKNOWN";
  const age = now - timestamp;
  if (age > MAX_ESTOP_TELEMETRY_AGE_MS || age < -MAX_ESTOP_TELEMETRY_AGE_MS) return "UNKNOWN";
  return estopActive ? "ACTIVE" : "CLEAR";
}
