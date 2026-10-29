import { describe, expect, it } from "vitest";
import { MAX_ESTOP_TELEMETRY_AGE_MS, resolveEStopState } from "../src/services/estopState";

describe("E-STOP telemetry status", () => {
  const now = Date.parse("2026-10-27T12:00:00.000Z");
  const fresh = new Date(now - 1_000).toISOString();

  it("reports ACTIVE or CLEAR only from explicit, fresh telemetry while connected", () => {
    expect(resolveEStopState(true, true, fresh, now)).toBe("ACTIVE");
    expect(resolveEStopState(false, true, fresh, now)).toBe("CLEAR");
  });

  it("reports UNKNOWN when connectivity, telemetry, or the E-STOP sample is missing", () => {
    expect(resolveEStopState(false, false, fresh, now)).toBe("UNKNOWN");
    expect(resolveEStopState(false, true, null, now)).toBe("UNKNOWN");
    expect(resolveEStopState(undefined, true, fresh, now)).toBe("UNKNOWN");
  });

  it("reports UNKNOWN for stale, invalid, and implausibly future timestamps", () => {
    const stale = new Date(now - MAX_ESTOP_TELEMETRY_AGE_MS - 1).toISOString();
    const future = new Date(now + MAX_ESTOP_TELEMETRY_AGE_MS + 1).toISOString();
    expect(resolveEStopState(false, true, stale, now)).toBe("UNKNOWN");
    expect(resolveEStopState(false, true, future, now)).toBe("UNKNOWN");
    expect(resolveEStopState(false, true, "not-a-timestamp", now)).toBe("UNKNOWN");
  });
});
