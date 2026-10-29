import { describe, expect, it } from "vitest";
import { MANUAL_COMMAND_REFRESH_MS } from "../src/services/manualCommand";

describe("dead-man manual command lease", () => {
  it("refreshes inside the existing 400 ms backend command lease", () => {
    expect(MANUAL_COMMAND_REFRESH_MS).toBe(100);
    expect(MANUAL_COMMAND_REFRESH_MS).toBeLessThan(400);
  });
});
