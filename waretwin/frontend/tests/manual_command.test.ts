import { describe, expect, it } from "vitest";
import { MANUAL_COMMAND_REFRESH_MS, nextManualCommand } from "../src/services/manualCommand";

describe("latched manual command semantics", () => {
  it("toggles a repeated direction off and switches directions without an intermediate stop", () => {
    const commands = ["FORWARD", "BACKWARD", "LEFT", "RIGHT", "ROTATE_LEFT", "ROTATE_RIGHT"] as const;
    for (const command of commands) {
      expect(nextManualCommand(null, command)).toBe(command);
      expect(nextManualCommand(command, command)).toBeNull();
    }
    expect(nextManualCommand("FORWARD", "LEFT")).toBe("LEFT");
  });

  it("refreshes inside the existing 400 ms backend command lease", () => {
    expect(MANUAL_COMMAND_REFRESH_MS).toBe(100);
    expect(MANUAL_COMMAND_REFRESH_MS).toBeLessThan(400);
  });
});
