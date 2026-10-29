import { describe, expect, it } from "vitest";
import type { RobotState } from "../src/schema/twin_state";
import { controlModeBlockReason, isRobotStopped } from "../src/services/controlModeSafety";

function robot(overrides: Partial<RobotState> = {}): RobotState {
  return { navigation_state: "IDLE", vx: 0, vy: 0, wz: 0, ...overrides } as RobotState;
}

describe("robot control-mode safety gate", () => {
  it("accepts a verified, stopped robot with fresh E-STOP-clear telemetry", () => {
    const current = robot();
    expect(isRobotStopped(current)).toBe(true);
    expect(controlModeBlockReason({ online: true, robot: current, estopClear: true, pending: false })).toBeNull();
  });

  it("blocks mode changes while any measured velocity is above the stop threshold", () => {
    const current = robot({ vx: 0.051 });
    expect(isRobotStopped(current)).toBe(false);
    expect(controlModeBlockReason({ online: true, robot: current, estopClear: true, pending: false }))
      .toContain("zero velocity");
  });

  it("blocks nonterminal navigation, E-STOP, pending, busy, and unverified states", () => {
    const base = { robot: robot(), estopClear: true, pending: false };
    expect(controlModeBlockReason({ ...base, online: false })).toContain("verified online");
    expect(controlModeBlockReason({ ...base, online: true, robot: undefined })).toContain("not available");
    expect(controlModeBlockReason({ ...base, online: true, estopClear: false })).toContain("E-STOP");
    expect(controlModeBlockReason({ ...base, online: true, pending: true })).toContain("transition");
    expect(controlModeBlockReason({ ...base, online: true, busy: true })).toContain("action to finish");
    expect(controlModeBlockReason({ ...base, online: true, robot: robot({ navigation_state: "NAVIGATING" }) }))
      .toContain("active navigation goal");
  });
});
