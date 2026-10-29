import { describe, expect, it } from "vitest";
import { displayFixtureMapName, displayRuntimeRows, displayRuntimeValue } from "../src/mock/staticPresentation";

describe("STATIC_UI presentation adapter", () => {
  it("replaces fixture runtime status at the display boundary", () => {
    expect(displayRuntimeValue(true, "OFFLINE")).toBe("—");
    expect(displayRuntimeRows(true, [["ROS", "OFFLINE"], ["E-STOP", "UNKNOWN"], ["NAV2", "UNAVAILABLE"]]))
      .toEqual([["ROS", "—"], ["E-STOP", "—"], ["NAV2", "—"]]);
    expect(displayFixtureMapName(true, "SAMPLE MAP · NOT SAVED")).toBe("MAP");
  });

  it("preserves backend-connected runtime values and fixture-independent map names", () => {
    expect(displayRuntimeValue(false, "OFFLINE")).toBe("OFFLINE");
    expect(displayRuntimeValue(false, "UNKNOWN")).toBe("UNKNOWN");
    expect(displayRuntimeRows(false, [["ROS", "OFFLINE"], ["E-STOP", "UNKNOWN"]]))
      .toEqual([["ROS", "OFFLINE"], ["E-STOP", "UNKNOWN"]]);
    expect(displayFixtureMapName(false, "SAMPLE MAP · NOT SAVED")).toBe("SAMPLE MAP · NOT SAVED");
  });
});
