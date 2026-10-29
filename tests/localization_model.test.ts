import { describe, expect, it } from "vitest";
import { displayYawToRadians, localizationMapIdentityKey, normalizeYawRadians, radiansToDisplayYaw,
  validateInitialPose } from "../src/pages/localization/localizationModel";
import type { RobotDetailMapSnapshot } from "../src/schema/twin_state";

const map: RobotDetailMapSnapshot = {
  robot_id: "R01", frame_id: "map", map_source: "LOCAL_MAP",
  active_map_id: "saved-map", active_map_revision: "rev-2", width: 2, height: 2,
  resolution: 1, origin: { x: 0, y: 0, yaw: 0 }, data: [0, 100, -1, 0],
};
const cells = Int8Array.from(map.data!);

describe("localization pose model", () => {
  it("keeps yaw in radians and converts the editor display unit", () => {
    expect(displayYawToRadians(90, "DEG")).toBeCloseTo(Math.PI / 2);
    expect(radiansToDisplayYaw(Math.PI, "DEG")).toBeCloseTo(180);
    expect(displayYawToRadians(0.75, "RAD")).toBe(0.75);
    expect(normalizeYawRadians(3 * Math.PI / 2)).toBeCloseTo(-Math.PI / 2);
  });

  it.each([
    [{ x: 0.5, y: 0.5, yaw: 0 }, "FREE"],
    [{ x: 1.5, y: 0.5, yaw: 0 }, "OCCUPIED"],
    [{ x: 0.5, y: 1.5, yaw: 0 }, "UNKNOWN"],
    [{ x: -0.01, y: 0.5, yaw: 0 }, "OUTSIDE"],
  ] as const)("validates only a known free occupancy cell (%s)", (pose, result) => {
    const validated = validateInitialPose(pose, map, cells);
    expect(validated.valid).toBe(result === "FREE");
    if (result !== "FREE") expect(validated).toMatchObject({ valid: false, classification: result });
  });

  it("rejects non-finite values, out-of-range yaw, and missing occupancy/map identity", () => {
    expect(validateInitialPose({ x: Number.NaN, y: 0.5, yaw: 0 }, map, cells).valid).toBe(false);
    expect(validateInitialPose({ x: 0.5, y: Number.POSITIVE_INFINITY, yaw: 0 }, map, cells).valid).toBe(false);
    expect(validateInitialPose({ x: 0.5, y: 0.5, yaw: Math.PI + 0.01 }, map, cells).valid).toBe(false);
    expect(validateInitialPose({ x: 0.5, y: 0.5, yaw: 0 }, null, cells).valid).toBe(false);
    expect(validateInitialPose({ x: 0.5, y: 0.5, yaw: 0 }, map, null).valid).toBe(false);
  });

  it("binds the pose draft to map source, ID, frame, session and revision", () => {
    const original = localizationMapIdentityKey("R01", map);
    expect(localizationMapIdentityKey("R01", { ...map, active_map_revision: "rev-3" })).not.toBe(original);
    expect(localizationMapIdentityKey("R01", { ...map, active_map_id: "other-map" })).not.toBe(original);
    expect(localizationMapIdentityKey("R02", map)).not.toBe(original);
  });
});
