import { describe, expect, it } from "vitest";
import { createWorldTransform, worldToScreen } from "../src/layout/coordinates";
import {
  centerMapViewportCamera,
  fitMapViewportCamera,
  fixedWorldTransform,
  occupancyMapWorldBounds,
  resolveMapViewport,
  zoomMapViewportCamera,
} from "../src/layout/mapViewport";
import { occupancyRasterKey } from "../src/components/control/occupancyRaster";
import type { RobotDetailMapSnapshot } from "../src/schema/twin_state";

const size = { width: 1000, height: 700 };
const fixedWorldPoint = { x: 5, y: 5 };

function frame(overrides: Partial<RobotDetailMapSnapshot> = {}): RobotDetailMapSnapshot {
  return {
    robot_id: "R01", frame_id: "map", map_source: "SLAM_TOOLBOX",
    mapping_session_id: "session-1", active_map_id: "SLAM-session-1",
    active_map_revision: "session-session-1", map_content_revision: "cells-a",
    width: 400, height: 500, resolution: 0.05, origin: { x: -5, y: -10, yaw: 0 },
    data: Array(400 * 500).fill(-1),
    ...overrides,
  };
}

function screenPoint(camera: NonNullable<ReturnType<typeof resolveMapViewport>>["camera"]) {
  return worldToScreen(fixedWorldPoint, fixedWorldTransform(size, camera));
}

describe("stable SLAM map viewport", () => {
  const frameA = frame();
  const frameB = frame({
    active_map_revision: "session-session-1-b", map_content_revision: "cells-b",
    width: 600, height: 700, origin: { x: -8, y: -15, yaw: 0 },
    data: Array(600 * 700).fill(0),
  });
  const frameC = frame({
    active_map_revision: "session-session-1-c", map_content_revision: "cells-c",
    width: 800, height: 900, origin: { x: -12, y: -20, yaw: 0 },
    data: Array(800 * 900).fill(100),
  });

  it("keeps a fixed world point stationary as one SLAM session grows through frames A, B, and C", () => {
    const stateA = resolveMapViewport(null, frameA, size)!;
    const pixelA = screenPoint(stateA.camera);
    const stateB = resolveMapViewport(stateA, frameB, size)!;
    const pixelB = screenPoint(stateB.camera);
    const stateC = resolveMapViewport(stateB, frameC, size)!;
    const pixelC = screenPoint(stateC.camera);
    const drift = Math.max(
      Math.hypot(pixelB.x - pixelA.x, pixelB.y - pixelA.y),
      Math.hypot(pixelC.x - pixelA.x, pixelC.y - pixelA.y),
    );

    expect(drift).toBeLessThanOrEqual(1);
    expect(stateB.camera).toEqual(stateA.camera);
    expect(stateC.camera).toEqual(stateA.camera);
    expect(frameB.data).not.toEqual(frameA.data);
    expect(frameC.data).not.toEqual(frameB.data);
    expect(occupancyRasterKey(frameA)).not.toBe(occupancyRasterKey(frameB));
    expect(occupancyRasterKey(frameB)).not.toBe(occupancyRasterKey(frameC));
    expect(frameA.map_content_revision).not.toBe(frameC.map_content_revision);

    const autoFitPixelA = worldToScreen(fixedWorldPoint,
      createWorldTransform(size, occupancyMapWorldBounds(frameA)));
    const autoFitPixelC = worldToScreen(fixedWorldPoint,
      createWorldTransform(size, occupancyMapWorldBounds(frameC)));
    expect(Math.hypot(autoFitPixelC.x - autoFitPixelA.x, autoFitPixelC.y - autoFitPixelA.y))
      .toBeGreaterThan(2);
  });

  it("changes camera only for explicit FIT, zoom, and CENTER ROBOT actions", () => {
    const fittedA = resolveMapViewport(null, frameA, size)!;
    const zoomed = { ...fittedA, camera: zoomMapViewportCamera(fittedA.camera, 1.25) };
    const afterGrowth = resolveMapViewport(zoomed, frameC, size)!;
    expect(afterGrowth.camera.scalePxPerMeter).toBe(zoomed.camera.scalePxPerMeter);

    const centered = { ...afterGrowth, camera: centerMapViewportCamera(afterGrowth.camera, { x: 2, y: 3 }) };
    const afterMoreGrowth = resolveMapViewport(centered, frameB, size)!;
    expect(afterMoreGrowth.camera.centerX).toBe(2);
    expect(afterMoreGrowth.camera.centerY).toBe(3);
    expect(afterMoreGrowth.camera.scalePxPerMeter).toBe(centered.camera.scalePxPerMeter);

    const fittedC = fitMapViewportCamera(size, occupancyMapWorldBounds(frameC));
    expect(fittedC.centerX).not.toBe(afterMoreGrowth.camera.centerX);
    expect(fittedC.scalePxPerMeter).not.toBe(afterMoreGrowth.camera.scalePxPerMeter);
  });

  it("preserves world center and scale across a viewport resize, and refits only for a new session", () => {
    const state = resolveMapViewport(null, frameA, size)!;
    const resized = resolveMapViewport(state, frameB, { width: 1280, height: 720 })!;
    expect(resized.camera).toEqual(state.camera);

    const newSession = frame({ mapping_session_id: "session-2", active_map_id: "SLAM-session-2" });
    const reset = resolveMapViewport(state, newSession, size)!;
    expect(reset.sessionKey).not.toBe(state.sessionKey);
    expect(reset.camera).toEqual(fitMapViewportCamera(size, occupancyMapWorldBounds(newSession)));
  });
});
