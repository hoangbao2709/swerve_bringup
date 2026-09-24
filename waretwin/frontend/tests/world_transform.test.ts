import { describe, expect, it } from "vitest";
import { createWorldTransform, screenToWorld, worldToScreen } from "../src/layout/coordinates";

describe("canonical world/screen transform", () => {
  it("round-trips negative world metres with a nonzero origin", () => {
    const transform = createWorldTransform(
      { width: 1280, height: 720 },
      { minX: -4, maxX: 16, minY: -7, maxY: 9 },
      1.75,
    );
    const world = { x: 2.35, y: -1.8 };
    const screen = worldToScreen(world, transform);
    const result = screenToWorld(screen, transform);
    expect(result.x).toBeCloseTo(world.x, 10);
    expect(result.y).toBeCloseTo(world.y, 10);
  });

  it("maps positive ROS Y upward on screen", () => {
    const transform = createWorldTransform(
      { width: 600, height: 400 },
      { minX: -2, maxX: 8, minY: 5, maxY: 15 },
    );
    expect(worldToScreen({ x: 0, y: 12 }, transform).y)
      .toBeLessThan(worldToScreen({ x: 0, y: 8 }, transform).y);
  });
});
